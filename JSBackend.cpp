//===-- JSBackend.cpp - Library for converting LLVM code to JavaScript --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This library converts LLVM code to JavaScript code, just a proof-of-concept
// toy program.
//
//===----------------------------------------------------------------------===//

#include "JSTargetMachine.h"
#include "JSTargetMachine.h"
#include "llvm/CallingConv.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/TypeSymbolTable.h"
#include "llvm/Intrinsics.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/InlineAsm.h"
#include "llvm/Analysis/ConstantsScanner.h"
#include "llvm/Analysis/FindUsedTypes.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/IntrinsicLowering.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Target/TargetMachineRegistry.h"
#include "llvm/Target/TargetAsmInfo.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/InstVisitor.h"
#include "llvm/Support/Mangler.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Config/config.h"
#include <algorithm>
#include <sstream>

#include <iostream> // for debug

using namespace llvm;

#define JSBE_SHOULD_NOT_BE_REACHED() assert(!"Should Not Be Reached!")
#define JSBE_NOT_IMPLEMENTED_YET() assert(!"Not Implemented Yet!")
#define JSBE_CURRENTLY_ASSUME(cond) assert((cond) || !"Not Implemented Yet!")

namespace jsbackend_impl { // anonymous namespace generates weird sybol, so we give a name...

  // copied from CBackend.cpp:
  // isDirectAlloca - Define fixed sized allocas in the entry block as direct
  // variables which are accessed with the & operator.  This causes GCC to
  // generate significantly better code than to emit alloca calls directly.
  //
#if 0
  static const AllocaInst *isDirectAlloca(const Value *V) {
    const AllocaInst *AI = dyn_cast<AllocaInst>(V);
    if (!AI) return 0;

    //std::cerr << isa<CompositeType>(AI->getType()->getTypeAtIndex(unsigned(0))) << std::endl;
    //std::cerr << AI->getType()->getTypeAtIndex(unsigned(0))->getDescription() << std::endl;
    if (isa<PointerType>(AI->getType()->getTypeAtIndex(unsigned(0)))) {
      return AI;
    }

    if (isa<CompositeType>(AI->getType()->getTypeAtIndex(unsigned(0)))) {
      return 0;
    }

    return AI;
  }
#else
  static const AllocaInst *isDirectAlloca(const Value *V) {
     return 0;
  }
#endif

  typedef std::map<const Value*, std::string> ValueNameMap;
  static  ValueNameMap s_namedValueMap;

  static std::string assignName(const Value* V) {
    ValueNameMap::iterator i = s_namedValueMap.find(V);
    if (i != s_namedValueMap.end()) {
      return i->second;
    } else {
      std::ostringstream ret;
      if (isa<Instruction>(V)) {
        ret << dyn_cast<Instruction>(V)->getOpcodeName();
      } else {
        ret << V->getType()->getDescription();
      }

      ret << s_namedValueMap.size();
      s_namedValueMap.insert(std::make_pair(V, ret.str()));
      return ret.str();
    }
  }

  static std::string encodeName(const Value* V) {
    std::string ret = V->getName();
    if (ret.empty()) { ret = assignName(V); }
    for (std::string::iterator i=ret.begin(); i!=ret.end(); ++i) {
      if (!isalnum(*i)) { *i = '_'; }
    }
    return ret;
  }

  static std::string makeConstant(const Constant* CPV, bool Static);

  static std::string makeValueRepr(const Value *V) {
    std::string ret;
    if (isDirectAlloca(V)) {
      ret += "new __JSBE.make_ptr(";
      ret += encodeName(V);
      ret += ")";
    } else {
      const Constant* CPV = dyn_cast<Constant>(V);
      if (CPV){
        if (!isa<GlobalValue>(CPV)) {
          ret += makeConstant(CPV, false);
        } else {
          JSBE_NOT_IMPLEMENTED_YET();
        }
      } else {
        ret += encodeName(V);
      }
    }
    return ret;
  }

  std::string makeGetElementPtrExp(const Value *Ptr,
                                   const User::const_op_iterator I,
                                   const User::const_op_iterator E) {
    std::string ret;
    ret += encodeName(Ptr);
    ret += ".element_at(";
    bool PrintedArg = false;
    for (User::const_op_iterator OI=I; OI != E; ++OI) {
      if (PrintedArg) { ret += ", "; }
      ret += makeValueRepr(*OI);
      PrintedArg = true;
    }
    ret += ")";
    return ret;
  }

  static std::string makeConstant(const Constant* CPV, bool Static) {
    //JSBE_CURRENTLY_ASSUME(dyn_cast<const ConstantInt>(CPV));
    std::ostringstream ret;
    if (const ConstantInt* CI = dyn_cast<const ConstantInt>(CPV)) {
      ret << CI->getSExtValue();
    } else if(const ConstantExpr *CE = dyn_cast<ConstantExpr>(CPV)) {
      switch (CE->getOpcode()) {
      case Instruction::GetElementPtr:
        ret << makeGetElementPtrExp(CE->getOperand(0), CE->op_begin()+1, CE->op_end());
        break;
      default:
        JSBE_NOT_IMPLEMENTED_YET();
        break;
      }
    } else {
      //std::cerr << typeid(CPV).name() << std::endl;
      if (1 < CPV->getNumOperands()) {
        ret << "new __JSBE.MemoryArray([";
        bool PrintedArg = false;
        for (unsigned CI=0; CI<CPV->getNumOperands(); ++CI) {
          if (PrintedArg) { ret << ", "; }
          ret << makeConstant(CPV->getOperand(CI), Static);
          PrintedArg = true;
        }
        ret << "])";
      } else {
        JSBE_NOT_IMPLEMENTED_YET();
      }
    }
    return ret.str();
  }

  static size_t indexOf(Function& F, BasicBlock& BB) {
    size_t Index = 0;
    for (Function::iterator IBB = F.begin(); IBB != F.end(); ++IBB) {
      if (&(*IBB) == &BB) {
        return Index;
      }

      Index++;
    }

    JSBE_SHOULD_NOT_BE_REACHED();
    return 0;
  }

  class JSInstWriter : public InstVisitor<JSInstWriter> {
  public:
    JSInstWriter(std::ostream& Out) : m_Out(Out) {}

    void PrintFunctionSignature(Function& F) {
      JSBE_CURRENTLY_ASSUME(!F.isDeclaration());
      JSBE_CURRENTLY_ASSUME(!F.hasStructRetAttr());

      m_Out << "function " << encodeName(&F) << "(";

      bool printFirst = false; // XXX: const_arg_iterator does not give enough impl to use our idiom...
      for (Function::const_arg_iterator I = F.arg_begin(); I != F.arg_end(); ++I) {
        JSBE_CURRENTLY_ASSUME(I->hasName());

        if (printFirst) {
          m_Out << ", ";
        }

        m_Out << encodeName(I);
        printFirst = true;
      }

      m_Out << ")\n";
    }

    void PrintFunctionPrologue(Function& F) {

      m_Out << "{\n";

      for (inst_iterator I = inst_begin(&F), E = inst_end(&F); I != E; ++I) {
        if (isa<PHINode>(*I)) {
          m_Out << "  var " << encodeName(&(*I)) << ";\n";
        }
      }

      if (1 < F.size()) {
        m_Out << "  var __bb = 0;\n";
        m_Out << "  while (-1 != __bb) {\n";
        m_Out << "    switch(__bb) {\n";
      }
    }

    void PrintFunctionEpilogue(Function& F) {
      if (1 < F.size()) {
        m_Out << "    default: break;\n";
        m_Out << "    }\n";
        m_Out << "  }\n";
      }
      m_Out << "}\n\n";
    }

    void printBasicBlockOrLoop(BasicBlock& BB, size_t Index, bool HasCase) {
      // FIXME: use LoopInfo to output loop structure
      printBasicBlock(BB, Index, HasCase);
    }

    void printLoop(Loop& L) {
      JSBE_NOT_IMPLEMENTED_YET();
    }

    void printBasicBlock(BasicBlock& BB, size_t Index, bool HasCase) {
      if (HasCase) { m_Out << "    case " << Index << ":\n"; }
      for (BasicBlock::iterator II = BB.begin(), E = BB.end(); II != E; ++II) {
        visit(*II);
      }
      if (HasCase) { m_Out << "      break;\n"; }
    }

    void printGlobalVariable(GlobalVariable* GV) {
      m_Out << "var " << encodeName(GV) << " = "
            << makeValueRepr(GV->getInitializer())
            << ";\n";
    }

    void printPHICopiesForSuccessor (BasicBlock *CurBlock,
                                     BasicBlock *Successor,
                                     unsigned Indent) {
      for (BasicBlock::iterator I = Successor->begin(); isa<PHINode>(I); ++I) {
        PHINode *PN = cast<PHINode>(I);
        // Now we have to do the printing.
        Value *IV = PN->getIncomingValueForBlock(CurBlock);
        if (!isa<UndefValue>(IV)) {
          m_Out << "      "
                << encodeName(I) << " = " << makeValueRepr(IV)
                << ";\n";
        }
      }
    }

    //
    // visitations:
    //

    void visitAllocaInst(AllocaInst &I) {
      if (isDirectAlloca(&I)) {
        m_Out << "      var " << encodeName(&I) << ";\n";
      } else {
        m_Out << "      var " << encodeName(&I) << " = __JSBE.make_memory();\n";
      }
    }

    void visitReturnInst(ReturnInst &I) {
      JSBE_CURRENTLY_ASSUME(!I.getParent()->getParent()->hasStructRetAttr());
      Value* retval = I.getReturnValue();
      if (retval) {
        m_Out << "      return " << makeValueRepr(retval) << ";\n";
      } else {
        m_Out << "      return;\n";
      }
    }

    void visitBranchInst(BranchInst &I) {
      if (I.isConditional()) {
        m_Out << "      if (" << encodeName(I.getCondition()) << ") {\n";
        printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(0), 2);
        size_t IndexTrue  = indexOf(*(I.getParent()->getParent()), *(I.getSuccessor(0)));
        m_Out << "        __bb = " << IndexTrue << ";\n";
        m_Out << "      } else { \n";
        printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(1), 2);
        size_t IndexFalse = indexOf(*(I.getParent()->getParent()), *(I.getSuccessor(1)));
        m_Out << "        __bb = " << IndexFalse << ";\n";
        m_Out << "      }\n";
        // TODO: impl
      } else {
        printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(0), 2);
        size_t Index = indexOf(*(I.getParent()->getParent()), *(I.getSuccessor(0)));
        m_Out << "      __bb = " << Index << ";\n";
      }
    }

    void visitLoadInst(LoadInst &I) {
      Value* V = I.getPointerOperand();
      if (isDirectAlloca(V)) {
        m_Out << "      var " << encodeName(&I) << " = ";
        m_Out << encodeName(V);
        m_Out << ";\n";
      } else {
        m_Out << "      var " << encodeName(&I) << " = ";
        m_Out << encodeName(V);
        m_Out << ".ref();\n";
      }
    }

    void visitStoreInst(StoreInst &I) {
      Value* V = I.getPointerOperand();
      if (isDirectAlloca(V)) {
        m_Out << "      ";
        m_Out << encodeName(V) << " = ";
        m_Out << makeValueRepr(I.getOperand(0));
        m_Out << ";\n";
      } else {
        m_Out << "      ";
        m_Out << encodeName(V) << ".set_ref(";
        m_Out << makeValueRepr(I.getOperand(0));
        m_Out << ");\n";
      }
    }

    void visitCallInst(CallInst &I) {
      JSBE_CURRENTLY_ASSUME(!I.hasStructRetAttr());

      if (I.getType() != Type::VoidTy) {
        m_Out << "      var " << encodeName(&I) << " = ";
      } else {
        m_Out << "      ";
      }

      m_Out << encodeName(I.getCalledValue()) << "(";

      bool PrintedArg = false;
      // first operand is called function itself
      for (CallSite::arg_iterator AI = I.op_begin()+1; AI != I.op_end(); ++AI) {
        if (PrintedArg) { m_Out << ", "; }
        m_Out << makeValueRepr((*AI));
        PrintedArg = true;
      }

      m_Out << ");\n";
    }

    void visitGetElementPtrInst(GetElementPtrInst &I) {
      m_Out << "      var " << encodeName(&I) << " = ";
#if 0
      m_Out << encodeName(I.getPointerOperand()) << ".element_at(";
      bool PrintedArg = false;
      for (User::op_iterator OI=I.idx_begin(); OI != I.idx_end(); ++OI) {
        if (PrintedArg) { m_Out << ", "; }
        m_Out << makeValueRepr(*OI);
        PrintedArg = true;
      }
      m_Out << ");\n";
#else
      m_Out << makeGetElementPtrExp(I.getPointerOperand(), I.op_begin()+1, I.op_end());
      m_Out << ";\n";
#endif

    }

    void visitCastInst(CastInst &I) {
      m_Out << "      var " << encodeName(&I) << " = __JSBE.cast(";
      m_Out << makeValueRepr(I.getOperand(0));
      m_Out << ");\n";
    }

    void visitPHINode(PHINode &I) {
#if 0 // already define at the top of the function
      m_Out << "      var " << encodeName(&I) << ";\n";
#endif
    }

    std::string makeBinaryOperator(int opcode) {
      switch (opcode) {
      case Instruction::Add:  return " + "; break;
      case Instruction::Sub:  return " - "; break;
      case Instruction::Mul:  return " * "; break;
      case Instruction::URem:
      case Instruction::SRem:
      case Instruction::FRem: return " % "; break;
      case Instruction::UDiv:
      case Instruction::SDiv:
      case Instruction::FDiv: return " / "; break;
      case Instruction::And:  return " & "; break;
      case Instruction::Or:   return " | "; break;
      case Instruction::Xor:  return " ^ "; break;
      case Instruction::Shl : return " << "; break;
      case Instruction::LShr:
      case Instruction::AShr: return " >> "; break;
      default:
        std::cerr << "Invalid operator type!" << opcode; abort();
        return "";
      }
    }

    void visitBinaryOperator(Instruction &I) {
      JSBE_CURRENTLY_ASSUME(!BinaryOperator::isNeg(&I));
      JSBE_CURRENTLY_ASSUME(I.getOpcode() != Instruction::FRem);

      m_Out << "      var " << encodeName(&I) << " = ";
      m_Out << makeValueRepr(I.getOperand(0));
      m_Out << makeBinaryOperator(I.getOpcode());
      m_Out << makeValueRepr(I.getOperand(1));
      m_Out << ";\n";
    }

    std::string makeComparationOperator(int opcode) {
      switch (opcode) {
      case ICmpInst::ICMP_EQ:  return " == ";
      case ICmpInst::ICMP_NE:  return " != ";
      case ICmpInst::ICMP_ULE:
      case ICmpInst::ICMP_SLE: return " <= ";
      case ICmpInst::ICMP_UGE:
      case ICmpInst::ICMP_SGE: return " >= ";
      case ICmpInst::ICMP_ULT:
      case ICmpInst::ICMP_SLT: return " < ";
      case ICmpInst::ICMP_UGT:
      case ICmpInst::ICMP_SGT: return " > ";
      default:
        std::cerr << "Invalid operator type!" << opcode; abort();
        return "";
      }
    }

    void visitICmpInst(ICmpInst &I) {
      m_Out << "      var " << encodeName(&I) << " = ";
      m_Out << makeValueRepr(I.getOperand(0));
      m_Out << makeComparationOperator(I.getPredicate());
      m_Out << makeValueRepr(I.getOperand(1));
      m_Out << ";\n";
    }

    void visitInstruction(Instruction &I) {
      // TODO: disable this: here is temporal workaround.
      m_Out << "      var " << encodeName(&I)
            << "; // not implemented yet:" << I.getOpcodeName() << "\n";
    }
  private:
    std::ostream& m_Out;
  };

  class JSWriter : public FunctionPass {
  public:
    static char ID;
    explicit JSWriter(std::ostream &o)
      : FunctionPass(reinterpret_cast<intptr_t>(&ID)), m_Out(o), m_Writer(o) {}

    virtual const char *getPassName() const { return "JS backend"; }

    virtual bool doInitialization(Module &M) {
      m_Out << std::endl;
      for (Module::global_iterator I = M.global_begin(), E = M.global_end(); I != E; I++) {
        m_Writer.printGlobalVariable(I);
      }
      m_Out << std::endl;
      return false;
    }

    virtual bool doFinalization(Module &M) {
      return false;
    }

    virtual bool runOnFunction(Function &F) {
      m_Writer.PrintFunctionSignature(F);
      m_Writer.PrintFunctionPrologue(F);

      size_t Index = 0;
      for (Function::iterator BB = F.begin(); BB != F.end(); ++BB) {
        m_Writer.printBasicBlockOrLoop(*BB, Index, 1 < F.size());
        Index++;
      }

      m_Writer.PrintFunctionEpilogue(F);
      m_Out << std::endl;
      return false;
    }

  private:
    std::ostream &m_Out;
    JSInstWriter m_Writer;
  };

  char JSWriter::ID = 0;

}

//===----------------------------------------------------------------------===//
//                       External Interface declaration
//===----------------------------------------------------------------------===//

FileModel::Model JSTargetMachine::addPassesToEmitFile(PassManagerBase &PM,
                                                      std::ostream &Out,
                                                      CodeGenFileType FileType,
                                                      bool /*Fast*/) {
  using namespace jsbackend_impl;

  PM.add(createGCLoweringPass());
  PM.add(createLowerAllocationsPass(true));
  PM.add(createLowerInvokePass());
  PM.add(createCFGSimplificationPass());   // clean up after lower invoke.
  PM.add(new JSWriter(Out));
  PM.add(createCollectorMetadataDeleter());

  return FileModel::AsmFile;
}


bool JSTargetMachine::addPassesToEmitFileFinish(PassManagerBase &,
                                                MachineCodeEmitter *, bool /*Fast*/) {
  return false; // passed successfully.
}
