//===-- jsllc.cpp - Implement the LLVM JavaScript Code Generator ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This is the jsllc code generator driver. It provides a convenient
// command-line interface for generating JavaScript code given LLVM bitcode.
//
//===----------------------------------------------------------------------===//

#include "llvm/Bitcode/ReaderWriter.h"
#include "llvm/CodeGen/FileWriters.h"
#include "llvm/CodeGen/LinkAllCodegenComponents.h"
#include "llvm/Target/SubtargetFeature.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetMachineRegistry.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Module.h"
#include "llvm/ModuleProvider.h"
#include "llvm/PassManager.h"
#include "llvm/Pass.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/PluginLoader.h"
#include "llvm/Support/FileUtilities.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/System/Signals.h"
#include "llvm/Config/config.h"
#include "llvm/LinkAllVMCore.h"
#include <fstream>
#include <iostream>
#include <memory>

#include "JSTargetMachine.h"

using namespace llvm;

// General options for llc.  Other pass-specific options are specified
// within the corresponding llc passes, and target-specific options
// and back-end code generation options are specified with the target machine.
//
static cl::opt<std::string>
InputFilename(cl::Positional, cl::desc("<input bitcode>"), cl::init("-"));

static cl::opt<std::string>
OutputFilename("o", cl::desc("Output filename"), cl::value_desc("filename"));

// GetFileNameRoot - Helper function to get the basename of a filename.
static inline std::string
GetFileNameRoot(const std::string &InputFilename) {
  std::string IFN = InputFilename;
  std::string outputFilename;
  int Len = IFN.length();
  if ((Len > 2) &&
      IFN[Len-3] == '.' && IFN[Len-2] == 'b' && IFN[Len-1] == 'c') {
    outputFilename = std::string(IFN.begin(), IFN.end()-3); // s/.bc/.s/
  } else {
    outputFilename = IFN;
  }
  return outputFilename;
}

static std::ostream *GetOutputStream(const char *ProgName) {
  if (OutputFilename != "") {
    if (OutputFilename == "-")
      return &std::cout;

    // Make sure that the Out file gets unlinked from the disk if we get a
    // SIGINT
    sys::RemoveFileOnSignal(sys::Path(OutputFilename));

    return new std::ofstream(OutputFilename.c_str());
  }

  if (InputFilename == "-") {
    OutputFilename = "-";
    return &std::cout;
  }

  OutputFilename = GetFileNameRoot(InputFilename);

  // Make sure that the Out file gets unlinked from the disk if we get a
  // SIGINT
  sys::RemoveFileOnSignal(sys::Path(OutputFilename));

  std::ostream *Out = new std::ofstream(OutputFilename.c_str());
  if (!Out->good()) {
    std::cerr << ProgName << ": error opening " << OutputFilename << "!\n";
    delete Out;
    return 0;
  }

  return Out;
}

// main - Entry point for the llc compiler.
//
int main(int argc, char **argv) {
  llvm_shutdown_obj X;  // Call llvm_shutdown() on exit.
  cl::ParseCommandLineOptions(argc, argv, "llvm system compiler\n");
  sys::PrintStackTraceOnErrorSignal();

  TargetMachine::CodeGenFileType FileType = TargetMachine::AssemblyFile;
  bool Fast = true;

  // Load the module to be compiled...
  std::string ErrorMessage;
  std::auto_ptr<Module> M;

  std::auto_ptr<MemoryBuffer> Buffer(
                   MemoryBuffer::getFileOrSTDIN(InputFilename, &ErrorMessage));
  if (Buffer.get())
    M.reset(ParseBitcodeFile(Buffer.get(), &ErrorMessage));
  if (M.get() == 0) {
    std::cerr << argv[0] << ": bitcode didn't read correctly.\n";
    std::cerr << "Reason: " << ErrorMessage << "\n";
    return 1;
  }
  Module &mod = *M.get();

#if 0
  // Find CTargetMachine
  std::auto_ptr<TargetMachine> target;
  for (TargetMachineRegistry::iterator i=TargetMachineRegistry::begin();
       i != TargetMachineRegistry::end(); ++i)  {
     //std::cout << i->Name << std::endl;
     if (0 == strcmp(i->Name, "c")) {
        target.reset(i->CtorFn(mod, ""));
        break;
     }
  }
#else
  std::auto_ptr<TargetMachine> target(new JSTargetMachine(mod, ""));
#endif

  assert(target.get() && "Could not allocate target machine!");
  TargetMachine &Target = *target.get();

  // Figure out where we are going to send the output...
  std::ostream *Out = GetOutputStream(argv[0]);
  if (Out == 0) return 1;

  // If this target requires addPassesToEmitWholeFile, do it now.  This is
  // used by strange things like the C backend.
  if (Target.WantsWholeFile()) {
    PassManager PM;
    PM.add(new TargetData(*Target.getTargetData()));
    PM.add(createVerifierPass());

    // Ask the target to add backend passes as necessary.
    if (Target.addPassesToEmitWholeFile(PM, *Out, FileType, Fast)) {
      std::cerr << argv[0] << ":" << __LINE__ << ": target does not support generation of this"
                << " file type!\n";
      if (Out != &std::cout) delete Out;
      // And the Out file is empty and useless, so remove it now.
      sys::Path(OutputFilename).eraseFromDisk();
      return 1;
    }
    PM.run(mod);
  } else {
    // Build up all of the passes that we want to do to the module.
    ExistingModuleProvider Provider(M.release());
    FunctionPassManager Passes(&Provider);
    Passes.add(new TargetData(*Target.getTargetData()));

#ifndef NDEBUG
    Passes.add(createVerifierPass());
#endif

    // Ask the target to add backend passes as necessary.
    MachineCodeEmitter *MCE = 0;

    switch (Target.addPassesToEmitFile(Passes, *Out, FileType, Fast)) {
    default:
      assert(0 && "Invalid file model!");
      return 1;
    case FileModel::Error:
      std::cerr << argv[0] << ":" << __LINE__ << ": target does not support generation of this"
                << " file type!\n";
      if (Out != &std::cout) delete Out;
      // And the Out file is empty and useless, so remove it now.
      sys::Path(OutputFilename).eraseFromDisk();
      return 1;
    case FileModel::AsmFile:
      break;
    }

    if (Target.addPassesToEmitFileFinish(Passes, MCE, Fast)) {
      std::cerr << argv[0] << ":" << __LINE__
                << ": target does not support generation of this"
                << " file type! \n";
      if (Out != &std::cout) delete Out;
      // And the Out file is empty and useless, so remove it now.
      sys::Path(OutputFilename).eraseFromDisk();
      return 1;
    }

    Passes.doInitialization();

    // Run our queue of passes all at once now, efficiently.
    // TODO: this could lazily stream functions out of the module.
    for (Module::iterator I = mod.begin(), E = mod.end(); I != E; ++I)
      if (!I->isDeclaration())
        Passes.run(*I);

    Passes.doFinalization();
  }

  // Delete the ostream if it's not a stdout stream
  if (Out != &std::cout) delete Out;

  return 0;
}
