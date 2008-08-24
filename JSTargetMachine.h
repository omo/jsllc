//===-- JSTargetMachine.h - TargetMachine for the JavaScript backend ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the TargetMachine that is used by the C backend.
//
//===----------------------------------------------------------------------===//

#ifndef JSTARGETMACHINE_H
#define JSTARGETMACHINE_H

#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"

namespace llvm {

struct JSTargetMachine : public TargetMachine {
  const TargetData DataLayout;       // Calculates type size & alignment

  JSTargetMachine(const Module &M, const std::string &FS)
    : DataLayout(&M) {}

  virtual FileModel::Model addPassesToEmitFile(PassManagerBase &PM,
                                               std::ostream &Out,
                                               CodeGenFileType FileType,
                                               bool /*Fast*/);

  virtual bool addPassesToEmitFileFinish(PassManagerBase &,
                                         MachineCodeEmitter *, bool /*Fast*/);

  // This class always works, but shouldn't be the default in most cases.
  static unsigned getModuleMatchQuality(const Module &M) { return 1; }

  virtual const TargetData *getTargetData() const { return &DataLayout; }
};

} // End llvm namespace


#endif
