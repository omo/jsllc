
* JSBackend : toy llvm backend for javascript

components:
- jsllc : llc dialect to (only) use JSBackend.
  see hello/Makefile to know how to use
- JSBackend: CBackend-based code generator

build:

  # checkout llvm trunk
  $ svn co llvm http://llvm.org/svn/llvm-project/llvm/trunk/
  # make llvm
  $ cd trunk
  $ configure; make
  # checkout jsllc into "tools" directory
  $ cd tools
  $ git clone http://github.com/omo/jsllc/master
  # make jsllc
  $ cd jsllc
  $ make
  # try the example
  $ cd hello
  $ make
  $ firefox hello.html
