BIN_DIR=../../../Debug/bin/
SRC=hello.c
all:
	${BIN_DIR}/clang --emit-llvm-bc hello.c
	${BIN_DIR}/clang --emit-llvm    hello.c
	${BIN_DIR}/opt   --std-compile-opts -print hello.bc -o=- > hello_opt.bc 2> hello_opt.ll
	${BIN_DIR}/jsllc hello_opt.bc -o hello_opt.js
	${BIN_DIR}/jsllc hello.bc -o hello.js

clean:
	-rm *.ll
	-rm *.bc
	-rm *_opt.*
	-rm hello.js

.PHONY: all