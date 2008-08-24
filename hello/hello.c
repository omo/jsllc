
void* malloc(int);
void  putc(char);
void  extern_puts(const char*);

int add(int a, int b) {
  int x = a + b;
  return x;
}

int fac(int n) {
  int x = 1, i = 0;
  for (i=0; i<n; i++) { x *= (i+1); }
  return x;
}

void puts(const char* str) {
  while (*str) {
    putc(*(str++));
  }
}

void put_hello() {
   puts("hello\n");
}

void put_hello_extern() {
   extern_puts("hello\n");
}

void upcase(char* str) {
  while (*str) {
    if ('A' <= *str && *str <= 'z') { *str -= ('a' - 'A'); }
    str++;
  }
}

void put_hello_upcase() {
   char hello[] = "hello";
   upcase(hello);
   puts(hello);
}

void put_hello_upcase_extern() {
   char hello[] = "hello";
   upcase(hello);
   extern_puts(hello);
}
