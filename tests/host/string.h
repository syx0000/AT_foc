#ifndef HOST_STRING_H
#define HOST_STRING_H
#include <stddef.h>
void *memcpy(void *d,const void *s,size_t n) {
  unsigned char *a=d; const unsigned char *b=s; while(n--) *a++=*b++; return d;
}
void *memset(void *d,int c,size_t n) {
  unsigned char *a=d; while(n--) *a++=(unsigned char)c; return d;
}
int memcmp(const void *a,const void *b,size_t n) {
  const unsigned char *x=a,*y=b;
  while(n--) { if(*x!=*y) return *x<*y?-1:1; x++; y++; } return 0;
}
#endif
