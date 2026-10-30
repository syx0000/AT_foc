#ifndef HOST_STDIO_H
#define HOST_STDIO_H
/* Freestanding tests report success through the runner's exit-code check. */
static int printf(const char *format, ...) { (void)format; return 0; }
#endif
