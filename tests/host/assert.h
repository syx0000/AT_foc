#ifndef HOST_ASSERT_H
#define HOST_ASSERT_H
#define assert(condition) do { if (!(condition)) __builtin_trap(); } while (0)
#endif
