#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
uint32_t HAL_GetTick(void);
#endif
