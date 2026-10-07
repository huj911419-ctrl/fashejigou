#ifndef TEST_USB_MAIN_H
#define TEST_USB_MAIN_H
#include <stdint.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t previous);
#endif
