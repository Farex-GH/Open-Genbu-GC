#ifndef ARM_UTILS_H
#define ARM_UTILS_H

#define __DSB do {        \
    asm volatile ("dsb"); \
} while(0)

#define __ISB do {        \
    asm volatile ("isb"); \
} while(0)

#define __WFI do {        \
    asm volatile ("wfi"); \
} while(0)

#define __WFE do {        \
    asm volatile ("wfe"); \
} while(0)

#define __DISABLE_IRQ do {    \
    asm volatile ("cpsid i"); \
} while (0)

#define __ENABLE_IRQ do {     \
    asm volatile ("cpsie i"); \
} while (0)

#define __SEV do {        \
    asm volatile ("sev"); \
} while (0)

#define __NOP do {        \
    asm volatile ("nop"); \
} while (0)

#endif
