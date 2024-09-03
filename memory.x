/* https://docs.rs/cortex-m-rt/0.7.3/cortex_m_rt/#memoryx */
/* copy of generated memory.x from stm32f3xx-hal's build.rs
 * https://github.com/stm32-rs/stm32f3xx-hal/blob/9ca5e01/build.rs
 */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 64K
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 4K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 12K
}
