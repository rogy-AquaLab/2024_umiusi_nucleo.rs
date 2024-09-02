/* https://docs.rs/cortex-m-rt/0.7.3/cortex_m_rt/#memoryx */
/* https://github.com/rtic-rs/rtic/blob/67c2286/examples/stm32f3_blinky/memory.x */

MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 256K
    RAM : ORIGIN = 0x20000000, LENGTH = 40K
}
