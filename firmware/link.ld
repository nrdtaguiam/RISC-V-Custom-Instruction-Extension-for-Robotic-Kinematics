/* link.ld
 * Linker script for riscv_3d_kinematics_accel core simulation.
 */

OUTPUT_ARCH( "riscv" )
ENTRY(_start)

MEMORY
{
  ROM (rx)  : ORIGIN = 0x00000000, LENGTH = 16K
  RAM (rwx) : ORIGIN = 0x00004000, LENGTH = 16K
}

SECTIONS
{
  /* Text Section (Code) */
  .text 0x00000000 : {
    . = ALIGN(4);
    KEEP(*(.init))     /* Force initialization section at 0x00000000 */
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections */
    *(.rodata)         /* Read-only data */
    *(.rodata*)        /* Read-only data* */
    . = ALIGN(4);
  } > ROM

  /* Data Section */
  .data : {
    . = ALIGN(4);
    *(.data)           /* Initialized data */
    *(.data*)          /* Initialized data* */
    *(.sdata)          /* Small initialized data */
    *(.sdata*)
    . = ALIGN(4);
  } > RAM AT > ROM

  /* BSS Section (Uninitialized data) */
  .bss : {
    . = ALIGN(4);
    __bss_start = .;
    *(.bss)
    *(.bss*)
    *(.sbss)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
  } > RAM

  /* End of memory */
  _end = .;
}
