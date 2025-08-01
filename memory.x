MEMORY
{
  FLASH : ORIGIN = 0x80000000, LENGTH = 1024K
  RAM : ORIGIN = 0x00000000, LENGTH = 640K
  RAM_NC : ORIGIN = 0x000A0000, LENGTH = 128K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
_stext = ORIGIN(RAM) + 0x420;

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/* SECTIONS {
     .ram2bss (NOLOAD) : ALIGN(4) {
       *(.ram2bss);
       . = ALIGN(4);
     } > RAM2
   } INSERT AFTER .bss;
*/
SECTIONS
{
  .ram_nc (NOLOAD) : ALIGN(16)
  {
    __ram_nc_start = .;
    KEEP(*(.ram_nc));
    __ram_nc_end = .;
  } > RAM_NC
}
