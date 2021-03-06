MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 2M
    RAM : ORIGIN = 0x20000000, LENGTH = 192K
    CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/*
   ENTRY directive marks the given function as used, so the
   linker doesn't discard it.
*/
ENTRY(Reset);

/*
    By default, linker will discard RESET_VECTOR if it's not
    referenced from the main function. So we need EXTERN to
    keep it in the source code.
*/
EXTERN(RESET_VECTOR);
EXTERN(SYSTICK_VECTOR);

SECTIONS
{
    .vector_table ORIGIN(FLASH) :
    {
        _vt_start = .;
        // First entry: the vectored interrupt table
        // initial stack pointer value
        LONG(ORIGIN(RAM) + LENGTH(RAM));    // 0x0000 0000

        // Second entry: reset vector
        // KEEP forces the linker to insert the reset_vector section here
        KEEP(*(.vector_table.reset_vector)); // 0x0000 0004

        . = ABSOLUTE(0x0800003c);
        KEEP(*(.vector_table.systick_vector));
    } > FLASH

    .text :
    {
        *(.text .text.*);
    } > FLASH

    .rodata : {
        *(.rodata .rodata.*);
    } > FLASH

    .bss : {
        _sbss = .;
        *(.bss .bss.*);
        _ebss = .;
    } > RAM

    .data : AT(ADDR(.rodata) + SIZEOF(.rodata))
    {
        _sdata = .;
        *(.data .data.*);
        _edata = .;
    } > RAM

    _sidata = LOADADDR(.data);

    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*);
    }
}
