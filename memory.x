MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 2M
    RAM : ORIGIN = 0x20000000, LENGTH = 256K
    CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

ENTRY(Reset);

EXTERN(RESET_VECTOR);

SECTIONS
{
    .vector_table ORIGIN(FLASH) :
    {
        LONG(ORIGIN(RAM) + LENGTH(RAM));

        KEEP(*(.vector_table.reset_vector));
    } > FLASH

    .text :
    {
        *(.text .text.*);
    } > FLASH

    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*);
    }
}

/* _stack_start = ORIGIN(CCRAM) + LENGTH(CCRAM); */
