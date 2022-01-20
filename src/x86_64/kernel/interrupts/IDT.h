/*
* Copyright (C) 2022 Ian Marco Moffett.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef IDT_H
#define IDT_H

// The IDT entries are called gates.
#define TRAP_GATE_FLAGS 0x8F
#define INT_GATE_FLAGS 0x8E

#define ENTRY_LIMIT 256

#include "../util/types.h"

// IDT entry.
typedef struct {
    uint16_t isr_address_low;   // Lower 16 bits of ISR address
    uint16_t kernel_cs;         // Code segment the ISR is in.
    uint8_t reserved;           // Reserved by intel.
    uint8_t attr;               // Flags.
    uint16_t isr_address_high;  // Upper 16 bits of ISR address.
} __attribute__((packed)) idt_entry_32_t;


// Where the location of the IDT is stored.
typedef struct {
    uint16_t limit;
    uint32_t base;
} __attribute__((packed)) idtr_32_t;

idt_entry_32_t idt32[ENTRY_LIMIT];
idtr_32_t idtr32;


// Interrupt frame.
typedef struct {
    uint32_t eip;
    uint32_t cs;
    uint32_t eflags;
    uint32_t sp;
    uint32_t ss;
} __attribute__((packed)) int_frame_32_t;


// Exception handler (no error code).
__attribute__((interrupt)) void default_exception_handler(int_frame_32_t* frame) {
    
}

__attribute__((interrupt)) void default_exception_handler_err_code(int_frame_32_t* frame, uint32_t error_code) {

}


__attribute__((interrupt)) void default_int_handler(int_frame_32_t* frame) {
    ++frame->eip;
}


void set_idt_desc_32(uint8_t entry_number, void* isr, uint8_t flags) {
    idt_entry_32_t* descriptor = &idt32[entry_number];
    descriptor->isr_address_low = (uint32_t)isr & 0xFFFF;
    descriptor->kernel_cs = 0x8;        // Kernel code segment.
    descriptor->reserved = 0;
    descriptor->attr = flags;
    descriptor->isr_address_high = ((uint32_t)isr >> 16) & 0xFFFF;
}


void setup_idt32() {
    idtr32.limit = (uint16_t)(8 * 256);
    idtr32.base = (uint32_t)&idt32;         // If it doesn't work FIXME.
    __asm__ __volatile__("lidt %0" : : "memory"(idtr32));  // Load IDT to IDTR.

    for (uint8_t entry = 0; entry < 32; ++entry) {
        switch (entry) {
            case 8:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 17:
            case 21:
                // Exception takes error code.
                set_idt_desc_32(entry, default_exception_handler_err_code, TRAP_GATE_FLAGS);
                break;
            default:
                // Exception doesn't take error code.
                set_idt_desc_32(entry, default_exception_handler, TRAP_GATE_FLAGS);
                break;

        }
    }

    for (uint16_t entry = 32; entry < 256; ++entry) {
        set_idt_desc_32(entry, default_int_handler, INT_GATE_FLAGS);
    }
}

#endif
