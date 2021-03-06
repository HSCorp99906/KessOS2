;; Copyright (C) 2022 Ian Marco Moffett.
;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions
;; are met:
;; 1. Redistributions of source code must retain the above copyright
;; notice, this list of conditions and the following disclaimer.
;; 2. Redistributions in binary form must reproduce the above copyright
;; notice, this list of conditions and the following disclaimer in the
;; documentation and/or other materials provided with the distribution.

;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;; POSSIBILITY OF SUCH DAMAGE.




bits 16     ; Tells nasm to emit 16 bit code.
org 0x7C00  ; Adds label offsets corrently.

; Setup segment registers so random combinations aren't being used.
xor ax, ax
mov ds, ax
mov es, ax
mov ss, ax

mov bp, 0x9C00
mov sp, bp
cld 

%define KERNEL_OFFSET 0x1000


_start:
    mov [BOOT_DRIVE], dl

    mov si, start_msg
    call puts

    mov si, newline
    call puts

    mov si, boot_choice
    call puts

    mov si, reboot_choice
    call puts

    call read_keys      ; Start reading keys.
    
    mov ah, 0x00    ; Change video mode function.
    mov al, 0x02    ; Video mode (80x25).
    int 0x10        ; Video services interrupt.

    mov si, boot_msg
    call puts

     ; Delay.
    mov cx, 0xF
    mov dx, 0x4240
    mov ah, 0x86
    int 0x15

    mov cx, 0xF
    mov dx, 0x4240
    mov ah, 0x86
    int 0x15
    
    mov bx, KERNEL_OFFSET
    mov dl, [BOOT_DRIVE]
    mov dh, 0x01
    call disk_load

    cli
    lgdt [gdt_desc]

    mov eax, cr0
    or eax, 0x1
    mov cr0, eax

    ; We are now in protected mode.
    ; We need to flush the CPU's pipeline so we don't have 16-bit
    ; processes running, and we also need to change the code segment
    ; to the one we defined in our GDT since this one won't work anymore.
    ; We can do all that by doing a far jump.
    ; We will jump to our new code segment offseted by a label.

    jmp CODE_SEG:init_pm

    jmp $

BOOT_DRIVE: db 0

read_keys:
    mov ah, 0x00    ; Read key function.
    int 0x16        ; Keyboard interrupts.
    cmp al, 'r'
    je reboot
    cmp al, 'b'
    je rm_ret
    jmp read_keys

; Jump to 0xFFFF offsetted by 0x0.
reboot: jmp 0xFFFF:0x0

puts:
    mov ah, 0x0E        ; Teletype function.
    lodsb               ; Load a byte from SI into AL and increment SI.
    or al, al           ; If AL is zero then zero flag will be set.
    jz rm_ret           ; If zero flag is set then return.
    int 0x10            ; BIOS video interrupt.
    jmp puts            ; Continue.

rm_ret: ret

disk_load:
        push dx         ; Save DX so we can see how many sectors were requested.
        mov ah, 0x02    ; BIOS read sectors function.
        mov al, dh      ; Read DH sectors.
        mov ch, 0x00    ; Cylinder 0.
        mov dh, 0x00    ; Head 0.
        mov cl, 0x02
        int 0x13
        jc disk_error
        pop dx
        cmp dh, al
        jne disk_error
        ret

disk_error:
    mov si, disk_error_msg
    call puts
    cli                 ; Make CPU ignore interrupts.
    hlt                 ; Halt the CPU. 


start_msg: db "Welcome to KessOS! Please select an option.", 0x0D, 0x0A, 0
boot_choice: db "[B]oot", 0x0D, 0x0A, 0
reboot_choice: db "[R]eboot", 0x0D, 0x0A, 0
boot_msg: db "Booting KessOS..", 0x0D, 0x0A, 0
newline: db 0x0D, 0x0A, 0
disk_error_msg: db "FATAL: Failed to read from disk.", 0x0D, 0x0A, 0


; GDT: It defines what 
; memory can be executed (The Code Descriptor), 
; and what area contains data (Data Descriptor). 

gdt_start:
    gdt_null:
        dd 0x0
        dd 0x0
    gdt_code:
        ; Type flags:
        ; Present: 1 since we are using code.
        ; Privilege: 00 higest privilige.
        ; Descriptor type: 1 for code/data.
        ; Code: 1.
        ; Conforming: 0 so segments with a lower privilege may not call code in this segment.
        ; Readable: 1.
        ; Accessed: 0.

        ; Other flags:
        ; Granularity: 1 so we can reach father into memory.
        ; 32-bit default: 1 since our segment will have 32-bit code.
        ; 64-bit code segment: 0.
        ; AVL 0.
        ; Limit: 1111.

        dw 0xFFFF       ; Limit.
        dw 0x0          ; Base.
        db 0x0          ; Base.
        db 10011010b    ; 1st flags, type flags.
        db 11001111b    ; 2nd flags, type flags.
        db 0x0
    gdt_data:
        ; Type flags: 
        ; Code: 0.
        ; Expand down: 0.
        ; Writable: 0.
        ; Accessed: 0.

        dw 0xFFFF       ; Limit.
        dw 0x0          ; Base.
        db 0x0          ; Base.
        db 10010010b    ; 1st flags, type flags.
        db 11001111b    ; 2nd flags, type flags.
        db 0x0
gdt_end:


gdt_desc:
    dw gdt_end - gdt_start - 1
    dd gdt_start

CODE_SEG equ gdt_code - gdt_start
DATA_SEG equ gdt_data - gdt_start

bits 32

init_pm:
   ; Our segments are incorrect so we reset them.
    mov ax, DATA_SEG        ; Set AX to data segments.
    mov ds, ax
    mov ss, ax
    mov es, ax
    mov fs, ax
    mov gs, ax

    mov ebp, 0x9000
    mov esp, ebp
    jmp _PM
    
_PM:
    call KERNEL_OFFSET
    hlt

; Sector padding to position our 
; magic number at the last two bytes.
times 510-($-$$) db 0

; Magic number.
dw 0xAA55
