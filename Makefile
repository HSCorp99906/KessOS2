all:
	@ ##
	@ # Buildscruot for KessOS.
	@ #
	@ # @Makefile
	@ # @version 1.0	
	nasm -fbin src/x86_64/boot/bootloader.S -o bin/bootloader.bin
	nasm -felf src/x86_64/kernel/kasm.S -o obj/kasm.o
	gcc -c -m32 -masm=intel src/x86_64/kernel/kmain.c -ffreestanding -fno-pie -fstack-protector -o obj/kmain.o
	i686-elf-ld -Tlink.ld obj/kasm.o  obj/kmain.o --oformat binary -o bin/kernel.bin
	cat bin/bootloader.bin bin/kernel.bin > bin/KessOS.bin
	@ # Prepare the image.
	sudo dd if=/dev/zero of=KessOS.img bs=1024 count=1440
	@ # Put the OS stuff in the image.
	sudo dd if=bin/KessOS.bin of=KessOS.img
	sudo dd if=KessOS.img of=/dev/sdb


run:
	sudo qemu-system-x86_64 -hda /dev/sdb -monitor stdio
