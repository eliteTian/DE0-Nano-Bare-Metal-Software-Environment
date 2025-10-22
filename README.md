This repo contains the necessary scripts to create a bootbale bare-metal .bin file
which can be baked into existing preloader and executes immediately.

File description:
cycloneV-dk-ram-modified.ld: 
    Linker file for Cyclone V SoC.

preloader-mkpimage.bin: 
    preloader that is precompiled which loads the next stage program at address 0x40000 at partition 0xA2.

gen_img:
    mkimage command that is executed under embedded_command_shell.sh environment, which generates the main.bin.img from main.bin.
    can be incorporated into Makefile.

Instructions:
    1.Load preloader: sudo dd if=preloader-mkpimage.bin of=/dev/sdb3 

    2.Load app: sudo dd if=main.bin.img of=/dev/sdb3 bs=262144 seek=1


