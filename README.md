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

=================================================================================================================================
Cyclone-V SoC development procedures based upon Quartus 17.0 Lite
1. HW phase:
    a. Create Quartus Project, preferably from GHRD and modify based upon it.
    b. Generate rbf file, which U-boot uses to configure FPGA fabric.
    Note*:
    1.When generating .rbf file from .sof file, select in properties->compression.
    2.Mode is 1-bit passive serial.

2. SW phase:
    a. Preloader: Use bsp-editor with eds embedded tools in hps_isw_handoff folder, the bsp tool will generate Makefile to create preloader.
                  The generated image is preloader.mkimage. In the settings gui, it can be instructed to load u-boot, file is hardcoaded to u-boot.img
    b. U-boot: check-out socfpga-uboot from git and generate u-boot.img according to instructions.
    c. Linux-kernel: Build linux kernel according to instructions.

3. Application notes:
    Utilize U-boot and Linux to hasten bare-metal development:
    To develop bare-metal app, it is advisable to utilize u-boot, which by default loads linux kernel. A seperate u-boot script <name>.scr can be created
    to load bare-metal .bin file. This is convenient as although preloader can load directly the bare-metal app, bypassing u-boot. It involves manually
    and physically deasserting and asserting SD card to burn bare-metal .bin. Using linux with u-boot, the bare-metal .bin file can be transfered via USB OTG
    cable directly to the FAT32 partition of the SD card, greatly reduces manual work as SD card can safely stay in this socket during development phase.
    At the same time, u-boot can also by default load linux to allow other embedded linux features to be used.

    Enable watch-dog warm-reset:
    During bare-metal developement, watchdog can be enabled to warm reset the system to avoid pushing warm-reset button or power-cycle the board. Watch-dog
    warm reset can bring the system to boot to u-boot by loading preloader upon warm-reset. This avoids the need of manually resetting the board when bare-metal 
    app finishes debugging execution

