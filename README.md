# gzml

Linux CLI Monitor Loader for Motorola/Freescale/NXP MC68HC908GZ60

## Terminology

### Monitor loader

Monitor loader is a PC side software which can update program memory of an
empty (virgin) microcontroller.
Disadvantages are it needs special hardware interface and it is slow.
My one is available
[here](https://github.com/butyi/gzml.py/) in Python language, or
[here](https://github.com/butyi/gzml.c/) in C language.
I propose the Python one.

### Bootloader

Bootloader is software embedded in the microcontroller, which can receive data
from a hardware interface and write it into its own program memory.
This software needs to be programmed into the microcontroller only once by a
monitor loader. 
My one is available 
[here](https://github.com/butyi/gzbl/).

### Downloader

Downloader is PC side software. The communication partner of Bootloader.
It can send the pre-compiled software (or any other data) to data to
microcontroller through the supported hardware interface.
My one is available
[here](https://github.com/butyi/gzdl.py/) in Python language, or
[here](https://github.com/butyi/gzdl.c/) in C language.
I propose the Python one.

### Application

Application is the real software for the main purpose. It is easier if you
start your application software from an application template.
Application template is a sample software which initializes and uses almost
all modules of microcontroller for some basic purpose. You can download into
uC and it already works and does something. You just need to modify it for your 
purpose. 
My one is available 
[here](https://github.com/butyi/gzat/).

## Why I write a monitor loader for HC908GZ in 2019 while this uC should be already in a museum?

My motivation was, I still have working hardwares, but I couldn't update the sw because loaders developed
for WinXP and built in COM port do not work on Win7, Win10 and USB-RS232 devices.
Moreover, I switched from Windows to Linux several years ago.
Therefore I could not use prog08sz any longer, as I used 10 years before.

## What are supported by gzml?

It supports only basic monitor features what are needed to erase flash and download
software into flash. Usually I just use monitor loader to download the bootloader into
an empty uC. From this point on, system software can be downloaded easily through
RS232 UART or CAN any times. Therefore I just implemented those functions, which are
needed to download my bootloader.

### Supports
- Mass erase entire flash memory
- Write S19 file content into flash memory
- Dump data from memory
- Define security bytes for connect device

### Does not support
- Erase range
- Debugging
- Change baud rate
- Write bytes or ranges
- ... (many other features prog08sz supported) 

## Hardware

To be HW interface easy and cheap, buy TTL USB-Serial interface from China.
I use FT232RL FTDI USB to TTL Serial Adapter Module for communication. This is supported by both Linux and Windows 10.
The mini step up power supply board is used to generate Vtst for IRQ pin.
With this, you just need some cable and that's it.

![GZ monitor interface circuit](https://github.com/butyi/gzml/raw/master/FT232RL_FTDI_USB_to_TTL_Serial_Adapter_Module_for_monitor_download_into_CEM35_CPU_with_MC68HC908GZ60.jpg)

My first CPU card circuit diagram (schematic) is visible here. 

![GZ monitor interface circuit](https://github.com/butyi/gzml/raw/master/homemat_cem35_cpu_0_sch.jpg)

More info is [here](http://butyi.hu/cem35).

## How get use gzml?

The keyword is Linux. The software was developed on my Ubuntu Linux 16.04 LTS. 
- Open terminal
- First check out files into a folder.
- Compile gzml by `gcc gzml.c -o gzml`
- Enjoy it.

## How to use gzml?
`./gzml`

Print out usual help about version, command line options, tipical usage. 

`./gzml -e`

Erase the entire Flash memory. If the security failed during connection phrase,
the command will only mass erase Flash 1 (0x8000-0xFFFF) including security bytes.
In this case, to erase Flash 2 (0x0462 - 0x7FFF) too, you need to execute the same
command again. Since security bytes were erased, security will pass with default
full 0xFF bytes, and Flash 2 will also erased.

`./gzml hmdl.s19`

Download software into Flash memory. This command does not erase Flash even if it
would be needed. If you download into not erased memory, you will have verify errors.
 
`./gzml -d 0xFF00`

Dump memory content from address 0xFF00 - 0xFFFF. Default length of dump is 256.
You can increase/decrease it by -l command line parameter. See help for more details.

## Further Development

If you only improve PC side features, just 
- Edit gzml.c 
- Compile gzml by `gcc gzml.c -o gzml`
- Enjoy it.

If you want to improve uC side code too (as I did), use `bash update_loader`.

You need php installed. Without php you need to convert loader routines 
from S19 to initialized C array manually. It is not a nice job, but possible.
You can also write script for this in your favorite programming language.
The task what my php script does is written easy to understand way in the script
and the generated file (loader.c).

Let see `update_loader` bash script. It:
- compiles the flash erase and program routines (loader.asm) using asm8 compiler
- php script creates C source defines for start address of routines from LST file
- php script creates C source array definition from S19 file
- deletes gzml previous instance if there is
- compiles gzml by gcc 

Development procedure:
- Modify any source (gzml.c , loader.asm)
- Call `bash update_loader`
- Try it

## Baud rate

Unfortunately I couldn't get work non-standard baud rate (14400) on Linux.
Therefore I decided to change quarz from 8MHz to 5.33MHz to have standard
(9600) monitor baudrate. I haven't found 5.33MHz quarz only if I buy 1000 pieces. :)
I could only buy 5.2MHz from Conrad, which is still inside the tolerance range.
So, baud rate is 9600 when PTB4=0 or 4800 when PTB4=1. Defauls baudrate is 9600.
If you do not know state of PTB4, first use default 9600. If you got error message
"Target echo failed, sent 0xFF, got 0xFE", try it with 4800.
Do not forget, quarz change need to update PLL setup in system software too.

## Test

I just tested those features what I used during download bootloader. Not all available
features were tested. If you find bug, or do not like how it works, feel free to fix or modify.

### Tested functions
- Mass erase when security passed 
- Mass erase when security failed
- Program into area 0x8000 - 0xFFFF
- Program into area 0x0980 - 0x1000
- Dump from memory
- Motherboard mounted ttyS0
- USB Serial interface ttyUSB0 
  (both [FT231XS](https://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_Chipi-X.pdf) and
  [ATEN UC232A1](https://www.aten.com/global/en/products/usb-&-thunderbolt/usb-converters/uc232a1/) ) 

## License

This is free. You can do anything you want with it.
While I am using Linux, I got so many support from free projects, I am happy if I can help for the community.

## Thanks

Many thanks for my predecessors! They saved my hardwares for further use
and by this way helped to save green environment. I mean I do not need to produce/buy new device.

### Predecessors
- Based on bl08.c (https://github.com/jaromir-sukuba/bl08):
- Copyright (c) 2004,2008	Kustaa Nyholm
- Copyright (c) 2008 	Robert Larice (SWI return to MON, QY2 chip)
- Copyright (c) 2010 	Tormod Volden
- Copyright (c) 2013 	Jaromir Sukuba (added A and B ROM routines entry instances, 


### Keywords
Motorola, Freescale, NXP, MC68HC908GZ60, 68HC908GZ60, HC908GZ60, MC908GZ60, 908GZ60, HC908GZ48, HC908GZ32, HC908GZ16, HC908GZ, 908GZ

###### 2019 Janos Bencsik



