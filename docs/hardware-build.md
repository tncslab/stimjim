# Building your own Stimjim

How to get boards made, populated and boxed. Nothing here depends on which firmware you run;
[../README.md](../README.md) covers flashing and first use.

Here are the steps to build your very own Stimjim box. 

### Option 1: Assemble (solder components onto) the board yourself.

1. Order printed circuit boards (PCBs) for the [main board](../PCB/stimjimFabricationFiles_v0.181.zip) (4-layer, 129x79.5 mm, otherwise standard order parameters) and the [enclosure front and back panels](../PCB/stimjimPanelFabricationFiles_v0.18.zip) (1-layer, 2 designs, total size 140x71 mm) from [JLCPCB](https://jlcpcb.com/), [Seeedstudio](https://www.seeedstudio.com/fusion_pcb.html) or any other PCB manufacturer. Note that for the panels, the PCB manufacturer may alert you to the fact that there is no exposed copper. This is correct, there should be a copper layer entirely covered by soldermask.
2. Order [components](../stimjim_BOM.xlsx) (entirely from [Digikey](https://www.digikey.com/)).
3. Solder components onto the PCB, using the [schematic](../schematic.pdf) and [layout](../pcb.pdf) files for reference. This may take a few hours, depending on your soldering experience and capability. I typically use a pair of good tweezers, leaded solder paste, and a soldering iron, although a reflow oven also will work as well and may be easier.

### Option 2: Use [SeeedStudio's assembly service](https://www.seeedstudio.com/fusion_pcb.html). 
Recent estimate is about $800 for two assembled boards, plus another roughly $100 for the enclosure and panels. Thanks to Vincent Prevosto for trying this out and submitting these instructions!

1.  The main board PCB and components can be ordered and assembled together on SeeedStudio.
    To order assembled boards, go to [https://www.seeedstudio.com/fusion_pcb.html](https://www.seeedstudio.com/fusion_pcb.html).
   
    *   Click on **Add Gerber Files** and upload [the main board Gerber files](../PCB/stimjimFabricationFiles_v0.181.zip). 
        Make sure to select 4 layers and enter correct dimensions (129x79.5 mm). 
        Select PCB quantity (minimum 5).
    *   Move now to the **PCB Assembly** section.
    *   Click on **Add Assembly Drawing & Pick and Place File** and upload the [zip file containing pdf assembly files and position files](../PCB/stimjim_SeeedStudioAssembly_PickAndPlace.zip).
    *   Select the requested PCB assembly quantity (tested with 2 for $780 USD in late 2019).
    *   Click on **Add BOM File** and upload the [Seeedstudio-formatted Bill of Materials](../PCB/stimjim_SeeedStudioAssembly_BOM.xlsx). The service will check parts availability. Seeedstudio may not be able to provide an instant quote, but they should send you one within a day.
    *   Seeedstudio-assembled Stimjims have been successfully tested by one user, but you have the option to have SeeedStudio test the assembly as well.
   
2. Order the [front and back panels for the enclosure](../PCB/stimjimPanelFabricationFiles_v0.18.zip) (1-layer, 2 designs, total size 140x71 mm) from [Seeedstudio](https://www.seeedstudio.com/fusion_pcb.html), [JLCPCB](https://jlcpcb.com/) or any other PCB company you like.
3. Order the enclosure itself (see the line "enclosure" in the [Bill of materials](../stimjim_BOM.xlsx)) from [Digikey](https://www.digikey.com/).

### Continuing from either option 1 or 2:

4.  Put the main board inside the enclosure (bottom slot), and screw the panels onto the front and back of the enclosure. 
5.  Connect the Stimjim to the computer over USB and flash the firmware — see
    [../README.md](../README.md) for the Arduino IDE and `arduino-cli` routes.
6.  On boot the firmware calibrates itself and prints the result over the serial connection:

        Booting StimJim on Teensy 3.5!
        Initializing inputs...
        ADC offsets (+-2.5V): -11.030000, -11.150000
        ADC offsets (+-10V): -10.220000, -10.340000
        current offsets: 11, 8
        voltage offsets: 0, 0
        Ready to go!

    The ADC offsets are the reading (in ADC units, 2.4 mV each) taken with the output grounded,
    for channels 0 and 1. The converter has a small but non-zero zero-offset error; Stimjim
    measures it on two range settings and corrects every later reading. Values under 20 are
    normal. **A value of ±4096 means the board has a fault.**

    The current offsets are the raw DAC codes (−32768…32767) that read closest to 0 µA. The
    firmware uses them as the parked level, so a train leaves no standing DC current between
    pulses.
