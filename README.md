# Shield Code 2.0
This is the 317 Lab repository for the Arduino Due version of the shield code. Things that are new:
* There is no longer a "coprocessor" - everything is run from the Arduino Due. The Due uses a 32-bit sam3x8e processor, far more powerful and advanced than the old Arduino Uno. Information about the board can be found [here](https://docs.arduino.cc/hardware/due/). The processor's datasheet can be found [here](https://ww1.microchip.com/downloads/en/devicedoc/atmel-11057-32-bit-cortex-m3-microcontroller-sam3x-sam3a_datasheet.pdf).
* This is now a Platformio project. This means we no longer have to use the arduous and painful Arduino IDE. Instructions for using and installing Platformio can be found [here](#platformio). This also means that the project is now entirely in C++.
* The project is now documented using [Doxygen](https://www.doxygen.nl/). Information on accessing documentation can be found [here](#documentation).
## Table of Contents
1. [Overview](#overview)
2. [Platformio](#platformio)  
    1. [Installing and Setting Up](#installing-and-setting-up)
    2. [Programming the Board](#programming-the-board)
    3. [Serial Monitor](#serial-monitor)
    4. [Debugging with GDB+Black Magic](#debugging-with-gdb--black-magic)
3. [Documentation](#documentation)

## Overview
The project structure is basically identical to the old version, albeit simpler, as deprecated and unnecessary code and files were removed. The core functionality can be found in [main.cpp](src/main.cpp). This (will) use the same ["state machine"](https://en.wikipedia.org/wiki/Finite-state_machine) as the old code. A detailed explanation of this can be found in [Ruth Nordhoff's thesis](https://drive.google.com/file/d/1XH5SRRh2m84pu7f2B1RSFuYGV1u3EBdw/view?usp=sharing). Note that the hardware section of that thesis is now outdated.  
  
All source (.cpp) files are housed in the [src](src) directory. The header (.hpp) files for custom libraries are housed in the [include](include) directory. All other project libraries are included through the `lib_deps` flag in [platformio.ini](platformio.ini), which is explained [below](#platformio) and will be automatically included when the project is built.

## Platformio
### Installing and Setting Up
1. Download VSCode. [Windows instructions](https://code.visualstudio.com/docs/setup/windows), [Mac instructions](https://code.visualstudio.com/docs/setup/mac). It is an extremely powerful, industry standard IDE. I recommend connecting it to your Dartmouth github account (so you can get access to github copilot) and installing the C++ extension.
2. Open VSCode and navigate to the extensions window - the building blocks icon on the left side.
3. Install PlatformIO IDE. It should take you to PIO Home - from there, click "Open Project" and select this repository. Instructions on cloning a github repository to your machine can be found [here](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository) - make sure you have Git installed!  
    1. Side note - make sure you create your own working branch, and commit all changes to that. I strongly recommend [this beginners tutorial](https://github.com/skills/introduction-to-github) if you haven't used Github before.

Once you have PIO installed, whenever you open the project in vscode it should automatically recognize it as  PIO project.

### Programming the board
(For current development, not on final board)  
  
Very simple. Connect your computer to the Due's programming port using micro-USB (it's the one next to the round jack, farther from the reset button). Then, either:
1. Click the small right arrow in the bottom left of the window  
OR
2. Type `pio run --target upload` in terminal.  

If you get a "no device found on [PORT]" error, just unplug from your machine and plug it back in. Platformio should correctly autodetect the correct port, but if you're having problems, double check in your device manager and set the correct port using `upload_port` in [platformio.ini](platformio.ini).
### Serial Monitor
I highly recommend using HTerm, not the built in Platformio serial monitor. It is far more flexible and user friendly. You can download it [here](https://www.der-hammer.info/pages/terminal.html)  

Remember that the data the state machine is sending is not ASCII readable. If you need to quickly output readable data for debugging, use Serial.print(). It does not conflict with our PDC library.  

If you do want to use the Platformio serial monitor, either click the plug icon in the top right or run `pio device monitor` in terminal.

### Debugging with GDB + Black Magic
Our lab now has a JTAG probe that allows you to debug the Arduino code line-by-line using breakpoints. It is absolutely life changing. I will write a more complete guide on using it at some point, but for now I recommend:
1. The Art of Debugging with GDB, DDD, and Eclipse by Norman Matloff and Peter Jay Salzman for a guide to GDB.
2. This [guide to the Black Magic probe](https://github.com/compuphase/Black-Magic-Probe-Book/tree/main).
3. This [guide to the PIO debugger](https://docs.platformio.org/en/latest/plus/debugging.html).
4. This [guide to connecting the probe](https://www.hackster.io/rpatterson/hardware-debugging-arduino-due-with-gdb-and-blackmagic-probe-bfbe10).

## Documentation
The project is now documented with Doxygen. This means that in every header file, there is a well organized and commented explanation of everything in that library/class. It also generates a nice webpage of documentation - [see here](https://eigen.tuxfamily.org/dox/).  

Because it is bad practice to include generated files in a github repository, you need to generate the documentation locally. After you do it once, it's extremely easy. If this is too much, I will try to maintain a version you can just download from the Google Drive and skip to step 3 - but no promises!  

**Generating Local Documentation:**
1. Install Doxygen. On Windows, download the installer [here](https://www.doxygen.nl/download.html). On Mac, use homebrew with `brew install doxygen`. If you don't have Homebrew, you need it for life in general - install and read about it [here](https://brew.sh/). 
2. I recommend using Doxywizard - I have had a lot of trouble with this project running doxygen from command line.  
    1. Run `doxywizard` in terminal. You can/should do this directly in the platformio terminal.
    2. Choose the project directory. Also choose this directory for the soruce code option.
    3. Select "scan recursively."
    4. Make a folder for the documentation files and select it as the destination directory. Make sure that this folder is OUTSIDE of the project repository.
    5. Click next, select "All Entities" and "optimize for C++ output"
    6. Click next again. You don't need to change anything on the output page.
    7. Select "use built-in class diagram generator." Click next.
    8. Click "run doxygen."
3. Now, go to the folder you made, open html, and double click on "index.html" - and it will open the documentation! 

After generating the documentation, you can go directly to the last step in the future. 

If you don't want to use doxywizard, just run `doxygen Doxyfile` in terminal - but be warned that it probably won't find all of the files. It probably works better on Unix, I've only tried it on Windows.

