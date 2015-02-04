#Embedded Control System Project 2015

The Makefile is created by Sudar, check on GitHub for details:
https://github.com/sudar/Arduino-Makefile.git

To compile the project:

1. Install arduino
sudo apt-get install arduino arduino-core arduino-mk

2. Install serial port module for perl (needed by Arduino-Makefile)
sudo apt-get install libdevice-serialport-perl

3. Copy the third party repositories from repos to your sketchbook
cp -r repos/libraries/* ~/sketchbook/libraries/

4. Build the project:
make

5. Upload to Arduino
make upload

The default board is uno, you can change it in the Makefile.
