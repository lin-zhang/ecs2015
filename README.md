#Embedded Control System Project 2015

The Arduino Makefile is from Sudar's project, check on GitHub for details:

[Arduino-Makefile](https://github.com/sudar/Arduino-Makefile.git)

The (adopted) third party libraries used in the project:

[1] [MsTimer2](http://playground.arduino.cc/Main/MsTimer2)

[2]  [PID_v1](http://playground.arduino.cc/Code/PIDLibrary)

[3] [LiquidCrystal_I2C](https://github.com/marcmerlin/NewLiquidCrystal.git)

[4] [Motor Shield](http://playground.arduino.cc/Main/AdafruitMotorShield)

([1]-[3] are in repo directory; some functions in hw_functions.cpp are adopted from [4].)

###To compile the project:

####Install arduino
```
	sudo apt-get install arduino arduino-core arduino-mk
```

####Install serial port module for perl (needed by Arduino-Makefile)
```
	sudo apt-get install libdevice-serialport-perl
```
####Copy the third party repositories from repos to your sketchbook
```
	cp -r repos/libraries/* ~/sketchbook/libraries/
```
####Build the project
```
	make
```
####Upload to Arduino
```
	make upload
```
The default board is uno, you can change it in the Makefile.
