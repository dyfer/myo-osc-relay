# myo-osc-relay

Relays data from Thalmic Labs Myo armband to Supercollider (or any other softwarE) via OSC. 

Compiles on OS X with Xcode. 

Versions and dependencies are described below. 

Developed for UW Center for Digital Arts and Experimental Media (DXARTS), July 2016

Robert Twomey - roberttwomey.com

# parameters

Parameters are set by command line options: 
```
 -v 	verbose text output
 -a 	ADDR OSC send address
 -p 	PORT OSC send port
 -emg 	send EMG data
 -pose 	send pose data
 -quat 	send orientation quaternion
 -accel send acceleration
 -gyro	send gyroscope
 -linaccel	send linear acceleration
```

 example: 

```./myo-osc-relay -v -emg -pose -quat -accel -gyro -linaccel```

# os x / xcode versions

xcode Version 7.3 (7D175)

os x 10.11.5

# myo versions

myo connect Version 1.0.1

firmware 1.5.1970

# external library 

oscpack http://www.rossbencina.com/code/oscpack

