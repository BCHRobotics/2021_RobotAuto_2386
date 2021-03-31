# FRC Robotics Team 2386 - 2021 Code

This is the main working code for the 2021 robotics season, this is created for the FRC Recharge At Home Challenges.

## Getting Started

### Installation

To run the code you must install the following items to your computer:

* FRC tools (Windows only, for driving): (https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)
* VSCode: (https://code.visualstudio.com/)
* WPILib Extension: (https://marketplace.visualstudio.com/items?itemName=wpilibsuite.vscode-wpilib)
* Java Tools: (https://code.visualstudio.com/docs/languages/java)


### Vendor Libraries

These are the vendor libraries that we are using. These are what you need to copy if they are missing.

```
https://www.kauailabs.com/dist/frc/2020/navx_frc.json
http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json
https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
```

### Running the code

1. Clone or download this repo to your computer
2. Connect to the robot with one of the following
    * USB A to B
    * Wireless
    * Wired Ethernet
3. Right Click on **build.gradle** and click **Deploy Robot Code**
4. Launch Driver Station and Shuffleboard
5. Drive the Robot!

### Controls

We use 2 XBox controllers to control the robot, below is the button maps for the controllers.

Driver

Button | Function
-------|---------
Left Bumper | Snail (While held down robot will drive at 50% speed)
Right Bumper | Turbo (While held down robot will drive at 100% speed)

Joystick | Function
---------|---------
Left Y | Drive Forward/Backward (70% speed unless override by snail and turbo)

---
Operator

Button | Function
-------|---------
Left Bumper | Intake Arm Up (Hold down and will go up until hits encoder limit)
Right Bumper | Intake Arm Down (Hold down and will go down until hits encoder limit)
Y | Load Intake Without Intake (Not used much)
X | Delayed Unload (Hold down to unload the Stager into the shooter, this will turn on each motor with a delay to stop clogs)
B | Limelight Turn Turret to Target (Hold down and will find target)

Trigger | Function
--------|---------
Left Trigger | Intake Bar Out (Turns the intake backwards, used for clearing balls)
Right Trigger | Intake Bar In (Turns the intake inwards to pick up balls from the ground, also calls stager and stages balls)

Joystick | Function
---------|---------
Left Y Down | Manual adjustment of shooter flywheel speed
Left X | Turn turret side to side (will slow down at right and left encoder limits)

DPad | Function
-----|---------
Up | Set shooter velocity for Accuracy Challenge Green Zone (Press to set, don't hold)
Right | Set shooter velocity for Accuracy Challenge Yellow Zone (Press to set, don't hold)
Down | Set shooter velocity for Accuracy Challenge Blue Zone (Press to set, don't hold)
Left | Set shooter velocity for Accuracy Challenge Red Zone (Press to set, don't hold)

Note: To reset from DPad Accuracy Challenge Velocity, just override with Left Y Down manual control

## TODO

* Create drive auto command for turning on the spot
* Tune drive turn PID
* Program and Test all autos for Recharge at Home Challenges

## License

FIRST BSD License

Copyright (c) 2009-2021 FIRST and other WPILib contributors 
Copyright (c) 2020-2021 Team 2386
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   Neither the name of FIRST, WPILib, nor the names of other WPILib contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.