# haptic-wheel
Haptic feedback wheel implementing a volume knob. Inspired by Scott Bezek's Smart Knob and University of Michigan EECS 461 Haptics Lab

This volume knob is implemented with a feedback controlled motor and software rather than mechanical notches. When you turn it with your hand you feel real feedback force just as if it were an actual precisely made mechanical knob. Haptic Volume Knob works by monitoring the motor shaft's angle and applying the appropriate torque to implement the knob's feeling for that position. An ESP32 S3 monitors this position and commands a torque 7,000 times per second to give real time feedback that is convincing to a user as well as implements a stable control loop.

Two haptic effects are implemented here: A virtual knob with detents and a virtual wall which creates the sensation of the knob reaching end of travel. These sensations can be customized to feel "harder" or "softer" depending on spring constants. There is a button on the back of the enclosure to iterate through different knob detent feelings ranging from very light to very stiff. This variable haptic effect cannot be acheived with a traditional knob. 

TODO: write about HID

TODO: write about parts used

TODO: write about C# service

![View 1](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9906.jpg)
![View 2](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9907.jpg)
![View 3](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9908.jpg)

# Install Service
In powershell run the following line replacing paths with your own
`New-Service -Name "HIDVolumeSync" -BinaryPathName "C:\Users\joshk\source\repos\haptic-wheel\HID Volume Sync Service\bin\Debug\HID Volume Sync Service.exe"`
 
Make sure the service is set to startup type automatic in the Windows Services UI to start on boot up.
