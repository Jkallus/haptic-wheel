# haptic-wheel
Haptic feedback wheel implementing a volume knob. Inspired by Scott Bezek's Smart Knob and University of Michigan EECS 461 Haptics Lab

![View 1](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9906.jpg)
![View 2](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9907.jpg)
![View 3](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9908.jpg)

# Install Service
In powershell run the following line replacing paths with your own
`New-Service -Name "HIDVolumeSync" -BinaryPathName "C:\Users\joshk\source\repos\haptic-wheel\HID Volume Sync Service\bin\Debug\HID Volume Sync Service.exe"`
 
Make sure the service is set to startup type automatic in the Windows Services UI to start on boot up.
