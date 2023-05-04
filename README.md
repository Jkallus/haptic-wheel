# Haptic Volume Knob
Haptic feedback wheel implementing a volume knob. Inspired by Scott Bezek's Smart Knob and University of Michigan EECS 461 Haptics Lab

This volume knob is implemented with a feedback controlled motor and software rather than mechanical notches. When you turn it with your hand you feel real feedback force just as if it were an actual precisely made mechanical knob. Haptic Volume Knob works by monitoring the motor shaft's angle and applying the appropriate torque to implement the knob's feeling for that position. An ESP32 S3 monitors this position and commands a torque 7,000 times per second to give real time feedback that is convincing to a user as well as implements a stable control loop.

Two haptic effects are implemented here: A virtual knob with detents and a virtual wall which creates the sensation of the knob reaching end of travel. These sensations can be customized to feel "harder" or "softer" depending on spring constants. There is a button on the back of the enclosure to iterate through different knob detent feelings ranging from very light to very stiff. This variable haptic effect cannot be acheived with a traditional knob. 

The ESP32 communicates with the PC using HID. The TinyUSB HID library allows the ESP32 to describe itself as a custom HID device. It sends volume updates to Windows in the same way as the volume up and down keys on any keyboard work. The ESP32 gets the initial volume and any updates to the volume level made elsewhere by a service running on Windows which monitors volume changes and sends updates as HID reports. 

# Bill of Materials
| Item                            | Source                                                                                                                                                                                                                                           |   |   |   |
|---------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|---|---|
| ESP32 S3                        | https://www.amazon.com/Teyleten-Robot-ESP32-S3-DevKitC-1-N8R2-Development-Integrates/dp/B0B6HT7V7P/ref=sr_1_3?crid=10W15OHSL1KK4&keywords=esp32+s3&qid=1683168772&sprefix=esp32+s3%2Caps%2C89&sr=8-3                                             |   |   |   |
| Busboard ST2                    | https://www.amazon.com/Busboard-ST2-Traditional-Stripboard-Prototyping/dp/B00LLQFRAU/ref=sr_1_3?crid=2GTU52YF41W9I&keywords=busboard+prototype+systems+stripboard&qid=1683168802&sprefix=busboard+prototype+systems+stripboard%2Caps%2C78&sr=8-3 |   |   |   |
| TMC6300 Brushless Motor Driver  | https://www.sparkfun.com/products/retired/21220                                                                                                                                                                                                  |   |   |   |
| Three Phase Brushless Motor     | https://www.sparkfun.com/products/20441                                                                                                                                                                                                          |   |   |   |
| AS5048A Magnetic Rotary Encoder | https://www.digikey.com/en/products/detail/ams-osram/AS5048A-TS-EK-AB/3188612                                                                                                                                                                    |   |   |   |
| Neopixel 24 LED Ring            | https://www.adafruit.com/product/1586                                                                                                                                                                                                            |   |   |   |
| BH1750 Light Sensor             | https://www.amazon.com/ACEIRMC-BH1750FVI-Intensity-Illumination-arduino/dp/B08ZS4PJSW/ref=sr_1_3?crid=OM3RPFPPF5MP&keywords=bh1750&qid=1683169028&sprefix=bh1750+%2Caps%2C82&sr=8-3                                                              |   |   |   |
| LM2596 Buck Converter           | https://www.amazon.com/Converter-Adjustable-Regulator-Efficiency-Transformer/dp/B08L5VRBZS/ref=sr_1_3?crid=15WDAI3AV3JW1&keywords=lm2596&qid=1683169056&sprefix=lm2596%2Caps%2C85&sr=8-3                                                         |   |   |   |
| Panel Mount Button              | https://www.amazon.com/dp/B0752RMB7Q?psc=1&ref=ppx_yo2ov_dt_b_product_details                                                                                                                                                                    |   |   |   |
| DC Socket                       | https://www.amazon.com/dp/B082GHXS1Q?psc=1&ref=ppx_yo2ov_dt_b_product_details                                                                                                                                                                    |   |   |   |
| Micro USB Female Breakout       | https://www.amazon.com/gp/product/B07T91S7L2/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1                                                                                                                                                     |   |   |   |
| PH 2.0 Crimp                    | https://www.amazon.com/gp/product/B07VHCFGZS/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1                                                                                                                                                     |   |   |   |
|                                 |                                                                                                                                                                                                                                                  |   |   |   |
|                                 |                                                                                                                                                                                                                                                  |   |   |   |



![View 1](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9906.jpg)
![View 2](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9907.jpg)
![View 3](https://github.com/Jkallus/haptic-wheel/blob/main/images/IMG_9908.jpg)

# Install Service
In powershell run the following line replacing paths with your own
`New-Service -Name "HIDVolumeSync" -BinaryPathName "C:\Users\joshk\source\repos\haptic-wheel\HID Volume Sync Service\bin\Debug\HID Volume Sync Service.exe"`
 
Make sure the service is set to startup type automatic in the Windows Services UI to start on boot up.
