# ESP32_MPU9250_HP206C_SDCARD_tester
ESP32-arduino/PlatformIO tester program, test hardware initialisation and test for 9DOF MPU9250, Barometer HP206C, GPS NEO 7M, and SDcard reader for arduino

This is more a lab than anything of a finished program, but allows plugging the mentioned devices and check that they are functioning, test their connectivity.
A useful code in any system requiring diagnostic i.e. for a datalogger or a wearable.

# lib ependencies
Be sure to check the platformio.ini file for dependencies
Libs used:
- Sparkfun MPU9250 DMP lib minor changes, embedded in this repo
- HP20x adapted for ESP32 see my fork git  https://github.com/romatou18/Grove_Barometer_HP20x
copy it in the lib directory.
- TinyGPS find it on platformIO
