# ESP32Focuser
Focuser for astronomy telescope based on ESP32 controller. Comportable with Indilib (http://indilib.org/) and ASCOM (https://ascom-standards.org/).
The controller uses the protocol Moonlite (as documented in the indilib project).

This is a fork from https://github.com/simonachmueller/ESP32Focuser which was a port of an awesome code from https://github.com/Hansastro/Focuser to ESP32 platform.

My modifications:
1. It uses a D1 mini ESP32.
1. It uses AccelStepper library for an ULN2003 stepper motor driver for 28BYJ-48.
1. It uses SPIFFS and ArduinoJson libraries for reading and writing the last saved position.

# Installation for computer-controlled use
## Windows
1. Install CP210x driver from https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers if not installed automatically by windows itself
2. From here https://focuser.com/downloads.php install MoonLite Single Focuser if you wan to use the focuser in the manual mode or test it, or Universal ASCOM driver if you want to access the focuser from other astronomical tools like SharpCap
