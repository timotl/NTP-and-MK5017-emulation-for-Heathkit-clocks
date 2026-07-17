# NTP-and-MK5017-emulation-for-Heathkit-clocks
Documentation and modification of original clock to use ESP32 module instead of MK5017. 
Uses original chip socket and only requires 5v power module. 
No additional changes to the original hardware. 
Includes schematic and PCB for interface board. 
Full code and description of function. 
Documentation of the MK5017 

ESP32 sketch to replace MK5017AA and BA used in HeathKit panaplex clocks
Requires shift register and level shifting interface which connects to the original board via chip socket
No modifications to the original clock, except adding a 5v power supply module at the 120v input line
Heathkit GC-1092D, Heathkit GC-1005, Heathkit GC-1092A, Heathkit GC-1094 all fully tested
Display updates are state driven and writing to display is handled by hardware timer

WIFI mode:
Portal will be active if the configured wifi settings do not connect or if the time cannot be set from the configured NTP server after 1 minute.
NTP server, timezone, display off times are set through the portal.

ESP Touch (also snooze or alarmoff switches) changes display from Time to Date, Date to display off, display off to on.
GC-1092A/D - Original 555 touch hardware can be used, however each touch event must be at least 5 seconds apart.

MK5017 mode:
Switches and all behavior faithful to original hardware
RTC simulates battery backup of the GC-1092A/D (but works on all models) - Will consider time valid for configurable number of hours.
Switching modes will reset the model. Set model by pressing Hour/Month switch until the correct model is displayed, then toggle Time Set.

Switching modes:
Holding Hour/Month switch at power up will change between MK5017 and Wifi mode. Display will go from 888888 to ------ and restart

ArduinoOTA mode:
Alarm/Date switch also enters OTA mode. OTA mode uses AP mode with password and times out after 10 min.

Troubleshooting - Toggle Time Set switch 3x in under 5 seconds.
WIFI mode: Restart to portal to adjust WIFI and NTP settings.
MK5017 mode: Restart clearing RTC and model

Select ESP32-WROOM-DA Module in Arduino IDE
Partition scheme "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS) required if SerialBT is enabled

Also includes Manuals and other datasheets for the various models I can find.
