# ESP32 Pico Kit D4 CO2 Ventilation

This project is an ESP32-based indoor air quality and ventilation indicator built around a Sensirion SCD30 CO2 sensor. The firmware reads CO2, temperature, and humidity, shows the live values on an OLED display, and drives a Shelly RGBW2 light so the room status can be understood at a glance. 

In version 9, the device also exposes a built-in web interface for sensor status, LED diagnostics, system configuration, and SCD30 calibration.

The main goal of the project is to make poor air quality visible early enough that a room can be ventilated before CO2 levels become uncomfortable or unhealthy. Instead of relying only on raw numbers, the firmware converts the measured CO2 concentration into an RGB color state. 
Low CO2 stays green, rising CO2 moves through yellow and orange, and high CO2 becomes red. 
This makes the device useful in offices, meeting rooms, classrooms, bedrooms, or any enclosed space where fresh air management matters.

## Firmware Purpose

The v9 firmware in [co2-Ventilation-ESP-v9/co2-Ventilation-ESP-v9.ino](co2-Ventilation-ESP-v9/co2-Ventilation-ESP-v9.ino) is designed to do four jobs:

1. Measure indoor air quality with the SCD30 sensor.
2. Present the data locally on the OLED and over a browser-based status interface.
3. Control an external Shelly RGBW2 Gen1 light over HTTP so CO2 levels are visible from a distance.
4. Store configuration in ESP32 non-volatile storage so settings survive reboots and can be changed without recompiling.

## How It Works

At boot, the firmware loads default values from the secrets header and then applies any saved overrides from ESP32 Preferences (NVS). It starts the SCD30 before Wi-Fi, configures altitude compensation, and optionally initializes a BME280 for additional environmental data. 

Once the network is up, the device starts mDNS support, configures NTP time sync, resolves the Shelly host, and launches an asynchronous web server.

During runtime, the sketch reads CO2 values on a fixed interval, filters the measurements with an exponential moving average, and optionally applies hysteresis to avoid rapid LED color flicker. The processed CO2 value is then mapped to RGB output and sent to the Shelly controller. In parallel, the web UI provides separate pages for sensor values, LED/Shelly diagnostics, and protected system configuration.

Version 9 also adds SCD30 calibration management. The web interface can:

- enable or disable ASC (Auto Self-Calibration)
- store a pending FRC (Forced Recalibration) target in ppm
- trigger an immediate 400 ppm calibration when the sensor is in fresh outdoor air
- record the last calibration time and reason
- show whether calibration settings come from defaults or persisted NVS values

Administrative routes are protected with HTTP Basic Auth and can optionally be restricted to the local LAN only.

## arduino_secrets - Template Variables

The file [co2-Ventilation-ESP-v9/arduino_secrets.h.example](co2-Ventilation-ESP-v9/arduino_secrets.h.example) contains the compile-time default configuration. These values are used on first boot and whenever no saved override exists in NVS.

`SECRET_SSID`
Default Wi-Fi network name used by the ESP32 to connect to the local network.

`SECRET_PASS`
Default Wi-Fi password for the configured SSID.

`SECRET_LED_IP`
Address of the Shelly RGBW2 controller. This can be a fixed IP address, a DNS hostname, or an mDNS name such as `device.local`. The firmware sanitizes the value and uses it to send HTTP color commands to the light.

`SECRET_HOSTNAME`
Hostname assigned to the ESP32 on the network. This is also used for mDNS, so the device can typically be reached as `http://<hostname>.local/`.

`SECRET_BAROMETERAltitude_m`
Default installation altitude in meters. This is used as the SCD30 altitude compensation value and also acts as the fallback altitude configuration if no saved value exists.

`SECRET_REPORT`
Controls what the device prints to the serial monitor.

- `A` = all values
- `T` = temperature only
- `H` = humidity only
- `C` = CO2 only

`SECRET_REPORT_TXT`
Controls whether serial output includes human-readable labels and units.

- `Y` = include labels such as `CO2:` and units such as `ppm`
- `N` = print compact raw values only

`DEBUG`
Default verbose logging flag for serial diagnostics. When enabled, the firmware prints additional information about sensor setup, Wi-Fi, DNS resolution, NTP sync, Shelly requests, and calibration actions.

`LAN_only`
Default access policy for the protected web pages. When enabled, configuration and maintenance routes are limited to clients on the same local subnet. When disabled, protection falls back to HTTP Basic Auth only.

## Notes About Configuration
The template file provides only the initial defaults. After the first boot, most settings can be changed either from the serial startup menu or from the protected web configuration page. Those updated values are stored in ESP32 Preferences (NVS) and will override the template values until a factory reset clears them.

For public repositories, use the template file as the example configuration and avoid committing personal Wi-Fi credentials or device-specific secrets.

Copy and edit the Arduino_secrets.h.example file. Once the correct Wifi details has been entered save it as Arduino_secrets.h


# PCB
The Arduino ESP32 Pico Kit D4 board on this Git Account has a board that can easily be used to connect a sensor and OLED display.

A Shelly RGBW can drive a colour LED strip for effect.