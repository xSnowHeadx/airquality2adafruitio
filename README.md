# airquality2adafruitio

Based on projects of [Hypfer](https://github.com/Hypfer/esp8266-vindriktning-particle-sensor) and
[callimero](https://github.com/callimero/vindriktning2adafruitio)

Inspiration from [heise](https://www.heise.de/ratgeber/Ikea-Feinstaubsensor-Vindriktning-zum-IoT-Device-aufbohren-6164149.html)

This project is heavily under construction! 

Based on the projects above I want to have a sensor for all relevant ambient air quality parameters fitting in this nice case from Ikea.
So I added a SCD30-sensor for CO2-concentration and a BMP250-Sensor for humidity and air-pressure (needed for SCD30).
At the moment the configuration of the wifi- and adafruit-io parameters with the wifimanager does not work so let the define "_VAR_CONFIG_" to 0 and enter your fix data in the source.
On io.adafruit.com you need to create the feeds "co2", "humidity", "temperature", "pressure" and "pm25" to store the data. 
An example of the recorded data you can find [here](https://io.adafruit.com/SnowHead/dashboards/vindriktning-plus)