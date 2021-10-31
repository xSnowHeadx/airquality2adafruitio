/*
 VINDRIKTNING Ikea air quality sensor to AdafruitIO
 Serial code based on https://github.com/Hypfer/esp8266-vindriktning-particle-sensor
 */
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include "ESP8266WiFi.h"
#include "FS.h"
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include "config.h"
#include "SerialCom.h"
#include "Types.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include <Adafruit_BMP280.h>
#include <WiFiManager.h>

#define _VAR_CONFIG_ 0

// set up the 'PM25' feed
AdafruitIO_Feed *a_pm25 = io.feed("PM25");
AdafruitIO_Feed *a_temperature = io.feed("Temperature");
AdafruitIO_Feed *a_humidity = io.feed("Humidity");
AdafruitIO_Feed *a_pressure = io.feed("Pressure");
AdafruitIO_Feed *a_co2 = io.feed("CO2");

//flag for saving data
bool shouldSaveConfig = false;

uint8_t serialRxBuf[255];
uint8_t rxBufIdx = 0;

SensorState_t state =
{ 0 };

SCD30 airSensor;

#define I2C_ADDRESS 0x76
//create a BMx280I2C object using the I2C interface with I2C Address 0x76
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
bool bmp_init = false;

uint32_t targetTime = 0, storeTime = 0;
char tstr[64], vstr[12];
uint16_t co2;
uint16_t pressure = 973;
float temperature;
float humidity;

#if _VAR_CONFIG_
const char AP_NAME[] = "AirSens";
char WIFI_SSID[128] = "";
char WIFI_PASS[128] = "";
char AIO_USERNAME[33] = "";
char AIO_KEY[33] = "";
#else
// enter your own account and wifi data here, wifimanager is not working at the moment
char WIFI_SSID[128] = "";
char WIFI_PASS[128] = "";
char AIO_USERNAME[33] = "";
char AIO_KEY[33] = "aio_";
#endif

//callback notifying us of the need to save config
void saveConfigCallback()
{
	Serial.println("Should save config");
	shouldSaveConfig = true;
}

void setup()
{
	//-- WiFiManager --
	//Local intialization. Once its business is done, there is no need to keep it around
	int connectionStatus = WL_IDLE_STATUS;
#if _VAR_CONFIG_
	WiFiManager wifiManager;
	//set config save notify callback
	wifiManager.setSaveConfigCallback(saveConfigCallback);
#endif

	// start the serial connection
	Serial.begin(115200);
	// wait for serial monitor to open
	while (!Serial)
		;

	// Software Serial for Sensor
	SerialCom::setup();

#if _VAR_CONFIG_
	//clean FS, for testing
	//SPIFFS.format();
	//read configuration from FS json
	Serial.println("mounting FS...");

	if (SPIFFS.begin())
	{
		Serial.println("mounted file system");
		if (SPIFFS.exists("/config.json"))
		{
			//file exists, reading and loading
			Serial.println("reading config file");
			File configFile = SPIFFS.open("/config.json", "r");
			if (configFile)
			{
				Serial.println("opened config file");
				size_t size = configFile.size();
				// Allocate a buffer to store contents of the file.
				std::unique_ptr<char[]> buf(new char[size]);

				configFile.readBytes(buf.get(), size);

#ifdef ARDUINOJSON_VERSION_MAJOR >= 6
				DynamicJsonDocument json(1024);
				auto deserializeError = deserializeJson(json, buf.get());
				serializeJson(json, Serial);
				if (!deserializeError)
				{
#else
	        DynamicJsonBuffer jsonBuffer;
	        JsonObject& json = jsonBuffer.parseObject(buf.get());
	        json.printTo(Serial);
	        if (json.success()) {
	#endif
					Serial.println("\nparsed json");
					strcpy(AIO_USERNAME, json["aio_username"]);
					strcpy(AIO_KEY, json["aio_key"]);
				}
				else
				{
					Serial.println("failed to load json config");
				}
				configFile.close();
			}
		}
	}
	else
	{
		Serial.println("failed to mount FS");
	}
	//end read

	// The extra parameters to be configured (can be either global or just in the setup)
	// After connecting, parameter.getValue() will get you the configured value
	// id/name placeholder/prompt default length
	WiFiManagerParameter aio_username("user", "AIO Username", (const char*) &AIO_USERNAME, 33);
	WiFiManagerParameter aio_key("key", "AIO API-Key", (const char*) &AIO_KEY, 33);
	wifiManager.setConnectTimeout(30);
	//add all your parameters here
	wifiManager.addParameter(&aio_username);
	wifiManager.addParameter(&aio_key);

	if (!strlen(AIO_USERNAME) || !strlen(AIO_KEY))
	{
		if (!wifiManager.startConfigPortal(AP_NAME))
		{
			Serial.println("failed to connect and hit timeout");
		}
		else
		{
			strcpy((char*) &WIFI_SSID, wifiManager.getWiFiSSID().c_str());
			strcpy((char*) &WIFI_PASS, wifiManager.getWiFiPass().c_str());
		}

	}
	if (!shouldSaveConfig && (strlen(WIFI_SSID) > 0))
	{
		Serial.println("Using stored credentials:");
		Serial.println(WIFI_SSID);
		Serial.println(WIFI_PASS);

		connectionStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
		Serial.print("WiFi.begin returned ");
		Serial.println(connectionStatus);
	}
	else
	{

		//fetches ssid and pass from eeprom and tries to connect
		//if it does not connect it starts an access point with the specified name wifiManagerAPName
		//and goes into a blocking loop awaiting configuration
		connectionStatus = wifiManager.autoConnect(); //wifiManagerAPName, wifiManagerAPPassword);
		Serial.print("autoConnect returned ");
		Serial.println(connectionStatus);
	}

	if (connectionStatus == 1)
	{
		//read updated parameters
		strcpy((char*) &AIO_USERNAME, aio_username.getValue());
		strcpy((char*) &AIO_KEY, aio_key.getValue());
		Serial.println("The AIO-values in the file are: ");
		Serial.println("\tAIO Username : " + String((char*) &AIO_USERNAME));
		Serial.println("\tAIO API-Key : " + String((char*) &AIO_KEY));

		//save the custom parameters to FS
		if (shouldSaveConfig)
		{
			Serial.println("saving config");
#ifdef ARDUINOJSON_VERSION_MAJOR >= 6
			DynamicJsonDocument json(1024);
#else
		    DynamicJsonBuffer jsonBuffer;
		    JsonObject& json = jsonBuffer.createObject();
		#endif
			json["aio_username"] = (char*) &AIO_USERNAME;
			json["aio_key"] = (char*) &AIO_KEY;

			File configFile = SPIFFS.open("/config.json", "w");
			if (!configFile)
			{
				Serial.println("failed to open config file for writing");
			}

#ifdef ARDUINOJSON_VERSION_MAJOR >= 6
			serializeJson(json, Serial);
			Serial.println();
			serializeJson(json, configFile);
#else
		    json.printTo(Serial);
		    json.printTo(configFile);
#endif
			configFile.close();
			//end save
		}
//		strcpy((char*) &WIFI_SSID, wifiManager.getWiFiSSID().c_str());
//		strcpy((char*) &WIFI_PASS, wifiManager.getWiFiPass().c_str());

#else
	{
		connectionStatus = (WiFi.begin(WIFI_SSID, WIFI_PASS) == 7) ? 1 : 0;
		Serial.print("WiFi.begin returned ");
		Serial.println(connectionStatus);

#endif
		if (strlen(AIO_USERNAME) && strlen(AIO_KEY))
		{
			char tries = 20;

			Serial.println("Connecting to Adafruit IO");

			// connect to io.adafruit.com
			io.connect();

			// wait for a connection
			while (--tries && (io.status() < AIO_CONNECTED))
			{
				Serial.print(". ");
				Serial.print(io.status() + " ");
				Serial.println(io.statusText());
				delay(500);
			}

			// we are connected
			Serial.println();
			Serial.println(io.statusText());
		}
	}
	Wire.begin();
	airSensor.begin();
	airSensor.setMeasurementInterval(5);
	airSensor.setAmbientPressure(pressure);

#if USE_BMP
	if (!bmp.begin(I2C_ADDRESS))
	{
		Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
	}
	else
	{
		bmp_init = true;
		/* Default settings from datasheet. */
		bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
		Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
		Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
		Adafruit_BMP280::FILTER_X16, /* Filtering. */
		Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
		Serial.print("BMP280 found. Pressure = ");
		Serial.print(bmp.readPressure() / 100.0);
		Serial.println(" hPa");
	}
#endif
	storeTime = millis() + 60000UL;
}

void loop()
{
	bool got_temp = false;

	if (io.status() == AIO_CONNECTED)
	{
		io.run();
	}

	if (targetTime < millis())
	{
		targetTime = millis() + 5000;
#if USE_BMP
		if (bmp_init)
		{
			pressure = bmp.readPressure() / 100.0;
			airSensor.setAmbientPressure(pressure);
			state.pressure.avg += pressure;
			state.pressure.mcount++;
			temperature = bmp.readTemperature() - 7.0;
			got_temp = true;
		}
#endif
		if (airSensor.dataAvailable())
		{
			co2 = airSensor.getCO2();
			state.co2.avg += co2;
			state.co2.mcount++;
			if (!got_temp)
				temperature = airSensor.getTemperature();
			state.temperature.avg += temperature;
			state.temperature.mcount++;
			humidity = airSensor.getHumidity();
			state.humidity.avg += humidity;
			state.humidity.mcount++;

			Serial.print("co2(ppm):");
			Serial.print(co2);
			Serial.print(" temp(C):");
			Serial.print(temperature, 1);

#if USE_BMP
			Serial.print(" humidity(%):");
			Serial.print(humidity, 1);
			Serial.print(" pressure(hPa):");
			Serial.print(pressure);
		}
#endif
		Serial.println();
	}
//	Serial.println("Handle UART");
	SerialCom::handleUart(&state);
	if (storeTime < millis())
	{
		storeTime = millis() + 60000UL;
		if (io.status() == AIO_CONNECTED)
		{
			if (state.temperature.mcount)
			{
				a_temperature->save(state.temperature.avg / state.temperature.mcount);
			}
			if (state.humidity.mcount)
			{
				a_humidity->save(state.humidity.avg / state.humidity.mcount);
			}
			if (state.co2.mcount)
			{
				a_co2->save((uint16_t) (state.co2.avg / state.co2.mcount));
			}
			if (state.pm25.mcount)
			{
				a_pm25->save((uint16_t) (state.pm25.avg / state.pm25.mcount));
			}
#if USE_BMP
			if (state.pressure.mcount)
			{
				a_pressure->save((uint16_t) (state.pressure.avg / state.pressure.mcount));
			}
#endif
			Serial.println("Saved values to io.adafruit.com");
			memset((void*) &state, 0, sizeof(state));
		}
	}
}

