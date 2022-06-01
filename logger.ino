/*
    {"feeds":[{"key": "temperature", "value": body.temp},
    {"key": "pressure", "value": body.pressure},
    {"key": "voltage", "value": body.voltage},
    {"key": "card", "value": body.card}], "location":
    {"lat": where_lat, "lon": where_lon,  "ele": 0}, "created_at": $fromMillis(when * 1000)}
*/


/*
   Sleepy Sensor Application

   The goal of this application is to show the elements required to construct an
   application which leverages the capabilities of the Notecard to minimize
   battery consumption by controlling the wake time of the host MCU.

   Use the following diagram to wire up the circuit. Note the Notecard's `ATTN`
   pin (exposed via the Notecarrier) is wired to the enable pin of the host MCU.
    ________________________________________
   |  ____________________________________  |               ____________________
   | |                                    | |              /
   | |        _________________           | |             |  O
   | |       |O|             |O|          | |             |     (Notecarrier-AL)
   | --> SDA-|*  ___________  *|-21       | |       VUSB>-|[]
   ----> SCL-|* | ESPRESSIF | *|-TX       | |  ----> BAT>-|[]
          14-|* |ESP32-WROOM| *|-RX       | |  |    MAIN>-|[]
          32-|* |CE         | *|-MI       | |  |     VIO>-|[]
          15-|* |           | *|-MO       | |  |          |[]
          33-|* |       ___ | *|-SCK      | |  |       V+-|[]
          27-|* |      |   || *|-A5    ---^-^--^----> GND-|[]
          12-|* |______|___|| *|-A4    |  | |  |       EN-|[]
          13-|*               *|-A3    |  | |  |      RST-|[]
         USB-|*               *|-A2    |  | |  |          |[]
   -----> EN-|*         ----- *|-A1    |  | ---^----> SCL-|[]
   | --> BAT-|*___      |   | *|-A0    |  -----^----> SDA-|[]
   | |       |    |     ----- *|-GND <--       | --> ATTN-|[]
   | |       |    |        _  *|-NC            | |  AUXEN-|[]
   | |       |-----       |O| *|-3V            | |  AUXRX-|[]
   | |       |      -----     *|-RST           | |  AUXTX-|[]
   | |       |O___0_|___|_0___O|               | |   AUX1-|[]
   | |                                         | |   AUX2-|[]
   | |_________________________________________| |   AUX3-|[]
   |_____________________________________________|   AUX4-|[]
                                                       RX-|[]
                                                       TX-|[]
                                                          |
                                                          |  O
                                                           \____________________

   NOTE: This sample is intended to compile for any Arduino compatible MCU
   architecture and Notecard/Notecarrier combination. However, it was created
   and tested using the Adafruit Huzzah32 and Notecarrier-AL.

   NOTE: This example has intentionally omitted error checking in order to
   highlight the elements required to make a power-efficient application.
*/

#include <Arduino.h>
#include <SensorModbusMaster.h>

#include <Notecard.h>

#include <Wire.h>
#include "main.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#ifndef ARDUINO_ARCH_ESP32
#error "this sketch exclusively targets the ESP32 because it uses esp-idf"
#endif

#include <WiFi.h>

// C trickery to convert a number to a string
#define STRINGIFY(x) STRINGIFY_(x)
#define STRINGIFY_(x) #x

// Definitions used by firmware update
#define PRODUCT_ORG_NAME        ""
#define PRODUCT_DISPLAY_NAME    "Updatable"
#define PRODUCT_FIRMWARE_ID     "1.2.0.0"
#define PRODUCT_DESC            ""
#define PRODUCT_MAJOR           1
#define PRODUCT_MINOR           2
#define PRODUCT_PATCH           0
#define PRODUCT_BUILD           0
#define PRODUCT_BUILT           __DATE__ " " __TIME__
#define PRODUCT_BUILDER         ""
#define PRODUCT_VERSION         STRINGIFY(PRODUCT_MAJOR) "." STRINGIFY(PRODUCT_MINOR) "." STRINGIFY(PRODUCT_PATCH)

// Define pin numbers based on the Feather and the Notecarrier-AF's user push button
#define buttonPin           21
#define buttonPressedState  LOW
#define ledPin              13

// Note that both of these definitions are optional; just prefix either line with // to remove it.
//  Remove serialNotecard if you wired your Notecard using I2C SDA/SCL pins instead of serial RX/TX
//  Remove serialDebug if you don't want the Notecard library to output debug information
//#define serialNotecard Serial1
#define serialDebug Serial

// This is the unique Product Identifier for your device.
// #define myProductID "com.gmail.brecky.morris:ucsc_eps"
#define productUID "com.gmail.brecky.morris:ucsc_eps"

Notecard notecard;

// Define the sensor's modbus address
byte modbusAddress = 0x01;  // The sensor's modbus address, or SlaveID

// Construct the modbus instance
modbusMaster modbus;

// Define pin number variables
const int DEREPin = -1;   // The pin controlling Recieve Enable and Driver Enable
// on the RS485 adapter, if applicable (else, -1)
// Setting HIGH enables the driver (arduino) to send text
// Setting LOW enables the receiver (sensor) to send text


/*
  #include <SPI.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
*/
#include <SD.h>
#include <SDI12.h>

#define RELAY_SET     12
#define RELAY_UNSET   27

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

// This period controls the waking frequency of your host MCU, and will have a
// direct impact on battery conservation. It should be used to strike a balance
// between battery performance and data collection requirements.
int sampling_interval = 600; // seconds

void setup() {

  delay(5000); // delay to ensure that the serial UART has started

  // pinMode(10, OUTPUT);

  pinMode(RELAY_SET, OUTPUT); // set pin
  pinMode(RELAY_UNSET, OUTPUT);

#ifdef ARDUINO_ARCH_ESP32
  // Disable radios to improve power profile
  WiFi.mode(WIFI_OFF);
  ::btStop();
#endif

  // Provide visual signal when the Host MCU is powered
  ::pinMode(LED_BUILTIN, OUTPUT);
  ::digitalWrite(LED_BUILTIN, HIGH);

  // Initialize Debug Output
  serialDebug.begin(115200);
  static const size_t MAX_SERIAL_WAIT_MS = 5000;
  size_t begin_serial_wait_ms = ::millis();
  while (!serialDebug && (MAX_SERIAL_WAIT_MS > (::millis() - begin_serial_wait_ms))) {
    ; // wait for serial port to connect. Needed for native USB
  }
  notecard.setDebugOutputStream(serialDebug);
  notecard.logDebugf("\n");

  // As the first thing, show the DFU partition information
  dfuShowPartitions();


  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    serialDebug.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization for the BMP sensor
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (! bmp.performReading()) {
    serialDebug.println("Failed to obtain a reading...");
    return;
  }

  float temperature = bmp.temperature;
  serialDebug.print("Temperature = ");
  serialDebug.print(temperature);
  serialDebug.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  // Initialize Notecard
  notecard.begin();

  if (DEREPin > 0) pinMode(DEREPin, OUTPUT);

  Serial.println("ok arduino");
  Serial1.begin(9600, SERIAL_8N1);  // port for communicating with sensor
  modbus.begin(modbusAddress, &Serial1, DEREPin); //DEREPin

  // Turn on debugging
  modbus.setDebugStream(&Serial);

  // Configure Notecard to synchronize with Notehub periodically, as well as
  // adjust the frequency based on the battery level
  {
    J * req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", productUID);
    JAddStringToObject(req, "mode", "periodic");
    // JAddStringToObject(req, "vinbound", "usb:60;high:120;normal:240;low:480;dead:0");
    // JAddStringToObject(req, "voutbound", "usb:30;high:60;normal:90;low:120;dead:0");
    JAddNumberToObject(req, "inbound", 60);
    JAddNumberToObject(req, "outbound", 60);
    notecard.sendRequest(req);
  }

  // Optimize voltage variable behaviors for LiPo battery
  {
    J * req = notecard.newRequest("card.voltage");
    JAddStringToObject(req, "mode", "lipo");
    notecard.sendRequest(req);
  }

  // Establish a template to optimize queue size and data usage
  {
    J * req = notecard.newRequest("note.template");
    JAddStringToObject(req, "file", "sensor.qo");
    J * body = JAddObjectToObject(req, "body");
    JAddNumberToObject(body, "temp", 12.1);
    JAddNumberToObject(body, "pressure", 12.1);
    JAddNumberToObject(body, "voltage", 12.1);
    JAddNumberToObject(body, "card", 12.1);
    JAddNumberToObject(body, "temprs485", 12.1);
    JAddNumberToObject(body, "humidity", 12.1);
    notecard.sendRequest(req);
  }

  // Sample and queue results
  {

    Serial.println("this is the new version of the firmware");

    // Notify the Notehub of our current firmware version
    J * req = notecard.newRequest("dfu.status");
    if (req != NULL) {
      JAddStringToObject(req, "version", firmwareVersion());
      notecard.sendRequest(req);
    }

    sampling_interval = ncGetEnvInt("sampling_interval", 600);
    Serial.print("sampling interval: ");
    Serial.println(sampling_interval); 

    Serial.println("opening relay");
    digitalWrite(27, HIGH);
    delay(100);
    digitalWrite(27, LOW);
    digitalWrite(12, LOW);
    delay(100);

    Serial.println("keeping relay open while sampling.");

    delay(5000); // let it warm up
    

    int temperatureModbusRaw = -3276;
    int humidityModbusRaw = -3276;

    digitalWrite(10, HIGH);

    delay(3000);

    // let's see if this fixes the problem
    int retries = 0;
    while (temperatureModbusRaw == -3276) {
      delay(3000);
      temperatureModbusRaw = modbus.int16FromRegister(0x03, 0, bigEndian);

      if (retries > 10) {
        Serial.printf("temperature from modbus isn't working correctly.");
        Serial.print("raw temperature: "); Serial.println(temperatureModbusRaw);
        temperatureModbusRaw = 0;
        break;
      }
      retries++;
    }

    retries = 0;
    while (humidityModbusRaw == -3276) { // not sure if this is the right number  // -3267
      delay(3000);
      humidityModbusRaw = modbus.int16FromRegister(0x03, 1, bigEndian);

      if (retries > 10) {
        Serial.printf("humidity from modbus isn't working correctly.");
        Serial.print("raw humidity: "); Serial.println(temperatureModbusRaw);
        temperatureModbusRaw = 0;
        break;
      }
      retries++;
    }

    Serial.print("raw humidity: "); Serial.println(humidityModbusRaw);

    float temperatureModbus = temperatureModbusRaw / 10.0;
    float humidityModbus = humidityModbusRaw / 10.0;

    digitalWrite(10, LOW);

    Serial.print("rs485 temperature: "); Serial.println(temperatureModbus);
    Serial.print("rs485 humidity: "); Serial.println(humidityModbus);
    delay(1000);

    Serial.println("closing relay for power savings");
    digitalWrite(27, LOW);
    digitalWrite(12, HIGH);
    delay(100);
    digitalWrite(12, LOW);

    if (! bmp.performReading()) {
      serialDebug.println("Failed to obtain a reading...");
      return;
    }

    float temperature = bmp.temperature;
    serialDebug.print("Temperature = ");
    serialDebug.print(temperature);
    serialDebug.println(" *C");

    float pressure = bmp.pressure / 100.0;
    serialDebug.print("Pressure = ");
    serialDebug.print(pressure);
    serialDebug.println(" hPa");

    serialDebug.print("Approx. Altitude = ");
    serialDebug.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    serialDebug.println(" m");

    double voltage = 0.0;
    J * rsp = notecard.requestAndResponse(notecard.newRequest("card.voltage"));
    if (rsp != NULL) {
      voltage = JGetNumber(rsp, "value");
      notecard.deleteResponse(rsp);
    }

    double loggertemperature = 0.0;
    rsp = notecard.requestAndResponse(notecard.newRequest("card.temp"));
    if (rsp != NULL) {
      loggertemperature = JGetNumber(rsp, "value");
      notecard.deleteResponse(rsp);
    }

    req = notecard.newRequest("note.add");
    JAddStringToObject(req, "file", "sensor.qo");
    J * body = JAddObjectToObject(req, "body");
    JAddNumberToObject(body, "temp", temperature); // JAddNumberToObject(body, "temp", temperature);
    JAddNumberToObject(body, "pressure", pressure);
    JAddNumberToObject(body, "voltage", voltage);
    JAddNumberToObject(body, "card", loggertemperature);
    JAddNumberToObject(body, "temprs485", temperatureModbus);
    JAddNumberToObject(body, "humidity", humidityModbus);

    notecard.sendRequest(req);
  }
}

void loop() {
  // Request sleep from loop to safeguard against tranmission failure, and
  // ensure sleep request is honored so power usage is minimized
  {

    dfuPoll(true);
    // Create a "command" instead of a "request", because the host
    // MCU is going to power down and cannot receive a response.
    J * req = NoteNewCommand("card.attn");
    JAddStringToObject(req, "mode", "sleep");
    JAddNumberToObject(req, "seconds", sampling_interval);
    notecard.sendRequest(req);
  }

  // Wait 1s before retrying
  ::delay(1000);
}

// This is a product configuration JSON structure that enables the Notehub to recognize this
// firmware when it's uploaded, to help keep track of versions and so we only ever download
// firmware buildss that are appropriate for this device.
#define QUOTE(x) "\"" x "\""
#define FIRMWARE_VERSION_HEADER "firmware::info:"
#define FIRMWARE_VERSION FIRMWARE_VERSION_HEADER          \
  "{" QUOTE("org") ":" QUOTE(PRODUCT_ORG_NAME)            \
  "," QUOTE("product") ":" QUOTE(PRODUCT_DISPLAY_NAME)    \
  "," QUOTE("description") ":" QUOTE(PRODUCT_DESC)        \
  "," QUOTE("firmware") ":" QUOTE(PRODUCT_FIRMWARE_ID)    \
  "," QUOTE("version") ":" QUOTE(PRODUCT_VERSION)         \
  "," QUOTE("built") ":" QUOTE(PRODUCT_BUILT)             \
  "," QUOTE("ver_major") ":" STRINGIFY(PRODUCT_MAJOR)     \
  "," QUOTE("ver_minor") ":" STRINGIFY(PRODUCT_MINOR)     \
  "," QUOTE("ver_patch") ":" STRINGIFY(PRODUCT_PATCH)     \
  "," QUOTE("ver_build") ":" STRINGIFY(PRODUCT_BUILD)     \
  "," QUOTE("builder") ":" QUOTE(PRODUCT_BUILDER)         \
  "}"

// In the Arduino IDE, the ino is built regardless of whether or not it is modified.  As such, it's a perfect
// place to serve up the build version string because __DATE__ and __TIME__ are updated properly for each build.
const char *productVersion() {
  return ("Ver " PRODUCT_VERSION " " PRODUCT_BUILT);
}

// Return the firmware's version, which is both stored within the image and which is verified by DFU
const char *firmwareVersion() {
  return &FIRMWARE_VERSION[strlen(FIRMWARE_VERSION_HEADER)];
}

// get float enviromental varialble from BluesIO, with default value
float ncGetEnv(char* envVariable, float defaultValue) {
  float rValue=defaultValue;
  J *req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", envVariable);
    J *rsp = notecard.requestAndResponse(req);
    if (rsp != NULL) {
      if (!notecard.responseError(rsp)) {
        if (JIsPresent(rsp, "text")) rValue=JAtoN(JGetString(rsp, "text"),NULL);
      }
      notecard.deleteResponse(rsp);
    }
  }
  return rValue;
}

// get int enviromental varialble from BluesIO, with default value
int ncGetEnvInt(char* envVariable, int defaultValue) {
  int rValue=defaultValue;
  J *req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", envVariable);
    J *rsp = notecard.requestAndResponse(req);
    if (rsp != NULL) {
      if (!notecard.responseError(rsp)) {
        if (JIsPresent(rsp, "text")) rValue=JAtoN(JGetString(rsp, "text"),NULL);
      }
      notecard.deleteResponse(rsp);
    }
  }
  return rValue;
}
