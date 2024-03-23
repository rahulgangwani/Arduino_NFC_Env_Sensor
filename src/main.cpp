#include <Arduino.h>
#include <Wire.h>
#include <SmeNfc.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 11
#define BME_MISO 10
#define BME_MOSI 9
#define BME_CS 8

#define FIELD_DETECT_PIN 2  // When an NFC reader is near, this pin changes state

#define SME_2_1         "#!Hello, World!!#"

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
bool bme280_failed = false;

bool nfcOk = false; // True when NFC comm is good
byte buffer[UID_SIZE];  // Used to store the buffer

// put function declarations here:
void fieldDetectISR();

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(FIELD_DETECT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FIELD_DETECT_PIN), fieldDetectISR, FALLING);

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the SmartEverything as Master
  Wire.begin();

  // just clear the buffer
  for (int i = 0; i < UID_SIZE; i++) {
    buffer[i] = 0;
  }

  if (smeNfcDriver.readUID(buffer)) {
    smeNfc.storeText(NDEFFirstPos, SME_2_1);
    nfcOk = true;
  }

  if (nfcOk) {
    Serial.print("Serial number (UID): ");
    for (int i = 0; i < UID_SIZE; i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(':');
    }
  }
  else {
    Serial.println("Error setting up NFC tag");
  }
  Serial.println();

  unsigned status;
  status = bme.begin(); //Default BME280 settings

  if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        bme280_failed = true;
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);
  delay(500);

  if (!bme280_failed) {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
  }
}

/*********************************************************************
 * Function triggered on a falling edge of the Field detect pin
 * This is used to wake up the processor to process NFC data
 * In an RTOS, this is used to defer a task to write/read NFC
**********************************************************************/
void fieldDetectISR()
{
  Serial.println("NFC FIELD DETECTED");
}