#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(112500);
  #if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels. We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(255); // bright as the sun
  pixels.fill(0x00FF00);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    pixels.fill(0xFF0000); pixels.show();
    while(1);
  } delay(1000); pixels.show();
  bno.setExtCrystalUse(false);

  Wire.begin(0xBB);
  Wire.onRequest(transmit_data);
}

void transmit_data() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  Serial.println("X:");
  Serial.println(gyro.x());
  Serial.println("Y:");
  Serial.println(gyro.y());
  Serial.println("Z:");
  Serial.println(gyro.z());
  Serial.println("");
}

void loop() {
  transmit_data();
  delay(500);
}
