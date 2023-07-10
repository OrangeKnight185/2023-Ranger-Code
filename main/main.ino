#include <Ethernet.h>
#include <EthernetUDP.h>
#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

#define DEBUG

byte mac[] = {0x53, 0x48, 0x41, 0x52, 0x4B, 0x53};
IPAddress ip(10, 11, 57, 2);
unsigned int port = 8888;
EthernetUDP udp;
const uint8_t LARGEST_PACKET = 17;
byte buffer[LARGEST_PACKET];
const uint8_t ETH_CS = 10;

const uint8_t THRUSTER_PINS[] = {A0, A1, A2, A3, A4, A5, 0, 1};
const uint8_t NUM_OF_THRUSTERS = 8;
Servo thruster_array[NUM_OF_THRUSTERS];
Servo newton;
Servo camera;
Servo sketch_newton;
const uint8_t SD_CS = 12;

MS5837 depth_sensor;
int16_t depth;
bool depth_sensor_init = false;
int depth_last_send = millis();

const uint8_t I2C_ADDR = 0xBB;
char i2c_buffer;

bool is_light_on = false;

struct acceleration_t {
  float x, y, z;
  int16_t x_scaled, y_scaled, z_scaled;
} acceleration;

struct euler_t {
  float yaw, pitch, roll;
  int16_t yaw_scaled, pitch_scaled, roll_scaled;
} angles;

void set_thruster(uint8_t thruster_id, uint16_t speed) {
    thruster_array[thruster_id].writeMicroseconds(speed);
}

void initialize_thrusters() {
    for (uint8_t i = 0; i < NUM_OF_THRUSTERS; i++) {
        thruster_array[i].attach(THRUSTER_PINS[i]);
        set_thruster(i, 1500);
    }
}

void stop_thrusters() {
    for (uint8_t i = 0; i < NUM_OF_THRUSTERS; i++) {
        set_thruster(i, 1500);
    } newton.writeMicroseconds(1500);
}

uint16_t bytesToInt(uint8_t upper, uint8_t lower) {
    uint16_t ret = upper << 8;
    ret = ret | lower;
    return ret;
}

void setup() {
  Ethernet.init(ETH_CS);
  Ethernet.begin(mac, ip);
  udp.begin(port);

  // #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Serial initialized.");
  // #endif

  initialize_thrusters();
  newton.attach(9);
  sketch_newton.attach(6);
  camera.attach(11);
  pinMode(SD_CS, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);

  if (!depth_sensor.init()) {
    #ifdef DEBUG
      Serial.println("Init failed!");
      Serial.println("Are SDA/SCL connected correctly?");
      Serial.println("Blue Robotics Bar02: White = SDA, Green = SCL");
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("Depth sensor found!");
    #endif
    depth_sensor.setFluidDensity(1000);
    depth_sensor_init = true;
   }

  while (millis() < 8000);

  for (int i = 0; i < NUM_OF_THRUSTERS; i++) {
    thruster_array[i].writeMicroseconds(1500);
    delay(100);

    thruster_array[i].writeMicroseconds(1500);
    delay(200);

    thruster_array[i].writeMicroseconds(1450);
    delay(100);

    thruster_array[i].writeMicroseconds(1500);
    delay(1000);
  }

  digitalWrite(SD_CS, HIGH);
  delay(500);
  digitalWrite(SD_CS, LOW);
  delay(500);
  digitalWrite(SD_CS, HIGH);
  delay(500);
  digitalWrite(SD_CS, LOW);
  delay(1000);

  newton.writeMicroseconds(1600);
  delay(300);
  newton.writeMicroseconds(1400);
  delay(300);

  sketch_newton.writeMicroseconds(1600);
  delay(300);
  sketch_newton.writeMicroseconds(1400);
  delay(300);
}

void loop() {
  static int time_at_last_command = millis();

  uint16_t packet_size = udp.parsePacket();

  #ifdef DEBUG
    //Serial.println("Loop execution.");
  #endif
  if (packet_size != 0 && packet_size <= LARGEST_PACKET) {
    udp.read(buffer, packet_size);
    time_at_last_command = millis();
    #ifdef DEBUG
      Serial.println("Got a packet!");
    #endif

    if (buffer[0] == 'T') {
      for (uint8_t i = 0; i < NUM_OF_THRUSTERS; i++) {
        uint16_t pwm_speed = bytesToInt(buffer[2 * i + 1], buffer[2 * i + 2]);

        set_thruster(i, pwm_speed);
        #ifdef DEBUG
            Serial.println(i);
            Serial.println(pwm_speed);
        #endif
      }
    } else if (buffer[0] == 'M') {
      if (buffer[1] == 'N') {
        uint16_t pwm_speed = bytesToInt(buffer[2], buffer[3]);
        newton.writeMicroseconds(pwm_speed);
      } else {
        uint16_t pwm_speed = bytesToInt(buffer[2], buffer[3]);
        sketch_newton.writeMicroseconds(pwm_speed);
      }
    } else if (buffer[0] == 'L') {
//      Wire.beginTransmission(I2C_ADDR);
//      Wire.write(buffer[2]);
//      Wire.write(buffer[3]);
//      Wire.endTransmission();
        if (buffer[1] == 'G') {
          if (buffer[2] == '0') {
            digitalWrite(SD_CS, LOW);
          } else {
            digitalWrite(SD_CS, HIGH);
          }
        }
    } else if (buffer[0] == 'C') {
      uint16_t pwm_speed = bytesToInt(buffer[1], buffer[2]);
      camera.writeMicroseconds(pwm_speed);
    }
  }
  
  if (millis() - time_at_last_command > 100) {
      stop_thrusters();
  }

  if (depth_sensor_init && millis() - depth_last_send > 250) {
    depth_sensor.read();
    #ifdef DEBUG
      Serial.println("Depth: ");
      Serial.print(depth_sensor.depth());
      Serial.println(" (m)");
    #endif
    depth = depth_sensor.depth() * 100;
    depth_packet(depth);
    depth_last_send = millis();
  }

  // Wire.requestFrom(I2C_ADDR, 1);
  // if (Wire.available()) {
  //   i2c_buffer = Wire.read();
  //   if (i2c_buffer == 'L') {
  //     while (true) {
  //       leak_packet();
  //       stop_thrusters();
  //     }
  //   }    
  // }
}

void gyro_packet(int16_t yaw, int16_t pitch, int16_t roll) {
  byte packet[7];
  packet[0] = 'G';

  packet[1] = highByte(yaw);
  packet[2] = lowByte(yaw);
  packet[3] = highByte(pitch);
  packet[4] = lowByte(pitch);
  packet[5] = highByte(roll);
  packet[6] = lowByte(roll);

  if(udp.remoteIP() != IPAddress(0, 0, 0, 0)) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(packet, 7);
    udp.endPacket();
  }  
}

void accel_packet(int16_t x, int16_t y, int16_t z) {
  byte packet[7];
  packet[0] = 'A';

  packet[1] = highByte(x);
  packet[2] = lowByte(x);
  packet[3] = highByte(y);
  packet[4] = lowByte(y);
  packet[5] = highByte(z);
  packet[6] = lowByte(z);

  if(udp.remoteIP() != IPAddress(0, 0, 0, 0)) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(packet, 7);
    udp.endPacket();
  }
}

void depth_packet(int16_t depth) {
  byte packet[3];
  packet[0] = 'D';

  packet[1] = highByte(depth);
  packet[2] = lowByte(depth);

  if(udp.remoteIP() != IPAddress(0, 0, 0, 0)) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(packet, 3);
    udp.endPacket();
  }  
}
