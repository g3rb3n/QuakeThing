#define MQTT_SERVER "10.0.0.66"
#define MQTT_PORT 1883
#define MQTT_USER "gerbenvaneerten@gmail.com"
#define MQTT_PASSWORD "Dwe57Kj8"

#define MQTT_DISPLAY_TOPIC "displays/DS18B20"
#define MQTT_SENSOR_TOPIC "sensors/DS18B20"

#define PIN_RESET 255  //
#define DC_JUMPER 0  // I2C Addres: 0 - 0x3C, 1 - 0x3D

//#define _DEBUG_

#include <Arduino.h>

#include <Thing.h>

#include <MPU9250.h>
#include <Point3D_Print.h>

#include <BlinkAsync.h>

using namespace g3rb3n;

Thing thing;
MPU9250 mpu;
BlinkAsync led(LED_BUILTIN, HIGH, LOW);

std::string client;
std::string sensor_topic;
std::string display_topic;

Point3D<float> a;
Point3D<float> avg;
Point3D<float> avgAdd;
Point3D<float> devP;

long next;
long now;
uint16_t count = 0;
bool debug = false;
float threshold = .03;
float avgD = 100.;
float maxDev = 0;
float dev;

float max(float a, float b)
{
  return (a > b) ? a : b;
}

float abso(float a)
{
  return (a > 0) ? a : -a;
}

bool sample()
{
  now = millis();
  if (now > next)
  {
    Serial.print("@ ");
    Serial.print(static_cast<float>(count), 2);
    Serial.print(" Hz");
    Serial.println();
    count = 0;
    next = now + 1000;
  }
  ++count;
  mpu.acceleration(a);

  avg *= .99;
  avgAdd = a;
  avgAdd /= 100.;
  avg += avgAdd;
  devP = a;
  devP -= avg;
  dev = abso(devP.x);
  maxDev = max(maxDev, dev);
  dev = abso(devP.y);
  maxDev = max(maxDev, dev);
  dev = abso(devP.z);
  maxDev = max(maxDev, dev);
  if(
    abso(a.x - avg.x) > threshold ||
    abso(a.y - avg.y) > threshold ||
    abso(a.z - avg.z) > threshold
  )
  {
    Serial.print("quake : ");
    Serial.print(abso(devP.x), 5);
    Serial.print(' ');
    Serial.print(abso(devP.y), 5);
    Serial.print(' ');
    Serial.print(abso(devP.z), 5);
    Serial.println();
  }
  a = avg;
  if(debug)
  {
    Serial.print(millis(), 10);
    Serial.print('\t');
    Serial.print(a.x, 10);
    Serial.print('\t');
    Serial.print(a.y, 10);
    Serial.print('\t');
    Serial.print(a.z, 10);
    Serial.println();
  }
  return true;
}

void printSettings()
{
  Serial.print("AccelerationResolution: ");
  Serial.print(mpu.accelometerScaleMode());
  Serial.print(' ');
  Serial.println(mpu.accelerationResolution());
  Serial.print("GyroscopeResolution:    ");
  Serial.print(mpu.gyroscopeScaleMode());
  Serial.print(' ');
  Serial.println(mpu.gyroscopeResolution());
  Serial.print("SampleRateDivider:      ");
  Serial.print(mpu.sampleRateDividerMode());
  Serial.print(' ');
  Serial.println(mpu.sampleRateDivider());
  Serial.print("GyroFrequency:          ");
  Serial.println(mpu.gyroscopeFrequency());
  Serial.print("AccelFrequency:         ");
  Serial.println(mpu.accelometerFrequency());
  Serial.print("FifoMode:               ");
  Serial.println(mpu.fifoMode());
  Serial.print("ClockSource:            ");
  Serial.println(mpu.clockSource());
  Serial.println();
}

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);  
  Serial.begin(230400);
  setupThing();
  setupQuake();
  led.setIntervals(900, 100);
}


void loop() 
{
  led.handle();
  thing.handle();
  sample();
}

void setupThing() {

  onActivity(true);

  char chipid[40];
  sprintf(chipid, "%08X", ESP.getChipId());

  Serial.println();
  client += chipid;
  Serial.print("Client:");
  Serial.print(client.c_str());
  Serial.println();
  
  thing.onActivity(&onActivity);
  thing.addWiFi("woonkamer", "ongratis");
  thing.addWiFi("central", "ongratis");
  thing.addWiFi("zuid", "ongratis");
  thing.addWiFi("rfln", "ongratis");
  thing.setupWiFi();
  thing.setMQTT(MQTT_SERVER, MQTT_PORT, client.c_str(), MQTT_USER, MQTT_PASSWORD);

  sensor_topic += "sensors/Quake/";
  sensor_topic += chipid;
  thing.addSensorTopic(sensor_topic.c_str(), 1000, getValue);
  Serial.println(sensor_topic.c_str());

  display_topic += "displays/Quake/";
  display_topic += chipid;
  thing.addActuatorTopic(display_topic.c_str(), callback);
  Serial.println(display_topic.c_str());

  onActivity(false);

  thing.onActivity(0);
}

void onActivity(bool active)
{
  digitalWrite(BUILTIN_LED, active ? LOW : HIGH);  
}

void setupQuake() 
{
  while(!mpu.connected())
  {
    Serial.print("Waiting for MPU9250, it responded with : ");
    Serial.print(mpu.identification());
    Serial.println();
  }
  
  mpu.reset();
  while (!mpu.isReset())
  {
    Serial.println("Wait for MPU reset");
  }

  mpu.initialize();
  mpu.wakeUp();

  mpu.setClockSource(MPU9250_CS_INTERNAL);
  mpu.setAccelometerScale(AS_2G);
  mpu.setAccelometerFrequencyMode(AF_1046);
  mpu.setSampleRateDividerMode(RATE_DIVIDER_1);

  printSettings();
  
  Serial.println("MPU9250 initialized");

  delay(1000);
}

void getValue(float& value)
{
  onActivity(true);
  value = maxDev * 1000;
  maxDev = 0;
  onActivity(false);
}

void callback(float &s)
{
  Serial.println(s, 10);
}
