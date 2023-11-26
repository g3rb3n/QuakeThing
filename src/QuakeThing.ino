#include <Thing.h>
#include <MPU9250.h>
#include <devices/SSD1306I2C.h>
#include <Point3D_Print.h>
#include <BlinkAsync.h>

#include <I2C.h>
#include <SPI.h>

using namespace g3rb3n;
using namespace ootb;

Thing thing;
MPU9250 mpu;
BlinkAsync led(LED_BUILTIN, HIGH, LOW);
SSD1306I2C oled(0x3C, 128, 64);

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

void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);

    setupSerial();
    setupOLED();
    setupThing();
    setupQuake();

    display(thing.clientId(), 0, 3);
    Serial.println(thing.clientId());
    led.setIntervals(900, 100);
}

void loop()
{
    led.handle();
    thing.handle();
    sample();
}

void setupSerial()
{
    Serial.begin(230400);
    Serial.println();
}

void setupOLED()
{
    oled.begin();
    oled.flipHorizontal(true);
    oled.flipVertical(true);
}

void setupThing()
{
    thing.onStateChange([](const String &msg)
    {
        display(msg, 6, 2);
        Serial.println(msg);
    });

    thing.begin();
    String sensorTopic = "things/" + thing.clientId() + "/quake/acceleration";
    thing.addSensor(sensorTopic, 1000, getValue);

    String actuatorTopic = "things/" + thing.clientId() + "/quake/display";
    thing.addActuator(actuatorTopic, callback);
}

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
        String s = "@ " + String(static_cast<float>(count)) + " Hz";
        display(s, 5, 1);
        Serial.println(s);
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
    if (debug && (abso(a.x - avg.x) > threshold ||
                  abso(a.y - avg.y) > threshold ||
                  abso(a.z - avg.z) > threshold))
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
    if (debug)
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

void setupQuake()
{
    while (!mpu.connected())
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

void getValue(Value &value)
{
    value = maxDev * 1000;
    String s = "@ ";
    s += (float)value;
    s += " mG";
    display(s, 4, 1);
    maxDev = 0;
}

void callback(Value &s)
{
    String msg(s);
    display(msg, 0, 3);
    Serial.println(msg);
}

String empty("                     ");

void display(const String &value, int row, int rows)
{
    // oled.clear();
    for (uint8_t i = 0; i < rows; ++i)
    {
        oled.setCursor(i + row, 0);
        oled.print(empty);
    }
    oled.setCursor(row, 0);
    oled.print(value);
    oled.display();
}
