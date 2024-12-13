// C:\Users\Andy\Documents\Arduino\libraries\MPU6050
// https://www.youtube.com/watch?v=M9lZ5Qy5S2s
// https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
// https://forum.arduino.cc/t/mpu-6050s-linear-acceleration-is-way-off/171978/4
// https://github.com/jrowberg/i2cdevlib/tree/master
// Video explaining sensor fusion theory: https://www.youtube.com/watch?v=C7JQ7Rpwn2k&t=996s
// https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
// https://howtomechatronics.com/how-it-works/electrical-engineering/mems-accelerometer-gyrocope-magnetometer-arduino/
// https://atadiat.com/en/e-towards-understanding-imu-basics-of-accelerometer-and-gyroscope-sensors/
// https://forum.arduino.cc/t/mpu6050-sensor-resolution-and-sensitivity/582985
// https://plaw.info/articles/sensorfusion/

// https://how2electronics.com/connecting-esp32-to-amazon-aws-iot-core-using-mqtt/
// https://www.youtube.com/watch?v=idf-gGXvIu4
// https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/
// https://docs.aws.amazon.com/sns/latest/dg/sns-setting-up.html
// https://docs.aws.amazon.com/IAM/latest/UserGuide/enable-virt-mfa-for-root.html
// https://docs.aws.amazon.com/singlesignon/latest/userguide/quick-start-default-idc.html
// https://docs.aws.amazon.com/iot/latest/developerguide/iot-sns-rule.html
// https://eu-north-1.console.aws.amazon.com/dynamodbv2/home?region=eu-north-1#tables
// https://docs.aws.amazon.com/amazondynamodb/latest/developerguide/GettingStartedDynamoDB.html
// https://docs.aws.amazon.com/iot/latest/developerguide/iot-ddb-rule.html
// https://docs.aws.amazon.com/amazondynamodb/latest/developerguide/SettingUp.DynamoWebService.html

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/* This driver reads quaternion data from the MPU6060 and sends
   Open Sound Control messages.

  ESP32 DevKitC-WROOM
  MPU6050 devkit 1.0
  board   Lolin         Description
  ======= ==========    ====================================================
  VCC     5V USB   Not available on all boards so use 3.3V if needed.
  GND     GND             Ground
  SCL     GPIO022   I2C clock
  SDA     GPIO021   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D19 (GPIO19)   Interrupt pin

*/

/*
  Can the DMP produce 6-axis fused orientation data using the internal accelerometer and gyroscope?
  Yes. The DMP is capable of generating 6-axis orientation data in the form of a quaternion, which
  is a special vector/rotation matrix in the form of [w x y z], where each component is a value 
  between -1 and +1. Using the DMP binary code block from InvenSense' published MotionApps 2.0 platform,
  the quaternion is scaled to a 16-bit or integer representation, and more specifically into a signed
  15-bit integer between -16384 and +16383 for each of the four elements. The DMP uses the MPU's internal
  FIFO to output this data, and can send raw accelerometer and gyroscope values at the same time as well.
  The easiest way to achieve 6-axis DMP output without using the full MotionApps codebase from
  InvenSense is with the MPU6050_DMP6.info example sketch for the Arduino. 
*/

// ================================================================
// ===              WIFI AWS             ===
// ================================================================
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"


#define AWS_IOT_PUBLISH_TOPIC   "esp32iot/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32iot/sub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
uint8_t publishMsg = 1;

// ================================================================
// ===              MPU6050             ===
// ================================================================

#include "esp32/clk.h" // esp_clk_cpu_freq
#include "esp_log.h" // esp_log_timestamp()

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5/3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ESP8266 GPIO15
   pin.
 * ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float angVel[3]; // angular velocity [z:yaw, y:pitch, x:roll]

#define INTERRUPT_PIN 19 // use pin 19 on ESP32

const char DEVICE_NAME[] = "mpu6050";

// ================================================================
// ===               CUSTOM VARIABLES              ===
// ================================================================

#define NST 150 // total number of feature extraction window samples
#define NSW 15 // number of samples per feature window 
#define NFW 6 // number of feature windows stored
#define NTI 200 // total number of interations

uint8_t winSampleCount = 0; // Feature window sample count
uint8_t winSamplePrintCount = 0; // Feature window sample print count
uint16_t totalSampleCount = 0;
uint16_t count = 0;
char inputChar;

int32_t xFeatSum = 0;// Feature window: sum of samples. Max val (max 16 signed integer * number of feature samples): 16384*15=245,760. Max int32: 2,147,483,647
int32_t xFeatSumAbs = 0;
int32_t xFeatSumBuf[NFW] = {0}; 
uint32_t xFeatSumSq = 0; // Feature window: sum of squared samples. Max val (max 16 signed integer^2 * number of samples): 15*(16384^2)=4,026,531,840. Max uint32: 4,294,967,295
uint32_t xFeatSumSqBuf[NFW] = {0}; 
int32_t dxFeatSum = 0; // featSum delta between current feature window and 5 feature windows before
uint32_t dxFeatSumSq = 0; // featSumSq delta between current feature window and 5 feature windows before
uint32_t xStdDev = 0; // Standard deviation
uint32_t xStdDevBuf[NFW] = {0}; 
int32_t xMean = 0; // Mean
int32_t xMeanBuf[NFW] = {0}; 
uint32_t xStdDevL = 0; // Standard deviation left (before) of fall
uint32_t xStdDevR = 0; // Standard deviation right (after) of fall
int32_t xMeanL = 0; // Mean left (before) of fall
int32_t xMeanR = 0; // Mean right (after) of fall

int32_t yFeatSum = 0;// Feature window: sum of samples. Max val (max 16 signed integer * number of feature samples): 16384*15=245,760. Max int32: 2,147,483,647
int32_t yFeatSumAbs = 0;
int32_t yFeatSumBuf[NFW] = {0}; 
uint32_t yFeatSumSq = 0; // Feature window: sum of squared samples. Max val (max 16 signed integer^2 * number of samples): 15*(16384^2)=4,026,531,840. Max uint32: 4,294,967,295
uint32_t yFeatSumSqBuf[NFW] = {0}; 
int32_t dyFeatSum = 0; // featSum delta between current feature window and 5 feature windows before
uint32_t dyFeatSumSq = 0; // featSumSq delta between current feature window and 5 feature windows before
uint32_t yStdDev = 0; // Standard deviation
uint32_t yStdDevBuf[NFW] = {0}; 
int32_t yMean = 0; // Mean
int32_t yMeanBuf[NFW] = {0}; 
uint32_t yStdDevL = 0; // Standard deviation left (before) of fall
uint32_t yStdDevR = 0; // Standard deviation right (after) of fall
int32_t yMeanL = 0; // Mean left (before) of fall
int32_t yMeanR = 0; // Mean right (after) of fall

int32_t zFeatSum = 0;// Feature window: sum of samples. Max val (max 16 signed integer * number of feature samples): 16384*15=245,760. Max int32: 2,147,483,647
int32_t zFeatSumAbs = 0;
int32_t zFeatSumBuf[NFW] = {0}; 
uint32_t zFeatSumSq = 0; // Feature window: sum of squared samples. Max val (max 16 signed integer^2 * number of samples): 15*(16384^2)=4,026,531,840. Max uint32: 4,294,967,295
uint32_t zFeatSumSqBuf[NFW] = {0}; 
int32_t dzFeatSum = 0; // featSum delta between current feature window and 5 feature windows before
uint32_t dzFeatSumSq = 0; // featSumSq delta between current feature window and 5 feature windows before
uint32_t zStdDev = 0; // Standard deviation
uint32_t zStdDevBuf[NFW] = {0}; 
int32_t zMean = 0; // Mean
int32_t zMeanBuf[NFW] = {0}; 
uint32_t zStdDevL = 0; // Standard deviation left (before) of fall
uint32_t zStdDevR = 0; // Standard deviation right (after) of fall
int32_t zMeanL = 0; // Mean left (before) of fall
int32_t zMeanR = 0; // Mean right (after) of fall

uint32_t xyStdDevL = 0;
uint32_t xyStdDevR = 0;
uint32_t xzStdDevL = 0;
uint32_t xzStdDevR = 0;

// ================================================================
// ===               INTERRUPT TIMER               ===
// ================================================================

/* Timer variables start */
uint32_t cpu_freq;
uint8_t timer_id = 0; // The ESP32 has 4 independent timers, selected by an id between 0 and 3 (it's just the number of timers that can be running at the same time). Then we select the prescaler to apply to the timer clock signal. On the ESP32, this is the APB_CLK clock, clocked at 80 MHz.
uint16_t prescaler = 80; // Between 0 and 65535. In most examples available, a prescaler of 80 is used to obtain a counting period of 1µs. 80 / 80MHz = 80/80x10^6 = 1µs.
int threshold = 25000; // 64 bits value (limited to int size of 32bits) 1000000 = 1s if counting period is 1µs.
uint32_t freqXtal;
uint32_t freqCpu;
uint32_t freqApb;
volatile boolean timer_flag = false; // Volatile so variable persists
hw_timer_t * timer = NULL;
/* Timer variables end */

// void IRAM_ATTR timer_isr() {
//     // ets_printf("1ms timer\n"); // Specific print inside ISRs
//     timer_flag = true;
// }

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock (2.5µs). Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize(); // default is +/- 2g and +/- 250 rad/s sensitivity
  // mpu.initialize(ACCEL_FS::A4G,GYRO_FS::G2000DPS); // A4G = 4g and G250DPS = 250rad/s. Using overloaded function. Appears the DMP (algorithm that runs internal to the MPU-6050 to produce quarternion - so can't see it!) can't be used with anything other than 2g and 250rad/s sensitivity, so it doesn't appear the DMP library can be used with anything other than those sensitivities. https://forum.arduino.cc/t/mpu6050-sensor-resolution-and-sensitivity/582985/8
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize(); // This sets sample rate of 5ms, don't want to alter this function/use a different one to change sample rate for now. Default: Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV): 1000 / (1 + 4) = 200Hz. Gryroscope output rate of 1kHz determined by MPU6050_CFG_EXT_SYNC_SET_BIT = 5 in MPU6050_RA_CONFIG (DLPF_CFG register).

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(108);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(-30);
  mpu.setXAccelOffset(-193);
  mpu.setYAccelOffset(279);
  mpu.setZAccelOffset(1113); // 1688 factory default for test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // // Check array is initialised to zero
  // Serial.print("xFeatSum:");
  // for(uint8_t i=0;i<6;i++)
  // {
  //   Serial.print(xFeatSum[i]);
  //   Serial.print(",");
  // }
  // Serial.println();
}

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}

void publishMessage()
{
  // StaticJsonDocument<200> doc;
  JsonDocument doc;
  doc["fall"] = "Person A has fallen";
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);
 
  // StaticJsonDocument<200> doc;
  JsonDocument doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void setup(void)
{
  Serial.begin(115200);
  connectAWS();
  Serial.println(F("\nOrientation Sensor OSC output")); Serial.println();

  cpu_freq = esp_clk_cpu_freq();
  Serial.print("CPU freq (Hz):");
  Serial.println(cpu_freq);

  // timer = timerBegin(timer_id, prescaler, true);
  // timerAttachInterrupt(timer, &timer_isr, true);
  // timerAlarmWrite(timer, threshold, true);
  // timerAlarmEnable(timer);
  
  freqXtal = getXtalFrequencyMhz();
  Serial.print("Xtal freq = ");
  Serial.print(freqXtal);
  Serial.println(" MHz");
  
  freqCpu = getCpuFrequencyMhz();
  Serial.print("Cpu freq = ");
  Serial.print(freqCpu);
  Serial.println(" MHz");

  freqApb = getApbFrequency(); 
  Serial.print("Cpu Apb = ");
  Serial.print(freqApb);
  Serial.println(" Hz");

  mpu_setup();
}

void LeftShiftArrayInt32(int32_t* array)
{
    for (uint8_t i=0; i<NFW; i++)
    {
        if (i<(NFW-1))
        {
            array[i] =  array[i+1];
        }
        else
        {
            array[i] = 0;
        }
    }
}

void LeftShiftArrayUint32(uint32_t* array)
{
    for (uint8_t i=0; i<NFW; i++)
    {
        if (i<(NFW-1))
        {
            array[i] =  array[i+1];
        }
        else
        {
            array[i] = 0;
        }
    }
}

void GetMpuDmpValues()
{
    mpu.dmpGetQuaternion(&q, fifoBuffer); // See register data sheet. Data from registers placed in fifoBuffer, these values are then divided by 16384 (accuracy of signed 16 bit register and 2g sensitivity. 4g sensitivity is 8192 LSB/g, etc) to get values in g (i.e. 1g = 9.81 m/(s^2)). Value of 8192 is 1g with 2g sensitivity implemented. Bytes 0 to 13 in FIFO buffer. Gyro sensivity is 131 LSB/deg/s at 250 deg/s, 65.5 LSB/deg/s at 500 deg/s. Quaternion fuses the accelerometer and gyroscope data to correct rotational drift, correct accel due to gravity,  helps remove accelerometer noise without needing low pass filter (which can remove important data peaks). Accelerometers can give you tilt/angle relative to gravity (i.e. upwards direction), but have lots of hand jitter, gyroscopes are better at measuring tilt/angle, but they get drift due to not measuring gravity.
    // mpu.dmpGetEuler(euler, &q); // display Euler angles in degrees. Calculated from quarternions, suffers from Gimbal lock.
    mpu.dmpGetAccel(&aa, fifoBuffer); // Data from registers placed in fifoBuffer and raw values placed into aa (hasn't been divided by 16384)
    mpu.dmpGetGravity(&gravity, &q); // gravity vector
    mpu.dmpGetYawPitchRoll(angVel, &q, &gravity); // display yaw, pitch, roll in degrees/s. Calculated from quarternions, suffers from Gimbal lock, needs gravity vector for calculations.
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // display real acceleration that is in body coordinate frame (sensor coordinate frame), adjusted to remove gravity
    mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q); // display initial world coordinate frame acceleration, adjusted to remove gravity and rotated based on known orientation from quaternion. Yaw is relative to initial orientation, since no magnetometer is present in this case.
}

void SerialPlotter()
{
    Serial.print("axWorld:");
    Serial.print(aaWorld.x);
    Serial.print(",");
    Serial.print("ayWorld:");
    Serial.print(aaWorld.y);
    Serial.print(",");
    Serial.print("azWorld:");
    Serial.println(aaWorld.z);
}

void PrintHeading()
{
    Serial.println("---------------");
    Serial.print("System Time (ms)");
    // Serial.print(",\t");
    // Serial.print("wx (deg/s)");
    // Serial.print(",\t");
    // Serial.print("wy (deg/s)");
    // Serial.print(",\t");
    // Serial.print("wz (deg/s)");
    // Serial.print(",\t");
    // Serial.print("axRaw (int16)");
    // Serial.print(",\t");
    // Serial.print("ayRaw (int16)");
    // Serial.print(",\t");
    // Serial.print("azRaw (int16)");
    // Serial.print(",\t");
    // Serial.print("axReal (int16)");
    // Serial.print(",\t");
    // Serial.print("ayReal (int16)");
    // Serial.print(",\t");
    // Serial.print("azReal (int16)");
    Serial.print(",\t");
    Serial.print("axWorld (int16)");
    Serial.print(",\t");
    Serial.print("ayWorld (int16)");
    Serial.print(",\t");
    Serial.print("azWorld (int16)");
    // Serial.print(",\t");
    // Serial.print("gx (g)");
    // Serial.print(",\t");
    // Serial.print("gy (g)");
    // Serial.print(",\t");
    // Serial.print("gz (g)");
    // Serial.print(",\t");
    // Serial.print("axMeanL (int16)");
    // Serial.print(",\t");
    // Serial.print("axMeanR (int16)");
    // Serial.print(",\t");
    // Serial.print("ayMeanL (int16)");
    // Serial.print(",\t");
    // Serial.print("ayMeanR (int16)");
    // Serial.print(",\t");
    // Serial.print("azMeanL (int16)");
    // Serial.print(",\t");
    // Serial.print("azMeanR (int16)");
    // Serial.print(",\t");
    // Serial.print("axStdDevL");
    // Serial.print(",\t");
    // Serial.print("axStdDevR");
    // Serial.print(",\t");
    // Serial.print("ayStdDevL");
    // Serial.print(",\t");
    // Serial.print("ayStdDevR");
    // Serial.print(",\t");
    // Serial.print("azStdDevL");
    // Serial.print(",\t");
    // Serial.print("azStdDevR");
    Serial.print(",\t");
    Serial.print("axyStdDevL");
    Serial.print(",\t");
    Serial.print("axyStdDevR");
    // Serial.print(",\t");
    // Serial.print("axzStdDevL");
    // Serial.print(",\t");
    // Serial.print("axzStdDevR");
    Serial.println();
}

void PrintMpu6050Values()
{
    Serial.print(esp_log_timestamp()); // milliseconds
    // Serial.print(q.w, 6); // Serial.print(value, 6): 6 decimal places
    // Serial.print(q.x, 6);
    // Serial.print(q.y, 6);
    // Serial.print(q.z, 6);
    // Serial.print(euler[0] * 180/M_PI);
    // Serial.print(euler[1] * 180/M_PI);
    // Serial.print(euler[2] * 180/M_PI);
    // Serial.print(",\t");
    // Serial.print(angVel[2] * 180/M_PI); // roll: x
    // Serial.print(",\t");
    // Serial.print(angVel[1] * 180/M_PI); // pitch: y
    // Serial.print(",\t");
    // Serial.print(angVel[0] * 180/M_PI); // yaw: z
    // Serial.print(",\t");
    // Serial.print(aa.x); // aRaw
    // Serial.print(",\t");
    // Serial.print(aa.y); // aRaw
    // Serial.print(",\t");
    // Serial.print(aa.z); // aRaw
    // Serial.print(",\t");
    // Serial.print(aaReal.x); 
    // Serial.print(",\t");
    // Serial.print(aaReal.y);
    // Serial.print(",\t");
    // Serial.print(aaReal.z);
    Serial.print(",\t");
    Serial.print(aaWorld.x);
    Serial.print(",\t");
    Serial.print(aaWorld.y);
    Serial.print(",\t");
    Serial.print(aaWorld.z);
    // Serial.print(",\t");
    // Serial.print(gravity.x);
    // Serial.print(",\t");
    // Serial.print(gravity.y);
    // Serial.print(",\t");
    // Serial.print(gravity.z);
}

void PrintFeatures()
{
    // Serial.print(esp_log_timestamp()); // milliseconds
    // Serial.print(",\t");
    // Serial.print(xMeanL);
    // Serial.print(",\t");
    // Serial.print(xMeanR);
    // Serial.print(",\t");
    // Serial.print(yMeanL);
    // Serial.print(",\t");
    // Serial.print(yMeanR);
    // Serial.print(",\t");
    // Serial.print(zMeanL);
    // Serial.print(",\t");
    // Serial.print(zMeanR);
    // Serial.print(",\t");
    // Serial.print(xStdDevL);
    // Serial.print(",\t");
    // Serial.print(xStdDevR);
    // Serial.print(",\t");
    // Serial.print(yStdDevL);
    // Serial.print(",\t");
    // Serial.print(yStdDevR);
    // Serial.print(",\t");
    // Serial.print(zStdDevL);
    // Serial.print(",\t");
    // Serial.print(zStdDevR);
    Serial.print(",\t");
    Serial.print(xyStdDevL);
    Serial.print(",\t");
    Serial.print(xyStdDevR);
    // Serial.print(",\t");
    // Serial.print(xzStdDevL);
    // Serial.print(",\t");
    // Serial.print(xzStdDevR);
    // Serial.println();
}

void mpu_loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize); // Gets data from FIFO read write register 116 (0x74), which is populated with data from registers 59 to 96 (38 bytes). Look at the 16 bit functions. 59 to 64 (6 bytes - X 2bytes, Y 2bytes, Z 2bytes): accelerometer (buffer byte 28 to 37), 65 to 66 (2 bytes): temperature (buffer byte 26 to 27), 67 to 72 (6 bytes): gyro (buffer byte 16 to 25), 73 to 96 (24 bytes): external sensors (buffer bytes 0 to 25)

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

          // if (timer_flag == true)
          // {timer_flag = false;}

        count++;

        if (count % 2 == 0) // Sample count of 2 appears to be around 20ms (50Hz)
        {
            count = 0;
            winSampleCount++;
            winSamplePrintCount = winSampleCount;

            GetMpuDmpValues();

            // SerialPlotter();
            
            xFeatSum += (int32_t)aaWorld.x;
            yFeatSum += (int32_t)aaWorld.y;
            zFeatSum += (int32_t)aaWorld.z;
            xFeatSumAbs = (uint32_t)abs(aaWorld.x);
            xFeatSumSq += sq(xFeatSumAbs);
            yFeatSumAbs = (uint32_t)abs(aaWorld.y);
            yFeatSumSq += sq(yFeatSumAbs);
            zFeatSumAbs = (uint32_t)abs(aaWorld.z);
            zFeatSumSq += sq(zFeatSumAbs);

            if (winSampleCount % NSW == 0)
            {
                winSampleCount = 0;

                // x mean
                xFeatSumBuf[NFW-1] = xFeatSum;
                dxFeatSum = dxFeatSum + xFeatSumBuf[NFW-1] - xFeatSumBuf[0]; 
                LeftShiftArrayInt32(xFeatSumBuf);
                xMean = (2*dxFeatSum)/NST;
                xMeanBuf[NFW-1] = xMean;
                xMeanL =  xMeanBuf[0];
                xMeanR = xMeanBuf[NFW-1];
                LeftShiftArrayInt32(xMeanBuf);

                // x std dev
                xFeatSumSqBuf[NFW-1] = xFeatSumSq;
                dxFeatSumSq = dxFeatSumSq + xFeatSumSqBuf[NFW-1] - xFeatSumSqBuf[0];
                LeftShiftArrayUint32(xFeatSumSqBuf);
                xStdDev = sqrt((dxFeatSumSq - 2*sq(dxFeatSum)/NST)/(NST/2 - 1));
                xStdDevBuf[NFW-1] = xStdDev;
                xStdDevL = xStdDevBuf[0];
                xStdDevR = xStdDevBuf[NFW-1];
                LeftShiftArrayUint32(xStdDevBuf);

                // y mean
                yFeatSumBuf[NFW-1] = yFeatSum;
                dyFeatSum = dyFeatSum + yFeatSumBuf[NFW-1] - yFeatSumBuf[0]; 
                LeftShiftArrayInt32(yFeatSumBuf);
                yMean = (2*dyFeatSum)/NST;
                yMeanBuf[NFW-1] = yMean;
                yMeanL =  yMeanBuf[0];
                yMeanR = yMeanBuf[NFW-1];
                LeftShiftArrayInt32(yMeanBuf);

                // y std dev
                yFeatSumSqBuf[NFW-1] = yFeatSumSq;
                dyFeatSumSq = dyFeatSumSq + yFeatSumSqBuf[NFW-1] - yFeatSumSqBuf[0];
                LeftShiftArrayUint32(yFeatSumSqBuf);
                yStdDev = sqrt((dyFeatSumSq - 2*sq(dyFeatSum)/NST)/(NST/2 - 1));
                yStdDevBuf[NFW-1] = yStdDev;
                yStdDevL = yStdDevBuf[0];
                yStdDevR = yStdDevBuf[NFW-1];
                LeftShiftArrayUint32(yStdDevBuf);

                // z mean
                zFeatSumBuf[NFW-1] = zFeatSum;
                dzFeatSum = dzFeatSum + zFeatSumBuf[NFW-1] - zFeatSumBuf[0]; 
                LeftShiftArrayInt32(zFeatSumBuf);
                zMean = (2*dzFeatSum)/NST;
                zMeanBuf[NFW-1] = zMean;
                zMeanL =  zMeanBuf[0];
                zMeanR = zMeanBuf[NFW-1];
                LeftShiftArrayInt32(zMeanBuf);

                // z std dev
                zFeatSumSqBuf[NFW-1] = zFeatSumSq;
                dzFeatSumSq = dzFeatSumSq + zFeatSumSqBuf[NFW-1] - zFeatSumSqBuf[0];
                LeftShiftArrayUint32(zFeatSumSqBuf);
                zStdDev = sqrt((dzFeatSumSq - 2*sq(dzFeatSum)/NST)/(NST/2 - 1));
                zStdDevBuf[NFW-1] = zStdDev;
                zStdDevL = zStdDevBuf[0];
                zStdDevR = zStdDevBuf[NFW-1];
                LeftShiftArrayUint32(zStdDevBuf);

                xyStdDevL = sqrt(sq(xStdDevL) + sq(yStdDevL));
                xyStdDevR = sqrt(sq(xStdDevR) + sq(yStdDevR));
                // xzStdDevL = sqrt(sq(xStdDevL) + sq(zStdDevL));
                // xzStdDevR = sqrt(sq(xStdDevR) + sq(zStdDevR));

                xFeatSum = 0;
                xFeatSumSq = 0;
                yFeatSum = 0;
                yFeatSumSq = 0;
                zFeatSum = 0;
                zFeatSumSq = 0;
            }

            // // Test
            // if (inputChar=='a')
            // {
            //   publishMessage();
            //   inputChar = 'p';
            //   Serial.println("publishMessage");
            // }

            if (xyStdDevL<300 && xyStdDevR>850 && publishMsg==1)
            {
              publishMessage();
              Serial.println("publishMessage");
              publishMsg=0;
            }

            // Need more elgant solution to make sure fall is only detected once every sliding window (3s)? Maybe simple latch is just the answer.
            if (inputChar=='r')
            {
              publishMsg = 1;
              Serial.println("publishMessageReset");
              inputChar='p';
            }

            // Data sampling
            if (inputChar=='o') // In serial monitor need to select "No Line Ending" option, otherwise serial monitor will add return or line ending character. https://www.programmingelectronics.com/serial-read/. Putty config local echo "force off" to not show characters on screen that you have entered, local line ending "force off" so that no line ending or return characters are entered along with your own char entry. https://stackoverflow.com/questions/4999280/how-to-send-characters-in-putty-serial-communication-only-when-pressing-enter
            {
                totalSampleCount++;

                if (totalSampleCount==1)
                {
                    PrintHeading();
                }

                if (totalSampleCount <= NTI) // sample every 20ms for 3s. 3s is 150 samples
                {
                    PrintMpu6050Values();

                    if (winSamplePrintCount % NSW == 0)
                    {
                        PrintFeatures();
                    }
                    Serial.println();
                }
                else
                {
                    inputChar = 'p';
                    totalSampleCount = 0;
                }
            } 
        }
    }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete 
*/
/**************************************************************************/
void loop(void)
{
  
    if (Serial.available() > 0)
    {
        inputChar = Serial.read();
    }

    mpu_loop(); 
    client.loop();
}

