// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "helper_3dmath.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

double targetAngleDeg;
uint8_t fifoBuffer;
Quaternion quaternion;
VectorFloat gravity;
float ypr;

void loop() {
    
    String targetString = Serial.readString();
    if (targetString != "") {
      targetAngleDeg = targetString.toDouble();
    }

    mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
    mpudmpGetGravity(&gravity, &quaternion);
    mpudmpGetYawPitchRoll(ypr, &quaternion, &gravity);
    Yaw = (ypr[0] * 180.0 / M_PI);
    Pitch = (ypr[1] *  180.0 / M_PI);
    Roll = (ypr[2] *  180.0 / M_PI);
    DPRINTSTIMER(100) {
      DPRINTSFN(15, "\tYaw:", Yaw, 6, 1); // \t is the tab character // fancy Serial.Print
      DPRINTSFN(15, "\tPitch:", Pitch, 6, 1); // how to use DPRINTSFN() 15: USING 15 CHARACTERS MAX for the number to string conversion, "\tPitch": displayed name, Pitch: Floating point value to display, 6: using 6 characters in the conversion, 1: trim the float to 1 decimal place
      DPRINTSFN(15, "\tRoll:", Roll, 6, 1); //DPRINTSFN() Will display any floating point number with simple and space everything nicely
      DPRINTLN(); // Simple New Line
    }
    
  
//    // these methods (and a few others) are also available
//    mpu.getAcceleration(&ax, &ay, &az);
//    mpu.getRotation(&gx, &gy, &gz);
//
//    #ifdef OUTPUT_READABLE_ACCELGYRO
//        // display tab-separated accel/gyro x/y/z values
//        Serial.print("a/g:\t");
//        Serial.print(ax); Serial.print("\t");
//        Serial.print(ay); Serial.print("\t");
//        Serial.print(az); Serial.print("\t");
//        Serial.print(gx); Serial.print("\t");
//        Serial.print(gy); Serial.print("\t");
//        Serial.println(gz);
//    #endif
//
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);
    delay(300);

}
