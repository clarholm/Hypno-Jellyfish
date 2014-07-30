// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

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
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

//Neopixel conf

#include <Adafruit_NeoPixel.h>
#define PIN 5
 
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(12, PIN, NEO_RGB + NEO_KHZ800);

uint8_t  mode   = 0, // Current animation effect
         offset = 0; // Position of spinny eyes
uint32_t color  = 0xFF0000; // Start red
uint32_t currentTimeRead;
uint32_t prevTimeRead;
uint32_t prevTime;
int noOfAddedSamples;
int currentX;
int currentY;
int currentZ;
int totalX;
int totalY;
int totalZ;
float averageX;
float averageY;
float averageZ;
float lastAverageX;
float lastAverageY;
float lastAverageZ;
float CurrentXspeed;
float CurrentYspeed;
float CurrentZspeed;
int movementTreshold = 8;
int red;
int green;
int blue;
int c;
uint8_t myColors[][3] = {{232, 100, 255},   // purple
                         {200, 200, 20},   // yellow 
                         {30, 200, 200},   // blue
                         {130, 22, 200}, 
                         {30, 50, 55}, 
                         {22, 160 , 70}, //pink
                          };
int wait = 25;
// don't edit the line below
#define FAVCOLORS sizeof(myColors) / 3
//Neopixel conf done

//MCU conf start



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//MCU conf done

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
    //Neo Pixel start
    pixels.begin();
    pixels.setBrightness(85); // 1/3 brightness
    prevTime = millis();
    //Neo Pixel done
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    }
        
        //Neo Pixel part start
        uint8_t  i;
        uint32_t t;
        
        readDataFromMCU();
        sumDataFromMCU();
        if (checkIfTimeToCalculateAverage()== true){
        calculateAverage();
        CurrentXspeed = calculateSpeed(averageX, lastAverageX);
        CurrentYspeed = calculateSpeed(averageY, lastAverageY);
        CurrentZspeed = calculateSpeed(averageZ, lastAverageZ);
        if (abs(CurrentXspeed)>movementTreshold || abs(CurrentYspeed)>movementTreshold || abs(CurrentZspeed)>movementTreshold){
        Serial.println("Movement detected");
        printCurrentSpeedToCommandLine();
        fadeOutCurrentColor(10);
        pickNewColor();
        changeColor();
        

       }
        //printCurrentSpeedToCommandLine();
        //printCurrentAverageToCommandLine();
        clearMCUValues();
        }
        
        

        /*
        switch(mode) {
       
         case 0: // Random sparks - just one LED on at a time!
          i = random(12);
          pixels.setPixelColor(i, color);
          pixels.show();
          delay(10);
          pixels.setPixelColor(i, 0);
          break;
       
         case 1: // Spinny wheels (3 LEDs on at a time)
          for(i=0; i<9; i++) {
            uint32_t c = 0;
            if(((offset + i) & 2) < 2) c = color; // 4 pixels on...
            pixels.setPixelColor(   i, c); // First eye
          }
          pixels.show();
          offset++;
          delay(50);
          break;
        }
       */
       
       /*
        t = millis();
        if((t - prevTime) > 8000) {      // Every 8 seconds...
          mode++;                        // Next mode
          if(mode > 1) {                 // End of modes?
            mode = 0;                    // Start modes over
            color >>= 8;                 // Next color R->G->B
            if(!color) color = 0xFF0000; // Reset to red
          }
          for(i=0; i<32; i++) pixels.setPixelColor(i, 0);
          prevTime = t;
        }
        */
         //Neo Pixel part stop     
        
}
void fadeOutCurrentColor(int steps){
for (int x=steps; x >= 0; x--) {
      int r = red * x; r /= steps;
      int g = green * x; g /= steps;
      int b = blue * x; b /= steps;
            for(int i=0; i<12; i++) {
      pixels.setPixelColor(i, pixels.Color(r, g, b));
      }
      pixels.show();
      delay(wait);
      }
      

}

void changeColor()
{
  
      for (int x=0; x < 10; x++) {
      int r = red * (x+1); r /= 10;
      int g = green * (x+1); g /= 10;
      int b = blue * (x+1); b /= 10;
      
      for(int i=0; i<12; i++) {
      pixels.setPixelColor(i, pixels.Color(r, g, b));
      }
      pixels.show();
      delay(wait);
      }
      
        for(int i=0; i<12; i++) {
        pixels.setPixelColor(i, pixels.Color(red, green, blue)); // First eye
        }
      printColor(red, green,blue);
        pixels.show();


}

void printColor(int red, int green, int blue){
            Serial.print("R,G,B\t");
            Serial.print(red);
            Serial.print("\t");
            Serial.print(green);
            Serial.print("\t");
            Serial.println(blue);
}

void pickNewColor(){
    c = random(FAVCOLORS);
    red = myColors[c][0];
    green = myColors[c][1];
    blue = myColors[c][2]; 
}

float calculateSpeed(float currentDir, float lastDir){
  float currentSpeed = lastDir - currentDir;
  return currentSpeed;
}

void readDataFromMCU(){
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}

void sumDataFromMCU(){
  totalX = totalX + (q.x*100);
  totalY = totalY + (q.y*100);
  totalZ = totalZ + (q.z*100);
  noOfAddedSamples = noOfAddedSamples + 1;
}

void calculateAverage(){
  lastAverageX = averageX;
  lastAverageY = averageY;
  lastAverageZ = averageZ;
  averageX = 0;
  averageY = 0;
  averageZ = 0;
  averageX = totalX / noOfAddedSamples;
  averageY = totalY / noOfAddedSamples;
  averageZ = totalZ / noOfAddedSamples;
  }


boolean checkIfTimeToCalculateAverage(){
        currentTimeRead = millis();
        if((currentTimeRead - prevTimeRead) > 300){
        prevTimeRead = currentTimeRead;
        return true;
        }
        else
        return false;
}

void printCurrentSpeedToCommandLine(){
     Serial.print("Current Speed\t");
            Serial.print(CurrentXspeed);
            Serial.print("\t");
            Serial.print(CurrentYspeed);
            Serial.print("\t");
            Serial.println(CurrentZspeed);
}

void printCurrentAverageToCommandLine(){
     Serial.print("current average\t");
            Serial.print(averageX);
            Serial.print("\t");
            Serial.print(averageY);
            Serial.print("\t");
            Serial.println(averageZ);
            Serial.print("\t");
            Serial.println(noOfAddedSamples);
}

void clearMCUValues(){
noOfAddedSamples = 0;
currentX = 0;
currentY = 0;
currentZ = 0;
totalX = 0;
totalY = 0;
totalZ = 0;

}



