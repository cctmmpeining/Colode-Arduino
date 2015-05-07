#include <Adafruit_VC0706_new.h>
#include <Arduino.h>
#include <AltSoftSerial.h>
#include <string.h>
// This is a basic snapshot sketch using the VC0706 library.
// On start, the Arduino will find the camera and SD card and
// then snap a photo, saving it to the SD card.
// Public domain.

// If using an Arduino Mega (1280, 2560 or ADK) in conjunction
// with an SD card shield designed for conventional Arduinos
// (Uno, etc.), it's necessary to edit the library file:
//   libraries/SD/utility/Sd2Card.h
// Look for this line:
//   #define MEGA_SOFT_SPI 0
// change to:
//   #define MEGA_SOFT_SPI 1
// This is NOT required if using an SD card breakout interfaced
// directly to the SPI bus of the Mega (pins 50-53), or if using
// a non-Mega, Uno-style board.

#include <SD.h>

// comment out this line if using Arduino V23 or earlier
#include <SoftwareSerial.h>         

// uncomment this line if using Arduino V23 or earlier
// #include <NewSoftSerial.h>       

// SD card chip select line varies among boards/shields:
// Adafruit SD shields and modules: pin 10
// Arduino Ethernet shield: pin 4
// Sparkfun SD shield: pin 8
// Arduino Mega w/hardware SPI: pin 53
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
#define chipSelect 10

// For UNO, AltSoftSerial library is required, please get it from:
// http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)  
  AltSoftSerial BLEMini;  
#else
  #define BLEMini Serial1
#endif

// Pins for camera connection are configurable.
// With the Arduino Uno, etc., most pins can be used, except for
// those already in use for the SD card (10 through 13 plus
// chipSelect, if other than pin 10).
// With the Arduino Mega, the choices are a bit more involved:
// 1) You can still use SoftwareSerial and connect the camera to
//    a variety of pins...BUT the selection is limited.  The TX
//    pin from the camera (RX on the Arduino, and the first
//    argument to SoftwareSerial()) MUST be one of: 62, 63, 64,
//    65, 66, 67, 68, or 69.  If MEGA_SOFT_SPI is set (and using
//    a conventional Arduino SD shield), pins 50, 51, 52 and 53
//    are also available.  The RX pin from the camera (TX on
//    Arduino, second argument to SoftwareSerial()) can be any
//    pin, again excepting those used by the SD card.
// 2) You can use any of the additional three hardware UARTs on
//    the Mega board (labeled as RX1/TX1, RX2/TX2, RX3,TX3),
//    but must specifically use the two pins defined by that
//    UART; they are not configurable.  In this case, pass the
//    desired Serial object (rather than a SoftwareSerial
//    object) to the VC0706 constructor.

// Using SoftwareSerial (Arduino 1.0+) or NewSoftSerial (Arduino 0023 & prior):
#if ARDUINO >= 100
// On Uno: camera TX connected to pin 2, camera RX to pin 3:
SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
// On Mega: camera TX connected to pin 69 (A15), camera RX to pin 3:
//SoftwareSerial cameraconnection = SoftwareSerial(69, 3);
#else
NewSoftSerial cameraconnection = NewSoftSerial(2, 3);
#endif


Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);
int len1;
int incomingByte = 0;   // for incoming serial data
const int redPin = 0;
const int greenPin = 0;
const int bluePin = 0;

//#define CLK_PIN 2 // 定義連接腳位
//#define DT_PIN 3
//#define SW_PIN 4
//#define interruptA 0
volatile long count = 1;
unsigned long t = 0;
unsigned int rgbColour[3]={0};
char str[8];


// Using hardware serial on Mega: camera TX conn. to RX1,
// camera RX to TX1, no SoftwareSerial object is required:
//Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

void setup() {
         Serial.begin(38400);
 //        attachInterrupt(interruptA, rotaryEncoderChanged, FALLING);
 //        pinMode(CLK_PIN, INPUT_PULLUP); // 輸入模式並啟用內建上拉電阻
 //        pinMode(DT_PIN, INPUT_PULLUP); 
 //        pinMode(SW_PIN, INPUT_PULLUP); 
 //        pinMode (13,INPUT);
  // When using hardware SPI, the SS pin MUST be set to an
  // output (even if not connected or used).  If left as a
  // floating input w/SPI on, this can cause lockuppage.
#if !defined(SOFTWARE_SPI)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if(chipSelect != 53) pinMode(53, OUTPUT); // SS on Mega
#else
  if(chipSelect != 10) pinMode(10, OUTPUT); // SS on Uno, etc.
#endif
#endif


  Serial.println("VC0706 Camera snapshot test");
  // Try to locate the camera
  cam.begin();

  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  //cam.setImageSize(VC0706_160x120);          // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");

  Serial.println("Snap in 3 secs...");
  delay(3000);


}



void loop() {
   int i;//定义变量
   int o=0;
   unsigned char colode[4];
   uint16_t len1;
   sprintf(str, "%02d%02X%02X%02X", count, rgbColour[0], rgbColour[1], rgbColour[2]);
   Serial.println(str);
  while(1)
  {
  
    i=analogRead(5);//读取模拟5口电压值
    if((i>1000)&&(o==0))//如果电压值大于1000（即4.88V）
      {
        Serial.println(i);
        cam.takePicture();
        Serial.println("get photo");
        if(len1=cam.frameLength())
        {
        Serial.println("get length");
        Serial.println(len1,DEC);
        cam.readPicture(len1);
        Serial.println("sent data");
        len1=0;
      }
        o=1;
        
      }
    else if(i<1000)
    {
      o=0;
    }
      if(cam.readData(VC0706_GET_RGB, 9)){
      Serial.println("Get RGB");
      Serial.println(cam.getRGB_r());
      Serial.println(cam.getRGB_g());
      Serial.println(cam.getRGB_b());
      rgbColour[0]=cam.getRGB_r();
      rgbColour[1]=cam.getRGB_g();
      rgbColour[2]=cam.getRGB_b();
//   setColourRgb((rgbColour[0]/5)*count, (rgbColour[1]/5)*count, (rgbColour[2]/5)*count);
      sprintf(str, "%02d%02X%02X%02X", count, rgbColour[0], rgbColour[1], rgbColour[2]);
 //   sendBluetooth(str);
      }
      // 按下開關，歸零
  //    if(digitalRead(SW_PIN) == LOW){
  //    count = 1;
 //     Serial.println("count reset to 0");
 //     delay(300);}
      
        }
  }

//rotary
//void setColourRgb(unsigned int red, unsigned int green, unsigned int blue) {
//  analogWrite(redPin, red);
//  analogWrite(greenPin, green);
//  analogWrite(bluePin, blue);
// }

/*void rotaryEncoderChanged(){ // when CLK_PIN is FALLING
   unsigned long temp = millis();
   if(temp - t < 200) // 去彈跳
   return;
   t = temp;
   count += (digitalRead(DT_PIN) == HIGH ? 1 : -1);
   Serial.println(count);
   Serial.println(rgbColour[0]/count);
   setColourRgb((rgbColour[0]/5)*count, (rgbColour[1]/5)*count, (rgbColour[2]/5)*count);
   sprintf(str, "%02d%02X%02X%02X", count, rgbColour[0], rgbColour[1], rgbColour[2]);
   sendBluetooth(str);
   
 }
 */
 
//boolean sendBluetooth(char string[]){
//  Serial.println("get count+RGB string");
//  Serial.println(str);
//  Serial.begin(57600);
//  for(int i = 0; i<8; i++)
//  {
//    BLEMini.write(string[i]);
//  }
//  Serial.begin(38400);
//}

 
  // if (! cam.takePicture()) 
 //   Serial.println("Failed to snap!");
 // else 
 //   Serial.println("Picture taken!");
  
  // Create an image with the name IMAGExx.JPG
  //char filename[13];
  //strcpy(filename, "IMAGE00.JPG");
  //for (int i = 0; i < 100; i++) {
  //  filename[5] = '0' + i/10;
  //  filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
  //  if (! SD.exists(filename)) {
  //    break;
  //  }
  //}
  
  // Open the file for writing
  //File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  //uint16_t jpglen = cam.frameLength();
  //Serial.print("Storing ");
  //Serial.print(jpglen, DEC);
  //Serial.print(" byte image.");

  //int32_t time = millis();
  //pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  //byte wCount = 0; // For counting # of writes
  //while (jpglen > 0) {
    // read 32 bytes at a time;
  //  uint8_t *buffer;
  //  uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
  //  buffer = cam.readPicture(bytesToRead);
  //  imgFile.write(buffer, bytesToRead);
  //  if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
  //    Serial.print('.');
  //    wCount = 0;
  //  }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
  //  jpglen -= bytesToRead;
  //}
  //imgFile.close();

  //time = millis() - time;
  //Serial.println("done!");
  //Serial.print(time); Serial.println(" ms elapsed");
