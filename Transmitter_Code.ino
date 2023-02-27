

// SimpleTxAckPayload - the master or the transmitter

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include<math.h>
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SoftwareSerial.h> //Load the Software Serial Library. This library in effect gives the arduino additional serial ports
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//i2c pins
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // 
SoftwareSerial mySerial(11,10); //Initialize SoftwareSerial, and tell it you will be connecting through pins 2 and 3. Rx pin of gps to pin 2 of arduino and Tx of gps to pin 3 of arduino.
Adafruit_GPS GPS(&mySerial); //Create
String NMEA1;  //We will use this variable to hold our first NMEA sentence
String NMEA2; 
char c;
bool stat = true;
bool rslt;


int outPin=2;
int val =0;
const int buzzer = 3;


#define CE_PIN   9
#define CSN_PIN 53
const byte slaveAddress[5] = {'R','x','A','A','A'}; //same address as for receiver
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
unsigned long msg[2];
float  msg_a[2];
unsigned long ackData[2] = {-1, -1}; // to hold the two values coming from the slave/receiver
bool newData = false;
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second 
 
 
float lat1,long1,lat2,long2;
float aa[2];
float pi=3.14159265;
float dist;


//===============

void setup() {

    Serial.begin(9600);
    lcd.begin(16,4);
    lcd.backlight();//Power on the back light
    lcd.clear();
    pinMode(outPin,OUTPUT);
    pinMode(buzzer, OUTPUT); 

  GPS.begin(9600);       //Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  delay(1000);  //Pause
  //Serial.println("GPS start");

    //Serial.println("SimpleTxAckPayload Starting");
    lcd.setCursor(0,0);
    lcd.print("GPS starting");
    delay(300);
    lcd.clear();
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.enableAckPayload();
    radio.setRetries(3,5); // delay, count
    radio.openWritingPipe(slaveAddress); //
    
}

//=============

void loop() { 
  
  readGPS();
           if (stat)
       {
        if (GPS.fix==1){
    currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {
        send();
    }
    
    showData();    
}
       }
}

//================

void send() {

          val = digitalRead(outPin);
      if (val == HIGH)
      {


          lcd.setCursor(0,0);
          lcd.print("Speed=");
          lcd.setCursor(6,0);
          lcd.print(GPS.speed);
          delay(300);
       


          lcd.setCursor(0,1);
          lcd.print("Angle=");
          lcd.setCursor(6,1);
          lcd.print(GPS.angle);
          delay(300);
    

          lcd.setCursor(0,2);
          lcd.print("Satellites=");
          lcd.setCursor(11,2);
          lcd.print((int)GPS.satellites);
          delay(300);
          lcd.clear();
          
     
      }

          msg_a[0]=GPS.latitudeDegrees,6;
          lcd.setCursor(0,0);
          lcd.print("lat=");
          lcd.setCursor(5,0);
          lcd.print(msg_a[0]);
          delay(300);
  
          
         
          msg_a[1]=GPS.longitudeDegrees,6;
          lcd.setCursor(0,1);
          lcd.print("long=");
          lcd.setCursor(5,1);
          lcd.print(msg_a[1]);
          delay(300);

     lat1=msg_a[0]*pi/180;
     long1=msg_a[1]*pi/180;
  

    msg[0]= msg_a[0]*1000000;
    msg[1]= msg_a[1]*1000000;
    rslt = radio.write( &msg, sizeof(msg) );
        // Always use sizeof() as it gives the size as the number of bytes.
        // For example if dataToSend was an int sizeof() would correctly return 2

    if (rslt) {
      
        if ( radio.isAckPayloadAvailable() ) {
            radio.read(&ackData, sizeof(ackData));
            newData = true;
        }
        else {;
          lcd.clear();
            //Serial.println("  Acknowledge but no data ");
            lcd.setCursor(0,0);
            lcd.print(" Acknowledge but");
            lcd.setCursor(0,1);
            lcd.print("no data ");
        }
     
    }
    else {
  
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Tx Failed");
        
    }

    prevMillis = millis();
 }
       
//=================

void showData() {
    if (newData == true) {
     
        aa[0]=ackData[0]/1000000.000000;
        aa[1]=ackData[1]/1000000.000000;
        
        lat2=aa[0]*pi/180;   // value of lat from receiver module
        long2=aa[1]*pi/180;   //value of long from receiver module
        

    long r= 6371;
    int  d;
    float u,v;
    u=sin((lat1-lat2)/2);
    v=sin((long1-long2)/2);
    dist=2*r*asin( sqrt(u*u + cos (lat1) * cos (lat2) * v *v) );//distance in km
    float dis;//distance in meters
    dis=(dist*1000)-0.2; //in meters
    lcd.setCursor(0,2);

    lcd.setCursor(0,2);
    lcd.print("Distance");
    lcd.setCursor(0,3);
    lcd.print(dis);
    lcd.setCursor(10,3);
    lcd.print("M");
    delay(500);

  if (dis<=10){
  tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec
         }

        newData = false;
    }
}


//=============

void readGPS() { //This function will read and remember two NMEA sentences from GPS
clearGPS();    //Serial port probably has old or corrupt data, so begin by clearing it all out
for (int i=0;i<=10;i++) { //Keep reading characters in this loop until a good NMEA sentence is received
c = GPS.read(); //read a character from the GPS
}
GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
NMEA1 = GPS.lastNMEA();    //Once parsed, save NMEA sentence into NMEA1
for (int i=0;i<=10;i++) { //Go out and get the second NMEA sentence, should be different type than the first one read above.
 c = GPS.read();
 GPS.parse(GPS.lastNMEA());
 NMEA2 = GPS.lastNMEA();
 //Serial.println(NMEA1);
//Serial.println(NMEA2);
//Serial.println("");
 }

}
void clearGPS() {  //Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
while (!GPS.newNMEAreceived()) {
  c = GPS.read();
}
  GPS.parse(GPS.lastNMEA());
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

}
