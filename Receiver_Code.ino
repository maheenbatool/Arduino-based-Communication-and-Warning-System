

// SimpleRxAckPayload- the slave or the receiver
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SoftwareSerial.h>
#include <math.h>
#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//i2c pins
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //
SoftwareSerial mySerial(11,10); // Rx|Tx pins for Gps
Adafruit_GPS GPS(&mySerial); //Create GPS object
String NMEA1;  //We will use this variable to hold our first NMEA sentence (for gps)
String NMEA2;   // We will use this variable to hold our second NMEA sentence (for gps)
char c;
float pi=3.14159265;

int outPin = 2;   // choose the input pin (for a pushbutton)
int val = 0;     // variable for reading the pin 
const int buzzer = 3; //buzzer to arduino pin 

#define CE_PIN  9   //CE pin of nrf24L01 to pin 9 of arduino mega
#define CSN_PIN 53   // CSN pin of nrf24L01 to pin 53 of arduino mega 

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};  //receiver address on which data of GPS(i.e lat long) will be raceived and acknowledgment will be sent back to the transmitter 

RF24 radio(CE_PIN, CSN_PIN); 

unsigned long dataReceived[2]; // this must match dataToSend in the TX
unsigned long ackData[2];// the two values to be sent to the master/transmitter 
float dist;

float red1[2]; //for converting data recv back to decimal values
float msg[2]; //for reading lat long of gps attached to this(receiver) module
bool newData = false;

//==============

void setup() {
    lcd.begin(16,4);
    lcd.backlight();//Power on the back light
    
    pinMode(outPin, OUTPUT); //output pin 2 for pushbutton that will display data of gps(satellites, angel and speed) on pressing
    pinMode(buzzer, OUTPUT); //output pin 3 for buzzer

    Serial.begin(9600);
    
    GPS.begin(9600);       //Turn GPS on at baud rate of 9600
    GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    delay(1000); 
    
    lcd.setCursor(0,0); 
    lcd.print(" GPS Start");
    Serial.println("GPS Start");
    delay(300);
    lcd.clear();

  //  Serial.println("SimpleRxAckPayload Starting");
    radio.begin();
    radio.setDataRate( RF24_250KBPS ); //Setting Data Rate for Nrf24L01
    radio.openReadingPipe(1, thisSlaveAddress); //Opening reading pipe of nrf24l01 for getting data from transmitter module

    radio.enableAckPayload(); // Ack payload function for nrf24l01 that will help to send back the data of gps (lat,long) at the time it will receive the data(lat,long) from the transmitter
    radio.writeAckPayload(1, &ackData, sizeof(ackData)); // pre-load data

    radio.startListening(); //built in function for nrf24L01 that will listen on reading pipe while data is being received
}

//==========



void loop() {
    readGPS(); //funtion for reading GPS
  if (GPS.fix == 1) {   //while gps is connected to the satellites 
    getData();
//    showData();
  }    
}



//Functions
//============

void getData() {
    
  val = digitalRead(outPin);  // read input value for pushbutton 
  if (val == HIGH) {  //if push button is pressed speed,angle and satellites will be displayed on LCD. This data has nothing to do with the calculations.
  
  
         lcd.clear();
         delay(1000);
         
         lcd.setCursor(0,0);
         lcd.print("Speed:"); 
         lcd.setCursor(6,0);
         lcd.print(GPS.speed);
         delay(300);
         

         
         lcd.setCursor(0,1);
         lcd.print("Angle:");
         lcd.setCursor(6,1);
         lcd.print(GPS.angle);
         delay(300); 
         
         lcd.setCursor(0,2);
         lcd.print("Satellites:");
         lcd.setCursor(11,2);
         lcd.print((int)GPS.satellites); 
         delay(300); 
         lcd.clear();
        
  } 
  //if push button is not pressed lat long and distance will continuously display on LCD
    lcd.setCursor(0,0); 
    lcd.print("Lat=");
    lcd.setCursor(5,0); 
    lcd.print(GPS.latitudeDegrees,6);
    delay(300);

     lcd.setCursor(0,1); 
     lcd.print("Long=");
     lcd.setCursor(5,1);
     lcd.print(GPS.longitudeDegrees,6);
     delay(300);
         
    
      if ( radio.available() ) { // condition if transmitter nrf is in range of this receiver nrf
        
      msg[0]=GPS.latitudeDegrees,6; //lat of gps attached to this(receiver) module
      msg[1]=GPS.longitudeDegrees,6; //lat of gps attached to this(receiver) module
   
      radio.read( &dataReceived, sizeof(dataReceived) ); //reading data of nrf that is received from transmiter 
        
      updateReplyData();//function for updating values of lat long of this module that will be sent in acknowledgment message back to the transmitter at the time it will receive message from the transmitter 
        
      float lat1,lon1,lat2,lon2;
      red1[0]=dataReceived[0]/1000000.000000; 
      red1[1]=dataReceived[1]/1000000.000000;
            
//             Serial.print("Lat conv=");
//             Serial.println(red1[0]);
//             Serial.print("Lon conv=");
//             Serial.println(red1[1]);

             lat1=red1[0]* pi/180 ; //module 2(transmitter) lattitude which is received
             lon1=red1[1]* pi/180; //module 2(transmitter) longitude which is received
             lat2=msg[0]* pi/180;  //lat of gps attached to this(receiver) module
             lon2=msg[1]* pi/180;   //long of gps attached to this module

             long r= 6371;
             int  d;
             float u,v;
             u=sin((lat1-lat2)/2);
             v=sin((lon1-lon2)/2);
             dist=2*r*asin( sqrt(u*u + cos (lat1) * cos (lat2) * v *v) );//distance in km
             float dis;//distance in meters
             dis=(dist*1000)-0.2; //in meters
//             Serial.print("Dist: ");
//             Serial.print(dis);//meters
//             Serial.println("meters");
            

             lcd.setCursor(0,2);
             lcd.print("Distance");
             lcd.setCursor(0,3);
             lcd.print(dis);
             lcd.setCursor(10,3);
             lcd.print("M");
             delay(500);

             if (dis<=10)
         { 
             tone(buzzer, 1000); // Send 1KHz sound signal...
             delay(1000);        // ...for 1 sec
             noTone(buzzer);     // Stop sound...
             delay(1000);
  
          } 
            newData = true;
    }
  } 


//================

//void showData() {
//    if (newData == true) {
//        Serial.print("Data received ");
//        Serial.println(dataReceived[0]);
//        Serial.println(dataReceived[1]);
//        Serial.print(" ackPayload sent ");
//        Serial.print(ackData[0]);
//        Serial.print(", ");
//        Serial.println(ackData[1]);
//        newData = false;
//    }
//}

//================

void updateReplyData() { 
  
 msg[0]=GPS.latitudeDegrees,6;
 msg[1]=GPS.longitudeDegrees,6;
 
 ackData[0]=msg[0]*1000000;
 ackData[1]=msg[1]*1000000;
 

    
    radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the payload for the next time
}

//================
+
void readGPS() { //This function will read and remember two NMEA sentences from GPS
clearGPS();    //Serial port probably has old or corrupt data, so begin by clearing it all out
for (int i=0;i<=10;i++) { //Keep reading characters in this loop until a good NMEA sentence is received
 c = GPS.read(); //read a character from the GPS
  }
  GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
  NMEA1 = GPS.lastNMEA();    //Once parsed, save NMEA sentence into NMEA1
for (int i=0;i<=10;i++) { //Go out and get the second NMEA sentence, should be different type than the first one read above.
    c = GPS.read();
  }
 GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();
//  Serial.println(NMEA1);
  //Serial.println(NMEA2);
 // Serial.println("");
}

//================

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
 
