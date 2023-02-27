
#include <SoftwareSerial.h>
SoftwareSerial BTserial(18, 19); // RX | TX


void setup() {

BTserial.begin(9600); }

void loop() {
 float lat1=33.597549;
 float lat2=33.597617;
 float long1,long2;
long1=73.123608;
long2=73.133478;

//IMPORTANT: The complete String has nto be of the Form: 1234,1234,1234,1234;

//(every Value has to be seperated through a comma (',') and the message has to

//end with a semikolon (';'))

//BTserial.print("Lat1: ");

BTserial.println(lat1*1000000.000000);

if (lat2>0)   //conditions for lat2
{ lat2=lat2+180; } 
else if(lat2<0)
{ lat2=lat2-180; } 
else 
{ BTserial.print("Invalid Number"); }

//BTserial.print("Lat2: "); //sending lat2
//BTserial.println(lat2*1000000.000000);



if (long1>0)  //conditions for long1
{ long1=long1+360; }
else if (long1<0)
{ long1=long1-360; }
else 
{ BTserial.print("Invalid Number"); }
 
//BTserial.print("Long1: "); //sending long1
BTserial.println(long1*1000000.000000);

if (long2>0)   //conditions for long2
{ long2=long2+550; }
else if (long2<0)
{ long2=long2-550; }
else 
{ BTserial.print("Invalid Number"); }
//BTserial.print("Long2: ");    //sending long2
BTserial.println(long2*1000000.000000);
//message to the receiving device

delay(500);

}
