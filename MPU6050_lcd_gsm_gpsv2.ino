#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f,16,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
boolean resp_read=0;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
int display_self_sensor=0;
///gsm gps

#include <TinyGPS.h>
//33.6271848,73.1060767
float g_flat=32.197680;//34.197680 , 73.246551
float g_flon=73.246551;
float o_lat=0;
float o_lon=0;
boolean updata=0;
boolean getr=0;
boolean getd=0;
float myspeed=0;
float carspeed=0;
long int last_time_beat=0;
long int now_time_beat=0;
boolean send_sms_f=0;
unsigned int speed_limit=8;///////////////////////////////////change the speed limit from here
////
boolean stringComplete=0;
String inputString="";
//SoftwareSerial11 Serial3(6, 4);
boolean show_f=0;
String response="";
String sms="";
///
boolean send_sms_info=0;
boolean smsr=0;



TinyGPS gps;



int buzzer=3;
int buzzer2=13;
float distance_to_other=0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
    Serial3.begin(9600);
  Serial2.begin(9600);
  pinMode(buzzer,OUTPUT);
  pinMode(buzzer2,OUTPUT);
 lcd.init();                      // initialize the lcd 
//  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("GSM GPS Base ");
  lcd.setCursor(2,1);
  lcd.print("System");
   lcd.setCursor(0,2);
  //lcd.print("Arduino LCM IIC 2004");
  // lcd.setCursor(2,3);
  //lcd.print("Power By Ec-yuan!");
 // 
 digitalWrite(buzzer,1);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  lcd.print("MPU ERROR");
  }else
  {
    lcd.print("MPU OK");
  }

  delay(3000); // Wait for sensor to stabilize
digitalWrite(buzzer,0);
///gsm enabling commands
keep_a();//AT\r
keep_a();//AT\r
eco_off();//ATE0\r
text_mode();//AT+CMGF=1\r
delet_msg();//AT+CMGD=1,4\r
sms_forwarding();//AT+CNMI=1,2,0,0,0// sms forwarding
 /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}
long td=0;
long check_gps=0;
long send_sms_time=0;
void loop() {
  if(millis()-send_sms_time>20000)
  {
    send_sms_location(g_flat,g_flon);
    send_sms_time=millis();
  }
  if(millis()-td>50)
  {
 sensor();
  td=millis();
  }
  if(millis()-check_gps>5000)
  {
    get_gps();
   check_gps=millis();
  }
  SerialEvent2();
  if(smsr)
  {
    String ns=response;
    Serial.println("Data Received : ");
    Serial.println(ns);
    unsigned int di=ns.indexOf('$');
    unsigned int si=ns.indexOf('*');
     unsigned int fc=0;
    unsigned int cc=0;
    String s1="";
    String s2="";
        if(si>di)
    {
      for(fc=1;fc<si;fc++)
      {
        if(ns.charAt(fc)==',')
        {
          cc++;
          goto lst;
        }
        if(cc==0)
        {
          s1=s1+ns.charAt(fc);
        }
        if(cc==1)
        {
          s2=s2+ns.charAt(fc);
        }
        
      lst:
      delay(1);
      }
      Serial.print("Lat2 :");Serial.println(s1);
      Serial.print("Lon2 :");Serial.println(s2);
      o_lat=s1.toFloat();
      o_lon=s2.toFloat();
      Serial.println(o_lat,6);
      Serial.println(o_lon,6);
  //    g_flat
   //   g_flon
   
        float distLat = abs(o_lat - g_flat) * 111194.9;
        float distLong = 111194.9 * abs(o_lon - g_flon) * cos(radians((o_lat + g_flat) / 2));
        float distance = sqrt(pow(distLat, 2) + pow(distLong, 2));
        distance_to_other=distance;
        Serial.print("Distance : ");Serial.println(distance_to_other);
      }
    response="";
    smsr=0;
  }
}
double lroll=0;
double lpitch=0;
void sensor()
{
   /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
if(++display_self_sensor>10)
{

  Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
 // Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");
  Serial.print("\t");
  Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
 // Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
  Serial.print("\r\n");
  delay(2);
  lcd.clear();
  lcd.setCursor(-4,0);
  lcd.print(g_flat,4);
  lcd.print(" ");
  lcd.print(g_flon,4);
  lcd.setCursor(-4,1);
  lcd.print(carspeed,1);
  lcd.print("km/h ");
  lcd.print(distance_to_other,1);
  lcd.print("m ");
 
  lcd.setCursor(-4,3);
  
  lcd.print("P: ");
  lcd.print(pitch,1);

  lcd.print(" R: ");
  lcd.print(roll,1);
  lcd.print("  ");
 if(abs(pitch-lpitch)>5||abs(roll-lroll)>5)
 {
  lcd.setCursor(-4,2);
  lcd.print("!!!!Alert!!!!");
  digitalWrite(buzzer,1);digitalWrite(buzzer2,1);
 }else
 {
    lcd.setCursor(-4,2);
  lcd.print("!!!!GOOD!!!!");
  digitalWrite(buzzer,0);digitalWrite(buzzer2,0);
 }
 lpitch=pitch;
 lroll=roll;
display_self_sensor=0;
}
}
///// gsm
///////////////////////////////////
void check_response()
{
  int uuuui=0;
  resp_read=1;
   do
  {
    SerialEvent2();
    delay(1);
    uuuui++;
    }while(uuuui<1000);
      resp_read=0;
    //  Serial.println(response);
}
//////
void keep_a()
{
 Serial.println("TAP");
   response="";
   Serial2.write('A');delay(100);Serial2.write('T');delay(100);Serial2.write('\r');
check_response();
}
//////
void eco_off()
{
 Serial.println("ECO OFF");
   response="";
   Serial2.write('A');delay(100);Serial2.write('T');delay(100);Serial2.write('E');delay(100);Serial2.write('0');delay(100);Serial2.write('\r');
check_response();
}
//
void delet_msg()
{ Serial.println("delet sms");
   response="";
  Serial2.write('A');delay(100);Serial2.write('T');delay(100);Serial2.write('+');delay(100);Serial2.write('C');delay(100);Serial2.write('M');delay(100);Serial2.write('G');delay(100);Serial2.write('D');delay(100);Serial2.write('=');delay(100);Serial2.write('1');delay(100);Serial2.write(',');delay(100);Serial2.write('4');Serial2.write('\r');
check_response();
check_response();
check_response();
}
//
void text_mode()
{ Serial.println("Text  Mode");
   response="";
  Serial2.print("AT+CMGF=1");Serial2.write('\r');
 check_response();
}
void sms_forwarding()
{ 
  
   Serial.println("Sms Forwarding");
   response="";
  Serial2.println("AT+CNMI=1,2,0,0,0");  
 
check_response();
}
//
void send_sms_location(float my_latitude,float my_longitude)
{
  
  Serial2.print("AT");Serial2.write(13);check_response();
   Serial2.print("AT");Serial2.write(13);check_response();
   Serial2.print("AT+CMGS=");Serial2.write('"');Serial2.print("03005409012");Serial2.write('"');Serial2.write(13);check_response();//code is same 
  //Serial2.print("AT+CMGS=");Serial2.write('"');Serial2.print("03335421860");Serial2.write('"');Serial2.write(13);check_response();//code is same u just need to add different simnumbers and upload to two boards anr u r good to go.
 
    Serial2.print("$");
    Serial2.print(my_latitude,6);
    Serial2.write(',');
    Serial2.print(my_longitude,6);
    Serial2.print('*');
  Serial2.write(26);
  long tc=millis();
  do
  {
  check_response();
  }while(millis()- tc<4000);
}
///
void SerialEvent2() {
  while (Serial2.available()) {
    
    char inChar1 = (char)Serial2.read();
    Serial.write(inChar1);
      if(inChar1=='*')
      {
      smsr=1;
      }
      if(inChar1=='$')
      {
    response="";
      
     }
      response+=inChar1;
    
    }
}
///////gps
void get_gps()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial3.available())
    {
      char c = Serial3.read();
      
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
 // float f_speed_mph();
 // float f_speed_mps();
 // float f_speed_kmph();
    myspeed = gps.f_speed_kmph();
  //  myspeed=1.852*myspeed;
    
    carspeed=myspeed;
   // Serial11.print("LAT=");
    //Serial11.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    //Serial11.print(" LON=");
    //Serial11.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //Serial1.print(" SAT=");
    //Serial1.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    //Serial1.print(" PREC=");
    //Serial1.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  
   // new_dataa=1;  
    g_flat=flat;
    g_flon=flon;
    updata=1;
  }
  
  gps.stats(&chars, &sentences, &failed);
  //Serial1.print(" CHARS=");
  //Serial1.print(chars);
  //Serial1.print(" SENTENCES=");
  //Serial1.print(sentences);
  //Serial1.print(" CSUM ERR=");
  //Serial1.println(failed);
}
