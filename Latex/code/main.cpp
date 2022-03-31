#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_GPS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <iostream>
#include <Tone32.h>

//Set Update Rate to 1Hz
#define PMTK_SET_NMEA_UPDATE_1sek  "$PMTK220,1000*1F<CR><LF>" 

//Buzzer
#define BUZZER_PIN 13
#define BUZZER_CHANNEL 0

//Status LED
int gpsled = 27; 
int greenled = 14; 
int yellowled = 12; 
int redled = 33;  

//UART Ports
HardwareSerial GPSSerial( 1 );
HardwareSerial TFMiniSerial( 2 );

//GPS
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();

//Network credentials 
const char* ssid     = "iPhone von Luca";
const char* password = "08154711";

//Temp
int temppin = 32; 

//Multitasking
unsigned long GPSsendmillis; 
unsigned long Tempmillis; 

//Decode Degress
float ToDecimalDegrees(float formattedLatLon)
{
  float decDegrees = (float)((int)formattedLatLon / 100);
  float decMinutes = formattedLatLon - (decDegrees * 100);
  float fractDegrees = decMinutes / 60.0;
   return decDegrees + fractDegrees;
}

//Parse LIDAR Data
void getTFminiData(int* distance, int* strength) {
  static char i = 0;
  char j = 0;
  int checksum = 0; 
  static int rx[9];
  if(TFMiniSerial.available()) {  
    rx[i] = TFMiniSerial.read();
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    } else {
      i++;
    }}}
//==================================================================
//                              SETUP
//==================================================================

void setup(){

//Serial Monitor
Serial.begin(115200);
 
//AD GPS TX Rot 25 RX Gelb 26
GPSSerial.begin(9600, SERIAL_8N1, 26, 25);
TFMiniSerial.begin(115200, SERIAL_8N1, 34, 35);

GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
GPS.sendCommand(PGCMD_ANTENNA);
delay(1000);

//Firmware Version Ausgeben 
GPSSerial.println(PMTK_Q_RELEASE);

//Connect to network
WiFi.begin(ssid, password); 
Serial.println("Connecting");
while(WiFi.status() != WL_CONNECTED) 
{ 
  delay(500);
  Serial.print("."); 
}
Serial.println("");
Serial.print("Connected to WiFi network with IP Address: ");
Serial.println(WiFi.localIP());

//Multitasking
GPSsendmillis = millis(); 
Tempmillis = millis(); 

//Status LED
pinMode (gpsled, OUTPUT);
pinMode (greenled, OUTPUT); 
pinMode (yellowled, OUTPUT); 
pinMode (redled, OUTPUT);

//Temp
pinMode(temppin, INPUT); 

//Start Buzzer
tone(BUZZER_PIN, NOTE_C4, 500, BUZZER_CHANNEL); 
delay(1000); 
noTone(BUZZER_PIN, BUZZER_CHANNEL);

//Start Sequence
digitalWrite(gpsled, HIGH); 
delay(500);
digitalWrite(gpsled, LOW); 
digitalWrite(redled, HIGH); 
delay(500); 
digitalWrite(redled, LOW); 
digitalWrite(yellowled, HIGH); 
delay(500);
digitalWrite(yellowled, LOW); 
digitalWrite(greenled, HIGH); 
delay(500);
digitalWrite(greenled, LOW); 

}

//==================================================================
//                              LOOP
//==================================================================

void loop(){
    //LIDARDATA
      int distance = 0;
      int strength = 0;
      getTFminiData(&distance, &strength);
        while(!distance) {
          getTFminiData(&distance, &strength);
            if(distance) {
              if (distance <= 150){
                  digitalWrite(redled, HIGH); 
                  digitalWrite(yellowled, LOW); 
                  digitalWrite(greenled, LOW); 
                  tone(BUZZER_PIN, NOTE_C4, 5, BUZZER_CHANNEL);
              }
              else if (distance > 150 && distance <= 250){
                  digitalWrite(redled, LOW); 
                  digitalWrite(yellowled, HIGH); 
                  digitalWrite(greenled, LOW);
                  noTone(BUZZER_PIN, BUZZER_CHANNEL); 
              }
              else if (distance > 250){
                  digitalWrite(redled, LOW); 
                  digitalWrite(yellowled, LOW); 
                  digitalWrite(greenled, HIGH);
                  noTone(BUZZER_PIN, BUZZER_CHANNEL);  
              }
            }
        }

    //TempData
    if (( millis()- Tempmillis) >= 2000){
    int rawvoltage= analogRead(temppin);
    float millivolts= (rawvoltage/3595.0) * 5000;
    float celsius= millivolts/10;
    Serial.print(celsius);
    Serial.print(" degrees Celsius, ");
    Serial.print((celsius * 9)/5 + 32);
    Serial.println(" degrees Fahrenheit");
    Serial.print(distance);
    Serial.print("cm\t");
    Serial.print("strength: ");
    Serial.println(strength);
    Tempmillis = millis(); 
    }
    //GPSData 
      char c = GPS.read();
      if (GPSECHO)
      if (c) Serial.print(c);
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) 
          return;}
      if (millis() - timer > 2000) {
        timer = millis(); 
        Serial.print("\nTime: ");
        if (GPS.hour < 10) { Serial.print('0'); }
        Serial.print(GPS.hour, DEC); Serial.print(':');
        if (GPS.minute < 10) { Serial.print('0'); }
        Serial.print(GPS.minute, DEC); Serial.print(':');
        if (GPS.seconds < 10) { Serial.print('0'); }
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        if (GPS.milliseconds < 10) {
          Serial.print("00");} 
        else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
          Serial.print("0");
        }
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        if ((int)GPS.fix == 0){
          digitalWrite(gpsled, HIGH); 
        }
        else {
          digitalWrite(gpsled, LOW); 
        }
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);}
          
    //Send GPS Data to Server 
    if (( millis()- GPSsendmillis) >= 10000) {
      //Send Data to Raspi
      if(WiFi.status()== WL_CONNECTED) {
        //Http Header 
        HTTPClient http;
        Serial.println("");
        Serial.println("SEND:");
        String GPSData = String("&Zeit=" + String(GPS.hour) + ":" + GPS.minute + ":" + GPS.seconds 
        + "&Datum=" + GPS.day + ":" + GPS.month + ":" + GPS.year
        + "&Position=" + ToDecimalDegrees(GPS.latitude) + GPS.lat + " " + ToDecimalDegrees(GPS.longitude) + GPS.lon);
        Serial.println(GPSData);
        http.begin("http://3dprinting.dnshome.de:83/gps.php?" + GPSData); 
      GPSsendmillis = millis();}}}