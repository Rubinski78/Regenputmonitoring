#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <avr/wdt.h> // needed to use watchdog

/*
  VERSION HISTORY:
  ----------------
   V1.1   7/11/2016   Ruben Desmet    copy from v1.0
   V1.2   23/11/2016  Ruben Desmet    implementation of NewPing lib
   V1.3   18/12/2016  Ruben Desmet    opkuis + slow interval van 2h naar 1h
   V2.0   16/06/2016  Ruben Desmet    implementatie brobbelbuis
   V2.1   21/07/2016  Ruben Desmet    multiple functionalities for reset button
                                      - if low level alarm is active and push       -> reset alarm
                                      - if low level alarm is not active and push   -> perform measurement
   V3.0   14/10/2017  Ruben Desmet    implemented new sensor MPX5050GP
   V3.1   20/10/2017  Ruben Desmet    no more measurement at start of the Arduino
                                      delay between stop pump and measurement is now 2sec (was 1 sec)
   V3.2   21/10/2017  Ruben Desmet    implemented external reference voltage
   V3.3   22/10/2017  Ruben Desmet    only send the RAW ADC Value to Node-RED
                                      includes BACKLOG ITEM : [SOFTWARE]            remove the % level calculation from the Arduino to Node-RED
   V3.3.1 24/10/2017  Ruben Desmet    sourcecode opgekuist en TODO's bijgezet
   V4.0   25/10/2017  Ruben Desmet    decommissioning of the LCD display
                                      includes BACKLOG ITEM : [SOFTWARE][HARDWARE]  branch a new version without display (progress shown with LEDs)
   V4.1   4/11/2017   Ruben Desmet    removed "measure_b"
                                      includes BACKLOG ITEM :  [HARDWARE]            implement a small relais with transistor circuit to recuperate the 4 relay board
                                      although not transistor has been used since powering the coil only takes 3,6mA
   V5.0   4/11/2017   Ruben Desmet    decommissioning of the <ethercard.h> lib in favour for the <Ethernet.h> lib
                                      includes BACKLOG ITEM : [SOFTWARE][HARDWARE]  implement a version with a Arduino UNO by using the Ethershield.h library
                                      includes BACKLOG ITEM : [SOFTWARE]            distance and level calculations are now done in the Arduino and in Node-Red
   V5.1   16/11/2017  Ruben Desmet    remove delay statements to prepare for the Watchdog mechanisme
   V5.2   27/11/2017  Ruben Desmet    includes BACKLOG ITEM : [SOFTWARE]            reimplement heartBeatStatusId, is not set any more
   V5.2.1 12/12/2017  Ruben Desmet    implemented extra logging
   V5.3   17/12/2017  Ruben Desmet    includes BACKLOG ITEM : [SOFTWARE]            implement WATCHDOG (mail Lieven Dossche 19/10/2017)
   V6.0   14/01/2017  Ruben Desmet    commissioning of led array to indicate the level

  BACKLOG:
  --------
    - [SOFTWARE]            remove all masterdata from Arduino into Node-RED

  CANCELLED:
  ---------
    - [SOFTWARE]            It would be better that Node-Red is the master and the display in the garage shows the data
                            that is calculated in Node-Red so that the Arduino doesn't need to calculate any more.
*/

const static byte udpServer[] = {192, 168, 1, 130};
const int udpServerPort = 8888;

const static byte myMac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
//the IP address for the shield:
const static byte myIp[] = {192, 168, 1, 3};

const unsigned int localPort = 8890; // local port to listen on
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet

EthernetUDP Udp;

const bool communicate = true;
const bool buzzerOnAlarm = true;

// program variables
bool measurementAskedFromServer = false;
bool measurementAskedFromButton = false;

//constants for pump
byte PUMP_RELAIS_PIN = 4;
//TODO : masterdata in Node-RED
long pump_delay = 120000; //ms
bool measurementBusy = false;
long measurementStarted = 0; // time the last measurement was started
long loopLogMsgSent = 0; //time the last loopLogMsg has been sent

// constants/variables for LEDs
const byte externalLedPin = 13; // the number of the external pin (=D4)

//constants for buttons
const byte debounceDelay = 50; // the debounce time; increase if the output flickers

// constants for cistern T1
//TODO : masterdata in Node-RED
const int t1_capacity_with_manhole = 20000; //liters : toch manhole niet meegerekend

// alarm light and reset button
const byte alarmLightPin = 8;
const byte alarmResetPin = 6;

//TODO : masterdata in Node-RED
const byte lowAlarmLimit = 25; //if tank reaches <lowAlarmLimit% then give alarm
//TODO : masterdata in Node-RED
//const byte alarmLightInterval = 1000;
//TODO : masterdata in Node-RED
const unsigned long alarmDelay = 3600000; //1h
unsigned long controllerStarted = 0;

boolean alarmLightIsActive = false;
unsigned long previousMillisAlarm = 0;
boolean alarmIsActive = false;
unsigned long lastMillisAlarmAcked = 0;
long lastResetButtonDebounceTime = 0; // the last time the output pin was toggled
byte resetButtonState = LOW;
byte lastResetButtonState = LOW;
unsigned long lastTimeBuzzer = 0;

String paramsStr;
char msg[40];

//const int listenPort = 8890;            // local port to listen on
long lastHeartbeatReceivedOn = 0; // the last time the heartbeat was received from de UDPDataLogger
//TODO : masterdata in Node-RED
const long heartBeatDelay = 360000; //6min

byte heartBeatStatusId = 2;
//0 = no heartbeat
//1 = heartbeat OK
//2 = waiting for heartbeat
//-1 = disabled

//buzzer
const byte buzzerPin = 7;
unsigned long lastPeriodStart;
//TODO : masterdata in Node-RED
const int onDuration = 1000;
const int periodDuration = 3000;

int16_t rawADCvalue; // The is where we store the value we receive from the ADS1115

//for watchdog
int i;

//LED array
//Pin connected to ST_CP of 74HC595
int latchPin = 3;     //groen
//Pin connected to SH_CP of 74HC595
int clockPin = 5;  //geel
////Pin connected to DS of 74HC595
int dataPin = 2;    //wit

byte dataRED;
byte dataGREEN;
byte dataArray1[11];
byte dataArray2[11];
byte percentArray[] = {3,105,73,77,32,203,19,20,21,24,25,26};

void setup()
{
  Serial.begin(9600);

  Serial.println("SETUP START");

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  //Arduino doesn't seem to have a way to write binary straight into the code 
  //so these values are in HEX.  Decimal would have been fine, too. 
  dataArray1[0] = 0xFF; //11111111
  dataArray1[1] = 0xFE; //11111110
  dataArray1[2] = 0xFC; //11111100
  dataArray1[3] = 0xF8; //11111000
  dataArray1[4] = 0xF0; //11110000
  dataArray1[5] = 0xE0; //11100000
  dataArray1[6] = 0xC0; //11000000
  dataArray1[7] = 0x80; //10000000
  dataArray1[8] = 0x00; //00000000
  dataArray1[9] = 0x00; //00000000
  dataArray1[10] = 0x00; //00000000

  //Arduino doesn't seem to have a way to write binary straight into the code 
  //so these values are in HEX.  Decimal would have been fine, too. 
  dataArray2[0] = 0xFF; //11111111
  dataArray2[1] = 0x7F; //01111111
  dataArray2[2] = 0x3F; //00111111
  dataArray2[3] = 0x1F; //00011111
  dataArray2[4] = 0x0F; //00001111
  dataArray2[5] = 0x07; //00000111
  dataArray2[6] = 0x03; //00000011
  dataArray2[7] = 0x03; //00000001
  dataArray2[8] = 0x03; //00000000
  dataArray2[9] = 0x02; //00000000
  dataArray2[10] = 0x00; //00000000

  SetLEDLevel(0);

  controllerStarted = millis();

  measurementAskedFromServer = false;
  measurementAskedFromButton = false;

  //this is code to disable the SD card, there is stated that you should do this if you do not use an sd card in the ethernet shield
  //but if i execute this code (in test environment) then i get an irregular PING
  // // disable SD card
  //   pinMode(4, OUTPUT);
  //   digitalWrite(4, HIGH);

  pinMode(A1, INPUT);
  analogReference(EXTERNAL); // use AREF for reference voltage

  //do some analog measurements to calibrate
  analogRead(A1);
  analogRead(A1);
  analogRead(A1);

  pinMode(externalLedPin, OUTPUT);

  digitalWrite(PUMP_RELAIS_PIN, HIGH);
  pinMode(PUMP_RELAIS_PIN, OUTPUT);

  pinMode(alarmLightPin, OUTPUT);
  pinMode(alarmResetPin, INPUT);

  if (communicate)
  {
    heartBeatStatusId = 2; //2 = waiting for heartbeat
    // start the Ethernet and UDP:
    //Ethernet.begin(myMac);
    Serial.println("Ethernet.begin");
    Ethernet.begin(myMac, myIp);
    Udp.begin(localPort);

    WriteToLog("warn", "Ethernet interface started");
  }
  else
  {
    heartBeatStatusId = -1; //-1 = disabled
  }

  ActivateAlarmLight();

  //apparantly this is needed else after the DHCP setup the measurementAskedFromServer variable gets 49, not known why
  measurementAskedFromServer = false;
  measurementAskedFromButton = false;

  DeactivateAlarmLight(true);

  // enable the watchdog
  // possible values: WDTO_xxxx whith xxxx=15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
  // 4 and 8 s not with all devices
  wdt_enable(WDTO_8S); // enable watchdog; timeout = 8s
  i = 0; // put in comment to check if reboot clears this value

  blinkAll_2Bytes(2,500); 

  Serial.println("SETUP COMPLETE");
  WriteToLog("warn", "SETUP COMPLETE");
}

void AlarmReset()
{
  //Serial.println("ALARM RESET DETECTED");
  alarmIsActive = false;
  lastMillisAlarmAcked = millis();

  tone(buzzerPin, 750, 100);
  delay(100);
  tone(buzzerPin, 1000, 100);
  delay(100);
  tone(buzzerPin, 1250, 100);

  WriteToLog("info", "ALARM has been RESET"); 
}

void StartMeasurement()
{
  WriteToLog("info", "start of measurement"); 
  WriteToLog("silly", "Activating pump");
  //Serial.println("Activating pump");
  digitalWrite(PUMP_RELAIS_PIN, LOW);
  measurementBusy = true;

  measurementStarted = millis();
}

double StopMeasurement(double &volume)
{
  //Serial.println("Deactivating pump");
  WriteToLog("silly", "Deactivating pump");
  digitalWrite(PUMP_RELAIS_PIN, HIGH);
  measurementBusy = false;

  delay(2000);

  rawADCvalue = analogRead(A1);
  WriteToLog("silly", String("rawADCvalue = " + String(rawADCvalue)).c_str());

  //rawADCvalue = (rawADCvalue - offset);

  //5500 = 5.5V = referentievoltage gemeten op huidig gebruikte Arduino
  //4700 = 4.7V = voltage bij full scale van sensor
  double f1 = (5500.0 / 1023) * (50.0 / 4700) * (1000.0 / (9.81 * 999.58)) + 0.0; //zonder temperatuurcompensatie want zit eigenlijk al in de sensor
  double mm = rawADCvalue * f1 * 1000;
  WriteToLog("silly", String("mm = " +  String(mm)).c_str());                                            //mm

  //2* omdat er 2 putten zijn die met een hevel verbonden zijn
  volume = 2 * (rawADCvalue * f1) * ((3.1415 * 2.35 * 2.35) / 4) * 1000;
  WriteToLog("silly", String("volume = " + String(volume)).c_str());    

  return mm;
}

void ActivateAlarmLight()
{
  digitalWrite(alarmLightPin, HIGH);
  alarmLightIsActive = true;
  WriteToLog("info", "Activated ALARM"); 
}

void DeactivateAlarmLight(bool log)
{
  digitalWrite(alarmLightPin, LOW);
  alarmLightIsActive = false;
  if (log) WriteToLog("info", "Deactivated ALARM"); 
}

void WriteToLog(char level[], char msg[60])
{
  if (communicate)
  {
    char logmsg[70];
    sprintf(logmsg, "%s;%s", level, msg);
    WriteToUDPServer(logmsg);
  }
}

void WriteToUDPServer(char msg[40])
{
  if (communicate)
  {
    Udp.beginPacket(udpServer, udpServerPort);
    Udp.write(msg);
    Udp.endPacket();
  }
}

void loop()
{

  if ((millis() - loopLogMsgSent) > 30000) //send each 30 sec 
  {
    char loopLogMsg[70];

    //Unsigned long variables are extended size variables for number storage, and store 32 bits (4 bytes). 
    //Unlike standard longs unsigned longs won’t store negative numbers, making their range from 0 to 4,294,967,295 (2^32 - 1).
    sprintf(loopLogMsg, "Start of loop : measurementBusy=%s;millis()=%lu", measurementBusy?"true":"false",millis());
    WriteToLog("silly", loopLogMsg);
    loopLogMsgSent = millis();
  }

  if (communicate)
  {
    bool heartBeatCheckArmed = false;
    if ((millis() - controllerStarted) > heartBeatDelay)
      heartBeatCheckArmed = true;

    if (heartBeatCheckArmed)
    {
      if ((millis() - lastHeartbeatReceivedOn) < heartBeatDelay)
        heartBeatStatusId = 1; //heartbeat is OK
      else
        heartBeatStatusId = 0; //heartbeat is NOT OK
    }

    int packetSize = Udp.parsePacket();

    //Serial.print("packetsize = ");
    //Serial.println(packetSize);

    if (Udp.available())
    {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remote = Udp.remoteIP();
      for (int i = 0; i < 4; i++)
      {
        Serial.print(remote[i], DEC);
        if (i < 3)
        {
          Serial.print(".");
        }
      }
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      Serial.print("Contents:");
      Serial.println(packetBuffer);

      String payload(packetBuffer);
      //Serial.println(payload);

      for (int i = 0; i < UDP_TX_PACKET_MAX_SIZE; i++)
        packetBuffer[i] = 0;

      WriteToLog("info", String("UDP message received : '" + payload + "'").c_str());

      if (payload == "heartbeat")
      {
        lastHeartbeatReceivedOn = millis();
        WriteToLog("silly", "'heartbeat' received");
      }
      else if (payload == "measure")
      {
        if (!measurementBusy)
        {
          measurementAskedFromServer = true;
          Serial.println("measure received");
        }
      }
      else if (payload.startsWith("setleds"))
      {
        String levelPart = getValue(payload,';',1);
        //set the LEDS
        SetLEDLevel(levelPart.toInt());
      }
      else
      {
        Serial.println(payload);
      }
    }
  }

  if (!measurementBusy && (measurementAskedFromServer || measurementAskedFromButton))
  {
    StartMeasurement();
  }

  if (measurementBusy && (millis() - measurementStarted) > pump_delay)
  {
    //Serial.print("measurementAskedFromServer = ");
    //Serial.println(measurementAskedFromServer);
    //Serial.print("measurementAskedFromButton = ");
    //Serial.println(measurementAskedFromButton);

    digitalWrite(externalLedPin, HIGH);

    //double heightWater;
    double volume;
    StopMeasurement(volume);

    //double t1_percentage_full = ((volume / (t1_capacity_with_manhole * 1.0)) * 100);
    //WriteToLog("silly", String("t1_percentage_full = " +  String(t1_percentage_full)).c_str());   
    
    //byte t1_percentage_full_rounded = (byte)(t1_percentage_full + 0.5);
    //WriteToLog("silly", String("t1_percentage_full_rounded = " + String(t1_percentage_full_rounded)).c_str());  

    //Check if we have to raise the alarm
    //alarmIsActive = (t1_percentage_full_rounded < lowAlarmLimit) and (millis() - lastMillisAlarmAcked > alarmDelay or lastMillisAlarmAcked == 0);
    //WriteToLog("silly", "alarmIsActive = " + alarmIsActive);  

    //Send the measurement data to the server (for database storage)
    if (communicate)
    {
      //t=1&adc=425&hb=1
      paramsStr = "MEAS;";
      paramsStr = F("t=");
      paramsStr = paramsStr + F("1");
      paramsStr = paramsStr + F("&adc=");
      paramsStr = paramsStr + rawADCvalue;
      paramsStr = paramsStr + F("&hb=");
      paramsStr = paramsStr + heartBeatStatusId;
      strcpy(msg, paramsStr.c_str());

      //ether.sendUdp(msg, sizeof msg, listenPort, UDPDataCollectorIp, 8888);
      // send a reply to the IP address and port that sent us the packet we received
      // Udp.beginPacket(udpServer, udpServerPort);
      // Udp.write(msg);
      // Udp.endPacket();

      WriteToUDPServer(msg);
    }

    measurementAskedFromButton = false;
    measurementAskedFromServer = false;

    digitalWrite(externalLedPin, LOW);

    WriteToLog("info", "end of measurement"); 
  }

  //Check if the RESET button has been pressed
  byte resetButtonReading;
  resetButtonReading = digitalRead(alarmResetPin);

  if (resetButtonReading != lastResetButtonState)
    lastResetButtonDebounceTime = millis();

  if ((millis() - lastResetButtonDebounceTime) > debounceDelay)
  {
    if (resetButtonReading != resetButtonState)
    {
      resetButtonState = resetButtonReading;
      //ack alarm
      if (resetButtonState == HIGH)
      {
        if (alarmIsActive)
          AlarmReset();
        else
          measurementAskedFromButton = true;
      }
    }
  }
  lastResetButtonState = resetButtonReading;

  //Set the alarm light and alarm buzzer
  if (alarmIsActive)
  {
    //Play a tone on the buzzer and blink the red alarm light
    if (millis() - lastPeriodStart >= periodDuration)
    {
      lastPeriodStart += periodDuration;
      if (buzzerOnAlarm)
        tone(buzzerPin, 550, onDuration); // play 550 Hz tone in background for 'onDuration'ms
      lastTimeBuzzer = millis();
      ActivateAlarmLight();
    }
    else if (millis() - lastTimeBuzzer >= onDuration)
      DeactivateAlarmLight(true);
  }
  else
    DeactivateAlarmLight(false);

  delay(500);

  //last instruction of loop: reset the watcho timer; make sure this code is executed regulary
  wdt_reset(); // put in comment to check the reboot when timeout has elapsed
  //WriteToLog("silly", "WATCHDOG RESET");
}

void SetLEDLevel(byte percent)
{
  Serial.println(percent);
    if (percent > 100) percent = 100;
  
    byte level = (byte)((percent / 10.0) + 0.5);
    Serial.println(level);
    byte levelToSet = 10 - level;
    Serial.println(levelToSet);
    Serial.println("----");
    //load the light sequence you want from array
    dataRED = dataArray1[levelToSet];
    dataGREEN = dataArray2[levelToSet];
    //ground latchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, 0);
    //move 'em out
    shiftOut(dataPin, clockPin, dataGREEN);   
    shiftOut(dataPin, clockPin, dataRED);
    //return the latch pin high to signal chip that it 
    //no longer needs to listen for information
    digitalWrite(latchPin, 1);
    delay(100);
}

// the heart of the program
void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first, 
  //on the rising edge of the clock,
  //clock idles low

  //internal function setup
  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);

  //clear everything out just in case to
  //prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);

  //for each bit in the byte myDataOut
  //NOTICE THAT WE ARE COUNTING DOWN in our for loop
  //This means that %00000001 or "1" will go through such
  //that it will be pin Q0 that lights. 
  for (i=7; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    //if the value passed to myDataOut and a bitmask result 
    // true then... so if we are at i=6 and our value is
    // %11010100 it would the code compares it to %01000000 
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
      Serial.println("1");
    }
    else {  
      pinState= 0;
      Serial.println("0");
    }

    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myDataPin, pinState);
    //register shifts bits on upstroke of clock pin  
    digitalWrite(myClockPin, 1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(myDataPin, 0);
  }

  //stop shifting
  digitalWrite(myClockPin, 0);
}

//blinks the whole register based on the number of times you want to 
//blink "n" and the pause between them "d"
//starts with a moment of darkness to make sure the first blink
//has its full visual effect.
void blinkAll_2Bytes(int n, int d) {
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, 0);
  shiftOut(dataPin, clockPin, 0);
  digitalWrite(latchPin, 1);
  delay(200);
  for (int x = 0; x < n; x++) {
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 255);
    shiftOut(dataPin, clockPin, 255);
    digitalWrite(latchPin, 1);
    delay(d);
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 0);
    shiftOut(dataPin, clockPin, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}