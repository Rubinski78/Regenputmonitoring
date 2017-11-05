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

  BACKLOG:
  --------
    - [SOFTWARE]            implement WATCHDOG (mail Lieven Dossche 19/10/2017)
    - [SOFTWARE]            remove all masterdata from Arduino into Node-RED
    - [SOFTWARE]            distance and level calculations are now done in the Arduino and in Node-Red
                            It would be better that Node-Red is the master and the display in the garage shows the data
                            that is calculated in Node-Red so that the Arduino doesn't need to calculate any more.
*/
#include <Ethernet.h>
#include <SPI.h>
#include <EthernetUdp.h> // UDP library from: bjoern@cs.stanford.edu 12/30/2008


IPAddress ip(192, 168, 1, 132);
static byte myMac[] = { 0x74, 0x54, 0x69, 0x2D, 0x30, 0x31 };
unsigned int localPort = 8888; // local port to listen on
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
long pump_delay = 120000;      //ms

// constants/variables for LEDs
const byte externalLedPin = 13;                         // the number of the external pin (=D4)

//constants for buttons
const byte debounceDelay = 50;    // the debounce time; increase if the output flickers

// constants for cistern T1
//TODO : masterdata in Node-RED
const int t1_capacity_with_manhole = 20000;  //liters : toch manhole niet meegerekend

// alarm light and reset button
const byte alarmLightPin = 8;
const byte alarmResetPin = 6;

//TODO : masterdata in Node-RED
const byte lowAlarmLimit = 25;                //if tank reaches <lowAlarmLimit% then give alarm
//TODO : masterdata in Node-RED
//const byte alarmLightInterval = 1000;
//TODO : masterdata in Node-RED
const unsigned long alarmDelay = 3600000;       //1h

boolean alarmLightIsActive = false;
unsigned long previousMillisAlarm = 0;
boolean alarmIsActive = false;
unsigned long lastMillisAlarmAcked = 0;
long lastResetButtonDebounceTime = 0;  // the last time the output pin was toggled
byte resetButtonState = LOW;
byte lastResetButtonState = LOW;
unsigned long lastTimeBuzzer = 0;

//byte UDPDataCollectorIp[] = {192, 168, 1, 130};                     //raspberrypi : this machine gets a fixed adres from the router
//byte Ethernet::buffer[300];

String paramsStr;
char msg[40];

const int listenPort = 8890;            // local port to listen on
long lastHeartbeatReceivedOn = 0;       // the last time the heartbeat was received from de UDPDataLogger
//TODO : masterdata in Node-RED
const long heartBeatDelay = 360000;      //6min

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

int16_t rawADCvalue;  // The is where we store the value we receive from the ADS1115

void setup() {
  Serial.begin (9600);

  // start the Ethernet and UDP:
  Ethernet.begin(myMac, ip);
  Udp.begin(localPort);

  measurementAskedFromServer = false;
  measurementAskedFromButton = false;

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

  ActivateAlarmLight();

   if (communicate)
  {
    heartBeatStatusId = 2;   //2 = waiting for heartbeat

//TODO : ethershield initialisation code

  }
  else
  {
    heartBeatStatusId = -1;   //-1 = disabled
  }

  //apparantly this is needed else after the DHCP setup the measurementAskedFromServer variable gets 49, not known why
  measurementAskedFromServer = false;
  measurementAskedFromButton = false;

  DeactivateAlarmLight();
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
}

double GetHeightWater(double& volume)
{
    //TODO : masterdata in Node-Red
    //int offset = 30;

    //Serial.println("Activating pump");
    digitalWrite(PUMP_RELAIS_PIN, LOW);

    delay(pump_delay);

    //Serial.print("value from ADC : ");
    //Serial.println(rawADCvalue);

    //Serial.println("Deactivating pump");
    digitalWrite(PUMP_RELAIS_PIN, HIGH);

    delay(2000);

    rawADCvalue = analogRead(A1);

   //rawADCvalue = (rawADCvalue - offset);

   //5500 = 5.5V = referentievoltage gemeten op huidig gebruikte Arduino
   //4700 = 4.7V = voltage bij full scale van sensor
    double f1 = (5500.0 / 1023) * (50.0 / 4700) * (1000.0 / (9.81*999.58)) + 0.0;   //zonder temperatuurcompensatie want zit eigenlijk al in de sensor
    double mm = rawADCvalue * f1 * 1000;   //mm

    //2* omdat er 2 putten zijn die met een hevel verbonden zijn
    volume = 2 * (rawADCvalue * f1) * ((3.1415 * 2.35 * 2.35) / 4) * 1000;

    return mm;
}

// void udpSerialPrint(word port, byte ip[4], word unknown, const char *data, word len) {
//   String payload(data);
//
//   //Serial.println(measurementAskedFromServer);
//   //Serial.println(payload);
//
//   if (payload == "heartbeat")
//   {
//     lastHeartbeatReceivedOn = millis();
//   }
//   else if (payload == "measure")
//   {
//     lastHeartbeatReceivedOn = millis();
//     measurementAskedFromServer = true;
//     Serial.print("measure received");
//   }
//   else
//   {
//     Serial.println(payload);
//   }
// }

void ActivateAlarmLight()
{
  digitalWrite(alarmLightPin, LOW);
  alarmLightIsActive = true;
}

void DeactivateAlarmLight()
{
  digitalWrite(alarmLightPin, HIGH);
  alarmLightIsActive = false;
}

void loop() {
  if (communicate)
  {
    //while (ether.clientWaitingGw())
    //ether.packetLoop(ether.packetReceive());

    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remote = Udp.remoteIP();
      for (int i = 0; i < 4; i++) {
        Serial.print(remote[i], DEC);
        if (i < 3) {
          Serial.print(".");
        }
      }
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      Serial.println("Contents:");
      Serial.println(packetBuffer);

      String payload(packetBuffer);
      Serial.println(payload);

      if (payload == "heartbeat")
      {
        lastHeartbeatReceivedOn = millis();
      }
      else if (payload == "measure")
      {
        lastHeartbeatReceivedOn = millis();
        measurementAskedFromServer = true;
        Serial.print("measure received");
      }
      else
      {
        Serial.println(payload);
      }
    }

  }

  unsigned long currentMillis = millis();
  if (measurementAskedFromServer || measurementAskedFromButton) {

    //Serial.print("measurementAskedFromServer = ");
    //Serial.println(measurementAskedFromServer);
    //Serial.print("measurementAskedFromButton = ");
    //Serial.println(measurementAskedFromButton);

    digitalWrite(externalLedPin, HIGH);

    //double heightWater;
    double volume;
    //heightWater = GetHeightWater(volume);
    GetHeightWater(volume);

    //Serial.println(heightWater);

    double t1_percentage_full = ((volume / (t1_capacity_with_manhole * 1.0)) * 100);
    byte t1_percentage_full_rounded = (byte) (t1_percentage_full + 0.5);

    //Check if we have to raise the alarm
    alarmIsActive = (t1_percentage_full_rounded < lowAlarmLimit) and (currentMillis - lastMillisAlarmAcked > alarmDelay or lastMillisAlarmAcked == 0);

    //Send the measurement data to the server (for database storage)
    if (communicate)
    {
      //t=1&adc=425&hb=1
      paramsStr = "";
      paramsStr = F("t=");
      paramsStr = paramsStr + F("1");
      paramsStr = paramsStr + F("&adc=");
      paramsStr = paramsStr + rawADCvalue;
      paramsStr = paramsStr + F("&hb=");
      paramsStr = paramsStr + heartBeatStatusId;
      strcpy(msg, paramsStr.c_str());

      //ether.sendUdp(msg, sizeof msg, listenPort, UDPDataCollectorIp, 8888);
      // send a reply to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(msg);
      Udp.endPacket();
    }

    measurementAskedFromButton = false;
    measurementAskedFromServer = false;

    digitalWrite(externalLedPin, LOW);
  }

  //Check if the RESET button has been pressed
  byte resetButtonReading;
  resetButtonReading = digitalRead(alarmResetPin);

  if (resetButtonReading != lastResetButtonState) lastResetButtonDebounceTime = millis();

  if ((millis() - lastResetButtonDebounceTime) > debounceDelay)
  {
    if (resetButtonReading != resetButtonState)
    {
      resetButtonState = resetButtonReading;
      //ack alarm
      if (resetButtonState == HIGH) {
        if (alarmIsActive) AlarmReset();
        else measurementAskedFromButton = true;
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
      if (buzzerOnAlarm) tone(buzzerPin, 550, onDuration); // play 550 Hz tone in background for 'onDuration'ms
      lastTimeBuzzer = millis();
      ActivateAlarmLight();
    }
    else if (millis() - lastTimeBuzzer >= onDuration) DeactivateAlarmLight();
  }
  else DeactivateAlarmLight();

  delay(20);
}
