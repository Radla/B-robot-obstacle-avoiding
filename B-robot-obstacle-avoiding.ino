/* 
Obstacle Avoiding B-Robot EVO 
coded by radla

Hardware: 
- Board: WEMOS/LOLIN D1 MINI 
- HC-SR04 Ultrasonic Sensor https://github.com/jshaw/NewPingESP8266 
- SG90 Micro Servo
electroic components:
- a Zener Diode und a 330 Ohm Resistor for the HC-SR04: https://www.letscontrolit.com/wiki/index.php/HC-SR04#Connecting_using_a_Zener_Diode

optional: (remove the corresponding code)
- Infrared Sensor TSOP4838: move and start the autonomous mode, and a arbitrary ir-remote (i used an AppleTV3 remote)
- Sharp Infrared distance Sensor GP2Y0A21YK0F 
- OLED https://github.com/ThingPulse/esp8266-oled-ssd1306

Software:
- IDE used Arduino IDE 1.8.9  
- For OSC communication with devia board: https://github.com/CNMAT/OSC
- no need to modify the original Firmware from BRobot! 
- SG90 Micro Servo for the ultrasonic sensor is connected to PIN SERVO1 on the Devia board and controlled via OSC /1/fader3/
- micro usb from wemos board is powered by AUX +5V and Ground

Details and questions here: http://forums.jjrobots.com/showthread.php?tid=2327 

- Sonar function not used at the moment, a compass modul will be a nice option
required wiring should be obvious from the code

*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include <OSCMessage.h>
#include <WiFiUdp.h>

#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

SSD1306Wire display(0x3c, D2, D1); // ADDRESS, SDA, SCL

// INFRAROT EMPFÄNGER
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// AppleRemote
#define IR_MENU_CODE 0x77E1C04A      //Menu
#define IR_CHP_CODE 0x77E1604A  //Ch+
#define IR_CHM_CODE 0x77E1904A  //Ch-
#define IR_OK_CODE 0x77E13A4A //0x77E1A04A   //OK
#define IR_VOLP_CODE 0x77E1504A //Vol+
#define IR_VOLM_CODE 0x77E1304A //Vol-

#define IR_VCC_PIN D5

#define LED_PIN 2
// geht sonst nicht mit float
#define abs(x) ((x) > 0 ? (x) : -(x))

#define irTimerInt 300
#define throttleTimerInt 500
#define steeringTimerInt 500
// warten bis aufgerichtet
#define obstacleTimerInt 3000

unsigned long irTimer = 0;
unsigned long throttleTimer = 0;
unsigned long steeringTimer = 0;
unsigned long obstacleTimer = 0;

const uint16_t IR_REC_PIN = D6;
IRrecv irrecv(IR_REC_PIN);
decode_results results;

// INFRAROT ENDE

/* SONAR */
#include <NewPingESP8266.h>

#define DEBUG false

#define PING_PIN D7 // Arduino pin tied to both trigger and echo pins on the ultrasonic sensor.
#define ECHO_PIN D8

#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MIN_DISTANCE 30 // minimaler Abstand vom Hindernis in cm
#define MIN_DISTANCE_BOTTOM 8 // IR Sensor bottom, should be small, otherwise detection of ground

#define AREA_DEGREEES 180
int min_degrees = 90 - (AREA_DEGREEES / 2);
int max_degrees = 90 + (AREA_DEGREEES / 2);
int pos1 = 0;

int distance = 100;
boolean goesForward = false;

boolean autonomous = false;
boolean irSensorActive = false;

boolean letsDance = false;

float lastFB = 0.5;
float lastLR = 0.5;

int OledRow = 1;
int OledFontsize = 16;
int screenW = 128;
int screenH = 64;

int centerX = screenW / 2;
int centerY = screenH;
int radius = screenW / 2;

NewPingESP8266 sonar(PING_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pin and maximum distance.

/* WIFI */
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP

const IPAddress outIp(192, 168, 4, 1); // remote IP to receive OSC
const unsigned int outPort = 2222;     // remote port to receive OSC

const char *ssid = "JJROBOTS_XX";
const char *password = "87654321";

//INFRAROT DISTANCE

const int minimum_position_analog = 1024;  // Your sensor analog reading at 4 cm (typical value is 822)
const int maximum_position_analog = 330;  // Your sensor analog reading at 30 cm (typical value is 124)

// Sensor constants
const float minimum_position_meter = 0.06;  // Sensor minimum position (fixed at 4 cm) edit 6cm
const float maximum_position_meter = 0.30;  // Sensor maximum position (fixed at 30 cm)
const float sensor_offset = 0.0042;         // Sensor offset position (fixed at 0.42 cm)
const float sensor_slope = ((1.0 / (minimum_position_meter + sensor_offset)) - (1.0 / (maximum_position_meter + sensor_offset))) / (minimum_position_analog - maximum_position_analog);
const float sensor_yintercept = (1.0 / (minimum_position_meter + sensor_offset)) - (sensor_slope * minimum_position_analog);

float distanceIR = 0;

OSCErrorCode error;

void setup()
{
  Serial.begin(115200);
  display.init();
  display.flipScreenVertically();

  display.setFont(ArialMT_Plain_16);

  printOled("hello!");
  printOled("waiting 15s...");
  // wait until robot is finished (an receiving osc event would be much better!)
  if (!DEBUG) {
    delay(15000);
    Serial.println("Connecting to WIFI");
    // Connect to WiFi
    WiFi.begin(ssid, password);
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    Serial.println(WiFi.localIP());
    printOled("WIFI connected");
  }
  // infrarot distance
  pinMode(A0, INPUT);

  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  pinMode(IR_VCC_PIN, OUTPUT);
  digitalWrite(IR_VCC_PIN, HIGH);
  irrecv.enableIRIn(); // Start the receiver

  OledRow = 1;
  display.clear();
}

// Convert analog reading (0..1023) to position in meter
float convertPosition(int analogPosition) {
  // Local variables
  float realPosition = 0;
  // Too far
  if (analogPosition < maximum_position_analog) {
    realPosition = maximum_position_meter;
  }
  // Too close
  else if (analogPosition > minimum_position_analog) {
    realPosition = minimum_position_meter;
  }
  // Within sensor range
  else {
    realPosition = (1.0 / ((sensor_slope * analogPosition) + sensor_yintercept)) - sensor_offset;
  }
  return realPosition;
}
// Read sensor output analog voltage
void readIRPing() {
  int current_position_analog = (int)analogRead(A0);
  float current_position_volt = 3.3 * current_position_analog / 1023;
  distanceIR = convertPosition(current_position_analog) * 100.0;
}

int readPing()
{
  delay(10);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = MAX_DISTANCE;
  }
  return cm;
}

int lookRight()
{
  int distMax = 0;
  // sonar area
  display.drawCircle(centerX, centerY, radius);

  for (pos1 = 90; pos1 >= min_degrees; pos1 -= 10) {
    float a = pos1 / 90.0 * 0.5;
    lookAround(a);
    delay(50);
    int distance = readPing();
    drawCLine(pos1, distance);
    if (distance > distMax) {
      distMax = distance;
    }
  }
  //printOled("rightMax: " + String(distMax));
  delay(50);
  int dist = readPing();

  lookAround(.5);
  return dist;
}

int lookLeft()
{
  int distMax = 0;
  display.drawCircle(centerX, centerY, radius);

  for (pos1 = 90; pos1 <= max_degrees; pos1 += 10) {
    float a = pos1 / 90.0 * 0.5;
    lookAround(a);
    delay(50);
    int distance = readPing();
    drawCLine(pos1, distance);
    if (distance > distMax) {
      distMax = distance;
    }
  }
  //printOled("leftMax: " + String(distMax));
  delay(50);
  int dist = readPing();
  lookAround(.5);
  return dist;
}

void drawCLine(float angle, int val) {
  angle = ( angle / 57.29577951 );
  float x = val * cos(angle);
  float y = val * sin(angle);
  float xs =  x / MAX_DISTANCE * radius;
  float ys =  y / MAX_DISTANCE * radius;

  // zero bottom right
  //display.drawLine(centerX, centerY, centerX-xs, ys);
  // mirrored: zero is top left
  display.drawLine(centerX, centerY, centerX + xs, centerY - ys);
  display.display();
}

void stopTurning()
{
  steering(0.5);
}

void moveStop()
{
  throttle(0.5);
  steering(0.5);
  goesForward = false;
}
void moveForward()
{
  if (!goesForward)
  {
    goesForward = true;
    throttle(0.6);
  }
}
void moveBackward()
{
  goesForward = false;
  throttleTimer = millis();
  throttle(0.4);
}
void turnRight()
{
  steeringTimer = millis();
  steering(0.1);
  delay(steeringTimerInt);
}
void turnLeft()
{
  steeringTimer = millis();
  steering(0.9);
  delay(steeringTimerInt);
}

void lookAround(float val)
{
  String addr_prefix = "/1/fader3/";
  const char *addr = addr_prefix.c_str();
  OSCMessage msg(addr);
  msg.add((float)val);

  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void printOled(String content) {
  if (OledRow >= 4) {
    OledRow = 1;
    display.clear();
    //display.setColor(BLACK);
    //display.fillRect(0, 5, screenW, screenH);
  }
  display.setColor(WHITE);
  display.drawString(0, OledRow * OledFontsize, content);
  display.display();
  OledRow++;
}


void loop()
{
  // visualize both distances on top off the olwd
  int ds =  (float)distance / MAX_DISTANCE * screenW;
  display.setColor(BLACK);
  display.fillRect(0, 0, screenW, 10);
  display.setColor(WHITE);
  display.fillRect(0, 0, ds, 4);
  if (irSensorActive) {
    int dsb =  distanceIR / 40.0 * screenW;
    display.fillRect(0, 5, dsb, 4);
  }
  display.display();
  digitalWrite(LED_PIN, autonomous);

  if (autonomous) {
    int distanceRight = 0;
    int distanceLeft = 0;
    delay(50);
    if ((distance <= MIN_DISTANCE) || (distanceIR <= MIN_DISTANCE_BOTTOM)) {
      display.setFont(ArialMT_Plain_24);
      display.clear();
      if (distance <= MIN_DISTANCE) {
        printOled("TOP SENSOR");
      }
      if (distanceIR <= MIN_DISTANCE_BOTTOM) {
        printOled("BOTTOM Sensor");
      }
      display.setFont(ArialMT_Plain_16);
      moveStop();
      delay(300);
      moveBackward();
      delay(500);
      moveStop();
      delay(300);
      display.clear();
      distanceRight = lookRight();
      delay(300);
      distanceLeft = lookLeft();
      if (DEBUG) {
        String dist_str = String(distance);
        String left_str = String(distanceLeft);
        String right_str = String(distanceRight);
        String distIR_str = String(distanceIR);
        OledRow = 0;
        printOled(left_str + " : " + right_str + " : " + dist_str + " : " + distIR_str);
        delay(3000);
      }
      if (distance >= distanceLeft) {
        turnRight();
        moveStop();
      } else {
        turnLeft();
        moveStop();
      }
    // reset
      distance = 200;
      distanceIR = 200.0;

      irSensorActive = false;
      obstacleTimer = millis();
      //printOled("timerReset");
    } else {
      moveForward();
    }
  } // ende autonoumos
  else if (letsDance) {
    letsDance = false;
    dancing();
  }
  if (millis() > irTimerInt + irTimer) {
    irAnalyse();
    //receiveOSCMsg();
    irTimer = millis();
  }
  if (millis() > steeringTimerInt + steeringTimer) {
    //stopTurning();
  }
  if (millis() > throttleTimerInt + throttleTimer) {
    //moveStop();
  }
  if (millis() > obstacleTimerInt + obstacleTimer) {
    irSensorActive = true;
    // wait until robot is nearly horizontally
  }
  distance = readPing();
  if (irSensorActive) readIRPing();

}

void throttle(float fb)
{
  float d = abs(fb - lastFB);
  if (d > 0.01) {
    String addr_prefix = "/1/fader1/";
    const char *addr = addr_prefix.c_str();
    OSCMessage msg(addr);
    msg.add((float)fb);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
  }
  lastFB = fb;
}

void steering(float lr)
{
  float d = abs(lr - lastLR);

  if (d > 0.01) {
    //printOled("throttleTimerInt: "+String(throttleTimerInt));
    String addr_prefix = "/1/fader2/";
    const char *addr = addr_prefix.c_str();
    OSCMessage msg(addr);
    msg.add((float)lr);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
  }
  lastLR = lr;
}

void servo(float val) {
    String addr_prefix = "/1/push1/";
    const char *addr = addr_prefix.c_str();
    OSCMessage msg(addr);
    msg.add((float)val);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();    
}

void standUp() {
  OSCMessage msg("/1/push1");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void toogleProMode(float val) {
   String addr_prefix = "/1/toggle1/";
  const char *addr = addr_prefix.c_str();
  OSCMessage msg(addr);
  msg.add((float)val);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

// does not work!!!
void receiveOSCMsg() {
  OSCMessage msg;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg.fill(Udp.read());
    }
    if (!msg.hasError()) {
      printOled("osc received");
      //msg.dispatch("/1/led", led);
    } else {
      error = msg.getError();
      Serial.print("error: ");
      Serial.println(error);
      printOled("osc error");
    }
  }
}


void irAnalyse()
{
  if (irrecv.decode(&results))
  {
    serialPrintUint64(results.value, HEX);
    irrecv.resume(); // Receive the next value

    switch (results.value)
    {
      case IR_MENU_CODE:
        autonomous = !autonomous;
        if (autonomous) {
          printOled("automode ON");
          goesForward = false;
        } else {
          printOled("automode OFF");
        }
        break;

      case IR_CHP_CODE:
        printOled("->");
        turnRight();
        moveStop();
        break;

      case IR_CHM_CODE:
        printOled("<-");
        turnLeft();
        moveStop();
        break;

      case IR_OK_CODE:
        printOled("STOP");
        moveStop();
        break;

      case IR_VOLP_CODE:
        printOled("Forward");
        moveForward();
        /*
        moveStop();
        printOled("LETS DANCE!");
        letsDance = true;
        */
        break;

      case IR_VOLM_CODE:
        printOled("Backward");
        moveBackward();
        break;

      default:
        //Serial.println(results.value);
        break;
    }
  }
}



void steeringPyt(float value) {
  value = (value + 1.0) / 2.0;
  steering(value);
}
void throttlePyt(float value) {
  value = (value + 1.0) / 2.0;
  throttle(value);
}

// code from python example
void dancing() {
  //mode(1)  # PRO MODE
  toogleProMode(1);
  
  for (int i = 0; i < 2; i++) {
    servo(1);
    delay(150);
    servo(0);
    delay(150);
  }
    
  for (int i = 0; i < 3; i++) {
    steeringPyt(0.5 + i * 0.1);
    delay(250);
    steeringPyt(0);
    steeringPyt(-0.5 - i * 0.1);
    delay(250);
    steeringPyt(0);
    steeringPyt(-0.5 - i * 0.1);
    delay(250);
    steeringPyt(0);
    steeringPyt(0.5 + i * 0.1);
    delay(250);
    steeringPyt(0);
    servo(1);
    servo(0);
  }

  for (int i = 0; i < 5; i++) {
    throttlePyt(0.3);
    delay(80);
    throttlePyt(0);
    delay(80);
    throttlePyt(-0.3);
    delay(80);
    throttlePyt(0);
    servo(1);
    servo(0);
  }
  steeringPyt(1);
  delay(1000);
  steeringPyt(0);
  delay(100);
  steeringPyt(-1);
  delay(1000);
  steeringPyt(0);
  
  toogleProMode(0);
}
