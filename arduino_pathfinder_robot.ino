/*
 * Arduino Pathfinder Robot
 * The bot  follows a black line using three QTI infrared sensors. At each of
 * 5 hash marks it stops and uses a PING))) ultrasonic sensor to check for an
 * object. After the last hash mark it reads data from 4 other teams' robots
 * over XBee, chooses a path, broadcasts its choice, shows the results on an
 * LCD, and finishes with a celebration routine.
 *
 * Hardware: Arduino Mega, 2 continuous-rotation servos, 3 QTI sensors,
 * PING))) sensor, XBee (Serial2), serial LCD, onboard + external RGB LEDs.
 */

#include <Servo.h>
#include <SoftwareSerial.h>


// Pins and constants
// Serial LCD: transmit only. RX is set to 255 (no pin) because we never read
// from the LCD.
#define TxPin 14
SoftwareSerial lcdSerial = SoftwareSerial(255, TxPin);

// QTI line sensors
#define lineSensor1 47  // left
#define lineSensor2 51  // middle
#define lineSensor3 53  // right

// QTI readings below this decay time (microseconds) mean a light surface;
// readings at or above it mean a dark surface (the line).
const int QTI_DARK_THRESHOLD = 250;

// Onboard RGB LED (common anode: logic is inverted, 0 = fully on, 255 = off)
#define redpin 45
#define greenpin 46
#define bluepin 44

// External RGB LED (common cathode: HIGH / 255 = on, LOW / 0 = off)
// Green = no object at this hash mark, red = object detected.
const int redpin2 = 5;
const int greenpin2 = 6;
const int bluepin2 = 7;

// PING))) ultrasonic sensor
const int pingPin = 50;
const int OBJECT_DISTANCE_CM = 4;  // anything this close counts as an object
long duration, cm;                 // echo time (microseconds) and distance (cm)

// Continuous-rotation servos: 1500 us = stop
//   left wheel:  > 1500 = forward, < 1500 = backward
//   right wheel: < 1500 = forward, > 1500 = backward
Servo servoLeft;
Servo servoRight;

// Song played on the LCD speaker: durations, octaves, and notes
#define NUM_NOTES 33
int durs[NUM_NOTES]  = {210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 209, 209
};
int octs[NUM_NOTES]  = {217, 216, 216, 216, 217, 216, 216, 216, 217, 216, 216, 216, 216, 216, 216, 216, 216, 215, 215, 215, 216, 215, 215, 215, 216, 215, 215, 215, 215, 215, 215, 215, 215
};
int notes[NUM_NOTES] = {220, 223, 228, 223, 220, 223, 228, 223, 220, 223, 228, 223, 221, 224, 228, 230, 220, 223, 228, 223, 220, 223, 228, 223, 220, 223, 228, 223, 221, 224, 228, 228, 228
};

// ---------------------------------------------------------------------------
// Path and team data
// ---------------------------------------------------------------------------

const int NUM_PATHS = 5;
const int NUM_TEAMS = 4;
const int NO_DATA = -1;

// blocked[i] is true if an object was detected at hash mark i + 1.
// Starts as true so an unchecked path is never chosen.
bool blocked[NUM_PATHS] = {true, true, true, true, true};

// Dino count received from each other team (teams 5-8 -> index 0-3).
// NO_DATA until that team's message arrives.
int incoming[NUM_TEAMS] = {NO_DATA, NO_DATA, NO_DATA, NO_DATA};

// Set by decide(): true if the chosen path has fewer than 3 dinos.
bool happy = true;

int count = 0;  // number of hash marks passed so far

// Function declarations
void set_RGB(int r, int g, int b);
void atHashMark();
void check(int c);
void finish();
void read();
bool full();
char decide();
void display(char choice);
void celebrate(char choice);
void externalOff();
void setColorSetTimeAndFlash(int R_color, int G_color, int B_color, int time);
void setLEDColorTimeAndFlash(int R_color, int G_color, int B_color, int time);
void setBothColorSetTimeAndFlash(int R_color, int G_color, int B_color, int time);
void mysterious();
void eatenByDinosaurs();
void happyDance();
bool isOpen();
long microsecondsToCentimeters(long microseconds);
long rcTime(int pin);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);   // Serial Monitor (debugging)
  Serial2.begin(9600);  // XBee

  // LCD: clear screen (12), display on with no cursor (22), backlight on (17/18)
  lcdSerial.begin(9600);
  lcdSerial.write(12);
  lcdSerial.write(22);
  lcdSerial.write(18);

  servoLeft.attach(12);
  servoRight.attach(11);

  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  pinMode(redpin2, OUTPUT);
  pinMode(greenpin2, OUTPUT);
  pinMode(bluepin2, OUTPUT);

  set_RGB(255, 255, 255);  // onboard LED off
  externalOff();
}

// Sets the onboard LED. Values are inverted: 0 = fully on, 255 = off.
void set_RGB(int r, int g, int b) {
  analogWrite(redpin, r);
  analogWrite(greenpin, g);
  analogWrite(bluepin, b);
}

// ---------------------------------------------------------------------------
// Main loop: line following
// ---------------------------------------------------------------------------

void loop() {
  long qti1 = rcTime(lineSensor1);
  long qti2 = rcTime(lineSensor2);
  long qti3 = rcTime(lineSensor3);

  // true = light surface, false = dark (on the line)
  bool qti1Light = qti1 < QTI_DARK_THRESHOLD;
  bool qti2Light = qti2 < QTI_DARK_THRESHOLD;
  bool qti3Light = qti3 < QTI_DARK_THRESHOLD;

  // Combine the 3 readings into one number (0-7): left = 4, middle = 2, right = 1.
  // In the case labels below, d = dark and l = light (left, middle, right).
  int state = 4 * qti1Light + 2 * qti2Light + qti3Light;

  switch (state) {
    case 0:  // ddd: all sensors dark, so the robot is on a hash mark
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      atHashMark();
      break;

    case 1:  // ddl: line is to the left, so ease left
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1300);
      break;

    case 2:  // dld: unlikely pattern, so keep driving straight
      servoLeft.writeMicroseconds(1700);
      servoRight.writeMicroseconds(1300);
      break;

    case 3:  // dll: line is far to the left, so turn sharply left
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1300);
      break;

    case 4:  // ldd: line is to the right, so ease right
      servoLeft.writeMicroseconds(1700);
      servoRight.writeMicroseconds(1400);
      break;

    case 5:  // ldl: centered on the line, so drive straight
      servoLeft.writeMicroseconds(1700);
      servoRight.writeMicroseconds(1300);
      break;

    case 6:  // lld: line is far to the right, so turn sharply right
      servoLeft.writeMicroseconds(1700);
      servoRight.writeMicroseconds(1500);
      break;

    case 7:  // lll: line lost, so back up to find it again
      servoLeft.writeMicroseconds(1300);
      servoRight.writeMicroseconds(1700);
      break;
  }

  delay(50);
}

// ---------------------------------------------------------------------------
// Hash marks
// ---------------------------------------------------------------------------

// Runs when the robot stops on a hash mark. Shows which mark it is on the
// onboard LED, checks that path for an object, and drives past the mark.
// On the last mark, it runs the end-of-course sequence instead.
void atHashMark() {
  // Onboard LED color for each hash mark (inverted values)
  if (count == 0) {
    set_RGB(127, 255, 127);  // purple
  } else if (count == 1) {
    set_RGB(255, 0, 255);    // green
  } else if (count == 2) {
    set_RGB(255, 255, 0);    // blue
  } else if (count == 3) {
    set_RGB(0, 255, 255);    // red
  } else if (count == 4) {
    set_RGB(0, 0, 0);        // white
    delay(500);
  }

  check(count);

  if (count == NUM_PATHS - 1) {
    finish();  // never returns
  }

  count++;
  delay(500);
  set_RGB(255, 255, 255);  // onboard LED off

  // Drive forward to clear the hash mark
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1300);
  delay(500);
}

// Checks hash mark c (0-4) for an object and records the result.
// Flashes the external LED green if the path is clear, red if an object is found.
void check(int c) {
  if (isOpen()) {
    digitalWrite(greenpin2, HIGH);
    blocked[c] = false;
  } else {
    digitalWrite(redpin2, HIGH);
    blocked[c] = true;
  }
  delay(250);
  externalOff();
}

// End of the course: collect other teams' data, choose a path, send the
// choice, show it, celebrate, and stop for good.
void finish() {
  read();

  set_RGB(0, 0, 0);
  delay(250);

  char choice = decide();  // decide once so every output matches
  Serial2.print(choice);   // send to other teams (one byte, 91-95)
  Serial.print("Chosen path: ");
  Serial.println(choice % 10);

  set_RGB(255, 255, 0);
  delay(250);

  display(choice);
  celebrate(choice);

  servoLeft.detach();
  servoRight.detach();
  set_RGB(255, 255, 255);
  externalOff();

  while (true) {
    // stop permanently
  }
}

// ---------------------------------------------------------------------------
// XBee communication and path decision
// ---------------------------------------------------------------------------

// Reads other teams' messages from the XBee (Serial2).
// Each team sends one byte holding a two-digit number:
//   tens digit = team ID (5, 6, 7, or 8)
//   ones digit = that team's dino count (0-9)
// For example, 63 means "team 6 counted 3 dinos."
// Team ID minus 5 gives the array index (team 5 -> incoming[0]).
// Stops when no data is waiting or every team has reported.
void read() {
  while (Serial2.available() > 0 && !full()) {
    int in = Serial2.read();
    int team = in / 10 - 5;

    // Ignore bytes that don't match the message format
    if (team < 0 || team >= NUM_TEAMS) {
      continue;
    }
    incoming[team] = in % 10;
  }
}

// Returns true once every team's slot in incoming[] has data.
bool full() {
  for (int i = 0; i < NUM_TEAMS; i++) {
    if (incoming[i] == NO_DATA) {
      return false;
    }
  }
  return true;
}

// Chooses which path to take and returns 91-95 (91 = path 1, 95 = path 5).
//   1. Among paths 1-4 with no object detected, find the one with the fewest
//      dinos (ignoring teams that never reported).
//   2. If that path has fewer than 3 dinos, take it.
//   3. Otherwise take path 5 if it's clear.
//   4. Otherwise take the fewest-dino path anyway (happy = false).
// If no path 1-4 qualifies, falls back to path 5.
char decide() {
  int leastDino = 10;  // higher than any possible count (0-9)
  int bestPath = -1;

  for (int i = 0; i < NUM_TEAMS; i++) {
    if (!blocked[i] && incoming[i] != NO_DATA && incoming[i] < leastDino) {
      bestPath = i;
      leastDino = incoming[i];
    }
  }

  if (bestPath == -1) {  // no usable path 1-4
    happy = !blocked[4];
    return 95;
  }

  if (leastDino < 3) {
    happy = true;
    return 91 + bestPath;
  }

  if (!blocked[4]) {
    happy = true;
    return 95;
  }

  happy = false;
  return 91 + bestPath;
}

// Shows each team's dino count and the chosen path number on the LCD.
void display(char choice) {
  lcdSerial.write(12);  // clear screen
  for (int i = 0; i < NUM_TEAMS; i++) {
    lcdSerial.print(incoming[i]);
    lcdSerial.print(", ");
  }
  lcdSerial.print(choice % 10);
}

// ---------------------------------------------------------------------------
// Celebrations
// ---------------------------------------------------------------------------

// Path 5 plays a song; otherwise the robot dances if happy, or acts out
// being eaten by dinosaurs if not.
void celebrate(char choice) {
  if (choice == 95) {
    mysterious();
  } else if (happy) {
    happyDance();
  } else {
    eatenByDinosaurs();
  }
}

// Turns the external LED off.
void externalOff() {
  digitalWrite(redpin2, LOW);
  digitalWrite(greenpin2, LOW);
  digitalWrite(bluepin2, LOW);
}

// Color helpers. All take normal (non-inverted) RGB values from 0-255;
// the functions handle each LED's wiring.

// Flashes the onboard LED.
void setColorSetTimeAndFlash(int R_color, int G_color, int B_color, int time) {
  analogWrite(redpin, 255 - R_color);  // onboard LED is inverted
  analogWrite(greenpin, 255 - G_color);
  analogWrite(bluepin, 255 - B_color);
  delay(time * 1.5);
  set_RGB(255, 255, 255);  // off
  delay(10);
}

// Flashes the external LED.
void setLEDColorTimeAndFlash(int R_color, int G_color, int B_color, int time) {
  analogWrite(redpin2, R_color);  // external LED is not inverted
  analogWrite(greenpin2, G_color);
  analogWrite(bluepin2, B_color);
  delay(time * 1.5);
  externalOff();
  delay(10);
}

// Flashes both LEDs at the same time.
void setBothColorSetTimeAndFlash(int R_color, int G_color, int B_color, int time) {
  analogWrite(redpin, 255 - R_color);
  analogWrite(greenpin, 255 - G_color);
  analogWrite(bluepin, 255 - B_color);

  analogWrite(redpin2, R_color);
  analogWrite(greenpin2, G_color);
  analogWrite(bluepin2, B_color);

  delay(time * 1.5);

  set_RGB(255, 255, 255);
  externalOff();
  delay(10);
}

// Plays the song on the LCD speaker (only once).
void mysterious() {
  static bool played = false;  // static: keeps its value between calls
  if (played) {
    return;
  }
  played = true;

  for (int k = 0; k < NUM_NOTES; k++) {
    // Send duration, octave, and note to the LCD
    lcdSerial.write(durs[k]);
    lcdSerial.write(octs[k]);
    lcdSerial.write(notes[k]);

    // Duration code 214 = whole note; each step lower halves the length
    int len = 214 - durs[k];
    float del = 2000 / pow(2, len);

    // Wait for the note to finish, scaled to a tempo of 74 bpm
    delay(int(del * 240.0 / 74.0));
  }
}

// Dance for when the chosen path still has 3 or more dinos.
void eatenByDinosaurs() {
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(100);  // stop

  servoLeft.writeMicroseconds(1300);
  servoRight.writeMicroseconds(1700);
  delay(500);  // back up
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1300);
  delay(500);  // drive forward

  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1600);
  delay(300);  // pivot backward to the left
  servoLeft.writeMicroseconds(1300);
  servoRight.writeMicroseconds(1700);
  delay(300);  // back up

  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1500);
  delay(100);  // pivot forward to the right
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1450);
  delay(300);  // pivot forward to the left

  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1550);
  delay(400);  // slow pivot backward to the left
  servoLeft.writeMicroseconds(1450);
  servoRight.writeMicroseconds(1500);
  delay(400);  // slow pivot backward to the right
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1550);
  delay(400);  // slow pivot backward to the left

  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);  // stop
}

// Light show for a successful path choice, timed to a song's rhythm.
void happyDance() {
  setBothColorSetTimeAndFlash(255, 0, 0, 300);    // red
  delay(100);
  setBothColorSetTimeAndFlash(255, 180, 0, 300);  // orange
  delay(100);
  setBothColorSetTimeAndFlash(0, 150, 0, 300);    // green
  delay(200);

  setColorSetTimeAndFlash(255, 0, 0, 100);
  setLEDColorTimeAndFlash(255, 165, 0, 100);
  setColorSetTimeAndFlash(0, 0, 255, 100);
  setLEDColorTimeAndFlash(255, 255, 0, 100);
  setColorSetTimeAndFlash(0, 255, 0, 300);
  setLEDColorTimeAndFlash(220, 0, 220, 300);
  setColorSetTimeAndFlash(0, 0, 255, 550);

  delay(50);

  setColorSetTimeAndFlash(255, 0, 0, 100);
  setLEDColorTimeAndFlash(0, 165, 0, 100);
  setColorSetTimeAndFlash(0, 100, 168, 100);
  setLEDColorTimeAndFlash(200, 100, 0, 100);
  setColorSetTimeAndFlash(120, 255, 20, 300);
  setLEDColorTimeAndFlash(128, 0, 120, 300);
  setColorSetTimeAndFlash(100, 0, 200, 550);

  delay(50);

  setColorSetTimeAndFlash(255, 0, 0, 100);
  setLEDColorTimeAndFlash(0, 165, 0, 100);
  setColorSetTimeAndFlash(200, 200, 0, 100);
  setLEDColorTimeAndFlash(0, 0, 168, 100);
  setColorSetTimeAndFlash(0, 165, 0, 400);
  setLEDColorTimeAndFlash(200, 200, 0, 200);
  setColorSetTimeAndFlash(0, 0, 168, 300);
  setLEDColorTimeAndFlash(0, 255, 120, 100);
  setColorSetTimeAndFlash(200, 200, 0, 100);

  setLEDColorTimeAndFlash(128, 0, 128, 400);
  setColorSetTimeAndFlash(0, 0, 168, 800);
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

// Returns true if no object is within OBJECT_DISTANCE_CM of the PING))) sensor.
bool isOpen() {
  // Send a short trigger pulse
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // Time how long the echo takes to return
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);

  Serial.print(cm);
  if (cm <= OBJECT_DISTANCE_CM) {
    Serial.println(" cm: object detected");
    return false;
  }
  Serial.println(" cm: clear");
  return true;
}

// Converts echo time to distance. Sound travels about 29.155 us per cm, and
// the echo covers the distance twice (there and back).
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.155 / 2.0;
}

// Reads a QTI sensor: charges its capacitor, then times how long the voltage
// takes to decay. Dark surfaces reflect less light, so they decay more slowly.
long rcTime(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(1);

  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);

  long startTime = micros();
  while (digitalRead(pin)) {
    // wait for the voltage to decay
  }
  return micros() - startTime;
}
