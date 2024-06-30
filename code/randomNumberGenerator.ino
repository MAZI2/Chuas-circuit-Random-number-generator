#include <LiquidCrystal.h>

int encoderOutA = 7;
int encoderOutB = 6;
int switchPin = 5;
int stopPin = 4;
int resetPin = 3;

const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


int selectStart = true;
int pressed = false;

//AdvanceSevenSegment sevenSegment(12, 11, 10, 9, 8, 7, 6, 5);

const int calibrationPin = 2;
const int analogPin = A5;    // Pin connected to the voltage to be read
float threshold = 29.99;  // 0-1 Threshold

// sliding average buffer
//int buffer[100];
//int counter = 0;
//int len = 100;

int start = 0;
int end = 1000;
int pres = -1;
int prev = -1;  

int calibrating = false;

// 32bit register
uint32_t arr[33];
int arrC = 0;
/*
int perm[32] = {
        14, 5, 27, 9, 18, 3, 30, 1,
        21, 11, 26, 8, 17, 19, 23, 6,
        15, 7, 20, 0, 12, 31, 10, 24,
        16, 2, 29, 22, 13, 4, 28, 25
    };
*/

// 0-1 ratio statistics
long o=0;
long z=0;
long c=0;

unsigned long previousMillis = 0;
unsigned long lockMillis = 0; 

unsigned long timer = 0;
unsigned long countGenerated = 0;

int lock = false;
int speed = 1;

int stopButtonPressed = false;


uint32_t arrayToInteger() {
    uint32_t result = 0;
    for (int i = 0; i < 32; i++) {
        result |= (arr[i] << (31 - i));
    }
    return result;
}

void calibrate() {
  int check = 2;//3;
  int display = true;

  float a = 0;
  float b = 100;

  calibrating = true;
  lcd.clear();

  while(check != 0) {
    lcd.setCursor(0, 0);
    lcd.print("Calibrating...");
    float sensorValue = analogRead(analogPin);  // Read the analog input
    float voltage = sensorValue;

    if(voltage != 0) {
      if(voltage > (a+b)/2)
        o++;
      else
        z++;
      c++;
    }
    //Serial.println(voltage);

    unsigned long currentMillis = millis();
    unsigned long interval = 10000;

    // Check if the interval has passed
    if(currentMillis - previousMillis >= interval) {
      // Save the last time you printed the message
      previousMillis = currentMillis;

      lcd.setCursor(0,1);
      lcd.print((a+b)/2);
      lcd.print(": ");
      lcd.print(round((float)(z*100)/(float)c));
      lcd.print(" ");
      lcd.print(round((float)(o*100)/(float)c));
      /*
      Serial.print((a+b)/2);
      Serial.print(": ");
      Serial.print(round((float)(z*100)/(float)c));
      Serial.print(" ");
      Serial.print(round((float)(o*100)/(float)c));
      Serial.println();
      */

      if(round((float)(z*100)/(float)c) == round((float)(o*100)/(float)c)) {
        check--;
      } else {
        check = 3;

        if(o > z)
          a=(a+b)/2;
        else
          b=(a+b)/2;
        
        o=0;
        z=0;
        c=0;
      }
    }
    
    //if((currentMillis - previousMillis) % 1000 == 0) {
    //  if(display)
    //    sevenSegment.setCharacter('c');
    //  else
    //    sevenSegment.clean();
    //  display = !display;
    //}
    
  }
  calibrating = false;
  threshold = (a+b)/2;
  //sevenSegment.clean();
  Serial.println("Calibrated!");

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Calibrated!");
}

void setup() {
  Serial.begin(115200);       // Start the serial communication
  pinMode(calibrationPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(encoderOutA,INPUT_PULLUP);
  pinMode(encoderOutB,INPUT_PULLUP);
  pinMode(switchPin,INPUT_PULLUP);
  pinMode(stopPin,INPUT_PULLUP);

  lcd.begin(16, 2);
  timer = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // reset bounds
  if(digitalRead(resetPin) == LOW) {
    start = 0;
    end = 0;
  }

  if(digitalRead(stopPin) == HIGH) {
    stopButtonPressed = !stopButtonPressed;

    while(digitalRead(stopPin) == HIGH) {
      delay(50);
    }
  }

  // lock/unlock
  unsigned long interval = 3000;

  // Check if the interval has passed
  if(currentMillis - lockMillis >= interval) {
    lock = false;
  }
  if(currentMillis - previousMillis >= 50) {
    previousMillis = currentMillis;
    if(speed - 3 > 1)
      speed -= 3;
    else
      speed = 1;
  }

  // rotary handler
  pres = digitalRead(encoderOutA);
  if(prev == -1)
    prev = pres;
  
  if(!pressed && digitalRead(switchPin) == LOW) {
    pressed = true;
    selectStart = !selectStart;
  } else if(pressed && digitalRead(switchPin) == HIGH)
    pressed = false;

  pres = digitalRead(encoderOutA);
  if (pres != prev) {     
    lock = true;
    lockMillis = currentMillis;

    if (digitalRead(encoderOutB) != pres) {
      if(selectStart == true)
        start += speed;
      else
        end += speed;
    } else {
      if(selectStart == true)
        start -= speed;
      else
        end -= speed;
    }
    speed += 3;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("L: ");
    lcd.print(start);
    lcd.setCursor(0, 1);
    lcd.print("U: ");
    lcd.print(end);
  } 
  prev = pres;
  
  int calibration = digitalRead(calibrationPin);
  if(calibration == LOW && calibrating == false)
    calibrate();
  
  int sensorValue = analogRead(analogPin);  // Read the analog input
  float voltage = sensorValue;// * (referenceVoltage / analogMaxValue);  // Convert  to voltage

  // BUILD 32bit INTEGER
  if(voltage != 0 && !calibrating && !lock) {
    if(voltage > threshold)
      arr[arrC]=1;
    else
      arr[arrC]=0;
    
    if(arrC<31)
      arrC++;
    else {
      // number built
      arrC=0;

      uint32_t nn = arrayToInteger();
      double rr = (double)nn/(double)UINT32_MAX;

      /* timer
      countGenerated++;

      unsigned int seconds = (currentMillis - timer)/1000;

      if(seconds % 10 == 0) {
        Serial.print(seconds);
        Serial.print(": ");
        Serial.print((float)countGenerated/(float)seconds);
        Serial.println();
        Serial.println(countGenerated);
      }
      */

      // print generated random double [0-1], 10 decimal places
      //Serial.println(rr, 10);
      //Serial.println(nn);
      //Serial.println(stopButtonPressed);

      if(!stopButtonPressed) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(rr*(end-start)+start);
        delay(50);
      }
    }
    //Serial.println(voltage);
  }
}