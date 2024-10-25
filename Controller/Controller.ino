#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define R_ENCODER_SIG_Pin 2
#define L_ENCODER_SIG_Pin 3
#define GND 4
#define Red_Button_Pin 7
#define Green_Button_Pin 8
#define RS 6
#define LS 5
#define RF 9
#define RB 10
#define LF 11
#define LB 12
#define NeoPixel_PIN 13

#define LDR_PIN A0
#define B_PIN A1
#define R_PIN A2
#define G_PIN A3
#define Voltage_PIN A7

unsigned long previousMillisL = 0;
unsigned long previousMillisR = 0;
unsigned long previousMillisF = 0;
unsigned long previousMillisV = 0;
unsigned long previousMillisS = 0;  

const long intervalS = 1000;      
const long interval = 2000;

bool lDelayActive = false;
bool rDelayActive = false;

bool moveForwardFlag = false;

float lastVoltage = 0.0;

int lastLdrValue = 0;                  
unsigned long previousLdrMillis = 0;   
const long printLdrInterval = 1000;    

int ldrValue = 0;
int sensorValue = 0;
float voltage = 0.0;

long gulCount = 0;
unsigned short gusEncoder = 0;
unsigned short gusEncoder_Bak = 0;
unsigned short gusEncoder_Change = 0;
long glEncoder_Count_Bak = 100;
unsigned short usIsFoward = 0;

char lastCommand = '\0'; 

bool stat = true;
bool stat2 = true;
bool L_S = false;
bool R_S = false;
bool M_S = false;

int stopCount = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, NeoPixel_PIN, NEO_GRB + NEO_KHZ800);

void config() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();
  pinMode(R_ENCODER_SIG_Pin, INPUT);
  pinMode(L_ENCODER_SIG_Pin, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(Voltage_PIN, INPUT);

  pinMode(Red_Button_Pin, INPUT_PULLUP);
  pinMode(Green_Button_Pin, INPUT_PULLUP);

  pinMode(GND, OUTPUT);
  pinMode(RS, OUTPUT);
  pinMode(LS, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(NeoPixel_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  digitalWrite(GND, LOW);

  lcd.begin();
  lcd.backlight();
  lcd.clear();

  lcd.clear();
  lcd.print("   Group - 48");
  lcd.setCursor(0, 1);
  lcd.print(" Pasindu Sahan");
  delay(2000);
}

void setup() {
  Serial.begin(9600);
  config();
  analogWrite(LS, 180);
  analogWrite(RS, 180);
}

void showWhite() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255)); 
  }
  strip.show();  
}

void loop() {
  if (digitalRead(Green_Button_Pin) == 1 && stat == true) {
    stat = false;
  }

  if (digitalRead(Red_Button_Pin) == 1 && stat2 == true) {
    stat2 = false;
  }

  if (stat == false) {
    //check();
  }

  // if (stat2 == false) {
  //   turnLeft();
  //   // stopMotors();
  // }


  else if (M_S == true) {
    analogWrite(LS, 225);
    analogWrite(RS, 225);
    digitalWrite(B_PIN, HIGH);
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, HIGH);
    long lEncoder_Count = 0;
    int acceleration = ((180) * 1);
    if ((glEncoder_Count_Bak <= acceleration - 34)) {
      digitalWrite(RB, HIGH);
      digitalWrite(LB, HIGH);
      usIsFoward = 1;
    } else {
      M_S = false;
      digitalWrite(RF, LOW);
      digitalWrite(RB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(LB, LOW);

      int realValue = glEncoder_Count_Bak;
      stat = true;
      gulCount = 0;
      gusEncoder = 0;
      gusEncoder_Bak = 0;
      gusEncoder_Change = 0;
      glEncoder_Count_Bak = 100;
      usIsFoward = 0;
    }

    lEncoder_Count = vRead_Encoder(R_ENCODER_SIG_Pin, usIsFoward);

    if (lEncoder_Count != glEncoder_Count_Bak) {
      glEncoder_Count_Bak = lEncoder_Count;
    }
  }

  else if (L_S == true) {
    analogWrite(LS, 225);
    analogWrite(RS, 225);
    digitalWrite(B_PIN, HIGH);
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, LOW);


    long lEncoder_Count = 0;
    int acceleration = ((360) * 1);
    if (glEncoder_Count_Bak <= acceleration - 34) {
      digitalWrite(RB, HIGH);
      digitalWrite(LF, HIGH);
      usIsFoward = 1;
    } else {
      digitalWrite(RF, LOW);
      digitalWrite(RB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(LB, LOW);
      L_S = false;
      Serial.println("ACK");

      int realValue = glEncoder_Count_Bak;
      stat2 = true;
      gulCount = 0;
      gusEncoder = 0;
      gusEncoder_Bak = 0;
      gusEncoder_Change = 0;
      glEncoder_Count_Bak = 100;
      usIsFoward = 0;
    }

    lEncoder_Count = vRead_Encoder(L_ENCODER_SIG_Pin, usIsFoward);

    if (lEncoder_Count != glEncoder_Count_Bak) {
      glEncoder_Count_Bak = lEncoder_Count;
    }
  }

  else if (R_S == true) {
    analogWrite(LS, 225);
    analogWrite(RS, 225);
    digitalWrite(B_PIN, HIGH);
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, HIGH);
    long lEncoder_Count = 0;
    int acceleration = ((360) * 1);
    if ((glEncoder_Count_Bak <= acceleration - 34)) {
      digitalWrite(RF, HIGH);
      digitalWrite(LB, HIGH);
      usIsFoward = 1;
    } else {
      R_S = false;
      Serial.println("ACK");
      digitalWrite(RF, LOW);
      digitalWrite(RB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(LB, LOW);

      int realValue = glEncoder_Count_Bak;
      stat = true;
      gulCount = 0;
      gusEncoder = 0;
      gusEncoder_Bak = 0;
      gusEncoder_Change = 0;
      glEncoder_Count_Bak = 100;
      usIsFoward = 0;
    }

    lEncoder_Count = vRead_Encoder(R_ENCODER_SIG_Pin, usIsFoward);

    if (lEncoder_Count != glEncoder_Count_Bak) {
      glEncoder_Count_Bak = lEncoder_Count;
    }
  }

  if (lDelayActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisL >= interval) {
      L_S = true;
      lDelayActive = false;  
    }
  }

  if (rDelayActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisR >= interval) {
      R_S = true;
      rDelayActive = false;  
    }
  }

  if (moveForwardFlag) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisF >= interval) {
      moveForward();
      moveForwardFlag = false; 
    }
  }

  handleMotorCommands();
  delay(1);
}

void ldr() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousLdrMillis >= printLdrInterval) {
    previousLdrMillis = currentMillis;

    ldrValue = analogRead(LDR_PIN);

    if (abs(ldrValue - lastLdrValue) > 100) {
      Serial.println(ldrValue);
      lastLdrValue = ldrValue; 
    }
  }
}

void voltageCalc() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisV >= interval) {
    previousMillisV = currentMillis; 

    int sensorValue = analogRead(Voltage_PIN);
    voltage = mapToVoltage(sensorValue);

    if (abs(voltage - lastVoltage) > 1.0) {
      //Serial.print("Voltage: - ");
      Serial.println(voltage);
      lastVoltage = voltage; 
    }
  }
}

float mapToVoltage(int sensorValue) {
  return map(sensorValue, 148, 488, 281, 1212) / 100.0;
}

long vRead_Encoder(unsigned short Encoder_Pin, unsigned short usForward_Reverse) {
  gusEncoder = digitalRead(Encoder_Pin);

  if (gusEncoder != gusEncoder_Bak) {
    gusEncoder_Change = 1;
  }

  if (gusEncoder_Change == 1) {
    if (gusEncoder == gusEncoder_Bak) {
      gusEncoder_Change = 0;

      if (usForward_Reverse == 1) {
        gulCount++;
      } else {
        gulCount--;
      }
    }
  }

  return gulCount;
}

void moveForward() {
  analogWrite(LS, 180);
  analogWrite(RS, 180);
  digitalWrite(G_PIN, HIGH);
  digitalWrite(B_PIN, LOW);
  digitalWrite(R_PIN, LOW);

  digitalWrite(RF, HIGH);
  digitalWrite(RB, LOW);
  digitalWrite(LF, HIGH);
  digitalWrite(LB, LOW);
}

void moveBackward() {
  digitalWrite(RF, LOW);
  digitalWrite(RB, HIGH);
  digitalWrite(LF, LOW);
  digitalWrite(LB, HIGH);
}

void showoff() {
  digitalWrite(B_PIN, HIGH);

  digitalWrite(RF, HIGH);
  digitalWrite(LB, HIGH);
}

void stopMotors() {
  digitalWrite(R_PIN, HIGH);
  digitalWrite(G_PIN, LOW);
  digitalWrite(B_PIN, LOW);

  digitalWrite(RF, LOW);
  digitalWrite(RB, LOW);
  digitalWrite(LF, LOW);
  digitalWrite(LB, LOW);
}

void intial() {
  gulCount = 0;
  gusEncoder = 0;
  gusEncoder_Bak = 0;
  gusEncoder_Change = 0;
  glEncoder_Count_Bak = 100;
  usIsFoward = 0;
  stat = true;
  stat2 = true;
  L_S = false;
  R_S = false;
  M_S = false;
  stopCount = 0;

  lcd.clear();
  displayMessage("Connected With Raspberry PI");
}

void arrow() {
  strip.begin();
  strip.show();
  strip.setBrightness(255);
  showWhite();
  lcd.clear();
  displayMessage("Arrow Identify Begins");
}

void off() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();

  lcd.clear();
  displayMessage("Arrow Identify Stopped");
}

void quit() {
  lcd.clear();
  displayMessage("Disconnect From Raspberry PI");
}


void handleMotorCommands() {
  static unsigned long lastAckTime = 0; 
  const unsigned long ackDelay = 200;

  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '\n' || command == '\r' || command == '\0') {
      return;
    }

    if ((command != lastCommand) || command == 'S' || command == 'D') {
      lastCommand = command;
      lcd.clear();
      switch (command) {
        case 'F':
          stopMotors();
          lcd.print("Forward");

          previousMillisF = millis();
          moveForwardFlag = true;
          break;
        case 'B':
          stopMotors();
          lcd.print("Backward");
          moveBackward();
          break;
        case 'L':
          stopMotors();
          lcd.print("Left");
          //M_S = true;
          previousMillisL = millis();
          lDelayActive = true;
          break;
        case 'R':
          stopMotors();
          //M_S = true;
          lcd.print("Right");
          previousMillisR = millis();
          rDelayActive = true;
          break;
        case 'M':
          stopMotors();
          M_S = true;
          break;
        case 'S':
          stopMotors();
          lcd.print("Stop");
          stopCount++;
          break;
        case 'D':
          sendStatusToPython();
          break;
        case 'I':
          intial();
          break;
        case 'A':
          arrow();
          break;
        case 'O':
          stopMotors();
          off();
          break;
        case 'Q':
          stopMotors();
          quit();
          break;
      }
      Serial.println("ACK");
    } else {
      Serial.println("ACK");
    }
  }
}


void sendStatusToPython() {
  int ldrValue = analogRead(LDR_PIN);
  float voltage = mapToVoltage(analogRead(Voltage_PIN));
  Serial.print("LDR: ");
  Serial.print(ldrValue);
  Serial.print(", Voltage: ");
  Serial.println(voltage);
}

void displayMessage(const String& text) {
  lcd.clear();
  if (text.length() <= 16) {
    lcd.print(text); 
  } else {
    int lastSpace = text.lastIndexOf(' ', 16); 
    if (lastSpace == -1) {
      lastSpace = 16;
    }

    lcd.print(text.substring(0, lastSpace));
    lcd.setCursor(0, 1);
    lcd.print(text.substring(lastSpace + 1));
  }
}

