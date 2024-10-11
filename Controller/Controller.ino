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
#define Voltage_PIN A0

unsigned long previousMillisL = 0;
unsigned long previousMillisR = 0;
const long interval = 2000;  // 4 seconds
bool lDelayActive = false;
bool rDelayActive = false;



int ldrValue = 0;
int sensorValue = 0;
float voltage = 0.0;

int acc, real;
long gulCount = 0;
unsigned short gusEncoder = 0;
unsigned short gusEncoder_Bak = 0;
unsigned short gusEncoder_Change = 0;
long glEncoder_Count_Bak = 100;
unsigned short usIsFoward = 0;

char lastCommand = '\0';  // Initialize to a character that won't be used


bool stat = true;
bool stat2 = true;
bool L_S = false;
bool R_S = false;
bool M_S = false;  //Miss stop

LiquidCrystal_I2C lcd(0x27, 16, 2);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, NeoPixel_PIN, NEO_GRB + NEO_KHZ800);

void config() {
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

  lcd.print("Pasindu Sahan");
  Serial.write("N", 1);
}

void setup() {
  Serial.begin(115200);
  config();
  strip.begin();
  strip.show();
  strip.setBrightness(250);
  showWhite();

  analogWrite(LS, 180);
  analogWrite(RS, 180);

  // turnRight();
}

void showWhite() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 0));  // Set pixel color to white
  }
  strip.show();  // Update the strip to show the changes
}

void loop() {
  // ldr();
  // voltageCalc();
  if (digitalRead(Green_Button_Pin) == 1 && stat == true) {
    // // // // //Serial.println("Acc:- ");
    stat = false;
    // Serial.write("i", 1);
  }

  if (digitalRead(Red_Button_Pin) == 1 && stat2 == true) {
    // // // // //Serial.println("Acc:- ");
    stat2 = false;
    // Serial.write("h", 1);
  }

  if (stat == false) {
    // turnRight();
    // showoff();
    check();
  }

  // if (stat2 == false) {
  //   turnLeft();
  //   // stopMotors();
  // }

  else if (M_S == true) {
    //// // // //Serial.println("turnRight");

    analogWrite(LS, 225);
    analogWrite(RS, 225);
    digitalWrite(B_PIN, HIGH);
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, HIGH);
    long lEncoder_Count = 0;
    int acceleration = ((180) * 1);
    if ((glEncoder_Count_Bak <= acceleration - 34)) {
      // digitalWrite(IN1, HIGH);
      // digitalWrite(IN2, LOW);
      // // // // //Serial.println(glEncoder_Count_Bak);

      digitalWrite(RB, HIGH);
      digitalWrite(LB, HIGH);
      usIsFoward = 1;
    } else {
      // // // //Serial.println("Stop");
      M_S = false;
      //Serial.write("A", 1);

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
      // calibrate(acceleration, realValue);
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
      // digitalWrite(IN1, HIGH);
      // digitalWrite(IN2, LOW);
      digitalWrite(RB, HIGH);
      digitalWrite(LF, HIGH);
      usIsFoward = 1;
    } else {
      digitalWrite(RF, LOW);
      digitalWrite(RB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(LB, LOW);
      L_S = false;
      Serial.write("A", 1);

      int realValue = glEncoder_Count_Bak;
      stat2 = true;
      gulCount = 0;
      gusEncoder = 0;
      gusEncoder_Bak = 0;
      gusEncoder_Change = 0;
      glEncoder_Count_Bak = 100;
      usIsFoward = 0;
      // calibrate(acceleration, realValue);
    }

    lEncoder_Count = vRead_Encoder(L_ENCODER_SIG_Pin, usIsFoward);

    if (lEncoder_Count != glEncoder_Count_Bak) {
      glEncoder_Count_Bak = lEncoder_Count;
    }
  }

  else if (R_S == true) {
    //// // // //Serial.println("turnRight");

    analogWrite(LS, 225);
    analogWrite(RS, 225);
    digitalWrite(B_PIN, HIGH);
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, HIGH);
    long lEncoder_Count = 0;
    int acceleration = ((360) * 1);
    if ((glEncoder_Count_Bak <= acceleration - 34)) {
      // digitalWrite(IN1, HIGH);
      // digitalWrite(IN2, LOW);
      // // // // //Serial.println(glEncoder_Count_Bak);

      digitalWrite(RF, HIGH);
      digitalWrite(LB, HIGH);
      usIsFoward = 1;
    } else {
      R_S = false;
      Serial.write("A", 1);

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
      // calibrate(acceleration, realValue);
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
      lDelayActive = false;  // Reset the flag
      // // // //Serial.println("LL");
    }
  }

  if (rDelayActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisR >= interval) {
      R_S = true;
      rDelayActive = false;  // Reset the flag
      // // // //Serial.println("RR");
    }
  }

  handleMotorCommands();
}

void check() {
  // // // //Serial.print("Check Value: ");
  Serial.write("H", 1);
  stat = true;
}
void ldr() {
  ldrValue = analogRead(LDR_PIN);
  // // // // //Serial.print("LDR Value: ");
  // // // // //Serial.println(ldrValue);
}

void voltageCalc() {
  sensorValue = analogRead(Voltage_PIN);
  // Calculate the voltage (assuming a 5V reference voltage)
  voltage = sensorValue * (12 / 1023.0);
  // // // //Serial.print("Sensor Value: ");
  // // // //Serial.print(sensorValue);
  // // // //Serial.print(" - Voltage: ");
  // // // //Serial.println(voltage);
}

void calibrate(int acc, int real) {
  // // // //Serial.print("Acc:- ");
  // // // //Serial.print(acc);
  // // // //Serial.print(" Real:-");
  // // // //Serial.print(real);
  // // // //Serial.print("  Diff:- ");
  // // // //Serial.println(real - acc);
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
  analogWrite(LS, 170);
  analogWrite(RS, 170);
  digitalWrite(G_PIN, HIGH);
  digitalWrite(B_PIN, LOW);
  digitalWrite(R_PIN, LOW);

  digitalWrite(RF, HIGH);
  digitalWrite(RB, LOW);
  digitalWrite(LF, HIGH);
  digitalWrite(LB, LOW);
  delay(500);
}

void moveBackward() {
  Serial.write("A", 1);
  digitalWrite(RF, LOW);
  digitalWrite(RB, HIGH);
  digitalWrite(LF, LOW);
  digitalWrite(LB, HIGH);
}

void turnLeft() {
  analogWrite(LS, 225);
  analogWrite(RS, 225);
  digitalWrite(B_PIN, HIGH);
  digitalWrite(R_PIN, LOW);
  digitalWrite(G_PIN, LOW);


  long lEncoder_Count = 0;
  int acceleration = ((360) * 1);
  if (glEncoder_Count_Bak <= acceleration - 34) {
    // digitalWrite(IN1, HIGH);
    // digitalWrite(IN2, LOW);
    digitalWrite(RB, HIGH);
    digitalWrite(LF, HIGH);
    usIsFoward = 1;
  } else {
    digitalWrite(RF, LOW);
    digitalWrite(RB, LOW);
    digitalWrite(LF, LOW);
    digitalWrite(LB, LOW);

    int realValue = glEncoder_Count_Bak;
    stat2 = true;
    gulCount = 0;
    gusEncoder = 0;
    gusEncoder_Bak = 0;
    gusEncoder_Change = 0;
    glEncoder_Count_Bak = 100;
    usIsFoward = 0;
    // calibrate(acceleration, realValue);
  }

  lEncoder_Count = vRead_Encoder(L_ENCODER_SIG_Pin, usIsFoward);

  if (lEncoder_Count != glEncoder_Count_Bak) {
    glEncoder_Count_Bak = lEncoder_Count;
  }
}

void turnRight() {
  // // // //Serial.println("turnRight");

  analogWrite(LS, 225);
  analogWrite(RS, 225);
  digitalWrite(B_PIN, HIGH);
  digitalWrite(R_PIN, LOW);
  digitalWrite(G_PIN, HIGH);
  long lEncoder_Count = 0;
  int acceleration = ((360) * 1);
  if ((glEncoder_Count_Bak <= acceleration - 34) && R_S == false) {
    // digitalWrite(IN1, HIGH);
    // digitalWrite(IN2, LOW);
    // // // // //Serial.println(glEncoder_Count_Bak);

    digitalWrite(RF, HIGH);
    digitalWrite(LB, HIGH);
    usIsFoward = 1;
  } else {
    // // // //Serial.println("Stop");
    R_S = true;
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
    // calibrate(acceleration, realValue);
  }

  lEncoder_Count = vRead_Encoder(R_ENCODER_SIG_Pin, usIsFoward);

  if (lEncoder_Count != glEncoder_Count_Bak) {
    glEncoder_Count_Bak = lEncoder_Count;
  }
}

void showoff() {
  digitalWrite(B_PIN, HIGH);

  digitalWrite(RF, HIGH);
  digitalWrite(LB, HIGH);
}

void stopMotors() {
  Serial.write("A", 1);

  digitalWrite(R_PIN, HIGH);
  digitalWrite(G_PIN, LOW);
  digitalWrite(B_PIN, LOW);

  digitalWrite(RF, LOW);
  digitalWrite(RB, LOW);
  digitalWrite(LF, LOW);
  digitalWrite(LB, LOW);
}


void handleMotorCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    lcd.clear();
    lcd.print(command);
    // Debug: Print the current command
    // // // // //Serial.print("Received command: ");
    // // // // //Serial.println(command);

    // Ignore empty or invalid characters
    if (command == '\n' || command == '\r' || command == '\0') {
      return;  // Skip processing for empty or invalid input
    }

    // Only process the command if it's different from the last command
    if (command != lastCommand) {

      lastCommand = command;

      // Debug: Print the last command for comparison
      // // // // // //Serial.print("Last command was: ");
      // // // // // //Serial.println(lastCommand);

      switch (command) {
        case 'F':
          stopMotors();
          delay(2000);
          Serial.write("A", 1);

          moveForward();
          break;
        case 'B':
          stopMotors();
          delay(2000);
          moveBackward();
          break;
        case 'L':
          stopMotors();
          delay(2000);
          // turnLeft();
          M_S = true;
          previousMillisL = millis();  // Start the delay
          lDelayActive = true;         // Set the flag to indicate delay is active
          break;
        case 'R':
          stopMotors();
          delay(2000);
          // turnRight();
          M_S = true;
          previousMillisR = millis();  // Start the delay for R_S
          rDelayActive = true;         // Set the flag to indicate delay is active
          break;
        case 'M':
          stopMotors();
          delay(2000);
          // turnRight();
          M_S = true;
          break;
        case 'S':
          stopMotors();
          break;
        default:
          // Handle unknown commands
          // // // //Serial.print("Unknown command: ");
          // // // //Serial.println(command);
          break;
      }

      // Debug: Print the updated last command
      // // // // // //Serial.print("Updated last command to: ");
      // // // // // //Serial.println(lastCommand);
    } else {
      // Debug: Print a message if the command is the same as the last
      // // // // // //Serial.println("Command is the same as the last command. Ignoring.");
    }
  }
}
