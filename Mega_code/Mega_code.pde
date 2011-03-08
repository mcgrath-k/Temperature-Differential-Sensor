//Sketch for MY project
// Arduino Duemilanove with Atmel ATMega 328
// Includes two SHT15 temperature/humidity sensor, One red and one blue LED, and a 4x20 character display
//
#include <LiquidCrystal.h>
#include <VirtualWire.h>

#undef int
#undef abs
#undef double
#undef float
#undef round

int inByte = 0;
char firstRead = 0;
int inValue = -1;
int readTemp = -1;
int readHumid = -1;

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

byte newChar[8] = {
  B01100,
  B10010,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
  B00000
};


int gTempCmd  = 0b00000011;
int gHumidCmd = 0b00000101;

int shiftIn(int dataPin, int clockPin, int numBits) {
  int ret = 0;
  int i;

  for (i=0; i<numBits; ++i) {
    digitalWrite(clockPin, HIGH);
    delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
    ret = ret*2 + digitalRead(dataPin);
    digitalWrite(clockPin, LOW);
  }

  return(ret);
}

void sendCommandSHT(int command, int dataPin, int clockPin) {
  int ack;

  // Transmission Start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // Verify we get the coorect ack
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);
  ack = digitalRead(dataPin);
  if (ack != LOW) {
    Serial.println("Ack Error 0");
  }
  digitalWrite(clockPin, LOW);
  ack = digitalRead(dataPin);
  if (ack != HIGH) {
    Serial.println("Ack Error 1");
  }
}

void waitForResultSHT(int dataPin) {
  int i;
  int ack;

  pinMode(dataPin, INPUT);

  for(i= 0; i < 100; ++i) {
    delay(10);
    ack = digitalRead(dataPin);

    if (ack == LOW) {
      break;
    }
  }

  if (ack == HIGH) {
    Serial.println("Ack Error 2");
  }
}

int getData16SHT(int dataPin, int clockPin) {
  int val;

  // Get the lest significant bits
  pinMode(dataPin, INPUT);
  val |= shiftIn(dataPin, clockPin, 8);

  return val;
}

void skipCrcSHT(int dataPin, int clockPin)
{
  // Skip acknowledge to end trans (no CRC)
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}
const int ledPin = 6;
const int ledPin2 = 13;

void setup() {
  lcd.createChar(0, newChar);
  lcd.begin(20, 2);

  Serial.begin(9600);             // open serial

  vw_set_ptt_inverted(true);     // Required for RX Link Module
  vw_setup(2000);               // Bits per sec
  vw_set_rx_pin(25);            // We will be receiving on pin 3 ie the RX pin from the module connects to this pin. 
  vw_rx_start();              // Start the receiver 

  pinMode (ledPin, OUTPUT);
  pinMode (ledPin2, OUTPUT);
}

void loop(){
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  if (vw_get_message(buf, &buflen)) {
    Serial.println("got something");
    int i;
    for (i = 0; i < buflen; i++) {
      inByte = buf[i];
      char inChar = (char) inByte;

      if (inChar == 't' || inChar == 'h') {
        int theDataPin   = 2;
        int theClockPin  = 3;
        char cmd = 0;
        int ack;
        int val;
        int temp;
        int hval;
        int humid;
        
        if (inChar == 't') {
          readTemp = inValue;
        } else {
          readHumid = inValue;
        }

        sendCommandSHT(gTempCmd, theDataPin, theClockPin);
        waitForResultSHT(theDataPin);
        val = getData16SHT(theDataPin, theClockPin);
        skipCrcSHT(theDataPin, theClockPin);
        Serial.print("Inside Temp is:");
        Serial.print(val, HEX);
        temp = -40.0 + 2 + 0.018 * (float)val;
        Serial.print("  ");
        Serial.println(temp, DEC);         
        Serial.print("Outside Temp (temp2) is:");
        Serial.print(readTemp, HEX);
        Serial.print("  ");
        Serial.println(readTemp, DEC);
        //The statements that control the lcd.
        lcd.setCursor (0, 0);

        lcd.print("Inside Temp is ");
        lcd.print(temp, DEC);
        lcd.write(0);
        lcd.print("F ");

        lcd.print("Outer Temp is ");
        lcd.print(readTemp, DEC);
        lcd.write(0);
        lcd.print("F  ");

        // The statements that control which led turns on: red, blue, or green.
        //RED
        if (readTemp > temp) {
          digitalWrite (ledPin, HIGH);
          Serial.print("Outside temp is hotter");
        } else {
          digitalWrite (ledPin,LOW);
        }
        //BLUE
        if (temp > readTemp) {
          digitalWrite (ledPin2, HIGH);
          Serial.print("Inside temp is hotter");
        } else {
          digitalWrite (ledPin2, LOW);
        }
        //Purple!
        if (temp == readTemp) {
          digitalWrite (ledPin2, HIGH);
          digitalWrite (ledPin, HIGH);
          Serial.print("The temps are the same");
        }
        sendCommandSHT(gHumidCmd, theDataPin, theClockPin);
        waitForResultSHT(theDataPin);
        val = getData16SHT(theDataPin, theClockPin);
        skipCrcSHT(theDataPin, theClockPin);
        humid = -4.0 + 0.0405 * val + -0.0000028 * val * val;
        Serial.print("  ");

        lcd.print("The air is ");
        lcd.print(humid, DEC);
        lcd.print("%");
        lcd.print(" humid");

        lcd.print("The air is ");
        lcd.print(readHumid, DEC);
        lcd.print("%");
        lcd.print(" humid");
        inValue = -1;
      } else if (inValue == -1) {
        inValue = inByte;
      }
    }
  }
}


