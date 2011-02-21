#include <VirtualWire.h>

#undef int
#undef abs
#undef double
#undef float
#undef round

int gTempCmd  = 0b00000011;
int gHumidCmd = 0b00000101;

int shiftIn(int dataPin, int clockPin, int numBits)
{
  int ret = 0;
  int i;

  for (i=0; i<numBits; ++i)
  {
    digitalWrite(clockPin, HIGH);
    delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
    ret = ret*2 + digitalRead(dataPin);
    digitalWrite(clockPin, LOW);
  }

  return(ret);
}

void sendCommandSHT(int command, int dataPin, int clockPin)
{
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
  if (ack != LOW)
    Serial.println("Ack Error 0");
  digitalWrite(clockPin, LOW);
  ack = digitalRead(dataPin);
  if (ack != HIGH)
    Serial.println("Ack Error 1");
}

void waitForResultSHT(int dataPin)
{
  int i;
  int ack;

  pinMode(dataPin, INPUT);

  for(i= 0; i < 100; ++i)
  {
    delay(10);
    ack = digitalRead(dataPin);

    if (ack == LOW)
      break;
  }

  if (ack == HIGH)
    Serial.println("Ack Error 2");
}

int getData16SHT(int dataPin, int clockPin)
{
  int val;

  // Get the most significant bits
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  val = shiftIn(dataPin, clockPin, 8);
  val *= 256;

  // Send the required ack
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

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

void setup()
{
  Serial.begin(9600);
  vw_set_ptt_inverted(true);
  vw_setup(2000);
  vw_set_tx_pin(3);
}

void loop()
{
  int theDataPin  = 10;
  int theClockPin = 11;
  char cmd = 0;
  int ack;

  int val;
  int temp;
  
  sendCommandSHT(gTempCmd, theDataPin, theClockPin);
  waitForResultSHT(theDataPin);
  val = getData16SHT(theDataPin, theClockPin);
  skipCrcSHT(theDataPin, theClockPin);
  temp = -40.0 + 0.018 * (float)val;
  Serial.print("t");
  Serial.print(temp, HEX);
    
  int tempC1 = (int)temp;
  char msg[24];
  sprintf(msg, "%i", tempC1);
  vw_send((uint8_t *)msg, strlen(msg)); 
   
  delay(1000);  
}


