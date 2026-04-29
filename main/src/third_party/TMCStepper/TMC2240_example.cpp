#include "SPI.h"

const int PIN_EN = 21;
const int PIN_STEP = 5;
const int PIN_DIR = 10;
const int PIN_CS = 23;
const int PIN_CLK = 19;
const int PIN_MOSI = 22;
const int PIN_MISO = 18;

uint8_t CurrentRun = 16;
uint8_t CurrentHold = 16;
uint8_t Microsteps = 4;
uint8_t TOff = 0x05;
float SPISpeed = 16000000 / 8;

bool Setup = false;

void setup() 
{
  Serial.begin(9600);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_CS, OUTPUT);

  digitalWrite(PIN_EN, LOW);

  SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);
  digitalWrite(PIN_CS, HIGH);
}

void loop() 
{
  if (!Setup)
  {
    Setup = MotorInit(CurrentRun, CurrentHold, Microsteps);

    if (!Setup)
    {
      Serial.println("Failed to initialize! Retrying...");
      delay(1000);
      return;
    }
    else
    {
      Serial.println("Successfully initialized!");
    }
  }
  
  Serial.println("Status: " + ByteToBin(MotorGetStatus()));
  Serial.println("Temp: " + String(MotorGetTemp()));

  MotorRun(400, 0, 1000);
  MotorRun(400, 1, 1000);

  delay(5000);
}

uint8_t MotorGetStatus()
{
  uint8_t status;
  uint32_t dataDummy;
  RegRead(0x00, &dataDummy, &status);

  return status;
}

float MotorGetTemp()
{
  uint8_t status;
  uint32_t data;
  RegRead(0x51, &data, &status);

  return (float)((uint16_t)(data & 0x00001FFF) - 2038) / 7.7;
}

void MotorRun(const uint32_t steps, const bool dir, const uint32_t stepDelay)
{
  digitalWrite(PIN_DIR, dir);

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(stepDelay);
  }
}

bool MotorInit(const uint8_t currentRun, const uint8_t currentHold, const uint8_t microsteps)
{
  uint32_t ms;

  switch (microsteps)
  {
    case 128: ms = 0x1; break;
    case 64: ms = 0x2; break;
    case 32: ms = 0x3; break;
    case 16: ms = 0x4; break;
    case 8: ms = 0x5; break;
    case 4: ms = 0x6; break;
    case 2: ms = 0x7; break;
    case 1: ms = 0x8; break;
    default: ms = 0x0; break;
  }

  // Setting TOFF flag & microsteps...
  RegWrite(0x6C, 0x10410150 | (ms << 24) | TOff);
  // Setting running & holding current...
  RegWrite(0x10, 0x00060000 | ((uint32_t)(currentRun & 0x1F) << 8) | ((uint32_t)currentHold & 0x1F));

  uint8_t status;
  uint32_t data;
  RegRead(0x6C, &data, &status);

  if ((data & 0x0000000F) != TOff)
    return false;

  return true;
}

void RegWrite(const uint8_t address, const uint32_t data)
{
  uint8_t buff[5] = {address | 0x80, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
  SPIExchange(buff, 5);
}

void RegRead(const uint8_t address, uint32_t* data, uint8_t* status)
{
  uint8_t buff[5];

  for (int i = 0; i < 2; i++)
  {
    buff[0] = address; buff[1] = 0x00; buff[2] = 0x00; buff[3] = 0x00; buff[4] = 0x00;
    SPIExchange(buff, 5);
  }

  *status = buff[0];
  *data = (buff[1] << 24) | (buff[2] << 16) | (buff[3] << 8) | buff[4];
}

void SPIExchange(uint8_t* data, const int size)
{
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(SPISpeed, MSBFIRST, SPI_MODE3));
  delayMicroseconds(1);
  
  SPI.transfer(data, size);
  delayMicroseconds(1);

  SPI.endTransaction();
  delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
}

String ByteToBin(const uint8_t num)
{
  String str = String(bitRead(num, 0)) + String(bitRead(num, 1)) + String(bitRead(num, 2)) + String(bitRead(num, 3)) +
               String(bitRead(num, 4)) + String(bitRead(num, 5)) + String(bitRead(num, 6)) + String(bitRead(num, 7));

  return StrReverse(str);
}

String Int16ToBin(const uint16_t num)
{
  String str = String(bitRead(num, 0)) + String(bitRead(num, 1)) + String(bitRead(num, 2)) + String(bitRead(num, 3)) +
               String(bitRead(num, 4)) + String(bitRead(num, 5)) + String(bitRead(num, 6)) + String(bitRead(num, 7)) + " " +
               String(bitRead(num, 8)) + String(bitRead(num, 9)) + String(bitRead(num, 10)) + String(bitRead(num, 11)) +
               String(bitRead(num, 12)) + String(bitRead(num, 13)) + String(bitRead(num, 14)) + String(bitRead(num, 15));

  return StrReverse(str);
}

String Int32ToBin(const uint32_t num)
{
  String str = String(bitRead(num, 0)) + String(bitRead(num, 1)) + String(bitRead(num, 2)) + String(bitRead(num, 3)) +
               String(bitRead(num, 4)) + String(bitRead(num, 5)) + String(bitRead(num, 6)) + String(bitRead(num, 7)) + " " +
               String(bitRead(num, 8)) + String(bitRead(num, 9)) + String(bitRead(num, 10)) + String(bitRead(num, 11)) +
               String(bitRead(num, 12)) + String(bitRead(num, 13)) + String(bitRead(num, 14)) + String(bitRead(num, 15)) + " " +
               String(bitRead(num, 16)) + String(bitRead(num, 17)) + String(bitRead(num, 18)) + String(bitRead(num, 19)) +
               String(bitRead(num, 20)) + String(bitRead(num, 21)) + String(bitRead(num, 22)) + String(bitRead(num, 23)) + " " +
               String(bitRead(num, 24)) + String(bitRead(num, 25)) + String(bitRead(num, 26)) + String(bitRead(num, 27)) +
               String(bitRead(num, 28)) + String(bitRead(num, 29)) + String(bitRead(num, 30)) + String(bitRead(num, 31));

  return StrReverse(str);
}

String StrReverse(String str)
{
  String strReversed = "";

  for (int i = (str.length() - 1); i >= 0; i--)
    strReversed += str[i];

  return strReversed;
}