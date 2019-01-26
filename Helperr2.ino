
#include <Wire.h>
#include <SoftwareSerial.h>

boolean isBluetoothEnabled = true;


int rxPin = 0;
int txPin = 1;
SoftwareSerial bluetooth(rxPin, txPin);


int btnPin1 = 9;
int btnPin2 = 8;
int yellowLedPin = 13;
int redLedPin = 12;


const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Status variables, used with buttons
int precBtn1 = HIGH;
int precBtn2 = HIGH;


void setup() {

  pinMode(btnPin1, INPUT_PULLUP);
  pinMode(btnPin2, INPUT_PULLUP);

  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);   
  Wire.endTransmission(true);
  Serial.begin(9600);
  bluetooth.begin(9600);
}
void loop() {
  int resultBtn1 = digitalRead(btnPin1);
  int resultBtn2 = digitalRead(btnPin2);

  if (precBtn1 == HIGH && resultBtn1 == LOW)
  {
    digitalWrite(yellowLedPin, HIGH);
    startBatch();
  }


  if (precBtn2 == HIGH && resultBtn2 == LOW)
  {
    isBluetoothEnabled = !isBluetoothEnabled;
  }

 
  if (isBluetoothEnabled)
  {
    digitalWrite(redLedPin, HIGH);
  } else {
    digitalWrite(redLedPin, LOW);
  }

  if (resultBtn1 == LOW)
  {
     Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

 
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

 
    if (isBluetoothEnabled)
    {
      bluetooth.print("START");
      bluetooth.print(" "); bluetooth.print(AcX);
      bluetooth.print(" "); bluetooth.print(AcY);
      bluetooth.print(" "); bluetooth.print(AcZ);
      bluetooth.print(" "); bluetooth.print(GyX);
      bluetooth.print(" "); bluetooth.print(GyY);
      bluetooth.print(" "); bluetooth.print(GyZ);
      bluetooth.println(" END");
    } else {
      Serial.print("START");
      Serial.print(" "); Serial.print(AcX);
      Serial.print(" "); Serial.print(AcY);
      Serial.print(" "); Serial.print(AcZ);
      Serial.print(" "); Serial.print(GyX);
      Serial.print(" "); Serial.print(GyY);
      Serial.print(" "); Serial.print(GyZ);
      Serial.println(" END");
    }

  }

  if (precBtn1 == LOW && resultBtn1 == HIGH)
  {
    digitalWrite(yellowLedPin, LOW);
    closeBatch();
  }

  precBtn1 = resultBtn1;
  precBtn2 = resultBtn2;
}

void startBatch()
{
  if (isBluetoothEnabled)
  {
    bluetooth.println("STARTING BATCH");
  } else {
    Serial.println("STARTING BATCH");
  }
}

void closeBatch()
{
  if (isBluetoothEnabled)
  {
    bluetooth.println("CLOSING BATCH");
  } else {
    Serial.println("CLOSING BATCH");
  }
}

