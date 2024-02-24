#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

#include <Keypad.h>

#define ROW_NUM     4 // four rows
#define COLUMN_NUM  4 // four columns

const int pinClock = 12;  // Clock pin
const int pinData = 13;   // Data pin
const int pinLatch = 14;  // Latch pin

const int cd4028Outputs[8] = {15, 16, 17, 18, 19, 21, 22, 23}; // Connect Y0 - Y7 to your output devices (e.g., LEDs)

char keys[ROW_NUM][COLUMN_NUM] = {
  {'A', 'B', 'C', 'D'},
  {'E', 'F', 'G', 'H'},  
  {'I', 'J', 'K', 'L'},
  {'M', 'N', 'O', 'P'}
};
//D1=A,D2=B,D3=C,D4=D,D5=E,D6=F,D7=G,D8=H,F0=I,F1=J,F2=K,RO=L,R1=M,R2=N,AMP=O,SUB=P
const int F1=19,F2=18,FO=5,R1=17,R2=16,RO=4,AMP=2,SUB=15;;


byte pin_rows[ROW_NUM]      = {13, 12, 14, 27}; // GPIO13, GPIO12, GPI14, GPIO27 connect to the row pins
byte pin_column[COLUMN_NUM] = {26, 25, 33, 32};   // GPIO26, GPIO25, GPIO23, GPIO32 connect to the column pins

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

// const int ledRowPins[ROW_NUM] = {23, 22, 1, 3};
// const int ledColPins[COLUMN_NUM] = {21, 19, 18, 5}; 

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Esp-32-Test");
  pinMode(F1, OUTPUT);
  pinMode(F2, OUTPUT);
  pinMode(FO, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(RO, OUTPUT); 
  pinMode(AMP, OUTPUT);
  pinMode(SUB, OUTPUT);
  digitalWrite(F1,HIGH);
  digitalWrite(R1,HIGH);
  digitalWrite(AMP,HIGH);

  pinMode(pinClock, OUTPUT);
  pinMode(pinData, OUTPUT);
  pinMode(pinLatch, OUTPUT);

  // Set CD4028 output pins as outputs
  for (int i = 0; i < 8; i++) {
    pinMode(cd4028Outputs[i], OUTPUT);
  }

  setPinHigh(0);

}
void loop() {
  char msg;
  if(SerialBT.available()){
    msg=SerialBT.read();
    Serial.println(msg);
  }
  // displayPattern(5);
  //if(SerialBT.available()){}
    // char message=SerialBT.read();
  char key = keypad.getKey();
  // if (key) {
  //   Serial.println(key);
  // }
  if(key=='A' || msg=='A'){
    Serial.println(key);
    
    setPinLow(1);
    setPinLow(2);
    setPinLow(3);
    setPinLow(4);
    setPinLow(5);
    setPinLow(6);
    setPinLow(7);

    setPinHigh(0);
  }
  if(key=='B' || msg=='B'){
    Serial.println(key);

    setPinLow(0);
    setPinLow(2);
    setPinLow(3);
    setPinLow(4);
    setPinLow(5);
    setPinLow(6);
    setPinLow(7);

    setPinHigh(1);
  }
  if(key=='C' || msg=='C'){
    Serial.println(key);

    setPinLow(1);
    setPinLow(0);
    setPinLow(3);
    setPinLow(4);
    setPinLow(5);
    setPinLow(6);
    setPinLow(7);

    setPinHigh(2);
  }
  if(key=='D' || msg=='D'){
    Serial.println(key);

    setPinLow(1);
    setPinLow(2);
    setPinLow(0);
    setPinLow(4);
    setPinLow(5);
    setPinLow(6);
    setPinLow(7);
    
    setPinHigh(3);
  }
  if(key=='E' || msg=='E'){
    Serial.println(key);

    setPinLow(1);
    setPinLow(2);
    setPinLow(3);
    setPinLow(0);
    setPinLow(5);
    setPinLow(6);
    setPinLow(7);

    setPinHigh(4);
  }

  if(key=='F' || msg=='F'){
    Serial.println(key);

    setPinLow(1);
    setPinLow(2);
    setPinLow(3);
    setPinLow(4);
    setPinLow(0);
    setPinLow(6);
    setPinLow(7);

    setPinHigh(5);
  }

  if(key=='G' || msg=='G'){
    Serial.println(key);

    setPinLow(1);
    setPinLow(2);
    setPinLow(3);
    setPinLow(4);
    setPinLow(5);
    setPinLow(0);
    setPinLow(7);

    setPinHigh(6);
  }

  if(key=='H' || msg=='H'){
    Serial.println(key);

    setPinLow(1);
    setPinLow(2);
    setPinLow(3);
    setPinLow(4);
    setPinLow(5);
    setPinLow(6);
    setPinLow(0);
    
    setPinHigh(7);
  }
  if(key=='I' || msg=='I'){
    Serial.println(key);
      digitalWrite(F1,LOW);
      digitalWrite(F2,LOW);
      digitalWrite(FO,HIGH);
  }

  if(key=='J' || msg=='J'){
    Serial.println(key);
      digitalWrite(F1,HIGH);
      digitalWrite(F2,LOW);
      digitalWrite(FO,LOW);
  }
  if(key=='K' || msg=='K'){
    Serial.println(key);
      digitalWrite(F1,LOW);
      digitalWrite(FO,LOW);
      digitalWrite(F2,HIGH);
  }
  
  if(key=='M' || msg=='M'){
    Serial.println(key);
      digitalWrite(R1,HIGH);
      digitalWrite(R2,LOW);
      digitalWrite(RO,LOW);
  }
  if(key=='N' || msg=='N'){
    Serial.println(key);
      digitalWrite(R1,LOW);
      digitalWrite(R2,HIGH);
      digitalWrite(RO,LOW);
  }
  if(key=='L' || msg=='L'){
    Serial.println(key);
      digitalWrite(R1,LOW);
      digitalWrite(R2,LOW);
      digitalWrite(RO,HIGH);
  }
  if(key=='O' || msg=='O'){
    Serial.println(key);
      digitalWrite(AMP,HIGH);
      digitalWrite(SUB,LOW);
  }
  if(key=='P' || msg=='P'){
    Serial.println(key);
      digitalWrite(AMP,LOW);
      digitalWrite(SUB,HIGH);
  }
     
  
  
}

void setPinHigh(int pinIndex) {
  // Set latch pin LOW to begin data transfer
  digitalWrite(pinLatch, LOW);

  // Shift out the pattern using bit-banging
  for (int i = 0; i < 4; i++) {
    // Write the data bit
    digitalWrite(pinData, bitRead(pinIndex, i));
    
    // Pulse the clock pin
    digitalWrite(pinClock, HIGH);
    delayMicroseconds(1); // Adjust as necessary
    digitalWrite(pinClock, LOW);
  }
  
  // Set latch pin HIGH to update outputs
  digitalWrite(pinLatch, HIGH);
}

void setPinLow(int pinIndex) {
  digitalWrite(pinLatch, LOW);

  // Shift out the pattern using bit-banging
  for (int i = 0; i < 4; i++) {
    // Write the data bit (all low to turn off all outputs)
    digitalWrite(pinData, LOW);
    
    // Pulse the clock pin
    digitalWrite(pinClock, HIGH);
    delayMicroseconds(1); // Adjust as necessary
    digitalWrite(pinClock, LOW);
  }
  
  // Set latch pin HIGH to update outputs
  digitalWrite(pinLatch, HIGH);
}
