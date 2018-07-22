#include <Servo.h>
#include <Encoder.h>
#include "SPTPP.h"

//KONFIGURACJA
#define serwoLusterkoPin 10
#define serwo1PwmPin 5
#define serwo1KierunekLewoPin 11
#define serwo1KierunekPrawoPin 12
#define serwo1EnkoderAPin 3
#define serwo1EnkoderBPin 4
#define serwo2PwmPin 6
#define serwo2KierunekLewoPin 8
#define serwo2KierunekPrawoPin 9
#define serwo2EnkoderAPin 2
#define serwo2EnkoderBPin 7
//Konfiguracja uniwersalnych wejść / wyjść
#define PinI1 A0
#define PinI2 A1
#define PinI3 A5
#define PinI4 A6
#define PinO1 A2
#define PinO2 A3
#define PinO3 A4
#define PinO4 13

//PROGRAM
Servo serwoLusterko;
Encoder enkoder1(serwo1EnkoderAPin, serwo1EnkoderBPin);
Encoder enkoder2(serwo2EnkoderAPin, serwo2EnkoderBPin);

#define StatusBitPozycjonowanie1 0
#define StatusBitPozycjonowanie2 1
#define StatusBitJazdaLewo1 2
#define StatusBitJazdaPrawo1 3
#define StatusBitJazdaLewo2 4
#define StatusBitJazdaPrawo3 5
#define StatusBitPIActive1 6
#define StatusBitPIActive2 7

struct DaneKomunikacja {
  volatile uint8_t Status = 0;
  volatile int16_t AktualneImpulsy1 = 0;
  volatile int16_t AktualneImpulsy2 = 0;
  volatile uint8_t PortyIO = 0;
  volatile uint8_t UstawionyKatLusterko = 0;
  volatile int16_t UstawioneImpulsy1 = 0;
  volatile int16_t UstawioneImpulsy2 = 0;
  volatile uint16_t PredkoscMax1 = 0;
  volatile uint16_t PredkoscMax2 = 0;
};

volatile DaneKomunikacja Dane;
SPTPP DaneProtokol((uint8_t*)(&Dane), (int)sizeof(DaneKomunikacja), 20ul);

void setup() {
  pinMode(serwoLusterkoPin, OUTPUT);
  pinMode(serwo1PwmPin, OUTPUT);
  pinMode(serwo1KierunekLewoPin, OUTPUT);
  pinMode(serwo1KierunekPrawoPin, OUTPUT);
  pinMode(serwo1EnkoderAPin, INPUT_PULLUP);
  pinMode(serwo1EnkoderBPin, INPUT_PULLUP);
  pinMode(serwo2PwmPin, OUTPUT);
  pinMode(serwo2KierunekLewoPin, OUTPUT);
  pinMode(serwo2KierunekPrawoPin, OUTPUT);
  pinMode(serwo2EnkoderAPin, INPUT_PULLUP);
  pinMode(serwo2EnkoderBPin, INPUT_PULLUP);

  pinMode(PinI1, INPUT_PULLUP);
  pinMode(PinI2, INPUT_PULLUP);
  pinMode(PinI3, INPUT_PULLUP);
  pinMode(PinI4, INPUT_PULLUP);
  pinMode(PinO1, OUTPUT);
  pinMode(PinO2, OUTPUT);
  pinMode(PinO3, OUTPUT);
  pinMode(PinO4, OUTPUT);

  digitalWrite(serwo1EnkoderAPin, HIGH);
  digitalWrite(serwo1EnkoderBPin, HIGH);
  digitalWrite(serwo2EnkoderAPin, HIGH);
  digitalWrite(serwo2EnkoderBPin, HIGH);
  digitalWrite(PinI1, HIGH);
  digitalWrite(PinI2, HIGH);
  digitalWrite(PinI3, HIGH);
  digitalWrite(PinI4, HIGH);

  Serial.begin(115200);
  serwoLusterko.attach(serwoLusterkoPin, 1, 2);
  enkoder1.write(0);
  enkoder2.write(0);
}

int serwoLusterkoPozycja = 0;

void loop() {
  serwoLusterkoPozycja = constrain(Dane.UstawionyKatLusterko, 0, 180);
  if (serwoLusterko.read() != serwoLusterkoPozycja)
    serwoLusterko.write(serwoLusterkoPozycja);

  Dane.AktualneImpulsy1 = enkoder1.read();
  Dane.AktualneImpulsy2 = enkoder2.read();

  ObsluzIO();

  //Dyskretny kontroler PI: https://www.embeddedrelated.com/showarticle/121.php
  
  delay(20);
}

void serialEvent(){
  while (Serial.available())
    DaneProtokol.GetedByte((uint8_t)Serial.read());
}


#define ReadPin(value, pin, b) if (digitalRead((pin))) bitSet((value), (b)); else bitClear((value), (b));
#define WritePin(value, pin, b) if (bitRead((value),(b))) digitalWrite((pin), HIGH); else digitalWrite((pin), LOW);
void ObsluzIO()
{
  uint8_t x = Dane.PortyIO;
  ReadPin(x,PinI1,0);
  ReadPin(x,PinI2,1);
  ReadPin(x,PinI3,2);
  ReadPin(x,PinI4,3);
  WritePin(x,PinO1,4);
  WritePin(x,PinO2,5);
  WritePin(x,PinO3,6);
  WritePin(x,PinO4,7);

  Dane.PortyIO = x;
}

