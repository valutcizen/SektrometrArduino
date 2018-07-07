#include <Servo.h>
#include <Encoder.h>
#include "SPTPP.h"

//KONFIGURACJA
#define serwoLusterkoPin 2
#define serwo1PwmPin 4
#define serwo1KierunekLewoPin 6
#define serwo1KierunekPrawoPin 7
#define serwo1EnkoderAPin 18
#define serwo1EnkoderBPin 19
#define serwo2PwmPin 5
#define serwo2KierunekLewoPin 8
#define serwo2KierunekPrawoPin 9
#define serwo2EnkoderAPin 20
#define serwo2EnkoderBPin 21

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
  volatile uint8_t UstawionyKatLusterko = 0;
  volatile int16_t AktualneImpulsy1 = 0;
  volatile int16_t AktualneImpulsy2 = 0;
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

  digitalWrite(serwo1EnkoderAPin, HIGH);
  digitalWrite(serwo1EnkoderBPin, HIGH);
  digitalWrite(serwo2EnkoderAPin, HIGH);
  digitalWrite(serwo2EnkoderBPin, HIGH);

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

  //Dyskretny kontroler PI: https://www.embeddedrelated.com/showarticle/121.php
  
  delay(20);
}

void serialEvent(){
  while (Serial.available())
    DaneProtokol.GetedByte((uint8_t)Serial.read());
}
