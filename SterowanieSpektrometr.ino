#include <Servo.h>
#include "SPTPP.h"

//KONFIGURACJA
#define serwoLusterkoPin 2
#define serwo1PwmPin 3
#define serwo1KierunekLewoPin 5
#define serwo1KierunekPrawoPin 6
#define serwo1EnkoderAPin 9
#define serwo1EnkoderBPin 10
#define serwo2PwmPin 4
#define serwo2KierunekLewoPin 7
#define serwo2KierunekPrawoPin 8
#define serwo2EnkoderAPin 11
#define serwo2EnkoderBPin 12

//PROGRAM
Servo serwoLusterko;

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
  pinMode(serwo1EnkoderAPin, INPUT);
  pinMode(serwo1EnkoderBPin, INPUT);
  pinMode(serwo2PwmPin, OUTPUT);
  pinMode(serwo2KierunekLewoPin, OUTPUT);
  pinMode(serwo2KierunekPrawoPin, OUTPUT);
  pinMode(serwo2EnkoderAPin, INPUT);
  pinMode(serwo2EnkoderBPin, INPUT);

  Serial.begin(115200);
  serwoLusterko.attach(serwoLusterkoPin);
}

int serwoLusterkoPozycja = 0;
uint16_t serwo1Pwm = 90;
uint16_t serwo2Pwm = 90;

void loop() {
  serwoLusterkoPozycja = constrain(Dane.UstawionyKatLusterko, 0, 180);
  serwoLusterko.write(serwoLusterkoPozycja);
  delay(15);
}

void serialEvent(){
  while (Serial.available())
    DaneProtokol.GetedByte((uint8_t)Serial.read());
}