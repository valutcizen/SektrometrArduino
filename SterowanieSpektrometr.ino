#include <Servo.h>
#include <Encoder.h>
#include "SPTPP.h"

//Konfiguracja pinów sterowania
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
#define PinI4 A5
#define PinO1 A2
#define PinO2 A3
#define PinO3 A4
#define PinO4 13
//Konfiguracja pozycjonowania
#define Pozycjonowanie1Lewo
#define Pozycjonowanie2Lewo
#define PredkoscPozycjonowanie1 20
#define PredkoscPozycjonowanie2 20
#define Pozycjonowanie1Timeout 200
#define Pozycjonowanie2Timeout 200
//Konfiguracja sterowania - Kp numerator, Kp denumerator, Ki, Ti in miliseconds
#define Kn1 10
#define Kd1 100
#define Ki1 1
#define Ti1 20
#define Kn2 10
#define Kd2 100
#define Ki2 1
#define Ti2 20

//PROGRAM
Servo serwoLusterko;
Encoder enkoder1(serwo1EnkoderAPin, serwo1EnkoderBPin);
Encoder enkoder2(serwo2EnkoderAPin, serwo2EnkoderBPin);

#define StatusBitPozycjonowanie1 0
#define StatusBitPozycjonowanie2 1
#define StatusBitJazdaLewo1 2
#define StatusBitJazdaPrawo1 3
#define StatusBitJazdaLewo2 4
#define StatusBitJazdaPrawo2 5
#define StatusBitPIActive1 6
#define StatusBitPIActive2 7

struct DaneKomunikacja {
  volatile uint8_t Status = 0;
  volatile int32_t AktualneImpulsy1 = 0;
  volatile int32_t AktualneImpulsy2 = 0;
  volatile uint8_t PortyIO = 0;
  volatile uint8_t UstawionyKatLusterko = 0;
  volatile int32_t UstawioneImpulsy1 = 0;
  volatile int32_t UstawioneImpulsy2 = 0;
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
  RozpocznijPozycjonowanieSerwGalek();
}

int serwoLusterkoPozycja = 0;
unsigned long lastReadTime = 0;

void loop() {
  serwoLusterkoPozycja = constrain(Dane.UstawionyKatLusterko, 0, 180);
  if (serwoLusterko.read() != serwoLusterkoPozycja)
    serwoLusterko.write(serwoLusterkoPozycja);

  ObsluzSerwaGalek();

  ObsluzIO();
  
  delay(20);
}

unsigned long serwo1LastChange = 0;
unsigned long serwo2LastChange = 0;
inline void RozpocznijPozycjonowanieSerwGalek()
{
  bitSet(Dane.Status, StatusBitPozycjonowanie1);
  bitSet(Dane.Status, StatusBitPozycjonowanie2);

#ifdef Pozycjonowanie1Lewo
  bitSet(Dane.Status, StatusBitJazdaLewo1);
  bitClear(Dane.Status, StatusBitJazdaPrawo1);
  digitalWrite(serwo1KierunekLewoPin, HIGH);
  digitalWrite(serwo1KierunekPrawoPin, LOW);
#else
  bitClear(Dane.Status, StatusBitJazdaLewo1);
  bitSet(Dane.Status, StatusBitJazdaPrawo1);
  digitalWrite(serwo1KierunekLewoPin, LOW);
  digitalWrite(serwo1KierunekPrawoPin, HIGH);
#endif

#ifdef Pozycjonowanie2Lewo
  bitSet(Dane.Status, StatusBitJazdaLewo2);
  bitClear(Dane.Status, StatusBitJazdaPrawo2);
  digitalWrite(serwo2KierunekLewoPin, HIGH);
  digitalWrite(serwo2KierunekPrawoPin, LOW);
#else
  bitClear(Dane.Status, StatusBitJazdaLewo2);
  bitSet(Dane.Status, StatusBitJazdaPrawo2);
  digitalWrite(serwo2KierunekLewoPin, LOW);
  digitalWrite(serwo2KierunekPrawoPin, HIGH);
#endif

  analogWrite(serwo1PwmPin, PredkoscPozycjonowanie1);
  analogWrite(serwo2PwmPin, PredkoscPozycjonowanie2);

  lastReadTime = millis();
  serwo1LastChange = lastReadTime;
  serwo2LastChange = lastReadTime;
}

unsigned long noweReadTime = 0;
unsigned long roznicaCzasu = 0;
int32_t staraPozycja1 = 0;
int32_t staraPozycja2 = 0;
int32_t roznicaPozycja1 = 0;
int32_t roznicaPozycja2 = 0;
int32_t e = 0;
unsigned long dt = 0;
int32_t predkosc = 0;
int16_t state1 = 0;
int16_t state2 = 0;

inline void ObsluzSerwaGalek()
{
  noweReadTime = millis();
  if (noweReadTime == lastReadTime)
    return;

  roznicaCzasu = noweReadTime - lastReadTime;
  lastReadTime = noweReadTime;
  staraPozycja1 = Dane.AktualneImpulsy1;
  staraPozycja2 = Dane.AktualneImpulsy2;
  Dane.AktualneImpulsy1 = enkoder1.read();
  Dane.AktualneImpulsy2 = enkoder2.read();
  roznicaPozycja1 = Dane.AktualneImpulsy1 - staraPozycja1;
  roznicaPozycja2 = Dane.AktualneImpulsy2 - staraPozycja2;

  if (bitRead(Dane.Status, StatusBitPozycjonowanie1))
  {
    if (roznicaPozycja1 == 0)
    {
      if (noweReadTime - serwo1LastChange > Pozycjonowanie1Timeout)
        ZakonczPozycjonowanie1();
    }
    else
      serwo1LastChange = noweReadTime;
  }
  else
  {
    e = Dane.UstawioneImpulsy1 - Dane.AktualneImpulsy1;
    if (e == 0)
      UstawSerwo1(0, false);
    else if (abs(e) * Kn1 / Kd1 > Dane.PredkoscMax1)
    {
      UstawSerwo1(Dane.PredkoscMax1, e > 0);
      state1 = 0;
    }
    else
    {
      dt = noweReadTime - serwo1LastChange;
      state1 = constrain(state1 + e * dt, INT16_MIN, INT16_MAX);
      predkosc = e * Kn1 / Kd1 + state1 * Ki1 / Ti1;
      UstawSerwo1(abs(predkosc), predkosc > 0);
    }
    serwo1LastChange = noweReadTime;
  }

  if (bitRead(Dane.Status, StatusBitPozycjonowanie2))
  {
    if (roznicaPozycja2 == 0)
    {
      if (noweReadTime - serwo2LastChange > Pozycjonowanie2Timeout)
        ZakonczPozycjonowanie2();
    }
    else
      serwo2LastChange = noweReadTime;
  }
  else
  {
    e = Dane.UstawioneImpulsy2 - Dane.AktualneImpulsy2;
    if (e == 0)
      UstawSerwo2(0, false);
    else if (abs(e) * Kn2 / Kd2 > Dane.PredkoscMax2)
    {
      UstawSerwo2(Dane.PredkoscMax2, e > 0);
      state2 = 0;
    }
    else
    {
      dt = noweReadTime - serwo2LastChange;
      state2 = constrain(state2 + e * dt, INT16_MIN, INT16_MAX);
      predkosc = e * Kn2 / Kd2 + state2 * Ki2 / Ti2;
      UstawSerwo2(abs(predkosc), predkosc > 0);
    }
    serwo2LastChange = noweReadTime;
  }
}

inline void UstawSerwo1(int predkosc, bool czyLewo)
{
  analogWrite(serwo1PwmPin, predkosc);
  if (predkosc == 0)
  {
    digitalWrite(serwo1KierunekLewoPin, LOW);
    digitalWrite(serwo1KierunekPrawoPin, LOW);
  }
  else if (czyLewo)
  {
    digitalWrite(serwo1KierunekLewoPin, HIGH);
    digitalWrite(serwo1KierunekPrawoPin, LOW);
  }
  else
  {
    digitalWrite(serwo1KierunekLewoPin, LOW);
    digitalWrite(serwo1KierunekPrawoPin, HIGH);
  }
}

inline void UstawSerwo2(int predkosc, bool czyLewo)
{
  analogWrite(serwo2PwmPin, predkosc);
  if (predkosc == 0)
  {
    digitalWrite(serwo2KierunekLewoPin, LOW);
    digitalWrite(serwo2KierunekPrawoPin, LOW);
  }
  else if (czyLewo)
  {
    digitalWrite(serwo2KierunekLewoPin, HIGH);
    digitalWrite(serwo2KierunekPrawoPin, LOW);
  }
  else
  {
    digitalWrite(serwo2KierunekLewoPin, LOW);
    digitalWrite(serwo2KierunekPrawoPin, HIGH);
  }
}

inline void ZakonczPozycjonowanie1()
{
  analogWrite(serwo1PwmPin, 0);
  bitClear(Dane.Status, StatusBitJazdaLewo1);
  bitClear(Dane.Status, StatusBitJazdaPrawo1);
  digitalWrite(serwo1KierunekLewoPin, LOW);
  digitalWrite(serwo1KierunekPrawoPin, LOW);
  enkoder1.write(0);
  serwo1LastChange = noweReadTime;
}

inline void ZakonczPozycjonowanie2()
{
  analogWrite(serwo2PwmPin, 0);
  bitClear(Dane.Status, StatusBitJazdaLewo2);
  bitClear(Dane.Status, StatusBitJazdaPrawo2);
  digitalWrite(serwo2KierunekLewoPin, LOW);
  digitalWrite(serwo2KierunekPrawoPin, LOW);
  enkoder2.write(0);
  serwo2LastChange = noweReadTime;
}

void serialEvent()
{
  while (Serial.available())
    DaneProtokol.GetedByte((uint8_t)Serial.read());
}

#define ReadPin(value, pin, b) if (digitalRead((pin))) bitSet((value), (b)); else bitClear((value), (b));
#define WritePin(value, pin, b) if (bitRead((value),(b))) digitalWrite((pin), HIGH); else digitalWrite((pin), LOW);
inline void ObsluzIO()
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

