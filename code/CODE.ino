// Bibliotheken
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// Instelbare magnitude die invloed heeft op de snelheid van de motor
float magnitude = 8;

// Berekent een vertraging op basis van de ingestelde magnitude
float calcDelay(float magnitude) {
  float k = 0.69;
  float delayValue = exp(k * (9.0 - magnitude));
  return delayValue;
}

Adafruit_MPU6050 mpu;
int teller = 0;

// Referentiewaarden voor kalibratie van de hoekmetingen
float refX = 0;
float refY = 0;
bool gekalibreerd = false;

// snelheden in X- en Y-richting
float vx = 0;
float vy = 0;
unsigned long vorigeTijd = 0;

// Pinconfiguratie voor beide stappenmotoren
int motor1Pins[4] = {4, 5, 6, 7};
int motor2Pins[4] = {8, 9, 10, 11};

// Sequentie voor het aansturen van de stappenmotor
int steps[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

int stepIndex1 = 0;
int stepIndex2 = 0;

int dir1 = 1;
int dir2 = 1;

void setup() {
  Serial.begin(115200);

  // Initialisatie van de MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("MPU6050 niet gevonden!");
    while (1) delay(10);
  }

  // Instellen van motorpinnen als output
  for (int i = 0; i < 4; i++) {
    pinMode(motor1Pins[i], OUTPUT);
    pinMode(motor2Pins[i], OUTPUT);
  }

  vorigeTijd = millis();

  // Kleine initiële beweging om motor in referentiepositie te brengen
  for (int i = 0; i < 40; i++) {
    stepMotor(motor2Pins, stepIndex2, -1);
    delay(5);
  }

  delay(1000);
}

// Stuurt de stappenmotor één stap vooruit of achteruit volgens de sequentie
void stepMotor(int motorPins[4], int &index, int dir) {
  index += dir;

  if (index >= 8) index = 0;
  if (index < 0) index = 7;

  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], steps[index][i]);
  }
}

void loop() {
  // Bepalen van vertraging op basis van magnitude
  float d = calcDelay(magnitude);

  // Motoren laten stappen met vaste snelheid
  for (int i = 0; i < 300; i++) {
    stepMotor(motor1Pins, stepIndex1, dir1);
    stepMotor(motor2Pins, stepIndex2, dir2);
    delay(d);
  }

  // Sensorwaarden uitlezen
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long huidigeTijd = millis();
  float dt = (huidigeTijd - vorigeTijd) / 1000.0;
  vorigeTijd = huidigeTijd;

  // Hoekberekening uit accelerometerdata
  float hoekX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float hoekY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // Eerste meting gebruiken als nulreferentie (kalibratie)
  if (!gekalibreerd) {
    refX = hoekX;
    refY = hoekY;
    gekalibreerd = true;
  }

  // Relatieve hoek t.o.v. startpositie
  float relX = hoekX - refX;
  float relY = hoekY - refY;

  // Versnellingen corrigeren voor zwaartekrachtcomponent
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;

  float ax_corr = ax - 9.81 * sin(relY * PI / 180);
  float ay_corr = ay - 9.81 * sin(relX * PI / 180);

  // Simpele integratie naar snelheid
  vx = ax_corr * dt;
  vy = ay_corr * dt;

  // Demping om ruis en drift te verminderen
  vx *= 0.99;
  vy *= 0.99;

  // Data naar Serial Monitor sturen
  Serial.print(millis() / 1000.0);
  Serial.print(",");
  Serial.print(relX);
  Serial.print(",");
  Serial.print(relY);
  Serial.print(",");
  Serial.print(vx);
  Serial.print(",");
  Serial.println(vy);

  teller++;

  if (teller == 24) {
    Serial.println("copybreak");
    teller = 0;
  }
}
