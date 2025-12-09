#include <QTRSensors.h>
#include <EEPROM.h>

// Motor A
int PWMA = 5;  // Speed control
int AIN1 = 6;  // Direction
int AIN2 = 4;  // Direction

// Motor B
int PWMB = 3;  // Speed control
int BIN1 = 8;  // Direction
int BIN2 = 7;  // Direction

int dato = 0;
float KP = 0;
float KD = 0;
int Velocidad = 0;
int Velocidad2 = 0;
int BOTON = 12;
int led = 10;

#define NUM_SENSORS             8  // número de sensores utilizados
#define NUM_SAMPLES_PER_SENSOR  4  // promedio de 4 muestras analógicas por lectura del sensor
#define EMITTER_PIN             13 // emisor es controlado por pin digital 13

QTRSensors qtra;

unsigned int sensorValues[NUM_SENSORS];
int lastError = 0;
int valor = LOW;
int cont = 0;

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BOTON, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT); // Agregado para evitar error

  // Selección de velocidad
  for (int i = 0; i < 10; i++) {
    digitalWrite(led, HIGH);
    digitalWrite(13, HIGH);
    while (digitalRead(BOTON)) {}
    if (valor == LOW) {
      cont++;
      digitalWrite(led, LOW);
      digitalWrite(13, LOW);
      delay(500);
      valor = HIGH;
    }
  }

  digitalWrite(led, HIGH);
  digitalWrite(13, HIGH);
  delay(500);

  for (int i = 0; i < cont; i++) {
    digitalWrite(led, HIGH);
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    digitalWrite(13, LOW);
    delay(500);
  }

  switch (cont) {
    case 1:
      KP = 0.09; KD = 0.8; Velocidad2 = 130; Velocidad = 110; break;
    case 2:
      KP = 0.1; KD = 0.9; Velocidad2 = 150; Velocidad = 125; break;
    case 3:
      KP = 0.15; KD = 0.9; Velocidad2 = 170; Velocidad = 150; break;
    case 4:
      KP = 0.14; KD = 0.95; Velocidad2 = 200; Velocidad = 180; break;
    case 5:
      KP = 0.17; KD = 1.0; Velocidad2 = 210; Velocidad = 190; break;
    case 6:
      KP = 0.19; KD = 1.1; Velocidad2 = 220; Velocidad = 200; break;
    case 7:
      KP = 0.19; KD = 1.1; Velocidad2 = 250; Velocidad = 200; break;
    case 8:
      KP = 0.19; KD = 1.1; Velocidad2 = 255; Velocidad = 230; break;
  }

  delay(3000);
  digitalWrite(led, HIGH);
  digitalWrite(13, HIGH);
  while (digitalRead(BOTON));
  delay(500);
  digitalWrite(led, LOW);
  digitalWrite(13, LOW);

  // Configuración de los sensores
  unsigned char sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Pines analógicos
  qtra.setTypeAnalog();
  qtra.setSensorPins(sensorPins, NUM_SENSORS);

  for (int i = 0; i < 100; i++) {
    digitalWrite(led, HIGH);
    digitalWrite(13, HIGH);
    delay(20);
    qtra.calibrate();
    digitalWrite(led, LOW);
    digitalWrite(13, LOW);
    delay(20);
  }

  Serial.begin(9600);
  Serial.setTimeout(1);
  delay(20);
  Serial.println("READY");

  digitalWrite(led, HIGH);
  while (digitalRead(BOTON));
  delay(500);
  digitalWrite(11, HIGH);
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(11, LOW);
  digitalWrite(13, LOW);
  delay(200);
}

void loop() {
  // Lee los valores brutos de los sensores
  qtra.read(sensorValues);

  // Calcula la posición de la línea (ajusta según el color del fondo)
  int position = qtra.readLineBlack(sensorValues); // o readLineWhite()

  // Calcula el error
  int error = position - 3500; // 3500 es el centro para 8 sensores calibrados
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;


  int rightMotorSpeed = Velocidad2 + motorSpeed;
  int leftMotorSpeed = Velocidad2 - motorSpeed;

  rightMotorSpeed = constrain(rightMotorSpeed, 0, Velocidad);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, Velocidad);

  move(1, rightMotorSpeed, 1);
  move(2, leftMotorSpeed, 1);

  while (Serial.available() > 0) {
    dato = Serial.read();

    if (dato == 'Q') KP += 0.01;
    if (dato == 'A' && KP >= 0.01) KP -= 0.01;
    if (dato == 'W') KD += 0.01;
    if (dato == 'S' && KD >= 0.01) KD -= 0.01;
    if (dato == 'E' && Velocidad < 255) Velocidad++;
    if (dato == 'D' && Velocidad > 0) Velocidad--;
    if (dato == 'R' && Velocidad2 < 255) Velocidad2++;
    if (dato == 'F' && Velocidad2 > 0) Velocidad2--;

    if (dato == 'P') {
      EEPROM.write(1, KP * 100);
      EEPROM.write(2, KD * 100);
      EEPROM.write(3, Velocidad);
      EEPROM.write(4, Velocidad2);
    }

    if (dato == 'X') Velocidad = 0;
    if (dato == 'C') {
      KP = 0.55;
      KD = 0.7;
      Velocidad = 150;
    }

    Serial.print("Kp = ");
    Serial.print(KP);
    Serial.print(" Kd = ");
    Serial.print(KD);
    Serial.print(" Vel = ");
    Serial.print(Velocidad);
    Serial.print(" Vel2 = ");
    Serial.println(Velocidad2);
  }
}

void move(int motor, int speed, int direction) {
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 2) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  } else {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}
