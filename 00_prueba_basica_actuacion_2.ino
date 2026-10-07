const bool ENCODER_CONECTADO = false;
const bool CSA_CONECTADO     = false;

const int PIN_ENA = 9, PIN_IN1 = 7, PIN_IN2 = 8;
const int PIN_ENCA = 2, PIN_ENCB = 3;
const int PIN_CSA = A1;
const float R_SENSE = 0.5;

const int PWM_PRUEBA = 25;
const int PWM_MAX    = 40;
const float RELACION = 30.0;
const unsigned long T_MUESTRA = 100;

volatile long cuentas = 0;
volatile uint8_t estPrev = 0;
const int8_t QDEC[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
void isrEnc() {
  uint8_t e = (digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB);
  cuentas += QDEC[(estPrev << 2) | e];
  estPrev = e;
}

int pasoActual = 0;
const char *descActual = "inicio";
int sentidoActual = 0;
int pwmActual = 0;
unsigned long t0 = 0;
bool abortado = false;

void detener() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  sentidoActual = 0; pwmActual = 0;
}

void girar(int sentido, int pct) {
  pct = constrain(pct, 0, PWM_MAX);
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, sentido > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, sentido > 0 ? LOW  : HIGH);
  analogWrite(PIN_ENA, map(pct, 0, 100, 0, 255));
  sentidoActual = sentido; pwmActual = pct;
}

void filaRegistro() {
  long c = 0;
  if (ENCODER_CONECTADO) { noInterrupts(); c = cuentas; interrupts(); }
  Serial.print(millis() - t0);  Serial.print(',');
  Serial.print(pasoActual);     Serial.print(',');
  Serial.print(descActual);     Serial.print(',');
  Serial.print(sentidoActual);  Serial.print(',');
  Serial.print(pwmActual);      Serial.print(',');
  if (ENCODER_CONECTADO) Serial.print(c); else Serial.print(F("NA"));
  Serial.print(',');
  if (CSA_CONECTADO) Serial.print(analogRead(PIN_CSA) * 5.0 / 1023.0 / R_SENSE, 3);
  else Serial.print(F("NA"));
  Serial.println();
}

bool esperar(unsigned long ms) {
  unsigned long ini = millis(), ult = 0;
  bool primera = true;
  while (millis() - ini < ms) {
    if (Serial.available() && Serial.read() == 's') {
      detener(); abortado = true;
      descActual = "ABORTADO_POR_USUARIO"; filaRegistro();
      return false;
    }
    if (primera || millis() - ult >= T_MUESTRA) { primera = false; ult = millis(); filaRegistro(); }
  }
  return true;
}

void fijarPaso(int n, const char *d) { pasoActual = n; descActual = d; }

bool rampa() {
  for (int p = 20; p <= PWM_MAX; p += 5) { girar(+1, p); if (!esperar(1500)) return false; }
  for (int p = PWM_MAX; p >= 20; p -= 5) { girar(+1, p); if (!esperar(800))  return false; }
  return true;
}

void secuencia() {
  abortado = false;
  noInterrupts(); cuentas = 0; interrupts();
  t0 = millis();

  Serial.println(F("#INICIO_REGISTRO"));
  Serial.println(F("#proyecto=IRB2002 Grupo 2 - Guante de rehabilitacion"));
  Serial.println(F("#prueba=Prueba basica de actuacion"));
  Serial.println(F("#placa=Arduino UNO R4 WiFi"));
  Serial.println(F("#driver=CanaKit UK1122 (canal A)"));
  Serial.println(F("#motor=Pololu 37D 12V 30:1 (item 4752)"));
  Serial.print(F("#relacion=")); Serial.println(RELACION, 1);
  Serial.print(F("#cuentas_por_vuelta_salida=")); Serial.println(64.0 * RELACION, 0);
  Serial.print(F("#encoder_conectado=")); Serial.println(ENCODER_CONECTADO ? 1 : 0);
  Serial.print(F("#csa_conectado=")); Serial.println(CSA_CONECTADO ? 1 : 0);
  Serial.print(F("#periodo_muestreo_ms=")); Serial.println(T_MUESTRA);
  Serial.println(F("t_ms,paso,descripcion,sentido,pwm_pct,cuentas,corriente_A"));

  detener();
  bool ok = true;
  fijarPaso(0, "espera_inicial");                           ok = esperar(3000);
  if (ok) { fijarPaso(1, "sentido_A_PWM_bajo"); girar(+1, PWM_PRUEBA); ok = esperar(2000); }
  if (ok) { fijarPaso(2, "stop");               detener();             ok = esperar(1000); }
  if (ok) { fijarPaso(3, "sentido_B_PWM_bajo"); girar(-1, PWM_PRUEBA); ok = esperar(2000); }
  if (ok) { fijarPaso(4, "stop");               detener();             ok = esperar(1000); }
  if (ok) { fijarPaso(5, "rampa_sentido_A");    ok = rampa(); }
  if (ok) { fijarPaso(6, "stop_final");         detener();             esperar(500); }

  detener();
  Serial.println(abortado ? F("#resultado=ABORTADO") : F("#resultado=COMPLETADO"));
  Serial.println(F("#FIN_REGISTRO"));
  Serial.println(F("Copia TODO el Monitor Serie a registro.txt y ejecuta generar_reporte.py. 'r' repite."));
}

void setup() {
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  detener();
  if (ENCODER_CONECTADO) {
    pinMode(PIN_ENCA, INPUT_PULLUP); pinMode(PIN_ENCB, INPUT_PULLUP);
    estPrev = (digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCA), isrEnc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCB), isrEnc, CHANGE);
  }
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  secuencia();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') secuencia();
    else if (c != '\n' && c != '\r') detener();
  }
}
