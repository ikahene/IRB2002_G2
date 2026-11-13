// Jimmy V9 - controlador del guante de rehabilitacion (IRB2002, Grupo 2)
// Arduino UNO R4 WiFi + L298N + Pololu 37D + celda con HX711 + AS5600 + TFT 1.8"
// Botones: A0 corto = siguiente, A0 largo = volver, A1 = OK, D4 = parada por software
// El monitor serie (115200) muestra eventos y datos en CSV
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
// Fuente 5x7 de Adafruit GFX
#include "glcdfont.c"

// ===== Ajustes: todo lo que se cambia sin tocar el resto del codigo =====
const bool MOTOR_HABILITADO  = true;
const bool ENCODER_INSTALADO = true;
const bool HX711_INSTALADO   = true;
const bool AS5600_INSTALADO  = true;

// Si al "recoger" el motor suelta cable, cambiar a -1
const int8_t MOTOR_SENTIDO_RECOGER = +1;

// D4: false = pulsador normalmente abierto, true = normalmente cerrado
const bool STOP_CONTACTO_NC = false;

// 0 = usar el factor que guardo la calibracion de la celda
const float HX_CUENTAS_POR_N = 0.0f;

// Control automatico. Valores para pruebas tempranas de rutina, fuerzas en N
const float TENSION_OBJETIVO   = 5.0f;    //Modificar para evaluar experiencia de usuario
const float TOLERANCIA_TENSION = 0.25f;
const float TENSION_MAX_SEGURA = 10.0f;   //Modificar para evaluar experiencia de usuario
const int   PWM_MINIMO = 20;              //Modificar para evaluar experiencia de usuario
const int   PWM_MAXIMO = 45;              //Modificar para evaluar experiencia de usuario
const unsigned long TIEMPO_MAX_RECOGIDA = 10000;

// Ajuste del lazo de tension
const float KP_TENSION          = 15.0f;
const float ADELANTO_MAX_N      = 0.6f;
const float RAMPA_CONSIGNA_NPS  = 0.8f;
const float ZONA_APROXIMACION_N = 0.5f;
const unsigned long TIEMPO_EN_BANDA = 400;
const float TENSION_INICIAL_MAX = 0.6f;
const float ALFA_FUERZA = 0.5f;

// Retorno: con false se suelta hasta volver a la tension inicial
const bool  USAR_ENCODER_EN_RETORNO = false;
const int   PWM_RETORNO            = 15;
const float FRACCION_LENTA_RETORNO = 0.20f;
const float FRACCION_FIN_RETORNO   = 0.02f;
const long  ENC_RECORRIDO_MIN      = 40;
const float HOLGURA_RETORNO_N      = 0.2f;
const unsigned long TIEMPO_FLOJO_MS    = 300;
const unsigned long TIEMPO_MAX_RETORNO = 15000;
const float AUMENTO_MAX_RETORNO_N  = 0.5f;

// Movimiento manual (PWM en %)
const uint8_t NIVELES_PWM_MANUAL[] = {20, 25, 30, 35, 40, 45, 50};
const uint8_t N_NIVELES_PWM = sizeof(NIVELES_PWM_MANUAL);
const uint8_t NIVEL_PWM_INICIAL = 2;
const unsigned long MANUAL_MAX_CONTINUO_MS = 8000;

// true solo para probar con el cable suelto del dedo
const bool RECOGER_MANUAL_SIN_CELDA = false;

// Motor: rampa, deteccion de atasco y vigilancia de ordenes
const float RAMPA_PWM_PCT_POR_S = 150.0f;
const unsigned long T_SIN_AVANCE_MS = 700;
const long  ENC_AVANCE_MIN   = 3;
const unsigned long T_ORDEN_MOTOR_MS = 100;
const int   PWM_LIBERAR      = 25;
const unsigned long T_LIBERAR_MAX_MS = 3000;

// Sensores
const unsigned long T_HX_SIN_DATOS_MS = 400;
const float   HX_TASA_MAX_HZ   = 95.0f;
const uint8_t HX_REPETIDAS_MAX = 25;
const uint8_t N_TARA_RAPIDA    = 10;
const unsigned long AS_PERIODO_MS = 50;

// Pantalla
const bool PANTALLA_BGR = false;
const int16_t PANTALLA_OFFSET_X = 0;
const int16_t PANTALLA_OFFSET_Y = 0;
const uint32_t PANTALLA_SPI_HZ = 4000000;
const unsigned long TFT_DATO_MS = 50;

// ===== Tipos =====
enum EventoBoton : uint8_t { BTN_NADA, BTN_CORTO, BTN_LARGO };
struct Boton {
  uint8_t pin;
  bool raw, estable, largoEmitido;
  unsigned long tCambio, tInicio;
};
enum Vista : uint8_t {
  V_BIENVENIDA, V_MENU, V_CONEXIONES, V_DIAG, V_DIAG_AS, V_DIAG_CELDA,
  V_MANUAL_CFG, V_MANUAL, V_AUTO_PREP, V_AUTO, V_AUTO_FIN, V_STOP, V_MENSAJE
};
enum EstadoAS : uint8_t { AS_NO_DETECTADO, AS_SIN_IMAN, AS_CON_IMAN };
enum EstadoHX : uint8_t { HX_SIN_DATOS, HX_OK, HX_SATURADO, HX_INDETERMINADO };
enum EstadoManual : uint8_t {
  MAN_DETENIDO, MAN_RECOGIENDO, MAN_SOLTANDO, MAN_LIMITE, MAN_SIN_CELDA,
  MAN_SIN_AVANCE, MAN_TIEMPO
};
enum FaseAuto : uint8_t { FA_RECOGIDA, FA_RETORNO };
enum ModoRetorno : uint8_t { RET_ENCODER, RET_TENSION };

// Lo que se guarda en la EEPROM (factor en cuentas por gramo)
struct CalibCelda { uint32_t magic; long offset; float factor; };

struct CicloAuto {
  FaseAuto fase;
  ModoRetorno modoRet;
  unsigned long tInicio, tFase, tEnBanda, tFlojo, tMuestraPrev, tLog, tSubida;
  float fInicial, fMax, fFinal, ref, u, fMinRet, fSubida, tRecogidaS;
  long encInicio, encPico;
  bool sobrepaso;
};

// ===== Pines =====
const uint8_t PIN_STOP    = 4;
const uint8_t PIN_TFT_CS  = 10;
const uint8_t PIN_TFT_DC  = 9;
const uint8_t PIN_TFT_RST = 8;
const uint8_t PIN_BOTON_A = A0;
const uint8_t PIN_BOTON_B = A1;
const uint8_t PIN_MOTOR_ENA = 5;
const uint8_t PIN_MOTOR_IN1 = 7;
const uint8_t PIN_MOTOR_IN2 = 6;
const uint8_t PIN_MOTOR_ENCA = 2;
const uint8_t PIN_MOTOR_ENCB = 3;
const uint8_t PIN_HX_DOUT = A2;
const uint8_t PIN_HX_SCK  = A3;
const uint8_t AS5600_ADDR = 0x36;

// Colores (RGB565)
const uint16_t C_NEGRO    = 0x0000;
const uint16_t C_BLANCO   = 0xFFFF;
const uint16_t C_GRIS     = 0x94B2;
const uint16_t C_GRIS_OSC = 0x5ACB;
const uint16_t C_CIAN     = 0x06FF;
const uint16_t C_AMARILLO = 0xFEE0;
const uint16_t C_VERDE    = 0x072C;
const uint16_t C_ROJO     = 0xFA28;
const uint16_t C_NARANJA  = 0xFCA0;

const uint16_t C_UI_BG     = 0x0000;
const uint16_t C_UI_PANEL  = 0x0884;
const uint16_t C_UI_TITULO = 0xD08A;
const uint16_t C_UI_TEXTO  = 0xFE7C;
const uint16_t C_UI_BORDE  = 0xFC96;
const uint16_t C_UI_ACENTO = 0xD08A;
const uint16_t C_UI_PIE    = 0x0884;
const uint16_t C_UI_PIEL   = 0xE696;
const uint16_t C_UI_PIEL_S = 0xA4B0;
const uint16_t C_UI_TRAZO  = 0x734C;
const uint16_t C_UI_BRILLO = 0xEEB7;

// ===== Pantalla: driver propio =====
#define TFT_DELAY 0x80

// Comandos de arranque del controlador de la pantalla
static const uint8_t INIT_PANTALLA[] = {
  19,
  0x01, TFT_DELAY, 50,
  0x11, TFT_DELAY, 100,
  0x26, 1, 0x04,
  0xB1, 2, 0x0B, 0x14,
  0xC0, 2, 0x08, 0x00,
  0xC1, 1, 0x05,
  0xC5, 2, 0x41, 0x30,
  0xC7, 1, 0xC1,
  0xEC, 1, 0x1B,
  0x3A, 1 | TFT_DELAY, 0x55, 100,
  0x2A, 4, 0x00, 0x00, 0x00, 0x7F,
  0x2B, 4, 0x00, 0x00, 0x00, 0x9F,
  0x36, 1, 0xC8,
  0xB7, 1, 0x00,
  0xF2, 1, 0x00,
  0xE0, 15 | TFT_DELAY,
    0x28, 0x24, 0x22, 0x31, 0x2B, 0x0E, 0x53, 0xA5,
    0x42, 0x16, 0x18, 0x12, 0x1A, 0x14, 0x03, 50,
  0xE1, 15 | TFT_DELAY,
    0x17, 0x1B, 0x1D, 0x0E, 0x14, 0x11, 0x2C, 0xA5,
    0x3D, 0x09, 0x27, 0x2D, 0x25, 0x2B, 0x3C, 50,
  0x13, TFT_DELAY, 10,
  0x29, TFT_DELAY, 255
};

class PantallaTFT : public Adafruit_GFX {
public:
  PantallaTFT(uint8_t cs, uint8_t dc, uint8_t rst)
    : Adafruit_GFX(128, 160), _cs(cs), _dc(dc), _rst(rst),
      _spi(PANTALLA_SPI_HZ, MSBFIRST, SPI_MODE0) {}

  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_dc, OUTPUT);
    digitalWrite(_dc, HIGH);
    SPI.begin();
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
    ejecutarLista(INIT_PANTALLA);
    setRotation(1);
    setTextWrap(false);
  }

  // Escribe texto pintando tambien el fondo
  void textoFondo(int16_t x, int16_t y, const char *t, uint16_t fg,
                  uint16_t bg, uint8_t tam = 1, uint8_t ancho = 0) {
    uint8_t n = 0;
    while (*t || n < ancho) {
      char c = *t ? *t++ : ' ';
      caracter(x, y, c, fg, bg, tam);
      x += 6 * tam;
      n++;
    }
  }

  void textoCentrado(int16_t y, const char *t, uint16_t fg, uint16_t bg,
                     uint8_t tam = 1) {
    int16_t x = (_width - (int16_t)strlen(t) * 6 * tam) / 2;
    textoFondo(x < 0 ? 0 : x, y, t, fg, bg, tam);
  }

  void setRotation(uint8_t r) override {
    rotation = r & 3;
    uint8_t madctl;
    switch (rotation) {
      case 0:  madctl = 0xC0; _width = 128; _height = 160; break;
      case 1:  madctl = 0xA0; _width = 160; _height = 128; break;
      case 2:  madctl = 0x00; _width = 128; _height = 160; break;
      default: madctl = 0x60; _width = 160; _height = 128; break;
    }
    if (PANTALLA_BGR) madctl |= 0x08;
    inicioSPI();
    comando(0x36);
    SPI.transfer(madctl);
    finSPI();
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    inicioSPI();
    ventana(x, y, 1, 1);
    SPI.transfer16(color);
    finSPI();
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                uint16_t color) override {
    if (w < 0) { x += w + 1; w = -w; }
    if (h < 0) { y += h + 1; h = -h; }
    if (x >= _width || y >= _height) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > _width)  w = _width - x;
    if (y + h > _height) h = _height - y;
    if (w <= 0 || h <= 0) return;
    inicioSPI();
    ventana(x, y, w, h);
    uint32_t n = (uint32_t)w * h;
    while (n--) SPI.transfer16(color);
    finSPI();
  }

  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    fillRect(x, y, w, 1, color);
  }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    fillRect(x, y, 1, h, color);
  }
  void fillScreen(uint16_t color) override {
    fillRect(0, 0, _width, _height, color);
  }

private:
  uint8_t _cs, _dc, _rst;
  SPISettings _spi;

  void inicioSPI() {
    SPI.beginTransaction(_spi);
    digitalWrite(_cs, LOW);
  }
  void finSPI() {
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
  }
  void comando(uint8_t c) {
    digitalWrite(_dc, LOW);
    SPI.transfer(c);
    digitalWrite(_dc, HIGH);
  }
  // Define la zona donde se van a escribir los pixeles
  void ventana(int16_t x, int16_t y, int16_t w, int16_t h) {
    uint16_t x0 = x + PANTALLA_OFFSET_X, y0 = y + PANTALLA_OFFSET_Y;
    uint16_t x1 = x0 + w - 1,            y1 = y0 + h - 1;
    comando(0x2A);
    SPI.transfer16(x0);
    SPI.transfer16(x1);
    comando(0x2B);
    SPI.transfer16(y0);
    SPI.transfer16(y1);
    comando(0x2C);
  }
  // Dibuja una letra de la fuente escalada
  void caracter(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg,
                uint8_t s) {
    int16_t w = 6 * s, h = 8 * s;
    if (x < 0 || y < 0 || x + w > _width || y + h > _height) return;
    inicioSPI();
    ventana(x, y, w, h);
    for (int16_t fy = 0; fy < h; fy++) {
      uint8_t fila = fy / s;
      for (int16_t fx = 0; fx < w; fx++) {
        uint8_t col = fx / s;
        bool on = col < 5 && ((font[(uint8_t)c * 5 + col] >> fila) & 1);
        SPI.transfer16(on ? fg : bg);
      }
    }
    finSPI();
  }
  void ejecutarLista(const uint8_t *p) {
    uint8_t cantidad = *p++;
    while (cantidad--) {
      uint8_t cmd = *p++;
      uint8_t nArgs = *p++;
      bool conPausa = nArgs & TFT_DELAY;
      nArgs &= ~TFT_DELAY;
      inicioSPI();
      comando(cmd);
      while (nArgs--) SPI.transfer(*p++);
      finSPI();
      if (conPausa) {
        uint16_t ms = *p++;
        if (ms == 255) ms = 500;
        delay(ms);
      }
    }
  }
};

PantallaTFT screen(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

// ===== Estado general =====
Vista vista = V_BIENVENIDA, vistaRetorno = V_MENU;
unsigned long vistaDesde = 0;
bool redibujar = true;
uint8_t datoIdx = 0;
unsigned long ultimoDato = 0;
uint8_t curMenu = 0, curDiag = 0, curManual = 0;
uint8_t nivelPwmManual = NIVEL_PWM_INICIAL;
bool paradaEnclavada = false;
char causaParada[26] = "";
char aviso1[26] = "", aviso2[26] = "";

void pararSeguro(const char *causa);
void stopSeguro();
void cambiar(Vista v);
void avisar(const char *a, const char *b, Vista volver);
void dibujarOpcionMenu(uint8_t n);

// La calibracion de la celda esta al final del archivo
namespace CalHX {
  void iniciar();
  void cancelar();
  bool activa();
  void tick();
}

// ===== Botones =====
const unsigned long ANTIRREBOTE_MS = 30;
const unsigned long PULSACION_LARGA_MS = 600;

Boton btnA = {PIN_BOTON_A, HIGH, HIGH, false, 0, 0};
Boton btnB = {PIN_BOTON_B, HIGH, HIGH, false, 0, 0};
EventoBoton evA = BTN_NADA, evB = BTN_NADA;
bool ignorarBotones = false;

// Corto: se avisa al soltar. Largo: apenas se cumple el tiempo manteniendo
EventoBoton leerBoton(Boton &b) {
  bool r = digitalRead(b.pin);
  unsigned long ahora = millis();
  if (r != b.raw) { b.raw = r; b.tCambio = ahora; }
  if (ahora - b.tCambio >= ANTIRREBOTE_MS && r != b.estable) {
    b.estable = r;
    if (r == LOW) { b.tInicio = ahora; b.largoEmitido = false; }
    else if (!b.largoEmitido) return BTN_CORTO;
  }
  if (b.estable == LOW && !b.largoEmitido &&
      ahora - b.tInicio >= PULSACION_LARGA_MS) {
    b.largoEmitido = true;
    return BTN_LARGO;
  }
  return BTN_NADA;
}

bool presionado(const Boton &b) { return b.estable == LOW; }

// Ignora un boton que venia apretado desde la pantalla anterior
bool mantiene(const Boton &b) { return !ignorarBotones && b.estable == LOW; }

void actualizarBotones() {
  EventoBoton a = leerBoton(btnA);
  EventoBoton b = leerBoton(btnB);
  if (ignorarBotones) {
    if (!presionado(btnA) && !presionado(btnB)) ignorarBotones = false;
    return;
  }
  if (a != BTN_NADA) evA = a;
  if (b != BTN_NADA) evB = b;
}

// D4 sin antirrebote: cualquier pulso cuenta como parada
bool botonStopFisico() {
  bool alto = digitalRead(PIN_STOP) == HIGH;
  return STOP_CONTACTO_NC ? alto : !alto;
}

// ===== Encoder del motor (cuadratura x4 con interrupciones) =====
volatile long cuentasMotor = 0;
volatile uint8_t estadoPrevioEncoder = 0;
const int8_t TABLA_QDEC[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void isrEncoderMotor() {
  uint8_t actual = (digitalRead(PIN_MOTOR_ENCA) << 1) |
                    digitalRead(PIN_MOTOR_ENCB);
  cuentasMotor += TABLA_QDEC[(estadoPrevioEncoder << 2) | actual];
  estadoPrevioEncoder = actual;
}

// Copia segura del contador, que cambia dentro de la interrupcion
long leerCuentasMotor() {
  noInterrupts();
  long c = cuentasMotor;
  interrupts();
  return c;
}

// ===== Motor (L298N) =====
// Las pantallas solo piden un PWM; motorActualizar() decide si se aplica
float pwmAct = 0.0f;
float pwmPedido = 0.0f;
int8_t dirAct = 0;
unsigned long tOrdenMotor = 0, tMotorPrev = 0;
bool liberandoDesdeParada = false;
bool encoderVisto = false;
long encUltimoMov = 0;
unsigned long tUltimoMov = 0;

// Unica funcion que toca los pines del driver. PWM positivo = recoger cable
void motorEscribir(float pct) {
  pct = constrain(pct, -100.0f, 100.0f);
  if (!MOTOR_HABILITADO || fabs(pct) < 0.5f) {
    // Puente apagado: el motor queda libre, sin freno
    analogWrite(PIN_MOTOR_ENA, 0);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    pwmAct = 0.0f;
    dirAct = 0;
    return;
  }
  int8_t dir = (pct > 0 ? 1 : -1) * MOTOR_SENTIDO_RECOGER;
  if (dir != dirAct) {
    analogWrite(PIN_MOTOR_ENA, 0);
    digitalWrite(PIN_MOTOR_IN1, dir > 0 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_IN2, dir > 0 ? LOW : HIGH);
    dirAct = dir;
  }
  analogWrite(PIN_MOTOR_ENA, (int)(fabs(pct) * 2.55f + 0.5f));
  pwmAct = pct;
}

void motorDetener() {
  pwmPedido = 0.0f;
  motorEscribir(0);
}

// La orden vence en 100 ms si la pantalla no la renueva
void motorPedir(float pct) {
  pwmPedido = pct;
  tOrdenMotor = millis();
}

// Solo "Mover motor" y "Control automatico" pueden mover el motor
bool motorPermitido() {
  if (!MOTOR_HABILITADO || botonStopFisico()) return false;
  if (paradaEnclavada) return vista == V_STOP && liberandoDesdeParada;
  return vista == V_MANUAL || vista == V_AUTO;
}

// Una vez por vuelta: permisos, rampa de subida y cambio de sentido seguro
void motorActualizar() {
  unsigned long ahora = millis();
  float dt = (ahora - tMotorPrev) / 1000.0f;
  tMotorPrev = ahora;
  if (dt > 0.1f) dt = 0.1f;

  if (!motorPermitido() || ahora - tOrdenMotor > T_ORDEN_MOTOR_MS) {
    motorDetener();
    return;
  }
  float obj = constrain(pwmPedido, -100.0f, 100.0f);
  if (fabs(obj) < 0.5f) { motorEscribir(0); return; }

  // Antes de invertir, una vuelta con el puente apagado
  if (pwmAct != 0.0f && (obj > 0) != (pwmAct > 0)) { motorEscribir(0); return; }

  float mag = fabs(pwmAct), magObj = fabs(obj);
  // Bajar es inmediato; subir parte en la zona util y sigue en rampa
  if (magObj <= mag) {
    mag = magObj;
  } else if (mag < PWM_MINIMO) {
    mag = fminf(magObj, (float)PWM_MINIMO);
  } else {
    mag = fminf(magObj, mag + RAMPA_PWM_PCT_POR_S * dt);
  }
  motorEscribir(obj > 0 ? mag : -mag);
}

// Guarda el ultimo momento en que el encoder se movio
void vigilarEncoder() {
  long c = leerCuentasMotor();
  if (labs(c - encUltimoMov) >= ENC_AVANCE_MIN) {
    encoderVisto = true;
    encUltimoMov = c;
    tUltimoMov = millis();
  }
  if (fabs(pwmAct) < PWM_MINIMO) tUltimoMov = millis();
}

// Motor energizado pero quieto (atasco o falta de torque)
bool motorSinAvance() {
  return ENCODER_INSTALADO && encoderVisto && fabs(pwmAct) >= PWM_MINIMO &&
         millis() - tUltimoMov > T_SIN_AVANCE_MS;
}

// ===== Celda de carga =====
const uint32_t CAL_MAGIC = 0xCE11A711;
const float G_STD = 9.80665f;
const uint8_t VENTANA_HX = 10;

bool celdaCalibrada = false;
float hxCuentasPorN = 0.0f;
int32_t hxRaw = 0, hxTara = 0;
bool hxSat = false, hxNuevoCtrl = false, fFiltInit = false;
uint8_t hxRepetidas = 0;
unsigned long hxUltimoMs = 0, hxUltimoUs = 0;
float hxTasaHz = 0.0f;
float fN = 0.0f, fFilt = 0.0f;

bool taraEnCurso = false;
uint8_t taraN = 0;
int64_t taraSuma = 0;

int32_t hxVentana[VENTANA_HX];
uint8_t hxVentanaN = 0, hxVentanaIdx = 0;

bool hxListo() { return digitalRead(PIN_HX_DOUT) == LOW; }

// Lee 24 bits. Las interrupciones se apagan solo por bit para no perder cuentas del encoder
int32_t hxLeerRaw() {
  uint32_t v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    // Pulso 25: canal A con ganancia 128
    noInterrupts();
    digitalWrite(PIN_HX_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_HX_DOUT) ? 1UL : 0UL);
    digitalWrite(PIN_HX_SCK, LOW);
    interrupts();
    delayMicroseconds(1);
  }
  noInterrupts();
  digitalWrite(PIN_HX_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);
  interrupts();
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (int32_t)v;
}

void hxVentanaAgregar(int32_t v) {
  hxVentana[hxVentanaIdx] = v;
  hxVentanaIdx = (hxVentanaIdx + 1) % VENTANA_HX;
  if (hxVentanaN < VENTANA_HX) hxVentanaN++;
}

int32_t hxVentanaPicoPico() {
  if (hxVentanaN < 2) return -1;
  int32_t mn = hxVentana[0], mx = hxVentana[0];
  for (uint8_t i = 1; i < hxVentanaN; i++) {
    if (hxVentana[i] < mn) mn = hxVentana[i];
    if (hxVentana[i] > mx) mx = hxVentana[i];
  }
  return mx - mn;
}

// Trae la tara y el factor que guardo la calibracion
void cargarCalibracionCelda() {
  hxCuentasPorN = HX_CUENTAS_POR_N;
  CalibCelda c;
  EEPROM.get(0, c);
  bool ok = (c.magic == CAL_MAGIC && c.factor != 0.0f && !isnan(c.factor));
  if (ok) {
    hxTara = c.offset;
    if (HX_CUENTAS_POR_N == 0.0f) hxCuentasPorN = c.factor * 1000.0f / G_STD;
  }
  celdaCalibrada = isfinite(hxCuentasPorN) && fabs(hxCuentasPorN) > 0.001f;
  fFiltInit = false;
}

// Si hay una lectura nueva la procesa; si no, vuelve sin esperar
void hxPoll() {
  if (!HX711_INSTALADO || !hxListo()) return;
  int32_t r = hxLeerRaw();
  unsigned long us = micros();
  if (hxUltimoUs != 0 && us != hxUltimoUs) {
    float hz = 1e6f / (float)(us - hxUltimoUs);
    hxTasaHz = (hxTasaHz == 0.0f) ? hz : hxTasaHz + 0.1f * (hz - hxTasaHz);
  }
  hxUltimoUs = us;
  hxUltimoMs = millis();
  if (r == hxRaw) { if (hxRepetidas < 255) hxRepetidas++; }
  else hxRepetidas = 0;
  hxRaw = r;
  hxSat = (r >= 0x7FFFFF || r <= -0x800000);
  if (hxSat) return;
  hxVentanaAgregar(r);

  // Tara rapida: promedia algunas lecturas, sin tocar la EEPROM
  if (taraEnCurso) {
    taraSuma += r;
    if (++taraN >= N_TARA_RAPIDA) {
      hxTara = (int32_t)(taraSuma / taraN);
      taraEnCurso = false;
      fFiltInit = false;
      Serial.print(F("# Tara rapida (solo RAM): ")); Serial.println(hxTara);
    }
    return;
  }
  if (!celdaCalibrada) return;
  // Fuerza en N y su version filtrada
  fN = (hxRaw - hxTara) / hxCuentasPorN;
  if (!fFiltInit) { fFilt = fN; fFiltInit = true; }
  else fFilt += ALFA_FUERZA * (fN - fFilt);
  // Avisa al controlador que hay una muestra nueva
  hxNuevoCtrl = true;
}

void taraIniciar() {
  taraEnCurso = true;
  taraN = 0;
  taraSuma = 0;
}

// Distingue entre sin datos, saturado o lecturas sospechosas
EstadoHX estadoHX() {
  if (!HX711_INSTALADO || hxUltimoMs == 0 ||
      millis() - hxUltimoMs > T_HX_SIN_DATOS_MS) return HX_SIN_DATOS;
  if (hxTasaHz > HX_TASA_MAX_HZ || hxRepetidas >= HX_REPETIDAS_MAX)
    return HX_INDETERMINADO;
  if (hxSat) return HX_SATURADO;
  return HX_OK;
}

// El control solo usa la fuerza si todo esta en orden
bool fuerzaValida() {
  return celdaCalibrada && estadoHX() == HX_OK && !taraEnCurso && fFiltInit &&
         isfinite(fN) && isfinite(fFilt);
}

const char *textoFallaCelda() {
  if (!celdaCalibrada) return "Celda sin calibrar";
  switch (estadoHX()) {
    case HX_SIN_DATOS:     return "HX711 sin datos";
    case HX_SATURADO:      return "Celda saturada";
    case HX_INDETERMINADO: return "HX711 indeterminado";
    default:               return "Lectura celda invalida";
  }
}

// ===== Encoder magnetico AS5600 =====
EstadoAS asEstado = AS_NO_DETECTADO;
uint8_t asStatus = 0, asAgc = 0;
uint16_t asRaw = 0, asMag = 0;
long asUnw = 0;
bool asPrimera = true;
float asCeroDeg = 0.0f;
unsigned long asUltimaConsulta = 0;

bool asLeerRegistros(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (size_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Lee estado, angulo, AGC y magnitud del campo
bool leerAS5600() {
  uint8_t a[3], b[3];
  if (!asLeerRegistros(0x0B, a, 3) || !asLeerRegistros(0x1A, b, 3)) {
    asEstado = AS_NO_DETECTADO;
    asPrimera = true;
    return false;
  }
  asStatus = a[0];
  uint16_t raw = (((uint16_t)a[1] << 8) | a[2]) & 0x0FFF;
  asAgc = b[0];
  asMag = (((uint16_t)b[1] << 8) | b[2]) & 0x0FFF;
  // Bit MD en 0: el sensor responde pero no ve el iman
  if (!(asStatus & 0x20)) {
    asEstado = AS_SIN_IMAN;
    asPrimera = true;
    asRaw = raw;
    return true;
  }
  asEstado = AS_CON_IMAN;
  if (asPrimera) { asUnw = raw; asPrimera = false; }
  else {
    // Acumula el giro para que el angulo no salte al pasar por 0
    int16_t d = (int16_t)raw - (int16_t)asRaw;
    if (d > 2048) d -= 4096;
    if (d < -2048) d += 4096;
    asUnw += d;
  }
  asRaw = raw;
  return true;
}

void asTick() {
  if (!AS5600_INSTALADO || millis() - asUltimaConsulta < AS_PERIODO_MS) return;
  asUltimaConsulta = millis();
  leerAS5600();
}

float asDeg()    { return asUnw * (360.0f / 4096.0f); }
float asRawDeg() { return asRaw * (360.0f / 4096.0f); }

// ===== Utilidades de pantalla =====
uint8_t yLinea(uint8_t i) { return 22 + 13 * i; }

void panelFondo() {
  screen.fillScreen(C_UI_BG);
  screen.fillRect(3, 18, 154, 92, C_UI_PANEL);
  screen.drawRect(3, 18, 154, 92, C_UI_BORDE);
}
void linea(uint8_t y, const char *s, uint16_t color) {
  screen.textoFondo(8, y, s, color, C_UI_PANEL, 1, 24);
}
void lineaN(uint8_t i, const char *s, uint16_t color) { linea(yLinea(i), s, color); }
void encabezado(const char *s) {
  panelFondo();
  screen.fillRect(0, 0, 160, 16, C_UI_TITULO);
  screen.textoFondo(5, 4, s, C_UI_PANEL, C_UI_TITULO, 1, 25);
}
void pie(const char *s) {
  screen.fillRect(0, 112, 160, 16, C_UI_PIE);
  screen.drawFastHLine(0, 112, 160, C_UI_BORDE);
  screen.textoFondo(2, 117, s, C_UI_TITULO, C_UI_PIE, 1, 26);
}
void item(uint8_t y, const char *s, bool elegido) {
  uint16_t fondo = elegido ? C_UI_ACENTO : C_UI_PANEL;
  screen.fillRect(7, y - 2, 146, 13, fondo);
  screen.drawRect(7, y - 2, 146, 13, elegido ? C_UI_BORDE : C_UI_PANEL);
  screen.textoFondo(10, y, s, C_UI_TEXTO, fondo, 1, 23);
}
char *fmtNum(char *b, float v, uint8_t dec) {
  if (!isfinite(v)) { strcpy(b, "--"); return b; }
  dtostrf(v, 1, dec, b);
  return b;
}

// Dibujo de Jimmy para la bienvenida
const uint8_t PORTADA_W = 40;
const uint8_t PORTADA_H = 40;
const char PORTADA_SPRITE[PORTADA_H][PORTADA_W + 1] = {
  "0000000222220000000000000000000000000000",
  "0000000222220000000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112200000000000000000000000000",
  "0000022111112222222220012222000000000000",
  "0000022111112211111112241111222000000000",
  "0000022111112211111112241111222000000000",
  "0000022222222211111112241111222222200000",
  "0000022222222211111112241111222222200000",
  "0022211111112211111112241111222111122200",
  "2231111113333322211112241111222111122200",
  "2231111113333322211112241111222111122200",
  "2231111221333322211112241111222111122200",
  "2231111221333322211112241111222111122200",
  "0022211114222233333332241111222111122222",
  "0022211111111122233332243333222333322222",
  "0022211111111122233332243333222333322222",
  "0000022111111111322221142222111222211122",
  "0000022111111111322221142222111222211122",
  "0000022111111111111111111111111111111322",
  "0000022111111111111111111111111111111322",
  "0000022111111111111111111111111111111322",
  "0000000224111111111111111111111111122200",
  "0000000224111111111111111111111111122200",
  "0000000001221111111111111111111111122200",
  "0000000001221111111111111111111112200000",
  "0000000001221111111111111111111112200000",
  "0000000000002211111111111111111220000000",
  "0000000000002211111111111111111220000000",
  "0000000000002211111111111111111220000000",
  "0000000000000022222222222222222000000000",
  "0000000000000022222222222222222000000000"
};

uint16_t spriteColor(char c) {
  switch (c) {
    case '1': return C_UI_PIEL;
    case '2': return C_UI_TRAZO;
    case '3': return C_UI_BRILLO;
    case '4': return C_UI_PIEL_S;
    default:  return C_UI_BG;
  }
}
void dibujarPortada(int16_t x, int16_t y, uint8_t escala) {
  for (uint8_t j = 0; j < PORTADA_H; j++) {
    uint8_t i = 0;
    while (i < PORTADA_W) {
      char color = PORTADA_SPRITE[j][i];
      uint8_t desde = i;
      do { ++i; } while (i < PORTADA_W && PORTADA_SPRITE[j][i] == color);
      if (color != '0') {
        screen.fillRect(x + desde * escala, y + j * escala,
                        (i - desde) * escala, escala, spriteColor(color));
      }
    }
  }
}

// ===== Navegacion =====
// Todo cambio de pantalla deja el motor libre y espera a que se suelten los botones
void cambiar(Vista v) {
  motorDetener();
  liberandoDesdeParada = false;
  vista = v;
  vistaDesde = millis();
  redibujar = true;
  evA = evB = BTN_NADA;
  ignorarBotones = true;
}

void avisar(const char *a, const char *b, Vista volver) {
  strncpy(aviso1, a, sizeof(aviso1) - 1); aviso1[sizeof(aviso1) - 1] = 0;
  strncpy(aviso2, b, sizeof(aviso2) - 1); aviso2[sizeof(aviso2) - 1] = 0;
  vistaRetorno = volver;
  cambiar(V_MENSAJE);
}

bool libBloqueada = false;
char libMsg[26] = "";
CicloAuto au;
void autoImprimirResumen(const char *motivo);

// Parada segura: quita la energia del motor y queda enclavada hasta rearmar
void pararSeguro(const char *causa) {
  motorDetener();
  bool estabaEnAuto = (vista == V_AUTO);
  paradaEnclavada = true;
  strncpy(causaParada, causa, sizeof(causaParada) - 1);
  causaParada[sizeof(causaParada) - 1] = 0;
  cambiar(V_STOP);
  libBloqueada = false;
  strcpy(libMsg, "Mant. A0: soltar cable");
  Serial.print(F("# PARADA: ")); Serial.println(causaParada);
  if (estabaEnAuto) autoImprimirResumen("PARADA");
}

void stopSeguro() { pararSeguro("Boton D4"); }

// ===== Menus =====
const char *const OPC_PRINCIPAL[] = {
  "1 Conexiones", "2 Diagnostico", "3 Mover motor", "4 Control automatico"
};
const char *const OPC_DIAG[] = {
  "Encoder AS5600", "Celda: calibrar", "Celda: lectura vivo", "Volver"
};

uint8_t nOpciones() {
  switch (vista) {
    case V_MENU:       return 4;
    case V_DIAG:       return 4;
    case V_MANUAL_CFG: return 3;
    default:           return 0;
  }
}

uint8_t *cursorActual() {
  switch (vista) {
    case V_MENU:       return &curMenu;
    case V_DIAG:       return &curDiag;
    case V_MANUAL_CFG: return &curManual;
    default:           return nullptr;
  }
}

void etiquetaOpcion(uint8_t n, char *b, size_t len) {
  b[0] = 0;
  switch (vista) {
    case V_MENU: if (n < 4) snprintf(b, len, "%s", OPC_PRINCIPAL[n]); break;
    case V_DIAG: if (n < 4) snprintf(b, len, "%s", OPC_DIAG[n]); break;
    case V_MANUAL_CFG:
      if (n == 0) snprintf(b, len, "PWM manual: %u %%", (unsigned)NIVELES_PWM_MANUAL[nivelPwmManual]);
      else if (n == 1) snprintf(b, len, "Mover motor");
      else if (n == 2) snprintf(b, len, "Volver");
      break;
    default: break;
  }
}

// Redibuja una sola opcion, sin borrar la pantalla
void dibujarOpcionMenu(uint8_t n) {
  uint8_t *c = cursorActual();
  if (!c || n >= nOpciones()) return;
  char b[28];
  etiquetaOpcion(n, b, sizeof(b));
  item(22 + 15 * n, b, *c == n);
}

// A0 corto avanza, A0 largo vuelve, A1 confirma
bool navegarMenu() {
  uint8_t *c = cursorActual();
  if (!c) return false;
  if (evA == BTN_CORTO) {
    uint8_t anterior = *c;
    *c = (*c + 1) % nOpciones();
    dibujarOpcionMenu(anterior);
    dibujarOpcionMenu(*c);
  }
  if (evA == BTN_LARGO && vista != V_MENU) { cambiar(V_MENU); return false; }
  return evB == BTN_CORTO;
}

// ===== Mover motor (manual) =====
EstadoManual manEstado = MAN_DETENIDO;
bool manBloqueo = false;
unsigned long tPulsacion = 0, tAmbos = 0, tLogManual = 0;

void manualReiniciar() {
  manEstado = MAN_DETENIDO;
  manBloqueo = false;
  tPulsacion = tAmbos = 0;
}

void procesarManual() {
  bool a = mantiene(btnA), b = mantiene(btnB);
  // A0 + A1 juntos por 0,6 s: salir
  if (a && b) {
    motorPedir(0);
    manEstado = MAN_DETENIDO;
    if (!tAmbos) tAmbos = millis();
    else if (millis() - tAmbos >= PULSACION_LARGA_MS) cambiar(V_MANUAL_CFG);
    return;
  }
  tAmbos = 0;
  int8_t dir = a ? 1 : (b ? -1 : 0);
  if (dir == 0) {
    motorPedir(0);
    manEstado = MAN_DETENIDO;
    manBloqueo = false;
    tPulsacion = 0;
    return;
  }
  if (manBloqueo) { motorPedir(0); return; }
  if (!tPulsacion) tPulsacion = millis();
  if (millis() - tPulsacion > MANUAL_MAX_CONTINUO_MS) {
    manBloqueo = true; manEstado = MAN_TIEMPO; motorPedir(0);
    return;
  }
  // Recoger solo con la celda funcionando y bajo el limite
  if (dir > 0) {
    bool fOk = fuerzaValida();
    if (!fOk && !RECOGER_MANUAL_SIN_CELDA) { manEstado = MAN_SIN_CELDA; motorPedir(0); return; }
    if (fOk && fN >= TENSION_MAX_SEGURA)   { manEstado = MAN_LIMITE;    motorPedir(0); return; }
  }
  if (motorSinAvance()) {
    manBloqueo = true; manEstado = MAN_SIN_AVANCE; motorPedir(0);
    Serial.print(F("# MANUAL sin avance: pwm=")); Serial.print(NIVELES_PWM_MANUAL[nivelPwmManual]);
    Serial.print(F(" F_N=")); Serial.println(fuerzaValida() ? fN : NAN, 2);
    return;
  }
  manEstado = dir > 0 ? MAN_RECOGIENDO : MAN_SOLTANDO;
  motorPedir(dir * (float)NIVELES_PWM_MANUAL[nivelPwmManual]);
}

// CSV mientras el motor gira: tiempo, PWM, encoder y fuerza
void logManual() {
  if (pwmAct == 0.0f || millis() - tLogManual < 100) return;
  tLogManual = millis();
  Serial.print(F("M,")); Serial.print(millis()); Serial.print(',');
  Serial.print(pwmAct, 1); Serial.print(',');
  Serial.print(leerCuentasMotor()); Serial.print(',');
  if (fuerzaValida()) Serial.println(fN, 3); else Serial.println(F("nan"));
}

// ===== Control automatico por tension (realimenta solo con la celda) =====
const char *autoMotivoNoInicio() {
  if (!MOTOR_HABILITADO) return "Motor deshabilitado";
  if (!(TENSION_OBJETIVO > 0.0f && TENSION_OBJETIVO + TOLERANCIA_TENSION < TENSION_MAX_SEGURA &&
        PWM_MINIMO < PWM_MAXIMO && PWM_MAXIMO <= 100)) return "Revisar parametros";
  if (!celdaCalibrada) return "Celda sin calibrar";
  if (taraEnCurso) return "Tarando...";
  if (!fuerzaValida()) return textoFallaCelda();
  if (fabs(fFilt) > TENSION_INICIAL_MAX) return "F inicial alta: tarar";
  return nullptr;
}

void autoIniciar() {
  au = CicloAuto();
  au.fase = FA_RECOGIDA;
  au.tInicio = au.tFase = au.tSubida = millis();
  au.fInicial = au.ref = au.fSubida = fFilt;
  au.fMax = fN;
  au.encInicio = leerCuentasMotor();
  au.tMuestraPrev = hxUltimoMs;
  hxNuevoCtrl = false;
  cambiar(V_AUTO);
  Serial.print(F("# AUTO inicio: objetivo_N=")); Serial.print(TENSION_OBJETIVO, 2);
  Serial.print(F(" F_inicial_N=")); Serial.println(au.fInicial, 3);
  Serial.println(F("t_ms,fase,ref_N,F_N,Ffilt_N,pwm_pct,encoder,as_deg"));
}

void autoImprimirResumen(const char *motivo) {
  Serial.print(F("# AUTO fin (")); Serial.print(motivo);
  Serial.print(F("): F_max_N=")); Serial.print(au.fMax, 3);
  Serial.print(F(" sobrepaso=")); Serial.print(au.sobrepaso ? F("si") : F("no"));
  Serial.print(F(" t_recogida_s=")); Serial.print(au.tRecogidaS, 2);
  Serial.print(F(" retorno=")); Serial.println(au.modoRet == RET_ENCODER ? F("encoder") : F("tension"));
}

// Objetivo alcanzado: guarda datos y pasa a soltar cable
void autoIniciarRetorno() {
  au.encPico = leerCuentasMotor();
  long d = au.encPico - au.encInicio;
  au.modoRet = (USAR_ENCODER_EN_RETORNO && ENCODER_INSTALADO && labs(d) >= ENC_RECORRIDO_MIN)
               ? RET_ENCODER : RET_TENSION;
  au.tRecogidaS = (millis() - au.tInicio) / 1000.0f;
  au.fase = FA_RETORNO;
  au.tFase = millis();
  au.fMinRet = fFilt;
  au.tFlojo = 0;
  au.u = 0;
  motorDetener();
  Serial.print(F("# OBJETIVO ALCANZADO: F_N=")); Serial.print(fFilt, 3);
  Serial.print(F(" encoder_recorrido=")); Serial.println(d);
}

void autoFinalizar() {
  motorDetener();
  au.fFinal = fFilt;
  autoImprimirResumen("OK");
  cambiar(V_AUTO_FIN);
}

// CSV a 20 Hz para graficar el ciclo
void autoLog() {
  if (millis() - au.tLog < 50) return;
  au.tLog = millis();
  Serial.print(millis() - au.tInicio); Serial.print(',');
  Serial.print(au.fase == FA_RECOGIDA ? F("REC") : F("RET")); Serial.print(',');
  Serial.print(au.ref, 3); Serial.print(',');
  Serial.print(fN, 3); Serial.print(',');
  Serial.print(fFilt, 3); Serial.print(',');
  Serial.print(pwmAct, 1); Serial.print(',');
  Serial.print(leerCuentasMotor()); Serial.print(',');
  if (asEstado == AS_CON_IMAN) Serial.println(asRawDeg(), 2); else Serial.println(F("nan"));
}

// Un paso del ciclo automatico (se llama en cada vuelta)
void autoTick() {
  unsigned long ahora = millis();

  // se revisa siempre, haya o no muestra nueva
  if (mantiene(btnA) || mantiene(btnB)) { pararSeguro("Boton en ciclo auto"); return; }
  if (!fuerzaValida())                  { pararSeguro(textoFallaCelda()); return; }
  if (fN > TENSION_MAX_SEGURA || fFilt > TENSION_MAX_SEGURA) {
    pararSeguro("Tension > max. segura"); return;
  }
  if (fN > au.fMax) au.fMax = fN;

  if (au.fase == FA_RECOGIDA) {
    if (ahora - au.tFase > TIEMPO_MAX_RECOGIDA) { pararSeguro("Timeout recogida"); return; }

    // Sin avance = encoder quieto y la tension tampoco sube
    if (fFilt > au.fSubida + 0.05f) { au.fSubida = fFilt; au.tSubida = ahora; }
    if (motorSinAvance() && ahora - au.tSubida > T_SIN_AVANCE_MS) {
      pararSeguro("Motor sin avance"); return;
    }

    // El controlador corre una vez por cada muestra nueva de la celda
    if (hxNuevoCtrl) {
      hxNuevoCtrl = false;
      float dt = (hxUltimoMs - au.tMuestraPrev) / 1000.0f;
      au.tMuestraPrev = hxUltimoMs;
      dt = constrain(dt, 0.0f, 0.3f);

      // La consigna sube en rampa y no se adelanta demasiado a la medicion
      au.ref = fminf(au.ref + RAMPA_CONSIGNA_NPS * dt, TENSION_OBJETIVO);
      au.ref = fminf(au.ref, fFilt + ADELANTO_MAX_N);

      // Dentro de la banda: motor libre y se cuenta el tiempo
      if (fFilt >= TENSION_OBJETIVO - TOLERANCIA_TENSION) {
        au.u = 0;
        if (!au.tEnBanda) au.tEnBanda = ahora;
      } else {
        au.tEnBanda = 0;
        float e = au.ref - fFilt;
        if (e <= 0.0f) au.u = 0;
        else {
          // Proporcional sobre el minimo que mueve el motor (zona muerta)
          float u = PWM_MINIMO + KP_TENSION * e;
          if (fFilt > TENSION_OBJETIVO - ZONA_APROXIMACION_N) u = PWM_MINIMO;
          au.u = fminf(u, (float)PWM_MAXIMO);
        }
      }
      if (fFilt > TENSION_OBJETIVO + TOLERANCIA_TENSION) au.sobrepaso = true;
    }
    if (au.tEnBanda && ahora - au.tEnBanda >= TIEMPO_EN_BANDA) {
      autoIniciarRetorno();
      autoLog();
      return;
    }
    motorPedir(au.u);

  } else {
    // Retorno: soltar cable hasta volver a la tension inicial
    if (ahora - au.tFase > TIEMPO_MAX_RETORNO) { pararSeguro("Timeout retorno"); return; }
    au.fMinRet = fminf(au.fMinRet, fFilt);
    // Si al soltar la tension sube, el sentido esta mal: parar
    if (fFilt > au.fMinRet + AUMENTO_MAX_RETORNO_N) {
      pararSeguro("Tension sube al soltar"); return;
    }
    if (au.modoRet == RET_ENCODER) {
      float rest = (leerCuentasMotor() - au.encInicio) / (float)(au.encPico - au.encInicio);
      if (rest <= FRACCION_FIN_RETORNO) { autoFinalizar(); return; }
      if (motorSinAvance()) { pararSeguro("Retorno sin avance"); return; }
      au.u = -(float)(rest < FRACCION_LENTA_RETORNO ? PWM_MINIMO : PWM_RETORNO);
    } else {
      float umbral = au.fInicial + HOLGURA_RETORNO_N;
      // Cable flojo: se confirma con el motor detenido
      if (fFilt <= umbral) {
        au.u = 0;
        if (!au.tFlojo) au.tFlojo = ahora;
        else if (ahora - au.tFlojo >= TIEMPO_FLOJO_MS) { autoFinalizar(); return; }
      } else {
        au.tFlojo = 0;
        au.u = -(float)(fFilt < umbral + ZONA_APROXIMACION_N ? PWM_MINIMO : PWM_RETORNO);
      }
    }
    motorPedir(au.u);
  }
  autoLog();
}

// ===== Pantalla de parada: soltar cable a mano manteniendo A0 =====
unsigned long tLib = 0;
float fMinLib = 0.0f;

void procesarParada() {
  if (botonStopFisico()) {
    liberandoDesdeParada = false;
    strcpy(libMsg, "Suelte D4");
    return;
  }
  // Sin A0 mantenido, A1 rearma y vuelve al menu
  bool a = mantiene(btnA) && !mantiene(btnB);
  if (!a) {
    liberandoDesdeParada = false;
    libBloqueada = false;
    strcpy(libMsg, "Mant. A0: soltar cable");
    if (evB == BTN_CORTO) {
      paradaEnclavada = false;
      Serial.println(F("# Rearmado por el usuario"));
      cambiar(V_MENU);
    }
    return;
  }
  if (libBloqueada) return;
  if (!MOTOR_HABILITADO) { strcpy(libMsg, "Motor deshabilitado"); return; }
  if (!fuerzaValida())   { strcpy(libMsg, "Sin celda: soltar a mano"); return; }
  if (!liberandoDesdeParada) {
    liberandoDesdeParada = true;
    tLib = millis();
    fMinLib = fFilt;
  }
  fMinLib = fminf(fMinLib, fFilt);
  if (fFilt > fMinLib + AUMENTO_MAX_RETORNO_N) {
    liberandoDesdeParada = false; libBloqueada = true;
    strcpy(libMsg, "Tension sube: sentido?"); return;
  }
  if (millis() - tLib > T_LIBERAR_MAX_MS) {
    liberandoDesdeParada = false; libBloqueada = true;
    strcpy(libMsg, "Tiempo max: suelte A0"); return;
  }
  if (fFilt <= HOLGURA_RETORNO_N) {
    liberandoDesdeParada = false; libBloqueada = true;
    strcpy(libMsg, "Cable flojo"); return;
  }
  strcpy(libMsg, "Soltando cable...");
  motorPedir(-(float)PWM_LIBERAR);
}

// ===== Dibujo de pantallas =====
uint8_t datosVista() {
  switch (vista) {
    case V_CONEXIONES: case V_DIAG_AS: case V_DIAG_CELDA:
    case V_MANUAL: case V_AUTO_PREP: case V_AUTO:
      return 6;
    case V_STOP:
      return 5;
    default:
      return 0;
  }
}

void lineaEstadoAS(uint8_t i) {
  if (!AS5600_INSTALADO) { lineaN(i, "AS5600: no instalado", C_GRIS); return; }
  switch (asEstado) {
    case AS_CON_IMAN: lineaN(i, "AS5600: CON IMAN", C_VERDE); break;
    case AS_SIN_IMAN: lineaN(i, "AS5600: SIN IMAN", C_NARANJA); break;
    default:          lineaN(i, "AS5600: NO DETECTADO", C_ROJO); break;
  }
}

void lineaEstadoHX(uint8_t i) {
  char b[30], n[16];
  switch (estadoHX()) {
    case HX_OK:
      snprintf(b, sizeof(b), "HX711: OK %s Hz", fmtNum(n, hxTasaHz, 1));
      lineaN(i, b, C_VERDE); break;
    case HX_SATURADO:      lineaN(i, "HX711: SATURADO", C_ROJO); break;
    case HX_INDETERMINADO: lineaN(i, "HX711: INDETERMINADO", C_NARANJA); break;
    default:               lineaN(i, "HX711: SIN DATOS", C_ROJO); break;
  }
}

void lineaFuerza(uint8_t i, const char *prefijo) {
  char b[30], n[16];
  if (fuerzaValida()) {
    snprintf(b, sizeof(b), "%s%s N", prefijo, fmtNum(n, fFilt, 2));
    lineaN(i, b, fabs(fN) >= TENSION_MAX_SEGURA ? C_ROJO : C_AMARILLO);
  } else {
    snprintf(b, sizeof(b), "%s-- (sin celda)", prefijo);
    lineaN(i, b, C_GRIS);
  }
}

// pinta una linea de datos de la pantalla actual
void dibujarDato(uint8_t i) {
  char b[30], n[16], m[16];
  switch (vista) {
    case V_CONEXIONES:
      if (i == 0) lineaEstadoAS(0);
      else if (i == 1) lineaEstadoHX(1);
      else if (i == 2) lineaN(2, celdaCalibrada ? "Celda: CALIBRADA" : "Celda: SIN CALIBRAR",
                             celdaCalibrada ? C_VERDE : C_NARANJA);
      else if (i == 3) {
        if (!ENCODER_INSTALADO) lineaN(3, "Encoder: no instalado", C_GRIS);
        else if (encoderVisto) { snprintf(b, sizeof(b), "Encoder: OK %ld c", leerCuentasMotor()); lineaN(3, b, C_VERDE); }
        else lineaN(3, "Encoder: sin verificar", C_GRIS);
      }
      else if (i == 4) lineaN(4, MOTOR_HABILITADO ? "Motor: reposo (libre)" : "Motor: DESHABILITADO",
                             MOTOR_HABILITADO ? C_UI_TEXTO : C_NARANJA);
      else if (i == 5) lineaN(5, botonStopFisico() ? "D4: ACTIVO (parada)" : "D4: suelto",
                             botonStopFisico() ? C_ROJO : C_UI_TEXTO);
      break;

    case V_DIAG_AS:
      if (i == 0) lineaEstadoAS(0);
      else if (i == 1) {
        if (asEstado == AS_CON_IMAN) { snprintf(b, sizeof(b), "Angulo: %s deg", fmtNum(n, asRawDeg(), 1)); lineaN(1, b, C_AMARILLO); }
        else lineaN(1, "Angulo: --", C_GRIS);
      } else if (i == 2) {
        if (asEstado != AS_NO_DETECTADO) snprintf(b, sizeof(b), "Bruto: %u / 4095", (unsigned)asRaw);
        else snprintf(b, sizeof(b), "Bruto: --");
        lineaN(2, b, C_UI_TEXTO);
      } else if (i == 3) {
        if (asEstado == AS_CON_IMAN) snprintf(b, sizeof(b), "Rel. cero: %s deg", fmtNum(n, asDeg() - asCeroDeg, 1));
        else snprintf(b, sizeof(b), "Rel. cero: --");
        lineaN(3, b, C_UI_TEXTO);
      } else if (i == 4) {
        if (asEstado != AS_NO_DETECTADO) snprintf(b, sizeof(b), "AGC: %u  Mag: %u", (unsigned)asAgc, (unsigned)asMag);
        else snprintf(b, sizeof(b), "AGC: --  Mag: --");
        lineaN(4, b, C_UI_TEXTO);
      } else if (i == 5) {
        if (asEstado == AS_NO_DETECTADO)  lineaN(5, "Revise SDA SCL VCC GND", C_ROJO);
        else if (asEstado == AS_SIN_IMAN) lineaN(5, "Acerque el iman", C_NARANJA);
        else if (asStatus & 0x10)         lineaN(5, "Campo DEBIL: acercar", C_NARANJA);
        else if (asStatus & 0x08)         lineaN(5, "Campo FUERTE: alejar", C_NARANJA);
        else                              lineaN(5, "Campo en rango", C_VERDE);
      }
      break;

    case V_DIAG_CELDA:
      if (i == 0) {
        if (!celdaCalibrada) lineaN(0, "Tension: sin calibrar", C_NARANJA);
        else lineaFuerza(0, "Tension: ");
      } else if (i == 1) {
        if (fuerzaValida()) snprintf(b, sizeof(b), "Masa eq.: %s g", fmtNum(n, fFilt * 1000.0f / G_STD, 1));
        else snprintf(b, sizeof(b), "Masa eq.: --");
        lineaN(1, b, C_UI_TEXTO);
      } else if (i == 2) {
        snprintf(b, sizeof(b), "Bruto: %ld c", (long)hxRaw); lineaN(2, b, C_UI_TEXTO);
      } else if (i == 3) {
        snprintf(b, sizeof(b), "Tasa: %s Hz", fmtNum(n, hxTasaHz, 1)); lineaN(3, b, C_UI_TEXTO);
      } else if (i == 4) {
        int32_t pp = hxVentanaPicoPico();
        if (pp < 0) snprintf(b, sizeof(b), "Ruido: --");
        else if (celdaCalibrada) snprintf(b, sizeof(b), "Ruido: %s N p-p", fmtNum(n, pp / fabs(hxCuentasPorN), 3));
        else snprintf(b, sizeof(b), "Ruido: %ld c p-p", (long)pp);
        lineaN(4, b, C_UI_TEXTO);
      } else if (i == 5) {
        if (taraEnCurso) lineaN(5, "Tarando: no tocar", C_AMARILLO);
        else if (estadoHX() != HX_OK) lineaN(5, textoFallaCelda(), C_ROJO);
        else if (!celdaCalibrada) lineaN(5, "Calibre la celda", C_NARANJA);
        else lineaN(5, "Lectura OK", C_VERDE);
      }
      break;

    case V_MANUAL: {
      static const char *const NOMBRES[] = {
        "DETENIDO", "RECOGIENDO", "SOLTANDO", "LIMITE TENSION", "SIN CELDA",
        "SIN AVANCE", "TIEMPO MAX"
      };
      if (i == 0) {
        snprintf(b, sizeof(b), "Estado: %s", NOMBRES[manEstado]);
        lineaN(0, b, manEstado <= MAN_SOLTANDO ? C_AMARILLO : C_NARANJA);
      } else if (i == 1) {
        lineaN(1, pwmAct > 0 ? "Sentido: recoger" : (pwmAct < 0 ? "Sentido: soltar" : "Sentido: -"), C_UI_TEXTO);
      } else if (i == 2) {
        snprintf(b, sizeof(b), "PWM: %d / %u %%", (int)fabs(pwmAct), (unsigned)NIVELES_PWM_MANUAL[nivelPwmManual]);
        lineaN(2, b, C_UI_TEXTO);
      } else if (i == 3) lineaFuerza(3, "F: ");
      else if (i == 4) { snprintf(b, sizeof(b), "Encoder: %ld c", leerCuentasMotor()); lineaN(4, b, C_UI_TEXTO); }
      else if (i == 5) {
        switch (manEstado) {
          case MAN_LIMITE:     lineaN(5, "Limite F: solo soltar", C_NARANJA); break;
          case MAN_SIN_CELDA:  lineaN(5, "Sin celda: no recoge", C_NARANJA); break;
          case MAN_SIN_AVANCE: lineaN(5, "Sin avance: suelte", C_ROJO); break;
          case MAN_TIEMPO:     lineaN(5, "Max 8 s: suelte boton", C_NARANJA); break;
          default:             lineaN(5, "A0+A1 0,6 s: salir", C_GRIS); break;
        }
      }
      break;
    }

    case V_AUTO_PREP:
      if (i == 0) lineaN(0, fuerzaValida() ? "Celda: OK" : textoFallaCelda(), fuerzaValida() ? C_VERDE : C_ROJO);
      else if (i == 1) lineaFuerza(1, "F actual: ");
      else if (i == 2) {
        snprintf(b, sizeof(b), "Obj: %s N tol %s", fmtNum(n, TENSION_OBJETIVO, 2), fmtNum(m, TOLERANCIA_TENSION, 2));
        lineaN(2, b, C_UI_TEXTO);
      } else if (i == 3) {
        snprintf(b, sizeof(b), "Max segura: %s N", fmtNum(n, TENSION_MAX_SEGURA, 2)); lineaN(3, b, C_UI_TEXTO);
      } else if (i == 4) {
        lineaN(4, (USAR_ENCODER_EN_RETORNO && ENCODER_INSTALADO) ? "Retorno: encoder" : "Retorno: por tension", C_UI_TEXTO);
      } else if (i == 5) {
        const char *mot = autoMotivoNoInicio();
        lineaN(5, mot ? mot : "Listo: A1 inicia", mot ? C_NARANJA : C_VERDE);
      }
      break;

    case V_AUTO:
      if (i == 0) {
        if (au.fase == FA_RECOGIDA) lineaN(0, "Fase: RECOGIDA", C_AMARILLO);
        else lineaN(0, au.modoRet == RET_ENCODER ? "Fase: RETORNO (enc.)" : "Fase: RETORNO (tension)", C_AMARILLO);
      } else if (i == 1) {
        snprintf(b, sizeof(b), "F: %s N ref %s", fmtNum(n, fFilt, 2), fmtNum(m, au.ref, 2)); lineaN(1, b, C_UI_TEXTO);
      } else if (i == 2) {
        snprintf(b, sizeof(b), "Objetivo: %s N", fmtNum(n, TENSION_OBJETIVO, 2)); lineaN(2, b, C_UI_TEXTO);
      } else if (i == 3) {
        snprintf(b, sizeof(b), "PWM: %d %%", (int)pwmAct); lineaN(3, b, C_UI_TEXTO);
      } else if (i == 4) {
        snprintf(b, sizeof(b), "Encoder: %ld c", leerCuentasMotor()); lineaN(4, b, C_UI_TEXTO);
      } else if (i == 5) {
        snprintf(b, sizeof(b), "t %s s  Fmax %s", fmtNum(n, (millis() - au.tInicio) / 1000.0f, 1), fmtNum(m, au.fMax, 2));
        lineaN(5, b, C_UI_TEXTO);
      }
      break;

    case V_STOP:
      if (i == 0) lineaN(0, causaParada, C_ROJO);
      else if (i == 1) lineaN(1, pwmAct != 0.0f ? "Motor: SOLTANDO" : "Motor: sin energia", C_UI_TEXTO);
      else if (i == 2) lineaFuerza(2, "F: ");
      else if (i == 3) lineaN(3, libMsg, liberandoDesdeParada ? C_AMARILLO : C_UI_TEXTO);
      else if (i == 4) lineaN(4, "A1: rearmar (D4 suelto)", C_GRIS);
      break;

    default: break;
  }
}

// Dibuja la pantalla completa al entrar en ella
void dibujarVista() {
  char b[30], n[16], m[16];
  switch (vista) {
    case V_BIENVENIDA:
      screen.fillScreen(C_UI_BG);
      dibujarPortada(40, 8, 2);
      screen.textoCentrado(95, "HOLA, SOY JIMMY", C_UI_TEXTO, C_UI_BG, 1);
      pie("A1:continuar  D4:STOP");
      return;
    case V_MENU:       encabezado("MENU PRINCIPAL"); pie("A0:sig A1:OK D4:STOP"); break;
    case V_DIAG:       encabezado("DIAGNOSTICO");    pie("A0:sig A1:OK A0L:volver"); break;
    case V_MANUAL_CFG: encabezado("MOVER MOTOR");    pie("A0:sig A1:OK A0L:volver"); break;
    case V_CONEXIONES: encabezado("CONEXIONES");     pie("A1/A0L:volver D4:STOP"); break;
    case V_DIAG_AS:    encabezado("DIAG. AS5600");   pie("A1:fijar cero A0L:volver"); break;
    case V_DIAG_CELDA: encabezado("CELDA EN VIVO");  pie("A0:tara A1/A0L:volver"); break;
    case V_MANUAL:     encabezado("MOTOR MANUAL");   pie("A0:recoger A1:soltar"); break;
    case V_AUTO_PREP:  encabezado("AUTO: PREPARACION"); pie("A1:ini A0:tara A0L:volver"); break;
    case V_AUTO:       encabezado("AUTO: EN CICLO"); pie("A0/A1/D4: PARAR"); break;
    case V_STOP:       encabezado("PARADA SEGURA");  pie("A1:rearmar A0:soltar"); break;
    case V_AUTO_FIN:
      encabezado("AUTO: CICLO TERMINADO");
      lineaN(0, au.sobrepaso ? "Objetivo: OK +sobrepaso" : "Objetivo: ALCANZADO",
             au.sobrepaso ? C_NARANJA : C_VERDE);
      snprintf(b, sizeof(b), "F max: %s N", fmtNum(n, au.fMax, 2)); lineaN(1, b, C_UI_TEXTO);
      snprintf(b, sizeof(b), "F objetivo: %s N", fmtNum(n, TENSION_OBJETIVO, 2)); lineaN(2, b, C_UI_TEXTO);
      snprintf(b, sizeof(b), "t recogida: %s s", fmtNum(n, au.tRecogidaS, 1)); lineaN(3, b, C_UI_TEXTO);
      lineaN(4, au.modoRet == RET_ENCODER ? "Retorno: encoder OK" : "Retorno: cable flojo", C_UI_TEXTO);
      if (au.modoRet == RET_ENCODER) {
        snprintf(b, sizeof(b), "F final: %s N", fmtNum(m, au.fFinal, 2)); lineaN(5, b, C_UI_TEXTO);
      } else lineaN(5, "Posicion NO verificada", C_NARANJA);
      pie("A1:menu A0:repetir");
      return;
    case V_MENSAJE:
      encabezado("JIMMY AVISA");
      linea(40, aviso1, C_NARANJA);
      linea(60, aviso2, C_UI_TEXTO);
      pie("A1:volver D4:STOP");
      return;
  }
  for (uint8_t i = 0; i < nOpciones(); i++) dibujarOpcionMenu(i);
  for (uint8_t i = 0; i < datosVista(); i++) dibujarDato(i);
}

// Una linea por vez cada 50 ms, para no frenar el control
void actualizarPantalla() {
  if (redibujar) {
    redibujar = false;
    dibujarVista();
    datoIdx = 0;
    ultimoDato = millis();
    return;
  }
  uint8_t n = datosVista();
  if (!n || millis() - ultimoDato < TFT_DATO_MS) return;
  ultimoDato = millis();
  if (datoIdx >= n) datoIdx = 0;
  dibujarDato(datoIdx++);
}

// ===== Maquina de estados de la interfaz =====
void procesar() {
  switch (vista) {
    case V_BIENVENIDA:
      if (evB == BTN_CORTO) cambiar(V_MENU);
      break;

    case V_MENU:
      if (navegarMenu()) {
        if (curMenu == 0) cambiar(V_CONEXIONES);
        else if (curMenu == 1) cambiar(V_DIAG);
        else if (curMenu == 2) cambiar(V_MANUAL_CFG);
        else cambiar(V_AUTO_PREP);
      }
      break;

    case V_CONEXIONES:
      if (evB == BTN_CORTO || evA == BTN_LARGO) cambiar(V_MENU);
      break;

    case V_DIAG:
      if (navegarMenu()) {
        if (curDiag == 0) {
          if (AS5600_INSTALADO) cambiar(V_DIAG_AS);
          else avisar("AS5600", "no instalado", V_DIAG);
        } else if (curDiag == 1) CalHX::iniciar();
        else if (curDiag == 2) cambiar(V_DIAG_CELDA);
        else cambiar(V_MENU);
      }
      break;

    case V_DIAG_AS:
      if (evB == BTN_CORTO && asEstado == AS_CON_IMAN) {
        asCeroDeg = asDeg();
        Serial.print(F("# AS5600 cero relativo en raw=")); Serial.println(asRaw);
      }
      if (evA == BTN_LARGO) cambiar(V_DIAG);
      break;

    case V_DIAG_CELDA:
      if (evA == BTN_CORTO) {
        if (estadoHX() == HX_OK) taraIniciar();
        else avisar("No se puede tarar", textoFallaCelda(), V_DIAG_CELDA);
      }
      if (evB == BTN_CORTO || evA == BTN_LARGO) cambiar(V_DIAG);
      break;

    case V_MANUAL_CFG:
      if (navegarMenu()) {
        if (curManual == 0) {
          nivelPwmManual = (nivelPwmManual + 1) % N_NIVELES_PWM;
          dibujarOpcionMenu(0);
        } else if (curManual == 1) {
          manualReiniciar();
          cambiar(V_MANUAL);
        } else cambiar(V_MENU);
      }
      break;

    case V_MANUAL:
      procesarManual();
      logManual();
      break;

    case V_AUTO_PREP:
      if (evA == BTN_LARGO) { cambiar(V_MENU); break; }
      if (evA == BTN_CORTO) {
        if (estadoHX() == HX_OK && celdaCalibrada) taraIniciar();
        else avisar("No se puede tarar", textoFallaCelda(), V_AUTO_PREP);
      }
      if (evB == BTN_CORTO) {
        const char *mot = autoMotivoNoInicio();
        if (mot) avisar("No se puede iniciar", mot, V_AUTO_PREP);
        else autoIniciar();
      }
      break;

    case V_AUTO:
      autoTick();
      break;

    case V_AUTO_FIN:
      if (evB == BTN_CORTO) cambiar(V_MENU);
      if (evA == BTN_CORTO) cambiar(V_AUTO_PREP);
      break;

    case V_STOP:
      procesarParada();
      break;

    case V_MENSAJE:
      if (evB == BTN_CORTO) cambiar(vistaRetorno);
      break;
  }
}

// ===== Calibracion de la celda (modulo 03 integrado) =====
// Se bloquea por momentos, pero aqui el motor siempre esta apagado
namespace CalHX {
// Usa los colores de la interfaz principal
const uint16_t C_NEGRO    = ::C_UI_BG;
const uint16_t C_BLANCO   = ::C_UI_TEXTO;
const uint16_t C_GRIS     = ::C_UI_BORDE;
const uint16_t C_GRIS_OSC = ::C_UI_PANEL;
const uint16_t C_CIAN     = ::C_UI_BORDE;
const uint16_t C_AMARILLO = ::C_UI_TEXTO;
const uint16_t C_VERDE    = ::C_UI_BORDE;
const uint16_t C_ROJO     = ::C_UI_TITULO;
const uint16_t C_NARANJA  = ::C_UI_BORDE;

enum Estado : uint8_t { E_INICIO, E_TARA, E_MASA, E_RESULTADO, E_VERIFICAR };
// Masas disponibles para calibrar [g]
const float MASAS_G[] = {50.0f, 100.0f, 200.0f, 500.0f, 1000.0f, 2000.0f};
const uint8_t N_MASAS = sizeof(MASAS_G) / sizeof(MASAS_G[0]);
const uint8_t MASA_INICIAL_IDX = 3;

const uint8_t N_TARA     = 20;
const uint8_t N_CAPTURA  = 20;
const uint8_t MAX_PUNTOS = 6;
const uint8_t VENTANA    = 10;
const int32_t UMBRAL_ESTABLE_MIN = 200;
const float   FACTOR_RUIDO = 6.0f;

const float F_LIMITE_N = 25.0f;
const float G_STD      = 9.80665f;
const uint32_t T_SIN_RESPUESTA_MS = 1000;

const uint32_t CAL_MAGIC = 0xCE11A711;
struct CalibCelda { uint32_t magic; long offset; float factor; };

bool hxOk = false, hxSat = false, hxNuevo = false;
int32_t hxRaw = 0;
unsigned long hxUltimoMs = 0, hxUltimoUs = 0;
float hxTasaHz = 0.0f;

int32_t ventanaBuf[VENTANA];
uint8_t ventanaIdx = 0, ventanaLlena = 0;

bool hxListo() { return digitalRead(PIN_HX_DOUT) == LOW; }

int32_t hxLeerRaw() {
  uint32_t v = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(PIN_HX_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_HX_DOUT) ? 1UL : 0UL);
    digitalWrite(PIN_HX_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_HX_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);
  interrupts();
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (int32_t)v;
}

void ventanaReiniciar() { ventanaIdx = 0; ventanaLlena = 0; }

void ventanaAgregar(int32_t v) {
  ventanaBuf[ventanaIdx] = v;
  ventanaIdx = (ventanaIdx + 1) % VENTANA;
  if (ventanaLlena < VENTANA) ventanaLlena++;
}

float ventanaMedia() {
  if (!ventanaLlena) return hxRaw;
  int64_t s = 0;
  for (uint8_t i = 0; i < ventanaLlena; i++) s += ventanaBuf[i];
  return (float)s / ventanaLlena;
}

int32_t ventanaPicoPico() {
  if (ventanaLlena < 2) return 0x7FFFFFFF;
  int32_t mn = ventanaBuf[0], mx = ventanaBuf[0];
  for (uint8_t i = 1; i < ventanaLlena; i++) {
    if (ventanaBuf[i] < mn) mn = ventanaBuf[i];
    if (ventanaBuf[i] > mx) mx = ventanaBuf[i];
  }
  return mx - mn;
}

void hxPoll() {
  if (!hxListo()) return;
  int32_t r = hxLeerRaw();
  unsigned long ahoraUs = micros();
  if (hxUltimoUs != 0) {
    float hz = 1e6f / (ahoraUs - hxUltimoUs);
    hxTasaHz = (hxTasaHz == 0.0f) ? hz : hxTasaHz + 0.1f * (hz - hxTasaHz);
  }
  hxUltimoUs = ahoraUs;
  hxUltimoMs = millis();
  hxSat = (r >= 0x7FFFFF || r <= -0x800000);
  hxRaw = r;
  hxNuevo = true;
  ventanaAgregar(r);
}

bool hxRespondiendo() { return millis() - hxUltimoMs < T_SIN_RESPUESTA_MS; }

int32_t taraCuentas = 0;
float ruidoTaraPP = 0.0f;
float factorCG = 0.0f;
bool calGuardada = false;

float masaG[MAX_PUNTOS];
float netoC[MAX_PUNTOS];
uint8_t nPuntos = 0;
uint8_t idxMasa = MASA_INICIAL_IDX;
float masaSerie = 0.0f;
float errMaxG = 0.0f;

float masaElegida() { return masaSerie > 0.0f ? masaSerie : MASAS_G[idxMasa]; }

int32_t umbralEstable() {
  int32_t u = (int32_t)(FACTOR_RUIDO * ruidoTaraPP);
  return u < UMBRAL_ESTABLE_MIN ? UMBRAL_ESTABLE_MIN : u;
}

bool lecturaEstable() {
  return ventanaLlena >= VENTANA && !hxSat && ventanaPicoPico() <= umbralEstable();
}

float netoActual() { return ventanaMedia() - taraCuentas; }
float gramosActual() { return factorCG != 0.0f ? netoActual() / factorCG : NAN; }
float newtonsActual() { return gramosActual() * G_STD / 1000.0f; }

void cargarEEPROM() {
  CalibCelda c;
  EEPROM.get(0, c);
  calGuardada = (c.magic == CAL_MAGIC && c.factor != 0.0f && !isnan(c.factor));
  if (calGuardada) {
    factorCG = c.factor;
    taraCuentas = c.offset;
  }
}

void guardarEEPROM() {
  CalibCelda c = {CAL_MAGIC, (long)taraCuentas, factorCG};
  EEPROM.put(0, c);
  calGuardada = true;
}

// Minimos cuadrados por el origen: cuentas por gramo
void calcularFactor() {
  double sxy = 0, sxx = 0;
  for (uint8_t i = 0; i < nPuntos; i++) {
    sxy += (double)netoC[i] * masaG[i];
    sxx += (double)masaG[i] * masaG[i];
  }
  factorCG = (sxx > 0) ? (float)(sxy / sxx) : 0.0f;
  errMaxG = 0.0f;
  for (uint8_t i = 0; i < nPuntos; i++) {
    float e = fabs(netoC[i] / factorCG - masaG[i]);
    if (e > errMaxG) errMaxG = e;
  }
}

// Promedia n lecturas para la tara o un punto; se corta con D4 o sin datos
bool promediar(uint8_t n, int32_t &prom, float &pp, const char *texto) {
  screen.fillRect(0, 76, 160, 30, C_NEGRO);
  screen.textoFondo(4, 78, texto, C_AMARILLO, C_NEGRO);
  int64_t suma = 0;
  int32_t mn = 0, mx = 0;
  for (uint8_t i = 0; i < n; i++) {
    unsigned long t = millis();
    while (!hxListo()) {
      if (botonStopFisico()) return false;
      if (millis() - t > T_SIN_RESPUESTA_MS) return false;
    }
    if (botonStopFisico()) return false;
    int32_t r = hxLeerRaw();
    if (r >= 0x7FFFFF || r <= -0x800000) return false;
    suma += r;
    if (i == 0 || r < mn) mn = r;
    if (i == 0 || r > mx) mx = r;
    int16_t w = (int16_t)(150L * (i + 1) / n);
    screen.fillRect(5, 92, w, 8, C_VERDE);
    screen.fillRect(5 + w, 92, 150 - w, 8, C_GRIS_OSC);
  }
  prom = (int32_t)(suma / n);
  pp = (float)(mx - mn);
  hxUltimoMs = millis();
  ventanaReiniciar();
  return true;
}

Estado estado = E_INICIO;

char *fmtF(char *buf, float v, int8_t ancho, uint8_t dec) {
  if (isnan(v)) { strcpy(buf, "--"); return buf; }
  dtostrf(v, ancho, dec, buf);
  return buf;
}

void titulo(const char *t) {
  screen.fillScreen(C_NEGRO);
  screen.textoFondo(4, 2, t, C_CIAN, C_NEGRO);
  screen.drawFastHLine(0, 12, 160, C_GRIS_OSC);
}

void pie(const char *l1, const char *l2) {
  screen.textoFondo(4, 108, l1, C_VERDE, C_NEGRO, 1, 26);
  screen.textoFondo(4, 118, l2, C_GRIS, C_NEGRO, 1, 26);
}

void mensajeBreve(const char *l1, const char *l2, uint16_t color) {
  screen.fillScreen(C_NEGRO);
  screen.textoCentrado(40, l1, color, C_NEGRO, 2);
  screen.textoCentrado(70, l2, C_BLANCO, C_NEGRO);
  unsigned long inicioMensaje = millis();
  while (millis() - inicioMensaje < 1400) {
    if (botonStopFisico()) return;
  }
}

void barra(int16_t y, float p, uint16_t color) {
  if (isnan(p)) p = 0;
  p = constrain(p, 0.0f, 1.0f);
  int16_t w = (int16_t)(p * 150);
  screen.fillRect(5, y, w, 8, color);
  screen.fillRect(5 + w, y, 150 - w, 8, C_GRIS_OSC);
}

// Pantallas de la calibracion
void entrarInicio() {
  estado = E_INICIO;
  char buf[28], n1[12];
  titulo("CALIBRAR CELDA");
  hxOk = hxRespondiendo();
  if (hxOk) {
    screen.textoFondo(4, 18, "HX711: OK", C_VERDE, C_NEGRO);
    snprintf(buf, sizeof(buf), "Tasa: %s Hz", fmtF(n1, hxTasaHz, 1, 1));
    screen.textoFondo(4, 30, buf, C_BLANCO, C_NEGRO);
  } else {
    screen.textoFondo(4, 18, "HX711: NO RESPONDE", C_ROJO, C_NEGRO);
    screen.textoFondo(4, 30, "Revisa DT->A2, SCK->A3,", C_BLANCO, C_NEGRO);
    screen.textoFondo(4, 40, "VCC 5V y GND.", C_BLANCO, C_NEGRO);
  }
  if (calGuardada) {
    snprintf(buf, sizeof(buf), "Guardada: %s c/g", fmtF(n1, factorCG, 1, 2));
    screen.textoFondo(4, 56, buf, C_AMARILLO, C_NEGRO);
  } else {
    screen.textoFondo(4, 56, "Sin calibracion guardada", C_NARANJA, C_NEGRO);
  }
  screen.textoFondo(4, 72, "Celda 10 kg, limite 25 N", C_GRIS, C_NEGRO);
  screen.textoFondo(4, 82, "(unos 2,5 kg colgados)", C_GRIS, C_NEGRO);
  if (!hxOk) pie("A1: reintentar", "A0 largo: menu");
  else pie("A1: calibrar", calGuardada ? "A0: verificar guardada" : "A0 largo: menu");
}

void entrarTara() {
  estado = E_TARA;
  nPuntos = 0;
  masaSerie = 0.0f;
  titulo("PASO 1/2: TARA");
  screen.textoFondo(4, 18, "Celda montada y cable en", C_BLANCO, C_NEGRO);
  screen.textoFondo(4, 28, "sus guias, SIN masa.", C_BLANCO, C_NEGRO);
  pie("A1: tarar", "A0 mant.: volver");
}

void camposTara() {
  char buf[28];
  snprintf(buf, sizeof(buf), "Raw  : %ld", (long)hxRaw);
  screen.textoFondo(4, 46, buf, C_AMARILLO, C_NEGRO, 1, 26);
  int32_t pp = ventanaPicoPico();
  if (pp == 0x7FFFFFFF) snprintf(buf, sizeof(buf), "Ruido: --");
  else snprintf(buf, sizeof(buf), "Ruido: %ld c p-p", (long)pp);
  screen.textoFondo(4, 58, buf, C_BLANCO, C_NEGRO, 1, 26);
  screen.textoFondo(4, 78, hxSat ? "SATURADO: revisa cables" : "", C_ROJO, C_NEGRO, 1, 26);
}

void entrarMasa() {
  estado = E_MASA;
  char buf[28];
  snprintf(buf, sizeof(buf), "PASO 2/2: MASA %u", nPuntos + 1);
  titulo(buf);
  screen.textoFondo(4, 18, "Cuelga la masa del cable", C_BLANCO, C_NEGRO);
  screen.textoFondo(4, 28, "y espera ESTABLE.", C_BLANCO, C_NEGRO);
  pie("A0: masa A1: capturar", nPuntos ? "A0 mant.: resultado" : "A0 mant.: volver tara");
}

void camposMasa() {
  char buf[28], n1[12];
  snprintf(buf, sizeof(buf), "Masa : %s g%s", fmtF(n1, masaElegida(), 1, 1),
           masaSerie > 0.0f ? " (serie)" : "");
  screen.textoFondo(4, 42, buf, C_CIAN, C_NEGRO, 1, 26);
  snprintf(buf, sizeof(buf), "Neto : %ld c", (long)netoActual());
  screen.textoFondo(4, 54, buf, C_BLANCO, C_NEGRO, 1, 26);
  if (factorCG != 0.0f) {
    snprintf(buf, sizeof(buf), "Aprox: %s g", fmtF(n1, gramosActual(), 1, 1));
    screen.textoFondo(4, 66, buf, C_GRIS, C_NEGRO, 1, 26);
  }
  if (hxSat) screen.textoFondo(4, 80, "SATURADO", C_ROJO, C_NEGRO, 1, 26);
  else if (factorCG != 0.0f && newtonsActual() > F_LIMITE_N)
    screen.textoFondo(4, 80, "SOBRE 25 N: quita masa", C_ROJO, C_NEGRO, 1, 26);
  else if (lecturaEstable()) screen.textoFondo(4, 80, "ESTABLE", C_VERDE, C_NEGRO, 1, 26);
  else screen.textoFondo(4, 80, "esperando...", C_NARANJA, C_NEGRO, 1, 26);
  snprintf(buf, sizeof(buf), "Puntos: %u/%u", nPuntos, MAX_PUNTOS);
  screen.textoFondo(4, 92, buf, C_GRIS, C_NEGRO, 1, 26);
}

void entrarResultado() {
  estado = E_RESULTADO;
  char buf[28], n1[12], n2[12];
  snprintf(buf, sizeof(buf), "RESULTADO (%u punto%s)", nPuntos, nPuntos == 1 ? "" : "s");
  titulo(buf);
  snprintf(buf, sizeof(buf), "Factor: %s c/g", fmtF(n1, factorCG, 1, 2));
  screen.textoFondo(4, 17, buf, C_AMARILLO, C_NEGRO);
  snprintf(buf, sizeof(buf), "        %s c/N", fmtF(n1, factorCG * 1000.0f / G_STD, 1, 0));
  screen.textoFondo(4, 27, buf, C_AMARILLO, C_NEGRO);
  uint8_t mostrar = nPuntos < 4 ? nPuntos : 4;
  for (uint8_t i = 0; i < mostrar; i++) {
    uint8_t k = nPuntos - mostrar + i;
    float e = netoC[k] / factorCG - masaG[k];
    snprintf(buf, sizeof(buf), "%s g  err %s g", fmtF(n1, masaG[k], 6, 0), fmtF(n2, e, 1, 1));
    screen.textoFondo(4, 41 + i * 10, buf, C_BLANCO, C_NEGRO);
  }
  float maxMasa = 0;
  for (uint8_t i = 0; i < nPuntos; i++) if (masaG[i] > maxMasa) maxMasa = masaG[i];
  snprintf(buf, sizeof(buf), "Err max: %s g (%s%%)", fmtF(n1, errMaxG, 1, 1),
           fmtF(n2, maxMasa > 0 ? 100.0f * errMaxG / maxMasa : 0, 1, 1));
  screen.textoFondo(4, 84, buf, nPuntos > 1 && errMaxG > 0.02f * maxMasa ? C_NARANJA : C_VERDE, C_NEGRO);
  if (factorCG < 0)
    screen.textoFondo(4, 95, "Signo -: invierte A+/A-", C_NARANJA, C_NEGRO);
  pie("A1: guardar A0: +punto", "A0 mant.: reiniciar");
}

void entrarVerificar() {
  estado = E_VERIFICAR;
  char buf[28], n1[12];
  titulo("VERIFICAR");
  ventanaReiniciar();
  snprintf(buf, sizeof(buf), "%s c/g", fmtF(n1, factorCG, 1, 2));
  screen.textoFondo(80, 2, buf, C_GRIS, C_NEGRO);
  pie("A0: tara   A1: menu", "Prueba con otra masa");
}

void camposVerificar() {
  char buf[28], n1[12];
  float n = newtonsActual(), g = gramosActual();
  snprintf(buf, sizeof(buf), "%s N", fmtF(n1, n, 1, 2));
  screen.textoFondo(4, 18, buf, n > F_LIMITE_N ? C_ROJO : C_AMARILLO, C_NEGRO, 2, 12);
  snprintf(buf, sizeof(buf), "%s g", fmtF(n1, g, 1, 1));
  screen.textoFondo(4, 40, buf, C_BLANCO, C_NEGRO, 1, 26);
  barra(54, n / F_LIMITE_N, n > 0.8f * F_LIMITE_N ? C_ROJO : C_VERDE);
  snprintf(buf, sizeof(buf), "%d %% del limite de 25 N", (int)(100.0f * n / F_LIMITE_N));
  screen.textoFondo(4, 66, buf, C_GRIS, C_NEGRO, 1, 26);
  int32_t pp = ventanaPicoPico();
  if (pp != 0x7FFFFFFF && factorCG != 0.0f)
    snprintf(buf, sizeof(buf), "Ruido: %s g p-p", fmtF(n1, pp / fabs(factorCG), 1, 1));
  else snprintf(buf, sizeof(buf), "Ruido: --");
  screen.textoFondo(4, 80, buf, C_BLANCO, C_NEGRO, 1, 26);
  screen.textoFondo(4, 92, hxSat ? "SATURADO" : "", C_ROJO, C_NEGRO, 1, 26);
}

// Acciones de la calibracion
void hacerTara() {
  int32_t prom;
  float pp;
  if (!promediar(N_TARA, prom, pp, "Tarando, no tocar...")) {
    if (botonStopFisico()) return;
    mensajeBreve("ERROR", "HX711 sin datos o saturado", C_ROJO);
    entrarInicio();
    return;
  }
  taraCuentas = prom;
  ruidoTaraPP = pp;
  Serial.print(F("# tara=")); Serial.print(taraCuentas);
  Serial.print(F(" ruido_pp=")); Serial.println(ruidoTaraPP, 0);
}

void capturarPunto() {
  if (!lecturaEstable()) {
    screen.textoFondo(4, 80, "Aun no estable...", C_NARANJA, C_NEGRO, 1, 26);
    return;
  }
  if (nPuntos >= MAX_PUNTOS) {
    entrarResultado();
    return;
  }
  int32_t prom;
  float pp;
  if (!promediar(N_CAPTURA, prom, pp, "Capturando, no tocar...")) {
    if (botonStopFisico()) return;
    mensajeBreve("ERROR", "HX711 sin datos o saturado", C_ROJO);
    entrarMasa();
    return;
  }
  float neto = (float)(prom - taraCuentas);
  if (fabs(neto) < 100) {
    mensajeBreve("SIN CAMBIO", "Cuelga la masa del cable", C_NARANJA);
    entrarMasa();
    return;
  }
  masaG[nPuntos] = masaElegida();
  netoC[nPuntos] = neto;
  nPuntos++;
  masaSerie = 0.0f;
  calcularFactor();

  Serial.print(F("# punto ")); Serial.print(nPuntos);
  Serial.print(F(": masa_g=")); Serial.print(masaG[nPuntos - 1], 1);
  Serial.print(F(" neto=")); Serial.print(neto, 0);
  Serial.print(F(" factor_c_g=")); Serial.print(factorCG, 3);
  Serial.print(F(" err_max_g=")); Serial.println(errMaxG, 2);
  entrarResultado();
}

void imprimirCalibracion() {
  Serial.println(F("# ---- Calibracion de la celda ----"));
  Serial.print(F("# factor_c_g=")); Serial.println(factorCG, 4);
  Serial.print(F("# factor_c_N=")); Serial.println(factorCG * 1000.0f / G_STD, 2);
  Serial.print(F("# tara=")); Serial.println(taraCuentas);
  Serial.print(F("# guardada_en_EEPROM=")); Serial.println(calGuardada ? F("si") : F("no"));
  Serial.print(F("# Para fijarla en el controlador (opcional): const float HX_CUENTAS_POR_N = "));
  Serial.print(factorCG * 1000.0f / G_STD, 2); Serial.println(F("f;"));
}

char cmdBuf[24];
uint8_t cmdLen = 0;

// Comandos por monitor serie: "m (g)" masa exacta, "i" informacion
void comandoSerie(char *l) {
  while (*l == ' ') l++;
  char c = l[0];
  float arg = atof(l + 1);
  if (c == 'm') {
    if (arg > 0) {
      masaSerie = arg;
      Serial.print(F("# proxima captura con masa_g=")); Serial.println(masaSerie, 2);
    } else Serial.println(F("# uso: m <gramos>, p. ej. m 523.4"));
  } else if (c == 'i') {
    imprimirCalibracion();
  }
}

void leerSerie() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; comandoSerie(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = ch;
  }
}

unsigned long tSerie = 0;
void registrarSerie() {
  if (!hxNuevo || millis() - tSerie < 100) return;
  if (estado != E_MASA && estado != E_VERIFICAR && estado != E_TARA) return;
  tSerie = millis();
  Serial.print(millis()); Serial.print(',');
  Serial.print(hxRaw); Serial.print(',');
  Serial.print(netoActual(), 0); Serial.print(',');
  if (factorCG != 0.0f) { Serial.print(gramosActual(), 2); Serial.print(','); Serial.println(newtonsActual(), 3); }
  else Serial.println(F("nan,nan"));
}

bool activo = false;
unsigned long tCampos = 0;

bool activa() { return activo; }
void cancelar() { activo = false; }

// Al salir, el control usa la calibracion guardada en la EEPROM
void salir() {
  activo = false;
  motorDetener();

  ::cargarCalibracionCelda();
  ::hxUltimoMs = 0;
  ::hxUltimoUs = 0;
  ::hxTasaHz = 0.0f;
  ::hxSat = false;
  ::hxRepetidas = 0;
  ::taraEnCurso = false;
  ::hxVentanaN = 0;
  cambiar(V_DIAG);
}

void iniciar() {
  motorDetener();
  if (paradaEnclavada || botonStopFisico()) return;
  activo = true;
  ignorarBotones = true;
  hxUltimoMs = 0; hxUltimoUs = 0; hxTasaHz = 0.0f;
  ventanaReiniciar();
  screen.fillScreen(C_NEGRO);
  screen.textoCentrado(40, "CELDA DE CARGA", C_CIAN, C_NEGRO, 1);
  screen.textoCentrado(56, "Iniciando...", C_BLANCO, C_NEGRO);
  unsigned long t = millis();
  while (millis() - t < 1200) {
    if (botonStopFisico()) { activo = false; stopSeguro(); return; }
    hxPoll();
  }
  cargarEEPROM();
  Serial.println(F("# 03_calibracion_celda integrado - IRB2002 G2"));
  Serial.println(F("# Comandos: m <g> (masa exacta), i (info)"));
  Serial.println(F("t_ms,raw,neto,gramos,newtons"));
  if (calGuardada) imprimirCalibracion();
  entrarInicio();
}

// Un paso de la calibracion
void tick() {
  if (!activo) return;
  if (botonStopFisico()) {
    activo = false;
    stopSeguro();
    return;
  }
  hxPoll();
  leerSerie();
  EventoBoton a = leerBoton(btnA);
  EventoBoton b = leerBoton(btnB);
  if (ignorarBotones) {
    if (btnA.estable == HIGH && btnB.estable == HIGH) ignorarBotones = false;
    a = b = BTN_NADA;
  }
  if (estado != E_INICIO && !hxRespondiendo()) {
    mensajeBreve("HX711", "Sin respuesta", C_ROJO);
    if (botonStopFisico()) { activo = false; stopSeguro(); return; }
    entrarInicio();
    return;
  }

  switch (estado) {
    case E_INICIO:
      if (b == BTN_CORTO) {
        if (hxRespondiendo()) entrarTara();
        else entrarInicio();
      } else if (a == BTN_CORTO && calGuardada && hxRespondiendo()) {
        entrarVerificar();
      } else if (a == BTN_LARGO) {
        salir();
        return;
      }
      break;

    case E_TARA:
      if (b == BTN_CORTO) {
        hacerTara();
        if (botonStopFisico()) { activo = false; stopSeguro(); return; }
        if (estado == E_TARA) entrarMasa();
      } else if (a == BTN_LARGO) entrarInicio();
      break;

    case E_MASA:
      if (a == BTN_CORTO) {
        masaSerie = 0.0f;
        idxMasa = (idxMasa + 1) % N_MASAS;
      } else if (a == BTN_LARGO) {
        if (nPuntos) entrarResultado();
        else entrarTara();
      } else if (b == BTN_CORTO) {
        capturarPunto();
        if (botonStopFisico()) { activo = false; stopSeguro(); return; }
      }
      break;

    case E_RESULTADO:
      if (b == BTN_CORTO) {
        if (factorCG == 0.0f) break;
        guardarEEPROM();
        imprimirCalibracion();
        mensajeBreve("GUARDADO", "El controlador ya la usa", C_VERDE);
        if (botonStopFisico()) { activo = false; stopSeguro(); return; }
        entrarVerificar();
      } else if (a == BTN_CORTO) {
        if (nPuntos < MAX_PUNTOS) entrarMasa();
      } else if (a == BTN_LARGO) {
        factorCG = 0.0f;
        cargarEEPROM();
        entrarTara();
      }
      break;

    case E_VERIFICAR:
      if (a == BTN_CORTO) {
        hacerTara();
        if (botonStopFisico()) { activo = false; stopSeguro(); return; }
        if (estado == E_VERIFICAR) entrarVerificar();
      } else if (b == BTN_CORTO) entrarInicio();
      break;
  }

  if (millis() - tCampos >= 120) {
    tCampos = millis();
    if (estado == E_TARA) camposTara();
    else if (estado == E_MASA) camposMasa();
    else if (estado == E_VERIFICAR) camposVerificar();
  }
  registrarSerie();
  hxNuevo = false;
}
}

// ===== Setup y loop =====
void setup() {

  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  motorEscribir(0);

  Serial.begin(115200);
  pinMode(PIN_BOTON_A, INPUT_PULLUP);
  pinMode(PIN_BOTON_B, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);

  if (ENCODER_INSTALADO) {
    pinMode(PIN_MOTOR_ENCA, INPUT_PULLUP);
    pinMode(PIN_MOTOR_ENCB, INPUT_PULLUP);
    estadoPrevioEncoder = (digitalRead(PIN_MOTOR_ENCA) << 1) | digitalRead(PIN_MOTOR_ENCB);
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCA), isrEncoderMotor, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCB), isrEncoderMotor, CHANGE);
  }
  pinMode(PIN_HX_DOUT, INPUT_PULLUP);
  pinMode(PIN_HX_SCK, OUTPUT);
  digitalWrite(PIN_HX_SCK, LOW);

  // I2C para el AS5600
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(5000);

  // La pantalla tarda ~1,5 s en arrancar; el motor sigue apagado
  screen.begin();
  cargarCalibracionCelda();
  encUltimoMov = leerCuentasMotor();
  tMotorPrev = tUltimoMov = millis();

  Serial.println(F("# Jimmy V9 - control por tension (celda) - monitor 115200"));
  Serial.print(F("# Celda: ")); Serial.println(celdaCalibrada ? F("calibrada (EEPROM)") : F("SIN CALIBRAR"));
  Serial.println(MOTOR_HABILITADO ? F("# Motor habilitado") : F("# Motor DESHABILITADO"));

  ignorarBotones = true;
  if (botonStopFisico()) pararSeguro("D4 activo al encender");
}

void loop() {

  // 1. Parada D4
  if (botonStopFisico()) {
    motorEscribir(0);
    if (!paradaEnclavada) { CalHX::cancelar(); stopSeguro(); }
  }

  // 2. Durante la calibracion, ella maneja la pantalla
  if (CalHX::activa()) {
    motorDetener();
    CalHX::tick();
    return;
  }

  // 3. Entradas y sensores 
  actualizarBotones();
  hxPoll();
  asTick();
  vigilarEncoder();

  // 4. Logica de la pantalla actual y motor
  procesar();
  motorActualizar();
  evA = evB = BTN_NADA;

  // 5. Pantalla, una linea por vez
  if (CalHX::activa()) return;
  actualizarPantalla();
}
