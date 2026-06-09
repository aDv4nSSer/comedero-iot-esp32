/*
 * ═══════════════════════════════════════════════════════════════════════
 * COMEDERO INTELIGENTE IoT — v4.0-SIM (Simulación Wokwi)
 * ═══════════════════════════════════════════════════════════════════════
 * Versión adaptada para simulación en Wokwi VS Code.
 * Wokwi NO soporta BLE ni OTA real, por lo que:
 *
 *   BLE  → reemplazado por comandos Serial (teclado en Serial Monitor)
 *   OTA  → simulado visualmente con millis() (5 segundos animados)
 *
 * TODO LO DEMÁS es idéntico a la versión hardware real v4.0:
 *   - FSM 4 estados, Edge Computing RER, EMA, Fusión sensorial
 *   - LCD 2004 I2C, NeoPixel 12 LEDs, Motor DC L298N
 *   - MQTT telemetría real, NTP Chile UTC-3
 *
 * ───────────────────────────────────────────────────────────────────────
 * COMANDOS SERIAL (escribir en Serial Monitor de Wokwi):
 *   CONFIG:nombre,peso_kg,factor_rer   → configura perfil del perro
 *   ALIMENTO:marca,kcal_g              → configura alimento
 *   DISPENSAR                          → dispensación manual
 *   OTA                                → simula actualización firmware
 *   BOVEDA                             → fuerza detección de atasco
 *   STATUS                             → muestra telemetría completa
 *
 * SECUENCIA DE DEMO (presentación):
 *   1. Arranque → bienvenida → REPOSO
 *   2. "STATUS"               → ver telemetría en Serial
 *   3. "CONFIG:Rex,20.0,130.0"→ cambiar perfil en vivo
 *   4. "DISPENSAR"            → motor gira + LCD dispensando
 *   5. Bajar slider HX711     → dispensación completa
 *   6. "BOVEDA"               → error + NeoPixel rojo parpadeando
 *   7. Esperar 10s            → auto-reset a REPOSO
 *   8. "OTA"                  → barra de progreso 5s + NeoPixel azul
 * ═══════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <time.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────
// WIFI WOKWI — Red virtual integrada en el simulador
// ─────────────────────────────────────────────────────────────────────
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// ─────────────────────────────────────────────────────────────────────
// MQTT
// ─────────────────────────────────────────────────────────────────────
const char* MQTT_BROKER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* TOPIC_TELEM = "comedero/telemetria";
const char* TOPIC_CMD   = "comedero/comando";

// ─────────────────────────────────────────────────────────────────────
// NTP — Chile UTC-3
// ─────────────────────────────────────────────────────────────────────
const char* NTP_SERVER   = "pool.ntp.org";
const long  GMT_OFFSET   = -3 * 3600;
const int   DAYLIGHT_SEC = 0;

// ─────────────────────────────────────────────────────────────────────
// PINES (idénticos al hardware real)
// ─────────────────────────────────────────────────────────────────────
#define PIN_HX711_DT    16
#define PIN_HX711_SCK   17
#define PIN_TRIG         5
#define PIN_ECHO        18
#define PIN_IN1         26
#define PIN_IN2         27
#define PIN_ENA         25
#define PIN_NEOPIXEL    19
#define PIN_SDA         21
#define PIN_SCL         22

#define PWM_CANAL   0
#define PWM_FREQ    5000
#define PWM_RES     8

#define NUM_PIXELS  12
#define BRILLO      80

// ─────────────────────────────────────────────────────────────────────
// CONSTANTES FÍSICAS (idénticas al hardware real)
// ─────────────────────────────────────────────────────────────────────
const float CAPACIDAD_TANQUE_G = 1000.0f;
const float DIST_VACIO_CM      =   20.0f;
const float UMBRAL_BOVEDA_PCT  =    0.30f;
const float ALPHA_EMA          =    0.30f;
const unsigned long TIMEOUT_MOTOR_MS    = 30000;

// ─────────────────────────────────────────────────────────────────────
// OTA SIMULADO — duración de la animación de "descarga"
// En hardware real esto lo maneja ArduinoOTA vía red.
// En Wokwi lo simulamos con un timer de millis().
// ─────────────────────────────────────────────────────────────────────
const unsigned long OTA_SIM_DURACION_MS = 5000;

// ─────────────────────────────────────────────────────────────────────
// FSM
// ─────────────────────────────────────────────────────────────────────
enum Estado { REPOSO, DISPENSANDO, ERROR, ACTUALIZANDO_OTA };
Estado estadoActual = REPOSO;

// ─────────────────────────────────────────────────────────────────────
// OBJETOS DE HARDWARE
// ─────────────────────────────────────────────────────────────────────
WiFiClient        espClient;
PubSubClient      mqtt(espClient);
HX711             balanza;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_NeoPixel anillo(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ─────────────────────────────────────────────────────────────────────
// PERFIL DEL DISPENSADOR
// En la versión real los parámetros llegan por BLE.
// En Wokwi se carga un perfil demo y se modifica por Serial.
// ─────────────────────────────────────────────────────────────────────
struct PerfilDispensador {
    char  nombre_perro[32]   = "Demo";
    float peso_kg            = 15.0f;
    float factor_rer         = 130.0f;
    char  marca_alimento[32] = "RoyalCanin";
    float kcal_por_gramo     = 3.8f;
    int   horarios[4]        = {8, 13, 19, -1};
    int   num_horarios       = 3;
    bool  configurado        = true;  // perfil demo listo desde el inicio
};

PerfilDispensador perfil;

// ─────────────────────────────────────────────────────────────────────
// VARIABLES OPERATIVAS (idénticas al hardware real)
// ─────────────────────────────────────────────────────────────────────
float         masaObjetivoG        = 0.0f;
float         masaInicialG         = 0.0f;
float         consumoEMA           = 50.0f;
float         masaUltimaLectura    = 0.0f;
float         distanciaUltima      = 0.0f;
int           ultimaHoraDispensada = -1;
unsigned long ultimoEnvioTelem     = 0;
unsigned long inicioDispensacion   = 0;
unsigned long ultimaActualizLCD    = 0;
unsigned long ultimaAnimacion      = 0;
int           pixelAnimado         = 0;

// ─────────────────────────────────────────────────────────────────────
// VARIABLES EXCLUSIVAS DE SIMULACIÓN
// ─────────────────────────────────────────────────────────────────────
bool          forzarBoveda   = false;   // "BOVEDA" por Serial activa esto
unsigned long otaSimInicio   = 0;       // timestamp inicio OTA simulado
String        serialBuffer   = "";      // buffer acumulador de Serial

// ─────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────
void  lcdMostrarOTA();
void  lcdMostrarError(const String& motivo);
void  lcdMostrarReposo();
void  lcdMostrarDispensando();
void  lcdMostrarBienvenida();
void  neoColorSolido(uint8_t r, uint8_t g, uint8_t b);
void  neoApagar();
void  neoNivelInventario(float masa);
void  neoGirar(uint8_t r, uint8_t g, uint8_t b);
void  neoParpadeoRojo();
void  motorDetener();
void  motorAvanzar(int vel = 200);
void  publicarTelemetria(const String& alerta = "");
float leerMasaG();
float calcularRacion(float porcentajeHorario = 1.0f);
void  conectarMQTT();
void  imprimirStatus();

// ═══════════════════════════════════════════════════════════════════════
// EDGE COMPUTING — CÁLCULO RER
// Idéntico al hardware real. Los parámetros llegan por Serial (sim BLE).
// ═══════════════════════════════════════════════════════════════════════
float calcularRacion(float porcentajeHorario) {
    if (perfil.peso_kg <= 0.0f || perfil.kcal_por_gramo <= 0.0f) return 0.0f;
    float rer = perfil.factor_rer * powf(perfil.peso_kg, 0.75f);
    return (rer / perfil.kcal_por_gramo) * porcentajeHorario;
}

// ═══════════════════════════════════════════════════════════════════════
// EMA — PREDICCIÓN DE CONSUMO DIARIO
// Idéntica al hardware real.
// ═══════════════════════════════════════════════════════════════════════
void actualizarEMA(float gramosDispensados) {
    consumoEMA = (ALPHA_EMA * gramosDispensados) + ((1.0f - ALPHA_EMA) * consumoEMA);
    Serial.printf("[EMA] Consumo estimado: %.1f g/dia\n", consumoEMA);
}

// ═══════════════════════════════════════════════════════════════════════
// SERIAL — REEMPLAZO DE BLE
//
// En hardware real: la app móvil escribe en características BLE.
// En Wokwi:         el usuario escribe comandos en el Serial Monitor.
//
// Comandos disponibles:
//   CONFIG:nombre,peso_kg,factor_rer   → perfil del perro
//   ALIMENTO:marca,kcal_g              → datos del alimento
//   DISPENSAR                          → dispensación inmediata
//   OTA                                → simular actualización firmware
//   BOVEDA                             → forzar efecto bóveda
//   STATUS                             → telemetría completa
// ═══════════════════════════════════════════════════════════════════════

void imprimirStatus() {
    int nH = max(perfil.num_horarios, 1);
    float dias = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    Serial.println(F("\n[STATUS] ────────────────────────────"));
    Serial.printf("  Estado FSM:   %s\n",
        estadoActual == REPOSO       ? "REPOSO" :
        estadoActual == DISPENSANDO  ? "DISPENSANDO" :
        estadoActual == ERROR        ? "ERROR" : "OTA");
    Serial.printf("  Masa:         %.1f g\n",    masaUltimaLectura);
    Serial.printf("  Distancia US: %.1f cm\n",   distanciaUltima);
    Serial.printf("  Consumo EMA:  %.1f g/dia\n", consumoEMA);
    Serial.printf("  Dias rest.:   %.1f dias\n",  dias);
    Serial.println(F("  ── Perfil ──────────────────────────"));
    Serial.printf("  Perro:        %s, %.1f kg\n", perfil.nombre_perro, perfil.peso_kg);
    Serial.printf("  Factor RER:   %.1f\n",        perfil.factor_rer);
    Serial.printf("  Alimento:     %s, %.2f kcal/g\n", perfil.marca_alimento, perfil.kcal_por_gramo);
    Serial.printf("  Racion/com.:  %.1f g\n",    calcularRacion(1.0f / nH));
    Serial.printf("  RER total:    %.1f g/dia\n", calcularRacion(1.0f));
    Serial.print(F("  Horarios:     "));
    for (int i = 0; i < perfil.num_horarios; i++) Serial.printf("%dh ", perfil.horarios[i]);
    Serial.println();
    Serial.println(F("[STATUS] ────────────────────────────\n"));
}

void procesarComandoSerial(const String& linea) {
    Serial.println("[SERIAL] Recibido: " + linea);

    // ── CONFIG:nombre,peso_kg,factor_rer ─────────────────────────────
    // Simula la característica BLE de perfil del perro
    if (linea.startsWith("CONFIG:")) {
        String params = linea.substring(7);
        int c1 = params.indexOf(',');
        int c2 = params.lastIndexOf(',');
        if (c1 > 0 && c2 > c1) {
            String nombre = params.substring(0, c1);
            float peso    = params.substring(c1 + 1, c2).toFloat();
            float factor  = params.substring(c2 + 1).toFloat();
            if (peso > 0.0f && factor > 0.0f) {
                strncpy(perfil.nombre_perro, nombre.c_str(), 31);
                perfil.peso_kg    = peso;
                perfil.factor_rer = factor;
                perfil.configurado = true;
                int nH = max(perfil.num_horarios, 1);
                Serial.printf("[SIM-BLE] Perfil: %s %.1fkg factor=%.1f → racion=%.1fg\n",
                              perfil.nombre_perro, perfil.peso_kg,
                              perfil.factor_rer, calcularRacion(1.0f / nH));
                ultimaActualizLCD = 0; // forzar actualización inmediata del LCD
            } else {
                Serial.println("[SIM-BLE] ERROR: peso y factor_rer deben ser > 0");
            }
        } else {
            Serial.println("[SIM-BLE] Formato: CONFIG:nombre,peso_kg,factor_rer");
        }
    }

    // ── ALIMENTO:marca,kcal_g ────────────────────────────────────────
    // Simula la característica BLE de datos del alimento
    else if (linea.startsWith("ALIMENTO:")) {
        String params = linea.substring(9);
        int c = params.indexOf(',');
        if (c > 0) {
            String marca = params.substring(0, c);
            float kcal   = params.substring(c + 1).toFloat();
            if (kcal > 0.0f) {
                strncpy(perfil.marca_alimento, marca.c_str(), 31);
                perfil.kcal_por_gramo = kcal;
                Serial.printf("[SIM-BLE] Alimento: %s → %.2f kcal/g\n",
                              perfil.marca_alimento, perfil.kcal_por_gramo);
                ultimaActualizLCD = 0;
            } else {
                Serial.println("[SIM-BLE] ERROR: kcal_g debe ser > 0");
            }
        } else {
            Serial.println("[SIM-BLE] Formato: ALIMENTO:marca,kcal_g");
        }
    }

    // ── DISPENSAR ────────────────────────────────────────────────────
    // Simula comando BLE {"cmd":"dispensar"}
    else if (linea == "DISPENSAR") {
        if (estadoActual == REPOSO && perfil.configurado) {
            int nH             = max(perfil.num_horarios, 1);
            masaObjetivoG      = calcularRacion(1.0f / nH);
            masaInicialG       = masaUltimaLectura;
            inicioDispensacion = millis();
            estadoActual       = DISPENSANDO;
            Serial.printf("[SIM-BLE] Dispensando %.1fg → estado DISPENSANDO\n", masaObjetivoG);
        } else {
            Serial.printf("[SIM-BLE] DISPENSAR ignorado (estado=%d, configurado=%d)\n",
                          estadoActual, perfil.configurado);
        }
    }

    // ── OTA ──────────────────────────────────────────────────────────
    // Simula comando BLE {"cmd":"ota"}
    // En hardware real inicia ArduinoOTA; aquí arranca la animación.
    else if (linea == "OTA") {
        Serial.println("[SIM-OTA] Iniciando simulacion de actualizacion firmware...");
        Serial.println("[SIM-OTA] (En hardware real: pio run -e esp32dev_ota --target upload)");
        otaSimInicio = 0; // resetear para que el handler tome nuevo timestamp
        estadoActual = ACTUALIZANDO_OTA;
        lcdMostrarOTA();
    }

    // ── BOVEDA ───────────────────────────────────────────────────────
    // Fuerza la detección de efecto bóveda (atasco)
    // En hardware real: combinación HX711 alto + HC-SR04 en vacío
    else if (linea == "BOVEDA") {
        Serial.println("[SIM] Forzando efecto boveda...");
        forzarBoveda = true;
    }

    // ── STATUS ───────────────────────────────────────────────────────
    // Simula comando BLE {"cmd":"status"}
    else if (linea == "STATUS") {
        imprimirStatus();
    }

    else {
        Serial.println(F("[SERIAL] Comandos: CONFIG:n,p,f | ALIMENTO:m,k | DISPENSAR | OTA | BOVEDA | STATUS"));
    }
}

// Acumula caracteres del Serial hasta recibir un salto de línea
void leerComandoSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                procesarComandoSerial(serialBuffer);
                serialBuffer = "";
            }
        } else if (c >= 32) { // ignorar caracteres de control
            serialBuffer += c;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// NEOPIXEL — ANIMACIONES POR ESTADO (idénticas al hardware real)
// ═══════════════════════════════════════════════════════════════════════

void neoApagar() { anillo.clear(); anillo.show(); }

void neoColorSolido(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_PIXELS; i++)
        anillo.setPixelColor(i, anillo.Color(r, g, b));
    anillo.show();
}

void neoNivelInventario(float masa) {
    int activos = (int)((masa / CAPACIDAD_TANQUE_G) * NUM_PIXELS);
    anillo.clear();
    for (int i = 0; i < activos && i < NUM_PIXELS; i++) {
        if      (i < NUM_PIXELS / 3)        anillo.setPixelColor(i, anillo.Color(150, 0,   0));
        else if (i < (2 * NUM_PIXELS) / 3)  anillo.setPixelColor(i, anillo.Color(150, 100, 0));
        else                                 anillo.setPixelColor(i, anillo.Color(0,   150, 0));
    }
    anillo.show();
}

void neoGirar(uint8_t r, uint8_t g, uint8_t b) {
    anillo.clear();
    anillo.setPixelColor(pixelAnimado, anillo.Color(r, g, b));
    anillo.setPixelColor((pixelAnimado + 1) % NUM_PIXELS, anillo.Color(r/3, g/3, b/3));
    anillo.show();
    pixelAnimado = (pixelAnimado + 1) % NUM_PIXELS;
}

void neoParpadeoRojo() {
    static bool enc = false;
    enc = !enc;
    if (enc) neoColorSolido(200, 0, 0);
    else     neoApagar();
}

// ═══════════════════════════════════════════════════════════════════════
// LCD — PANTALLAS POR ESTADO
// La línea de estado en REPOSO incluye "[BLE:SIM]" para indicar que
// la versión BLE fue reemplazada por Serial en esta simulación.
// ═══════════════════════════════════════════════════════════════════════

void lcdMostrarBienvenida() {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("Comedero IoT v4.0");
    lcd.setCursor(0, 1); lcd.print("Simulacion Wokwi");
    lcd.setCursor(0, 2); lcd.print("BLE=Serial OTA=SIM");
    lcd.setCursor(4, 3); lcd.print("Iniciando...");
}

void lcdMostrarReposo() {
    lcd.clear();
    // Fila 0: estado + indicador de simulación BLE + hora NTP
    struct tm t;
    char fila0[21];
    if (getLocalTime(&t)) {
        snprintf(fila0, sizeof(fila0), "REPOSO[SIM]  %02d:%02d", t.tm_hour, t.tm_min);
    } else {
        snprintf(fila0, sizeof(fila0), "REPOSO[SIM]  --:--");
    }
    lcd.setCursor(0, 0); lcd.print(fila0);

    // Fila 1: nombre + peso + masa actual en inventario
    char fila1[21];
    snprintf(fila1, sizeof(fila1), "%-7s%4.0fkg%4.0fg",
             perfil.nombre_perro, perfil.peso_kg, masaUltimaLectura);
    lcd.setCursor(0, 1); lcd.print(fila1);

    // Fila 2: marca del alimento + densidad calórica
    char fila2[21];
    snprintf(fila2, sizeof(fila2), "%-10s%.1fk/g",
             perfil.marca_alimento, perfil.kcal_por_gramo);
    lcd.setCursor(0, 2); lcd.print(fila2);

    // Fila 3: días restantes + ración por comida (Edge Computing)
    float dias = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    int   nH   = max(perfil.num_horarios, 1);
    char  fila3[21];
    snprintf(fila3, sizeof(fila3), "Dias:%4.1f R:%4.0fg",
             (dias > 99.9f ? 99.9f : dias), calcularRacion(1.0f / nH));
    lcd.setCursor(0, 3); lcd.print(fila3);
}

void lcdMostrarDispensando() {
    lcd.clear();
    lcd.setCursor(2, 0); lcd.print("> DISPENSANDO <");
    float dispensado = masaInicialG - masaUltimaLectura;
    if (dispensado < 0.0f) dispensado = 0.0f;
    float restante = masaObjetivoG - dispensado;
    if (restante < 0.0f) restante = 0.0f;
    char buf[21];
    snprintf(buf, sizeof(buf), "Obj:  %6.1f g", masaObjetivoG);  lcd.setCursor(0, 1); lcd.print(buf);
    snprintf(buf, sizeof(buf), "Disp: %6.1f g", dispensado);     lcd.setCursor(0, 2); lcd.print(buf);
    snprintf(buf, sizeof(buf), "Rest: %6.1f g", restante);       lcd.setCursor(0, 3); lcd.print(buf);
}

void lcdMostrarError(const String& motivo) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("!! ERROR CRITICO !!");
    lcd.setCursor(0, 1); lcd.print(motivo.substring(0, 20));
    lcd.setCursor(0, 2); lcd.print("Motor detenido");
    lcd.setCursor(0, 3); lcd.print("Recuperando en 10s.");
}

void lcdMostrarOTA() {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("** ACTUALIZANDO **");
    lcd.setCursor(2, 1); lcd.print("Firmware v4.1");
    lcd.setCursor(0, 2); lcd.print("Progreso:   0%");
    lcd.setCursor(0, 3); lcd.print("Simulacion Wokwi");
}

// ═══════════════════════════════════════════════════════════════════════
// MOTOR — CONTROL VIA L298N + PWM (idéntico al hardware real)
// ═══════════════════════════════════════════════════════════════════════

void motorAvanzar(int vel) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(PWM_CANAL, vel);
}

void motorDetener() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(PWM_CANAL, 0);
}

// ═══════════════════════════════════════════════════════════════════════
// SENSORES — FUSIÓN SENSORIAL HX711 + HC-SR04
// Idéntica al hardware real. El efecto bóveda también puede forzarse
// por Serial con el comando "BOVEDA" para la demo.
// ═══════════════════════════════════════════════════════════════════════

float leerMasaG() {
    if (balanza.is_ready()) {
        float lectura = (float)balanza.read() / 20.0f;
        masaUltimaLectura = constrain(lectura, 0.0f, CAPACIDAD_TANQUE_G);
    }
    return masaUltimaLectura;
}

float leerDistanciaCm() {
    digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long dur = pulseIn(PIN_ECHO, HIGH, 30000);
    distanciaUltima = (dur == 0) ? 999.0f : (float)dur * 0.0343f / 2.0f;
    return distanciaUltima;
}

// Fusión HX711 + HC-SR04. "BOVEDA" por Serial también activa esto.
bool detectarEfectoBoveda() {
    if (forzarBoveda) {
        forzarBoveda = false;
        Serial.println("[SIM] Efecto boveda forzado por Serial");
        return true;
    }
    return (distanciaUltima > DIST_VACIO_CM) &&
           (masaUltimaLectura > CAPACIDAD_TANQUE_G * UMBRAL_BOVEDA_PCT);
}

// ═══════════════════════════════════════════════════════════════════════
// MQTT — TELEMETRÍA REAL (idéntico al hardware real)
// Wokwi tiene acceso a internet real, por lo que MQTT funciona.
// ═══════════════════════════════════════════════════════════════════════

void publicarTelemetria(const String& alerta) {
    if (!mqtt.connected()) return;
    const char* etiquetas[] = {"REPOSO", "DISPENSANDO", "ERROR", "OTA"};
    JsonDocument doc;
    doc["estado"]         = etiquetas[(int)estadoActual];
    doc["sim"]            = true;          // indicador de simulación
    doc["masa_g"]         = masaUltimaLectura;
    doc["distancia_cm"]   = distanciaUltima;
    doc["consumo_ema_g"]  = consumoEMA;
    doc["dias_restantes"] = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    doc["nombre_perro"]   = perfil.nombre_perro;
    doc["peso_kg"]        = perfil.peso_kg;
    doc["alimento"]       = perfil.marca_alimento;
    doc["kcal_g"]         = perfil.kcal_por_gramo;
    if (alerta.length() > 0) doc["alerta"] = alerta;
    char buf[512];
    serializeJson(doc, buf);
    mqtt.publish(TOPIC_TELEM, buf);
}

void onMensajeMQTT(char* topic, byte* payload, unsigned int length) {
    String msg;
    msg.reserve(length);
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
    Serial.println("[MQTT CMD] " + msg);

    JsonDocument doc;
    if (deserializeJson(doc, msg)) return;
    String tipo = doc["tipo"] | "";
    if (tipo == "dispensar_ahora" && estadoActual == REPOSO && perfil.configurado) {
        int nH             = max(perfil.num_horarios, 1);
        masaObjetivoG      = calcularRacion(1.0f / nH);
        masaInicialG       = masaUltimaLectura;
        inicioDispensacion = millis();
        estadoActual       = DISPENSANDO;
    }
}

void conectarMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt.connected()) return;
    char cid[32];
    snprintf(cid, sizeof(cid), "ESP32_WOKWI_%08X", (unsigned int)esp_random());
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback(onMensajeMQTT);
    if (mqtt.connect(cid)) {
        mqtt.subscribe(TOPIC_CMD);
        Serial.println("[MQTT] Conectado al broker (simulacion)");
    }
}

// ═══════════════════════════════════════════════════════════════════════
// HORARIO AUTOMÁTICO (idéntico al hardware real)
// ═══════════════════════════════════════════════════════════════════════
bool esHoraDeDispensar() {
    if (!perfil.configurado || perfil.num_horarios == 0) return false;
    struct tm t;
    if (!getLocalTime(&t)) return false;
    if (t.tm_hour == ultimaHoraDispensada) return false;
    for (int i = 0; i < perfil.num_horarios; i++) {
        if (t.tm_hour == perfil.horarios[i] && t.tm_min == 0) {
            ultimaHoraDispensada = t.tm_hour;
            return true;
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
// FSM — MANEJADORES DE ESTADO
// ═══════════════════════════════════════════════════════════════════════

void handleReposo() {
    motorDetener();
    leerMasaG();
    leerDistanciaCm();

    if (detectarEfectoBoveda()) {
        Serial.println("[FSM] Efecto boveda → ERROR");
        publicarTelemetria("EFECTO_BOVEDA");
        lcdMostrarError("Atasco detectado");
        estadoActual = ERROR;
        return;
    }

    if (esHoraDeDispensar()) {
        Serial.println("[FSM] Horario → DISPENSANDO");
        int nH             = max(perfil.num_horarios, 1);
        masaObjetivoG      = calcularRacion(1.0f / nH);
        masaInicialG       = masaUltimaLectura;
        inicioDispensacion = millis();
        estadoActual       = DISPENSANDO;
        return;
    }

    if (millis() - ultimaAnimacion > 500) {
        ultimaAnimacion = millis();
        neoNivelInventario(masaUltimaLectura);
    }
    if (millis() - ultimaActualizLCD > 2000) {
        ultimaActualizLCD = millis();
        lcdMostrarReposo();
    }
}

void handleDispensando() {
    leerMasaG();
    leerDistanciaCm();

    if (detectarEfectoBoveda()) {
        motorDetener();
        Serial.println("[FSM] Efecto boveda durante dispensacion → ERROR");
        publicarTelemetria("EFECTO_BOVEDA_CRITICO");
        lcdMostrarError("Atasco critico!");
        estadoActual = ERROR;
        return;
    }

    float dispensado = masaInicialG - masaUltimaLectura;
    if (dispensado < 0.0f) dispensado = 0.0f;

    if (dispensado >= masaObjetivoG) {
        motorDetener();
        actualizarEMA(dispensado);
        publicarTelemetria("DISPENSACION_OK");
        Serial.printf("[FSM] Dispensacion completa: %.1fg\n", dispensado);
        estadoActual = REPOSO;
        return;
    }

    if (millis() - inicioDispensacion > TIMEOUT_MOTOR_MS) {
        motorDetener();
        Serial.println("[FSM] Timeout → ERROR");
        publicarTelemetria("TIMEOUT_MOTOR");
        lcdMostrarError("Timeout dispensador");
        estadoActual = ERROR;
        return;
    }

    motorAvanzar(180);

    if (millis() - ultimaAnimacion > 80) {
        ultimaAnimacion = millis();
        neoGirar(0, 100, 200);
    }
    if (millis() - ultimaActualizLCD > 500) {
        ultimaActualizLCD = millis();
        lcdMostrarDispensando();
    }
}

void handleError() {
    motorDetener();
    if (millis() - ultimaAnimacion > 300) {
        ultimaAnimacion = millis();
        neoParpadeoRojo();
    }
    static unsigned long tEntrada = 0;
    if (tEntrada == 0) tEntrada = millis();
    if (millis() - tEntrada > 10000) {
        tEntrada     = 0;
        estadoActual = REPOSO;
        neoApagar();
        lcd.clear();
        Serial.println("[FSM] Auto-recuperacion → REPOSO");
    }
}

// ───────────────────────────────────────────────────────────────────────
// OTA SIMULADO
//
// En hardware real: ArduinoOTA.handle() recibe el firmware por red.
// En Wokwi: animamos una barra de progreso durante 5 segundos con
//           millis(), actualizando LCD (porcentaje) y NeoPixel (LEDs
//           azules de izquierda a derecha).
// ───────────────────────────────────────────────────────────────────────
void handleActualizandoOTA() {
    if (otaSimInicio == 0) {
        otaSimInicio = millis();
        Serial.println("[SIM-OTA] Descargando firmware... (5 segundos)");
    }

    unsigned long elapsed = millis() - otaSimInicio;
    int pct = (int)min((elapsed * 100UL) / OTA_SIM_DURACION_MS, 100UL);

    // Actualizar NeoPixel: barra azul que crece de izquierda a derecha
    int ledsActivos = (pct * NUM_PIXELS) / 100;
    anillo.clear();
    for (int i = 0; i < ledsActivos && i < NUM_PIXELS; i++)
        anillo.setPixelColor(i, anillo.Color(0, 0, 200));
    anillo.show();

    // Actualizar porcentaje en LCD línea 2 cada 250ms
    static unsigned long ultimoOtaLCD = 0;
    if (millis() - ultimoOtaLCD > 250) {
        ultimoOtaLCD = millis();
        lcd.setCursor(0, 2);
        char buf[21];
        snprintf(buf, sizeof(buf), "Progreso: %3d%%      ", pct);
        lcd.print(buf);
        Serial.printf("[SIM-OTA] %d%%\r", pct);
    }

    // Completado
    if (pct >= 100) {
        Serial.println("\n[SIM-OTA] Actualizacion completada! → v4.1");
        neoColorSolido(0, 200, 0); // verde = éxito
        lcd.clear();
        lcd.setCursor(1, 0); lcd.print("  OTA completado!  ");
        lcd.setCursor(1, 1); lcd.print(" Firmware v4.1     ");
        lcd.setCursor(1, 2); lcd.print(" (simulacion)      ");
        lcd.setCursor(1, 3); lcd.print("  Volviendo...     ");
        delay(2000);
        otaSimInicio = 0;
        estadoActual = REPOSO;
        lcd.clear();
        neoApagar();
    }
}

// ═══════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n╔════════════════════════════╗"));
    Serial.println(F("║  Comedero IoT v4.0-SIM     ║"));
    Serial.println(F("║  Wokwi: BLE=Serial OTA=SIM ║"));
    Serial.println(F("╚════════════════════════════╝"));
    Serial.println(F("\nComandos Serial disponibles:"));
    Serial.println(F("  CONFIG:nombre,peso,factor   → perfil del perro"));
    Serial.println(F("  ALIMENTO:marca,kcal_g       → datos del alimento"));
    Serial.println(F("  DISPENSAR                   → dispensar ahora"));
    Serial.println(F("  OTA                         → simular update"));
    Serial.println(F("  BOVEDA                      → forzar atasco"));
    Serial.println(F("  STATUS                      → ver telemetria\n"));

    // ── Pines ─────────────────────────────────────────────────────────
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_IN1,  OUTPUT);
    pinMode(PIN_IN2,  OUTPUT);

    // ── PWM motor ─────────────────────────────────────────────────────
    ledcSetup(PWM_CANAL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_ENA, PWM_CANAL);

    // ── LCD I2C ───────────────────────────────────────────────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    lcd.init();
    lcd.backlight();
    lcdMostrarBienvenida();

    // ── NeoPixel ──────────────────────────────────────────────────────
    anillo.begin();
    anillo.setBrightness(BRILLO);
    neoColorSolido(0, 50, 0);
    delay(800);

    // ── HX711 ─────────────────────────────────────────────────────────
    balanza.begin(PIN_HX711_DT, PIN_HX711_SCK);
    motorDetener();

    // ── Perfil demo hardcodeado (simula NVS cargado) ──────────────────
    // En hardware real esto viene de Preferences (NVS) o de BLE.
    // En Wokwi usamos valores fijos para la demo.
    Serial.printf("[SIM] Perfil demo: %s %.1fkg factor=%.1f | %s %.2fkcal/g\n",
                  perfil.nombre_perro, perfil.peso_kg, perfil.factor_rer,
                  perfil.marca_alimento, perfil.kcal_por_gramo);

    // ── WiFi Wokwi-GUEST ──────────────────────────────────────────────
    Serial.printf("[WiFi] Conectando a Wokwi-GUEST...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 40) {
        delay(500); Serial.print("."); intentos++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Conectado — IP: %s\n", WiFi.localIP().toString().c_str());
        configTime(GMT_OFFSET, DAYLIGHT_SEC, NTP_SERVER);
        delay(2000);
        conectarMQTT();
    } else {
        Serial.println(F("\n[WiFi] Sin conexion — MQTT y NTP desactivados"));
    }

    Serial.println(F("[SETUP] Sistema listo — escribe comandos en Serial Monitor\n"));
    lcd.clear();
}

// ═══════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════
void loop() {
    // Leer comandos Serial (reemplazo de BLE)
    leerComandoSerial();

    // MQTT reconexión y loop
    if (!mqtt.connected()) conectarMQTT();
    mqtt.loop();

    // FSM
    switch (estadoActual) {
        case REPOSO:           handleReposo();          break;
        case DISPENSANDO:      handleDispensando();     break;
        case ERROR:            handleError();           break;
        case ACTUALIZANDO_OTA: handleActualizandoOTA(); break;
    }

    // Telemetría periódica cada 10 segundos
    if (millis() - ultimoEnvioTelem > 10000) {
        ultimoEnvioTelem = millis();
        publicarTelemetria();
    }

    delay(50);
}
