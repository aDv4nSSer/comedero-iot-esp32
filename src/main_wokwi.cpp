/*
 * ═══════════════════════════════════════════════════════════════════════
 * COMEDERO INTELIGENTE IoT — v4.1-SIM (Simulación Wokwi)
 * ═══════════════════════════════════════════════════════════════════════
 * Versión adaptada para Wokwi VS Code. Cambios vs hardware real:
 *
 *   BLE      → comandos Serial (Serial Monitor de Wokwi)
 *   OTA      → animación visual de 5 segundos con millis()
 *   Vibrador → pin definido pero sin efecto visual en Wokwi
 *   Buzzer   → tone() funciona nativamente en Wokwi
 *
 * COMANDOS SERIAL (escribir en Serial Monitor):
 *   CONFIG:nombre,peso_kg,factor_rer   → configura perfil del perro
 *   ALIMENTO:marca,kcal_g              → configura alimento
 *   DISPENSAR                          → dispensación manual
 *   PLATILLO                           → simula +80g en platillo
 *   OTA                                → simula actualización firmware
 *   BOVEDA                             → fuerza detección de atasco
 *   STATUS                             → muestra telemetría completa
 *
 * SECUENCIA DE DEMO v4.1 (presentación):
 *   1. Arranque → bienvenida → REPOSO
 *   2. "STATUS"               → ver telemetría en Serial
 *   3. "CONFIG:Rex,20.0,130.0"→ cambiar perfil en vivo
 *   4. "DISPENSAR"            → servo gira 90° + LCD dispensando
 *   5. Bajar slider HX711     → peso del tanque baja
 *   6. "PLATILLO"             → simula +80g en platillo mascota
 *   7. Sistema verifica       → LCD "DISPENSACION OK" o "NO VERIF"
 *   8. "BOVEDA"               → error + buzzer + NeoPixel rojo
 *   9. Esperar 10s            → auto-reset a REPOSO
 *  10. "OTA"                  → barra azul 5s → verde → "v4.1"
 * ═══════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <time.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────
// WIFI WOKWI
// ─────────────────────────────────────────────────────────────────────
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// ─────────────────────────────────────────────────────────────────────
// MQTT Y NTP
// ─────────────────────────────────────────────────────────────────────
const char* MQTT_BROKER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* TOPIC_TELEM = "comedero/telemetria";
const char* TOPIC_CMD   = "comedero/comando";
const char* NTP_SERVER   = "pool.ntp.org";
const long  GMT_OFFSET   = -3 * 3600;
const int   DAYLIGHT_SEC = 0;

// ─────────────────────────────────────────────────────────────────────
// PINES v4.1 (idénticos al hardware real)
// ─────────────────────────────────────────────────────────────────────
#define PIN_HX711_DT    16
#define PIN_HX711_SCK   17
#define PIN_TRIG         5
#define PIN_ECHO        18
#define PIN_SERVO       13
#define PIN_BUZZER      32
#define PIN_VIBRADOR    33   // definido para compatibilidad; sin efecto visual en Wokwi
#define PIN_HX711_DT2   14
#define PIN_HX711_SCK2  15
#define PIN_NEOPIXEL    19
#define PIN_SDA         21
#define PIN_SCL         22

#define NUM_PIXELS  12
#define BRILLO      80

// ─────────────────────────────────────────────────────────────────────
// CONSTANTES
// ─────────────────────────────────────────────────────────────────────
const float CAPACIDAD_TANQUE_G   = 1000.0f;
const float DIST_VACIO_CM        =   20.0f;
const float UMBRAL_BOVEDA_PCT    =    0.30f;
const float ALPHA_EMA            =    0.30f;
const float UMBRAL_VERIF_PCT     =    0.70f;
const unsigned long TIMEOUT_MOTOR_MS    = 30000;
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
Servo             compuerta;
HX711             balanza;
HX711             balanza2;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_NeoPixel anillo(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ─────────────────────────────────────────────────────────────────────
// PERFIL DEMO hardcodeado (en hardware real llega por BLE)
// ─────────────────────────────────────────────────────────────────────
struct PerfilDispensador {
    char  nombre_perro[32]   = "Demo";
    float peso_kg            = 15.0f;
    float factor_rer         = 130.0f;
    char  marca_alimento[32] = "RoyalCanin";
    float kcal_por_gramo     = 3.8f;
    int   horarios[4]        = {8, 13, 19, -1};
    int   num_horarios       = 3;
    bool  configurado        = true;
};

PerfilDispensador perfil;

// ─────────────────────────────────────────────────────────────────────
// VARIABLES OPERATIVAS
// ─────────────────────────────────────────────────────────────────────
float         masaObjetivoG        = 0.0f;
float         masaInicialG         = 0.0f;
float         consumoEMA           = 50.0f;
float         masaUltimaLectura    = 0.0f;
float         masaPlatilloAntes    = 0.0f;
float         masaPlatilloActual   = 0.0f;
float         distanciaUltima      = 0.0f;
String        estadoVerificacion   = "PENDIENTE";
int           ultimaHoraDispensada = -1;
unsigned long ultimoEnvioTelem     = 0;
unsigned long inicioDispensacion   = 0;
unsigned long ultimaActualizLCD    = 0;
unsigned long ultimaAnimacion      = 0;
unsigned long ultimaAlerta         = 0;
int           pixelAnimado         = 0;

// Variables exclusivas de simulación
bool          forzarBoveda  = false;
unsigned long otaSimInicio  = 0;
unsigned long demoInicio    = 0;
String        serialBuffer  = "";

// ─────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────
void  lcdMostrarOTA();
void  lcdMostrarError(const String& motivo);
void  lcdMostrarReposo();
void  lcdMostrarDispensando();
void  lcdMostrarBienvenida();
void  lcdMostrarVerificacion();
void  neoColorSolido(uint8_t r, uint8_t g, uint8_t b);
void  neoApagar();
void  neoNivelInventario(float masa);
void  neoGirar(uint8_t r, uint8_t g, uint8_t b);
void  neoParpadeoRojo();
void  abrirCompuerta();
void  cerrarCompuerta();
void  alertaAtasco();
void  publicarTelemetria(const String& alerta = "");
float leerMasaG();
float leerMasaPlatilloG();
float calcularRacion(float porcentajeHorario = 1.0f);
void  conectarMQTT();
void  imprimirStatus();
void  iniciarDispensacion();

// ═══════════════════════════════════════════════════════════════════════
// EDGE COMPUTING — CÁLCULO RER (idéntico al hardware real)
// ═══════════════════════════════════════════════════════════════════════
float calcularRacion(float porcentajeHorario) {
    if (perfil.peso_kg <= 0.0f || perfil.kcal_por_gramo <= 0.0f) return 0.0f;
    float rer = perfil.factor_rer * powf(perfil.peso_kg, 0.75f);
    return (rer / perfil.kcal_por_gramo) * porcentajeHorario;
}

void actualizarEMA(float gramosDispensados) {
    consumoEMA = (ALPHA_EMA * gramosDispensados) + ((1.0f - ALPHA_EMA) * consumoEMA);
    Serial.printf("[EMA] Consumo estimado: %.1f g/dia\n", consumoEMA);
}

// ═══════════════════════════════════════════════════════════════════════
// SERIAL — REEMPLAZO DE BLE
// ═══════════════════════════════════════════════════════════════════════

void imprimirStatus() {
    int nH = max(perfil.num_horarios, 1);
    float dias = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    Serial.println(F("\n[STATUS] ────────────────────────────────"));
    Serial.printf("  Estado FSM:   %s\n",
        estadoActual == REPOSO      ? "REPOSO" :
        estadoActual == DISPENSANDO ? "DISPENSANDO" :
        estadoActual == ERROR       ? "ERROR" : "OTA");
    Serial.printf("  Masa tanque:  %.1f g\n",    masaUltimaLectura);
    Serial.printf("  Masa platill: %.1f g\n",    masaPlatilloActual);
    Serial.printf("  Verificacion: %s\n",        estadoVerificacion.c_str());
    Serial.printf("  Distancia US: %.1f cm\n",   distanciaUltima);
    Serial.printf("  Consumo EMA:  %.1f g/dia\n", consumoEMA);
    Serial.printf("  Dias rest.:   %.1f dias\n",  dias);
    Serial.println(F("  ── Perfil ──────────────────────────────"));
    Serial.printf("  Perro:        %s, %.1f kg\n", perfil.nombre_perro, perfil.peso_kg);
    Serial.printf("  Factor RER:   %.1f\n",        perfil.factor_rer);
    Serial.printf("  Alimento:     %s, %.2f kcal/g\n", perfil.marca_alimento, perfil.kcal_por_gramo);
    Serial.printf("  Racion/com.:  %.1f g\n",    calcularRacion(1.0f / nH));
    Serial.print(F("  Horarios:     "));
    for (int i = 0; i < perfil.num_horarios; i++) Serial.printf("%dh ", perfil.horarios[i]);
    Serial.println();
    Serial.println(F("[STATUS] ────────────────────────────────\n"));
}

void procesarComandoSerial(const String& linea) {
    Serial.println("[SERIAL] Recibido: " + linea);

    // CONFIG:nombre,peso_kg,factor_rer → simula característica BLE perfil
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
                ultimaActualizLCD = 0;
            } else {
                Serial.println("[SIM-BLE] ERROR: peso y factor > 0");
            }
        } else {
            Serial.println("[SIM-BLE] Formato: CONFIG:nombre,peso_kg,factor_rer");
        }
    }

    // ALIMENTO:marca,kcal_g → simula característica BLE alimento
    else if (linea.startsWith("ALIMENTO:")) {
        String params = linea.substring(9);
        int c = params.indexOf(',');
        if (c > 0) {
            String marca = params.substring(0, c);
            float kcal   = params.substring(c + 1).toFloat();
            if (kcal > 0.0f) {
                strncpy(perfil.marca_alimento, marca.c_str(), 31);
                perfil.kcal_por_gramo = kcal;
                Serial.printf("[SIM-BLE] Alimento: %s %.2f kcal/g\n",
                              perfil.marca_alimento, perfil.kcal_por_gramo);
                ultimaActualizLCD = 0;
            }
        } else {
            Serial.println("[SIM-BLE] Formato: ALIMENTO:marca,kcal_g");
        }
    }

    // DISPENSAR → simula {"cmd":"dispensar"} por BLE
    else if (linea == "DISPENSAR") {
        if (estadoActual == REPOSO && perfil.configurado) {
            iniciarDispensacion();
            Serial.printf("[SIM-BLE] Dispensando %.1fg → servo abierto\n", masaObjetivoG);
        } else {
            Serial.printf("[SIM-BLE] DISPENSAR ignorado (estado=%d)\n", estadoActual);
        }
    }

    // PLATILLO → simula que la mascota recibió alimento en el platillo (+80g)
    // En hardware real: el HX711b lee el peso real; en Wokwi no hay sensor físico
    else if (linea == "PLATILLO") {
        masaPlatilloActual += 80.0f;
        if (masaPlatilloActual > CAPACIDAD_TANQUE_G)
            masaPlatilloActual = CAPACIDAD_TANQUE_G;
        Serial.printf("[SIM] Platillo simulado: +80g → total %.1fg\n", masaPlatilloActual);
        Serial.printf("[SIM] (En hardware: HX711b en GPIO14/15 lee el platillo fisico)\n");
    }

    // OTA → simula {"cmd":"ota"} por BLE
    else if (linea == "OTA") {
        Serial.println("[SIM-OTA] Iniciando simulacion de actualizacion...");
        Serial.println("[SIM-OTA] (Hardware real: pio run -e esp32dev_ota --target upload)");
        otaSimInicio = 0;
        estadoActual = ACTUALIZANDO_OTA;
        lcdMostrarOTA();
    }

    // BOVEDA → fuerza efecto bóveda (atasco)
    else if (linea == "BOVEDA") {
        Serial.println("[SIM] Forzando efecto boveda...");
        forzarBoveda = true;
    }

    // STATUS → muestra telemetría completa
    else if (linea == "STATUS") {
        imprimirStatus();
    }

    else {
        Serial.println(F("[SERIAL] Comandos disponibles:"));
        Serial.println(F("  CONFIG:n,p,f | ALIMENTO:m,k | DISPENSAR"));
        Serial.println(F("  PLATILLO     | OTA          | BOVEDA | STATUS"));
    }
}

void leerComandoSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                procesarComandoSerial(serialBuffer);
                serialBuffer = "";
            }
        } else if (c >= 32) {
            serialBuffer += c;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// NEOPIXEL (idéntico al hardware real)
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
// LCD (idéntico al hardware real con indicador [SIM])
// ═══════════════════════════════════════════════════════════════════════

void lcdMostrarBienvenida() {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("Comedero IoT v4.1");
    lcd.setCursor(0, 1); lcd.print("Simulacion Wokwi");
    lcd.setCursor(0, 2); lcd.print("Servo+Buzzer+Plato");
    lcd.setCursor(4, 3); lcd.print("Iniciando...");
}

void lcdMostrarReposo() {
    lcd.clear();
    struct tm t;
    char fila0[21];
    if (getLocalTime(&t)) {
        snprintf(fila0, sizeof(fila0), "REPOSO[SIM]  %02d:%02d", t.tm_hour, t.tm_min);
    } else {
        snprintf(fila0, sizeof(fila0), "REPOSO[SIM]  --:--");
    }
    lcd.setCursor(0, 0); lcd.print(fila0);
    char fila1[21];
    snprintf(fila1, sizeof(fila1), "%-7s%4.0fkg%4.0fg",
             perfil.nombre_perro, perfil.peso_kg, masaUltimaLectura);
    lcd.setCursor(0, 1); lcd.print(fila1);
    char fila2[21];
    snprintf(fila2, sizeof(fila2), "%-10s%.1fk/g",
             perfil.marca_alimento, perfil.kcal_por_gramo);
    lcd.setCursor(0, 2); lcd.print(fila2);
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
    char buf[21];
    snprintf(buf, sizeof(buf), "Obj:%4.0fg Plat:%4.0fg",
             masaObjetivoG, masaPlatilloActual);
    lcd.setCursor(0, 1); lcd.print(buf);
    float dispensado = masaInicialG - masaUltimaLectura;
    if (dispensado < 0.0f) dispensado = 0.0f;
    float restante = masaObjetivoG - dispensado;
    if (restante < 0.0f) restante = 0.0f;
    snprintf(buf, sizeof(buf), "Dispensado: %5.1fg", dispensado);
    lcd.setCursor(0, 2); lcd.print(buf);
    snprintf(buf, sizeof(buf), "Restante:   %5.1fg", restante);
    lcd.setCursor(0, 3); lcd.print(buf);
}

void lcdMostrarVerificacion() {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(">> Verificacion <<  ");
    float recibido = masaPlatilloActual - masaPlatilloAntes;
    if (recibido < 0.0f) recibido = 0.0f;
    if (estadoVerificacion == "OK") {
        lcd.setCursor(0, 1); lcd.print("DISPENSACION OK!    ");
        lcd.setCursor(0, 2); lcd.print("Alimento verificado ");
    } else {
        lcd.setCursor(0, 1); lcd.print("NO VERIFICADA       ");
        lcd.setCursor(0, 2); lcd.print("Posible atasco tubo ");
    }
    char buf[21];
    snprintf(buf, sizeof(buf), "Platillo: %6.1fg", recibido);
    lcd.setCursor(0, 3); lcd.print(buf);
}

void lcdMostrarError(const String& motivo) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("!! ERROR CRITICO !!");
    lcd.setCursor(0, 1); lcd.print(motivo.substring(0, 20));
    lcd.setCursor(0, 2); lcd.print("Compuerta cerrada");
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
// SERVO (idéntico al hardware real — Wokwi tiene componente wokwi-servo)
// ═══════════════════════════════════════════════════════════════════════

void abrirCompuerta() {
    compuerta.write(90);
    Serial.println("[SERVO] Compuerta abierta (90°)");
}

void cerrarCompuerta() {
    compuerta.write(0);
}

// ═══════════════════════════════════════════════════════════════════════
// BUZZER + VIBRADOR
//
// Buzzer:   tone() funciona en Wokwi con el componente wokwi-buzzer
// Vibrador: pin definido, digitalWrite funciona, pero Wokwi no tiene
//           componente visual de vibrador. El código es el mismo para
//           mantener paridad con el hardware real.
// ═══════════════════════════════════════════════════════════════════════
void alertaAtasco() {
    Serial.println("[ALERTA] Activando buzzer + vibrador (vibrador sin efecto visual en Wokwi)");
    for (int i = 0; i < 3; i++) {
        tone(PIN_BUZZER, 1000, 200);
        delay(400);
    }
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_VIBRADOR, HIGH);
        delay(300);
        digitalWrite(PIN_VIBRADOR, LOW);
        delay(200);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// SENSORES — FUSIÓN HX711 TANQUE + HC-SR04 + HX711b PLATILLO
// ═══════════════════════════════════════════════════════════════════════

float leerMasaG() {
    if (balanza.is_ready()) {
        float lectura = (float)balanza.read() / 20.0f;
        masaUltimaLectura = constrain(lectura, 0.0f, CAPACIDAD_TANQUE_G);
    }
    return masaUltimaLectura;
}

float leerMasaPlatilloG() {
    if (balanza2.is_ready()) {
        float lectura = (float)balanza2.read() / 20.0f;
        // En Wokwi el segundo HX711 puede no tener slider; la simulación
        // usa masaPlatilloActual directamente mediante el comando "PLATILLO"
        float leido = constrain(lectura, 0.0f, CAPACIDAD_TANQUE_G);
        if (leido > 0.0f) masaPlatilloActual = leido;
    }
    return masaPlatilloActual;
}

float leerDistanciaCm() {
    digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long dur = pulseIn(PIN_ECHO, HIGH, 30000);
    distanciaUltima = (dur == 0) ? 999.0f : (float)dur * 0.0343f / 2.0f;
    return distanciaUltima;
}

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
// MQTT (idéntico al hardware real — Wokwi tiene internet real)
// ═══════════════════════════════════════════════════════════════════════

void publicarTelemetria(const String& alerta) {
    if (!mqtt.connected()) return;
    const char* etiquetas[] = {"REPOSO", "DISPENSANDO", "ERROR", "OTA"};
    JsonDocument doc;
    doc["estado"]          = etiquetas[(int)estadoActual];
    doc["sim"]             = true;
    doc["masa_g"]          = masaUltimaLectura;
    doc["masa_platillo_g"] = masaPlatilloActual;
    doc["verificacion"]    = estadoVerificacion.c_str();
    doc["distancia_cm"]    = distanciaUltima;
    doc["consumo_ema_g"]   = consumoEMA;
    doc["dias_restantes"]  = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    doc["nombre_perro"]    = perfil.nombre_perro;
    doc["peso_kg"]         = perfil.peso_kg;
    doc["alimento"]        = perfil.marca_alimento;
    doc["kcal_g"]          = perfil.kcal_por_gramo;
    if (alerta.length() > 0) doc["alerta"] = alerta;
    char buf[600];
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
        iniciarDispensacion();
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
        Serial.println("[MQTT] Conectado al broker");
    }
}

// ═══════════════════════════════════════════════════════════════════════
// HORARIO AUTOMÁTICO
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
// HELPER — INICIAR DISPENSACIÓN
// ═══════════════════════════════════════════════════════════════════════
void iniciarDispensacion() {
    int nH             = max(perfil.num_horarios, 1);
    masaObjetivoG      = calcularRacion(1.0f / nH);
    masaInicialG       = masaUltimaLectura;
    masaPlatilloAntes  = masaPlatilloActual;  // snapshot antes de abrir
    estadoVerificacion = "PENDIENTE";
    inicioDispensacion = millis();
    abrirCompuerta();
    estadoActual       = DISPENSANDO;
    Serial.printf("[FSM] Dispensacion: %.1fg | platillo base=%.1fg\n",
                  masaObjetivoG, masaPlatilloAntes);
    Serial.println("[SIM] Baja el slider HX711 del tanque, luego escribe 'PLATILLO'");
}

// ═══════════════════════════════════════════════════════════════════════
// FSM — MANEJADORES DE ESTADO
// ═══════════════════════════════════════════════════════════════════════

void handleReposo() {
    leerMasaG();
    leerDistanciaCm();

    if (detectarEfectoBoveda()) {
        Serial.println("[FSM] Efecto boveda → ERROR");
        alertaAtasco();
        publicarTelemetria("EFECTO_BOVEDA");
        lcdMostrarError("Atasco detectado");
        estadoActual = ERROR;
        return;
    }

    if (esHoraDeDispensar()) {
        Serial.println("[FSM] Horario → DISPENSANDO");
        iniciarDispensacion();
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
    leerMasaPlatilloG();

    if (detectarEfectoBoveda()) {
        cerrarCompuerta();
        alertaAtasco();
        Serial.println("[FSM] Efecto boveda durante dispensacion → ERROR");
        publicarTelemetria("EFECTO_BOVEDA_CRITICO");
        lcdMostrarError("Atasco critico!");
        estadoActual = ERROR;
        return;
    }

    float dispensado = masaInicialG - masaUltimaLectura;
    if (dispensado < 0.0f) dispensado = 0.0f;

    if (dispensado >= masaObjetivoG) {
        cerrarCompuerta();
        actualizarEMA(dispensado);

        float recibido = masaPlatilloActual - masaPlatilloAntes;
        if (recibido >= masaObjetivoG * UMBRAL_VERIF_PCT) {
            estadoVerificacion = "OK";
            Serial.printf("[FSM] Verificacion OK: platillo recibio %.1fg\n", recibido);
            publicarTelemetria("DISPENSACION_VERIFICADA");
        } else {
            estadoVerificacion = "NO_VERIFICADA";
            Serial.printf("[FSM] Verificacion FALLA: platillo %.1fg de %.1fg\n",
                          recibido > 0.0f ? recibido : 0.0f, masaObjetivoG);
            Serial.println("[SIM] Escribe 'PLATILLO' para simular que alimento llego al platillo");
            publicarTelemetria("DISPENSACION_NO_VERIFICADA");
        }

        lcdMostrarVerificacion();
        delay(2000);
        estadoActual = REPOSO;
        return;
    }

    if (millis() - inicioDispensacion > TIMEOUT_MOTOR_MS) {
        cerrarCompuerta();
        Serial.println("[FSM] Timeout compuerta → ERROR");
        publicarTelemetria("TIMEOUT_SERVO");
        lcdMostrarError("Timeout compuerta");
        estadoActual = ERROR;
        return;
    }

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
    cerrarCompuerta();

    if (millis() - ultimaAnimacion > 300) {
        ultimaAnimacion = millis();
        neoParpadeoRojo();
    }

    if (millis() - ultimaAlerta > 5000) {
        ultimaAlerta = millis();
        alertaAtasco();
    }

    static unsigned long tEntrada = 0;
    if (tEntrada == 0) tEntrada = millis();
    if (millis() - tEntrada > 10000) {
        tEntrada     = 0;
        ultimaAlerta = 0;
        estadoActual = REPOSO;
        neoApagar();
        lcd.clear();
        Serial.println("[FSM] Auto-recuperacion → REPOSO");
    }
}

// OTA simulado con millis() — barra de progreso azul 8 segundos
void handleActualizandoOTA() {
    if (otaSimInicio == 0) {
        otaSimInicio = millis();
        Serial.println("[SIM-OTA] Descargando firmware... (8 segundos)");
    }

    unsigned long elapsed = millis() - otaSimInicio;
    int pct = (int)min((elapsed * 100UL) / 8000UL, 100UL);

    int ledsActivos = (pct * NUM_PIXELS) / 100;
    anillo.clear();
    for (int i = 0; i < ledsActivos && i < NUM_PIXELS; i++)
        anillo.setPixelColor(i, anillo.Color(0, 0, 200));
    anillo.show();

    static unsigned long ultimoOtaLCD = 0;
    if (millis() - ultimoOtaLCD > 250) {
        ultimoOtaLCD = millis();
        lcd.setCursor(0, 2);
        char buf[21];
        snprintf(buf, sizeof(buf), "Progreso: %3d%%      ", pct);
        lcd.print(buf);
        Serial.printf("[SIM-OTA] %d%%\r", pct);
    }

    if (pct >= 100) {
        Serial.println("\n[SIM-OTA] Actualizacion completada! → v4.2");
        neoColorSolido(0, 200, 0);
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("  OTA completado!  ");
        lcd.setCursor(0, 1); lcd.print("  Firmware v4.2    ");
        lcd.setCursor(0, 2); lcd.print("  (simulacion)     ");
        lcd.setCursor(0, 3); lcd.print("  Volviendo...     ");
        delay(2000);
        otaSimInicio = 0;
        estadoActual = REPOSO;
        lcd.clear();
        neoApagar();
    }
}

// ═══════════════════════════════════════════════════════════════════════
// DEMO AUTOMÁTICA — ciclo 95s para presentación
// ═══════════════════════════════════════════════════════════════════════
void ejecutarDemo() {
    if (demoInicio == 0) demoInicio = millis();
    unsigned long t = millis() - demoInicio;

    // FASE 1 — 10s: Simula BLE cambio perfil Senior
    if (t >= 10000 && t < 10100) {
        strncpy(perfil.nombre_perro, "Rex", 31);
        perfil.peso_kg    = 18.0;
        perfil.factor_rer = 70.0;
        perfil.configurado = true;
        Serial.println(">>> [BLE] Perfil: Rex Senior 18kg factor=70");
    }

    // FASE 2 — 20s: Simula BLE cambio alimento
    if (t >= 20000 && t < 20100) {
        strncpy(perfil.marca_alimento, "Hills Senior", 31);
        perfil.kcal_por_gramo = 3.2;
        Serial.println(">>> [BLE] Alimento: Hills Senior 3.2kcal/g");
    }

    // FASE 3 — 30s: Auto dispensar
    if (t >= 30000 && t < 30100) {
        if (estadoActual == REPOSO && perfil.configurado) {
            Serial.println(">>> [AUTO] Dispensacion iniciada por horario");
            iniciarDispensacion();
        }
    }

    // FASE 4 — 30-45s: Bajar masa tanque gradualmente
    if (t >= 30000 && t < 45000 && estadoActual == DISPENSANDO) {
        static unsigned long ultimaBajada = 0;
        if (millis() - ultimaBajada > 400) {
            ultimaBajada = millis();
            masaUltimaLectura = max(0.0f, masaUltimaLectura - 6.0f);
        }
    }

    // FASE 5 — 45s: Platillo recibe alimento
    if (t >= 45000 && t < 45100) {
        masaPlatilloActual += 85.0;
        Serial.println(">>> [SENSOR] Platillo: +85g recibidos");
    }

    // FASE 6 — 65s: Efecto Boveda (fusion sensorial)
    if (t >= 65000 && t < 65100) {
        if (estadoActual == REPOSO) {
            distanciaUltima   = 25.0;
            masaUltimaLectura = 400.0;
            Serial.println(">>> [FUSION] HC-SR04=25cm + HX711=400g = BOVEDA!");
        }
    }

    // FASE 7 — 80s: Simula OTA
    if (t >= 80000 && t < 80100) {
        if (estadoActual == REPOSO || estadoActual == ERROR) {
            Serial.println(">>> [OTA] Nueva version disponible, actualizando...");
            estadoActual = ACTUALIZANDO_OTA;
            otaSimInicio = millis();
            lcdMostrarOTA();
        }
    }

    // FASE 8 — 95s: Reset y nuevo ciclo
    if (t >= 95000) {
        demoInicio         = millis();
        masaUltimaLectura  = 750.0;
        masaPlatilloActual = 0.0;
        distanciaUltima    = 5.0;
        estadoActual       = REPOSO;
        lcd.clear();
        Serial.println(">>> [DEMO] Ciclo completo - reiniciando en 3s");
        delay(3000);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n╔════════════════════════════╗"));
    Serial.println(F("║  Comedero IoT v4.1-SIM     ║"));
    Serial.println(F("║  Servo + Buzzer + Platillo  ║"));
    Serial.println(F("╚════════════════════════════╝"));
    Serial.println(F("\nComandos Serial:"));
    Serial.println(F("  CONFIG:n,p,f | ALIMENTO:m,k"));
    Serial.println(F("  DISPENSAR    | PLATILLO"));
    Serial.println(F("  OTA          | BOVEDA | STATUS\n"));

    // ── Pines ─────────────────────────────────────────────────────────
    pinMode(PIN_TRIG,     OUTPUT);
    pinMode(PIN_ECHO,     INPUT);
    pinMode(PIN_BUZZER,   OUTPUT);
    pinMode(PIN_VIBRADOR, OUTPUT);  // definido aunque Wokwi no lo renderiza

    // ── Servo compuerta ───────────────────────────────────────────────
    compuerta.setPeriodHertz(50);
    compuerta.attach(PIN_SERVO, 500, 2400);
    cerrarCompuerta();

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

    // ── HX711 tanque + HX711b platillo ───────────────────────────────
    balanza.begin(PIN_HX711_DT, PIN_HX711_SCK);
    balanza2.begin(PIN_HX711_DT2, PIN_HX711_SCK2);

    Serial.printf("[SIM] Perfil demo: %s %.1fkg | %s %.2fkcal/g\n",
                  perfil.nombre_perro, perfil.peso_kg,
                  perfil.marca_alimento, perfil.kcal_por_gramo);

    // ── WiFi Wokwi-GUEST ──────────────────────────────────────────────
    Serial.print("[WiFi] Conectando a Wokwi-GUEST");
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
        Serial.println(F("\n[WiFi] Sin conexion"));
    }

    Serial.println(F("[SETUP] Sistema listo — escribe comandos en Serial Monitor\n"));
    lcd.clear();
}

// ═══════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════
void loop() {
    leerComandoSerial();

    if (!mqtt.connected()) conectarMQTT();
    mqtt.loop();

    ejecutarDemo();

    switch (estadoActual) {
        case REPOSO:           handleReposo();          break;
        case DISPENSANDO:      handleDispensando();     break;
        case ERROR:            handleError();           break;
        case ACTUALIZANDO_OTA: handleActualizandoOTA(); break;
    }

    if (millis() - ultimoEnvioTelem > 10000) {
        ultimoEnvioTelem = millis();
        publicarTelemetria();
    }

    delay(50);
}
