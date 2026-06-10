/*
 * ═══════════════════════════════════════════════════════════════════════
 * COMEDERO INTELIGENTE IoT — v4.1
 * ═══════════════════════════════════════════════════════════════════════
 * CAMBIOS v4.1 respecto a v4.0:
 *   - Servo GPIO13 reemplaza Motor DC + L298N (compuerta 0°↔90°)
 *   - Buzzer GPIO32 + Vibrador GPIO33 para alertas de atasco
 *   - HX711b GPIO14/15 verifica que alimento llegó al platillo
 *   - Verificación doble pesaje: dispensado OK si platillo >70% objetivo
 *   - MQTT telemetría incluye masa_platillo_g y estado verificacion
 *
 * ARQUITECTURA EN CAPAS:
 *   ┌────────────────────┬──────────────────────┬──────────────────────┐
 *   │  CONFIGURACIÓN     │  CONECTIVIDAD        │  HARDWARE            │
 *   │  BLE (setup init.) │  WiFi + MQTT + NTP   │  HX711 tanque (peso) │
 *   │  NVS (persistencia)│  OTA (fw update)     │  HX711b platillo     │
 *   └────────────────────┴──────────────────────┤  HC-SR04 (nivel)     │
 *                                               │  Servo (compuerta)   │
 *                     FSM (Máquina de Estados)  │  Buzzer + Vibrador   │
 *             REPOSO ↔ DISPENSANDO ↔ ERROR      └──────────────────────┘
 *                         ↕
 *                  ACTUALIZANDO_OTA
 *
 * EDGE COMPUTING — RER (Resting Energy Requirement):
 *   Ración (g) = factor_rer × peso_kg^0.75 / kcal_por_gramo
 *   Parámetros vienen desde la app por BLE. Sin hardcode nutricional.
 *
 * PINES v4.1:
 *   HX711_DT=16  HX711_SCK=17  TRIG=5    ECHO=18
 *   SERVO=13     BUZZER=32     VIBRADOR=33
 *   HX711_DT2=14 HX711_SCK2=15
 *   NEOPIXEL=19  SDA=21        SCL=22
 * ═══════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <time.h>
#include <math.h>

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
// PINES v4.1
// ─────────────────────────────────────────────────────────────────────
#define PIN_HX711_DT    16   // HX711 depósito — data
#define PIN_HX711_SCK   17   // HX711 depósito — clock
#define PIN_TRIG         5   // HC-SR04 trigger
#define PIN_ECHO        18   // HC-SR04 echo
#define PIN_SERVO       13   // Servo compuerta (reemplaza Motor DC + L298N)
#define PIN_BUZZER      32   // Buzzer piezoeléctrico
#define PIN_VIBRADOR    33   // Vibrador físico (activo HIGH)
#define PIN_HX711_DT2   14   // HX711 platillo mascota — data
#define PIN_HX711_SCK2  15   // HX711 platillo mascota — clock
#define PIN_NEOPIXEL    19   // Anillo NeoPixel DIN  ← GPIO19=USB-D- en S3+CDC
#define PIN_SDA         21   // I2C SDA (LCD)
#define PIN_SCL         22   // I2C SCL (LCD)

// ── Verificación de pines incompatibles con ESP32-S3 ──────────────────────
// GPIO32/33 están ocupados por Flash/PSRAM internos en el ESP32-S3-N16R8.
// GPIO19/20 son USB D-/D+ cuando ARDUINO_USB_CDC_ON_BOOT=1.
// Reasignar estos pines antes de compilar para S3.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #error "S3: PIN_BUZZER=32 → Flash interno. Reasignar a GPIO4 u otro libre."
  #error "S3: PIN_VIBRADOR=33 → PSRAM OPI. Reasignar a GPIO8 u otro libre."
  #error "S3: PIN_NEOPIXEL=19 → USB D- (CDC). Reasignar a GPIO38 u otro libre."
#endif

// ─────────────────────────────────────────────────────────────────────
// NEOPIXEL
// ─────────────────────────────────────────────────────────────────────
#define NUM_PIXELS  12
#define BRILLO      80

// ─────────────────────────────────────────────────────────────────────
// BLE — UUIDs del servicio "Comedero_IoT"
// ─────────────────────────────────────────────────────────────────────
#define BLE_SERVICE_UUID       "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_WIFI_UUID     "12345678-1234-1234-1234-123456789ab1"
#define BLE_CHAR_PERFIL_UUID   "12345678-1234-1234-1234-123456789ab2"
#define BLE_CHAR_ALIMENTO_UUID "12345678-1234-1234-1234-123456789ab3"
#define BLE_CHAR_COMANDO_UUID  "12345678-1234-1234-1234-123456789ab4"
#define BLE_CHAR_STATUS_UUID   "12345678-1234-1234-1234-123456789ab5"

// ─────────────────────────────────────────────────────────────────────
// CONSTANTES FÍSICAS DEL SISTEMA
// ─────────────────────────────────────────────────────────────────────
const float CAPACIDAD_TANQUE_G   = 1000.0f;
const float DIST_VACIO_CM        =   20.0f;
const float UMBRAL_BOVEDA_PCT    =    0.30f;
const float ALPHA_EMA            =    0.30f;
const float UMBRAL_VERIF_PCT     =    0.70f; // el 70% del objetivo debe llegar al platillo
const unsigned long TIMEOUT_MOTOR_MS = 30000;

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
Servo             compuerta;          // servo de la compuerta dispensadora
HX711             balanza;            // celda de carga — depósito de alimento
HX711             balanza2;           // celda de carga — platillo de la mascota
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_NeoPixel anillo(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Preferences       prefs;

BLEServer*         bleServer    = nullptr;
BLECharacteristic* bleStatus    = nullptr;
bool               bleConectado = false;

// ─────────────────────────────────────────────────────────────────────
// PERFIL DEL DISPENSADOR (parámetros llegan por BLE desde la app)
// ─────────────────────────────────────────────────────────────────────
struct PerfilDispensador {
    char  wifi_ssid[64]      = "";
    char  wifi_pass[64]      = "";
    char  nombre_perro[32]   = "Sin nombre";
    float peso_kg            = 0.0f;
    float factor_rer         = 95.0f;
    char  marca_alimento[32] = "Generico";
    float kcal_por_gramo     = 3.5f;
    int   horarios[4]        = {8, 13, 19, -1};
    int   num_horarios       = 3;
    bool  configurado        = false;
};

PerfilDispensador perfil;

// ─────────────────────────────────────────────────────────────────────
// VARIABLES OPERATIVAS
// ─────────────────────────────────────────────────────────────────────
float         masaObjetivoG        = 0.0f;
float         masaInicialG         = 0.0f;
float         consumoEMA           = 50.0f;
float         masaUltimaLectura    = 0.0f;
float         masaPlatilloAntes    = 0.0f;  // masa platillo al iniciar dispensación
float         masaPlatilloActual   = 0.0f;  // masa platillo en tiempo real
float         distanciaUltima      = 0.0f;
String        estadoVerificacion   = "PENDIENTE";
int           ultimaHoraDispensada = -1;
unsigned long ultimoEnvioTelem     = 0;
unsigned long inicioDispensacion   = 0;
unsigned long ultimaActualizLCD    = 0;
unsigned long ultimaAnimacion      = 0;
unsigned long ultimoReconectWiFi   = 0;
unsigned long ultimaAlerta         = 0;  // para alertaAtasco cada 5s en ERROR
int           pixelAnimado         = 0;

// ─────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────
void  lcdMostrarOTA();
void  lcdMostrarError(const String& motivo);
void  lcdMostrarReposo();
void  lcdMostrarDispensando();
void  lcdMostrarSinConfigurar();
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
void  publicarTelemetriaBLE();
void  guardarPerfil();
float leerMasaG();
float leerMasaPlatilloG();
float calcularRacion(float porcentajeHorario = 1.0f);
void  conectarWiFi();
void  iniciarOTA();
void  iniciarDispensacion();

// ═══════════════════════════════════════════════════════════════════════
// NVS — PERSISTENCIA EN FLASH
// ═══════════════════════════════════════════════════════════════════════

void guardarPerfil() {
    prefs.begin("comedero", false);
    prefs.putBool("existe", true);
    prefs.putBytes("perfil", &perfil, sizeof(perfil));
    prefs.putFloat("ema", consumoEMA);
    prefs.end();
    Serial.println("[NVS] Perfil guardado");
}

void cargarPerfil() {
    prefs.begin("comedero", true);
    if (prefs.getBool("existe", false)) {
        prefs.getBytes("perfil", &perfil, sizeof(perfil));
        consumoEMA = prefs.getFloat("ema", 50.0f);
        Serial.printf("[NVS] Perfil cargado: %s %.1fkg | %s %.2fkcal/g\n",
                      perfil.nombre_perro, perfil.peso_kg,
                      perfil.marca_alimento, perfil.kcal_por_gramo);
    } else {
        Serial.println("[NVS] Sin perfil — esperando configuracion BLE");
    }
    prefs.end();
}

// ═══════════════════════════════════════════════════════════════════════
// EDGE COMPUTING — CÁLCULO RER
// ═══════════════════════════════════════════════════════════════════════
float calcularRacion(float porcentajeHorario) {
    if (perfil.peso_kg <= 0.0f || perfil.kcal_por_gramo <= 0.0f) return 0.0f;
    float rer = perfil.factor_rer * powf(perfil.peso_kg, 0.75f);
    return (rer / perfil.kcal_por_gramo) * porcentajeHorario;
}

// ═══════════════════════════════════════════════════════════════════════
// EMA — PREDICCIÓN DE CONSUMO DIARIO
// ═══════════════════════════════════════════════════════════════════════
void actualizarEMA(float gramosDispensados) {
    consumoEMA = (ALPHA_EMA * gramosDispensados) + ((1.0f - ALPHA_EMA) * consumoEMA);
    prefs.begin("comedero", false);
    prefs.putFloat("ema", consumoEMA);
    prefs.end();
    Serial.printf("[EMA] Consumo estimado: %.1f g/dia\n", consumoEMA);
}

// ═══════════════════════════════════════════════════════════════════════
// BLE — CALLBACKS
// ═══════════════════════════════════════════════════════════════════════

class ServidorBLE : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        bleConectado = true;
        Serial.println("[BLE] App conectada");
    }
    void onDisconnect(BLEServer*) override {
        bleConectado = false;
        Serial.println("[BLE] App desconectada — reanunciando");
        BLEDevice::startAdvertising();
    }
};

class CaracteristicaWiFi : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        JsonDocument doc;
        if (deserializeJson(doc, c->getValue().c_str())) return;
        strncpy(perfil.wifi_ssid, doc["ssid"] | "", 63);
        strncpy(perfil.wifi_pass, doc["pass"] | "", 63);
        Serial.printf("[BLE] WiFi recibido: %s\n", perfil.wifi_ssid);
        guardarPerfil();
        WiFi.disconnect();
        delay(200);
        conectarWiFi();
    }
};

class CaracteristicaPerfil : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        JsonDocument doc;
        if (deserializeJson(doc, c->getValue().c_str())) return;
        strncpy(perfil.nombre_perro, doc["nombre"] | "Sin nombre", 31);
        perfil.peso_kg    = doc["peso_kg"]    | perfil.peso_kg;
        perfil.factor_rer = doc["factor_rer"] | perfil.factor_rer;
        if (doc["horarios"].is<JsonArray>()) {
            int i = 0;
            for (int h : doc["horarios"].as<JsonArray>()) {
                if (i < 4) perfil.horarios[i++] = h;
            }
            perfil.num_horarios = i;
            for (; i < 4; i++) perfil.horarios[i] = -1;
        }
        perfil.configurado = (perfil.peso_kg > 0.0f && perfil.kcal_por_gramo > 0.0f);
        guardarPerfil();
        Serial.printf("[BLE] Perfil: %s %.1fkg factor=%.1f\n",
                      perfil.nombre_perro, perfil.peso_kg, perfil.factor_rer);
        if (bleStatus) {
            int nH = max(perfil.num_horarios, 1);
            char resp[80];
            snprintf(resp, sizeof(resp),
                     "{\"ok\":true,\"msg\":\"Perfil guardado\",\"racion_g\":%.1f}",
                     calcularRacion(1.0f / nH));
            bleStatus->setValue(resp);
            bleStatus->notify();
        }
    }
};

class CaracteristicaAlimento : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        JsonDocument doc;
        if (deserializeJson(doc, c->getValue().c_str())) return;
        strncpy(perfil.marca_alimento, doc["marca"] | "Generico", 31);
        perfil.kcal_por_gramo = doc["kcal_g"] | 3.5f;
        perfil.configurado = (perfil.peso_kg > 0.0f && perfil.kcal_por_gramo > 0.0f);
        guardarPerfil();
        Serial.printf("[BLE] Alimento: %s — %.2f kcal/g\n",
                      perfil.marca_alimento, perfil.kcal_por_gramo);
        if (bleStatus) {
            bleStatus->setValue("{\"ok\":true,\"msg\":\"Alimento guardado\"}");
            bleStatus->notify();
        }
    }
};

class CaracteristicaComando : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        JsonDocument doc;
        if (deserializeJson(doc, c->getValue().c_str())) return;
        String cmd = doc["cmd"] | "";
        if (cmd == "dispensar") {
            if (estadoActual == REPOSO && perfil.configurado) {
                iniciarDispensacion();
                Serial.println("[BLE] CMD dispensar → DISPENSANDO");
            }
        }
        else if (cmd == "actualizar_peso") {
            float nuevoPeso = doc["peso_kg"] | 0.0f;
            if (nuevoPeso > 0.0f) {
                perfil.peso_kg = nuevoPeso;
                guardarPerfil();
                Serial.printf("[BLE] Peso actualizado: %.1f kg\n", nuevoPeso);
            }
        }
        else if (cmd == "ota") {
            Serial.println("[BLE] CMD ota → ACTUALIZANDO_OTA");
            estadoActual = ACTUALIZANDO_OTA;
            lcdMostrarOTA();
        }
        else if (cmd == "status") {
            publicarTelemetriaBLE();
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════
// BLE — INICIALIZACIÓN
// ═══════════════════════════════════════════════════════════════════════
void iniciarBLE() {
    BLEDevice::init("Comedero_IoT");
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ServidorBLE());
    BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);
    BLECharacteristic* cWifi = svc->createCharacteristic(
        BLE_CHAR_WIFI_UUID, BLECharacteristic::PROPERTY_WRITE);
    cWifi->setCallbacks(new CaracteristicaWiFi());
    BLECharacteristic* cPerfil = svc->createCharacteristic(
        BLE_CHAR_PERFIL_UUID, BLECharacteristic::PROPERTY_WRITE);
    cPerfil->setCallbacks(new CaracteristicaPerfil());
    BLECharacteristic* cAlim = svc->createCharacteristic(
        BLE_CHAR_ALIMENTO_UUID, BLECharacteristic::PROPERTY_WRITE);
    cAlim->setCallbacks(new CaracteristicaAlimento());
    BLECharacteristic* cCmd = svc->createCharacteristic(
        BLE_CHAR_COMANDO_UUID, BLECharacteristic::PROPERTY_WRITE);
    cCmd->setCallbacks(new CaracteristicaComando());
    bleStatus = svc->createCharacteristic(
        BLE_CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    bleStatus->addDescriptor(new BLE2902());
    svc->start();
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Servidor activo — anunciando como 'Comedero_IoT'");
}

// ═══════════════════════════════════════════════════════════════════════
// OTA — ACTUALIZACIÓN REMOTA
// ═══════════════════════════════════════════════════════════════════════
void iniciarOTA() {
    ArduinoOTA.setHostname("ESP32_COMEDERO");
    ArduinoOTA.setPassword("comedero2024");
    ArduinoOTA.onStart([]() {
        estadoActual = ACTUALIZANDO_OTA;
        cerrarCompuerta();
        Serial.println("[OTA] Iniciando...");
        lcdMostrarOTA();
        neoColorSolido(0, 0, 200);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int pct = (int)((progress * 100UL) / total);
        Serial.printf("[OTA] %d%%\r", pct);
        int ledsActivos = (pct * NUM_PIXELS) / 100;
        anillo.clear();
        for (int i = 0; i < ledsActivos && i < NUM_PIXELS; i++)
            anillo.setPixelColor(i, anillo.Color(0, 0, 200));
        anillo.show();
        lcd.setCursor(0, 2);
        char linea[21];
        snprintf(linea, sizeof(linea), "Progreso: %3d%%      ", pct);
        lcd.print(linea);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Completado — reiniciando...");
        neoColorSolido(0, 200, 0);
        lcd.clear();
        lcd.setCursor(1, 0); lcd.print("  OTA completado!  ");
        lcd.setCursor(1, 1); lcd.print("   Reiniciando...  ");
        delay(1500);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        const char* msg = "Error desconocido";
        if      (error == OTA_AUTH_ERROR)    msg = "Auth fallida";
        else if (error == OTA_BEGIN_ERROR)   msg = "Begin fallido";
        else if (error == OTA_CONNECT_ERROR) msg = "Sin conexion";
        else if (error == OTA_RECEIVE_ERROR) msg = "Error recepcion";
        else if (error == OTA_END_ERROR)     msg = "Error final";
        Serial.printf("[OTA] Error: %s\n", msg);
        estadoActual = ERROR;
        lcdMostrarError(String("[OTA] ") + msg);
    });
    ArduinoOTA.begin();
    Serial.println("[OTA] Listo — hostname: ESP32_COMEDERO");
}

// ═══════════════════════════════════════════════════════════════════════
// NEOPIXEL — ANIMACIONES POR ESTADO
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
// ═══════════════════════════════════════════════════════════════════════

void lcdMostrarBienvenida() {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("Comedero IoT v4.1");
    lcd.setCursor(0, 1); lcd.print("BLE + OTA + NVS");
    lcd.setCursor(0, 2); lcd.print("Servo + Verif Plato");
    lcd.setCursor(4, 3); lcd.print("Iniciando...");
}

void lcdMostrarSinConfigurar() {
    lcd.clear();
    lcd.setCursor(3, 0); lcd.print("Sin configurar");
    lcd.setCursor(0, 1); lcd.print("Abre la app y");
    lcd.setCursor(0, 2); lcd.print("conecta por BLE:");
    lcd.setCursor(0, 3); lcd.print(">> 'Comedero_IoT'");
}

void lcdMostrarReposo() {
    lcd.clear();
    struct tm t;
    lcd.setCursor(0, 0);
    if (getLocalTime(&t)) {
        char buf[21];
        snprintf(buf, sizeof(buf), "REPOSO      %02d:%02d", t.tm_hour, t.tm_min);
        lcd.print(buf);
    } else {
        lcd.print("REPOSO      --:--");
    }
    char fila1[21];
    snprintf(fila1, sizeof(fila1), "%-12s %5.0fg", perfil.nombre_perro, masaUltimaLectura);
    lcd.setCursor(0, 1); lcd.print(fila1);
    lcd.setCursor(0, 2); lcd.print(perfil.marca_alimento);
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
    // Línea 1: objetivo del tanque + masa actual del platillo
    snprintf(buf, sizeof(buf), "Obj:%4.0fg Plat:%4.0fg",
             masaObjetivoG, masaPlatilloActual);
    lcd.setCursor(0, 1); lcd.print(buf);
    // Líneas 2-3: dispensado y restante
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
    lcd.setCursor(0, 3); lcd.print("!No desconectar!");
}

// ═══════════════════════════════════════════════════════════════════════
// SERVO — COMPUERTA DISPENSADORA (reemplaza Motor DC + L298N)
//
// 0°  = compuerta cerrada → alimento no fluye (estado de reposo)
// 90° = compuerta abierta → alimento cae al platillo
// ═══════════════════════════════════════════════════════════════════════

void abrirCompuerta() {
    compuerta.write(90);
    Serial.println("[SERVO] Compuerta abierta (90°)");
}

void cerrarCompuerta() {
    compuerta.write(0);
}

// ═══════════════════════════════════════════════════════════════════════
// BUZZER + VIBRADOR — ALERTA DE ATASCO
//
// Se activa cuando se detecta Efecto Bóveda, y luego cada 5s en ERROR.
// 3 beeps de 200ms a 1000Hz + 3 pulsos de vibración de 300ms.
// ═══════════════════════════════════════════════════════════════════════
void alertaAtasco() {
    Serial.println("[ALERTA] Activando buzzer + vibrador");
    for (int i = 0; i < 3; i++) {
        tone(PIN_BUZZER, 1000, 200);
        delay(400);  // 200ms tono + 200ms silencio
    }
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_VIBRADOR, HIGH);
        delay(300);
        digitalWrite(PIN_VIBRADOR, LOW);
        delay(200);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// SENSORES — HX711 TANQUE, HX711 PLATILLO Y HC-SR04
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
        masaPlatilloActual = constrain(lectura, 0.0f, CAPACIDAD_TANQUE_G);
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

// Fusión sensorial HX711 + HC-SR04: bóveda = tanque lleno pero US dice vacío
bool detectarEfectoBoveda() {
    return (distanciaUltima > DIST_VACIO_CM) &&
           (masaUltimaLectura > CAPACIDAD_TANQUE_G * UMBRAL_BOVEDA_PCT);
}

// ═══════════════════════════════════════════════════════════════════════
// MQTT — TELEMETRÍA Y COMANDOS
// ═══════════════════════════════════════════════════════════════════════

void publicarTelemetria(const String& alerta) {
    if (!mqtt.connected()) return;
    const char* etiquetas[] = {"REPOSO", "DISPENSANDO", "ERROR", "OTA"};
    JsonDocument doc;
    doc["estado"]          = etiquetas[(int)estadoActual];
    doc["masa_g"]          = masaUltimaLectura;
    doc["masa_platillo_g"] = masaPlatilloActual;
    doc["verificacion"]    = estadoVerificacion.c_str();
    doc["distancia_cm"]    = distanciaUltima;
    doc["consumo_ema_g"]   = consumoEMA;
    doc["dias_restantes"]  = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    doc["ble_conectado"]   = bleConectado;
    doc["nombre_perro"]    = perfil.nombre_perro;
    doc["peso_kg"]         = perfil.peso_kg;
    doc["alimento"]        = perfil.marca_alimento;
    doc["kcal_g"]          = perfil.kcal_por_gramo;
    if (alerta.length() > 0) doc["alerta"] = alerta;
    char buf[600];
    serializeJson(doc, buf);
    mqtt.publish(TOPIC_TELEM, buf);
}

void publicarTelemetriaBLE() {
    if (!bleStatus || !bleConectado) return;
    const char* etiquetas[] = {"REPOSO", "DISPENSANDO", "ERROR", "OTA"};
    JsonDocument doc;
    doc["estado"]          = etiquetas[(int)estadoActual];
    doc["masa_g"]          = masaUltimaLectura;
    doc["masa_platillo_g"] = masaPlatilloActual;
    doc["verificacion"]    = estadoVerificacion.c_str();
    doc["dias_restantes"]  = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    doc["nombre_perro"]    = perfil.nombre_perro;
    char buf[300];
    serializeJson(doc, buf);
    bleStatus->setValue(buf);
    bleStatus->notify();
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
        Serial.println("[MQTT] Dispensacion por comando remoto");
    }
}

// ═══════════════════════════════════════════════════════════════════════
// WIFI Y MQTT — CONEXIÓN
// ═══════════════════════════════════════════════════════════════════════

void conectarWiFi() {
    if (strlen(perfil.wifi_ssid) == 0) {
        Serial.println("[WiFi] Sin credenciales — esperando BLE");
        return;
    }
    Serial.printf("[WiFi] Conectando a: %s\n", perfil.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(perfil.wifi_ssid, perfil.wifi_pass);
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 40) {
        delay(500); Serial.print("."); intentos++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Conectado — IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.printf("\n[WiFi] Fallo. Codigo: %d\n", WiFi.status());
    }
}

void conectarMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt.connected()) return;
    char cid[32];
    snprintf(cid, sizeof(cid), "ESP32_COMEDERO_%08X", (unsigned int)esp_random());
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
// Centraliza toda la lógica de entrada al estado DISPENSANDO,
// incluyendo captura de masa inicial del platillo para verificación.
// ═══════════════════════════════════════════════════════════════════════
void iniciarDispensacion() {
    int nH             = max(perfil.num_horarios, 1);
    masaObjetivoG      = calcularRacion(1.0f / nH);
    masaInicialG       = masaUltimaLectura;
    masaPlatilloAntes  = leerMasaPlatilloG();  // snapshot antes de abrir compuerta
    estadoVerificacion = "PENDIENTE";
    inicioDispensacion = millis();
    abrirCompuerta();                          // servo a 90°
    estadoActual       = DISPENSANDO;
    Serial.printf("[FSM] Dispensacion iniciada: %.1fg, platillo base=%.1fg\n",
                  masaObjetivoG, masaPlatilloAntes);
}

// ═══════════════════════════════════════════════════════════════════════
// FSM — MANEJADORES DE ESTADO
// ═══════════════════════════════════════════════════════════════════════

void handleReposo() {
    leerMasaG();
    leerDistanciaCm();
    leerMasaPlatilloG();

    if (detectarEfectoBoveda()) {
        Serial.println("[FSM] Efecto boveda en REPOSO");
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
        if (perfil.configurado) lcdMostrarReposo();
        else                    lcdMostrarSinConfigurar();
    }
}

void handleDispensando() {
    leerMasaG();
    leerDistanciaCm();
    leerMasaPlatilloG();  // actualizar platillo en tiempo real

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
        cerrarCompuerta();  // servo a 0°
        actualizarEMA(dispensado);

        // Verificación de que el alimento llegó al platillo
        float recibido = masaPlatilloActual - masaPlatilloAntes;
        if (recibido >= masaObjetivoG * UMBRAL_VERIF_PCT) {
            estadoVerificacion = "OK";
            Serial.printf("[FSM] Verificacion OK: platillo recibio %.1fg (%.0f%%)\n",
                          recibido, (recibido / masaObjetivoG) * 100.0f);
            publicarTelemetria("DISPENSACION_VERIFICADA");
        } else {
            estadoVerificacion = "NO_VERIFICADA";
            Serial.printf("[FSM] Verificacion FALLA: platillo recibio %.1fg de %.1fg (%.0f%%)\n",
                          recibido > 0.0f ? recibido : 0.0f, masaObjetivoG,
                          (recibido / masaObjetivoG) * 100.0f);
            publicarTelemetria("DISPENSACION_NO_VERIFICADA");
        }

        publicarTelemetriaBLE();
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

    // Servo ya está abierto — solo actualizar animación y LCD
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

    // Alerta sonora + vibración cada 5 segundos mientras dure el error
    if (millis() - ultimaAlerta > 5000) {
        ultimaAlerta = millis();
        alertaAtasco();
    }

    // Auto-recuperación tras 10 segundos
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

void handleActualizandoOTA() {
    ArduinoOTA.handle();
}

// ═══════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n╔════════════════════════════╗");
    Serial.println("║  Comedero IoT v4.1         ║");
    Serial.println("║  Servo + Buzzer + Platillo  ║");
    Serial.println("╚════════════════════════════╝\n");

    // ── Pines digitales ──────────────────────────────────────────────
    pinMode(PIN_TRIG,     OUTPUT);
    pinMode(PIN_ECHO,     INPUT);
    pinMode(PIN_BUZZER,   OUTPUT);
    pinMode(PIN_VIBRADOR, OUTPUT);

    // ── Servo compuerta ───────────────────────────────────────────────
    compuerta.setPeriodHertz(50);           // PWM estándar servo 50Hz
    compuerta.attach(PIN_SERVO, 500, 2400); // pulso mínimo/máximo en µs
    cerrarCompuerta();                      // iniciar en posición cerrada

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

    // ── HX711 tanque (depósito) ───────────────────────────────────────
    balanza.begin(PIN_HX711_DT, PIN_HX711_SCK);

    // ── HX711b platillo (verificación de dispensación) ────────────────
    balanza2.begin(PIN_HX711_DT2, PIN_HX711_SCK2);

    // ── NVS ───────────────────────────────────────────────────────────
    cargarPerfil();

    // ── BLE siempre activo ────────────────────────────────────────────
    iniciarBLE();

    // ── WiFi TEMPORAL para pruebas ────────────────────────────────────
    // TEMPORAL - reemplazar con BLE
    strncpy(perfil.wifi_ssid, "Redmi Note 13 PRO 5G", 63);
    strncpy(perfil.wifi_pass, "12345678",              63);
    perfil.configurado = true;

    // ── WiFi + NTP + MQTT + OTA ───────────────────────────────────────
    conectarWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        configTime(GMT_OFFSET, DAYLIGHT_SEC, NTP_SERVER);
        delay(2000);
        conectarMQTT();
        iniciarOTA();
        Serial.println("[SETUP] NTP + MQTT + OTA listos");
    }

    Serial.println("[SETUP] Sistema operativo v4.1\n");
    lcd.clear();
}

// ═══════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════
void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }

    if (WiFi.status() != WL_CONNECTED && strlen(perfil.wifi_ssid) > 0) {
        if (millis() - ultimoReconectWiFi > 30000) {
            ultimoReconectWiFi = millis();
            Serial.println("[WiFi] Reconectando...");
            WiFi.reconnect();
        }
    }

    if (!mqtt.connected()) conectarMQTT();
    mqtt.loop();

    switch (estadoActual) {
        case REPOSO:           handleReposo();          break;
        case DISPENSANDO:      handleDispensando();     break;
        case ERROR:            handleError();           break;
        case ACTUALIZANDO_OTA: handleActualizandoOTA(); break;
    }

    if (millis() - ultimoEnvioTelem > 10000) {
        ultimoEnvioTelem = millis();
        publicarTelemetria();
        publicarTelemetriaBLE();
    }

    delay(50);
}
