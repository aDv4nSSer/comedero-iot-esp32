/*
 * ═══════════════════════════════════════════════════════════════════════
 * COMEDERO INTELIGENTE IoT — v4.0
 * ═══════════════════════════════════════════════════════════════════════
 * Sistema de dispensación automática de alimento para mascotas.
 *
 * ARQUITECTURA EN CAPAS:
 *   ┌────────────────────┬──────────────────────┬──────────────────┐
 *   │  CONFIGURACIÓN     │  CONECTIVIDAD        │  HARDWARE        │
 *   │  BLE (setup init.) │  WiFi + MQTT + NTP   │  HX711 (peso)    │
 *   │  NVS (persistencia)│  OTA (fw update)     │  HC-SR04 (nivel) │
 *   └────────────────────┴──────────────────────┴──────────────────┘
 *                     FSM (Máquina de Estados Central)
 *             REPOSO ↔ DISPENSANDO ↔ ERROR ↔ ACTUALIZANDO_OTA
 *
 * EDGE COMPUTING — Cálculo RER (Resting Energy Requirement):
 *   Ración (g) = factor_rer × peso_kg^0.75 / kcal_por_gramo
 *   Los parámetros llegan desde la app por BLE. Sin hardcode nutricional.
 *
 * JUSTIFICACIÓN BLE:
 *   Canal de bootstrap local (≤10m, sin internet) para entregar
 *   credenciales WiFi, perfil del perro y datos del alimento.
 *   Elimina la necesidad de cables o interfaces físicas en el primer uso.
 *
 * JUSTIFICACIÓN OTA:
 *   Actualiza firmware remotamente cuando se agregan nuevas marcas de
 *   alimento, se revisan fórmulas RER veterinarias o se corrigen bugs,
 *   sin acceso físico al dispositivo. Esencial para mantenimiento.
 *
 * PINES:
 *   HX711_DT=16  HX711_SCK=17  TRIG=5   ECHO=18
 *   IN1=26       IN2=27        ENA=25
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
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <time.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────
// CONFIGURACIÓN MQTT
// ─────────────────────────────────────────────────────────────────────
const char* MQTT_BROKER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* TOPIC_TELEM = "comedero/telemetria";
const char* TOPIC_CMD   = "comedero/comando";

// ─────────────────────────────────────────────────────────────────────
// NTP — Chile (UTC-3, sin horario de verano)
// ─────────────────────────────────────────────────────────────────────
const char* NTP_SERVER   = "pool.ntp.org";
const long  GMT_OFFSET   = -3 * 3600;
const int   DAYLIGHT_SEC = 0;

// ─────────────────────────────────────────────────────────────────────
// PINES
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

// ─────────────────────────────────────────────────────────────────────
// PWM MOTOR
// ─────────────────────────────────────────────────────────────────────
#define PWM_CANAL   0
#define PWM_FREQ    5000
#define PWM_RES     8

// ─────────────────────────────────────────────────────────────────────
// NEOPIXEL
// ─────────────────────────────────────────────────────────────────────
#define NUM_PIXELS  12
#define BRILLO      80

// ─────────────────────────────────────────────────────────────────────
// BLE — UUIDs del servicio "Comedero_IoT"
// ─────────────────────────────────────────────────────────────────────
#define BLE_SERVICE_UUID       "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_WIFI_UUID     "12345678-1234-1234-1234-123456789ab1"  // RX credenciales WiFi
#define BLE_CHAR_PERFIL_UUID   "12345678-1234-1234-1234-123456789ab2"  // RX perfil del perro
#define BLE_CHAR_ALIMENTO_UUID "12345678-1234-1234-1234-123456789ab3"  // RX datos del alimento
#define BLE_CHAR_COMANDO_UUID  "12345678-1234-1234-1234-123456789ab4"  // RX comandos
#define BLE_CHAR_STATUS_UUID   "12345678-1234-1234-1234-123456789ab5"  // TX notify → app

// ─────────────────────────────────────────────────────────────────────
// CONSTANTES FÍSICAS DEL SISTEMA (parámetros del hardware, no del perro)
// ─────────────────────────────────────────────────────────────────────
const float CAPACIDAD_TANQUE_G = 1000.0f;  // Capacidad máxima del depósito en gramos
const float DIST_VACIO_CM      =   20.0f;  // Distancia US cuando el depósito está vacío
const float UMBRAL_BOVEDA_PCT  =    0.30f; // Umbral relativo para detectar efecto bóveda
const float ALPHA_EMA          =    0.30f; // Factor suavizado EMA para consumo diario
const unsigned long TIMEOUT_MOTOR_MS = 30000; // Tiempo máximo de dispensación

// ─────────────────────────────────────────────────────────────────────
// FSM — ESTADOS DE LA MÁQUINA
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
Preferences       prefs;

BLEServer*         bleServer    = nullptr;
BLECharacteristic* bleStatus    = nullptr;
bool               bleConectado = false;

// ─────────────────────────────────────────────────────────────────────
// PERFIL DEL DISPENSADOR
// Todos los parámetros nutricionales y de mascota son variables input
// recibidos por BLE desde la app. El ESP32 no tiene ningún valor
// nutricional hardcodeado — solo ejecuta el cálculo RER.
// ─────────────────────────────────────────────────────────────────────
struct PerfilDispensador {
    char  wifi_ssid[64]      = "";
    char  wifi_pass[64]      = "";
    char  nombre_perro[32]   = "Sin nombre";
    float peso_kg            = 0.0f;
    float factor_rer         = 95.0f;  // calculado en app según raza/edad/actividad
    char  marca_alimento[32] = "Generico";
    float kcal_por_gramo     = 3.5f;   // varía por marca, viene de base de datos en app
    int   horarios[4]        = {8, 13, 19, -1};  // -1 = slot vacío
    int   num_horarios       = 3;
    bool  configurado        = false;
};

PerfilDispensador perfil;

// ─────────────────────────────────────────────────────────────────────
// VARIABLES OPERATIVAS
// ─────────────────────────────────────────────────────────────────────
float         masaObjetivoG        = 0.0f;
float         masaInicialG         = 0.0f;
float         consumoEMA           = 50.0f; // estimación inicial: 50 g/día
float         masaUltimaLectura    = 0.0f;
float         distanciaUltima      = 0.0f;
int           ultimaHoraDispensada = -1;
unsigned long ultimoEnvioTelem     = 0;
unsigned long inicioDispensacion   = 0;
unsigned long ultimaActualizLCD    = 0;
unsigned long ultimaAnimacion      = 0;
unsigned long ultimoReconectWiFi   = 0;
int           pixelAnimado         = 0;

// ─────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// Necesarios para que el compilador resuelva referencias cruzadas entre
// funciones definidas más abajo y usadas en los callbacks BLE/OTA.
// ─────────────────────────────────────────────────────────────────────
void  lcdMostrarOTA();
void  lcdMostrarError(const String& motivo);
void  lcdMostrarReposo();
void  lcdMostrarDispensando();
void  lcdMostrarSinConfigurar();
void  lcdMostrarBienvenida();
void  neoColorSolido(uint8_t r, uint8_t g, uint8_t b);
void  neoApagar();
void  neoNivelInventario(float masa);
void  neoGirar(uint8_t r, uint8_t g, uint8_t b);
void  neoParpadeoRojo();
void  motorDetener();
void  motorAvanzar(int vel = 200);
void  publicarTelemetria(const String& alerta = "");
void  publicarTelemetriaBLE();
void  guardarPerfil();
float leerMasaG();
float calcularRacion(float porcentajeHorario = 1.0f);
void  conectarWiFi();
void  iniciarOTA();

// ═══════════════════════════════════════════════════════════════════════
// NVS — PERSISTENCIA EN FLASH NO VOLÁTIL
// ═══════════════════════════════════════════════════════════════════════

void guardarPerfil() {
    prefs.begin("comedero", false);
    prefs.putBool("existe", true);           // bandera que indica datos válidos
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
        Serial.println("[NVS] Sin perfil guardado — esperando configuracion BLE");
    }
    prefs.end();
}

// ═══════════════════════════════════════════════════════════════════════
// EDGE COMPUTING — CÁLCULO DE RACIÓN RER
//
// Fórmula estándar veterinaria NRC:
//   RER (kcal/día) = factor_rer × peso_kg ^ 0.75
//   Ración (g)     = RER / kcal_por_gramo × fracción_horaria
//
// La app calcula factor_rer según raza, edad y nivel de actividad.
// El ESP32 solo evalúa la expresión con los valores recibidos.
// ═══════════════════════════════════════════════════════════════════════
float calcularRacion(float porcentajeHorario) {
    if (perfil.peso_kg <= 0.0f || perfil.kcal_por_gramo <= 0.0f) return 0.0f;
    float rer = perfil.factor_rer * powf(perfil.peso_kg, 0.75f);
    return (rer / perfil.kcal_por_gramo) * porcentajeHorario;
}

// ═══════════════════════════════════════════════════════════════════════
// EMA — MEDIA MÓVIL EXPONENCIAL DEL CONSUMO DIARIO
//
// Predice cuántos días de alimento quedan en el depósito.
// α=0.30: equilibrio entre reactividad ante cambios y estabilidad.
// ═══════════════════════════════════════════════════════════════════════
void actualizarEMA(float gramosDispensados) {
    consumoEMA = (ALPHA_EMA * gramosDispensados) + ((1.0f - ALPHA_EMA) * consumoEMA);
    prefs.begin("comedero", false);
    prefs.putFloat("ema", consumoEMA);
    prefs.end();
    Serial.printf("[EMA] Consumo estimado: %.1f g/dia\n", consumoEMA);
}

// ═══════════════════════════════════════════════════════════════════════
// BLE — CALLBACKS DEL SERVIDOR
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

// Característica 1: Credenciales WiFi
// JSON esperado: {"ssid":"MiRed","pass":"MiClave"}
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

// Característica 2: Perfil del perro
// JSON esperado: {"nombre":"Firulais","peso_kg":15.0,"factor_rer":130.0,"horarios":[8,13,19]}
// factor_rer es calculado en la app según raza/edad/nivel de actividad
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

// Característica 3: Datos del alimento
// JSON esperado: {"marca":"Royal Canin","kcal_g":3.8}
// kcal_g viene de base de datos de alimentos en la app móvil
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

// Característica 4: Comandos de control
// {"cmd":"dispensar"}                    → dispensación manual inmediata
// {"cmd":"actualizar_peso","peso_kg":16} → actualizar peso del perro
// {"cmd":"ota"}                          → iniciar actualización OTA
// {"cmd":"status"}                       → solicitar telemetría inmediata
class CaracteristicaComando : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        JsonDocument doc;
        if (deserializeJson(doc, c->getValue().c_str())) return;

        String cmd = doc["cmd"] | "";

        if (cmd == "dispensar") {
            if (estadoActual == REPOSO && perfil.configurado) {
                int nH             = max(perfil.num_horarios, 1);
                masaObjetivoG      = calcularRacion(1.0f / nH);
                masaInicialG       = masaUltimaLectura;
                inicioDispensacion = millis();
                estadoActual       = DISPENSANDO;
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
// BLE — INICIALIZACIÓN DEL SERVIDOR
// Siempre activo, independiente del estado de WiFi.
// Permite configuración local de corto alcance cuando no hay internet.
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

    // Característica de status: ESP32 → app (notify)
    bleStatus = svc->createCharacteristic(
        BLE_CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    bleStatus->addDescriptor(new BLE2902());

    svc->start();
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Servidor activo — anunciando como 'Comedero_IoT'");
}

// ═══════════════════════════════════════════════════════════════════════
// OTA — ACTUALIZACIÓN REMOTA DE FIRMWARE
// Hostname: ESP32_COMEDERO  |  Password: comedero2024
// Desde PlatformIO: pio run -e esp32dev_ota --target upload
// Desde BLE:        {"cmd":"ota"}
// ═══════════════════════════════════════════════════════════════════════
void iniciarOTA() {
    ArduinoOTA.setHostname("ESP32_COMEDERO");
    ArduinoOTA.setPassword("comedero2024");

    ArduinoOTA.onStart([]() {
        estadoActual = ACTUALIZANDO_OTA;
        Serial.println("[OTA] Iniciando actualizacion de firmware...");
        lcdMostrarOTA();
        neoColorSolido(0, 0, 200);  // azul sólido = descargando
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int pct = (int)((progress * 100UL) / total);
        Serial.printf("[OTA] %d%%\r", pct);

        // Barra de progreso en NeoPixel (azul)
        int ledsActivos = (pct * NUM_PIXELS) / 100;
        anillo.clear();
        for (int i = 0; i < ledsActivos && i < NUM_PIXELS; i++)
            anillo.setPixelColor(i, anillo.Color(0, 0, 200));
        anillo.show();

        // Porcentaje en LCD línea 2
        lcd.setCursor(0, 2);
        char linea[21];
        snprintf(linea, sizeof(linea), "Progreso: %3d%%      ", pct);
        lcd.print(linea);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Completado — reiniciando...");
        neoColorSolido(0, 200, 0);  // verde = éxito
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

// REPOSO: mapa de inventario (rojo=bajo, naranja=medio, verde=lleno)
void neoNivelInventario(float masa) {
    int activos = (int)((masa / CAPACIDAD_TANQUE_G) * NUM_PIXELS);
    anillo.clear();
    for (int i = 0; i < activos && i < NUM_PIXELS; i++) {
        if      (i < NUM_PIXELS / 3)           anillo.setPixelColor(i, anillo.Color(150, 0,   0));
        else if (i < (2 * NUM_PIXELS) / 3)     anillo.setPixelColor(i, anillo.Color(150, 100, 0));
        else                                    anillo.setPixelColor(i, anillo.Color(0,   150, 0));
    }
    anillo.show();
}

// DISPENSANDO: efecto de giro cian
void neoGirar(uint8_t r, uint8_t g, uint8_t b) {
    anillo.clear();
    anillo.setPixelColor(pixelAnimado, anillo.Color(r, g, b));
    anillo.setPixelColor((pixelAnimado + 1) % NUM_PIXELS, anillo.Color(r/3, g/3, b/3));
    anillo.show();
    pixelAnimado = (pixelAnimado + 1) % NUM_PIXELS;
}

// ERROR: parpadeo rojo
void neoParpadeoRojo() {
    static bool enc = false;
    enc = !enc;
    if (enc) neoColorSolido(200, 0, 0);
    else     neoApagar();
}

// ═══════════════════════════════════════════════════════════════════════
// LCD — PANTALLAS POR ESTADO (20×4 caracteres)
// ═══════════════════════════════════════════════════════════════════════

void lcdMostrarBienvenida() {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("Comedero IoT v4.0");
    lcd.setCursor(2, 1); lcd.print("BLE + OTA + NVS");
    lcd.setCursor(2, 2); lcd.print("Edge Computing");
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
    // Fila 0: estado + hora NTP
    struct tm t;
    lcd.setCursor(0, 0);
    if (getLocalTime(&t)) {
        char buf[21];
        snprintf(buf, sizeof(buf), "REPOSO      %02d:%02d", t.tm_hour, t.tm_min);
        lcd.print(buf);
    } else {
        lcd.print("REPOSO      --:--");
    }
    // Fila 1: nombre del perro + masa actual
    char fila1[21];
    snprintf(fila1, sizeof(fila1), "%-12s %5.0fg", perfil.nombre_perro, masaUltimaLectura);
    lcd.setCursor(0, 1); lcd.print(fila1);
    // Fila 2: marca del alimento
    lcd.setCursor(0, 2); lcd.print(perfil.marca_alimento);
    // Fila 3: días restantes y ración por comida
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
    lcd.setCursor(2, 1); lcd.print("Firmware v4.0");
    lcd.setCursor(0, 2); lcd.print("Progreso:   0%");
    lcd.setCursor(0, 3); lcd.print("!No desconectar!");
}

// ═══════════════════════════════════════════════════════════════════════
// MOTOR — CONTROL VIA L298N + PWM
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
// SENSORES — HX711 Y HC-SR04 CON FUSIÓN SENSORIAL
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

// Fusión sensorial HX711 + HC-SR04:
// Si US reporta distancia > vacío (depósito aparentemente vacío)
// pero la balanza indica masa significativa → atasco tipo bóveda.
bool detectarEfectoBoveda() {
    return (distanciaUltima > DIST_VACIO_CM) &&
           (masaUltimaLectura > CAPACIDAD_TANQUE_G * UMBRAL_BOVEDA_PCT);
}

// ═══════════════════════════════════════════════════════════════════════
// MQTT — TELEMETRÍA Y COMANDOS REMOTOS
// ═══════════════════════════════════════════════════════════════════════

void publicarTelemetria(const String& alerta) {
    if (!mqtt.connected()) return;
    const char* etiquetas[] = {"REPOSO", "DISPENSANDO", "ERROR", "OTA"};
    JsonDocument doc;
    doc["estado"]         = etiquetas[(int)estadoActual];
    doc["masa_g"]         = masaUltimaLectura;
    doc["distancia_cm"]   = distanciaUltima;
    doc["consumo_ema_g"]  = consumoEMA;
    doc["dias_restantes"] = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    doc["ble_conectado"]  = bleConectado;
    doc["nombre_perro"]   = perfil.nombre_perro;
    doc["peso_kg"]        = perfil.peso_kg;
    doc["alimento"]       = perfil.marca_alimento;
    doc["kcal_g"]         = perfil.kcal_por_gramo;
    if (alerta.length() > 0) doc["alerta"] = alerta;
    char buf[512];
    serializeJson(doc, buf);
    mqtt.publish(TOPIC_TELEM, buf);
}

void publicarTelemetriaBLE() {
    if (!bleStatus || !bleConectado) return;
    const char* etiquetas[] = {"REPOSO", "DISPENSANDO", "ERROR", "OTA"};
    JsonDocument doc;
    doc["estado"]         = etiquetas[(int)estadoActual];
    doc["masa_g"]         = masaUltimaLectura;
    doc["dias_restantes"] = (consumoEMA > 0.1f) ? (masaUltimaLectura / consumoEMA) : 999.0f;
    doc["nombre_perro"]   = perfil.nombre_perro;
    char buf[256];
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
        int nH             = max(perfil.num_horarios, 1);
        masaObjetivoG      = calcularRacion(1.0f / nH);
        masaInicialG       = masaUltimaLectura;
        inicioDispensacion = millis();
        estadoActual       = DISPENSANDO;
        Serial.println("[MQTT] Dispensacion por comando remoto");
    }
}

// ═══════════════════════════════════════════════════════════════════════
// WIFI Y MQTT — CONEXIÓN
// ═══════════════════════════════════════════════════════════════════════

void conectarWiFi() {
    if (strlen(perfil.wifi_ssid) == 0) {
        Serial.println("[WiFi] Sin credenciales — esperando configuracion BLE");
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
        Serial.printf("\n[WiFi] Fallo. Codigo: %d (4=clave incorrecta, 1=red no encontrada)\n",
                      WiFi.status());
    }
}

void conectarMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt.connected()) return;
    // ID único para evitar colisiones en el broker público
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
// HORARIO AUTOMÁTICO — DISPENSACIÓN POR HORA DEL DÍA
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
        Serial.println("[FSM] Efecto boveda detectado en REPOSO");
        publicarTelemetria("EFECTO_BOVEDA");
        lcdMostrarError("Atasco detectado");
        neoParpadeoRojo();
        estadoActual = ERROR;
        return;
    }

    if (esHoraDeDispensar()) {
        Serial.println("[FSM] Horario alcanzado → DISPENSANDO");
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
        if (perfil.configurado) lcdMostrarReposo();
        else                    lcdMostrarSinConfigurar();
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
        publicarTelemetriaBLE();
        Serial.printf("[FSM] Dispensacion completa: %.1fg\n", dispensado);
        estadoActual = REPOSO;
        return;
    }

    if (millis() - inicioDispensacion > TIMEOUT_MOTOR_MS) {
        motorDetener();
        Serial.println("[FSM] Timeout motor → ERROR");
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

    // Auto-recuperación tras 10 segundos
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

// En ACTUALIZANDO_OTA la FSM solo procesa OTA.
// Bloquea dispensación, motor y telemetría durante la transferencia.
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
    Serial.println("║  Comedero Inteligente IoT  ║");
    Serial.println("║  v4.0  BLE + OTA + Edge    ║");
    Serial.println("╚════════════════════════════╝\n");

    // ── Pines digitales ──────────────────────────────────────────────
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_IN1,  OUTPUT);
    pinMode(PIN_IN2,  OUTPUT);

    // ── PWM para velocidad del motor (L298N enable) ──────────────────
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

    // ── HX711 (celda de carga) ────────────────────────────────────────
    balanza.begin(PIN_HX711_DT, PIN_HX711_SCK);
    motorDetener();

    // ── NVS: restaurar perfil guardado entre reinicios ────────────────
    cargarPerfil();

    // ── BLE: siempre activo para configuración y telemetría local ─────
    iniciarBLE();

    // ── WiFi TEMPORAL para pruebas (reemplazar por BLE en producción) ─
    // TEMPORAL - reemplazar con BLE
    strncpy(perfil.wifi_ssid, "Redmi Note 13 PRO 5G", 63);
    strncpy(perfil.wifi_pass, "12345678",              63);
    perfil.configurado = true;

    // ── WiFi: conectar si hay credenciales ────────────────────────────
    conectarWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        configTime(GMT_OFFSET, DAYLIGHT_SEC, NTP_SERVER);
        delay(2000);  // esperar sincronización NTP inicial
        conectarMQTT();
        iniciarOTA();
        Serial.println("[SETUP] NTP + MQTT + OTA listos");
    }

    Serial.println("[SETUP] Sistema operativo\n");
    lcd.clear();
}

// ═══════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════
void loop() {
    // OTA escucha siempre que haya WiFi (no solo en estado ACTUALIZANDO_OTA)
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }

    // Reconexión automática WiFi si se pierde señal
    if (WiFi.status() != WL_CONNECTED && strlen(perfil.wifi_ssid) > 0) {
        if (millis() - ultimoReconectWiFi > 30000) {
            ultimoReconectWiFi = millis();
            Serial.println("[WiFi] Reconectando...");
            WiFi.reconnect();
        }
    }

    // Reconexión MQTT automática
    if (!mqtt.connected()) conectarMQTT();
    mqtt.loop();

    // FSM: ejecutar lógica del estado actual
    switch (estadoActual) {
        case REPOSO:           handleReposo();          break;
        case DISPENSANDO:      handleDispensando();     break;
        case ERROR:            handleError();           break;
        case ACTUALIZANDO_OTA: handleActualizandoOTA(); break;
    }

    // Telemetría periódica cada 10 segundos (MQTT + BLE notify)
    if (millis() - ultimoEnvioTelem > 10000) {
        ultimoEnvioTelem = millis();
        publicarTelemetria();
        publicarTelemetriaBLE();
    }

    delay(50);
}
