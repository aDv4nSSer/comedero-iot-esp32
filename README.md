# Comedero Inteligente IoT

Sistema de dispensación automática de alimento para mascotas basado en **ESP32**, con configuración por **Bluetooth Low Energy (BLE)**, actualizaciones de firmware **Over-The-Air (OTA)**, cálculo nutricional por **Edge Computing** y telemetría **MQTT**.

---

## Descripción y Problema que Resuelve

Las mascotas domésticas requieren alimentación regular y en porciones controladas según su peso, raza, edad y nivel de actividad. Este proyecto automatiza ese proceso con un sistema IoT que:

- **Calcula la ración exacta** usando la fórmula RER (Resting Energy Requirement) estándar veterinaria
- **Dispensa automáticamente** en horarios configurables
- **Detecta atascos** (efecto bóveda) por fusión de sensor de peso y ultrasonido
- **Predice el inventario** restante usando media móvil exponencial (EMA)
- **Se configura desde el celular** por BLE sin necesidad de internet
- **Se actualiza remotamente** por OTA sin cables ni acceso físico

---

## Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                      APP MÓVIL (BLE)                            │
│   Configura: WiFi / Perfil del perro / Datos del alimento       │
│   Comandos:  dispensar / actualizar_peso / ota / status         │
└────────────────────────────┬────────────────────────────────────┘
                             │ BLE (≤10m, sin internet)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ESP32 DevKit V1                           │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐ │
│  │  CAPA CONFIG │  │  CAPA CONECT │  │  CAPA FÍSICA          │ │
│  │  BLE Server  │  │  WiFi + MQTT │  │  HX711 → peso (g)     │ │
│  │  NVS (Flash) │  │  NTP (UTC-3) │  │  HC-SR04 → nivel (cm) │ │
│  └──────┬───────┘  │  OTA Update  │  │  L298N + Motor DC     │ │
│         │          └──────┬───────┘  │  LCD 2004 I2C         │ │
│         └─────────────────┼──────────│  NeoPixel 12 LEDs     │ │
│                           ▼          └───────────────────────┘ │
│              ┌────────────────────────┐                        │
│              │  FSM (Máquina Estados) │                        │
│              │  REPOSO ↔ DISPENSANDO  │                        │
│              │  ERROR ↔ ACTUALIZ_OTA  │                        │
│              └──────────┬─────────────┘                        │
│                         │                                      │
│              ┌──────────▼─────────────┐                        │
│              │   EDGE COMPUTING       │                        │
│              │  RER = f × peso^0.75   │                        │
│              │  Ración = RER / kcal_g │                        │
│              │  EMA consumo diario    │                        │
│              └────────────────────────┘                        │
└────────────────────────────┬────────────────────────────────────┘
                             │ MQTT
                             ▼
                    broker.hivemq.com
                    comedero/telemetria
                    comedero/comando
```

---

## Justificación Técnica de BLE

**¿Por qué BLE y no solo WiFi?**

El dispositivo necesita recibir credenciales WiFi antes de poder conectarse a internet. BLE resuelve el problema del "bootstrap":

| Sin BLE | Con BLE |
|---------|---------|
| Requiere portal captivo (modo AP) o cables | Configuración desde app en segundos |
| El usuario debe conectarse a red del ESP32 | Conexión directa celular ↔ dispositivo |
| Complejidad extra de interfaz web embebida | JSON simple por característica BLE |

BLE también opera cuando WiFi falla (corte de servicio, cambio de router), permitiendo reconexión remota sin acceso físico.

---

## Justificación Técnica de OTA

**¿Por qué OTA es esencial?**

El firmware maneja datos que cambian con el tiempo:

- **Base de datos de alimentos**: nuevas marcas con diferente `kcal/g`
- **Fórmulas RER**: actualizaciones veterinarias NRC o AAFCO
- **Corrección de bugs**: sin desplazamiento físico al dispositivo
- **Nuevas funcionalidades**: sin desoldar ni conectar cables USB

Sin OTA, cada actualización requeriría acceso físico al ESP32, impractical en dispositivos instalados en hogares.

---

## Hardware Requerido

| Componente | Cantidad | Notas |
|-----------|----------|-------|
| ESP32 DevKit V1 | 1 | 38 pines |
| Celda de carga + HX711 | 1 | Rango 0–1 kg |
| HC-SR04 | 1 | Sensor ultrasonido |
| Motor DC | 1 | 6–12V |
| L298N | 1 | Puente H doble |
| LCD 2004 I2C | 1 | 20×4, dirección 0x27 |
| Anillo NeoPixel | 1 | 12 LEDs WS2812B |
| Fuente 12V / 5V | 1 | Al menos 2A |

### Tabla de Pines

| Señal | Pin ESP32 | Componente |
|-------|-----------|-----------|
| HX711_DT | GPIO 16 | HX711 data |
| HX711_SCK | GPIO 17 | HX711 clock |
| TRIG | GPIO 5 | HC-SR04 trigger |
| ECHO | GPIO 18 | HC-SR04 echo |
| IN1 | GPIO 26 | L298N dir 1 |
| IN2 | GPIO 27 | L298N dir 2 |
| ENA | GPIO 25 | L298N enable (PWM) |
| NEOPIXEL | GPIO 19 | NeoPixel DIN |
| SDA | GPIO 21 | LCD I2C data |
| SCL | GPIO 22 | LCD I2C clock |

---

## Comandos BLE

Conectar al dispositivo `Comedero_IoT` con cualquier app BLE (nRF Connect, LightBlue, etc.) y escribir en las características:

### Característica WiFi — `...ab1`
```json
{"ssid": "MiRed", "pass": "MiClave123"}
```

### Característica Perfil — `...ab2`
```json
{
  "nombre": "Firulais",
  "peso_kg": 15.0,
  "factor_rer": 130.0,
  "horarios": [8, 13, 19]
}
```
> `factor_rer`: calculado en la app según raza/edad/actividad. Ejemplos: Sedentario=95, Activo=130, Cachorro=200.

### Característica Alimento — `...ab3`
```json
{"marca": "Royal Canin Medium Adult", "kcal_g": 3.8}
```

### Característica Comando — `...ab4`
```json
{"cmd": "dispensar"}
{"cmd": "actualizar_peso", "peso_kg": 16.5}
{"cmd": "ota"}
{"cmd": "status"}
```

### Característica Status — `...ab5` (Notify)
El ESP32 envía telemetría:
```json
{
  "estado": "REPOSO",
  "masa_g": 650.0,
  "dias_restantes": 13.0,
  "nombre_perro": "Firulais"
}
```

---

## Cómo Hacer Update OTA

### Opción 1: Desde PlatformIO (red local)
```bash
# El ESP32 debe estar en la misma red WiFi
pio run -e esp32dev_ota --target upload
```
El hostname es `ESP32_COMEDERO.local`, password `comedero2024`.

### Opción 2: Desde BLE (sin internet)
1. Conectar por BLE a `Comedero_IoT`
2. Escribir en característica comando:
   ```json
   {"cmd": "ota"}
   ```
3. El ESP32 entra en modo OTA (LCD muestra progreso)
4. Subir desde PlatformIO o `espota.py`

### Opción 3: Desde PlatformIO directo
```bash
# Compilar y subir por cable USB (primera vez)
pio run -e esp32dev --target upload

# Actualizaciones posteriores por red
pio run -e esp32dev_ota --target upload
```

---

## Estructura del Código

```
src/
└── main.cpp          (archivo único, ~500 líneas)
    ├── Includes y constantes
    ├── Definiciones de pines y UUIDs BLE
    ├── struct PerfilDispensador   ← perfil del animal
    ├── Variables operativas
    ├── Forward declarations
    │
    ├── [NVS]    guardarPerfil / cargarPerfil
    ├── [EDGE]   calcularRacion (RER)
    ├── [EMA]    actualizarEMA
    │
    ├── [BLE]    ServidorBLE (callbacks conexión)
    ├── [BLE]    CaracteristicaWiFi
    ├── [BLE]    CaracteristicaPerfil
    ├── [BLE]    CaracteristicaAlimento
    ├── [BLE]    CaracteristicaComando
    ├── [BLE]    iniciarBLE()
    │
    ├── [OTA]    iniciarOTA() con callbacks
    │
    ├── [NEO]    Animaciones NeoPixel por estado
    ├── [LCD]    Pantallas por estado FSM
    ├── [MOTOR]  motorAvanzar / motorDetener
    ├── [SENS]   leerMasaG / leerDistanciaCm / detectarEfectoBoveda
    │
    ├── [MQTT]   publicarTelemetria / publicarTelemetriaBLE
    ├── [MQTT]   onMensajeMQTT / conectarWiFi / conectarMQTT
    ├── [HORA]   esHoraDeDispensar (NTP)
    │
    ├── [FSM]    handleReposo()
    ├── [FSM]    handleDispensando()
    ├── [FSM]    handleError()
    ├── [FSM]    handleActualizandoOTA()
    │
    ├── setup()  ← inicialización ordenada de todos los módulos
    └── loop()   ← OTA + WiFi reconnect + MQTT + FSM + Telemetría
```

### Máquina de Estados (FSM)

```
         horario/BLE cmd          dispensado OK
REPOSO ─────────────────► DISPENSANDO ──────────────► REPOSO
   │                           │ timeout/atasco
   │ efecto bóveda             ▼
   └──────────────────────► ERROR ──── 10s ──────────► REPOSO
                                │
                         cmd BLE "ota"
                                ▼
                        ACTUALIZANDO_OTA
                        (FSM bloqueada)
                                │ reinicio automático
                                ▼
                           [ESP32 restart]
```

---

## Edge Computing — Fórmula RER

```
RER (kcal/día) = factor_rer × peso_kg ^ 0.75
Ración (g)     = RER / kcal_por_gramo × (1 / num_horarios)
```

El **ESP32 no tiene hardcodeado ningún valor nutricional**. La app móvil:
1. Consulta su base de datos de alimentos → envía `kcal_por_gramo` por BLE
2. Calcula `factor_rer` según raza, edad y actividad → envía por BLE
3. El ESP32 solo evalúa la fórmula matemática con esos parámetros

---

## Tecnologías Utilizadas

| Tecnología | Uso |
|-----------|-----|
| ESP32 Arduino Framework | Plataforma de desarrollo |
| PlatformIO | Build system e IDE |
| BLE (ESP32 BLEDevice) | Configuración inicial local |
| ArduinoOTA | Actualización remota de firmware |
| HX711 | Lectura de celda de carga (peso) |
| HC-SR04 | Ultrasonido para nivel/bóveda |
| L298N + PWM | Control de motor DC |
| LCD 2004 I2C (LiquidCrystal_I2C) | Interfaz de usuario local |
| Adafruit NeoPixel | Indicador visual de estado |
| PubSubClient (MQTT) | Telemetría y comandos IoT |
| ArduinoJson v7 | Serialización de mensajes |
| ESP32 Preferences (NVS) | Persistencia en flash |
| NTP (pool.ntp.org) | Hora real para horarios |
| EMA (Exponential Moving Average) | Predicción de consumo |

---

## Configuración PlatformIO

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600
board_build.partitions = min_spiffs.csv

[env:esp32dev_ota]
extends = env:esp32dev
upload_protocol = espota
upload_port = ESP32_COMEDERO.local
upload_flags = --auth=comedero2024
```

> **Nota**: `min_spiffs.csv` es obligatorio porque BLE + WiFi + OTA requieren más espacio para el firmware. Esta partición asigna más flash a la aplicación reduciendo el área SPIFFS.

---

## Licencia

MIT License — libre para uso personal y educativo.
