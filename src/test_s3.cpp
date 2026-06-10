#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    48
#define LED_COUNT   1

Adafruit_NeoPixel ledRGB(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    // Con -DARDUINO_USB_CDC_ON_BOOT=1 Serial va por USB nativo
    Serial.begin(115200);
    delay(2000);

    ledRGB.begin();
    ledRGB.setBrightness(50);

    Serial.println("========================");
    Serial.println("ESP32-S3-N16R8 TEST");
    Serial.println("========================");
    Serial.printf("Flash: %d MB\n", 16);
    Serial.printf("PSRAM: %d MB\n", 8);
    Serial.printf("CPU Freq: %d MHz\n", getCpuFrequencyMhz());
    Serial.println("LED RGB: GPIO48");
    Serial.println("========================");
}

void loop() {
    ledRGB.setPixelColor(0, ledRGB.Color(0, 150, 0));
    ledRGB.show();
    Serial.println("ESP32-S3 OK v");
    delay(1000);

    ledRGB.setPixelColor(0, ledRGB.Color(150, 0, 0));
    ledRGB.show();
    delay(500);

    ledRGB.setPixelColor(0, ledRGB.Color(0, 0, 150));
    ledRGB.show();
    delay(500);

    ledRGB.clear();
    ledRGB.show();
    delay(500);
}
