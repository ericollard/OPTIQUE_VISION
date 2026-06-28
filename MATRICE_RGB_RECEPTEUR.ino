#include <ArduinoBLE.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   13

#define LED_COUNT 40

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

BLEService rgbService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLECharacteristic rxChar("19b10001-e8f2-537e-4f6c-d104768a1214",
                         BLEWrite | BLEWriteWithoutResponse, 20);

void applyPacket(const uint8_t *d, int len) {
  if (len < 1) return;
  switch (d[0]) {
    case 0x01: // colorier un pixel, sans afficher
      if (len >= 5) strip.setPixelColor(d[1], d[2], d[3], d[4]);
      break;
    case 0x02: // afficher
      strip.show();
      break;
    case 0x03: // remplir tout et afficher
      if (len >= 4) { for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, d[1], d[2], d[3]); strip.show(); }
      break;
    case 0x04: // tout eteindre
      strip.clear(); strip.show();
      break;
    case 0x05: // luminosite et afficher
      if (len >= 2) { strip.setBrightness(d[1]); strip.show(); }
      break;
    case 0x06: // colorier un pixel et afficher
      if (len >= 5) { strip.setPixelColor(d[1], d[2], d[3], d[4]); strip.show(); }
      break;
  }
}

void setup() {
  strip.begin();
  strip.setBrightness(40);
  strip.clear();
  strip.show();

  if (!BLE.begin()) { while (1); }
  BLE.setLocalName("ARD-RGBShield");
  BLE.setDeviceName("ARD-RGBShield");
  BLE.setAdvertisedService(rgbService);
  rgbService.addCharacteristic(rxChar);
  BLE.addService(rgbService);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      if (rxChar.written()) {
        applyPacket(rxChar.value(), rxChar.valueLength());
      }
    }
  }
}