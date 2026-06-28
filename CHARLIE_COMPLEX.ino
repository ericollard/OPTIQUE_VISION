/*
 * LoL Shield v1.5 sur Arduino UNO R4 WiFi
 * Pilotage BLE - ledMap Olimex (format AVR) + ISR corrigée
 *
 * Éric Collard - INSPE Clermont Auvergne
 * Bibliothèque requise : ArduinoBLE
 *
 * Protocole BLE :
 *   [0x01, col, row, val]  allume/éteint une LED
 *   [0x02, level]          luminosité globale 1..7
 *   [0x03, n]              figure prédéfinie n (0..21)
 *   [0x04]                 tout éteindre
 *   [0x05, col, row]       basculer une LED
 *   [0x06, b0..b15]        trame complète 126 bits
 */

#include <ArduinoBLE.h>
#include "FspTimer.h"

// ─── Accès direct aux registres GPIO RA4M1 (R4 WiFi) ────────────────────────
// Méthode PFS (Pin Function Select) - 3x plus rapide que digitalWrite
// Source : forum.arduino.cc/t/registers-and-bits/1349243

#define PORTBASE 0x40040000UL
#define PFS_BY(port,pin) *((volatile uint8_t*)(PORTBASE + 0x0800UL + ((port)*0x40UL) + 0x0003 + ((pin)*4)))
#define PFS_OUTPUT 0x04  // PDR=1, PODR=0 → OUTPUT LOW
#define PFS_HIGH   0x05  // PDR=1, PODR=1 → OUTPUT HIGH
#define PFS_INPUT  0x00  // PDR=0 → INPUT

// Mapping broche Arduino → {port RA4M1, pin RA4M1} pour broches 2..13 (R4 WiFi)
// Source : variant.cpp BSP R4 WiFi + bobcousins forum
static const uint16_t hw_pins[14] = {
    0,0,              // D0, D1 non utilisées
    0x0104,           // D2  = P104
    0x0105,           // D3  = P105
    0x0106,           // D4  = P106
    0x0107,           // D5  = P107
    0x010B,           // D6  = P10B (P1 pin 11)
    0x010C,           // D7  = P10C (P1 pin 12)
    0x0303,           // D8  = P303
    0x0302,           // D9  = P302
    0x0105,           // D10 = P105 -- à vérifier
    0x0206,           // D11 = P206 -- MOSI
    0x0207,           // D12 = P207 -- MISO
    0x0102,           // D13 = P102
};

// Inline rapide : direction + valeur en un seul accès registre
static inline void fast_output(uint8_t pin, uint8_t val) {
    uint16_t hw = hw_pins[pin];
    PFS_BY(hw >> 8, hw & 0xFF) = val ? PFS_HIGH : PFS_OUTPUT;
}
static inline void fast_input(uint8_t pin) {
    uint16_t hw = hw_pins[pin];
    PFS_BY(hw >> 8, hw & 0xFF) = PFS_INPUT;
}
// Même table que Charliplexing.cpp Olimex, validée sur UNO
// ledMap[x*2 + y*28]     = anode  (broche 2..13)
// ledMap[x*2 + y*28 + 1] = cathode (broche 2..13)
static const uint8_t ledMap[252] = {
    13, 5,13, 6,13, 7,13, 8,13, 9,13,10,13,11,13,12,13, 4, 4,13,13, 3, 3,13,13, 2, 2,13,
    12, 5,12, 6,12, 7,12, 8,12, 9,12,10,12,11,12,13,12, 4, 4,12,12, 3, 3,12,12, 2, 2,12,
    11, 5,11, 6,11, 7,11, 8,11, 9,11,10,11,12,11,13,11, 4, 4,11,11, 3, 3,11,11, 2, 2,11,
    10, 5,10, 6,10, 7,10, 8,10, 9,10,11,10,12,10,13,10, 4, 4,10,10, 3, 3,10,10, 2, 2,10,
     9, 5, 9, 6, 9, 7, 9, 8, 9,10, 9,11, 9,12, 9,13, 9, 4, 4, 9, 9, 3, 3, 9, 9, 2, 2, 9,
     8, 5, 8, 6, 8, 7, 8, 9, 8,10, 8,11, 8,12, 8,13, 8, 4, 4, 8, 8, 3, 3, 8, 8, 2, 2, 8,
     7, 5, 7, 6, 7, 8, 7, 9, 7,10, 7,11, 7,12, 7,13, 7, 4, 4, 7, 7, 3, 3, 7, 7, 2, 2, 7,
     6, 5, 6, 7, 6, 8, 6, 9, 6,10, 6,11, 6,12, 6,13, 6, 4, 4, 6, 6, 3, 3, 6, 6, 2, 2, 6,
     5, 6, 5, 7, 5, 8, 5, 9, 5,10, 5,11, 5,12, 5,13, 5, 4, 4, 5, 5, 3, 3, 5, 5, 2, 2, 5,
};

// ─── Buffer d'affichage ───────────────────────────────────────────────────────
static volatile uint8_t displayBuffer[24] = {};
static volatile uint8_t brightness = 7;

void lol_set(uint8_t x, uint8_t y, uint8_t c) {
    uint8_t pin_high = ledMap[x*2 + y*28];
    uint8_t pin_low  = ledMap[x*2 + y*28 + 1];
    if (c)
        displayBuffer[(pin_low-2)*2 + (pin_high/8)] |=  (1 << (pin_high & 7));
    else
        displayBuffer[(pin_low-2)*2 + (pin_high/8)] &= ~(1 << (pin_high & 7));
}

void lol_clear() {
    memset((void*)displayBuffer, 0, sizeof(displayBuffer));
}

// ─── ISR Charlieplexing ───────────────────────────────────────────────────────
// Portage fidèle de l'ISR AVR vers GPIO Arduino R4
// Sur AVR : cycle i < 6  -> cathode sur broche i+2 (port D)
//           cycle i >= 6 -> cathode sur broche i-6+8 (port B)
// DDRD/PORTD = broches 2..7, DDRB/PORTB = broches 8..13

static FspTimer charlie_timer;
static volatile uint8_t cycle_i  = 0;
static volatile uint8_t pwm_cnt  = 0;

void charlie_isr(timer_callback_args_t*) {
    // Éteindre toutes les broches (entrée = haute impédance)
    for (uint8_t p = 2; p <= 13; p++) pinMode(p, INPUT);

    if (pwm_cnt < brightness) {
        uint8_t ddrd=0, portd=0, ddrb=0, portb=0;

        if (cycle_i < 6) {
            ddrd  = (1 << (cycle_i+2)) | displayBuffer[cycle_i*2];
            portd =                       displayBuffer[cycle_i*2];
            ddrb  =                       displayBuffer[cycle_i*2+1];
            portb =                       displayBuffer[cycle_i*2+1];
        } else {
            ddrd  =                       displayBuffer[cycle_i*2];
            portd =                       displayBuffer[cycle_i*2];
            ddrb  = (1 << (cycle_i-6)) | displayBuffer[cycle_i*2+1];
            portb =                       displayBuffer[cycle_i*2+1];
        }

        // Port D : broches 2..7
        for (uint8_t p = 2; p <= 7; p++) {
            if ((ddrd >> p) & 1) {
                pinMode(p, OUTPUT);
                // cathode (dans ddr mais pas dans port) = LOW
                // anode   (dans ddr ET dans port)       = HIGH
                digitalWrite(p, (portd >> p) & 1);
            }
        }
        // Port B : broches 8..13
        for (uint8_t p = 8; p <= 13; p++) {
            if ((ddrb >> (p-8)) & 1) {
                pinMode(p, OUTPUT);
                digitalWrite(p, (portb >> (p-8)) & 1);
            }
        }
    }

    cycle_i++;
    if (cycle_i > 11) {
        cycle_i = 0;
        pwm_cnt = (pwm_cnt + 1) & 7;
    }
}

void charlie_init() {
    charlie_timer.begin(TIMER_MODE_PERIODIC, 1, 1,
                        5760.0f, 50.0f, charlie_isr);
    charlie_timer.setup_overflow_irq();
    charlie_timer.open();
    charlie_timer.start();
}

// ─── Figures DIDA-OPTIC ──────────────────────────────────────────────────────
const uint8_t  NB_FIG = 22;
const uint16_t FIGS[NB_FIG][9] = {
    {3,3,3,195,195,195,195,16383,16383},
    {129,483,822,1166,2188,1166,822,483,129},
    {0,0,0,0,128,0,0,0,0},
    {0,0,0,0,320,0,0,0,0},
    {0,0,0,0,544,0,0,0,0},
    {0,0,0,0,1040,0,0,0,0},
    {0,0,0,0,2056,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,320,0,0,0,0},
    {0,0,0,0,1344,0,0,0,0},
    {0,0,0,0,1360,0,0,0,0},
    {0,0,0,0,4369,0,0,0,0},
    {0,128,0,0,1040,0,0,128,0},
    {0,1040,0,0,0,0,0,1040,0},
    {0,0,0,0,16383,0,0,0,0},
    {64,64,64,64,64,64,64,64,64},
    {64,64,64,64,16383,64,64,64,64},
    {4,8,16,32,64,128,256,512,1024},
    {1024,512,256,128,64,32,16,8,4},
    {1028,520,272,160,64,160,272,520,1028},
    {224,1016,1016,2044,2044,2044,1016,1016,224},
    {16383,16383,16383,16383,16383,16383,16383,16383,16383}
};

void show_figure(uint8_t n) {
    if (n >= NB_FIG) return;
    lol_clear();
    for (uint8_t row = 0; row < 9; row++)
        for (uint8_t col = 0; col < 14; col++)
            if ((FIGS[n][row] >> col) & 1) lol_set(col, row, 1);
}

// ─── BLE ─────────────────────────────────────────────────────────────────────
BLEService        svc("19b10000-e8f2-537e-4f6c-d104768a1214");
BLECharacteristic cmd_char(
    "19b10001-e8f2-537e-4f6c-d104768a1214",
    BLEWrite | BLEWriteWithoutResponse, 20);

static uint8_t cur_fig = 2;

void handle_cmd(const uint8_t* d, int len) {
    if (len < 1) return;
    switch (d[0]) {
        case 0x01:
            if (len>=4 && d[1]<14 && d[2]<9) lol_set(d[1],d[2],d[3]);
            break;
        case 0x02:
            if (len>=2) brightness = constrain(d[1],1,7);
            break;
        case 0x03:
            if (len>=2){ cur_fig=d[1]%NB_FIG; show_figure(cur_fig); }
            break;
        case 0x04:
            lol_clear();
            break;
        case 0x05:
            if (len>=3 && d[1]<14 && d[2]<9){
                uint8_t ph = ledMap[d[1]*2 + d[2]*28];
                uint8_t pl = ledMap[d[1]*2 + d[2]*28 + 1];
                uint8_t cur = (displayBuffer[(pl-2)*2+(ph/8)] >> (ph&7)) & 1;
                lol_set(d[1],d[2],cur^1);
            }
            break;
        case 0x06:
            if (len>=17){
                lol_clear();
                for(uint8_t col=0;col<14;col++)
                    for(uint8_t row=0;row<9;row++){
                        uint8_t idx=col*9+row;
                        if((d[1+(idx>>3)]>>(idx&7))&1) lol_set(col,row,1);
                    }
            }
            break;
    }
}

void setup() {
    Serial.begin(9600);
    if (!BLE.begin()) { Serial.println("BLE init failed"); while(true); }
    BLE.setLocalName("LoL-DIDAOPTIC");
    BLE.setAdvertisedService(svc);
    svc.addCharacteristic(cmd_char);
    BLE.addService(svc);
    BLE.advertise();
    Serial.println("BLE ready");
    lol_clear();
    show_figure(cur_fig);
    charlie_init();
}

void loop() {
    BLEDevice central = BLE.central();
    if (central) {
        Serial.print("Connecté : "); Serial.println(central.address());
        while (central.connected()) {
            if (cmd_char.written()) {
                uint8_t buf[20];
                int len = cmd_char.readValue(buf, sizeof(buf));
                handle_cmd(buf, len);
            }
        }
        BLE.advertise();
        Serial.println("Déconnecté");
    }
}