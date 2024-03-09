// pin_definitions.h

#include <Arduino.h>

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

const uint32_t stepSizes[12] = {
  51149156,  //C
  54190643,  //C#
  57412986,  //D
  60826940,  //D#
  64443898,  //E
  68275931,  //F
  72335830,  //F#
  76637142,  //G
  81194224,  //G#
  86022284,  //A
  91137435,  //A#
  96556749   //B
};
const uint32_t Frequencies[12] = {
  261,  //C
  277,  //C#
  293,  //D
  311,  //D#
  329,  //E
  349,  //F
  367,  //F#
  392,  //G
  415,  //G#
  440,  //A
  466,  //A#
  494   //B
};
const float sinPhases[12] = {
  0.0745414257,  //C
  0.079111015,  //C#
  0.0836806043,  //D
  0.0888213923,  //D#
  0.0939621803,  //E
  0.0996741669,  //F
  0.104814955,  //F#
  0.111954938,  //G
  0.118523723,  //G#
  0.125663706,  //A
  0.133089289,  //A#
  0.14108607   //B
};

struct note {
  uint32_t stepSize;
  uint32_t phaseAcc;
  float sinAcc;
  bool active;
  
};

struct {
  std::array<note, 12> notes;
  SemaphoreHandle_t mutex;  
} notes;

void set_notes(){
  for (int i = 0; i < 12; i++){
    notes.notes[i].stepSize = stepSizes[i];
    notes.notes[i].phaseAcc = 0;
    notes.notes[i].active = false;
    Serial.println(notes.notes[i].stepSize);
  }
}

void set_pin_directions(){
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);
}

#endif  // PIN_DEFINITIONS_H
