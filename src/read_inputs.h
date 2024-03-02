#include <Arduino.h>
#include <bitset>
#include <U8g2lib.h>
#include "pin_definitions.h"

#ifndef READ_INPUTS_H
#define READ_INPUTS_H

const std::size_t inputSize = 20;

struct knob{
  int current_knob_value = 8;
  int lastIncrement = 8;
};

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

std::bitset<4> readCols(int rowId){
  setRow(rowId);
  delayMicroseconds(3);
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

std::bitset<inputSize> readInputs(){
  std::bitset<inputSize> inputs;
  //read keys
  for (int i = 0; i < inputSize/4 ; i++){
    std::bitset<4> cols = readCols(i);
    for (int j = 0; j < 4; j++){
      inputs[i*4+j] = cols[j];
    }
  }
  return inputs;
}

template<size_t F, size_t C>
std::bitset<C> extractBits(const std::bitset<F>& inputBits, int startPos, int length) {
    return std::bitset<C>((inputBits.to_ulong() >> startPos) & ((1 << length) - 1));
}

void readOneKnob(knob& knob, std::bitset<2> previous_knobs, std::bitset<2> current_knobs) {
  if ( (previous_knobs == 0b00 && current_knobs == 0b01)
        || (previous_knobs == 0b11 && current_knobs == 0b10)){
    int knobValue = knob.current_knob_value + 1;
    knob.current_knob_value = constrain( knobValue, 0, 8);
    knob.lastIncrement = 1;
  }

  else if ((previous_knobs == 0b01 && current_knobs == 0b00)
        || (previous_knobs == 0b10 && current_knobs == 0b11)){
    int knobValue = knob.current_knob_value - 1;
    knob.current_knob_value = constrain(knobValue, 0, 8);
    knob.lastIncrement = -1;
  }

  else if (previous_knobs[0] != current_knobs[0] && previous_knobs[1] != current_knobs[1]){
    int knobValue = knob.current_knob_value + knob.lastIncrement;
    knob.current_knob_value = constrain(knobValue, 0, 8);
  }
}

void updateKnob(std::array<knob, 4>& knobValues, std::bitset<8>& previous_knobs, std::bitset<8> current_knobs){
  for (int i = 0; i < 4; i++){
    readOneKnob(knobValues[3-i], extractBits<8,2>(previous_knobs, 2*i, 2), extractBits<8,2>(current_knobs, 2*i, 2));
  }
  previous_knobs = current_knobs;
}

#endif  // PIN_DEFINITIONS_H