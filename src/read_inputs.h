#include <Arduino.h>
#include <bitset>
#include <U8g2lib.h>
#include "pin_definitions.h"

#ifndef READ_INPUTS_H
#define READ_INPUTS_H

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

std::bitset<12> readInputs(){
  std::bitset<12> inputs;
  for (int i = 0; i < 3; i++){
    setRow(i);
    delayMicroseconds(3);
    std::bitset<4> cols = readCols();
    for (int j = 0; j < 4; j++){
      inputs[i*4+j] = cols[j];
    }
  }
  return inputs;
}

#endif  // PIN_DEFINITIONS_H