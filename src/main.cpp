#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>

#include "read_inputs.h"

#include "pin_definitions.h"

//Constants
const uint32_t interval = 100; //Display update interval
std::bitset<12> inputs;
volatile uint32_t currentStepSize;
const uint32_t sampleRate = 22000;  //Sample rate
//Step sizes
const uint32_t stepSizes[12] = {
  0,  //C
  0,  //C#
  0,  //D
  0,  //D#
  0,  //E
  0,  //F
  0,  //F#
  0,  //G
  0,  //G#
  static_cast<uint32_t>(pow(2, 32)) * 440 / sampleRate,  //A
  0,  //A#
  0   //B
};

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}



void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  set_pin_directions();

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
  volatile uint32_t localCurrentStepSize;
  

  static uint32_t next = millis();

  static uint32_t count = 0;

  analogWrite(OUTR_PIN,  0);

  while (millis() < next);  //Wait for next interval

  //read inputs
  inputs = readInputs();
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  //Display inputs
  for (int i = 0; i < 12; i++){
    u8g2.setCursor(5*(i+1), 10);
    u8g2.print(inputs[i]);
  }

  //check which key is pressed
  if (inputs.to_ulong() != 0xFFF){
    for (int i = 0; i < 12; i++){
      if (inputs[i]){
        localCurrentStepSize = stepSizes[i];
      }
    }
  }
  else{
    localCurrentStepSize = 0;
  }

  localCurrentStepSize = static_cast<uint32_t>(pow(2, 32)) / 440 / sampleRate,

  localCurrentStepSize = 0;


  __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);


  Serial.println(localCurrentStepSize);

  //Display count
  u8g2.setCursor(5, 30);
  u8g2.print(count++);
  u8g2.sendBuffer();

  next += interval;
}