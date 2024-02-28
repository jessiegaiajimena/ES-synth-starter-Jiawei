#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <cmath>

#include "read_inputs.h"

#include "pin_definitions.h"

//Constants
const uint32_t interval = 100; //Display update interval

struct {
std::bitset<12> inputs;  
} sysState;

volatile uint32_t currentStepSize;
const uint32_t sampleRate = 22000;  //Sample rate
//Step sizes
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
  if (currentStepSize == 0){
    analogWrite(OUTR_PIN, 0);
  }
  else{  
    phaseAcc += currentStepSize;
    int32_t Vout = (phaseAcc >> 24) - 128;
    analogWrite(OUTR_PIN, Vout + 128);
  }

}

void scanKeysTask(void * pvParameters) {

  volatile uint32_t localCurrentStepSize;

  const TickType_t xFrequency1 = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();

  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    std::bitset<12> inputs;
    for (int i = 0; i < 4; i++){
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (int j = 0; j < 4; j++){
        inputs[i*4+j] = cols[j];
      }
    }
    sysState.inputs = readInputs();

    if (sysState.inputs.to_ulong() != 0xFFF){
      for (int i = 0; i < 12; i++){
        if (!sysState.inputs[i]){
          localCurrentStepSize = stepSizes[i];
        }
      }
    }
    else{
      localCurrentStepSize = 0;
    }

    // if (knob.to_ulong() == 3)
    // {
    //   localCurrentStepSize *= 2;
    // }
    // else if (knob.to_ulong() == 12)
    // {
    //   localCurrentStepSize /= 2;
    // }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  static uint32_t count = 0;
  while (1) {
    vTaskDelayUntil( &xLastWakeTime2, xFrequency2);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    //Display inputs
    for (int i = 0; i < 16; i++){
      u8g2.setCursor(5*(i+1), 10);
      u8g2.print(sysState.inputs[i]);
    }
    //Display count
    u8g2.setCursor(5, 30);
    u8g2.print(count++);
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}


void setup() {
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
  
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  vTaskStartScheduler();
}

void loop() {
}