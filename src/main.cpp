#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <cmath>

#include "read_inputs.h"

#include "pin_definitions.h"

//Constants
const uint32_t interval = 100; //Display update interval

volatile uint8_t TX_Message[8] = {0};

struct {
std::bitset<16> inputs;
SemaphoreHandle_t mutex;  
int knob3_value = 8;
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
    Vout = Vout >> (8 - sysState.knob3_value);
    analogWrite(OUTR_PIN, Vout + 128);
  }

}

void scanKeysTask(void * pvParameters) {

  volatile uint32_t localCurrentStepSize;

  const TickType_t xFrequency1 = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();
  std::bitset<2> previous_knob3("00");
  int previous_knob3_value = 0;
  std::bitset<12> previou_keys;
  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = readInputs();
    xSemaphoreGive(sysState.mutex);

    // std::bitset<12> keys(sysState.inputs.to_ulong() >> 4);
    // std::bitset<4> knob(sysState.inputs.to_ulong() & 0b1111);

    std::bitset<12> keys(sysState.inputs.to_ulong() & 0b111111111111);
    std::bitset<2> current_knob3((sysState.inputs.to_ulong() >> 12) &0b11);
    
    if ( (previous_knob3 == 0b00 && current_knob3 == 0b01)
          || (previous_knob3 == 0b11 && current_knob3 == 0b10)){

      if (sysState.knob3_value >= 0 && sysState.knob3_value < 8){
        sysState.knob3_value += 1;
      }
      previous_knob3_value = 1;
    }

    else if ((previous_knob3 == 0b01 && current_knob3 == 0b00)
            || (previous_knob3 == 0b10 && current_knob3 == 0b11)){

      if (sysState.knob3_value > 0 && sysState.knob3_value <= 8){
        sysState.knob3_value -= 1;
      }
      previous_knob3_value = -1;
    }

    else if (previous_knob3[0] != current_knob3[0] && previous_knob3[1] != current_knob3[1]){
      if (sysState.knob3_value > 0 && sysState.knob3_value < 8){
        sysState.knob3_value += previous_knob3_value;
      }
    }

    previous_knob3 = current_knob3;

    if (keys.to_ulong() != 0xFFF){
      for (int i = 0; i < 12; i++){
        if (!keys[i]){
          localCurrentStepSize = stepSizes[i];
        }
      }
    }
    else{
      localCurrentStepSize = 0;
    }

    previou_keys = keys;

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

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    
    //Display inputs
    for (int i = 0; i < 16; i++){
      u8g2.setCursor(5*(i+1), 10);
      u8g2.print(sysState.inputs[i]);
    }
    xSemaphoreGive(sysState.mutex);

    //Display count
    u8g2.setCursor(120, 10);
    u8g2.print(sysState.knob3_value);

    u8g2.setCursor(66,30);
    u8g2.print((char) TX_Message[0]);
    u8g2.print(TX_Message[1]);
    u8g2.print(TX_Message[2]);
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

  sysState.mutex = xSemaphoreCreateMutex();
  vTaskStartScheduler();
}

void loop() {
}