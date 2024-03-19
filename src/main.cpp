#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <ES_CAN.h>
#include <cmath>

#include "read_inputs.h"
#include "pin_definitions.h"
#include "instrument_synth.h"
#include <stdio.h>
#include <stdatomic.h>

//Constants
const uint32_t interval = 100; //Display update interval

uint32_t ID = 0x123; //CAN ID
uint8_t RX_Message[8] = {0};  //CAN RX message

//Create message input and output queues
//36 messages of 8 bytes, each message takes around 0.7ms to process
QueueHandle_t msgInQ = xQueueCreate(36,8);; // Message input queue
QueueHandle_t msgOutQ = xQueueCreate(36,8);; // Message output queue

SemaphoreHandle_t CAN_TX_Semaphore; //CAN TX semaphore





//Struct to hold system state
struct {
  std::bitset<20> inputs;
  SemaphoreHandle_t mutex;  
  std::array<knob, 4> knobValues;
} sysState;

volatile uint32_t currentStepSize;

// const uint32_t sampleRate = 22000;  //Sample rate

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


// #define SAMPLE_RATE 22000
// #define FREQUENCY 440
// #define TWOPI 6.283185307
const int SAMPLE_BUFFER_SIZE =1100;
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE/2];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE/2];
SemaphoreHandle_t sampleBufferSemaphore;
volatile bool writeBuffer1 = false;

// void init_samplebuffer(){
//   for (int i=0;i<SAMPLE_BUFFER_SIZE/2;i++){
//     sampleBuffer0[i]=0;
//     sampleBuffer1[i]=0;
//   }
// }
void writeToSampleBuffer(uint32_t Vout, uint32_t writeCtr){
  if (writeBuffer1){
                sampleBuffer1[writeCtr] = Vout;
              }
            else{
                sampleBuffer0[writeCtr] = Vout ;
            }
}

void backgroundCalcTask(void * pvParameters){
  static uint32_t  phaseAcc=0;
  static float sinAcc=0;
  static float saxAcc=0;
  static float prevSinAmp=0;
  
  
  while(1){

	  xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
      uint32_t writeCtr=0;
    
  while( writeCtr < SAMPLE_BUFFER_SIZE/2){


      int vol_knob_value=__atomic_load_n(&sysState.knobValues[3].current_knob_value,__ATOMIC_RELAXED);
      int tune_knob_value=__atomic_load_n(&sysState.knobValues[2].current_knob_value,__ATOMIC_RELAXED);
      int version_knob_value=__atomic_load_n(&sysState.knobValues[1].current_knob_value,__ATOMIC_RELAXED);
      bool hasActiveKey=false;
      int keynum=0;
      float sinAmp=0;
      float triAmp=0;
      float pianoAmp=0;
      for (int i = 0; i < 12; i++) {
        if (writeCtr< SAMPLE_BUFFER_SIZE/2){

          
          bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);

          if (version_knob_value ==8){
            
            if (isactive) {
                // uint32_t phaseAcc=__atomic_load_n(&notes.notes[i].phaseAcc,__ATOMIC_RELAXED);
                uint32_t stepSize=__atomic_load_n(&notes.notes[i].stepSize,__ATOMIC_RELAXED);
                
                hasActiveKey=true;
                if ((tune_knob_value-4)>=0){
                  phaseAcc+=stepSize << (tune_knob_value-4);}
                else{
                  phaseAcc+=stepSize >> -(tune_knob_value-4);
                }
                uint32_t Vout = (phaseAcc >> 24) - 128;
                Vout = (Vout >> (8 - vol_knob_value))+128 ;
                // uint32_t Vout =calcSawtoothVout(phaseAcc,vol_knob_value,i);
                
                writeToSampleBuffer(Vout,writeCtr);
                writeCtr+=1;
                
                // notes.notes[i].phaseAcc=phaseAcc;
                }

             
            
          }
          else if(version_knob_value ==7){

              if (isactive) {
                keynum+=1;
                hasActiveKey=true;
                sinAmp+=getSample(notePhases[(tune_knob_value-1)*4+i],&notes.notes[i].sinAcc,sineTable);

                // uint32_t Vout = calcOtherVout(getSample(notePhases[(tune_knob_value-1)*4+i],&sinAcc,sineTable),vol_knob_value,i);
                
                // writeToSampleBuffer(Vout+128, writeCtr);
                // writeCtr+=1;

              }
              if (i==11 && keynum>0){
                sinAmp=sinAmp/keynum;
                sinAmp=calcNoEnvelopeVout(sinAmp,vol_knob_value);
                // uint32_t Vout = static_cast<uint32_t>(sinAmp*127)-128;
                // Vout = Vout >> (8 - vol_knob_value);
                // writeToSampleBuffer(Vout+128, writeCtr);
                // sinAmp+=generateLFO();

                writeToSampleBuffer(int(sinAmp)  , writeCtr);
                writeCtr+=1;
            }

          }
          else if (version_knob_value ==6){


            // }
            if (isactive) {
                // testsinAcc=notes.notes[i].sinAcc;
                keynum+=1;
                hasActiveKey=true;
                // sinAmp+=generateSin(sinPhases[i],&notes.notes[i].sinAcc);
                // ;
                sinAmp+=calcOtherVout( getSample(notePhases[(tune_knob_value-1)*4+i],&notes.notes[i].sinAcc,sineTable),vol_knob_value,i);

                
              }
            if (i==11 && keynum>0){
              sinAmp=sinAmp/keynum;
              // sinAmp=calcNoEnvelopeVout(sinAmp,vol_knob_value);
              writeToSampleBuffer(int(sinAmp)  , writeCtr);
              writeCtr+=1;
            }

          }
          else if (version_knob_value ==5){
            // float testsinAcc=0;
            if (isactive) {
                // testsinAcc=notes.notes[i].sinAcc;
                keynum+=1;
                hasActiveKey=true;
                // sinAmp+=generateSin(sinPhases[i],&notes.notes[i].sinAcc);
                sinAmp+=getSample(notePhases[(tune_knob_value-1)*4+i],&notes.notes[i].sinAcc,pianoTable);
                // testsinAcc+=sinPhases[i];
                // if (testsinAcc>=M_PI){
                //   testsinAcc-=M_PI;
                // }
                // sinAmp+=sin(testsinAcc);
                // notes.notes[i].sinAcc=testsinAcc;
                
              }
            if (i==11 && keynum>0){
              sinAmp=sinAmp/keynum;
              // sinAmp= lowPassFilter(sinAmp,prevSinAmp,500.0);
              // prevSinAmp=sinAmp;
              uint32_t Vout = static_cast<uint32_t>(sinAmp*255)-128;
                Vout = Vout >> (8 - vol_knob_value);
              writeToSampleBuffer(Vout+128, writeCtr);
              writeCtr+=1;
            }
          

          }
          else if (version_knob_value ==4){
             if (isactive){
            keynum+=1;
            hasActiveKey=true;
            float phase=notePhases[(tune_knob_value-1)*4+i];
            float amp=getSample(phase,&notes.notes[i].sinAcc, squareTable);
            pianoAmp+=calcHornVout(amp,vol_knob_value,i);

             }
          
          if (i==11 && keynum>0){
              pianoAmp=pianoAmp/keynum;
              uint32_t Vout = static_cast<uint32_t>(pianoAmp*127)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }
          else if (version_knob_value ==3){
             if (isactive){
            keynum+=1;
            hasActiveKey=true;
            float phase=notePhases[(tune_knob_value-1)*4+i];
            float amp=getSample(phase,&notes.notes[i].sinAcc, saxophoneTable);
            pianoAmp+=calcPianoVout(amp,vol_knob_value,i);
             }
          
          if (i==11 && keynum>0){
              pianoAmp=pianoAmp/keynum;
              uint32_t Vout = static_cast<uint32_t>(pianoAmp*127)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }
          else if (version_knob_value ==2){
             if (isactive){
            keynum+=1;
            hasActiveKey=true;
            float phase=notePhases[(tune_knob_value-1)*4+i];
            pianoAmp+=getSample(phase,&notes.notes[i].sinAcc, saxophoneTable);
             }
          
          if (i==11 && keynum>0){
              pianoAmp=pianoAmp/keynum;
              pianoAmp+=generateLFO(1);
              uint32_t Vout = static_cast<uint32_t>(pianoAmp*127)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }

          else{
            if (isactive){
              keynum+=1;
              hasActiveKey=true;
              // float triAcc= notes.notes[i].triAcc;
              // float tmp= generateTriangleWaveValue(Frequencies[i], &notes.notes[i].triprevVal,&notes.notes[i].triLastIncre);
              // float tmp=generateTriangleWaveValue(Frequencies[i],&notes.notes[i].triAcc);
              triAmp+=getSample(notePhases[(tune_knob_value-1)*4+i],&notes.notes[i].triAcc,triangleTable);
              // notes.notes[i].triAcc=triAcc+tmp;
              // triAmp+=tmp;
            }
            if (i==11 && keynum>0){
              triAmp=triAmp/keynum;
              // sinAmp= lowPassFilter(sinAmp,prevSinAmp,500.0);
              // prevSinAmp=sinAmp;
              uint32_t Vout = static_cast<uint32_t>(triAmp*255)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }
        }

      }
      if(!hasActiveKey && writeCtr< SAMPLE_BUFFER_SIZE/2){
            writeToSampleBuffer(0,writeCtr);
            writeCtr+=1;
          }

      }
        }  

 
}


void sampleISR() {
  static uint32_t readCtr = 0;
  int metronome_knob_value=__atomic_load_n(&sysState.knobValues[0].current_knob_value,__ATOMIC_RELAXED);
  static uint32_t metronomeCounter=0;
  if (readCtr == SAMPLE_BUFFER_SIZE/2) {
    readCtr = 0;
    
    writeBuffer1 = !writeBuffer1;
    presssedTimeCount();
    // Serial.print("gave buffer");
    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
    }
  if (metronome_knob_value!=8 && metronomeCounter>=metronomeTime[7-metronome_knob_value]){ 
    analogWrite(OUTR_PIN, 255);
    metronomeCounter=0;
  }
  else{
  if (writeBuffer1){
    
    analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
    metronomeCounter+=1;
    // readCtr+=1;
    // sampleBuffer0[readCtr]=128;
    }
  else{
    
    analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
    metronomeCounter+=1;
    // readCtr+=1;
    // sampleBuffer1[readCtr]=128;
  }
  }

}



  // int vol_knob_value=__atomic_load_n(&sysState.knobValues[3].current_knob_value,__ATOMIC_RELAXED);
  // int tune_knob_value=__atomic_load_n(&sysState.knobValues[2].current_knob_value,__ATOMIC_RELAXED);
  // int version_knob_value=__atomic_load_n(&sysState.knobValues[1].current_knob_value,__ATOMIC_RELAXED);
  
  // if (version_knob_value ==8){
  // for (int i = 0; i < 12; i++) {
  
  //   float sinAcc=notes.notes[i].sinAcc;

  //   bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);

  //   if (isactive) {
  //     // phaseAcc+=sinPhases[i]*pow(2,32);
  //     sinAcc+=sinPhases[i];
  //     if (sinAcc>=M_PI){
  //       sinAcc-=M_PI;
  //     }
  //     int32_t Vout = static_cast<int32_t>(sin(sinAcc)*127)-128;

  //     Vout = Vout >> (8 - vol_knob_value);
  //     analogWrite(OUTR_PIN, Vout+128 );
  //     // Serial.print("clog here2");S
  //   }
  //   else{
  //     // phaseAcc = 0;
  //     sinAcc=0;

  //   }
  //   notes.notes[i].sinAcc=sinAcc;
    
  //     // __atomic_store_n(&notes[i].phaseAcc, phaseAcc, __ATOMIC_RELAXED);
  //   }
  // }
  // else{

  //sawtooth representation
  // int vol_knob_value=__atomic_load_n(&sysState.knobValues[3].current_knob_value,__ATOMIC_RELAXED);
  // int tune_knob_value=__atomic_load_n(&sysState.knobValues[2].current_knob_value,__ATOMIC_RELAXED);
  // static uint32_t phase=0;
  // uint32_t pAcc=0;
  // int numkey=0;
  // for (int i = 0; i < 12; i++) {
    
  //   uint32_t phaseAcc=__atomic_load_n(&notes.notes[i].phaseAcc,__ATOMIC_RELAXED);
  //   uint32_t stepSize=__atomic_load_n(&notes.notes[i].stepSize,__ATOMIC_RELAXED);
  //   bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);
    
  //   if (isactive) {
  //     if ((tune_knob_value-4)>=0){
  //       phaseAcc+=stepSize << (tune_knob_value-4);}
  //     else{
  //       phaseAcc+=stepSize >> -(tune_knob_value-4);
  //     }
  //     numkey+=1;
  //     pAcc+=stepSize;
  //     // int32_t Vout = (phaseAcc >> 24) - 128;
  //     // Vout = Vout >> (8 - vol_knob_value);

  //     // analogWrite(OUTR_PIN, Vout + 128);
  //   }
  //   else {
  //     phaseAcc = 0;
  //   }
  //   __atomic_store_n(&notes.notes[i].phaseAcc, phaseAcc, __ATOMIC_RELAXED);
    
  // }
  // if(numkey>0){
  // phase+=static_cast<u_int32_t> (pAcc/numkey);}
  // int32_t Vout = (phase >> 24) - 128;
  // Vout = Vout >> (8 - vol_knob_value);

  // analogWrite(OUTR_PIN, Vout + 128);
  

 
// }

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
  // Serial.print("clog here 4");
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void scanKeysTask(void * pvParameters) {

  
  // volatile uint32_t localCurrentStepSize;

  const TickType_t xFrequency1 = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();
  std::bitset<12> keys;
  std::bitset<8> current_knobs;
  std::bitset<12> previou_keys("111111111111");
  std::bitset<8> previous_knobs("00000000");
  // volatile uint8_t TX_Message[8] = {0}; //CAN TX message

  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    

    sysState.inputs = readInputs();
    
    keys = extractBits<20, 12>(sysState.inputs, 0, 12);
    current_knobs = extractBits<20, 8>(sysState.inputs, 12, 8);
    
    updateKnob(sysState.knobValues, previous_knobs, current_knobs);

    xSemaphoreGive(sysState.mutex);
    // Serial.print("clog here 5");
    for (int i = 0; i < 12; i++){
      
      
      // Decode keys
      if (keys[i] != previou_keys[i]){
        volatile uint8_t TX_Message[8] = {0}; 
        TX_Message[0] = keys[i] ? 'R' : 'P';
        TX_Message[1] = i;
        TX_Message[2] = 4;
        // CAN_TX(0x123, const_cast<uint8_t*>(TX_Message));
        // Serial.print((char) TX_Message[0]);
        // Serial.print(TX_Message[1]);
        // Serial.print(TX_Message[2]);
        // Serial.println(); 
        xQueueSend( msgOutQ, const_cast<uint8_t*>(TX_Message), portMAX_DELAY);
        // Serial.print("clog here 3");
      }
    }

    previou_keys = keys;
    previous_knobs = current_knobs;
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
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
    for (int i = 0; i < 20; i++){
      u8g2.setCursor(5*(i+1), 10);
      u8g2.print(sysState.inputs[i]);
    }


    //Display knobs
    for (int i = 0; i < 4; i++){
      u8g2.setCursor(10*(i+1), 20);
      u8g2.print(sysState.knobValues[i].current_knob_value);
    }
    xSemaphoreGive(sysState.mutex);
    // u8g2.setCursor(66,30);
    // u8g2.print((char) RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);

    u8g2.sendBuffer();
	  
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  // std::bitset<12> keys_1;
  // std::bitset<12> previou_keys_1("111111111111");
  // volatile uint32_t localCurrentStepSize;
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    Serial.print((char) RX_Message[0]);
    Serial.print(RX_Message[1]);
    Serial.print(RX_Message[2]);
    Serial.println();
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    xSemaphoreTake(notes.mutex, portMAX_DELAY);
    if (RX_Message[0] == 'P'){
      sysState.inputs[RX_Message[1]] = 0;
      notes.notes[RX_Message[1]].active = true;
    }
    else{
      sysState.inputs[RX_Message[1]] = 1;
      notes.notes[RX_Message[1]].active = false;
    }
    // keys_1 = extractBits<20, 12>(sysState.inputs, 0, 12);
    // Serial.print("clog here1");
    xSemaphoreGive(notes.mutex);
    xSemaphoreGive(sysState.mutex);
    // for (int i = 0; i < 12; i++){
    //   if (keys_1.to_ulong() != 0xFFF){
    //     if (keys_1[i] != previou_keys_1[i]){
    //       notes[i].active = !keys_1[i];
    //       Serial.println(notes[i].active);
    //     }
    //   }
    //   else{
    //     localCurrentStepSize = 0;
    //   }
    // }
    // previou_keys_1 = keys_1;
    // localCurrentStepSize = localCurrentStepSize * pow(2, sysState.knobValues[2].current_knob_value-4);
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);

	}
}

void setup() {
  // init_samplebuffer();
  sysState.knobValues[2].current_knob_value = 4;
  sysState.knobValues[3].current_knob_value = 4;
  generatePhaseLUT();
  // generateSinLUT();
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

  //Initialise sample timer
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise CAN TX semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

  // //Initialise CAN Bus
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  //Initialise serial port
  Serial.begin(9600);
  Serial.println("Hello World");
  set_notes();
  
  //Create tasks
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  64 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  TaskHandle_t CAN_TX_Handle = NULL;
  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &CAN_TX_Handle );	/* Pointer to store the task handle */
  TaskHandle_t BackCalc_Handle = NULL;
  xTaskCreate(
  backgroundCalcTask,		/* Function that implements the task */
  "BackCalc",		/* Text name for the task */
  64 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &BackCalc_Handle );	/* Pointer to store the task handle */
  
  sysState.mutex = xSemaphoreCreateMutex(); //Create mutex
  notes.mutex = xSemaphoreCreateMutex(); //Create mutex

  vTaskStartScheduler();

}

void loop() {
}