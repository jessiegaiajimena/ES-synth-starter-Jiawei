#include <math.h>
#include <stdlib.h>

#define SAMPLE_RATE 22000
#define AMPLITUDE 0.5
#define TABLE_SIZE 256

// ADSR envelope parameters
float attack_time = 0.1;
float decay_time = 0.2;
float sustain_level = 0.6;
float release_time = 0.4;

// Filter parameters
float filter_cutoff = 0.5;
float filter_resonance = 0.2;
float filter_env_amount = 2;

// LFO parameters for pitch modulation
float lfo_frequency = 5.0;
float lfo_depth = 0.01;

// Noise parameters
float noise_level = 0.1;
float noise_filter_cutoff = 0.8;

float lfoFreq=10.0;
float lfoPhase=lfoFreq*M_PI*2/SAMPLE_RATE;

















// Function to generate a sawtooth wave
float sawtooth(float phase) {
    return 2.0 * (phase - floor(phase + 0.5));
}

// Function to apply ADSR envelope
float adsr_envelope(float time, float duration) {
    float envelope = 0.0;
    float attack_end = attack_time;
    float decay_end = attack_end + decay_time;
    float sustain_end = duration - release_time;

    if (time < attack_end) {
        envelope = time / attack_end;
    } else if (time < decay_end) {
        envelope = 1.0 - (time - attack_end) / decay_time * (1.0 - sustain_level);
    } else if (time < sustain_end) {
        envelope = sustain_level;
    } else {
        envelope = sustain_level * (1.0 - (time - sustain_end) / release_time);
    }

    return envelope;
}


float lowPassFilter(float cur, float prev, float cutoffFreq) {
    float rc = 1.0f / (2.0f * M_PI * cutoffFreq);
    float dt = 1.0f / SAMPLE_RATE;
    float alpha = dt / (rc + dt);

    // float prev = buffer[0];
    // for (int i = 1; i < bufferSize; i++) {
        // float curr = buffer[i];
    return  prev + alpha * (cur - prev);
        // prev = buffer[i];
    // }
}



float low_pass_filter(float input, float cutoff, float resonance, float* state1, float* state2) {
    float output = input * cutoff + *state1 * (1.0 - cutoff);
    *state1 = output * cutoff + *state2 * (1.0 - cutoff);
    *state2 = output;
    output += (output - *state2) * resonance;
    return output;
}

// Function to generate a saxophone-like sound
float saxophone_sound( float frequency, float* saxAcc) {
    float phase = 0.0;
    float sample = 0.0;
    float lfo = 0.0;
    float filter_state1 = 0.0;
    float filter_state2 =0.0;
    float noise = 0.0;
    float time=0.1;
    float duration=1;


    // Calculate the phase increment based on the desired frequency
    float phase_increment = frequency / SAMPLE_RATE;

    // Calculate the LFO value for pitch modulation
    // lfo = sin(2.0 * M_PI * lfo_frequency * time) * lfo_depth;

    // Generate the sawtooth wave with pitch modulation
    // phase += phase_increment + lfo;
    // phase += phase_increment;

    // lfo = sin(2.0 * M_PI * lfo_frequency * time) * lfo_depth;

    // Generate the sawtooth wave with pitch modulation
    *saxAcc += phase_increment;
    if (*saxAcc >= 1.0) {
        *saxAcc -= 1.0;
    }
    sample = sawtooth(*saxAcc);
    // sample = sawtooth(phase_increment);

    // Apply the ADSR envelope
    float envelope = adsr_envelope(time, duration);
    sample *= envelope;

    // Apply the low-pass filter with envelope modulation
    float filter_cutoff_mod = filter_cutoff + envelope * filter_env_amount;
    sample = low_pass_filter(sample, filter_cutoff_mod, filter_resonance, &filter_state1,&filter_state2);

    // // Generate noise for breath and air flow
    // noise = (float)rand() / (float)RAND_MAX * 2.0 - 1.0;
    // noise = low_pass_filter(noise, noise_filter_cutoff, 0.5, &filter_state);
    // sample += noise * noise_level;

    return sample;
}

float generateTriangleWave( float frequency ){
    float amplitude=0.5;
    float period = SAMPLE_RATE / frequency;
    float increment = 4.0f * amplitude / period;
    float value = -amplitude;

    if (value >= amplitude) {
        increment = -increment;
    } else if (value <= -amplitude) {
        increment = -increment;
    }

    value += increment;
    return value;
}

float generateTriangleWaveValue(float frequency, float* phase) {
    *phase += frequency / SAMPLE_RATE;
    if (*phase > 1.0f) {
        *phase -= 1.0f;
    }

    float value = 2.0f * AMPLITUDE * (fabs(2.0f * *phase - 1.0f) - 0.5f);
    return value;
}
float bhaskaraSin(float x) {
    float numerator = 16 * x * (PI - x);
    float denominator = 5 * PI * PI - 4 * x * (PI - x);
    return numerator / denominator;// faster sin but actually slower
}
float generateSin( float sinPhase, float* phase){
    float testsinAcc=*phase;
                
    testsinAcc+=sinPhase;
    if (testsinAcc>=M_PI){
        testsinAcc-=M_PI;
    }
    *phase=testsinAcc;
    return sin(testsinAcc);
}


float getSample(float phaseIcre, float* phaseAcc, float table[]) {
    *phaseAcc+=phaseIcre;
    if (int(*phaseAcc*TABLE_SIZE)>TABLE_SIZE){
        *phaseAcc-=1;
    }
    int index = (int)(*phaseAcc * TABLE_SIZE);
    index = index % TABLE_SIZE;
    return table[index];
}
float LFOAcc=0;
float generateLFO(int reduceVal){
    float amp=getSample(lfoPhase,&LFOAcc,sineTable)/reduceVal;
    return amp;
}
// float sinTable[256];

// void generateSinLUT(){
//     float step=(1/256)*2*M_PI;
//     for (int i=0; i<TABLE_SIZE;i++){
//         sinTable[i]=sin(step*i);
//     }
// }
int calcEnvelope(int pressedCount){
    int press=3;
    int decay=4;
    if (pressedCount<press){
        return 0;
    }
    else{
        return int( (pressedCount-press)/3);
    }

}
u_int32_t calcSawtoothVout(u_int32_t phaseAcc,int volume, int i){
    uint32_t Vout = (phaseAcc >> 24) - 128;
    int v=calcEnvelope(notes.notes[i].pressedCount);
    int volshift=8 - volume+v;

    Vout = ((Vout+128) >> (volshift)) ;
    return Vout;
}
u_int32_t calcNoProcessSawtoothVout(u_int32_t phaseAcc,int volume, int tune){
    
    uint32_t Vout = (phaseAcc >> 24) - 128;

    // int v=calcEnvelope(notes.notes[i].pressedCount);
    int volshift=8 - volume;

    Vout = ((Vout+128) >> (volshift)) ;
    return Vout;
}



u_int32_t calcOtherVout(float Amp,int volume, int i){
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    int v=calcEnvelope(notes.notes[i].pressedCount);
    int volshift=8 - volume+v;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}

u_int32_t calcNoEnvelopeVout(float Amp,int volume){
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    // int v=calcEnvelope(notes.notes[i].pressedCount);
    int volshift=8 - volume;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}



int adsrPiano(int pressedCount){
    int attack=1;
    int decay=4;
    int sustain=10;
    if (pressedCount<attack && pressedCount>0){
        return 2-pressedCount;
    }
    else if (pressedCount>= attack && decay>=pressedCount){
        return int( (pressedCount-attack));
    }
    else if(pressedCount>decay && pressedCount<sustain){
        return int( (decay-attack));
    }

    else{
        return (decay-attack)+floor((pressedCount-decay));
    }


}

int adsrHorn(int pressedCount){
    int lowtime=3; 
    // int hightime;
    int lowval=2;
    int highval=-2;
    int loopduration=20;
    static int countlow=0;
    static int counthigh=0;
    if ((pressedCount%loopduration)<=lowtime){
        counthigh=0;
        if (countlow<lowval){
            
            countlow+=1;
        }
        return countlow;
    }
    else{
        countlow=0;
        if (counthigh>highval){
            counthigh-=1;
        }
        return counthigh;
    }
    
}
u_int32_t calcPianoVout(float Amp,int volume, int i){
    Amp+=generateLFO(2);
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    int v=adsrPiano(notes.notes[i].pressedCount);
    // int v=adsrHorn(notes.notes[i].pressedCount);
    int volshift=8 - volume+v;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}
u_int32_t calcHornVout(float Amp,int volume, int i){
    Amp+=generateLFO(2);
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    int v=adsrHorn(notes.notes[i].pressedCount);
    // int v=adsrHorn(notes.notes[i].pressedCount);
    int volshift=8 - volume+v;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}


void presssedTimeCount(){
    for (int i=0;i<12;i++){
        bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);
        if (isactive){
            notes.notes[i].pressedCount+=1;
        }
        else{
            notes.notes[i].pressedCount=0;
        }
    }
}
