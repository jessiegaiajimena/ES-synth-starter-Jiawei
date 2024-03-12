#include <math.h>
#include <stdlib.h>

#define SAMPLE_RATE 22000
#define AMPLITUDE 0.5

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

// Function to apply a low-pass filter
// float low_pass_filter(float input, float cutoff, float resonance, float* state) {
//     float output = input * cutoff + *state * (1.0 - cutoff);
//     *state = output;
//     return output;
// }
// Function to apply a low-pass filter with resonance
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

// float generateTriangleWaveValue(float frequency, float* prevValue, float* prevIncrement) {
//     float period = SAMPLE_RATE / frequency;
//     float increment = 4.0f * AMPLITUDE / period;

//     float value = *prevValue;
//     if (value >= AMPLITUDE) {
//         increment = -increment;
//     } else if (value <= -AMPLITUDE) {
//         increment = -increment;
//     }
//     value += *prevIncrement;
//     *prevValue = value;
//     *prevIncrement = increment;

//     return value;
// }
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
