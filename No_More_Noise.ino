#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=463,392
AudioOutputI2S           i2s2;           //xy=689,395
AudioAnalyzeFFT1024      fft1024_1;      //xy=690,339.0000305175781
AudioConnection          patchCord1(i2s1, 0, fft1024_1, 0);
AudioConnection          patchCord2(i2s1, 1, i2s2, 0);
AudioControlSGTL5000     audioShield;     //xy=651.2333374023438,474.23333740234375
// GUItool: end automatically generated code

const float piano49Freq[49] = { //array of all 49 key's frequencies
    65.406, 69.296, 73.416, 77.782, 82.407, 87.307, 92.499, 97.999, 103.826, 110.000, 116.541, 123.471,
    130.813, 138.591, 146.832, 155.563, 164.814, 174.614, 184.997, 195.998, 207.652, 220.000, 233.082, 246.942,
    261.626, 277.183, 293.665, 311.127, 329.628, 349.228, 369.994, 391.995, 415.305, 440.000, 466.164, 493.883,
    523.251, 554.365, 587.330, 622.254, 659.255, 698.456, 739.989, 783.991, 830.609, 880.000, 932.328, 987.767,
    1046.502
};

const char* piano49Notes[49] = { //matching array of note names to match frequency array
    "C2","C#2","D2","D#2","E2","F2","F#2","G2","G#2","A2","A#2","B2",
    "C3","C#3","D3","D#3","E3","F3","F#3","G3","G#3","A3","A#3","B3",
    "C4","C#4","D4","D#4","E4","F4","F#4","G4","G#4","A4","A#4","B4",
    "C5","C#5","D5","D#5","E5","F5","F#5","G5","G#5","A5","A#5","B5",
    "C6"
};

// >>> NEW: sets a noise threshold for frames to be ignored (those that are too quiet; just noise) <<<
const float NOISE_THRESHOLD = 0.01f;  // tweak this value (0–1 range)
                                      //(0.01f is a float, using 0.01 would require the board to convert from double to float)

int findClosestNote(float freq) { //Function to find the nearest piano key
  int bestIndex = 0;
  float bestDiff = 9999999;

  for (int i = 0; i < 49; i++) {
    float diff = fabs(freq - piano49Freq[i]);
    if (diff < bestDiff) {
      bestDiff = diff;
      bestIndex = i;
    }
  }
  return bestIndex;
}

void setup() {
  Serial.begin(115200); //setting serial baud rate
  AudioMemory(40); //allocates Ram for the system

  audioShield.enable(); //enabling audio shield 
  audioShield.inputSelect(AUDIO_INPUT_LINEIN); //selecting the AUX input
 audioShield.volume(0.7); //setting volume to 70%

  fft1024_1.windowFunction(AudioWindowHanning1024); //windowing to improve FFT accuracy
}

void loop() {
  //Serial.print("test");
  //delay(1000);
  if (fft1024_1.available()) { //finding the strongest frequency component
    int maxIndex = 0;
    float maxVal = 0;

    for (int i = 0; i < 512; i++) {
      float v = fft1024_1.read(i);
      if (v > maxVal) {
        maxVal = v;
        maxIndex = i;
      }
    }

    // >>> NEW: ignore frames that are too quiet (just noise) <<<
    if (maxVal < NOISE_THRESHOLD) {
      // Optional debugging to tune NOISE_THRESHOLD:
      // Serial.print("Ignoring frame, maxVal = ");
      // Serial.println(maxVal);
      return;   // treat as silence
    }
    float binFreq = maxIndex * (44100.0 / 1024.0); //convert FFT bin to frequency

    if (binFreq < 40.0f) return; //filters out noise below 40Hz

    int index = findClosestNote(binFreq);

    Serial.print("Detected frequency above noise margin: ");
    Serial.print(binFreq);
    Serial.print(" Hz | Note: ");
    Serial.print(piano49Notes[index]);
    Serial.print(" (");
    Serial.print(piano49Freq[index]);
    Serial.println(" Hz)");
  }
}
