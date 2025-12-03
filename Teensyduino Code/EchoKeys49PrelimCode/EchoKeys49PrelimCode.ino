#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

AudioInputI2S            i2s1;           //xy=463,392
AudioOutputI2S           i2s2;           //xy=689,395
AudioAnalyzeNoteFrequency notefreq1;     // YIN-based pitch detector
AudioControlSGTL5000     audioShield;    //xy=651,474
AudioConnection          patchCord1(i2s1, 0, notefreq1, 0);

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

const int NOTE_WINDOW = 2;      // smaller window -> less lag
static int noteBuf[NOTE_WINDOW];
static int noteCount = 0;
static int notePos   = 0;
const float NOISE_THRESHOLD = 0.005f;  // tweak this value (0–1 range)
                                      //(0.01f is a float, using 0.01 would require the board to convert from double to float)
void resetNoteFilter() {
  noteCount = 0;
  notePos   = 0;
}

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

//>>NEW
// Simple median filter on the note index to avoid octave flicker
int smoothNoteIndex(int rawIndex) {
  // Add new index into circular buffer
  noteBuf[notePos] = rawIndex;
  notePos = (notePos + 1) % NOTE_WINDOW;
  if (noteCount < NOTE_WINDOW) noteCount++;

  // Copy to temp array and sort to get median
  int tmp[NOTE_WINDOW];
  for (int i = 0; i < noteCount; i++) {
    tmp[i] = noteBuf[i];
  }

  for (int i = 0; i < noteCount - 1; i++) {
    for (int j = i + 1; j < noteCount; j++) {
      if (tmp[j] < tmp[i]) {
        int t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }

  // Median value (robust to one bad outlier)
  return tmp[noteCount / 2];
}

// ----- Octave-locker to avoid C#5 <-> C#4 flicker -----
int applyOctaveLock(int candidateIndex) {
  static int lastStableIndex = -1;

  // First ever note: just accept it
  if (lastStableIndex < 0) {
    lastStableIndex = candidateIndex;
    return candidateIndex;
  }

  int diff = candidateIndex - lastStableIndex;
  int pcCandidate = candidateIndex % 12;   // pitch class (0..11)
  int pcLast      = lastStableIndex % 12;

  // If it's the same note name (same pitch class)
  // but ≥ 1 octave away, treat it as a subharmonic / octave glitch
  if (pcCandidate == pcLast && abs(diff) >= 12) {
    // Ignore this jump, keep the old octave
    return lastStableIndex;
  }

  // Otherwise, accept the change as the new stable note
  lastStableIndex = candidateIndex;
  return candidateIndex;
}

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  while (!Serial && millis() < 4000) ; // wait for Serial

  Serial.println("Setup starting...");

  AudioMemory(40);

  audioShield.enable();
  audioShield.inputSelect(AUDIO_INPUT_LINEIN);
  audioShield.volume(0.7);
  audioShield.lineInLevel(13);  // boost line-in a bit (0–15)

  // Initialize YIN algorithm. 0.15 is a good default threshold.
  notefreq1.begin(0.20f);  // lower = more sensitive, higher = more conservative

  Serial.println("Setup finished.");
}

void loop() {
  // Blink so we know loop() is alive
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(13, !digitalRead(13));
    lastBlink = millis();
  }

  if (!notefreq1.available()) {
    return;
  }

  float freq = notefreq1.read();        // Hz
  float prob = notefreq1.probability(); // 0–1

  // Treat low-probability as "no note"
  const float MIN_PROB = 0.90f;
  if (prob < MIN_PROB ||
      freq < piano49Freq[0] ||
      freq > piano49Freq[48]) {

    // If we've gone a bit of time with no valid note, clear filter history
    static uint32_t lastGoodMs = 0;
    if (millis() - lastGoodMs > 150) {  // ~150 ms of "no note"
      resetNoteFilter();
    }
    return;
  }

  static uint32_t lastGoodMs = 0;
  lastGoodMs = millis();

  int rawIndex = findClosestNote(freq);       // based on current frame only
  int index    = smoothNoteIndex(rawIndex);   // smoothed across frames
  index        = applyOctaveLock(index);

  Serial.print("freq = ");
  Serial.print(freq, 2);
  Serial.print(" Hz | raw=");
  Serial.print(piano49Notes[rawIndex]);
  Serial.print(" | smoothed=");
  Serial.print(piano49Notes[index]);
  Serial.print(" (");
  Serial.print(piano49Freq[index], 2);
  Serial.print(" Hz) | prob=");
  Serial.println(prob, 2);

  // Use 'index' for LEDs / key mapping
}

