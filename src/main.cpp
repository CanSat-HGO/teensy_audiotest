/*
 * ICS-43434 Frequenzmesser - Teensy 4.1-Variante
 * 
 * Verkabelung am Teensy 4.1 (an dieser Hardware verifiziert):
 *   3V3  -> 3.3V (Achtung: Max 3.6V!)
 *   GND  -> GND
 *   BCLK -> Pin 21
 *   WS   -> Pin 20 (bzw. LRCL)
 *   DOUT -> Pin 8
 *   SEL  -> GND (Mikrofonsignal liegt im linken Kanal)
 */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// ---------- FFT / Frequenzmess-Parameter ----------
constexpr uint32_t SAMPLE_RATE = 44100; // Teensy Audio Library Standard
constexpr size_t FFT_SIZE = 2048;       // Muss ein Vielfaches von 128 sein (2048 / 128 = 16 Blöcke)
constexpr float MIN_FREQUENCY_HZ = 20.0f;
constexpr float MAX_FREQUENCY_HZ = 8000.0f;
constexpr float RMS_THRESHOLD = 250.0f;

// Teensy Audio Objekte für das I2S-Mikrofon
AudioInputI2S       i2s1;
AudioRecordQueue    queue1;
AudioConnection     patchCord1(i2s1, 0, queue1, 0); // Kanal 0 (linker Kanal)

int32_t rawAudio[FFT_SIZE];
float realPart[FFT_SIZE];
float imaginaryPart[FFT_SIZE];
float windowValues[FFT_SIZE];

void setupI2SMic() {
  // Speicher für die Audio-Blöcke der Teensy Audio Library reservieren
  AudioMemory(60);
  // Queue starten, um die Audiodaten des Mikrofons abzufangen
  queue1.begin();
}

// Ersetzt das ESP32 i2s_read(): Holt die Daten aus den Teensy-Audio-Blöcken
void readAudioBlock() {
  size_t samplesRead = 0;
  while (samplesRead < FFT_SIZE) {
    if (queue1.available() >= 1) {
      int16_t* buffer = queue1.readBuffer();
      for (size_t i = 0; i < 128 && samplesRead < FFT_SIZE; i++) {
        rawAudio[samplesRead++] = static_cast<int32_t>(buffer[i]);
      }
      queue1.freeBuffer(); // Speicher nach dem Lesen wieder freigeben
    }
  }
}

float calculateRms() {
  double sum = 0.0;
  for (size_t index = 0; index < FFT_SIZE; ++index) {
    const float sample = static_cast<float>(rawAudio[index]);
    sum += static_cast<double>(sample) * sample;
  }
  return sqrtf(static_cast<float>(sum / FFT_SIZE));
}

void prepareSamples() {
  double mean = 0.0;
  for (size_t index = 0; index < FFT_SIZE; ++index) {
    mean += static_cast<double>(rawAudio[index]);
  }
  mean /= FFT_SIZE;

  for (size_t index = 0; index < FFT_SIZE; ++index) {
    const float sample = static_cast<float>(rawAudio[index]) - mean;
    realPart[index] = sample * windowValues[index];
    imaginaryPart[index] = 0.0f;
  }
}

// Deine bewährte FFT-Funktion bleibt komplett unverändert!
void calculateFft() {
  // Bit-reversal permutation.
  for (size_t index = 1, reversed = 0; index < FFT_SIZE; ++index) {
    size_t bit = FFT_SIZE >> 1;
    for (; reversed & bit; bit >>= 1) {
      reversed ^= bit;
    }
    reversed ^= bit;

    if (index < reversed) {
      float temp = realPart[index];
      realPart[index] = realPart[reversed];
      realPart[reversed] = temp;

      temp = imaginaryPart[index];
      imaginaryPart[index] = imaginaryPart[reversed];
      imaginaryPart[reversed] = temp;
    }
  }

  for (size_t length = 2; length <= FFT_SIZE; length <<= 1) {
    const float angle = -2.0f * PI / static_cast<float>(length);
    const float sine = sinf(angle);
    const float cosine = cosf(angle);

    for (size_t start = 0; start < FFT_SIZE; start += length) {
      float currentCosine = 1.0f;
      float currentSine = 0.0f;
      const size_t halfLength = length >> 1;

      for (size_t offset = 0; offset < halfLength; ++offset) {
        const size_t even = start + offset;
        const size_t odd = even + halfLength;
        const float multipliedReal = currentCosine * realPart[odd] - currentSine * imaginaryPart[odd];
        const float multipliedImaginary = currentCosine * imaginaryPart[odd] + currentSine * realPart[odd];

        realPart[odd] = realPart[even] - multipliedReal;
        imaginaryPart[odd] = imaginaryPart[even] - multipliedImaginary;
        realPart[even] += multipliedReal;
        imaginaryPart[even] += multipliedImaginary;

        const float nextCosine = currentCosine * cosine - currentSine * sine;
        currentSine = currentCosine * sine + currentSine * cosine;
        currentCosine = nextCosine;
      }
    }
  }
}

// Timecode als HH:MM:SS.mmm seit dem Start des Teensy
void printTimecode() {
  const uint32_t ms = millis();
  const uint32_t hours = ms / 3600000UL;
  const uint32_t minutes = (ms / 60000UL) % 60;
  const uint32_t seconds = (ms / 1000UL) % 60;
  const uint32_t millisPart = ms % 1000UL;
  Serial.printf("[%02lu:%02lu:%02lu.%03lu] ",
                (unsigned long)hours, (unsigned long)minutes,
                (unsigned long)seconds, (unsigned long)millisPart);
}

void printDominantFrequency(float rms) {
  size_t firstBin = static_cast<size_t>(ceilf(MIN_FREQUENCY_HZ * FFT_SIZE / SAMPLE_RATE));
  size_t lastBin = static_cast<size_t>(floorf(MAX_FREQUENCY_HZ * FFT_SIZE / SAMPLE_RATE));
  size_t peakBin = firstBin;
  float peakMagnitude = 0.0f;

  for (size_t bin = firstBin; bin <= lastBin; ++bin) {
    const float magnitude = sqrtf(
        realPart[bin] * realPart[bin] + imaginaryPart[bin] * imaginaryPart[bin]);
    if (magnitude > peakMagnitude) {
      peakMagnitude = magnitude;
      peakBin = bin;
    }
  }

  const float frequency = static_cast<float>(peakBin) * SAMPLE_RATE / FFT_SIZE;

  printTimecode();
  if (rms < RMS_THRESHOLD) {
    Serial.printf("Frequenz:      --- Hz | RMS: %10.1f | Peak: %12.0f\n", rms, peakMagnitude);
  } else {
    Serial.printf("Frequenz: %8.1f Hz | RMS: %10.1f | Peak: %12.0f\n", frequency, rms, peakMagnitude);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Hanning-Fenster initialisieren
  for (size_t index = 0; index < FFT_SIZE; ++index) {
    windowValues[index] = 0.5f * (1.0f - cosf(2.0f * PI * index / (FFT_SIZE - 1)));
  }

  setupI2SMic();

  Serial.println();
  Serial.println("======================================");
  Serial.println(" ICS-43434 LIVE FFT - Teensy 4.1");
  Serial.println("======================================");
  Serial.printf("Sample Rate : %lu Hz\n", SAMPLE_RATE);
  Serial.printf("FFT Size    : %u\n", FFT_SIZE);
  Serial.printf("Auflösung   : %.5f Hz\n", static_cast<float>(SAMPLE_RATE) / FFT_SIZE);
  Serial.println("Pins        : BCLK 21, WS 20, DATA 8");
  Serial.println("Baudrate    : 115200");
  Serial.println();
}

void loop() {
  readAudioBlock();

  const float rms = calculateRms();
  prepareSamples();
  calculateFft();
  printDominantFrequency(rms);
}