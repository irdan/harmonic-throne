#include <arduinoFFT.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <FastLED.h>

/*
 * Begin variables possible to change between each resonator tube
 */
// Set the tone station number here. Should be between 2 and 12 inclusive
const int toneNumber = 2; // used to calculate target FFT bin

#define DATA_PIN 2 // LED data pin
const int micPin = A6;  // Microphone connected to analog pin A6
/*
 * End variables to change between each resonator tube
 */


/*  
 * Begin variables to change light behavior
 *  keep the same for all resonator tubes 
 */
const int seconds_in_30_min = 1800;
const int hueTime = seconds_in_30_min * (1000 / 256); // 1000 microseconds, 256 hues. Parenthesis to not overflow
const double alpha = 0.7; // between 0 and 1. Using 0.1 is slow, to react, 0.8 very fast, but less smooth
int maxMagnitude = 0; // Dynamically tracks the maxMagnitude. Tracks the max for that microphone
const int minMagnitude = 200; // The mininum magnitude we want to take action from
/*
 * End variables to change light behavior
 *  keep the same for all resonator tubes 
 */

// Global tone signal variable for use in exponential moving average calculation
double toneSignal;

// Setup LEDS
// Data PIN defined at top in case a pin had to be relocated during build
#define NUM_LEDS 14
CRGB leds[NUM_LEDS];


/* Define FFT calculation related variables */
constexpr uint8_t sampleSize = 64; // Number of samples for FFT
constexpr uint8_t overlapSize = sampleSize / 4; // 25% overlap
constexpr double samplingFrequency = 1000; // Sampling frequency in Hz
constexpr double binSize = samplingFrequency / sampleSize;  // frequency width of FFT bins
// index where tone signal would reside. Multiple by 30 since there is a 30hz step between tones
constexpr uint8_t targetBinIndex = (toneNumber * 30) / binSize;
double buffer[sampleSize * 2]; // Buffer to handle overlapping samples
double vReal[sampleSize]; // Array to store real values for FFT
double vImag[sampleSize]; // Array to store imaginary values for FFT
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, sampleSize, samplingFrequency);
volatile uint8_t writeIndex = 0; // Track buffer position for writing samples
volatile uint8_t sampleCounter = 0; // Track number of samples taken
volatile bool sampleCompleteFlag = false; // Flag to indicate it is time to process samples
volatile unsigned long lastSampleMicros = 0; // track time of last sample

// Track hue. Increment gradually with an interrupt
volatile uint8_t hue = 0;

/* Compile time validation of constants */
static_assert( toneNumber > 1 && toneNumber < 13, "tone must be between 2 and 12 inclusive");

// fft library produces an infinite loop if sampleSize isn't a power of 2
static_assert( (sampleSize & (sampleSize - 1)) == 0, "sampleSize must be a power of 2");

// fft can't detect frequencies less than half the sample frequency
static_assert(targetBinIndex < (sampleSize / 2) - 1, "we cannot detect a frequency greater than the Nyquist Frequency");


void setup() {
  Serial.begin(115200);

  delay(1000 * toneNumber); // Stagger startup so that not all 11 microcontrollers start at once
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  fill_solid(leds, NUM_LEDS, CHSV(hue, 255, 255));
  FastLED.show();

  toneSignal = analogRead(micPin); // init for later user in EMA
  
  Timer1.initialize(1000000 / samplingFrequency); // Set timer to trigger at the desired sampling frequency
  Timer1.attachInterrupt(timerIsr);

  MsTimer2::set(hueTime, hueIsr); // This increments the color about every 7 seconds, or 0-255 in 30 minutes
  MsTimer2::start();
  
}

void loop() {
  if (sampleCompleteFlag) {
//    unsigned long startTime = micros(); // Used to meausre the length of processing the main loop

    // set readIndex to the write index minus sampleSize, but make sure it is positive before modulo
    // I know this looks silly, but it works. Other stuff might work too.
    // Also, this should make use of the buffer size for modulo, not the sample size
    // If we ever change the size of the buffer to not be sampleSize * 2 this will be a problem
    int readIndex = (writeIndex - sampleSize + (sampleSize * 2)) % ( sampleSize * 2);

    // Assign vReal values from buffer
    for (uint8_t i = 0; i < sampleSize; i++){
      vReal[i] = buffer[(readIndex + i) % (sampleSize * 2)];
    }
    // Set all elements of vImag back to 0
    memset(vImag, 0, sampleSize * sizeof(double));

    // Perform FFT and process results
    performFFT();
    processFFTResults();

    // Print out results. Uncomment either (but not both) as appropriate
    //printFFTResults();
    //histogramFFTResults();

/*    // Print out the duration of the main loop
    unsigned long endTime = micros();
    unsigned long duration = endTime - startTime;
    Serial.print("duration: ");
    Serial.print(duration);
    Serial.println(" microseconds");
*/
    // Disable flag that runs main loop
    sampleCompleteFlag = false;
  }
}

void hueIsr(){
  /*
   * Increment the hue. Don't worry about overflow since we use uint8_t
   */
   hue++;
}

void timerIsr() {
  /*
   * Take a sample. Adjust index, timestamp, counter, and flag.
   */
  buffer[writeIndex] = analogRead(micPin); // add a sample to the buffer
  writeIndex = (writeIndex + 1) % (sampleSize * 2); // update writeIndex
  
  if (++sampleCounter > sampleSize){ // increment sample counter and check if completed sample
    /*
     * Measured thousands of FFT sample processes, the longest observed was 35ms.
     * Since we are completing a sample every 48ms, this check isn't needed.
     * Leaving the commented out code in case we want to validate this hasn't changed.
     *
     * The above assumes we have:
     *  sample rate of 1000hz 
     *  sample count of 64 
     *  and overlap of 25%
    if (sampleComplete){
      Serial.println("!!!!!Sample finished before previous processing completed!!!!");
    }
    */
    sampleCompleteFlag = true;
    sampleCounter = overlapSize; // restart counter, accounting for overlapping samples
  }
  lastSampleMicros = micros(); // Update the microsecond time of the latest sample
}


void performFFT(){
  /*
   * Perform the actual FFT using ArduinoFFT library
   */
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
}

int getScalePosition(int value){
  /* Return the mapping of the value between min and max to 0 to 255.
   * Defined as a function to ensure consistent calcuation of 
   * color palette index and brightness values
   */
  return map(value, minMagnitude, maxMagnitude, 0, 255);
}


uint8_t getBrightness(){
  /* 
   *  Calculate brightness.
   */
  toneSignal = alpha * vReal[targetBinIndex] + (1 - alpha) * toneSignal; // EMA

  // Update maxMagnitude if needed
  if (toneSignal > maxMagnitude){
    maxMagnitude = toneSignal;
  }
  
  if (toneSignal < minMagnitude) {
    return 0;
  }

  return getScalePosition(toneSignal);
}


void processFFTResults(){
  uint8_t brightness = getBrightness();

  while (micros() - lastSampleMicros < 200){
    // All interrupts are disabled by FastLED when updating LED properties
    // wait until we *just* took a sample before adjusting LED brightness
    // The longest observed time to adjust 14 LEDS was 640 microseconds
    // this empty loop delay ensures fft samples are evenly spaced
  }

  fill_solid(leds, NUM_LEDS, CHSV(hue, 255, brightness));
  FastLED.show();
}

void printFFTResults(){
  for (uint8_t i = 0; i < sampleSize / 2; i++){
    double frequency = (i * samplingFrequency) / sampleSize;
    double magnitude = vReal[i];
    Serial.print(frequency, 1);
    Serial.print(" Hz: ");
    Serial.println(magnitude, 1);
  }
}

void histogramFFTResults(){
  /* 
   *  Produce serial output for parsing by the companion python script
   */
  for (uint8_t i = 0; i < sampleSize / 2; i++) {
    Serial.print("(");
    Serial.print((i * samplingFrequency) / sampleSize, 1);
    Serial.print(",");
    Serial.print(vReal[i], 1);
    Serial.print(")");
    if (i < (sampleSize / 2)- 1) {
      Serial.print(",");
    }
  }
  Serial.println();
}
