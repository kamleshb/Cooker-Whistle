#include <Arduino.h>
#include "driver/i2s.h"

// 1. UPDATE THIS HEADER NAME to match the .h file inside your lib/ folder:
#include <Cooker-whistle_inferencing.h>

// =====================================================
// PIN & I2S CONFIGURATION
// =====================================================
#define I2S_SCK 18
#define I2S_WS 19
#define I2S_SD 23
#define I2S_PORT I2S_NUM_0
#define LED 2
#define ALERT_LED_PIN 2 // Built-in LED toggles on whistle detection

//================== COUNTER SECTION ============================
// ================= COUNTDOWN + DISPLAY + BUZZER =================
#define BTN_UP 4     
#define BTN_DOWN 15  
#define BTN_RESET 32 
#define BUZZER_PIN 21

// 7-segment pins: a,b,c,d,e,f,g  (COMMON CATHODE: segment ON = HIGH)
#define SEG_A 26
#define SEG_B 27
#define SEG_C 14
#define SEG_D 12
#define SEG_E 13
#define SEG_F 25
#define SEG_G 33

const uint8_t segPins[7] = {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G};

// segment patterns for digits 0-9 (1 = ON), order a,b,c,d,e,f,g
const uint8_t digitPatterns[10][7] = {
    {1, 1, 1, 1, 1, 1, 0}, // 0
    {0, 1, 1, 0, 0, 0, 0}, // 1
    {1, 1, 0, 1, 1, 0, 1}, // 2
    {1, 1, 1, 1, 0, 0, 1}, // 3
    {0, 1, 1, 0, 0, 1, 1}, // 4
    {1, 0, 1, 1, 0, 1, 1}, // 5
    {1, 0, 1, 1, 1, 1, 1}, // 6
    {1, 1, 1, 0, 0, 0, 0}, // 7
    {1, 1, 1, 1, 1, 1, 1}, // 8
    {1, 1, 1, 1, 0, 1, 1}  // 9
};

int setCount = 0;       // target count the user dials in (0-9)
int remainingCount = 0; // counts down as whistles are detected
bool buzzerActive = false;

bool buzzerState = false;
unsigned long lastBeepToggle = 0;
const unsigned long BEEP_INTERVAL_MS = 1000; // on/off every 1s


unsigned long lastBtnTime = 0;
const unsigned long BTN_DEBOUNCE_MS = 300;
volatile bool upFlag = false;
volatile bool downFlag = false;
volatile bool resetFlag = false;

void IRAM_ATTR handleUpISR()    { upFlag = true; }
void IRAM_ATTR handleDownISR()  { downFlag = true; }
void IRAM_ATTR handleResetISR() { resetFlag = true; }
//==============================================

String firstDetection = "";
int whistleCount = 0;

// NEW: track how long the whistle has been continuously detected
unsigned long whistleStartTime = 0;
bool whistleActive = false;
const unsigned long WHISTLE_MIN_DURATION_MS = 400; // must sustain > 1 sec

// Edge Impulse Buffer Settings
static int16_t sampleBuffer[EI_CLASSIFIER_RAW_SAMPLE_COUNT];
static bool debug_nn = false;
//================ SECTION FOR COUNTER ==============================

void showDigit(int val)
{
  if (val < 0)
    val = 0;
  if (val > 9)
    val = 9;
  for (int i = 0; i < 7; i++)
    digitalWrite(segPins[i], digitPatterns[val][i] ? HIGH : LOW);
}

void setupCounter()
{
  for (int i = 0; i < 7; i++)pinMode(segPins[i], OUTPUT);
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLDOWN);
  pinMode(BTN_DOWN, INPUT_PULLDOWN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  showDigit(setCount);
  attachInterrupt(digitalPinToInterrupt(BTN_UP), handleUpISR, RISING);
  attachInterrupt(digitalPinToInterrupt(BTN_DOWN), handleDownISR, RISING);
  attachInterrupt(digitalPinToInterrupt(BTN_RESET), handleResetISR, FALLING);
}

void resetCounter()
{
  delay(50); 
  ESP.restart();
}

void handleCounterButtons()
{
  unsigned long now = millis();

  if (resetFlag)
  {
    resetFlag = false;
    if (now - lastBtnTime >= BTN_DEBOUNCE_MS)
    {
      resetCounter();
      lastBtnTime = now;
    }
    return;
  }

  if (buzzerActive)
  {
    upFlag = false;
    downFlag = false;
    return;
  }

  if (upFlag)
  {
    upFlag = false;
    if (now - lastBtnTime >= BTN_DEBOUNCE_MS)
    {
      setCount = min(setCount + 1, 9);
      remainingCount = setCount;
      showDigit(setCount);
      lastBtnTime = now;
    }
  }
  else if (downFlag)
  {
    downFlag = false;
    if (now - lastBtnTime >= BTN_DEBOUNCE_MS)
    {
      setCount = max(setCount - 1, 0);
      remainingCount = setCount;
      showDigit(setCount);
      lastBtnTime = now;
    }
  }
}
void onWhistleConfirmed()
{
  if (setCount == 0 || remainingCount == 0)
    return; // not armed / already done

  remainingCount--;
  showDigit(remainingCount);
  

  if (remainingCount == 0) // <-- change to `== 1` here if you meant buzz one early
  {
    buzzerActive = true;
    buzzerState = true;
    lastBeepToggle = millis();
    digitalWrite(BUZZER_PIN, HIGH);
   
  }
}

void updateBuzzer()
{
  if (!buzzerActive) return;

  unsigned long now = millis();
  if (now - lastBeepToggle >= BEEP_INTERVAL_MS)
  {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    lastBeepToggle = now;
  }
}


//===============================================================
// =====================================================
// I2S SETUP
// =====================================================
void setupI2S()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = EI_CLASSIFIER_FREQUENCY, // 16000 Hz from Edge Impulse
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false};

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD};

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

// Callback function required by Edge Impulse signal processing
static int raw_audio_get_data(size_t offset, size_t length, float *out_ptr)
{
  numpy::int16_to_float(sampleBuffer + offset, out_ptr, length);
  return 0;
}

/// -- setting it on start of the last whistle

void onWhistleStarted()
{
  if (setCount == 0 || buzzerActive)
    return;

  // if this is the LAST whistle we're waiting for, start buzzing right at onset
  if (remainingCount == 1)
  {
    buzzerActive = true;
    buzzerState = true;
    lastBeepToggle = millis();
    digitalWrite(BUZZER_PIN, HIGH);
  
  }
}

// counting the whistle

void whistleCounter(String type)
{
  // First detection must be whistle
  if (firstDetection == "")
  {
    if (type == "whistle")
    {
      firstDetection = "whistle";
      onWhistleStarted(); 
    
    }
    else
    {
      Serial.println("Ignoring NOISE - waiting for WHISTLE");
    }
  }

  // Second detection must be noise
  else if (firstDetection == "whistle")
  {
    if (type == "noise")
    {
      whistleCount++;
      onWhistleConfirmed();
     

      // Reset for next cooker whistle
      firstDetection = "";

      digitalWrite(ALERT_LED_PIN, HIGH);
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(ALERT_LED_PIN, LOW);
      digitalWrite(LED, LOW);
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  pinMode(ALERT_LED_PIN, OUTPUT);
  digitalWrite(ALERT_LED_PIN, LOW);
  pinMode(LED, OUTPUT);
  delay(1000);
  setupI2S();
  setupCounter(); 
}

// =====================================================
// MAIN INFERENCE LOOP
// =====================================================
void loop()
{

  // Fill sample buffer directly from INMP441 I2S stream
  size_t bytesRead = 0;
  int32_t raw32 = 0;

  handleCounterButtons();
  updateBuzzer();  
  for (int i = 0; i < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i++)
  {
    i2s_read(I2S_PORT, &raw32, sizeof(raw32), &bytesRead, portMAX_DELAY);

    if (i == 0)
      Serial.printf("bytesRead: %d raw32: %ld\n", bytesRead, raw32);
    // Convert 24-bit left-aligned audio to 16-bit PCM for TinyML model
    sampleBuffer[i] = (int16_t)(raw32 >> 14);
  }

  // Bind audio buffer to Edge Impulse signal pipeline
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &raw_audio_get_data;

  // Run neural network inference
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);


  /////////////////////////////////////////////////

  if (r != EI_IMPULSE_OK)
  {
    Serial.printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }



  // Print class probabilities to Serial Monitor
  
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    

    // Trigger action if 'whistle' confidence score exceeds 80% (0.80)
    if (strcmp(result.classification[ix].label, "cooker") == 0)
    {
      if (result.classification[ix].value > 0.85)
      {

        if (!whistleActive)
        {
          // just started detecting whistle — start the clock
          whistleActive = true;
          whistleStartTime = millis();
        }

        // only accept it once it's been sustained long enough
        if (millis() - whistleStartTime >= WHISTLE_MIN_DURATION_MS)
        {
          whistleCounter("whistle");
        }
      }
      else
      {
        // confidence dropped below threshold — reset the timer
        whistleActive = false;
      }
    }

    if (strcmp(result.classification[ix].label, "noise") == 0)
    {
      if (result.classification[ix].value > 0.85)
      {
        whistleCounter("noise");
      }
    }
  }

  Serial.println();
}
