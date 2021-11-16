

/*  Embedded Systems Lab 4
 *  @file lab4_2_1.ino
 *  @author    Clayden Boyd, Jin Terada White
 *  @date      9-June-2021  
 *  
 *  @Mainpage
 *  
 *  @Section
 *    Completes the RT Tasks for Lab 4. Task RT1 blinks an OFF-board LED for 100ms ON, 200ms OFF.
 *    Task RT2 Plays the Close Encounters Theme 3 Times with a 1.5 Second break between play-throughs
 *    Task RT3 
 * 
 */


#include <arduinoFFT.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h> 
#include <task.h>
#include <FreeRTOSConfig.h>
#define INCLUDE_vTaskSuspend 1




// Task RT1
#define LEDPIN           10
#define ENABLE           1
//Task RT2
#define SPEAKER            OCR4A
#define SPEAKDDR           DDRH
#define SPEAKBIT           3 
#define PRESCALE          (1<<CS42) // 62.5 kHz
#define CLK_FREQ          62500
#define TIMER4_CTC_A_SET   (1<<COM4A0)
#define TIMER4_CTC_A_CLR   (1<<COM4A1) | 3
//===== “Close Encounters of the Third Kind” ====
#define D4 106 //293 // hz 
#define E4 94 //329 // hz 
#define C4 119 //261 // hz  
#define C3 239 //130 // hz  
#define Q4 158 //196 // hz  
#define REST  0
#define SONG_LENGTH 127
int melody[] = {D4, REST, E4, REST, C4, REST, C3, REST, Q4, REST};
int rhythm[] = {10, 1, 10, 1 , 10, 1, 10, 1, 20, 63}; // notes durations 
//====================================================================

//Task RT3
arduinoFFT FFT = arduinoFFT();
const uint16_t samples = 128;
double randNumbs[samples];
double vReal[samples];
double vImag[samples];
static QueueHandle_t Q_fft_vals;
static QueueHandle_t Q_times;
static double (*ptr)[samples];
static int (*fftTime);
// define tasks
void TaskRT1( void *pvParameters );
void TaskRT2( void *pvParameters );
void TaskRT3p0( void *pvParameters );
void TaskRT3( void *pvParameters );
void TaskRT4( void *pvParameters );

/*
 * @brief initializes timer4 amd outputs. 
 */
void setup() {
  timer4Setup();
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  } 
  // Set up task RT1
  xTaskCreate(
    TaskRT1
    ,  "External Blink"   // A name just for humans
    ,  600  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  if(1){
    xTaskCreate(
      TaskRT2
      , "Theme Song" // name
      , 850
      , NULL
      , 3
      , NULL );
  } 
  if(1){
    xTaskCreate(
      TaskRT3p0
      , "Initialize FFT Array"
      , 330
      , NULL
      , 1
      , NULL ); 
  }
  if(0){
    xTaskCreate(
      TaskRT4
      , "Compute FFT's"
      , 1430
      , NULL
      , 0
      , NULL );
  }
   
  vTaskStartScheduler();


}
void loop() {
  // nothing! 
}

/*
 * @brief Turns on external LED for 100 ms, then off for 200 ms 
 */
void TaskRT1(void *pvParameters)  
{
  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LEDPIN, OUTPUT);

  for(;;) // A Task shall never return or exit.
  {
    digitalWrite(LEDPIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); //  100 ms ON
    digitalWrite(LEDPIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200  / portTICK_PERIOD_MS ); // 200 ms OFF 
  }
} // END of RT1


/*
 * @brief Plays the theme from "Close Encounters" on external speaker. Pauses 1.5 seconds between playbacks 
 *  and stops itself
 */
void TaskRT2(void *pvParameters)
{
  SPEAKDDR |= (1<<SPEAKBIT); //output mode
  SPEAKER=0;
  static int playTime=0; 
  static int i=0;
  static int noteDuration = rhythm[0]; 
  static int iters = 0;
  for (;;){
    if (iters >= 3) {
      SPEAKER = 0;
      vTaskSuspend( NULL );   
    }
    SPEAKER = melody[i];
    if (playTime == noteDuration){
      TCNT4 = 0;
      i++;
      SPEAKER = melody[i];
      noteDuration += rhythm[i];
    } else if (playTime>=SONG_LENGTH){
      playTime=-1;
      i=0;
      noteDuration = rhythm[0];
      iters++;
    }
    playTime++; 
    vTaskDelay(17 / portTICK_PERIOD_MS );
  }
}// END of RT2

/*
 * @brief sets up timer 4
 */
void timer4Setup() {
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  // Set to CTC mode and prescale
  TCCR4B |= (1<<WGM42); //CTC
  TCCR4B |= PRESCALE; 
  TCCR4A |= TIMER4_CTC_A_SET;
}// end of timer setup

/*
 * @brief initializes a queue and fills it with pseudo random numbers.
 *  Then starts task RT-3 and stops itself
 */
void TaskRT3p0(void *pvParameters) {
  static int counter;
  for(;;){
    counter++;
    int timeOf5;
    timeOf5 = 52 - random(9);
    if (counter==305){
      counter=-1;
      Serial.print("Wall Clock Time: "); Serial.println((timeOf5));
    }
  }
}// END 

/*
 * @brief initializes a queue of FFT times, sends random numbers to the Queue Q_fft_vals
 * 
 */
void TaskRT3(void *pvParameters) {
  int timeOf5;
  Q_times = xQueueCreate(samples, sizeof(uint16_t));
  ptr = &randNumbs;
  for(;;) {
    xQueueSendToBack(Q_fft_vals, ptr, 10);  // fill Q with the random' values
    if(xQueueReceive(Q_times, fftTime, 20)){
     timeOf5 = (int)fftTime/5;
     Serial.print("Wall Clock Time: "); Serial.println((timeOf5));
    }
  }
}

void TaskRT4(void *pvParameters) {
  vTaskDelay(1500/portTICK_PERIOD_MS);
  for (;;) {
    
    if(xQueueReceive(Q_fft_vals, ptr, 0) == pdPASS){ 
      int startTime = millis();
    
      for (uint16_t fftCount=0;fftCount<5;fftCount++){
         for (uint16_t i = 0; i < samples; i++)
        {
          vReal[i] = *ptr[i];
          vImag[i] = 0.0;
        }
        FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
        FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
      }
      int endTime= millis();
      *fftTime = (endTime - startTime);
      xQueueSendToBack(Q_times, fftTime, 0);
    }
  }  
}
