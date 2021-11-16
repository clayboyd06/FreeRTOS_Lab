



/*  ECE474 Lab 4
 *  @file lab4_2_1.ino
 *  @author    Clayden Boyd, Jin Terada White
 *  @date      9-June-2021  
 *  
 *  @Mainpage
 *  
 *  @Section
 *  @brief Creates a game of Frogger on an 8x8 LED Display
 *    
 *    Game Rules:
 *      Frog controlled by analog joystick, can go up, down, right or left (right is the goal)
 *        When the frog moves, speaker emits a sound 
 *      Cars are generated randomly then shifted vertically every enable (X # of tics)
 *      Task 5 Checks the location of the frog and the cars, if the car and the frog are in the same location,
 *        gameover occurs, 7 seg display shows L, speaker plays lose tune. 
 *      Every time the frog reaches the right side, the gamefield gets reset to act as a "rollover
 *      The 7 Segment display keeps of player Score!
 * 
 */

#include <Keypad.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h> 
#include <task.h>
#include <FreeRTOSConfig.h>
#define INCLUDE_vTaskSuspend 1

// Task RT1
#define LEDPIN           8
#define ENABLE           1
//------------TIMER 4 DEFINES---------------------
#define SPEAKER            OCR4A
#define SPEAKDDR           DDRH
#define SPEAKBIT           3 
#define PRESCALE4         (1<<CS42) // 62.5 kHz
#define CLK4_FREQ          62500
#define TIMER4_CTC_A_SET   (1<<COM4A0)
#define TIMER4_CTC_A_CLR   (1<<COM4A1) | 3
//--------------TIMER 5 DEFINES ----------------
#define PRESCALE5         (1<<CS51 | 1<<CS50) // 62.5 kHz
#define CLK5_FREQ          250000
#define TIMER5_CTC_A_SET   (1<<COM5A0)
#define TIMER5_CTC_A_CLR   (1<<COM5A1) | 3
//====================================================================
// 7 Seg Display Setup
/*
 *  7-seg wiring (Works with Clay's Setup)
 *
 *  Function   7-Seg Pin   ArdMega Pin
 *     Eseg       1           52   PB1
 *     Dseg       2           53   PB0
 *     pt.        3           51   PB2
 *     Cseg       4           46   PL3
 *     Gseg       5           47   PL2
 *     DIG4       6           45   PL4
 *     Bseg       7           27   PA5
 *     DIG3       8           29   PA7
 *     DIG2       9           31   PC6
 *     Fseg      10           37   PC0
 *     Aseg      11           39   PG2
 *     DIG1      12           41   PG0
 */
//  Specific ports and pins for the segments as wired by setup
volatile byte segbits[8]   = {2, 5,  3,      0,      1,      0,      2,      2 };
 
volatile uint8_t *  segports[8] = {&PORTG, &PORTA, &PORTL, &PORTB, &PORTB, &PORTC, &PORTL, &PORTB};
volatile uint8_t *  segddrs[8]  = {&DDRG,  &DDRA,  &DDRL,  &DDRB,  &DDRB,  &DDRC,  &DDRL,  &DDRB};

uint8_t  digbits[4]  = {0,     6,     7,     4};
volatile uint8_t *  digports[4] = {&PORTG, &PORTC, &PORTA, &PORTL};
volatile uint8_t *  digddrs[4]  = {&DDRG, &DDRC, &DDRA, &DDRL};

volatile static int digsegs[11][8] = { 
{1, 1, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 0, 1, 1, 0, 1, 0},
{1, 1, 1, 1, 0, 0, 1, 0},
{0, 1, 1, 0, 0, 1, 1, 0},
{1, 0, 1, 1, 0, 1, 1, 0},
{1, 0, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 0, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0}}; //L

////////////////////////////////////////////
//====================================================================
// 4x4 Keypad Setup
const byte numRows= 4; //number of rows on the keypad
const byte numCols= 4; //number of columns on the keypad

//keymap defines the key pressed according to the row and columns just as appears on the keypad
char keymap[numRows][numCols]=
{
{'<', '^', '>', 'R'},
{'<', '^', '>', 'R'},
{'<', '^', '>', 'R'},
{'<', 'v', '>', 'R'}
};

//Code that shows the the keypad connections to the arduino terminals
byte rowPins[numRows] = {A0,A1,A2,A3}; //Rows 0 to 3
byte colPins[numCols]= {A4,A5,A6,A7}; //Columns 0 to 3

//initializes an instance of the Keypad class
Keypad myKeypad= Keypad(makeKeymap(keymap), rowPins, colPins, numRows, numCols);

//////////////////////////////////
//  declare the Queue handle
static QueueHandle_t    Q_disp_value;

#define D4 106 //293 // hz 
#define E4 94 //329 // hz 
#define REST  0
#define SOUND_LENGTH 57
static int movSound[] = {D4, REST, E4, REST};
static int rhythm[] = {4, 1, 2, 50}; // notes durations 
TaskHandle_t xBoop = NULL;
TaskHandle_t xGame = NULL;

// ============= FROGGER ======================
#define OP_DECODEMODE  8
#define OP_SCANLIMIT   10
#define OP_SHUTDOWN    11
#define OP_DISPLAYTEST 14
#define OP_INTENSITY   10
#define INPUT_THRESHOLD 430
// ==================== Game Methods ==========================
static byte spidata[2]; //spi shift register uses 16 bits, 8 for ctrl and 8 for data
static int frogPos[2] = {3,0}; // row, col position for the frog
static int obstacle1[2] = {0,2}; // moves down
static int obstacle2[2] = {4,4}; // moves up
int obstacle3[2] = {0,7}; // moves down
static int playerScore = 230; // player score
static int timer = 0; // keeps track of obstacle time
static bool gameOver = false;
static int gameSpeed = 10;
//////////////////////
// gets a directional move as an int from analog input 
int getMove(char keyPressed);
// moves frog one space
void moveFrog(int pos);
//Transfers 1 SPI command to LED Matrix for given row
//Input: row - row in LED matrix
//       data - bit representation of LEDs in a given row; 1 indicates ON, 0 indicates OFF
void spiTransfer(volatile byte row, volatile byte data);
// draws the obstacles if draw is true, clears them otherwise
void drawObstacles(bool draw);
// Will toggle on/off a single LED at the specified row and col
// set to True will turn LED on or keep it on, False will turn it off or keep it off
void changeLed(int row, int col, bool set);
// Function to set the kth bit of n
int bit_set(int n, int k);
// Function to clear the kth bit of n
int bit_clr(int n, int k);
// will move an obstacle up (0) or down (1) 
void moveObstacle(int obstacle[], int direction);
// checks is frog has collided with obstacle
bool checkCollision();
// helper method for checkCollision()
bool checkCollisionHelper();
// checks if frog made it to other side
bool checkWin();
//////////////////////////////////
int DIN = 12;
int CS =  11;
int CLK = 10;
volatile byte led[8] = {0b00001000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000};

// define tasks
void TaskRT1( void *pvParameters );
void timer4Setup();
void timer5Setup();
void Task_Counter( void *pvParameters );

/*
 * @brief initializes timer4 amd outputs. 
 */
void setup() {
  int dzero[4] = {0};
  // initialize serial communication at 19200 bits per second:
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  } 
  //Setup Timer4 for Sounds, Setup Timer 5 for ISR 
  timer4Setup();
  timer5Setup();
  TIMSK5 |= (1 << OCIE5A); 
  SREG |= (1<<7);
  SPEAKER=0;
    // Set up DDRS so all bits are outputs
  for (int i=0; i<8;i++)
      *(segddrs[i]) |= (1<<segbits[i]);
  for (int i=0; i<4;i++)
      *(digddrs[i]) |= (1<<digbits[i]);
      
      
// clear all digits (which are active low) by setting HIGH
  for (int i=0;i<4;i++) 
       *(digports[i]) |= (1<<digbits[i]);  // set digit HIGH

    // Set up DDRS so all bits are outputs
  for (int i=0; i<8;i++)
      *(segddrs[i]) |= (1<<segbits[i]);
  for (int i=0; i<4;i++)
      *(digddrs[i]) |= (1<<digbits[i]);
    
// clear all digits (which are active low) by setting HIGH
  for (int i=0;i<4;i++) 
       *(digports[i]) |= (1<<digbits[i]);  // set digit HIGH

  ///////////////////////////
  //
  //  Set up Queue
  Q_disp_value = xQueueCreate(2, 4*sizeof(int));
  // put in an initial value
  xQueueSendToBack(Q_disp_value, dzero, 0);

  // SETUP LED 
  DDRB = B11111111;
  PORTB = B00100000; 
  spiTransfer(OP_DISPLAYTEST,0);
  spiTransfer(OP_SCANLIMIT,7);
  spiTransfer(OP_DECODEMODE,0);
  spiTransfer(OP_SHUTDOWN,1);
  ///////////////////////
  
  // Set up tasks
  xTaskCreate(
    TaskRT1
    ,  "External Blink"   // A name just for humans
    ,  600  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  if(1){
    xTaskCreate(
      TaskBoop
      , "Hop Sound" // name
      , 850
      , NULL
      , 1
      , &xBoop );
  } 
  if(0){
    xTaskCreate(
      Task_Counter
      ,  "Up Counter"
      ,  350  // Stack size
      ,  NULL
      ,  3  // Priority
      ,  NULL );   }     

  if(0){
    xTaskCreate(
      TaskGame
      ,  "Controls the GameField"
      ,  800  // Stack size
      ,  NULL
      ,  1  // Priority
      ,  &xGame );   }   
  vTaskStartScheduler();
  
} // END setup
void loop() {//nothing 
}//END LOOP


//===========================================================
// ======================= TASKS =========================

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
 * @brief Plays a sound to indicate the start of a game 
 */
void TaskBoop(void *pvParameters)
{
  SPEAKDDR |= (1<<SPEAKBIT); //output mode
  SPEAKER=0;
  static int playTime=0; 
  static int i=0;
  static int noteDuration = rhythm[0]; 
  static int iters = -3;
  for (;;){
    if (iters == 1) {
      iters = 0;
      SPEAKER = 0;
      vTaskSuspend( NULL );   
    }
    SPEAKER = movSound[i];
    if (playTime == noteDuration){
      TCNT4 = 0;
      i++;
      SPEAKER = movSound[i];
      noteDuration += rhythm[i];
    } else if (playTime>=SOUND_LENGTH){
      playTime=-1;
      i=0;
      noteDuration = rhythm[0];
      iters++;
    }
    playTime++; 
    vTaskDelay(17 / portTICK_PERIOD_MS );
  }
}// END of RT2


/* @brief controls the play field for the game
 *  
 */
void TaskGame( void *pvParameters ) {
  for(;;) {
      if(!gameOver) {
        // obstacles
        drawObstacles(false);
        if (timer >= gameSpeed) {
          // move the obstacles
          moveObstacle(obstacle1, 0);
          moveObstacle(obstacle2, 1);
          moveObstacle(obstacle3, 0);
          //reset timer
          timer = 0;
        }
        // draw obstacles
        drawObstacles(true);
    
        // move frog
        char keypressed = myKeypad.getKey();
        if (keypressed != NO_KEY)
        {
          vTaskResume( xBoop );
        }
         int mymove = getMove(keypressed);
         if (mymove != -1) moveFrog(mymove);
        vTaskDelay(100/portTICK_PERIOD_MS);
        timer++;
    } else {
      vTaskSuspend( NULL );
    }
  }
}

/* @brief keeps track of the time that the player is alive
 * 
 */
void Task_Counter(void *pvParameters){
 static int *dptr;
 for(;;){
   dptr = i2ds(playerScore*10);
   xQueueSendToBack(Q_disp_value, dptr, 0);
   vTaskDelay(1000/portTICK_PERIOD_MS);
   }
 }
   
 /** ISR  to scan 7 seg display faster than FreeRTOS (BH)
*/
volatile   static int isrcount=0;

ISR(TIMER5_COMPA_vect){   
  #define DIGIT_ON_STATE   0
  #define DIGIT_ON_COUNT   3
  #define DIGIT_OFF_STATE  1
  #define DIGIT_OFF_COUNT  4

  static byte state=0;    
  static byte digitpos=0;
  static int dptr[4]={0};
  
  xQueueReceiveFromISR(Q_disp_value, dptr, 0);// do not pend
  if (isrcount == 0){
    state = DIGIT_ON_STATE;
    if (++digitpos >= 4) {
         digitpos =0;  
         }
    digit_light(digitpos);   // light the digit
    int dval = dptr[digitpos];  // get 0-9 value
    segLightUp(digsegs[dval]);  // light the segments
    }
  if (isrcount == DIGIT_ON_COUNT){
    state = DIGIT_OFF_STATE;
    digits_dark();   // turn off the digit
    segsClear(); // clear the segs->OFF
  }
  if (isrcount == DIGIT_OFF_COUNT){        
      isrcount = -1;
      }          
  isrcount++;

  } // END OF THE ISR
////////////////////////////////////////// end ISR Code


/* @brief    Lower the cathode (to turn on) a specific digit (0-3) (BH)
*
*   @param  index    0-3, which digit to enable.
*/
void digit_light(int index){
   // light exactly one digit at a time
   *(digports[index]) &= ~(1<<digbits[index]);  // active low the digit
}

/*  @brief   Turn off all 4 digits.  (BH) 
 */
void digits_dark(){ // make all digit outputs high/OFF
  for (int i=0; i<4; i++)  *(digports[i])  |= (1<<digbits[i]);
}

  
 /* @brief Clear all the segment lines (BH)
  *
  */
 void segsClear() {
   for (int i=0;i<8;i++)  *(segports[i]) &= ~(1<<segbits[i]);
   return;
 } 

/** @brief  Set output bits to light specific segments (BH)
 *
 *  @param segs[8]    An array of 8 ints.  0 of the segment is off, 1 if on
 *
 */
void segLightUp(volatile int segs[8]){
  for (int i=0; i<8; i++){ // set sements to high/ON
    if ( segs[i] >0 ) {
          *(segports[i]) |= (1<<segbits[i]);
    }
    else *(segports[i]) &= ~(1<<segbits[i]); // maybe cleared already by segsClear()?
    }
  return;
  }

 
/* @brief   Convert int to  4 digits (BH)
 *
 */
int* i2ds(int i){
//  int *d = malloc(4*sizeof(int));
  static int d[4] = {0};
  d[0] = i/1000;
  d[1] = (i-1000*d[0])/100;
  d[2] = (i-1000*d[0]-100*d[1])/10;
  d[3] = (i-1000*d[0]-100*d[1]-10*d[2]);
  return d;
}


/*
* @brief returns an integer representing a move given a row and col position
*   returns -1 if no move is made with given row and col
*   
*   @param int row - the horizontal value
*   @param int column - the vertical value  
*/
int getMove(char keyPress) {
  if (keyPress == '^') {
    return 3; // move up
  } else if (keyPress == 'v') {
    return 2; //move down
  } else if (keyPress == '<') {
    return 1; // move left
  } else if (keyPress == '>') {
    return 0; // move right
  } else if (keyPress == 'R') { // keyPress == R 
    vTaskSuspend( xGame ) ;
  }
  
  return -1;
}
/*
 * @brief moves the Frog and check for gameOver and win condition
 *  right is 0, left is 1, down is 2, up is 3
 * 
 * @param int pos - the positional change of the frog
 */
void moveFrog(int pos) {
  // turn off previous position
  changeLed(frogPos[0],frogPos[1], false);
  if (pos == 0 && frogPos[1] < 7) {
    // move right
    frogPos[1] += 1;
  } else if (pos == 1 && frogPos[1] > 0) {
    // move right
    frogPos[1] -= 1;
  } else if (pos == 2 && frogPos[0] < 7) {
    // move down
    frogPos[0] += 1;
  } else if (pos == 3 && frogPos[0] > 0) {
    // move up
    frogPos[0] -= 1;
  }
  changeLed(frogPos[0],frogPos[1], true);
  
  if (checkCollision()) {
    // game over
    //Serial.println("Collision!");
    gameOver = true;
  } else if (checkWin()) {
    // increment player score by 1
    playerScore++;
    gameSpeed /= 2;
//    Serial.println("Win!");
//    Serial.print("Score: ");
//    Serial.println(playerScore);
    // reset frog position
    changeLed(frogPos[0],frogPos[1], false);
    frogPos[0] = 3;
    frogPos[1] = 0;
    changeLed(frogPos[0],frogPos[1], true);
  }
}

/*
 * @brief moves the obstacle LEDS across the board
 * @param int obstacle[] - the locations of the obstacles
 * @param int direction - the direction to move: 0 for down, 1 for up
 */
void moveObstacle(int obstacle[], int direction) {
  if (direction == 0) {
    if (obstacle[0] == 0) {
        obstacle[0] = 7;
    } else {
      obstacle[0] -= 1;
    }
  } else if (direction == 1) {
    if (obstacle[0] == 7) {
      obstacle[0] = 0;
    } else {
      obstacle[0] += 1;
    }
  }
  if (checkCollision()) {
    //Serial.println("Collision!");
    gameOver = true;
  }
}

/* @brief creates the cars to avoid
 *  
 *  @param bool draw - decides whether to draw an obstacle (false when game ends)
 *  
 */
void drawObstacles(bool draw) {
  changeLed(obstacle1[0], obstacle1[1], draw);
  if (obstacle1[0] == 7) {
    changeLed(0, obstacle1[1], draw);
  } else {
    changeLed(obstacle1[0] + 1, obstacle1[1], draw);
  }

  changeLed(obstacle2[0], obstacle2[1], draw);
  if (obstacle2[0] == 7) {
    changeLed(0, obstacle2[1], draw);
  } else {
    changeLed(obstacle2[0] + 1, obstacle2[1], draw);
  }

  changeLed(obstacle3[0], obstacle3[1], draw);
  if (obstacle3[0] == 7) {
    changeLed(0, obstacle3[1], draw);
  } else {
    changeLed(obstacle3[0] + 1, obstacle3[1], draw);
  }
}

/*
 * @brief  checks if frog reaches right side 
 * @return true if frog reaches right side 
 */
bool checkWin() {
  return frogPos[1] == 7;
}

/*
 * @brief compares frog position to all obstacle positions
 * 
 * @return true if any collisons occur
 */
bool checkCollision() {
  bool o1Collision = checkCollisionHelper(obstacle1);
  bool o2Collision = checkCollisionHelper(obstacle2);
  bool o3Collision = checkCollisionHelper(obstacle3);
  return o1Collision || o2Collision || o3Collision;
}

/* @brief helps the collison checker by comparing the  obstacle column to the frog column
 *    and then if they are in the same column it checks the row
 * @return True if Frog hits an obstacle 
 * @param int obstacle[] the obstacle position to compare
 *  
 */
bool checkCollisionHelper(int obstacle[]) {
  bool oRow;
  if (obstacle[0] == 7) {
    oRow = frogPos[0] == obstacle[0] || frogPos[0] == 0;
  } else {
    oRow = frogPos[0] == obstacle[0] || frogPos[0] == obstacle[0] + 1;
  }
  bool oCol = frogPos[1] == obstacle[1];
  return oRow && oCol;
}

/*
 * @brief changes the 8x8 matrix 
 */
void changeLed(int row, int col, bool set) {
  int j = 0;
  int i = 0;
  
  volatile byte change = led[col];
  if (set) {
    change = bit_set(change, row);
  } else {
    change = bit_clr(change, row);
  }
  led[col] = change;
  for (j = 0; j < 8; j++){ //for each row, set the LEDs
    spiTransfer(j, led[j]);
  }
}
/*
 * @ brief transfers SPI control to the 8x8 matrix
 * @acknowledgements Ishaan 
 */
void spiTransfer(volatile byte opcode, volatile byte data){
  int offset = 0; //only 1 device
  int maxbytes = 2; //16 bits per SPI command
  
  for(int i = 0; i < maxbytes; i++) { //zero out spi data
    spidata[i] = (byte)0;
  }
  //load in spi data
  spidata[offset+1] = opcode+1;
  spidata[offset] = data;
  PORTB = B00000000;
  for(int i=maxbytes;i>0;i--)
    shiftOut(DIN,CLK,MSBFIRST,spidata[i-1]); //shift out 1 byte of data starting with leftmost bit
  PORTB = B00100000;
}

/*
 * @brief sets the nth bit
 * @return the set bit
 * @param int n - the current binary numb
 * @param int k - the bit to set
 */
int bit_set(int n, int k) {
  return (n | (1 << (k)));
}

/*
 * @brief clears the nth bit
 * @return the cleared bit
 * @param int n - the current binary numb
 * @param int k - the bit to clear
 */
int bit_clr(int n, int k) {
  return (n & (~(1 << (k))));
}

//======================END OF TASKS =========================
/*
 * @brief sets up timer 4 for CTC mode
 */
void timer4Setup() {
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  // Set to CTC mode and prescale
  TCCR4B |= (1<<WGM42); //CTC
  TCCR4B |= PRESCALE4; 
  TCCR4A |= TIMER4_CTC_A_SET;
}// END of timer 4 setup
/*
 * @sets up timer 4 for CTC mode 
 */
void timer5Setup() {
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  // Set to CTC mode and prescale
  TCCR5B |= (1<<WGM52); //CTC
  TCCR5B |= PRESCALE5; 
  TCCR5A |= TIMER5_CTC_A_SET;
}// END of timer 5 setup
