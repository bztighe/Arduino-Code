
/* QUAD WALKING ROBOT MOTORS
 *  uses 4 High Power 75:1 pololu micro metal gear motors, 6 pole magnetic quadrature encoder, 
 *  DV8835 dual motor control, 9v boost regulator, Arduino MICRO. 
 *  *note this code only reads 6 of the 12 possible Counts Per Revolution
 *  pololu 75:1 microgear motor is actually 75.81:1
 *  
 *  Created by: Brandon Tighe
 *  On: 11/18/2018
 *  modified on: 01/14/19  (new board/congiguration, added 2 more motors) 
 *  ====================================================================================================
*/
//===========================================================================================
// INITIALIZE VARIABLES/PINS
//===========================================================================================
//note: arduino MICRO has 4 interrupt pins (2,3,7,tx,rx)
#define LF_encOutA 1 //RF encoder output A - pin 1 (interrupt)
#define LF_encOutB A5 //RF encoder output B- pin 19 (A5)

#define LR_encOutA 0 // RR encoder output A - pin 0 (interrupt)
#define LR_encOutB A4 // RR encoder output B - pin 18 (A4)

#define RF_encOutA 2 //RF encoder output A - pin 2 (interrupt)
#define RF_encOutB A3 //RF encoder output B- pin 17 (A3)

#define RR_encOutA 3 // RR encoder output A - pin 3 (interrupt)
#define RR_encOutB A2 // RR encoder output B - pin 16 (A2)

const byte LF_enable = 6;  //PWM speed control
const byte LF_phase = 4;  //motor direction

const byte LR_enable = 5;  //PWM speed control
const byte LR_phase = 8;  //motor direction     

const byte RF_enable = 11;  //PWM speed control
const byte RF_phase = 9;  //motor direction

const byte RR_enable = 12;  //PWM speed control
const byte RR_phase = 10;  //motor direction 

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//PID (proportional, integral, derivative) TUNING      //
// tunable constants for P, I, and D                   //
//`````````````````````````````````````````````````````//
//velocity                                             //
double vKp = 2000;                                      // 2000     .545
double vKi = 200;                                      // 200        0
double vKd = 0.005;                                     //  .005      .5
//position                                             //
double pKp = 300;                                       // 1300      12
double pKi = 0.0;                                      // 0 
double pKd = 0.51;                                       // 0       .51
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

//------------------------------------------------------------------------------------------------|
//MOTOR TRAVEL                                                                                    |
int full= 1180; //depends on gear ratio: 1200 for(75.81:1); 1620 for(100.37:1);     for(150.58:1) | 
int ninety= 730; //depends on gear ratio: 800 for(75.81:1); 1120 for(100.37:1);     for(150.58:1) |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
// MOTOR CONTROLS                                                                                 |
//````````````````````````````````````````````````````````````````````````````````````````````````|
int midCycleDelay=0 ;                      // seconds delay in the middle of a cycle              |
int loopDelay= 3;                          // seconds between cycle iterations                    |
int cycles=3 ;                             // number of cycles motors (per loop iteration)        |
int cableDisplacement= ninety*1+ full*0;   //number of encoder counts for specific configuration  |
const byte spLim = 255;                    // speed limit for speed control 208max for 7.4v batt  |
int LOOPTIME= 1;                           // Time for loop iterations                            |
int allowableError = 32;                   // Error to stop motor in PID control                  |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|

// Define position PID control variables
float RFp_error = 0;
float RFsum_P_error = 0;
float RFp_lastError = 0;
float RRp_error = 0;
float RRsum_P_error = 0;
float RRp_last_error = 0;

float LFp_error = 0;
float LFsum_P_error = 0;
float LFp_lastError = 0;
float LRp_error = 0;
float LRsum_P_error = 0;
float LRp_last_error = 0;
// 

// Define velocity PID control variables
float RFv_error = 0;
float RFv_totalError = 0;
float RFv_lastError = 0;
float RRv_error = 0;
float RRv_totalError = 0;
float RRv_lastError = 0; 

float LFv_error = 0;
float LFv_totalError = 0;
float LFv_lastError = 0;
float LRv_error = 0;
float LRv_totalError = 0;
float LRv_lastError = 0; 
//

// Velocity information
unsigned long RFv_t_initial = 0;
int  RFv_lastPos = 0;
float  RFv_lastSpeed = 0;

unsigned long RR_t_initial = 0;
int  RR_lastPos = 0;
float  RR_lastSpeed = 0;

unsigned long LFv_t_initial = 0;
int  LFv_lastPos = 0;
float  LFv_lastSpeed = 0;

unsigned long LR_t_initial = 0;
int  LR_lastPos = 0;
float  LR_lastSpeed = 0;
//-------------------

float   degPerSec = 0;

volatile int RF_encPos = 0;  // initial endcoder position 
volatile int RR_encPos = 0; 
volatile int LF_encPos = 0;  // initial endcoder position 
volatile int LR_encPos = 0; 

//===========================================================================================
// SETUP
//===========================================================================================
void setup() {
  // setup:

  pinMode(RF_encOutA, INPUT);      //set encoder out A to an input pin(2) on arduino
  digitalWrite(RF_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(RF_encOutB, INPUT);      //set encoder out B to an input pin(A3) on arduino
  digitalWrite(RF_encOutB, HIGH);  // turn on pull-up resistor

  pinMode(RR_encOutA, INPUT);      //set encoder out A to an input pin(3) on arduino
  digitalWrite(RR_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(RR_encOutB, INPUT);      //set encoder out B to an input pin(A2) on arduino
  digitalWrite(RR_encOutB, HIGH);  // turn on pull-up resistor

  pinMode(LF_encOutA, INPUT);      //set encoder out A to an input pin(1) on arduino
  digitalWrite(LF_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(LF_encOutB, INPUT);      //set encoder out B to an input pin(A5) on arduino
  digitalWrite(LF_encOutB, HIGH);  // turn on pull-up resistor

  pinMode(LR_encOutA, INPUT);      //set encoder out A to an input pin(0) on arduino
  digitalWrite(LR_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(LR_encOutB, INPUT);      //set encoder out B to an input pin(A4) on arduino
  digitalWrite(LR_encOutB, HIGH);  // turn on pull-up resistor

  attachInterrupt(1, RF_doEncoder, CHANGE);  //encoder pin on interrupt 1 - (pin 2)
  attachInterrupt(0, RR_doEncoder, CHANGE);  //encoder pin on interrupt 0 - (pin 3) 
  attachInterrupt(3, LF_doEncoder, CHANGE);  //encoder pin on interrupt 3 - (pin 1)
  attachInterrupt(2, LR_doEncoder, CHANGE);  //encoder pin on interrupt 2 - (pin 0) 

  Serial.begin(115200); //Using a higher serial begin rate increases accuracy
  Serial.print("Start");
  
  pinMode(RF_enable, OUTPUT); //sets RF_enable pin as output (to determine SPEED)
  pinMode(RF_phase, OUTPUT); //sets RF_phase pin as output (to determine DIRECTION)
  pinMode(RR_enable, OUTPUT); //sets RR_enable pin as output (to determine SPEED)
  pinMode(RR_phase, OUTPUT); //sets RR_phase pin as output (to determine DIRECTION)

  pinMode(LF_enable, OUTPUT); //sets LF_enable pin as output (to determine SPEED)
  pinMode(LF_phase, OUTPUT); //sets LF_phase pin as output (to determine DIRECTION)
  pinMode(LR_enable, OUTPUT); //sets LR_enable pin as output (to determine SPEED)
  pinMode(LR_phase, OUTPUT); //sets LR_phase pin as output (to determine DIRECTION)
}
int i;
int mSpeed=0;
int lastTimePrint=millis();

//===========================================================================================
// MAIN LOOP
//===========================================================================================

void loop() {

//testing Velocity control
//PIDVelControl(800,RF_encPos,RFv_error,RFv_totalError,RFv_lastError,RFv_lastPos,RFv_t_initial,RFv_lastSpeed, RF_phase,RF_enable);

 if ((millis()-lastTimePrint)>=LOOPTIME)  //allows a delay per loop iteration
 {
      lastTimePrint = millis(); 
gripCrawl();      
//symetric();
//stagger();
//LRstagger();

//testMotors();     //  **manual control of motors
}
}



//==================================================================================================================================================================================================
//Sub-routines (Functions)
//==================================================================================================================================================================================================
//-------------------------------------------------------------------------------------------
// GRIP/CRAWL
//...........................................................................................
void gripCrawl(){
do{  
//Grip  ***note: cableDisplacement was increased by factor of 2 to account for disprepency between walking and gripping requirements***
   PIDPosControl(2*cableDisplacement*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
   PIDPosControl(2*cableDisplacement*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);

//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();
}
 while((abs(RRp_error) >allowableError)||(abs(LRp_error) >allowableError));
delay(500); //grip delay

for (i=0;i<cycles; i++)          //for loop to delay after n cycles 
{
 do
 {
  //wind cable
  //note angle conversion .791452 in setpoint input

//testMotors();

//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

//Right Windup
  PIDPosControl(cableDisplacement*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
//Left Windup
  PIDPosControl(cableDisplacement*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);

}
//original while
 while((abs(RFp_error) >allowableError)||(abs(LFp_error) >allowableError));
  delay(midCycleDelay*1000);

do
{ // unwind cable
  
//Right Unwind
  PIDPosControl(0*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
//Left Unwind
  PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  
//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

}
 while((abs(RFp_error) >allowableError)||(abs(LFp_error) >allowableError)); 

}

do{
//Release grip
  PIDPosControl(0*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
  PIDPosControl(0*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);

//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();
}

while((abs(RRp_error) >allowableError)||(abs(LRp_error) >allowableError));
delay(loopDelay*1000);
}
//-------------------------------------------------------------------------------------------
// STAGGERED STEPS
//...........................................................................................
void stagger(){
for (i=0;i<cycles; i++)          //for loop to delay after 3 cycles 
{
 do
 {
//STAGGERED STEPS====================================================== 
  //wind cable
  //note angle conversion .791452 in setpoint input

//testMotors();

//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

//Right Windup
  PIDPosControl(cableDisplacement*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
  //PIDPosControl(cableDisplacement*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Windup
  //PIDPosControl(cableDisplacement*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  PIDPosControl(cableDisplacement*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
//Staggars
  PIDPosControl(0*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);  
  PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
 }
 
//Stagger while
 while((abs(RFp_error) >allowableError)||(abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)||(abs(LRp_error) >allowableError)); 
  delay(midCycleDelay*1000);

do
{ // unwind cable
  
//Right Unwind
  PIDPosControl(0*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
 //PIDPosControl(0*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Unwind
  //PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  PIDPosControl(0*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
//Staggars
  PIDPosControl(cableDisplacement*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
  PIDPosControl(cableDisplacement*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  
//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

}
 while((abs(RFp_error) >allowableError)||(abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)||(abs(LRp_error) >allowableError)); 

}

//Return from staggar 
 do{
  PIDPosControl(0*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Unwind
  PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  
//Print positions(for SERIAL PLOTTER):
//----------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();
 }
 
 while((abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)); 
delay(loopDelay*1000);

//speedTest();
}

//-------------------------------------------------------------------------------------------
// Left-Right STAGGERED STEPS
//...........................................................................................
void LRstagger(){
for (i=0;i<cycles; i++)          //for loop to delay after 3 cycles 
{
 do
 {
//STAGGERED STEPS====================================================== 
  //wind cable
  //note angle conversion .791452 in setpoint input

//testMotors();

//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

//Right Windup
  PIDPosControl(cableDisplacement*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
  PIDPosControl(cableDisplacement*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Windup
  //PIDPosControl(cableDisplacement*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  //PIDPosControl(cableDisplacement*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
//Staggars (left pause)
  PIDPosControl(0*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
  PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
 }
 
//Stagger while
 while((abs(RFp_error) >allowableError)||(abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)||(abs(LRp_error) >allowableError)); 
  delay(midCycleDelay*1000);

do
{ // unwind cable
  
//Right Unwind
  PIDPosControl(0*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
  PIDPosControl(0*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Unwind
  //PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  //PIDPosControl(0*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
//Staggars(left windup)
  PIDPosControl(cableDisplacement*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
  PIDPosControl(cableDisplacement*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  
//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

}
 while((abs(RFp_error) >allowableError)||(abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)||(abs(LRp_error) >allowableError)); 

}

//Return from staggar 
 do{
  //Left Unwind
  PIDPosControl(0*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
  PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  
//Print positions(for SERIAL PLOTTER):
//----------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();
 }
 
 while((abs(RRp_error) >allowableError)||(abs(LRp_error) >allowableError)); 
delay(loopDelay*1000);

//speedTest();
}
//-------------------------------------------------------------------------------------------
// SYMETRIC STEPS
//...........................................................................................
void symetric(){
  
for (i=0;i<cycles; i++)          //for loop to delay after n cycles 
{
 do
 {
  //wind cable
  //note angle conversion .791452 in setpoint input

//testMotors();

//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

//Right Windup
  PIDPosControl(cableDisplacement*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
  PIDPosControl(cableDisplacement*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Windup
  PIDPosControl(cableDisplacement*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  PIDPosControl(cableDisplacement*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);

}
//original while
 while((abs(RFp_error) >allowableError)||(abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)||(abs(LRp_error) >allowableError)); 
  delay(midCycleDelay*1000);

do
{ // unwind cable
  
//Right Unwind
  PIDPosControl(0*.791452315, RF_encPos, RFp_error, RFsum_P_error,RFv_error, RFv_totalError, RFv_lastError,RFv_lastPos, RFv_t_initial, RFv_lastSpeed, RF_phase, RF_enable);
  PIDPosControl(0*.791452315, RR_encPos, RRp_error, RRsum_P_error,RRv_error, RRv_totalError, RRv_lastError,RR_lastPos, RR_t_initial, RR_lastSpeed, RR_phase, RR_enable);
//Left Unwind
  PIDPosControl(0*.791452315, LF_encPos, LFp_error, LFsum_P_error,LFv_error, LFv_totalError, LFv_lastError,LFv_lastPos, LFv_t_initial, LFv_lastSpeed, LF_phase, LF_enable);
  PIDPosControl(0*.791452315, LR_encPos, LRp_error, LRsum_P_error,LRv_error, LRv_totalError, LRv_lastError,LR_lastPos, LR_t_initial, LR_lastSpeed, LR_phase, LR_enable);
  
//Print positions(for serial PLOTTER):
//--------------------------------------
  printPlotter();

////Print positions(for serial MONITOR):
////----------------------------------------
// printMonitor();

}
 while((abs(RFp_error) >allowableError)||(abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)||(abs(LRp_error) >allowableError)); 

}
 while((abs(RRp_error) >allowableError)||(abs(LFp_error) >allowableError)); 
delay(loopDelay*1000);

//speedTest();
}

//==================================================================================================================================================================================================
//==================================================================================================================================================================================================
//-------------------------------------------------------------------------------------------
// PRINT SERIAL PLOTTER
//...........................................................................................
void printPlotter(){
    //Print positions(for SERIAL PLOTTER):
//----------------------------------------
//Boundaries
Serial.print(0);  
Serial.print(" "); 
Serial.print(cableDisplacement);  
Serial.print(" "); 
//Motor Positions
Serial.print(RF_encPos);  
Serial.print(" ");  
Serial.print (RR_encPos);  
Serial.print(" ");
Serial.print(LF_encPos);  
Serial.print(" ");  
Serial.println (LR_encPos);  
}
//-------------------------------------------------------------------------------------------
// PRINT SERIAL MONITOR
//...........................................................................................
void printMonitor(){
//Print positions(for serial MONITOR):
//Motor positions
Serial.print("  RF_encPos:");
Serial.print(RF_encPos);  
Serial.print(" ");  
Serial.print("RR_encPos:");
Serial.print (RR_encPos); 
Serial.print("    ");  
Serial.print("  LF_encPos:");
Serial.print(LF_encPos);  
Serial.print(" ");  
Serial.print("LR_encPos:");
Serial.println (LR_encPos); 
}
//-------------------------------------------------------------------------------------------
// ENCODERS
//...........................................................................................
//turn the encoder outputs into a position/direction tracker: 
//turn the steps the encoder reads into 'ticks' to determine position

void RF_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(RF_encOutA) == digitalRead(RF_encOutB)) {
    RF_encPos--;
  } else {
    RF_encPos++;
    }
  }
  
  void RR_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(RR_encOutA) == digitalRead(RR_encOutB)) {
    RR_encPos--;
  } else {
    RR_encPos++;
  }
}

void LF_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(LF_encOutA) == digitalRead(LF_encOutB)) {
    LF_encPos--;
  } else {
    LF_encPos++;
    }
  }
  
  void LR_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(LR_encOutA) == digitalRead(LR_encOutB)) {
    LR_encPos--;
  } else {
    LR_encPos++;
  }
}
//===============================================================================================
// PID control (proportional, integral, derivative closed loop control)
//===============================================================================================
//-----------------------------------------------------------------------------------------------
// PID Postion control
//...............................................................................................
void PIDPosControl(int setpoint, int encPos, float &p_error, float &sum_P_error,float &v_error, float &v_totalError, float &v_lastError,int &v_lastPos, unsigned long &t_initial, float &lastSpeed,byte dir, byte pwm) {

//initialize variables
  float angle = 0;  
  float Pos_PID_adjustment = 0;
  float  P_pTerm, P_iTerm, P_dTerm, lastP_error, delta_P_error;
  
  angle = (0.791452315 * encPos);          //count to angle conversion (360/(75.81*6)counts per rot)*encPos
  
  p_error=  setpoint - angle;              // for (pKp)
  sum_P_error += p_error;                  // for (pKi)    
  delta_P_error=  p_error-lastP_error;     // for (pKd)   **note: += means sum_P_error= sum_P_error+ p_error   
  
  P_pTerm = (pKp * p_error);               //Proportion term
  P_iTerm = (pKi * sum_P_error);           //integration term 
  P_dTerm = (pKd * delta_P_error);         //derivatinve term

  Pos_PID_adjustment= P_pTerm + P_iTerm + P_dTerm;
  Pos_PID_adjustment  = constrain(Pos_PID_adjustment, -1*spLim, spLim); //make sure constrain the speed to 360 degPerSec

  lastP_error=p_error;                     // for calculating change in error (delta_P_error) 

int PIDadjustment_scaled = PIDVelControl(Pos_PID_adjustment, encPos, v_error, v_totalError, v_lastError,v_lastPos,t_initial, lastSpeed, dir, pwm);

//make setPoint "Pos_PID_adjustment" contstant to tune Velocity control:
//int PIDadjustment_scaled = PIDVelControl(1000, encPos, v_error, v_totalError, v_lastError,v_lastPos,t_initial, lastSpeed, dir, pwm);
    
 if (abs(p_error)< allowableError){
  PIDadjustment_scaled = 0;
 }
   analogWrite(pwm,  PIDadjustment_scaled);

//    Serial.print(RF_encPos);
//     Serial.print(",");
//    Serial.println (mSpeed );
     
  delay(10);
}

//-----------------------------------------------------------------------------------------------
// PID Velocity control
//..................................................................................................
int PIDVelControl(int setPoint, int encoderPos, float &error, float &totalError, float &lastError, int &lastPos, unsigned long &t_initial, float &lastSpeed, byte dir, byte pwm)
{ 
  float changeError = 0,pTerm=0,iTerm=0,dTerm=0;
  float PIDadjustment = 0;
  float PIDadjustment_scaled = 0; // if the total gain we get is not in the PWM range(|255|) we scale it down.
  
  float mSpeed = getSpeed(RF_encPos, RFv_lastPos, RFv_t_initial, RFv_lastSpeed);    //input (gets actual speed)
  
  error = abs(setPoint) - abs(mSpeed); //error for (Kp)
  totalError =   totalError + error; //accumalate errors for integral term (Ki)
  changeError = error - lastError; // for derivative term (Kd)

  pTerm=(vKp * error);        //Proportion term
  iTerm=(vKi * totalError); //integration term    
  dTerm=(vKd * changeError);  //derivatinve term  
  
  PIDadjustment =pTerm + dTerm+ iTerm ;//total gain +   
  PIDadjustment = abs(PIDadjustment);//constraining to appropriate value 
  
  PIDadjustment_scaled = constrain(PIDadjustment, 0, spLim);//make sure its between 0-255
  lastError = error;
  
  if (setPoint >0) {
    digitalWrite(dir, HIGH);// Forward motion
   // analogWrite(pwm, PIDadjustment);            //for testing Vel control w/o pos control
  }  
  else {
    digitalWrite(dir, LOW);//Reverse motion
  //  analogWrite(pwm, PIDadjustment);              //for testing Vel control w/o pos control
       }
  
//for testing Vel control w/o pos control:
// Serial.print ("PIDadjustment: "); Serial.print (PIDadjustment_scaled); 
// Serial.print ("   error: " ); Serial.print (error );
//Serial.print ("   mSpeed: " ); Serial.println (mSpeed );

return PIDadjustment_scaled; 

}

//-----------------------------------------------------------------------------------------------
// GET SPEED
//...............................................................................................
float getSpeed(int newPos, int &lastPos, unsigned long &t_initial, float &lastSpeed )
{
 // 
  float   degPerSec = 0;      // initialize variable
  unsigned long t_final = millis(); // millis() return 0.001s, better put when pos is sampled?
  int timeChange = (t_final - t_initial);  // calculate delta time
  int difference = newPos - lastPos; // calculate delta position
  
  if (difference < 50000 && difference > -50000)  //error negation
  {
   degPerSec = .791452315 * difference * 1000 / (timeChange);  // [(delta Pos in degrees)/delta time]
   lastSpeed = degPerSec;
  }
  else // use previous speed if overflow has occurred
  {
      degPerSec = lastSpeed;
  }
  lastPos = newPos; //set P_final
  t_initial = t_final;   //set t_initial
  
  
//print to check functionality 
//Serial.print("r pos:"  ); Serial.println(RR_encPos); 
// Serial.print("  degPerSec:  ");Serial.println (degPerSec);  
//  
  return degPerSec;
}
//----------------------------------------------------------------------------------------------
// TEST FOR "GETSPEED" FUNCTION
//..............................................................................................
void speedTest(){
//// SPEED CHECK- used to test "get speeed" and tune vel control      
float speed=getSpeed(RF_encPos, RFv_lastPos, RFv_t_initial, RFv_lastSpeed );

Serial.print("  degPerSec:  ");Serial.println (speed); 
 
digitalWrite(RR_phase, HIGH);
analogWrite(RR_enable,40);
  }
//----------------------------------------------------------------------------------------------
// TEST MOTOR FUNCTION
//..............................................................................................
void testMotors(){
digitalWrite(RR_phase, LOW); //(HIGH= windup, LOW= unwind)
analogWrite(RR_enable,150);
digitalWrite(RF_phase, LOW);
analogWrite(RF_enable,150);
digitalWrite(LR_phase, LOW);
analogWrite(LR_enable,150);
digitalWrite(LF_phase, LOW);
analogWrite(LF_enable,150);
}
//----------------------------------------------------------------------------------------------
// PRINT MOTOR INFO every 500 ms
//..............................................................................................
   void printMotorInfo() {
    if ((millis()-lastTimePrint)>=500){
      lastTimePrint = millis();
   Serial.print("r pos:"  );
  Serial.print(RF_encPos);
  Serial.print("  degPerSec:  ");
  Serial.println (degPerSec);  
 
    }
  }
