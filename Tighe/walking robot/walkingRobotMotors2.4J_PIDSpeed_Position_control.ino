
/* WALKING ROBOT MOTORS
 *  uses 2 pololu micro metal gear motors, 6 pole magnetic quadrature encoder, 
 *  and motor control. 
 *  *note this code only reads 6 of the 12 possible Counts Per Revolution
 *  pololu 75:1 microgear motor is actually 75.81:1
 *  
 *  Created by: Brandon Tighe
 *  On: 11/18/2018
*/
//------------------------------------------------------
// initialize variables:

    //note: arduino uno has two interrupt pins (2 and 3)
#define R_encOutA 2 //R encoder output A - pin 2 (interrupt)
#define R_encOutB 12 //R encoder output B

#define L_encOutA 3 // L encoder output A - pin 3 (interrupt)
#define L_encOutB 13 // L encoder output B 

const byte R_enable = 5;  //PWM speed control
const byte R_phase = 7;  //motor direction

const byte L_enable = 9;  //PWM speed control
const byte L_phase = 8;  //motor direction 

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//PID (proportional, integral, derivative)control      //
// tunable constants for P, I, and D                   //
//`````````````````````````````````````````````````````//
//velocity                                             //
double vKp = 1.1;                                      //
double vKi = 0.5;                                      //  
double vKd = .005;                                     //  
//position                                             //
double pKp = .9;                                       // 
double pKi = .00;                                      // not being used 
double pKd = .0;                                       //
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
int full= 1350;
int ninety= 800;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
// MOTOR CONTROLS                                                                                 |
//````````````````````````````````````````````````````````````````````````````````````````````````|
int midCycleDelay= 0;                      // seconds delay in the middle of a cycle              |
int loopDelay= 3;                          // seconds between cycle iterations                    |
int cycles= 3;                             // number of cycles motors do                          |
int cableDisplacement= ninety*2 + full*0 ; // number of encoder counts for specific configuration |
const byte spLim = 200;                    // speed limit for speed control                       |
int LOOPTIME= 1;                           // Time for loop iterations                            |
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|

// Define position PID control variables
float Rp_error = 0;
float Rsum_P_error = 0;
float Rp_lastError = 0;
float Lp_error = 0;
float Lsum_P_error = 0;
float Lp_last_error = 0;
 

// Define velocity PID control variables
float Rv_error = 0;
float Rv_totalError = 0;
float Rv_lastError = 0;
float Lv_error = 0;
float Lv_totalError = 0;
float Lv_lastError = 0; 
//

// Velocity information
unsigned long Rv_t_initial = 0;
int  Rv_lastPos = 0;
float  Rv_lastSpeed = 0;


unsigned long L_t_initial = 0;
int  L_lastPos = 0;
float  L_lastSpeed = 0;
//-------------------

 float   degPerSec = 0;


volatile int R_encPos = 0;  // initial endcoder position (to negate overspin problems)
volatile int L_encPos = 0; 

//-----------------------------------------------
void setup() {
  // setup:

  pinMode(R_encOutA, INPUT);      //set encoder out A to an input pin(2) on arduino
  digitalWrite(R_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(R_encOutB, INPUT);      //set encoder out B to an input pin(12) on arduino
  digitalWrite(R_encOutB, HIGH);  // turn on pull-up resistor

  pinMode(L_encOutA, INPUT);      //set encoder out A to an input pin(3) on arduino
  digitalWrite(L_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(L_encOutB, INPUT);      //set encoder out B to an input pin(13) on arduino
  digitalWrite(L_encOutB, HIGH);  // turn on pull-up resistor

  attachInterrupt(0, R_doEncoder, CHANGE);  //encoder pin on interrupt 0 - (pin 2)
  attachInterrupt(1, L_doEncoder, CHANGE);  //encoder pin on interrupt 1 - (pin 3) 

  Serial.begin(115200); //Using a higher serial begin rate increases accuracy
  Serial.print("Start");
  
  pinMode(R_enable, OUTPUT); //sets R_enable pin as output (to determine SPEED)
  pinMode(R_phase, OUTPUT); //sets R_phase pin as output (to determine DIRECTION)
  pinMode(L_enable, OUTPUT); //sets L_enable pin as output (to determine SPEED)
  pinMode(L_phase, OUTPUT); //sets L_phase pin as output (to determine DIRECTION)
}
int i;
int mSpeed=0;
int lastTimePrint=millis();
void loop() {
//  // put your main code here, to run repeatedly:

//testing Velocity control
//PIDVelControl(800,R_encPos,Rv_error,Rv_totalError,Rv_lastError,Rv_lastPos,Rv_t_initial,Rv_lastSpeed, R_phase,R_enable);

 if ((millis()-lastTimePrint)>=LOOPTIME){
      lastTimePrint = millis(); 
      
for (i=0;i<cycles; i++)          //for loop to delay after 3 cycles 
{
 do
 {  //wind cable
  //note angle conversion .791452 in setpoint input
  PIDPosControl(cableDisplacement*.791452315, R_encPos, Rp_error, Rsum_P_error,Rv_error, Rv_totalError, Rv_lastError,Rv_lastPos, Rv_t_initial, Rv_lastSpeed, R_phase, R_enable);
  //delay(1000); 
  PIDPosControl(cableDisplacement*.791452315, L_encPos, Lp_error, Lsum_P_error,Lv_error, Lv_totalError, Lv_lastError,L_lastPos, L_t_initial, L_lastSpeed, L_phase, L_enable);
  //delay(100);

Serial.print("  R_encPos:");
Serial.print(R_encPos);  
Serial.print("  ");  
Serial.print("L_encPos:");
Serial.println (L_encPos); 
}
 while((abs(Rp_error) >10)||(abs(Lp_error) >10)); 
  delay(midCycleDelay*1000);
do
{ // unwind cable
  
  
  PIDPosControl(0*.791452315, R_encPos, Rp_error, Rsum_P_error,Rv_error, Rv_totalError, Rv_lastError,Rv_lastPos, Rv_t_initial, Rv_lastSpeed, R_phase, R_enable);
  PIDPosControl(0*.791452315, L_encPos, Lp_error, Lsum_P_error,Lv_error, Lv_totalError, Lv_lastError,L_lastPos, L_t_initial, L_lastSpeed, L_phase, L_enable);

Serial.print("  R_encPos:");
Serial.print(R_encPos);  
Serial.print("  ");  
Serial.print("L_encPos:");
Serial.println (L_encPos);  

}
 while((abs(Rp_error) >10)||(abs(Lp_error) >10)); 
}
delay(loopDelay*1000);

//// SPEED CHECK- used to test "get speeed" and tune vel control      
//float speed=getSpeed(R_encPos, Rv_lastPos, Rv_t_initial, Rv_lastSpeed );
//
//Serial.print("  degPerSec:  ");Serial.println (speed); 
// } 
//digitalWrite(L_phase, HIGH);
//analogWrite(L_enable,40);

}}

//============================================================================
//Sub-routines (Functions)
//============================================================================

//-------------------------------------------------------------------------------------------
// ENCODERS
//...........................................................................................

//turn the encoder outputs into a position/direction tracker: 
//turn the steps the encoder reads into 'ticks' to determine position
void R_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(R_encOutA) == digitalRead(R_encOutB)) {
    R_encPos--;
  } else {
    R_encPos++;
    }
  }
  void L_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(L_encOutA) == digitalRead(L_encOutB)) {
    L_encPos--;
  } else {
    L_encPos++;
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
    
 if (abs(p_error)< 10){
  PIDadjustment_scaled = 0;
 }
   analogWrite(pwm,  PIDadjustment_scaled);

//    Serial.print(R_encPos);
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
  
  float mSpeed = getSpeed(R_encPos, Rv_lastPos, Rv_t_initial, Rv_lastSpeed);    //input (gets actual speed)
  
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
//Serial.print("r pos:"  ); Serial.println(L_encPos); 
// Serial.print("  degPerSec:  ");Serial.println (degPerSec);  
//  
  return degPerSec;
}

////----------------------------------------------------------------------------------------------
//// PRINT MOTOR INFO every 500 ms
////..............................................................................................
//
//   void printMotorInfo() {
//    if ((millis()-lastTimePrint)>=500){
//      lastTimePrint = millis();
//   Serial.print("r pos:"  );
//  Serial.print(R_encPos);
//  Serial.print("  degPerSec:  ");
//  Serial.println (degPerSec);  
// 
//    }
//  }
