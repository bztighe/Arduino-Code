
/* WALKING ROBOT MOTORS
 *  uses 2 pololu micro metal gear motors, 6 pole magnetic quadrature encoder, 
 *  and motor control. 
 *  *note this code only reads 6 of the 12 possible Counts Per Revolution
 *  
 *  Created by: Brandon Tighe
 *  On: 11/18/2018
*/
//------------------------------------------------------
// initialize variables:

//note arduino uno has two interrupt pins (2 and 3)
#define R_encOutA 2 //R encoder output A - pin 2 (interrupt)
#define R_encOutB 12 //R encoder output B

#define L_encOutA 3 // L encoder output A - pin 3 (interrupt)
#define L_encOutB 13 // L encoder output B 

const byte R_enable = 5;  //PWM speed control
const byte R_phase = 7;  //motor direction

const byte L_enable = 9;  //PWM speed control
const byte L_phase = 8;  //motor direction 

volatile float R_encPos = 100;  // initial endcoder position (to negate overspin problems)
volatile float L_encPos = 100; 

bool R_dir=1; // boolean variable (only 1 or 0) for direction of motor (1=CW, 0=CCW)
bool L_dir=1; // boolean variable (only 1 or 0) for direction of motor (1=CW, 0=CCW)

int R_change_t=millis(); // time clock variable used to delay direction change (dir)
int R_last_change_t=0;  // initial value for previous time check 

int L_change_t=millis(); // time clock variable used to delay direction change (dir)
int L_last_change_t=0;  // initial value for previous time check 


int thr=400;          //300ms delay used to "turn off" direction change (dir) momentarily to avoid problem 

//-----------------------------------------------------
// Rotation control
//  *for some reason 800 is 1 rotation, not 900 as calculated
int oneRot = 800; //6 quadrature readings per rotation,geared 1:150 => 6*150= 900
float totalRot= 2.2*oneRot; // input desired number of rotations

//-----------------------------------------------------
// Speed control 
int percentSpeed = 100; // choose what % of full speed is used (0-100%)
int pwmSpeed= (255*percentSpeed/100); //pwm has 256 steps
//=====================================================


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

void loop() {
//  // put your main code here, to run repeatedly:

Rot2();         // call Rot2 function 
//delay(5);
}


//============================================================================
//Sub-routines (Functions)
//----------------------------------------------------------------------------

//CW/CCW rotation 2- for two motors 
void Rot2() {
  
    Serial.print("  Right Encoder Position: "); //serial display message 
    Serial.println (R_encPos);                  // print the encoder position
    Serial.print("Left Encoder Position: ");
    Serial.print (L_encPos);

  //This keeps dir change from happening multiple times before position reaches desired location:
  R_change_t=millis();                             //records time stamp(2) in ms   
   
    if((R_encPos >= (totalRot+100))||(R_encPos <=100))   //if position is outside selected range
      {
      if ((R_change_t-R_last_change_t)>=thr)          //if the change in time stamps is greater than 'thr'
        { R_dir =!R_dir;                                //change the direction 
         R_last_change_t=millis();                   // records time stamp (1) in ms 
        }
      }
      
    L_change_t=millis();                                  //records time stamp(2) in ms

     if((L_encPos >= (totalRot+100))||(L_encPos <=100))   //if position is outside selected range
       {
       if ((L_change_t-L_last_change_t)>=thr)          //if the change in time stamps is greater than 'thr'
        { L_dir =!L_dir;                                //change the direction 
         L_last_change_t=millis();                   // records time stamp (1) in ms 
        }
      }

if (R_dir==1)                                      //if dir=1 (marker to rotate CW (HIGH phase)
   { digitalWrite(R_phase, HIGH);                //CW
    analogWrite(R_enable, pwmSpeed);             //MOTOR ON, select speed   
   }
   
else                                            // dir marker =0
   {
    digitalWrite(R_phase, LOW);                 //CCW
    analogWrite(R_enable, pwmSpeed);            // ON    
}

if (L_dir==1)                                      //if dir=1 (marker to rotate CW (HIGH phase)
   { digitalWrite(L_phase, HIGH);                //CW
    analogWrite(L_enable, pwmSpeed);             //MOTOR ON, select speed   
   }
   
else                                            // dir marker =0
   {
    digitalWrite(L_phase, LOW);                 //CCW
    analogWrite(L_enable, pwmSpeed);            // ON    
}

}


// turn the steps the encoder reads into 'ticks' to determine position
void R_doEncoder() {
  // Either adds a tick or subtracts one, depending on direction of encoder:
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(R_encOutA) == digitalRead(R_encOutB)) {
    R_encPos--;
  } else {
    R_encPos++;

  }}

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

