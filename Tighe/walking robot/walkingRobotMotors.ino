
/* WALKING ROBOT MOTORS
 *  uses pololu micro metal gear motor, 6 pole magnetic quadrature encoder, 
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
  pinMode(R_encOutB, INPUT);      //set encoder out B to an input pin(3) on arduino
  digitalWrite(R_encOutB, HIGH);  // turn on pull-up resistor

  pinMode(L_encOutA, INPUT);      //set encoder out A to an input pin(2) on arduino
  digitalWrite(L_encOutA, HIGH);  //turn on pull-up resistor
  pinMode(L_encOutB, INPUT);      //set encoder out B to an input pin(3) on arduino
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
  // put your main code here, to run repeatedly:
  
   //motor control

  CWrot();    //Call clockwise rotation function
  delay(100);

delay(200);

  CCWrot();   //Call counter-clockwise rotation function
  delay(100);

delay(200);

}
//============================================================================
//Sub-routines (Functions)
//----------------------------------------------------------------------------

// CW rotation
void CWrot() {
  do {
    Serial.print("Encoder Position: ");
    Serial.println (R_encPos);
    analogWrite(R_enable, pwmSpeed);
    digitalWrite(R_phase, HIGH);
  }
  while (R_encPos < (totalRot)+100 );
  analogWrite(R_enable, 0);
}

//CCW rotation
void CCWrot() {
  do {
    Serial.print("Encoder Position: ");
    Serial.println(R_encPos);
    analogWrite(R_enable, pwmSpeed);
    digitalWrite(R_phase, LOW);
  }
  while (R_encPos > 0+100);
  analogWrite(R_enable, 0);
}

// turn the steps the encoder reads into 'ticks' to determine position
void doEncoder() {
  //If pinA and pinB are both high or both low it spins forward,
  //if they are different, its going backward.

  if (digitalRead(R_encOutA) == digitalRead(R_encOutB)) {
    R_encPos--;
  } else {
    R_encPos++;

  }
}

