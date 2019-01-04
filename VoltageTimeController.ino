/* Voltage Time Controller- for use with relay 
 * 
 * best settings for ONE 3mmn PLA is 10.5 V for 5 seconds
 *                for TWO 3mm PLA (series) is 21.0 V for 5 seconds
 *                for TWO 3mm PLA (parallel) is 10.5 V for 5 seconds (2 cycles) 
 *                
 * Power supply limits the current to 5.2A, 
 * all 3 sides:(v~ 6.47V ), use this setting for 9 seconds 
 *     2 sides:(v~ 9.0V ), use this setting for 7 seconds 
 *   Note: turn down voltage to 8.5 for 1 side
 *     1 side: 7 seconds
 *     
 * Notes on reshaping rigid (cold) structure:
 *  ititial deployed height: 8cm
 *  collapsed height: 4cm
 *  total axial displacement 4cm; (axial strain=.5)
 *  
 *  reshaped to (cold) height= 6cm:
 *      structure can hold 1kg 
 *      structure colapses under 2kg
 *  reshaped to (cold) height= 7cm:
 *      structure can hold 1kg
 *      structure holds 2kg, but is not stable enought to balance unsupported load
 *  
 *  
 *  
 */
 

int relay1 = A1 ;     // use analog pin 1 for relay 1
int relay2 = 12 ;     // pin 12 for relay 2 
int relay3 = 13 ;     // pin 13 fro relay 3 

int relay6 = 6;
int relay7 = 7;
int relay8 = 8;

int relay9 = 9;
int relay10 = 10;
int relay11 = 11;

#define encoderPinA 2  //encoder output A
#define encoderPinB 3  //encoder output B

const byte enable = 5;   //PWM for speed control 
const byte phase = 4;    //motor direction

volatile int encoderPos = 0;

//-------------------------------------------------------------------------------------
// Use this section to adjust the time that voltage is applied:
int timeOn =  2 ;       // time in (s) the volatge is ON

int oneRotation = 5200;
long cooling = 18000;
int rotspeed = 150;
//-------------------------------------------------------------------------------------

void setup() {
// setup code:
  // heating elements
 //Serial.begin(9600);
pinMode(relay1, OUTPUT);        // Set pin as output
digitalWrite(relay1,HIGH);     // Start with voltage OFF  

pinMode(relay2, OUTPUT);        // Set pin as output
digitalWrite(relay2,HIGH);     // Start with voltage OFF 

pinMode(relay3, OUTPUT);        // Set pin as output
digitalWrite(relay3,HIGH);     // Start with voltage OFF 

pinMode(relay6, OUTPUT);        // Set pin as output
digitalWrite(relay6,HIGH);     // Start with voltage OFF 

pinMode(relay7, OUTPUT);        // Set pin as output
digitalWrite(relay7,HIGH);     // Start with voltage OFF 

pinMode(relay8, OUTPUT);        // Set pin as output
digitalWrite(relay8,HIGH);     // Start with voltage OFF 

pinMode(relay9, OUTPUT);        // Set pin as output
digitalWrite(relay9,HIGH);     // Start with voltage OFF 

pinMode(relay10, OUTPUT);        // Set pin as output
digitalWrite(relay10,HIGH);     // Start with voltage OFF 

pinMode(relay11, OUTPUT);        // Set pin as output
digitalWrite(relay11,HIGH);     // Start with voltage OFF 
//----------------------------------------------------------------- 
//motor control
  pinMode(encoderPinA,INPUT);
  digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(0, doEncoder, CHANGE);   // encoder pin on interrupt 0 - pin 2

  Serial.begin(115200);
  Serial.println("START");
  
  pinMode(enable, OUTPUT);
  pinMode(phase,OUTPUT);
  
//digitalWrite(phase,LOW);

}
//-----------------------------------------------------------------
//loop code:

void loop() {
//loop code:

  Serial.println("notSureWhatThisIsFor" );
 Serial.println (encoderPos);
//
//  //deploy
//////Module 1
////digitalWrite(relay1,LOW);     //Voltage ON (off for left turn)
////digitalWrite(relay2,LOW);     //Voltage ON (off for right turn)
////digitalWrite(relay3,LOW);     //Voltage ON (off for up turn)
//
////Module 2   
//digitalWrite(relay6,LOW);     //Voltage ON (off for right turn)  (topJoint)
////digitalWrite(relay7,LOW);     //Voltage ON (off for up turn)     (Ljoint)
//digitalWrite(relay8,LOW);     //Voltage ON (off for left turn)   (Rjoint)
//////
//////Module 3
////digitalWrite(relay9,LOW);      //Voltage ON (off for left turn)     (topJoint)
////digitalWrite(relay10,LOW);     //Voltage ON (off for right turn)  (Ljoint)
////digitalWrite(relay11,LOW);     //Voltage ON (off for up turn)     (Rjoint)
//Serial.println("HEATING");
//delay(timeOn*1000);

//////----------------------------------------------------------------------------------------

//motor control

    
cwRotation();
     delay(1000);

ccwRotation();
    delay(1000);

cwRotation();
     delay(1000);

ccwRotation();
    delay(1000);
    
cwRotation();
     delay(1000);

ccwRotation();
    delay(1000);
////cool 
digitalWrite(relay1,HIGH);    // Voltage OFF
digitalWrite(relay2,HIGH);    // Voltage OFF
digitalWrite(relay3,HIGH);    // Voltage OFF
// 
digitalWrite(relay6,HIGH);    // Voltage OFF
digitalWrite(relay7,HIGH);    // Voltage OFF
digitalWrite(relay8,HIGH);    // Voltage OFF

digitalWrite(relay9   ,HIGH);    // Voltage OFF
digitalWrite(relay10,HIGH);    // Voltage OFF
digitalWrite(relay11,HIGH);    // Voltage OFF
delay(1000);
//////------------------------------------------------------------------------------------------
}

void cwRotation(){
    do{
    Serial.println (encoderPos);
    analogWrite(enable,rotspeed);
    digitalWrite(phase,HIGH);
      }
      while(encoderPos < oneRotation*1.1);
      analogWrite(enable,0);
}

void ccwRotation(){

    do{
    Serial.println (encoderPos);
    analogWrite(enable,rotspeed);
    digitalWrite(phase,LOW);
      }
      while(encoderPos > 0);
      analogWrite(enable,0);
}

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.
  */
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPos--;
  } else {
    encoderPos++;
  }}
////------------------------------------------------------------------------------------------


