// This is to test to ensure motors/drivers are working properly
//*note on driver 8835 MODE must have HIGH value (or hardwire to 5v)

const byte LF_enable = 6;  //PWM speed control
const byte LF_phase = 4;  //motor direction

const byte LR_enable = 5;  //PWM speed control
const byte LR_phase = 8;  //motor direction     ///**********************************check these**********************************

const byte RF_enable = 11;  //PWM speed control
const byte RF_phase = 9;  //motor direction

const byte RR_enable = 12;  //PWM speed control
const byte RR_phase = 10;  //motor direction 

void setup() {
  // put your setup code here, to run once:
  pinMode(RF_enable, OUTPUT); //sets RF_enable pin as output (to determine SPEED)
  pinMode(RF_phase, OUTPUT); //sets RF_phase pin as output (to determine DIRECTION)
  pinMode(RR_enable, OUTPUT); //sets RR_enable pin as output (to determine SPEED)
  pinMode(RR_phase, OUTPUT); //sets RR_phase pin as output (to determine DIRECTION)

  pinMode(LF_enable, OUTPUT); //sets RF_enable pin as output (to determine SPEED)
  pinMode(LF_phase, OUTPUT); //sets RF_phase pin as output (to determine DIRECTION)
  pinMode(LR_enable, OUTPUT); //sets RR_enable pin as output (to determine SPEED)
  pinMode(LR_phase, OUTPUT); //sets RR_phase pin as output (to determine DIRECTION)

}

void loop() {
  // put your main code here, to run repeatedly:


digitalWrite(RR_phase, HIGH);
analogWrite(RR_enable,200);
digitalWrite(RF_phase, HIGH);
analogWrite(RF_enable,220);

digitalWrite(LR_phase, HIGH);
analogWrite(LR_enable,200);
digitalWrite(LF_phase, HIGH);
analogWrite(LF_enable,200);
}
