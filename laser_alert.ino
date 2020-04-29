// 5 pins trip wire

//===============================================================|
// ADJUSTABLE SETTINGS                                           |
//===============================================================|
int PercentVolume= 100; //Enter value 1-100                        |
//===============================================================|


// Variables
  long millisCount;
// Output Pins
  int laserPin = 16;
  int buzzerPin = 3;
// Input Pins
  int lightSens = A0;


void setup() {
  pinMode(laserPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
}


void loop() {


//Armed mode
  digitalWrite(laserPin,HIGH);  
  if (analogRead(lightSens) < 500) {
      beep(3);
  }
  else analogWrite(buzzerPin,0);
      
  if ((millis() - millisCount) >= 100) {
       millisCount = millis();
       stats();
      }
//---------------------------------------------------------------
//Buzzer volume test (used to determine buzzer volume increases 
//with PWM value 1-50, then cycles)
//for(int i=0; i<50;i++){
//analogWrite(buzzerPin,i);
//delay(200);
//Serial.println(i);
}
    
//}
//===============================================================
// SUB ROUTINES
//===============================================================
//Beep routine: beep(# of beeps)
void beep(int qty) {
  int count;
    for (count = 1;count<=qty;count++) {
      analogWrite(buzzerPin, PercentVolume/2);    //note max volume at 50, so divide user input value by 2
      delay(100);
      digitalWrite(buzzerPin, 0);
      delay(100);
    }
  } 



//Writes stats to the Serial Port
void stats() {
  Serial.print(" Light Sensor:");
  Serial.println(analogRead(lightSens));

}
