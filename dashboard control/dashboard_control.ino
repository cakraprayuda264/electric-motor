#include <Nextion.h>

#include <DHT.h>
DHT dht2(3, DHT22);
DHT dht1(7, DHT22);
#define Ain1 A4 // ampere


float vArduino = 0.0;
int vActual = 0.0;
float R1 = 11000.0;
float R2 = 1000.0;
int rawValueRead= 0;
int speedsens   = 5; // speed
int Ain2        = A6; // volt
int Ain3        = A3; // speed
const int Ain0        = A0; // throttle in
int Aout        = 6; // throttle out
float volt1;
int steps       = 0;
float steps_old = 0;
float rps       = 0;
float temp      = 0;
float s         = 0;
int percentage = 0;

unsigned long start_time = 0;
unsigned long end_time   = 0;

const int reverse = 23; // reverse input
const int cruise  = 25; // cruise input
const int Lsein   = 35; // l sein input
const int Rsein   = 33; // r sein input
const int Lbeam   = 37; // l beam input
const int Hbeam   = 41; // h beam input
const int hazard  = 31; // H beam input
const int Horn    = 29; // horn input
const int Pbeam   = 45; // p beam input

const int relRS = 26; // r sein output
const int relLS = 28; // l sein output
const int relSL = 22; // s beam output

const int relHS = 34; // horn output
const int relRS2 = 32; // r sein output
const int relLS2 = 24; // l sein output
const int relHB  = 30; // hb light output


const int revPin = 38; // reverse output

int thrValue = 0;
int outValue = 0;
int RevState = 0;
int CruState = 0;
int LBState  = 1;
int HBState  = 1;
int RSState  = 1;
int LSState  = 1;
int RHState  = 0;
int RS2State = 1;
int LS2State = 1;
int SLState  = 1;
const float vcc    = 5.00;      // supply voltage 5V or 3.3V
const float factor = 0.02;      // 20mV/A is the factor
                   

void setup() 
{
  Serial.begin(9600);            //The default baud rate of the Nextion TFT is 9600.    
             //Define the pin as input
  pinMode(Ain2,INPUT);
  pinMode(Ain3,INPUT);
  pinMode(reverse,INPUT);
  pinMode(cruise,INPUT);
  pinMode(Horn, INPUT);
  pinMode(hazard, INPUT);
  pinMode(Lsein,INPUT);
  pinMode(Rsein,INPUT);
  pinMode(Lbeam,INPUT);
  pinMode(Hbeam,INPUT);
  pinMode(Pbeam,INPUT);
            //Define the pin as output
  pinMode(relHS,OUTPUT); 
  pinMode(revPin,OUTPUT);
  pinMode(relRS,OUTPUT);
  pinMode(relLS,OUTPUT);
  pinMode(relRS2,OUTPUT);
  pinMode(relLS2,OUTPUT);
  pinMode(relSL,OUTPUT);
  pinMode(relHB,OUTPUT);
  pinMode(Aout,OUTPUT);
  
  
  dht1.begin();
  dht2.begin();
}

  
void loop() 
{ 
  start_time=millis();
  end_time=start_time+1000;
  while(millis()<end_time);
  {
    if(digitalRead(speedsens));
    {
      steps=steps+1;
      while(digitalRead(speedsens));
    }
  }

  temp=steps-steps_old;
  steps_old=steps;
  rps=(temp/20);
  s = (50 * (rps/60) * 0.001885);

// read the analog in value:
  thrValue = analogRead(Ain0);
  // map it to the range of the analog out:
  outValue = map(thrValue, 200, 860, 57, 180);
  // change the analog out value:
  analogWrite(Aout, outValue);

  // print the results to the Serial Monitor:
  Serial.print("n5.val=");
  Serial.print(thrValue);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
           
  getRsein();
  getLsein();
  getHazard();
  getHorn();
  getHbeam();
  getTmotor();
  getTbatt();
  getReverse();
  getAmpere();
  getBattind();
 
}

void getRsein(){
   RSState = digitalRead(Rsein);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (RSState == HIGH) {
      // turn indicator on:
      digitalWrite(relRS, LOW);
      digitalWrite(relRS2, LOW);
      
      int rs = 100;
      Serial.print("raightSein.val=");              //We print the variable we want to change on the screen
      Serial.print(rs);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);

      
      delay(400);
      digitalWrite(relRS, HIGH);
      digitalWrite(relRS2, HIGH);

      int ro = 0;
      Serial.print("raightSein.val=");              //We print the variable we want to change on the screen
      Serial.print(ro);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
  
      delay(400);
  } else {
      // turn indicator off:
      digitalWrite(relRS, HIGH);
      digitalWrite(relRS2, HIGH);
      
      int ro = 0;
      Serial.print("raightSein.val=");              //We print the variable we want to change on the screen
      Serial.print(ro);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
      
  }
}

void getLsein(){
   LSState = digitalRead(Lsein);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (LSState == HIGH) {
      // turn indicator on:
      digitalWrite(relLS, LOW);
      digitalWrite(relLS2, LOW);
      
      int ls = 100;
      Serial.print("leftSein.val=");              //We print the variable we want to change on the screen
      Serial.print(ls);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff); 
      
      delay(400);
      digitalWrite(relLS, HIGH);
      digitalWrite(relLS2, HIGH);

      int lo = 0;
      Serial.print("leftSein.val=");              //We print the variable we want to change on the screen
      Serial.print(lo);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff); 
      
      delay(400);
  } else {
      // turn indicator off:
      digitalWrite(relLS, HIGH);
      digitalWrite(relLS2, HIGH);
      
      int lo = 0;
      Serial.print("leftSein.val=");              //We print the variable we want to change on the screen
      Serial.print(lo);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
      
  }
}

void getHazard(){
   RSState = digitalRead(hazard);
   LSState = digitalRead(hazard);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (RSState == HIGH) {
      // turn indicator on:
      digitalWrite(relRS, LOW);
      digitalWrite(relLS, LOW);
      digitalWrite(relRS2, LOW);
      digitalWrite(relLS2, LOW);

      int ls = 100;
      Serial.print("leftSein.val=");              //We print the variable we want to change on the screen
      Serial.print(ls);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff); 
      
      int rs = 100;
      Serial.print("raightSein.val=");              //We print the variable we want to change on the screen
      Serial.print(rs);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);

      
      delay(400);
      digitalWrite(relRS, HIGH);
      digitalWrite(relLS, HIGH);
      digitalWrite(relRS2, HIGH);
      digitalWrite(relLS2, HIGH);

      int ro = 0;
      Serial.print("raightSein.val=");              //We print the variable we want to change on the screen
      Serial.print(ro);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);

      int lo = 0;
      Serial.print("leftSein.val=");              //We print the variable we want to change on the screen
      Serial.print(lo);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff); 
  
      delay(400);
  } else {
      // turn indicator off:
      digitalWrite(relRS, HIGH);
      digitalWrite(relRS2, HIGH);
      digitalWrite(relLS, HIGH);
      digitalWrite(relLS2, HIGH);
      
      int ro = 0;
      Serial.print("raightSein.val=");              //We print the variable we want to change on the screen
      Serial.print(ro);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);

      int lo = 0;
      Serial.print("leftSein.val=");              //We print the variable we want to change on the screen
      Serial.print(lo);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff); 
      
  }
}

void getHorn(){
   RHState = digitalRead(Horn);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (RHState == HIGH) {
      // turn indicator on:
      digitalWrite(relHS, LOW);
      
      Serial.print("horn.val=");              //We print the variable we want to change on the screen
      Serial.print(RHState);
      
  } else {
      // turn indicator off:
      Serial.print("horn.val=");              //We print the variable we want to change on the screen
      Serial.print(RHState);
      digitalWrite(relHS, HIGH);
      
  }
}

void getHbeam(){
   HBState = (digitalRead(Hbeam)|| digitalRead(Pbeam));

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (HBState == HIGH) {
      // turn indicator on:
      digitalWrite(relHB, LOW);
     
      
      int Hs = 100;
      Serial.print("HBSein.val=");              //We print the variable we want to change on the screen
      Serial.print(Hs);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);

  } else {
      // turn indicator off:
      digitalWrite(relHB, HIGH);
      
      int Ho = 0;
      Serial.print("HBSein.val=");              //We print the variable we want to change on the screen
      Serial.print(Ho);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
      
  }
}



void getTmotor(){
  int celcM = dht1.readTemperature();
  Serial.print("motorT.val=");              //We print the variable we want to change on the screen
  Serial.print(celcM);                        //Print the value we want to be displayed
  Serial.write(0xff);                         //Always add 3 full bytes after...       
  Serial.write(0xff);
  Serial.write(0xff);
}

void getTbatt(){
  int celcB = dht2.readTemperature();
  Serial.print("battT.val=");              //We print the variable we want to change on the screen
  Serial.print(celcB);                        //Print the value we want to be displayed
  Serial.write(0xff);                         //Always add 3 full bytes after...       
  Serial.write(0xff);
  Serial.write(0xff);
}

void getAmpere(){
  volt1 = (5.0 / 1023.0)* analogRead(Ain1);
  volt1 = volt1 - (vcc * 0.5) + 0.007;
  float cur = volt1 / factor;

  rawValueRead = analogRead(Ain2);
  vArduino = (rawValueRead * 5.0) / 1024.0;
  vActual = vArduino / (R2/(R1+R2));
 

  Serial.print("ampere.val=");              //We print the variable we want to change on the screen
  Serial.print(cur,2);                        //Print the value we want to be displayed
  Serial.write(0xff);                         //Always add 3 full bytes after...       
  Serial.write(0xff);
  Serial.write(0xff);
 
  Serial.print("volt.val=");              //We print the variable we want to change on the screen
  Serial.print(vActual);                        //Print the value we want to be displayed
  Serial.write(0xff);                         //Always add 3 full bytes after...       
  Serial.write(0xff);
  Serial.write(0xff);
}

 void getReverse(){
   RevState = digitalRead(reverse);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (RevState == HIGH) {
      // turn indicator on:
      int rev = 100;
      Serial.print("reversein.val=");              //We print the variable we want to change on the screen
      Serial.print(rev);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
  
      digitalWrite(revPin, HIGH);
  } else {
      // turn indicator off:
      int rev = 0;
      Serial.print("reversein.val=");              //We print the variable we want to change on the screen
      Serial.print(rev);                        //Print the value we want to be displayed
      Serial.write(0xff);                         //Always add 3 full bytes after...       
      Serial.write(0xff);
      Serial.write(0xff);
      digitalWrite(revPin, LOW);
  }
 }

 void getBattind(){
rawValueRead = analogRead(Ain2);
//Serial.print(Ain2);
vArduino = (rawValueRead * 5.0) / 1024.0;
vActual = vArduino / (R2/(R1+R2));

percentage = map(Ain2, 0, 1023, 0, 1600);
  
  Serial.print("battin1.val=");              //We print the variable we want to change on the screen
  Serial.print(percentage);                        //Print the value we want to be displayed
  Serial.write(0xff);                         //Always add 3 full bytes after...       
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("battin2.val=");              //We print the variable we want to change on the screen
  Serial.print(percentage);                        //Print the value we want to be displayed
  Serial.write(0xff);                         //Always add 3 full bytes after...       
  Serial.write(0xff);
  Serial.write(0xff);
}
