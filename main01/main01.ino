#include <EasyTransfer.h>

//create object
EasyTransfer ET; 

struct REC_DATA_STRUCTURE1{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float pitch;
  float roll;
};

int throttle;
int steer;

//give a name to the group of data
REC_DATA_STRUCTURE1 mydata1;


#include <Servo.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

Servo myservo;  // throttle

volatile unsigned long period;
volatile boolean done;
unsigned long start;

volatile unsigned long period2;
volatile boolean done2;
unsigned long start2;

int demand;
int pot;

int mode;

unsigned long currentMillis;

double Pk1 = 10;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, out1, out2;

PID PID1(&Input1, &out1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);


long previousMillis = 0;    // set up timers
long interval = 20;        // time constant for timers

long previousSafetyMillis = 0;



void setup() {

  Serial.begin(57600);

 
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(4, INPUT_PULLUP); // mode
  attachInterrupt(1, go, CHANGE);
  attachInterrupt(0, go2, CHANGE);

  PID1.SetMode(AUTOMATIC);              // PID Setup - streering servo
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

  myservo.attach(8);

  ET.begin(details(mydata1), &Serial);


}
void loop() {

   currentMillis = millis();
         if (currentMillis - previousMillis >= interval) {  //start timed event
            previousMillis = currentMillis;     

            mode = digitalRead(4);

            if (mode == 0) {   /// mode 0

                  if (!done) 
                    return;            
                  done = false;
      
                  if (!done2) 
                    return;            
                  done2 = false;
                 
      
                  pot = analogRead(A0);
                  pot = pot-479; 
                  pot = map(pot, -255,255,-140,140)-115;          
      
                  demand = map(period,1000,2000,-255,255);
                  demand = constrain(demand,-90,110);
      
                  Input1 = pot;
                  Setpoint1 = demand;
      
                  PID1.Compute();
      
                  period2 = map(period2, 1000, 2000, 600, 1950);
      
                  myservo.writeMicroseconds(period2);  // throttle
      
                  //Serial.print (demand);
                  //Serial.print (" , ");
                  //Serial.println (period2);
      
      
      /*
      
                  Serial.print (demand);
                  Serial.print (" , ");
                  Serial.print (pot);
                  Serial.print (" , ");
                  Serial.println (out1);      
      
      */          
      
                  if (out1 > 0) {
                    out2 = abs(out1);
                    analogWrite(5,out2);
                    analogWrite(6,0);              
                  }
                  else if (out1 < -0) {
                    out2 = abs(out1);
                    analogWrite(6,out2);
                    analogWrite(5,0);                 
                  }
      
                  else {
                    analogWrite(5, 0);
                    analogWrite(6, 0);
                  }             
               
      
               } // end of timed event

         } // end of mode 1

         if (mode == 1) {


              if (ET.receiveData()) {   // start of data event

                        previousSafetyMillis = currentMillis;

                        pot = analogRead(A0);
                        pot = pot-479;                         
                        pot = map(pot, -255,255,-140,140)-115;
          
                        steer = (map(mydata1.roll, -80,80,-255,255)*-1)+20;
                        steer = constrain(steer,-90,110);
          
                        Input1 = pot;
                        Setpoint1 = steer;
          
                        PID1.Compute();
                        
                        if (out1 > 0) {
                          out2 = abs(out1);
                          analogWrite(5,out2);
                          analogWrite(6,0);              
                        }
                        else if (out1 < -0) {
                          out2 = abs(out1);
                          analogWrite(6,out2);
                          analogWrite(5,0);                 
                        }
            
                        else {
                          analogWrite(5, 0);
                          analogWrite(6, 0);
                        }
          
                        // deadspot throttle
          
                        if (mydata1.pitch > -5 && mydata1.pitch < 5) {
                            throttle = 1300;
                        }
          
                        else if (mydata1.pitch >= 5) {
                            throttle = map(mydata1.pitch, 5,45, 1300,1950);
                        }
          
                        else if (mydata1.pitch <= -5) {
                            throttle = map(mydata1.pitch, -45,-5, 600,1299);
                        }              
                        myservo.writeMicroseconds(throttle);

              }                                         // end of data event
                                                    

              // safeties

              if (currentMillis - previousSafetyMillis > 200) {
                    analogWrite(5, 0);
                    analogWrite(6, 0);
                    myservo.writeMicroseconds(1300);  // throttle centre
              }

              
                                      
          
         }  // end of mode
}




void go() {


  if (digitalRead(3) == HIGH) {
    start = micros();
  }
  else {
    period = micros() - start;
    done = true;
  }
}

void go2() {


  if (digitalRead(2) == HIGH) {
    start2 = micros();
  }
  else {
    period2 = micros() - start2;
    done2 = true;
  }
}





