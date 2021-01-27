#include <Servo.h> //define the servo library
#define trigPinA 8
#define echoPinA 9
#define trigPinL 12
#define echoPinL 13
#define trigPinR 6
#define echoPinR 7
Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

float steering,throttle,cycle,PWM_ON; //defining global variables to use later
float x,q=0,p,r=0.9544,xp=0,pp=1,k,xpp=1;
float e = 0, e1 = 0, e2 = 0;
float u=90;
float kp = 6, ki = 5, kd = 5; // Define PID parameters
float k1 = kp + ki + kd;
float k2 = -kp - 2*kd;
float k3 = kd;
int f;
float ed = 0, e1d = 0, e2d = 0;
float ud=90;
float kpd = 4, kid = 0, kdd =0.1 ; // Define PID parameters
float k1d = kpd + kid + kdd;
float k2d = -kpd - 2*kdd;
float k3d = kdd;
int fd;

void setup() {
  Serial.begin(9600); //start serial connection. Uncomment for PC
  ssm.attach(10);    //define that ssm is connected at pin 10
  esc.attach(11);     //define that esc is connected at pin 11
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  }

void loop() {
// Add control logic here
  float durationA, durationL, durationR; 
  float distanceA, distanceL, distanceR; 
  float timeforoneA, timeforoneL, timeforoneR;  //Local variables declaration
  
  digitalWrite(trigPinA, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinA, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinA, LOW);
  durationA = pulseIn(echoPinA, HIGH);      
  timeforoneA= durationA/200;   //Time taken for one shoot of the pulse
  distanceA=((timeforoneA-0.1329)/0.2805);
  
  delayMicroseconds(500);
  digitalWrite(trigPinL, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  durationL = pulseIn(echoPinL, HIGH);       
  timeforoneL= durationL/200; 
  distanceL=(timeforoneL-0.2006)/0.2855;//Time taken for one shoot of the pulse
  
  delayMicroseconds(500);
  digitalWrite(trigPinR, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  durationR = pulseIn(echoPinR, HIGH);       
  timeforoneR= durationR/200;
  distanceR=(timeforoneR-0.0002)/0.2852;//Time taken for one shoot of the pulse
  
  //delayMicroseconds(500);
  //KALMAN FILTER
  //process model
  /* x=xp;
      //prediction update
      p = pp + q     
      //measurement update
      k = p / (p + r);  //Kalman Gain
      x = x + k * (timeforone - x); //Measurement estimation
      p = (1 - k) * p;  //Error Co-variance
      xp=x;
      pp=p;*/
   //Distance value after Calibration    
  
    
    Serial.print("Errord ");
    Serial.print(ed);
    Serial.print(" ");
    Serial.print("ud ");
    Serial.print(ud);
    Serial.println(" ");
    //Condition for display
    /*if(p<=0.15)
    {*/
     ACC(distanceA,distanceL,distanceR);  //Function 
    //}
       
}


void ACC(int distance, int distanceL, int distanceR)  //Calling the function
    {     
      e2 = e1;
      e1 = e;
      e = (distance-40);  // Compute current control error
      u = u + k1*e + k2*e1 + k3*e2;
      u=constrain(u,85,95);
      //steering=90;
      //throttle=90;
      e2d = e1d;
      e1d = ed;
      ed = distanceR-distanceL;  // Compute current control error
      ud = ud + k1d*ed + k2d*e1d + k3d*e2d;
      ud=constrain(ud,70,110);
      if(e>-4 && e<4)
      {
        Serial.print("range is on");
        u = 90;
        ud=90;
      }
      if(ed>=-3 && ed<=3)
      {
        ud=90;
      }
      setVehicle(ud,u);
      delay(100);
      setVehicle(ud,90);
      delay(50);
      if (u<90)
          {
          f=-1;
          }
      else if(u==90) 
         {
         f=0;
         }
      else 
      {
        f=1;
      }
   }
//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicle(int s, int v) 
{
  s=min(max(0,s),180);  //saturate steering command
  v=min(max(75,v),105); //saturate velocity command
  ssm.write(s); //write steering value to steering servo
  esc.write(v); //write velocity value to the ESC unit
}
//***************** Do not change above part *****************//
