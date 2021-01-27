#define trigPinA 8
#define echoPinA 10
#define trigPinB 12
#define echoPinB 13
int LED=4;
int BUZZER=5;
double x,q=0,p,r=0.2033,xp=0,pp=1,K,xpred,ppred,z;
void setup()
{
  Serial.begin(9600);
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
 }
void loop() {
  double durationA,durationB,distanceA,distanceB, timeforoneA,timeforoneB ;  //Local variables for distance and time measurement
  digitalWrite(trigPinA, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinA, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinA, LOW);
  durationA = pulseIn(echoPinA, HIGH);
  timeforoneA= durationA/200;  //time taken for sensor1
  delayMicroseconds(100);
  digitalWrite(trigPinB, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinB, LOW);
  durationB = pulseIn(echoPinB, HIGH);
  timeforoneB= durationB/200;  //time taken for sensor2
  delayMicroseconds(100);
  distanceA=(((timeforoneA+0.085)/0.2854));  //distance for sensor1
  distanceB=(((timeforoneB+0.085)/0.2854));  //distance for sensor2
  delay(250);
  //Serial.println('\t');
  xpred=xp;
  z=distanceB;
      //KALMAN FILTER
      //prediction update
      ppred=pp+q;
      //measurement update
      K = ppred / (ppred + r); //Kalman Gain
      x = xpred + K * (z- xpred); //Measurement update
      p = (1 - K) * ppred; //Error Co-variance
      xp=distanceA;
      pp=p;
    Serial.println(x*10);
    Serial.print(",");
    Serial.println(p);
    //Condition for display
     if(p<=0.01)
     {
      buzzer();
     }
}
void buzzer()
    {
        digitalWrite(LED,HIGH);
        digitalWrite(BUZZER,HIGH);
        delay(1000);
        digitalWrite(LED,LOW);
        digitalWrite(BUZZER,LOW);
        delay(1000);
        exit(0);
    }  
