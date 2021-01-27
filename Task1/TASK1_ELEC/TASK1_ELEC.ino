#define trigPin 8
#define echoPin 10
int LED=4;
int BUZZER=5;
double x,q=0,p,r=0.9544,xp=0,pp=1,k,xpp=1;

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  double duration, distance, timeforone;  //Local variables declaration
  digitalWrite(trigPin, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);       
  timeforone= duration/200;   //Time taken for one shoot of the pulse
  delayMicroseconds(1000);
  //KALMAN FILTER
  //process model
  x=xp;
      //prediction update
     
      p = pp + q;
     
      //measurement update
      k = p / (p + r);  //Kalman Gain
      x = x + k * (timeforone - x); //Measurement estimation
      p = (1 - k) * p;  //Error Co-variance
      xp=x;
      pp=p;
   //Distance value after Calibration    
    distance=((x-0.1329)/0.2805);
    Serial.println(distance*10);
    Serial.print(',');
    Serial.println(p);
    Serial.print('\n');
    //Condition for display
    if(p<=0.01)
    {
     buzzer();  //Function 
    }
   
}
    void buzzer()  //Calling the function
    {
     
      digitalWrite(LED,HIGH);
        digitalWrite(BUZZER,HIGH);
        delay(1000);
        digitalWrite(LED,LOW);
        digitalWrite(BUZZER,LOW);
        delay(1000);  
       exit(0);
    } 
