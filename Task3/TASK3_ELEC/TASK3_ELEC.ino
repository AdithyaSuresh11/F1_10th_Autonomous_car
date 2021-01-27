#define trigPinA 8
#define echoPinA 10
#define trigPinB 12
#define echoPinB 13
double LED=4;
double BUZZER=5;
double x,y,yp=0,q=0,px,py,kx,ky,r=0.2033,xp=0,ppx=1000,ppy=1000,xpred,ppred,z,l=6.5,X,Y,H1,H2,L,O;
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
  double durationA,durationB,distanceA,distanceB, timeforoneA,timeforoneB ; //Local variables declaration
  digitalWrite(trigPinA, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinA, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinA, LOW);
  durationA = pulseIn(echoPinA, HIGH);
  timeforoneA= durationA/200;  //time for sensor1
  delayMicroseconds(100);
  digitalWrite(trigPinB, LOW);
  delayMicroseconds (2);
  digitalWrite(trigPinB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinB, LOW);
  durationB = pulseIn(echoPinB, HIGH);
  timeforoneB= durationB/200;  //time for sensor2
  delayMicroseconds(100);
  distanceA= (timeforoneA+0.085)/0.2854;  //distance for sensor1
  distanceB=(timeforoneB+0.085)/0.2854;   //distance for sensor2
  delay(250);
  //LOCALIZATION
  H1=distanceA*distanceA;
  H2=distanceB*distanceB;
  L=l*l;
  X=abs((H1-H2+L)/(2*l));
  O=H1-H2+L;
  Y=sqrt(H1-((O*O)/(4*L)));
  //KALMAN FILTER
  //process model
 x=xp;
      //SENSOR 1
      //prediction update
     
      px = ppx + q;
     
      //measurement update
      kx = px / (px + r); //Kalman Gain 
      x = x + kx * (X - x); //Measurement Update
      px = (1 - kx) * px; //Error Co-variance
      xp=x;
      ppx=px;
      y=yp;

      //KALMAN FILTER
      //SENSOR 2
      //prediction update
     
      py = ppy + q;
     
      //measurement update
      ky = py / (py + r); //Kalman Gain
      y = y + ky * (Y - y); //Measurement Gain
      py = (1 - ky) * py;  //Error Co-variance
      yp=y;
      ppy=py;
      
    Serial.print(x*10);
    Serial.print(',');
    Serial.print(y*10);
    Serial.print(',');
    Serial.print(px);
    Serial.print(',');
    Serial.print(py);
    Serial.print('\n');
    //Condition for display
    if(px<0.01&&py<0.01)
    {
        digitalWrite(LED,HIGH);
        digitalWrite(BUZZER,HIGH);
        delay(1000);
        digitalWrite(LED,LOW);
        digitalWrite(BUZZER,LOW);
        delay(1000);
        exit(0);
    }
     
}  
