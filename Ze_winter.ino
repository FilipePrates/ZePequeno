#define LEFT_SENSOR           9
#define RIGHT_SENSOR          16

//Led's
#define NUM_LEDS              1

#define TEENSY_LED            13

#define RUN_LED               0

//Strategy Switch
#define SWITCH_ONE             4
#define SWITCH_TWO             3
#define SWITCH_THREE           2
#define SWITCH_FOUR            1


#define ON_BUTTON              5
//#define STRATEGY_BUTTON      5

#define LEFT_MOTOR_IN1        18  
#define LEFT_MOTOR_IN2        19  
#define LEFT_MOTOR_PWM        21  

#define RIGHT_MOTOR_IN1       15  
#define RIGHT_MOTOR_IN2       14  
#define RIGHT_MOTOR_PWM       20  

#define PWM_SEEK              300       // se der ruim bota 200
#define PWM_SPIN              200       //nao esta sendo usado por enquanto
#define PWM_MAX               500       //pwm usado qnd ele enxerga com os 2 sensores

//** Time Constants **// 
#define REVERSE_TIME          200       
#define TURN_TIME_LIMIT1      600       //Time limit to Blind Turn
//#define TURN_TIME_LIMIT2      600       
//#define TURN_TIME_LIMIT3      600       
#define DEBOUNCING_TIME       200

unsigned short strategy;
unsigned short error;
unsigned short leftIR;
unsigned short rightIR;
unsigned long turnTime = 0;
unsigned long initialTimeTurn = 0;
boolean buttonState = true;            //change when the push button is pressed
boolean enemyFound = false;


void initRobot() {
  //Outputs
  pinMode(LEFT_MOTOR_IN1,OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  
  pinMode(RUN_LED, OUTPUT);
  pinMode(TEENSY_LED, OUTPUT);

  //Inputs
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(ON_BUTTON, INPUT_PULLUP);
  
  pinMode(SWITCH_ONE, INPUT);
  pinMode(SWITCH_TWO, INPUT);
  pinMode(SWITCH_THREE, INPUT);
  pinMode(SWITCH_FOUR, INPUT);
  
  digitalWriteFast(TEENSY_LED, HIGH);

  analogWriteResolution(9);
  analogWriteFrequency(LEFT_MOTOR_PWM, 50000); 

  Serial.begin(9600);

  #if defined (DEBUG_SERIAL)  || defined (DEBUG_IR_CONTROLLER)
    Serial.begin(9600); //At Teensy 3.1, the baud rate will be always 12Mbit/s so the 9600 value here got no effect.
  #endif
}


void rightWheelForward()
{
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW); 
}



void leftWheelForward()
{
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
}


void rightWheelBackward()
{
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH); 
}



void leftWheelBackward()
{
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
}


void readSensors()                                            //Read the sensors while the robot initiates with blind turn
{
  leftIR = digitalReadFast(LEFT_SENSOR);
  rightIR = digitalReadFast(RIGHT_SENSOR);   

  if (leftIR == 1 || rightIR == 1)
    enemyFound = true;
}


void blindTurn()
{
  turnTime = millis();
  delay(50);

  while (!enemyFound || (turnTime - millis() < 800))        //era 600
  {                                                                               
    rightWheelForward();
    leftWheelForward();
    analogWrite(RIGHT_MOTOR_PWM, 400);                    //400 
    analogWrite(LEFT_MOTOR_PWM, 100);                      //100
    readSensors();
  }

/*
   while (!enemyFound || (turnTime - initialTimeTurn < 400))
  {
    leftWheelForward();
    rightWheelBackward();
    analogWrite(RIGHT_MOTOR_PWM, 100);
    analogWrite(LEFT_MOTOR_PWM, 100);
    readSensors();
  }

  while (!enemyFound || (turnTime - initialTimeTurn < 400))
  {
    leftWheelForward();
    rightWheelForward();
    analogWrite(RIGHT_MOTOR_PWM, 100);
    analogWrite(LEFT_MOTOR_PWM, 100);
    readSensors();
  }
*/
}




void selectStrategy()                 
{
  strategy = (digitalRead(SWITCH_FOUR) + digitalRead(SWITCH_THREE)*2 +  digitalRead(SWITCH_TWO)*4 + digitalRead(SWITCH_ONE)*8);
  
  if(strategy == 0)                             //Turn right
  {
    rightWheelBackward();
    leftWheelForward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
  }
  
  else if(strategy == 1)                         //Turn left
  {
    rightWheelForward();
    leftWheelBackward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
  }
  
  else if(strategy == 3)                          //Blind turn to left
  {
    seekAndDestroy();
  }

  
  else if(strategy == 4)                          //Blind turn to left
  {
    blindTurn();
  }


else if(strategy == 5)                          //Blind turn to left
  {
    blindTurn();
  }


else if(strategy == 6)                          //Blind turn to left
  {
    blindTurn();
  }
/*
  else if(strategy == 4)
  {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
    delay(REVERSE_TIME);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW); 
  }
  else if(strategy == 5)
  {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH); 
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
    delay(10);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  }
  */
  
}


void seekAndDestroy()
{
  leftIR = digitalReadFast(LEFT_SENSOR);
  rightIR = digitalReadFast(RIGHT_SENSOR);  

 if (leftIR == 0 && rightIR == 1)
 {
    //Serial.println("1");
    rightWheelBackward();
    leftWheelForward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
 }
 
 else if(leftIR == 1 && rightIR == 0)
 {
    //Serial.println("2");
    rightWheelForward();
    leftWheelBackward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
 }
 
 else if(leftIR == 1 && rightIR == 1)
 {
    //Serial.println("3");
    rightWheelForward();
    leftWheelForward();
    analogWrite(LEFT_MOTOR_PWM, PWM_MAX);
    analogWrite(RIGHT_MOTOR_PWM, PWM_MAX);
 }
}


void verifyOnButton()
{
  if (digitalReadFast(ON_BUTTON) == LOW)
    buttonState = false;
}


void setup() 
{
  initRobot();    
  while(digitalReadFast(ON_BUTTON) != LOW)      //Wait start button
    Serial.println("Waiting Start Button");
  digitalWrite(RUN_LED, HIGH);                 
   delay(5000);                                //Initial delay
  selectStrategy();                            
}


void loop() 
{                          
 while(digitalReadFast(ON_BUTTON) != LOW)
 {
  seekAndDestroy();
 }  

 //digitalWrite(RUN_LED, LOW); 
  
 analogWrite(LEFT_MOTOR_PWM, 0);
 analogWrite(RIGHT_MOTOR_PWM, 0);
 buttonState = false;
  }
