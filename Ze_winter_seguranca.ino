
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

#define PWM_SEEK              511             // 400 para reducao 20:1      511 para reducao 50:1
#define PWM_SEEK_MINOR        160              // 90 para reducao 20:1      200 para reducao 50:1

//#define PWM_SPIN              200
#define PWM_MAX               511             // 500 para reducao 20:1      511 para reducao 50:1
#define REVERSE_TIME          200

#define DEBOUNCING_TIME       200

unsigned short strategia;
float error = 0;
unsigned short power;
unsigned short leftIR;
unsigned short rightIR;
bool btn_state = true;
float derivative;
float integral = 0;
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
  digitalWriteFast(RIGHT_MOTOR_IN1, HIGH);
  digitalWriteFast(RIGHT_MOTOR_IN2, LOW); 
}



void leftWheelForward()
{
  digitalWriteFast(LEFT_MOTOR_IN1, HIGH);
  digitalWriteFast(LEFT_MOTOR_IN2, LOW);
}


void rightWheelBackward()
{
  digitalWriteFast(RIGHT_MOTOR_IN1, LOW);
  digitalWriteFast(RIGHT_MOTOR_IN2, HIGH); 
}



void leftWheelBackward()
{
  digitalWriteFast(LEFT_MOTOR_IN1, LOW);
  digitalWriteFast(LEFT_MOTOR_IN2, HIGH);
}


void seekAndDestroy()
{
  leftIR = digitalReadFast(LEFT_SENSOR);
  rightIR = digitalReadFast(RIGHT_SENSOR);  

    //turnTime = millis();
   // delay(50);

   if (leftIR == 0 && rightIR == 1)
   {
      Serial.println("1");
      rightWheelForward();
      leftWheelForward();
      analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
      analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK_MINOR);   
        //readSensors();                                      
    }
   
   else if(leftIR == 1 && rightIR == 0)
   {
      Serial.println("2");
      rightWheelForward();
      leftWheelForward();
      analogWrite(LEFT_MOTOR_PWM, PWM_SEEK_MINOR);
      analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);       
        //readSensors();                                      
   }
   
   else if(leftIR == 1 && rightIR == 1)
   {
      rightWheelForward();
      leftWheelForward();
      analogWrite(LEFT_MOTOR_PWM, PWM_MAX);
      analogWrite(RIGHT_MOTOR_PWM, PWM_MAX);   
        //readSensors();                                      
    }
   else
 {
  Serial.println("4");
 }
}


void selectStrategy()                 
{
  strategia = (digitalReadFast(SWITCH_FOUR) + digitalReadFast(SWITCH_THREE)*2 +  digitalReadFast(SWITCH_TWO)*4 + digitalReadFast(SWITCH_ONE)*8);
  
  if (strategia == 0){                              //Giro no proprio eixo pra direita
    rightWheelBackward();
    leftWheelForward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
  }
  else if(strategia == 1)                           //Giro no proprio eixo pra esquerda
  {
    rightWheelForward();
    leftWheelBackward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
  }
  else if(strategia == 3)                            //Pequeno arco pra direita 
  {
    rightWheelForward();
    leftWheelForward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK_MINOR);
   
  }
  else if(strategia == 4)                              //Pequeno arco pra esquerda
  {
    
    rightWheelForward();
    leftWheelForward();
    analogWrite(LEFT_MOTOR_PWM, PWM_SEEK_MINOR);
    analogWrite(RIGHT_MOTOR_PWM, PWM_SEEK);
  }
  else if(strategia == 5)
  {
    seekAndDestroy();
  } 
}




void verifyOnButton()
{
  if (digitalReadFast(ON_BUTTON) == LOW)
  {
    delay(200);
    btn_state = !btn_state;
  }
}


  
void setup() 
{
  initRobot();      
}


void loop()
{
  while(btn_state)
  {
    verifyOnButton();
  }
  //Wait start button
  digitalWriteFast(RUN_LED,HIGH);                 //Initial delay
  delay(5000);
  selectStrategy();
  while(digitalReadFast(ON_BUTTON))
   seekAndDestroy();
 analogWrite(LEFT_MOTOR_PWM, 0);
 analogWrite(RIGHT_MOTOR_PWM, 0);
 btn_state = true;
 delay(500);
 digitalWriteFast(RUN_LED,LOW);                 
}
