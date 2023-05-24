
/*******************************************************************************************************
 * Module Name  : ARDUINO MEGA CODE
 * Author       : TEAM 11 DESIGN OF MECHATRONICS SYSTEM 2 
 * Description  : Contains the CODE FOR the arduino Mega
 ******************************************************************************************************/
/*---------------------------------------------DATA types---------------------------------------------*/
typedef unsigned char             Pin_t;
typedef double                    RPM;
typedef unsigned char             Speed;
typedef unsigned long             counter_t;

typedef struct {
  Pin_t ENA;
  Pin_t IN1;
  Pin_t IN2;
  Pin_t ENCODER;
} Motor_t;

typedef struct {
  Pin_t DIR;
  Pin_t STEP;
} Stpr_t;

typedef struct{
  Pin_t TRIG;
  Pin_t ECHO;
}US_t;

/*---------------------------------------------Definitions---------------------------------------------*/
//for Test purposes only 
/*
 * NONE     : 0 
 * PID      : 1
 * STEPPER  : 2 
 * US       : 3
 */
#define TEST 2

//definitions needed for the stepper 
#define STEPS_PER_REV 800
#define MM_PER_REV (double(8))
#define STEPPERDELAY 250
#define REDUCTION 1

#define UPPERDISTANCE 45
#define LOWERDISTANCE 0

#define DOWNDIR   HIGH
#define UPDIR     LOW
//definitions needed for the motors
#define TICKS_PER_ROTATIONRR 400.0
#define TICKS_PER_ROTATIONRL 400.0
#define TICKS_PER_ROTATIONFR 400.0
#define TICKS_PER_ROTATIONFL 400.0

#define INTERRUPT_TIME 20
#define MAX_RPM 250
#define SET_RPM_RR 200
#define SET_RPM_RL 200
#define SET_RPM_FR 200
#define SET_RPM_FL 200

#define MIN_RPM 0
#define FW 1
#define BK 0
//definitions needed for the PID control
#define MAX_CONTROL  (255)
#define MIN_CONTROL  (0)

#define KP 0.01
#define KI 0.01
#define KD 0.15
//minimum distance to objects acording to ultrasonic

#define THRESHOLD (300)

/*---------------------------------------------HARDWARE SETUP---------------------------------------------*/
// static to prevent changes after initializations
static Pin_t Limit_Switch =  20;

static US_t RS            = {4 ,5 };
static US_t LS            = {7 ,6 };

static Stpr_t STPR        = {19  ,13 };

static Motor_t MRR        = {11 ,37, 36, 3  };
static Motor_t MRL        = {12 ,31, 30, 2  };
static Motor_t MFR        = {9  ,32, 33, 20 };
static Motor_t MFL        = {10 ,34, 35, 18 };

/*---------------------------------------------VARIABLES---------------------------------------------*/
long milliseconds = 0 ;

volatile counter_t RR, RL, FR, FL;

RPM SET_POINT_RR ,
    SET_POINT_RL ,
    ACTUAL_RR,
    ACTUAL_RL,
    SET_POINT_FR ,
    SET_POINT_FL ,
    ACTUAL_FR,
    ACTUAL_FL;
    
double  DISTANCE = 0;
unsigned int  USR = 0,
              USL = 0;                      //mm
bool updown = 0 ;                     //up = 1 , down = 0

int TErrorRR,TErrorRL,TErrorFR,TErrorFL;
int LErrorRR,LErrorRL,LErrorFR,LErrorFL;
int ContSRR,ContSRL,ContSFR,ContSFL;
/*---------------------------------------------ENCODER INTERRUPTS---------------------------------------------*/
void EncoderRR(void) { RR++; }
void EncoderRL(void) { RL++; }
void EncoderFR(void) { FR++; }
void EncoderFL(void) { FL++; }
/*---------------------------------------------FUNCTION PROTOTYPES---------------------------------------------*/
void Motor_STOP(void);
void Stepper_MOVE(void);
void Motor_INIT(Motor_t, void (void));
void Stepper_INIT(Stpr_t,Pin_t);
void UltraSonic_INIT(US_t);
void UltraSonic_MEASURE(US_t ,double*);
void TIMER_ROUTINE(void);
void move_me(bool,bool);
int PID_Control(int ,int ,int* ,int* ,int* );
void looping();
/*---------------------------------------------SETUP FUNCTION---------------------------------------------*/
/******************************************************************
 * Function Name   : setup
 * Inputs          : None
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : calls all Setup funtuins
 ******************************************************************/

void setup() {
  
  Serial.begin(19200);
  //Serial.println("lol");
  
  Motor_INIT(MRR, EncoderRR);
  Motor_INIT(MRL, EncoderRL);
  Motor_INIT(MFR, EncoderFR);
  Motor_INIT(MFL, EncoderFL);
  Motor_STOP();
  
  Stepper_INIT(STPR,Limit_Switch);
  
  UltraSonic_INIT(RS);
  UltraSonic_INIT(LS);
  
  milliseconds = millis();
  //while(1) looping();
}

/*---------------------------------------------LOOP FUNCTION---------------------------------------------*/
/******************************************************************
 * Function Name   : loop
 * Inputs          : None
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : repeats until the program terminates
 ******************************************************************/
void loop() {
     if ((millis()-milliseconds)>= INTERRUPT_TIME)
    {
      TIMER_ROUTINE();
      milliseconds = millis();
    }
    #if (TEST==2)
updown=1;
    Stepper_MOVE();

    delay(2000);
    
updown=0;
    Stepper_MOVE();
    delay(2000);
    #endif

}

/*---------------------------------------------SERIAL INTERRUPT FUNCTION---------------------------------------------*/
/******************************************************************
 * Function Name   : serialEvent
 * Inputs          : None
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Asynchronous
 * Description     : gets called once UART recieves something 
 ******************************************************************/
void serialEvent() {
  String var;
  var = Serial.readString();

  #if (TEST==0)
    if (var[4]-'0') Motor_STOP();
    else 
    {
      unsigned char temp = (var[0]-'0')|((var[1]-'0')<<1)|((var[2]-'0')<<2);
      
      switch(temp)
      {
      case 0: move_me(BK,BK); break;//straight  backward
      case 1: move_me(FW,FW); break;//straight  forward
      case 2: move_me(FW,BK); break;//right     backward
      case 3: move_me(BK,FW); break;//right     forward
      case 4: move_me(BK,FW); break;//left      backward
      case 5: move_me(FW,BK); break;//left      forward
      
      case 6: 
      case 7: 
      default: 
              Motor_STOP();//those 3 states shouldn't exist 
      }
  
      
    }
    
    updown = (var[3]-'0');
  #elif (TEST==1)
    counter_t ccc = 0;
    int tens = 1, KRR = 0;
  
    while (var[ccc] != '.')
    {
      KRR = int(var[ccc] - '0') + KRR * tens;
      tens *= 10;
      ccc++;
    }
    
    SET_POINT_RR = KRR*250/100;
    SET_POINT_RL = KRR*250/100;
    SET_POINT_FR = KRR*250/100;
    SET_POINT_FL = KRR*250/100;
  #elif (TEST ==2)
   updown = (var[0]-'0');
  #endif
}


/*---------------------------------------------MOTOR FUNCTIONS---------------------------------------------*/

/******************************************************************
 * Function Name   : Motor_INIT
 * Inputs          : Motor Structure , and pointer to function
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Setup Motor Pins
 ******************************************************************/
void Motor_INIT(Motor_t X, void FUN(void))
{
  pinMode(X.ENA, OUTPUT);
  pinMode(X.IN1, OUTPUT);
  pinMode(X.IN2, OUTPUT);

  pinMode(X.ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(int(X.ENCODER)), FUN, RISING);
}

/******************************************************************
 * Function Name   : Motor_STOP
 * Inputs          : None
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : stops all motors
 ******************************************************************/
void Motor_STOP(void) {
  digitalWrite(MRR.IN1, 0);
  digitalWrite(MRR.IN2, 0);

  analogWrite(MRR.ENA, 0);

  digitalWrite(MRL.IN1, 0);
  digitalWrite(MRL.IN2, 0);

  analogWrite(MRL.ENA, 0);

  digitalWrite(MFR.IN1, 0);
  digitalWrite(MFR.IN2, 0);

  analogWrite(MFR.ENA, 0);

  digitalWrite(MFL.IN1, 0);
  digitalWrite(MFL.IN2, 0);

  analogWrite(MFL.ENA, 0);
}
/******************************************************************
 * Function Name   : move_me
 * Inputs          : direction set for Right and Left
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : sets the direction for all motors
 ******************************************************************/
void move_me(bool dir_R,bool dir_L) {
  SET_POINT_RR = SET_RPM_RR;
  SET_POINT_RL = SET_RPM_RL;
  SET_POINT_FR = SET_RPM_FR;
  SET_POINT_FL = SET_RPM_FL;
  
  digitalWrite(MRR.IN1, dir_R);
  digitalWrite(MRR.IN2, !dir_R);
  
  digitalWrite(MRL.IN1, dir_L);
  digitalWrite(MRL.IN2, !dir_L);
  
  digitalWrite(MFR.IN1, dir_R);
  digitalWrite(MFR.IN2, !dir_R);

  digitalWrite(MFL.IN1, dir_L);
  digitalWrite(MFL.IN2, !dir_L);

}

/*---------------------------------------------STEPPER FUNCTIONS---------------------------------------------*/
/******************************************************************
 * Function Name   : Stepper_INIT
 * Inputs          : Stepper Structure, LimitSwitch pin
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Setup the pins for the stepper and its limitSwitch
 *                   then moves it to it's 0 position
 ******************************************************************/
void Stepper_INIT(Stpr_t x,Pin_t limitswitch){
  pinMode(x.DIR,OUTPUT);
  pinMode(x.STEP,OUTPUT);

#if (TEST==0)
  pinMode(limitswitch,INPUT);
  Stepper_DOWN();
  
  while(!digitalRead(limitswitch))
  {
    Serial.println("lol");
    Stepper_REV();
  }
 #endif
  }
/******************************************************************
 * Function Name   : Stepper_DOWN
 * Inputs          : Non
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Set direction pin for the stepper to down
 ******************************************************************/
void Stepper_DOWN(void)
{
  digitalWrite(STPR.DIR, DOWNDIR);
}
/******************************************************************
 * Function Name   : Stepper_UP
 * Inputs          : Non
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Set direction pin for the stepper to up
 ******************************************************************/
void Stepper_UP(void)
{
  digitalWrite(STPR.DIR, UPDIR);
}
/******************************************************************
 * Function Name   : Stepper_REV
 * Inputs          : Non
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : makes the stepper take 1 revolution
 ******************************************************************/
void Stepper_REV(void)
{ for (int i = 0 ; i <= STEPS_PER_REV/REDUCTION ; i++) {
    digitalWrite(STPR.STEP, HIGH);
    delayMicroseconds(STEPPERDELAY);
    digitalWrite(STPR.STEP, LOW);
    delayMicroseconds(STEPPERDELAY);
  }
}
/******************************************************************
 * Function Name   : Stepper_MOVE
 * Inputs          : Non
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Handles the stepper operations
 ******************************************************************/
void Stepper_MOVE(void) {
  if (updown )
  {
    while((DISTANCE < UPPERDISTANCE))
    {
    Stepper_UP();
    Stepper_REV();
    DISTANCE += MM_PER_REV/REDUCTION;
    }
  }
  else if (!updown )
  {
    while((DISTANCE > LOWERDISTANCE) && (!digitalRead(Limit_Switch)))
    {
    Stepper_DOWN();
    Stepper_REV();
    DISTANCE -= MM_PER_REV/REDUCTION;
    }
  }
  else
  {
    //do nothing
  }
}

/*---------------------------------------------ULTRASONIC FUNCTIONS---------------------------------------------*/
/******************************************************************
 * Function Name   : UltraSonic_INIT
 * Inputs          : Ultrasonic Structure
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Setup the pins for the ultrasonic
 ******************************************************************/
void UltraSonic_INIT(US_t x){
    pinMode(x.TRIG,OUTPUT);
    pinMode(x.ECHO,INPUT);
  }
/******************************************************************
 * Function Name   : UltraSonic_MEASURE
 * Inputs          : Ultrasonic Structure , pointer to double
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : calculates the distance UltraSonic_MEASUREd by the ultrasonic
 ******************************************************************/
void UltraSonic_MEASURE(US_t x,unsigned int* dis){

*dis = !digitalRead(x.ECHO);
}
/*---------------------------------------------PID FUNCTIONS---------------------------------------------*/
/******************************************************************
 * Function Name   : PID_Control
 * Inputs          : Setpoint Value 
 *                   Sensed output value
 *                   pointer to total error
 *                   pointer to last calculated error
 *                   pointer to last control signal used
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Calculates the output signal needed to negate the error
 ******************************************************************/
int PID_Control(int setpoint,int sensed_output,int* total_error,int* last_error,int* oldcontrolsignal) {

  
    int error = (setpoint) - sensed_output;
    int derror = error - *last_error;
    
    *total_error += error;
   
    int temp = *oldcontrolsignal+ KP*error+KD*derror+KI* (*total_error);
    if(temp>MAX_CONTROL) temp=MAX_CONTROL;
    else if(temp<MIN_CONTROL) temp=MIN_CONTROL;
    *last_error = error;
    *oldcontrolsignal = temp;
  return temp;
}
/*---------------------------------------------TIMER ROUTINE FUNCTIONS---------------------------------------------*/
int limit(int x)
{
  if(x>MAX_RPM) x= MAX_RPM;
  if(x<MIN_RPM) x =MIN_RPM;
  return x; 
}
/******************************************************************
 * Function Name   : TIMER_ROUTINE
 * Inputs          : None
 * Outputs         : None
 * Reentrancy      : Reentrant
 * Synchronous     : Synchronous
 * Description     : Sets values of the actual speeds acording to encoder Readings
 *                   calls the distance UltraSonic_MEASUREd by the ultrasonic sensor
 *                   Moves the stepper
 *                   Sends the Output values through UART
 ******************************************************************/
void TIMER_ROUTINE(void) {         // interrupt service routine


  #if (TEST==0)
  
  ACTUAL_RR = limit((((RR) *1.0/ TICKS_PER_ROTATIONRR) * 60*1000/INTERRUPT_TIME));
  ACTUAL_RL = limit((((RL) *1.0/ TICKS_PER_ROTATIONRL) * 60*1000/INTERRUPT_TIME));
  ACTUAL_FR = limit((((FR) *1.0/ TICKS_PER_ROTATIONFR) * 60*1000/INTERRUPT_TIME));
  ACTUAL_FL = limit((((FL) *1.0/ TICKS_PER_ROTATIONFL) * 60*1000/INTERRUPT_TIME));
  RR = 0 ; RL = 0; FR = 0; FL = 0;
  
  UltraSonic_MEASURE(LS,&USL);
  UltraSonic_MEASURE(RS,&USR);
  
  Stepper_MOVE();

  analogWrite(MRR.ENA,PID_Control(SET_POINT_RR,ACTUAL_RR,&TErrorRR,&LErrorRR,&ContSRR));
  analogWrite(MRL.ENA,PID_Control(SET_POINT_RL,ACTUAL_RL,&TErrorRL,&LErrorRL,&ContSRL));
  analogWrite(MFR.ENA,PID_Control(SET_POINT_FR,ACTUAL_FR,&TErrorFR,&LErrorFR,&ContSFR));
  analogWrite(MFL.ENA,PID_Control(SET_POINT_FL,ACTUAL_FL,&TErrorFL,&LErrorFL,&ContSFL));
  
  Serial.print(int(ACTUAL_RR));
  Serial.print(",");
  Serial.print(int(ACTUAL_RL));
  Serial.print(",");
  Serial.print(int(ACTUAL_FR));
  Serial.print(",");
  Serial.print(int(ACTUAL_FL));
  Serial.print(",");
  Serial.print(int(DISTANCE));
  Serial.print(",");
  Serial.print(int(USR));
  Serial.print(",");
  Serial.println(int(USL));
  
  #elif(TEST==1)
  
  ACTUAL_RR = limit((((RR) *1.0/ TICKS_PER_ROTATIONRR) * 60*1000/INTERRUPT_TIME));
  ACTUAL_RL = limit((((RL) *1.0/ TICKS_PER_ROTATIONRL) * 60*1000/INTERRUPT_TIME));
  ACTUAL_FR = limit((((FR) *1.0/ TICKS_PER_ROTATIONFR) * 60*1000/INTERRUPT_TIME));
  ACTUAL_FL = limit((((FL) *1.0/ TICKS_PER_ROTATIONFL) * 60*1000/INTERRUPT_TIME));
  RR = 0 ; RL = 0; FR = 0; FL = 0;

  analogWrite(MRR.ENA,PID_Control(SET_POINT_RR,ACTUAL_RR,&TErrorRR,&LErrorRR,&ContSRR));
  analogWrite(MRL.ENA,PID_Control(SET_POINT_RL,ACTUAL_RL,&TErrorRL,&LErrorRL,&ContSRL));
  analogWrite(MFR.ENA,PID_Control(SET_POINT_FR,ACTUAL_FR,&TErrorFR,&LErrorFR,&ContSFR));
  analogWrite(MFL.ENA,PID_Control(SET_POINT_FL,ACTUAL_FL,&TErrorFL,&LErrorFL,&ContSFL));
  
  Serial.print(int(ACTUAL_RR));
  Serial.print(",");
  Serial.print(int(ACTUAL_RL));
  Serial.print(",");
  Serial.print(int(ACTUAL_FR));
  Serial.print(",");
  Serial.print(int(ACTUAL_FL));
  Serial.print(",");
  Serial.println(int(SET_POINT_RL));
  
  #elif(TEST==2)
  
  Stepper_MOVE();
  Serial.println(int(DISTANCE));
  
  #elif(TEST==3)
  
  UltraSonic_MEASURE(LS,&USL);
  UltraSonic_MEASURE(RS,&USR);
  Serial.print(int(USR));
  Serial.print(",");
  Serial.println(int(USL));
  
  #endif
}
