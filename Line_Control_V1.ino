// V 1.0 DEVELOPED BY: HAROLD F. MURCIA -> OCTUBRE 26 de 2015
//
// This code was developed for Line follower Robots with a differential topology. 
// The project must be compatible with Pololu sensors and motors, the system is be 
// an Arduino board or a BabyOrangutan board.
//
//The Control is a simple PD, with a initial tune for a regular performance. 
// To improve the performance the control parameters must be re-tuned.

// Is important to config the number of sensors, the sensor pins and the type of boar (Analog or Digital)

//////////////////////////////////////////////////////////////////////////////////


#include <QTRSensors.h>
#include <OrangutanMotors.h>  // For BabyOrangutan

//////////////////////////////////////////////// VARIABLES

#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_EMITTERS_OFF  // emitter off
#define TIMEOUT       2500  //

#define LED                   9 // Enable driver
#define Buttom                10 // Enable driver
#define Motor_Up_Limit        255 // Upper motor Limit
#define Motor_Low_Limit       -50 //Lowesr motor limit
#define wait_time             1000 // Time in milliSeconds before RUN


// TIPE BOARD
String Robot="BABY";                  // PUT ARDUINO or BABY as appropriate
String S_Board="ANALOG";                 // PUT ANALOG or DIGITAL as appropriate


// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
// Comment for digital Pololu sensor board
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);


// Comment for Analog Pololu sensor board
//QTRSensorsRC qtrrc((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7},
//NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motors
OrangutanMotors motors;

// TIME VARIABLES
float Ts=2000;                            // Sample Time                 
float tic=0,toc=0,t_elapsed=0;            // Time measurement


// CONTROL VARIABLES
float Kp=0.05, Td=0.016, Up=0, Ud=0, U=0, UL=0, UR=0;
unsigned int position=2500;
float error=0,error_1=0;
float MX_Value=70.0,MN_Value=40.0, Max=0;

// CALIBRATION VARIABLES
float velcalibrate=20;   // Power on motors to auto - calibration




////////////////////////////////////////////// FUNCTIONS


void setup(){
  
  int val = 0;  // Buttom initial
  val = digitalRead(Buttom);
  
  if(Robot=="BABY"){
    motors.setSpeeds(0,0);// Motores detenidos  
  }else{
    ;
  }
  
  // Press to calibrate
  digitalWrite(LED, HIGH);
  while (val == HIGH){
   digitalWrite(LED, LOW);
   val = digitalRead(Buttom);
  }
   
//-------------Start Calibrating Sensors--------------------------------------//
  
  delay(1500); 
  digitalWrite(LED, HIGH);// Enciende el leds para indicar que se esta calibrando.
    for (int counter=0; counter<48; counter++){ 
    if (counter < 8 || counter >= 15)
      OrangutanMotors::setSpeeds(-velcalibrate, velcalibrate);
    else
      OrangutanMotors::setSpeeds(velcalibrate, -velcalibrate);
      if(S_Board=="ANALOG"){
        qtra.calibrate(); 
      }else{
      //  qtrc.calibrate(); 
      }
   delay(20);
  }
 
  digitalWrite(LED, LOW);     // Apaga el led para indicar que se termino la calibracion.

  if(Robot=="BABY"){
    OrangutanMotors::setSpeeds(0, 0);
  }else{
    ;
  }
  
  //---------------------------Fin Calibracion de Sensores----------------------------------------------------//
  pinMode(Buttom,INPUT);
  
  // Press to wait and run
  val = digitalRead(Buttom);
  while (val == HIGH){
   digitalWrite(LED, LOW);
   val = digitalRead(Buttom);
  }
  digitalWrite(LED, HIGH);
  delay(wait_time); // Retardo X segundos antes de Empezar a andar  
  digitalWrite(LED, LOW);
}











void loop(){
  
  tic=micros();

  if(S_Board=="ANALOG") {
    position = qtra.readLine(sensorValues);
  }else{
//    position = qtrrc.readLine(sensorValues);
  }

  error=2500.0 -(float)position;   // Calculate of error
  Up=error*Kp;                   // Proportional
  Ud=Kp*Td/(Ts/1000000.0)*(error-error_1);  //Control Action
  U=Up+Ud;                                  //Derivative Action
  error_1=error;
  
  Max=((-MX_Value + MN_Value)/2500.0)*abs(error) + MX_Value;  // speed tunnel
  
  UR= Max-U;  // Right Action
  UL= Max+U;  // Left Action
  
  if(abs(error)<50.0){
    digitalWrite(LED, HIGH);
  }else{
    digitalWrite(LED, LOW);
  }
  
  // Right Upper Limit
  if(UR>=Motor_Up_Limit){UR=Motor_Up_Limit;}
  
  // LEFT Upper Limit
  if(UL>=Motor_Up_Limit){UL=Motor_Up_Limit;}
  
  // Right Lower Limit
  if(UR<=Motor_Low_Limit){UR=Motor_Low_Limit;}
  
  // LEFT Lower Limit
  if(UL<=Motor_Low_Limit){UL=Motor_Low_Limit;}
  
  if(Robot=="BABY"){
    OrangutanMotors::setSpeeds(UL, UR);
  }else{
    ;
  }
  
  toc=micros();
  t_elapsed=(toc-tic);
    
  while(t_elapsed<Ts){
    toc=micros();
    t_elapsed=(toc-tic);
   }

}; 
