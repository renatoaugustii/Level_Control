/*****************************************************
LEVEL CONTROL USING PID
CENTRO UNIVESITÁRIO UNA - BH/MG - BRAZIL 
AUTHOR: RENATO AUGUSTO
TEACHER ADVISOR: NAISSES ZOIA LIMA
DATE: 10/11/2018 
*****************************************************/
// DEFAULT LIBRARY TO USE ULTRASONIC
#include <Ultrasonic.h>
 
//DEFINE ULTRASONIC PINS 
#define pino_trigger 4
#define pino_echo 5
 
//START PINS 
Ultrasonic ultrasonic(pino_trigger, pino_echo);

/** DEFINE PORTS USED ON ARDUINO **/
const int out_MV = 9;// Port used to action resistence (PWM port)

/** PARAMETERS TO PID **/
float Kp = 19.35; // Gain 
float Ki = 3.77; // Integrator  
float Kd = 0; // Derivation

/** VARIABLES TO ANALOG FILTER **/
const int const_filter =1500 ;
unsigned long timer;
float last_Input_filtered = 0;
float last_timer =0;
float input =0;

/** VARIABLES USED TO MATH **/
const int Ta = 0.1; // Time using seconds (sample Time)
float last_I = 0;
float last_PV_filtered;
float PV_filtered; 
//float lastError = 0;
//float lastPV = 0;
float lastSP = 0;
float MV = 0;
float MV_manual = 0;
 float SetPoint = 50;// Define SetPoint on %

float action_P = 0; //Proportional Action 
float action_I = 0; //integration Action 
float action_D = 0;// Derivation Action 

/** LIMITS MAX AND MIN **/
int MV_MAX = 100 ;// Ouput MAX in %
int MV_MIN = 0;// Output MIN in %
int PV_max = 100; 
int PV_min = 0; 

/** MODE **/
bool MANUAL = false; // CHANGE MODES MANUAL TO AUTOMATIC

/** DIRECT OR REVERSE **/
bool DIRECT = true; // To use REVERSE set DIRECT = false

void setup() {
      Serial.begin(9600); // Starting Serial Communication
      pinMode(out_MV, OUTPUT);;// Start port OUTPUT  
    }

void loop() {

    timer = millis();
    float cmMsec, inMsec;
    long microsec = ultrasonic.timing();
    cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
    float input = ((13.5-cmMsec)/8.5)*100;
    
    float Ta_filter = timer - last_timer;
    
    float talf = const_filter;
    float alfa = talf/(Ta_filter+talf);
    float PV = alfa*last_Input_filtered+(1-alfa)*input; //Using the rectangular backward integration approach
    
    last_Input_filtered = PV;
    last_timer = timer;
  
  if (Serial.available()>0)
  { 
    char reading  = Serial.read();
        switch(reading){
            case 'm': //change mode automatic or manual
              MANUAL = !MANUAL;
              Serial.println(MANUAL);
            break;

            case'd': //change direct or reverse mode
                DIRECT = !DIRECT;
            break;
            
            case 'o': //change MV_manual
              reading = Serial.parseFloat();
              MV_manual = reading;
              Serial.println(MV_manual);
            break;

            case 's':
              reading = Serial.parseFloat();
              SetPoint = reading;
            break;
          }
      }

  /**************DIRECT OR REVERSE *********************/
  MODE_DIRECT(DIRECT); // If MODE_DIRECT = FALSE THE PROGRAM WILL USE REVERSE MODE
  /*****************************************************/
    
  float PV_N = (PV-PV_min)/(PV_max-PV_min)*100; //Normalized PV in 0 to 100%  (N = (value/SPAN)*100)
  float SP_N = (SetPoint-PV_min)/(PV_max-PV_min)*100; //Normalized SP in 0 to 100%

  /***************** DERIVATIVE FILTER NOT USED IN THIS PROGRAM************************/
  //float talf = Kd/10;// const timer to derivative filter
  //float alfa = talf/(Ta+talf);
  //PV_filtered = alfa*last_PV_filtered+(1-alfa)*PV_N; //Using the rectangular backward integration approach
  /************************************************************/
  
  float error = SP_N - PV_N; // error between SetPoint an PV
    
  action_P = Kp*error; //Proportional Action 
  action_D = Kd*((SetPoint-lastSP)-(PV_filtered - last_PV_filtered));// Derivation Action fixes "Derivate Kick"
    
  if(MANUAL){ // if MANUAL make the code below
              /********* MANUAL MODE *******************/
              action_I = MV_manual - action_P - action_D; //Reduce effects when change MANUAL to AUTOMATIC
              MV = MV_manual;
              AnalogOutput(MV); // Arduino Output
                PlotGraph(PV,SetPoint,MV); // crtl + L to plot graph using inteface from arduino
            }

            else /****** AUTOMATIC MODE ****************/
            { //if AUTOMATIC mode make the code below
              action_I = last_I + Ki*Ta*error; //integration Action 
               // Paralel PID model
              MV = action_P + action_I + action_D; // MV_out = PID
              
              //ANTI-WINDUP BELOW and SATURATION
                if (MV>MV_MAX)//
                          { action_I = 100;
                            MV = MV_MAX;
                          }//ANTI Windup 
                else if (MV<MV_MIN)
                          { action_I = 0;
                            MV = MV_MIN;
                          }//ANTI Windup 
                           
            AnalogOutput(MV);// call function to convert MV to Output PWM
            MV_manual = MV; //smooth transition between modes AUTOMATIC to MANUAL
            PlotGraph(PV,SetPoint,MV); // crtl + L to plot graph using inteface from arduino                    
            } //end if else
     
  last_I = action_I;
  last_PV_filtered = PV_filtered;
  lastSP = SetPoint; // Used in derivative kick
  //lasterror = error;
  //lastPV= PV;
  
}
/*************************************************************************************
                                    FUNCTIONS BELOW
**************************************************************************************/
      
void PlotGraph(float pv, float sp, float mv)
{ 
    Serial.print(pv); // Writing on monitor serial the variable PV
    Serial.print(" "); // Writing SetPointace on Screen
    Serial.print(sp); // Writing on monitor serial the variable PV
    Serial.print(" "); // Writing Space on Screen
    Serial.println(mv); // Writing on monitor Serial the variable MV
    delay(Ta*1000);// Sample time using seconds 
  }

// SET OUTPUT % TO PWM (ANALOG OUTPUT)
 void AnalogOutput(float mv)
 {
    float PWM_out = map(mv,0,100,0,255);//Map output 0 to 255
    analogWrite(out_MV,PWM_out); // Output PWM
  }

  /** CONTROLLER DIRECTION **/
void MODE_DIRECT(bool direct_control)//if direct_control = false the will use the reverse mode
{  
  if (!direct_control)
  {
    if (Kp>0 || Ki>0 || Kd>0)//if negative don't do nothing
      {
        Kp = 0 - Kp; // Invert Kp Direction  Kp = 0 - Kp
        Ki = 0 - Ki; // Invert Ki Direction  Ki = 0 - Ki
        Kd = 0 - Kd; // Invert Kd Direction  Kd = 0 - Kd
        Serial.println("REVERSE");
        Serial.println(Kp);
      }
  } 
  if(direct_control)
      {
        if (Kp<0 || Ki<0 || Kd<0)//if positive don't do nothing
          {
            Kp = 0 - Kp; // Invert Kp Direction  Kp = 0 - Kp
            Ki = 0 - Ki; // Invert Ki Direction  Ki = 0 - Ki
            Kd = 0 - Kd; // Invert Kd Direction  Kd = 0 - Kd
            Serial.println("DIRECT");
            Serial.println(Kp);
          }
      }
}     


