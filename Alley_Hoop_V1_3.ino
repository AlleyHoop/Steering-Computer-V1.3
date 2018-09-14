//******************************************************************//
// Name:            ALley Hoop software V1.0                       //
// Creator:         Cyriel Swankaert                              //
// Date created:    30/04/2018                                   //
// Created for:     PCB Alley Hoop V1.0                         //
//*************************************************************//
// debugs en aangepast door John Seiffers
//// Include-files
#include "mcp_can.h"         
// Dus is het met polling
#include <SPI.h>

/*********************************************************************************************************
  Pindefinitions "Alley Hoop PCB v1.0"
*********************************************************************************************************/
/// CAN-BUS
#define CAN0_INT 4                      // Set INT to pin 4
MCP_CAN CAN0(47);                       // Set CS to pin 47
#define CAN1_INT 5                      // Set INT to pin 5
MCP_CAN CAN1(49);                       // Set CS to pin 49

/// Analog
const int ai_steer_pot = A0;            //analog pin A0, potentiometer of the steering system
const int ai_brake_pressure = A1;       //analog pin A1, pressure sensor of the braking system
const int ai_hmi_gas = A2;              //analog pin A2, gas signal of the joystick
const int ai_hmi_brake = A3;            //analog pin A3, brake signal of the joystick
const int ai_hmi_steering = A4;         //analog pin A4, steering signal of the joystick

/// PWM
const int pwm_steer_lpwm = 2;           //digital pin2, pwm signal to control the speed of the steering motor in the left direction
const int pwm_steer_rpwm = 3;           //digital pin3, pwm signal to control the speed if the steering motor in the right direction
const int pwm_brake_pump = 7;           //digital pin7, pwm signal to control the speed of the braking pump
const int pwm_drive_throttle = 8;       //digital pin8, throttle signal that will control the engine
const int di_remote_gas = 9;            //digital pin9, gas signal of the remoteote controller (also brake)
const int di_remote_steering = 10;      //digital pin10, steer signal of the remoteote controller
const int di_remote_switchfr = 11;      //digital pin11, switch to choose between forward en reverse drving of the remoteote
const int pwm_laadpaal_pulse = 12;      //digital pin12,

/// Digital
const int do_steer_r_en = 22 ;          //digital pin22, enable for the right direction of the steering motor
const int do_steer_l_en = 23;           //digital pin23, enable for the left direction of the steering motor
const int do_brake1_enable = 24;         //digital pin24, enable for the braking pump
const int do_brake2_enable = 25;         //digital pin24, enable for the braking pump
const int do_drive_enable = 26;         //digital pin26, enable for the engine control system
const int do_drive_forward = 27;        //digital pin27, forward enbale for the engine control system
const int do_drive_reverse = 28;        //digital pin28, reverse enable for the engine control system
const int do_drive_brake = 29;          //digital pin29, brake signal for the engine control sytem
const int do_drive_throttleswitch = 30; //digital pin30, signal that needs to be high for the trottle to work of hte engine control system
const int di_keysignaal = 31;           //digital pin31, input that lets the system know it is on because of the key
const int di_gnd_lader = 32;            //digital pin32, input that lets the system know it is on because of the charger
const int do_hv_relais = 33;            //digital pin33, if High HV-relais will turn on
const int do_verbinding = 34;           //digital pin34, if High the system will keep itself on
const int do_pilotswitch = 35;          //digital pin35, output to speak with the charging system
const int di_hmi_switchjoystick = 39;   //digital pin39, mode select of HMI: joystick
const int di_hmi_switchremote = 40;     //digital pin40, mode select of HMI: remoteote
const int di_hmi_switchauto = 41;       //digital pin41, mode select of HMI: auto
const int di_hmi_switchfr = 42;         //digital pin42, switch to choose between forward and reverse drive of the HMI
const int di_hmi_switchstop = 43;       //digital pin43, switch to make a safe stop of the HMI

/*********************************************************************************************************
  Variables
*********************************************************************************************************/
/// Global variables
unsigned long time;
bool  mode_charge = false;      //High when the charging plug is plugged in
bool  mode_drive = false;       //High when the charging plug is not plugged in and the key switch is on
bool  mode_joystick = false;    //High when the mode select switch of the HMI is on joystick
bool  mode_remote = false;      //High when the mode select switch of the HMI is on remoteote
bool  mode_auto = false;        //High when the mode select switch of the HMI is on auto
bool  charging = false;  //High when last mode was charging

///Drive variables
int   engine_sp = 0;                  //set point of the engine
int   brake_sp = 0;                   //set point of the brakes
long  steering_sp = 0;                //set point of the steering
int   engine_pv = 0;                  //process value of the engine
int   brake_pv = 0;                   //process value of the brakes
int   steering_pv = 0;                //process value of the steering
int   engine_delta = 0;               //delta of the engine (DELTA = SP-PV)
int   brake_delta = 0;                //delta of the brakes
long  steering_delta = 0;             //delta of the steering
int   engine_op = 0;                  //ouput value of the engine (OP = DELTA * Kp)
int   brake_op = 0;                   //output value of the brakes
long  steering_op = 0;                //output value of the steering
bool  mode_forward = false;           //High when the driver asks to drive forward
bool  mode_reverse = false;           //High when the driver asks to drive reverse

///Parameters - set these accordingly
//Engine parameters
int engine_joystick_offset = 510;     //offset for the engine sp from the joystick in %
int engine_DB = 5;                  //dead band of the engine in %
int engine_kp = 1;                  //Proportional gain of the engine

//Brake parameters
int brake_pv_offset = 215;          //offset for the brake pv in bar
int brake_DB = 5;                   //dead band of the brakes in bar
int brake_kp = 1;                   //Proportional gain of the brakes

//Steering parameters
int steering_joystick_offset = 508; //offset for the steering sp from the joystick in °
int steering_rasp_offset = 128;     //offset for the steering sp from the Raspberry Pi in °
int steering_pv_offset = 512;       //offset for the steering pv in °
int steering_DB = 5;                //dead band of the steering in °
int steering_kp = 70;                //Proportional gain of the steering

/*Charger parameters
  const int CHARGER_BAUD_RATE = 250;*/

/// CAN variables
long unsigned int   rxId_CAN0;
long unsigned int   rxId_CAN1;
unsigned char       len_CAN0 = 0;
unsigned char       len_CAN1 = 0;
unsigned char       rxBuf_CAN0[8];
unsigned char       rxBuf_CAN1[8];
char                msgsteering[128];             // Array to store serial steering
bool                failcan0 = false;             // ZOU BETER BENUT KUNNEN WORDEN. WORDT NU ALLEEN WAARGENOMEN IN SETUP. MISSCHIEN NIET VERDER OPSTARTEN OID
bool                failcan1 = false;             // ZOU BETER BENUT KUNNEN WORDEN. WORDT NU ALLEEN WAARGENOMEN IN SETUP. MISSCHIEN NIET VERDER OPSTARTEN OID
const unsigned long CAN_ID_Charger = 0x1806E5F4;  //CAN ID of the charger
const unsigned long CAN_ID_Curtis = 0x227;        //CAN ID of the curtis controller
const unsigned long CAN_ID_RASP = 0x0;            //CAN ID of the Raspberry Pi

//Curtis CAN
int low_voltage = 0;
int engine_rpm = 0;
int engine_temp = 0;
int control_emp = 0;
int high_voltage = 0;

//Raspberry Pi CAN
bool rasp_switchfr = false;
int  rasp_gas = 0;
int  rasp_brake = 0;
int  rasp_steering = 0;

/*Charger variables
  int charger_voltage = 0;    //Reported voltage on battery
  int charger_current = 0;    //Reported current
  int charger_status = 0;
  int charging_state = 0;
  boolean charger_error = false;
  int set_voltage = 0;        //Desired voltage
  int set_current = 0;        //Desired max current
  byte output_inhibit = 1;
  steering status_steering = "";*/

void setup() {

  Serial.begin(57600);                  // For debug messages
  Serial.println("Alley Hoop V1.0");

  //// Initialize MCP2515 running at 16MHz with a baudrate of 250kb/s
  // CAN 0 is the CANBUS for the Engine Control System
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 1 Initialized Successfully!");
  }
  else {
    failcan0 = true;
    Serial.println("Error Initializing MCP2515 2...");
  }

  // CAN1 is the CANBUS of the car (BMS, , HMI, RASP)
  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 2 Initialized Successfully!");
  }
  else {
    failcan1 = true;
    Serial.println("Error Initializing MCP2515 2...");
  }

  //Set the CAN controllers in mode normal
  CAN0.setMode(MCP_NORMAL);
  CAN1.setMode(MCP_NORMAL);


  ////IO-pin's settings
  pinMode(CAN0_INT,               INPUT_PULLUP);
  pinMode(CAN1_INT,               INPUT_PULLUP);
  pinMode(pwm_steer_lpwm,         OUTPUT);
  pinMode(pwm_steer_rpwm,         OUTPUT);
  pinMode(pwm_brake_pump,         OUTPUT);
  pinMode(pwm_drive_throttle,     OUTPUT);
  pinMode(di_remote_gas,          INPUT);
  pinMode(di_remote_steering,     INPUT);
  pinMode(di_remote_switchfr,     INPUT);
  pinMode(pwm_laadpaal_pulse,     OUTPUT);
  pinMode(do_steer_r_en,          OUTPUT);
  pinMode(do_steer_l_en,          OUTPUT);
  pinMode(do_brake1_enable,        OUTPUT);
  pinMode(do_brake2_enable,        OUTPUT);
  pinMode(do_drive_enable,        OUTPUT);
  pinMode(do_drive_forward,       OUTPUT);
  pinMode(do_drive_reverse,       OUTPUT);
  pinMode(do_drive_brake,         OUTPUT);
  pinMode(do_drive_throttleswitch, OUTPUT);
  pinMode(di_keysignaal,          INPUT);
  pinMode(di_gnd_lader,           INPUT);
  pinMode(do_hv_relais,           OUTPUT);
  pinMode(do_verbinding,          OUTPUT);
  pinMode(do_pilotswitch,         OUTPUT);
  pinMode(di_hmi_switchjoystick,  INPUT);
  pinMode(di_hmi_switchremote, INPUT);
  pinMode(di_hmi_switchauto,      INPUT);
  pinMode(di_hmi_switchfr,        INPUT);
  pinMode(di_hmi_switchstop,      INPUT);

  ////Turning on the relays
  digitalWrite(do_hv_relais,      HIGH);
  digitalWrite(do_verbinding,     HIGH);

  ///Detect in wich mode the car is and set variables accordinaly
  // Detect if the car is in joystick mode
  if (digitalRead(di_hmi_switchjoystick) == true) {
    mode_joystick = true;
    mode_remote = false;
    mode_auto = false;
    Serial.println("*****************************************************************");
    Serial.println("***********       Alle Hoop = Joystick Mode     *****************");
    Serial.println("*****************************************************************");
  }

  // Detect if the car is in remoteote mode
  else if (digitalRead(di_hmi_switchremote) == true) {
    mode_joystick = false;
    mode_remote = true;
    mode_auto = false;
    Serial.println("*****************************************************************");
    Serial.println("***********       Alle Hoop = Remote Mode       *****************");
    Serial.println("*****************************************************************");

  }

  // Detect if the car is in auto mode
  else if (digitalRead(di_hmi_switchauto) == true) {
    mode_joystick = false;
    mode_remote = false;
    mode_auto = true;
    Serial.println("*****************************************************************");
    Serial.println("***********       Alle Hoop = Autonoom Mode     *****************");
    Serial.println("*****************************************************************");

  }
  Serial.println("init done");
}

/*********************************************************************************************************
** Function name:           checkCAN_curtis
** Descriptions:            check for 50ms for curtis messages
*********************************************************************************************************/
// Mogelijke verbeteringen: check welk id niet wordt beantwoord
void checkCAN_curtis() {
  unsigned long time;
  time = millis();
  while (time + 50 > millis()) {
    if (digitalRead(CAN0_INT) == 0) {
      CAN0_interrupt_curtis();
    }
  }
}

/*********************************************************************************************************
** Function name:           checkCAN_rasp
** Descriptions:            check for ...ms for charger messages
*********************************************************************************************************/
// Mogelijke verbeteringen: check welk id niet wordt beantwoord
void checkCAN_rasp() {
  unsigned long time;
  time = millis();
  while (time + 50 > millis()) {
    if (digitalRead(CAN1_INT) == 0) {
      CAN1_interrupt_rasp();
    }
  }
}

/*********************************************************************************************************
** Function name:           CAN0_interupt_curtis
** Descriptions:            read data of CAN0 (engine control system)
*********************************************************************************************************/
void CAN0_interrupt_curtis(void) {
  // Messages of the CURTIS
  CAN0.readMsgBuf(&rxId_CAN0, &len_CAN0, rxBuf_CAN0);      // Read data: len = data length, buf = data byte(s)
  sprintf(msgsteering, "Extended ID: 0x%.8lX  DLC: %1d", (rxId_CAN0 & 0x1FFFFFFF), len_CAN0);
  Serial.println(msgsteering);
  Serial.print ("                                                    rxId_CAN0 =  ");
  Serial.println(rxId_CAN0);

  if (rxId_CAN0 & 0x40000000) {
    return; // remoteote request data is not important
  }
  else if (rxId_CAN0 == CAN_ID_Curtis) {
    low_voltage = rxBuf_CAN0[0] + (rxBuf_CAN0[1] * 256);
    engine_rpm = rxBuf_CAN0[2] + (rxBuf_CAN0[3] * 256);
    engine_temp = rxBuf_CAN0[4] + (rxBuf_CAN0[5] * 256);
    control_emp = rxBuf_CAN0[6] + (rxBuf_CAN0[7] * 256);
    high_voltage = rxBuf_CAN0[8] + (rxBuf_CAN0[7] * 256);
    Serial.print ("LowVoltage = ");
    Serial.println(low_voltage);
    Serial.print ("EngineRPM = ");
    Serial.println(engine_rpm);
  }
}

/*********************************************************************************************************
** Function name:           CAN0_interupt_rasp
** Descriptions:            read data of CAN1 (raspberry pi)
*********************************************************************************************************/
void CAN1_interrupt_rasp(void) {
  // Messages of the BMS and RASP
  CAN1.readMsgBuf(&rxId_CAN1, &len_CAN1, rxBuf_CAN1);      // Read data: len = data length, buf = data byte(s)
  if (rxId_CAN1 & 0x40000000) {
    return; // remoteote request data is not important
  }
  else if (rxId_CAN0 == CAN_ID_RASP) {

  }
}

/*********************************************************************************************************
** Function name:           mode_select
** Descriptions:            detrmine why the car is on
*********************************************************************************************************/
void mode_select() {
  //// Detect why the car is on
  // Detect if the charge plug is in the car or not
  if (digitalRead(di_gnd_lader)) {
    mode_charge = true;
    mode_drive = false;
  }
  //Detect if the key is switch is on and the charge plug is not in the car
  else if (digitalRead(di_keysignaal)) {
    mode_charge = false;
    mode_drive = true;
  }
  //Detect if the key switch is off and the charger is not plugged in
  else {
    mode_charge = false;
    mode_drive = false;
  }
}
/*********************************************************************************************************
** Function name:           charge
** Descriptions:            charging sequence of the alley hoop
*********************************************************************************************************/
void charge() {
  ////set charging true so we know in shut_down what the last called mode was (charging or driving)
  charging = true;

  ////Disable engine-, brake- and steering control system
  digitalWrite(do_drive_enable, LOW);
  digitalWrite(do_brake1_enable, LOW);
  digitalWrite(do_brake2_enable, LOW);
  digitalWrite(do_steer_r_en,   LOW);
  digitalWrite(do_steer_l_en,   LOW);

  ////Enable charger
  digitalWrite(do_pilotswitch,  HIGH);
  Serial.println("charge complete");
}

/*********************************************************************************************************
** Function name:           drive
** Descriptions:            driving sequence of the alley hoop
*********************************************************************************************************/
void drive() {
  ////set charging false so we know in shut_down what the last called mode was (charging or driving)
  charging = false;

  ////Disable charger
  digitalWrite(do_pilotswitch,  LOW); // t.b.v. laadpaal comm. 

  ////Enable steering-, brake- and engine control system
  digitalWrite(do_drive_enable, HIGH);
  digitalWrite(do_brake1_enable, HIGH);
  digitalWrite(do_brake2_enable, HIGH);
  digitalWrite(do_steer_r_en,   HIGH);
  digitalWrite(do_steer_l_en,   HIGH);

  ////Read the can of the curtis
  checkCAN_curtis();

  ////Read setpoint's for chosen mode and determine if you need to drive forward or reverse
  ///Check if the car is in joystick mode
  if (mode_joystick == true) {
    //Detect if the car needs to be in forward or reverse mode from the HMI signals
    if (digitalRead(di_hmi_switchfr) == true) {
      mode_forward = true;
      mode_reverse = false;
    }
    else {
      mode_forward = false;
      mode_reverse = true;
    }
    //Calculate setpoint from the HMI in the right unit
    engine_sp   = analogRead(ai_hmi_gas)  - engine_joystick_offset;        //read in the gas signal from the joystick and calculate this to %
    steering_sp = analogRead(ai_hmi_steering) - steering_joystick_offset; //read in the steering signal from the joystick and calcualte this to °

    if (engine_sp > 0){
      brake_sp = 0;
    }
    else if (engine_sp < 0){
      brake_sp = abs(engine_sp);
      engine_sp = 0;   
    }
    else{
      brake_sp = 0;
      engine_sp = 0;
    }
// *************************** heb mijn twijvels over de juiste waardes en begrenzingen ****************************************

    engine_sp = engine_sp * 100 / 123;          //calculate the engine setpoint to a range of 0 - 100 %
    brake_sp = brake_sp * 250 / 123;            //calculate the brake setpoint to a range of 0 - 250 bar
    steering_sp = steering_sp * 270L / 246L;      //calculate the steering setpoint to a range of -135 - 135°    
    steering_sp = constrain(steering_sp, -40, 40);
    steering_sp = -steering_sp; // tijdelijk gedaan omdat joystick verkeertom werkt
  }

  ///Check if the car is in remote mdoe
  //Detect if the car needs to be in forward or reverse mode from the remote signals
  else if (mode_remote == true) {
    if (digitalRead(di_remote_switchfr) == true) {
      mode_forward = true;
      mode_reverse = false;
    }
    else {
      mode_forward = false;
      mode_reverse = true;
    }


    //Calculate setpoint from the remote signals in the right unit

  }

  ///Detect if the car is in auto mode
  //Detect if the car needs to be in forward or reverse mode from the Raspberry Pi signals
  else if (mode_auto == true) {
    if (rasp_switchfr == true) {
      mode_forward = true;
      mode_reverse = false;
    }
    else {
      mode_forward = false;
      mode_reverse = true;
    }
  }

  //Calculate setpoint from the Raspberry Pi signals in the right unit
  else if (mode_auto == true) {
    //Read CAN data of the raspberry pi
    checkCAN_rasp();

    //Map the CAN data from 8 bit to 10 bit
    map(rasp_gas, 0, 255, 0, 1023);
    map(rasp_brake, 0, 255, 0, 1023);
    map(rasp_steering, 0, 255, 0, 1023);

    engine_sp = rasp_gas;
    brake_sp = rasp_brake;
    steering_sp = rasp_steering;
  }

  ////Read the proces value of the steering, brakes and engine
  engine_pv = (engine_rpm * float(100 / 1023));                                          //the proces value of the engine system is the rpm wich we read from the CAN-bus of the curtis
  brake_pv = (long(analogRead(ai_brake_pressure) - brake_pv_offset) * (250 / 1023));    //the proces value of the braking system is the pressure sensor
  steering_pv = (long(analogRead(ai_steer_pot)- steering_pv_offset) * 100/1023);    //the proces value of the steering system is the linear potentiometer


  ////When the car is standing still determine if the car needs to drive in forward or reverse
//  if (engine_rpm == 0) {
    ///If mode_forward is true enable do_drive_forward and disable do_drive_reverse

 
    if (mode_forward == true) {
      digitalWrite(do_drive_forward,  HIGH);
      digitalWrite(do_drive_reverse,  LOW);

    }
//    else {
//      digitalWrite(do_drive_forward,  LOW);
//    }

    ///If mode_reverse is true enable do_drive_reverse and disable do_drive_forward
    if (mode_reverse == true) {
      digitalWrite(do_drive_reverse,  HIGH);
      digitalWrite(do_drive_forward,  LOW);
    }
//    else {
//      digitalWrite(do_drive_reverse,  LOW);
//    }
//  }

  ////Set the curtis in the right mode for braking, driving or neutral
  ///Check if there is a brake stepoint, if yes we will configure the curtis in the following way
  if (brake_sp > brake_DB) {
    engine_sp = 0;                                //the engine setpoint will be set to zero
    digitalWrite(do_drive_throttleswitch, LOW);   //the throttleswitch of the curtis will be turned off (this detects if there is being pressed on the gas pedal if we turn this of it will not read the throttle signal)
    digitalWrite(do_drive_brake,          HIGH);  //the brake switch will be turned on so that the curtis can help braking
  }
  ///If there wasnt a brake setpoint check if there is an engine setpoint, if yes we will configure the curtis in the following way
  else if (engine_sp > engine_DB) {
    digitalWrite(do_drive_throttleswitch, HIGH);  //the throttle switch of the engine will be turned on so the curtis can receive a throttle signal
    digitalWrite(do_drive_brake,          LOW);   //the brake switch will be turned off
  }
  ///If there is no engine or brake setpoint, we will configure the curtis in the following way
  else {
    digitalWrite(do_drive_throttleswitch, LOW);   //the throttle switch will be turned off so the curtis can't recieve a throttle signal
    digitalWrite(do_drive_brake,          LOW);   //the brake switch will be turned off
  }

  ////Calculate delta (delta = SP - PV) for the engine, brakes and steering
  engine_delta = engine_sp - engine_pv;
  brake_delta = brake_sp - brake_pv;
  steering_delta = steering_sp - steering_pv;

  ///if the value of delta is smaller than the dead zone set to 0 if its larger substratct the dead zone
  if (abs(engine_delta) > engine_DB) {
    engine_delta = engine_delta - engine_DB;  // wat als engine_delta negatief is???? dan moet DB toch opgeteld worden?

  }
  else {
    engine_delta = 0;
  }

  ///if the value of delta is smaller than the dead zone set to 0 if its larger substratct the dead zone
  if (abs(brake_delta) > brake_DB) {
    brake_delta = brake_delta - brake_DB;
  }
  else {
    brake_delta = 0;
  }

  ///if the value of delta is smaller than the dead zone set to 0 if it is larger substratct the dead zone in case of positive and add the dead zone in case of a negative delta
  if (abs(steering_delta) > engine_DB) {
    if (steering_delta > 0) {
      steering_delta = steering_delta - steering_DB;
    }
    else {
      steering_delta = steering_delta + steering_DB;
    }
  }
  else {
    steering_delta = 0;
  }

  ////Determine the output
  ///Calculate OP
  engine_op = engine_delta * engine_kp;
  brake_op = brake_delta * brake_kp;
  steering_op = abs(steering_delta) * steering_kp;

  ///calculate the op value to a resolution for analog write, 8 bit (0-255)
  engine_op = engine_op * 255 / 100;
  brake_op = brake_op  * float(255 / 250);
  steering_op = steering_op * 255 / 270;

  ///safety feature to make sure the OP remains between boundaries
 engine_op = constrain(engine_op, 0, 255);
 brake_op =  constrain(brake_op, 0, 255);
 steering_op = constrain(steering_op, 0, 255);

  ////Send the OP to the devices
  ///Send OP to engine
  analogWrite(pwm_drive_throttle, engine_op); //Write op to throttle signal of curtis
  ///Send OP to brakes

// tijdelijk de stand van de joystick als pump aansturing gebruikt (Proportioneelmet stand). De druk wordt hierbij weggelaten
//  analogWrite(pwm_brake_pump, brake_op);         //Write op to brake pump
brake_sp =  constrain(brake_sp, 0, 255);
  analogWrite(pwm_brake_pump, brake_sp);         //Write op to brake pump

//sturen tijdelijk even uitgezet omdat dit eerst goed getest moet worden.
//                //Send OP to steering
//                //if delta is positive steer right
                if (steering_delta > 0) {
                  analogWrite(pwm_steer_rpwm, steering_op);
                  analogWrite(pwm_steer_lpwm, 0);
                }
                //if delta is negative steer left
                else if (steering_delta < 0) {
                  analogWrite(pwm_steer_rpwm, 0);
                  analogWrite(pwm_steer_lpwm, steering_op);
                }
                //if delta = 0 stay in position
                else {
                  analogWrite(pwm_steer_rpwm, 0);
                  analogWrite(pwm_steer_lpwm, 0);
                }

//
}

/*********************************************************************************************************
** Function name:           shut_down
** Descriptions:            shut down sequence of the alley hoop
*********************************************************************************************************/
void shut_down() {
  ////turn the systems off
  analogWrite(pwm_drive_throttle,  0);
  analogWrite(pwm_brake_pump,      0);
  analogWrite(pwm_steer_rpwm,      0);
  analogWrite(pwm_steer_lpwm,      0);
  digitalWrite(do_drive_enable,    LOW);
  digitalWrite(do_pilotswitch,     LOW);

  ////Wait 1 sec for the current in the HV_relay to go away
  delay(1000);

  ////open the high voltage relay
  digitalWrite(do_hv_relais,      LOW);
  ////Turn off the arduino
  digitalWrite(do_verbinding,     LOW);
  Serial.println("shut down complete");
}

/*********************************************************************************************************
** Function name:           stop
** Descriptions:            stop sequence of the alley hoop
*********************************************************************************************************/
void stop_car() {
  Serial.println("stop complete");
}

/*********************************************************************************************************
** Function name:           loop
** Descriptions:            main program, check wich function needs to be runned
*********************************************************************************************************/
void loop() {
  ////Detect the status of the stop switch if true poceed with normal loop if false run stop program
  if (digitalRead(di_hmi_switchstop) == true) {

    mode_select();

    ////check if charge_start is active if yes run charge else check if drive_start is active if yes run drive else run shut_down
    if (mode_charge == true) {
      charge();
      Serial.println(" Proces in Laadmode ");
    
    }
    else if (mode_drive == true) {
      drive();
     Serial.print(" mode_forward =  ");
     Serial.print(mode_forward);
     Serial.print(" mode_reverse =  ");
     Serial.print(mode_reverse);
     Serial.print(" engine_sp =  ");
     Serial.print(engine_sp);
     Serial.print(" Gassign. =  ");
     Serial.print(engine_op);
     Serial.print(" brake_sp = ");
     Serial.print(brake_sp);
     Serial.print(" brake_uit = ");
     Serial.print(brake_op);

     Serial.print(" Stuur_sp =  ");
     Serial.print(steering_sp);
     Serial.print(" Stuurhoek =  ");
     Serial.print(steering_pv);
     Serial.print(" Stuuruit =  ");
     Serial.println(steering_op);

     
     }
    else {
      shut_down();
      Serial.println(" Proces in shutdownmode ");

    }
    ///Send data to HMI
    /*
      send_hmi()
    */
  }
  else {
    stop_car();
    Serial.println(" Proces in Noodstop ");

  }
}
