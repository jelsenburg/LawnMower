Lawn Mower Missing Code.
// all functions in a single controller

/**************************** includes *********************************************/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <LIDAR_Wire_1.h>
#include <Servo.h>
#include <math.h>

// movement and motor control
#include <motor_data.h>
#include <Movement_Control.h>
#include <Motor_Control.h>
#include <Drive_Parameter_set.h>

// CAN
#include "variant.h"
#include <due_can.h>
/************************* end of includes ******************************************/

/**************************** FreeRTOS ********************************************/
/ creating the task handle */
TaskHandle_t xHandleGPS, xHandleLIDAR, xHandleCutterCurrentMeasurement;

/* creating the queue for the Serial send */
QueueHandle_t mot_ctrl_queue, mov_ctrl_queue, cutter_ctrl_queue;

SemaphoreHandle_t Left_interruptSemaphore, Right_interruptSemaphore;
/************************* FreeRTOS data ********************************************/

/******************** Kalman filter ***********************************************/
/
SimpleKalmanFilter(e_mea, e_est, q);
e_mea: Measurement Uncertainty
e_est: Estimation Uncertainty
q: Process Noise
*/
SimpleKalmanFilter Current_1_Filter(2, 2, 2.00), Current_2_Filter(2, 2, 2.00);
/************** end of Kalman filter ************************************************/

/******************** File *********************************************************/
File myFile, myFile_NA;

/******************** end of File **************************************************/

/******************** LIDAR *********************************************************/

#define Start_state 30000
#define Scanning_state 30001
#define Object_end_detection 30002
#define End_of_avoiding 30003

VL53L1X LIDAR;

/******************** end of LIDAR **************************************************/

/******************** Servo *********************************************************/
Servo scanning;

Servo vertical;

/******************** end of Servo **************************************************/

/********************* GPS *********************************************************/
TinyGPS gps;

/********************* end of GPS **************************************************/

/********************* Control data ************************************************/
struct Mov_Ctrl_Data{
String Source;
String Data;
};

struct SD_Data{
String Map;
String Data;
};
/********************* end of Control data ****************************************/

/********************* Point coordinates ********************************************/
struct P_cord{
float x;
float y;
bool valid;
};

/********************* end of Point coordinates *************************************/

/************************* Movement plan ********************************************/
#define Initializing 10001
#define Movement_plan_counting 10002
#define MovementPlanArrangement 10003
#define Perimeter_execution 10004
#define Movement_plan_exectuion 10005

typedef struct Movement_Plan{
int A_end_reached;
int B_end_reached;
int planned;
int next_line_index;
struct P_cord A_end;
struct P_cord B_end;
} movement_plan;

movement_plan MovPln[500];

// Perimeter execution states
#define Perimeter_Reading 20000
#define Line_execution 20001
#define Line_reading 20002

// object avoiding
#define LIDAR_resuming 40000
#define Correction 40001
#define Avoiding_Object 40002
#define Returning_to_P 40003

/********************** end of Movement plan **********************************/

/******************** MPU6050 ************************************************/
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

#define OUTPUT_READABLE_ACCELGYRO
/********************** end of MPU6050 ***************************************/

/********************** CAN *************************************************/

/********************** end of CAN ******************************************/

/******************** structure for the communication between tasks. ***************/
struct Data_struct{
String source;
float Right_SP;
float Left_SP;
};

float motor_max_speed = 0.71, max_speed_pwm = 25.0;

/**************** Motor control outputs and simple outputs***********************************/

#define MotorLeftPin_1 25
#define MotorLeftPin_2 24
#define MotorLeftPin_PWM 9

#define MotorRightPin_1 23
#define MotorRightPin_2 22
#define MotorRightPin_PWM 8

#define Cutter_1_PWM 7
#define Cutter_1_Direction 27
#define Cutter_2_PWM 6
#define Cutter_2_Direction 26

#define Encoder_Left_A_pin 28
#define Encoder_Left_B_pin 29
#define Encoder_Right_A_pin 30
#define Encoder_Right_B_pin 31

#define A_Distance_OK 26
#define B_Distance_OK 27
#define C_Distance_OK 28

int Right_interrupt_count = 0, Left_interrupt_count = 0, Encoder_trigger_limit = 30000;

/******************** Cutter outputs and inputs *******************************************/

#define Cutter_1 44
#define Cutter_2 45
#define Cutter_1_current A0
#define Cutter_2_current A1

void setup() {
Serial.begin(115200);
Serial.println("Startup");

Serial1.begin(9600); // Serial for GPS
Serial2.begin(9600); // Serial for Bluetooth
Serial3.begin(115200); // Serial for DWM1000

pinMode(Encoder_Left_A_pin, INPUT);
pinMode(Encoder_Left_B_pin, INPUT);
pinMode(Encoder_Right_A_pin, INPUT);
pinMode(Encoder_Right_B_pin, INPUT);
pinMode(MotorLeftPin_1, OUTPUT);
pinMode(MotorLeftPin_2, OUTPUT);
pinMode(MotorLeftPin_PWM, OUTPUT);
pinMode(MotorRightPin_1, OUTPUT);
pinMode(MotorRightPin_2, OUTPUT);
pinMode(MotorRightPin_PWM, OUTPUT);
pinMode(Cutter_1, OUTPUT);
pinMode(Cutter_2, OUTPUT);

pinMode(Cutter_1_PWM, OUTPUT);
pinMode(Cutter_2_PWM, OUTPUT);

digitalWrite(MotorLeftPin_1,LOW);
digitalWrite(MotorLeftPin_2, HIGH);

digitalWrite(MotorRightPin_1, LOW);
digitalWrite(MotorRightPin_2, HIGH);

digitalWrite(Cutter_1, LOW);
digitalWrite(Cutter_2, LOW);

/*********************** SD setup **************************************************/
Serial.print("Initializing SD card...");
if (!SD.begin(4)) {
Serial.println("initialization failed!");
while (1);
}
Serial.println("initialization done.");

/*********************** CAN setup **************************************************/
Serial.println("Doing Auto Baud scan on CAN0");
Can0.beginAutoSpeed();

// CAN filter
int filter;
//extended
for (filter = 0; filter < 3; filter++) {
Can0.setRXFilter(filter, 0, 0, true);

}

//standard
for (int filter = 3; filter < 7; filter++) {
Can0.setRXFilter(filter, 0, 0, false);

}

/*********************** Output setup *************************************************/

delay(1000);

/* Create a queue. https://www.freertos.org/a00116.html */

mot_ctrl_queue = xQueueCreate(10, sizeof(struct Data_struct));

mov_ctrl_queue = xQueueCreate(20, sizeof(Mov_Ctrl_Data));
cutter_ctrl_queue = xQueueCreate(10, sizeof(Mov_Ctrl_Data));

Left_interruptSemaphore = xSemaphoreCreateBinary();
Right_interruptSemaphore = xSemaphoreCreateBinary();

if ((mot_ctrl_queue != NULL)&&(mov_ctrl_queue != NULL)&&(cutter_ctrl_queue != NULL)&&(Left_interruptSemaphore != NULL)&&(Right_interruptSemaphore != NULL)){

attachInterrupt(digitalPinToInterrupt(Encoder_Left_A_pin), left_encoderInterrupt, FALLING);
attachInterrupt(digitalPinToInterrupt(Encoder_Right_A_pin), right_encoderInterrupt, FALLING);
xTaskCreate(left_encoder, (const portCHAR *)"left_encoder", 256, NULL, 3, NULL);
xTaskCreate(right_encoder, (const portCHAR *)"right_encoder", 256, NULL, 3, NULL);


xTaskCreate(CAN_Receive, (const portCHAR *)"CAN_Receive", 512, NULL, 2, NULL);
//xTaskCreate(CAN_Send, (const portCHAR *)"CAN_Send", 128, NULL, 1, NULL);
//xTaskCreate(MPU6050_task,(const portCHAR *)"MPU6050_Communication", 128, NULL, 3, NULL);        
xTaskCreate(Bluetooth, (const portCHAR *)"Bluetooth", 512, NULL, 2, NULL);
xTaskCreate(dwm1000,(const portCHAR *)"dwm1000", 512, NULL, 1, NULL);
xTaskCreate(mov_ctrl, (const portCHAR *)"movement_control", 1024, NULL, 2, NULL);
xTaskCreate(mot_ctrl, (const portCHAR *)"motor_control", 512, NULL, 2, NULL);
//xTaskCreate(GPS, (const portCHAR *)"GPS_task", 256, NULL, 1, &xHandleGPS);
//xTaskCreate(LIDAR_task,(const portCHAR *) "LIDAR", 256, NULL, 2, &xHandleLIDAR);
xTaskCreate(Cutter_task,(const portCHAR *) "Cutter", 256, NULL, 2, NULL);
xTaskCreate(Cutter_Current_task,(const portCHAR *) "CutterCurrent", 256, NULL, 2, &xHandleCutterCurrentMeasurement);
}

xTaskCreate(TaskBlink, "Blink", 128, NULL, 0, NULL);

vTaskStartScheduler();

Serial.println("Failed to start FreeRTOS scheduler");
while(1);

Serial.println("end_startup");
}

void loop() {}

/******************************************** HW interrupts ************************************************************************************
* With HW interrupts, checking the movement of the weehls shall be done.
/
void left_encoderInterrupt() {
Left_interrupt_count++;
/*if the interrupt count is equal to 1320, which is 1/5 th part of a whole turn, the movement control shall be notified./
if(Left_interrupt_count > Encoder_trigger_limit){
Left_interrupt_count = 0;
xSemaphoreGiveFromISR(Left_interruptSemaphore, NULL);
}
}

void left_encoder(void *pvParameters)
{
(void) pvParameters;
struct Mov_Ctrl_Data local_data;

for (;;)
{
if (xSemaphoreTake(Left_interruptSemaphore, portMAX_DELAY) == pdPASS) {

local_data.Source = "Left_Encoder";
local_data.Data = "Trigger";
xQueueSend(mov_ctrl_queue, &local_data, portMAX_DELAY);
}
}
}

void right_encoderInterrupt() {
Right_interrupt_count++;
/if the interrupt count is equal to 1320, which is 1/5 th part of a whole turn, the movement control shall be notified./
if(Right_interrupt_count > Encoder_trigger_limit){

Right_interrupt_count = 0;
xSemaphoreGiveFromISR(Right_interruptSemaphore, NULL);
}

}

void right_encoder(void *pvParameters)
{
(void) pvParameters;
struct Mov_Ctrl_Data local_data;

for (;;)
{
if (xSemaphoreTake(Right_interruptSemaphore, portMAX_DELAY) == pdPASS) {

  local_data.Source = "Right_Encoder";
  local_data.Data = "Trigger";
  xQueueSend(mov_ctrl_queue, &local_data, portMAX_DELAY);
}
}
}

/******************************************** end of HW intterrupts *******************************************************************************/

/******************************************** Sending data over CAN *********************************************************************************
* Currently no data will be sent over the CAN.
*/

void CAN_Send(void *pvParameters)
{
(void) pvParameters;

int i = 0;

for (;;)
{
/* CAN_FRAME output;
// Prepare transmit ID, data and data length in CAN0 mailbox 0
output.id = TEST1_CAN_TRANSFER_ID;
output.length = MAX_CAN_FRAME_DATA_LEN;
//Set first four bytes (32 bits) all at once
output.data.low = CAN_MSG_1;
//Set last four bytes (32 bits) all at once
output.data.high = CAN_MSG_DUMMY_DATA;
//Send out the frame on whichever mailbox is free or queue it for
//sending when there is an opening.
CAN.sendFrame(output);/
vTaskDelay( 500 / portTICK_PERIOD_MS );
}
}
/******************************************* end of Sending data over CAN **************************************************************************/

/******************************************** Receiving data over CAN *********************************************************************************
* The Left and right speed sensor and also the interface send CAN messages to the Master.
* Left speed is received with 0x051 CAN ID.
* Left distance is received with 0x052 CAN ID
* Right speed is received with 0x047 CAN ID
* Right distance is received with 0x048 CAN ID
* Data from the Interface is received with 0x031 CAN ID
*
*/

void CAN_Receive(void *pvParameters)
{
(void) pvParameters;

struct Mov_Ctrl_Data CAN_data; // !!!! used elements shall be checked!!!!!!

for (;;)
{
CAN_FRAME canMsg;

if(Can0.available() > 0){
  Can0.read(canMsg);
/* Serial.print("received: ");
Serial.println(canMsg.id, HEX);*/

  if(canMsg.id == 0x041){

      CAN_data.Source = "Distance";


    float distance_1 = 0.0, distance_2 = 0.0;
    distance_1 = (float)canMsg.data.bytes[0];
    distance_2 = (float)canMsg.data.bytes[2];

    CAN_data.Data = "";
    CAN_data.Data = String(distance_1);
    CAN_data.Data+=",";
    CAN_data.Data+=String(distance_2);

 /*   Serial.print("CAN: ");
    Serial.print(canMsg.data.bytes[0]);
    Serial.print(", ");
    Serial.print(canMsg.data.bytes[1]);
    Serial.print(", ");
    Serial.print(canMsg.data.bytes[2]);
    Serial.print(", ");
    Serial.print(canMsg.data.bytes[3]);
    Serial.print(", ");
    Serial.println(CAN_data.Data);*/


    // sending the received data to Motor Control
   // xQueueSend(mov_ctrl_queue, &CAN_data, portMAX_DELAY);

} /*else {
  CAN_data.source = "CAN error";
  //xQueueSend(mot_ctrl_queue, &CAN_data, portMAX_DELAY);

  Serial.println("CAN ERROR");
}*/

}
vTaskDelay(50);
}
}
/******************************************** end of Receiving data over CAN **************************************************************************/

/********************** Movement Control task *********************************************************************************************************
* Controlling the movement of the mover.
* In Manual control the Joystick provides the control data. If the A button is pressed, the
* mover will move straight forward. If the X value is changed, the Left or Right setpoint will
* be decreased.If the joystick is back in it original position it the mover will move
* straight again.
* If the Perimeter teach is set, the positions shall be saved to the memory card.
* In Automatic mode, the mover shall move all by itself.
*
*/
void mov_ctrl(void *pvParameters){

struct Mov_Ctrl_Data received_data, cutter_data;

struct Data_struct local_data;

struct P_cord Point_coordinates, actual_position, previous_position, next_point, next_to_be_saved, correction_point, return_point, first_point, planning_point_X, planning_point_X_1;

bool Automatic_mode = false, Automatic_mode_activated = false, Trigger_signal_for_SP_counting = false, Point_reached = false, Perimeter_teach = false, NA_teach = false, GPS_response = false, first_execution = false, init_prev_position = false, object_ahead = false, LIDAR_correction = false, A_point_selected = true, B_point_selected = false, E_Stop = false, Cutter_ON = false, SP_to_100_exec = false, count_SP = false;

String Radio_data = "", Serial_data = "", A_distance = "", B_distance = "", C_distance = "", FILE_name = "", FILE_name_NA = "", Comma = ",", GPS_coordinate_actual = "", GPS_coordinate_saved = "";

int number_of_Perimeter_points = 0, text_len = 31, act_file_pos = 0, movement_plan_state = Initializing, movement_plan_internal_state = 0, act_perimeter = 0, Mov_plan_exec_state = Perimeter_Reading, object_avoiding_state, numberofNApoints = 0, MovPln_pointer = 0, MovPln_execution_ptr = 0, SP_Counting_state = 0;

float Right_SP = 0.0, Left_SP = 0.0, SP_value = 100.0, Manual_control_threshold = 5.0, X_value_N_1 = 0.0, Usonic_1 = 0.0, Usonic_2 = 0.0, LIDAR_distance = 0.0, GPS_distance = 10.0, AB_distance = 7.7, AC_distance = 3.0, BC_distance = 6.0, Mower_Width = 0.44, Mower_Lenght = 0.22, Position_tolerance = 0.6, Coordinate_tolerance = 0.2, Heading = 0.0, correction_value = 0.0, object_end = 0.0, object_distance_ahead = 50.0, objec_ahead_E_Stop = 30;

unsigned long time_N = 0, time_N_1 = 0;
// mover width and lenght shall be given in meters.
// float AB_distance = 0.0, AC_distance = 0.0, BC_distance = 0.0;

char GPS_coordinate_saved_array[text_len];

for( ;; ){

if (xQueueReceive(mov_ctrl_queue, &received_data, portMAX_DELAY) == pdPASS) {

 /* if the received data source is the ethernet task
  * Two sources are received from eth:
* - Trigger: actual position shall be saved if this is received
* - E-Stop: movement controller shall stop the navigation and execution of the movement plan
  */     

   // if the received data is TRIGGER, and the Perimeter teaching is activated, then the actual poisition shall be counted and saved.
   // the perimeter points number shall be increased
 if(((received_data.Source =="Right_Encoder")||(received_data.Source =="Left_Encoder"))&&(received_data.Data == "Trigger")){   

    /* Providing the trigger singal for SP counting */
    Trigger_signal_for_SP_counting = true;
Serial.println("Trigger");
Serial.print(", actual_position.x : ");
Serial.print(actual_position.x);
Serial.print(", actual_position.y: ");
Serial.print(actual_position.y);
Serial.print(", previous_position.x : ");
Serial.print(previous_position.x);
Serial.print(", previous_position.y: ");
Serial.println(previous_position.y);

    Point_coordinates = Count_Coordinate(AB_distance, AC_distance, BC_distance, A_distance.toFloat(), B_distance.toFloat(), C_distance.toFloat()); 

    /* saving the coordinates shall only be done if the coordinate is not a nan */
    if(Point_coordinates.valid){
        String Data_to_be_Saved = String(Point_coordinates.x);
        Data_to_be_Saved+=',';
        Data_to_be_Saved+=String(Point_coordinates.y);
        Data_to_be_Saved+=',';
        Data_to_be_Saved+=A_distance;
        Data_to_be_Saved+=',';
        Data_to_be_Saved+=B_distance;
        Data_to_be_Saved+=',';
        Data_to_be_Saved+=C_distance;

        int i = Data_to_be_Saved.length();
        if(i < text_len - 2){
          for(; i < text_len - 2; i++){
            Data_to_be_Saved+=" ";
          }
        }

        if((Perimeter_teach)&&(!NA_teach)){
          myFile = SD.open(FILE_name, FILE_WRITE);
          myFile.println(Data_to_be_Saved);
          myFile.close();
          number_of_Perimeter_points++;
          Serial.println("Point saved: ");
          Serial.println(Data_to_be_Saved);
        } else if (NA_teach){
          myFile_NA = SD.open(FILE_name, FILE_WRITE);
          myFile_NA.println(Data_to_be_Saved);
          myFile_NA.close();
          numberofNApoints++;
        }
    }  else {
       Serial.println("* * * * NaN * * * *");
    }
 } 
else if(received_data.Source == "Distance"){

 String Usonic_1_in_String = received_data.Source.substring(0, received_data.Source.indexOf(Comma));
 Usonic_1 = Usonic_1_in_String.toFloat(), 
 received_data.Source.remove(0, received_data.Source.indexOf(Comma) + 1);
 Usonic_2 = received_data.Source.toFloat();

/* If the received distance is less than the limit, than start detecting it.
 * If the value is even less than the safety limit, then E-Stop shall be done. */
object_distance_ahead = 50.0, objec_ahead_E_Stop = 30;
if(((Usonic_1 < object_distance_ahead)&&(Usonic_1 > objec_ahead_E_Stop))||(((Usonic_2 < object_distance_ahead)&&(Usonic_2 > objec_ahead_E_Stop)))){
  object_ahead = true;
} else if ((Usonic_1 < objec_ahead_E_Stop)||(Usonic_2 < objec_ahead_E_Stop)){
  E_Stop = true;
}

    if(!object_ahead){
      object_ahead != object_ahead;
      object_avoiding_state = LIDAR_resuming;
    }       
} else if(received_data.Source == "MPU6050"){
if((received_data.Data = "E-Stop")||(received_data.Data = "Error")){
E_Stop = true;
}

}
// saving AB distance
  else if(received_data.Source == "LCD_AB_dist"){
  AB_distance = received_data.Data.toFloat();

// saving BC distance
} else if(received_data.Source == "LCD_AC_dist"){
  AC_distance = received_data.Data.toFloat();

 // saving distances received from DWM1000
} else if(received_data.Source == "DWM1000"){
String temp_DWM1000 = received_data.Data, temp_A_dis = "", temp_B_dis = "", temp_C_dis = "";
// getting A distance
temp_A_dis = temp_DWM1000.substring(0, temp_DWM1000.indexOf(Comma));
if(temp_A_dis != "Err"){
  A_distance = temp_A_dis;
  digitalWrite(A_Distance_OK, HIGH);
} else {
  digitalWrite(A_Distance_OK, !digitalRead(A_Distance_OK));      
}
  temp_DWM1000.remove(0, temp_DWM1000.indexOf(Comma) + 1);

// getting B distance
temp_B_dis = temp_DWM1000.substring(0, temp_DWM1000.indexOf(Comma));
if(temp_B_dis != "Err"){
  B_distance = temp_B_dis;
  digitalWrite(B_Distance_OK, HIGH);
} else {
  digitalWrite(B_Distance_OK, !digitalRead(B_Distance_OK));      
}
  temp_DWM1000.remove(0, temp_DWM1000.indexOf(Comma) + 1);

// getting C distance
temp_C_dis = temp_DWM1000.substring(0, temp_DWM1000.indexOf(Comma));
if(temp_C_dis != "Err"){
  C_distance = temp_C_dis;
  digitalWrite(C_Distance_OK, HIGH);
} else {
  digitalWrite(C_Distance_OK, !digitalRead(C_Distance_OK));      
}

  temp_DWM1000.remove(0, temp_DWM1000.indexOf(Comma) + 1);
} else if (received_data.Source == "LIDAR"){
if(!LIDAR_correction){
// if any data is received from LIDAR it shall be converted into float, since the P_cord struct is set up from floats.
correction_value = received_data.Data.toFloat();
LIDAR_correction = true;
} else {
object_end = received_data.Data.toFloat();
}
} else if(received_data.Source == "Manual_SP"){
/* Manual control's setpoint from Bluetooth task
* The Bluetooth task counts the actual SP, hence here, the SP shall only be assigned to the Output value
* but first the two values shall be readed from the string.*/
if(!Automatic_mode){
String manual_ctrl = received_data.Data;
String Left_SP_in_String = manual_ctrl.substring(0, manual_ctrl.indexOf(Comma));
Left_SP = Left_SP_in_String.toFloat();
manual_ctrl.remove(0, manual_ctrl.indexOf(Comma) + 1);

       String Right_SP_in_String = manual_ctrl.substring(0, manual_ctrl.indexOf(Comma));     
       Right_SP = Right_SP_in_String.toFloat();
       manual_ctrl.remove(0, manual_ctrl.indexOf(Comma) + 1);
     }

 } else if((received_data.Source == "Manual_NA")&&(received_data.Data == "Start")){
     NA_teach = true;                     
 } else if((received_data.Source == "Manual_NA")&&(received_data.Data == "Finish")){
    NA_teach = false;

 } else if((received_data.Source == "Manual_Automatic")&&(received_data.Data == "ON")){
    Automatic_mode = true;
    Automatic_mode_activated = true;
    movement_plan_state = Initializing;    
 } else if((received_data.Source == "Manual_Automatic")&&(received_data.Data == "OFF")){
    Automatic_mode = false;
    movement_plan_state = 0;


    /* if the automatic mode is deactiated, then stop the utter */
    cutter_data.Source = "Mov_Ctrl";
    cutter_data.Data = "Cutter_Stop"; 
    /*If the automatic mode is activated the cutter shall be started. */
    xQueueSend(cutter_ctrl_queue, &cutter_data, portMAX_DELAY);

 } else if((received_data.Source == "Manual_Perimeter") &&(received_data.Data == "Start")){

    int i = 1;
    FILE_name="P_";
    FILE_name+=String(i);
    FILE_name+=".txt";


    FILE_name_NA = "NA_";
    FILE_name_NA+=String(i++);
    FILE_name_NA+=".txt";

    SD.remove(FILE_name);
    SD.remove(FILE_name_NA);    

    // the perimeter teach variable shall be set.
    Perimeter_teach = true;
Serial.println("Perimeter start");

 } else if((received_data.Source == "Manual_Perimeter")&&(received_data.Data == "Finish")){
    // finishing the perimeter teaching
    Perimeter_teach = false;
Serial.println("Perimeter finish");

    // when the Perimeter searching is done the number of the perimeter points shall be saved at the end of the file

    String Perimeter_point_number = String(number_of_Perimeter_points);
    int i = Perimeter_point_number.length();
    if(i < text_len - 1){
      for(; i < text_len - 1; i++){
        Perimeter_point_number+= " ";
      }
    }
    myFile = SD.open(FILE_name, FILE_WRITE);
    myFile.println(String(Perimeter_point_number));
    myFile.close();


    String NA_point_number = String(numberofNApoints);
    int j = NA_point_number.length();
    if(j < text_len - 1){
      for(; j < text_len - 1; j++){
        NA_point_number+= " ";
      }
    }
    myFile_NA = SD.open(FILE_name_NA, FILE_WRITE);
    myFile_NA.println(NA_point_number);
    myFile_NA.close();        

 } else if((received_data.Source == "Manual_Cutter")&&(received_data.Data == "ON")){
    // starting the cutter  
      cutter_data.Source = "Mov_Ctrl";
      cutter_data.Data = "Cutter_Start"; 
      Serial.println("Mov Ctrl cutter start"); 
      /*If the automatic mode is activated the cutter shall be started. */
      xQueueSend(cutter_ctrl_queue, &cutter_data, portMAX_DELAY);
      // the cutter on variable shall be set.
      Cutter_ON = true;
      vTaskResume(xHandleCutterCurrentMeasurement);

 } else if((received_data.Source == "Manual_Cutter")&&(received_data.Data == "OFF")){
    //  stopping the cutter
     cutter_data.Source = "Mov_Ctrl";
     cutter_data.Data = "Cutter_Stop";  
     /*If the automatic mode is activated the cutter shall be started. */
     xQueueSend(cutter_ctrl_queue, &cutter_data, portMAX_DELAY);
     vTaskSuspend(xHandleCutterCurrentMeasurement); 
    Serial.println("Mov Ctrl cutter stop"); 
 }     
}

// counting the actual heading
// It shall only be counted if there are at least 5cm difference in any coordinate  
float act_prev_distance = sqrt((pow((actual_position.x - previous_position.x),2)) + (pow((actual_position.y - previous_position.y),2)));    
if((Position_tolerance < act_prev_distance)&&(Trigger_signal_for_SP_counting)){
  float PI_const = 3.1415926;
  Heading = atan((actual_position.x - previous_position.x)/(actual_position.y - previous_position.y)) *180/PI_const;
  previous_position = actual_position;
}

actual_position = Count_Coordinate(AB_distance, AC_distance, BC_distance, A_distance.toFloat(), B_distance.toFloat(), C_distance.toFloat());


/* to get the previous position initialized, otherwise the Coordinate_tolerance will be always higher than the distance*/ 
/* if((!init_prev_position)&&(!isnan(actual_position.x))&&(!isnan(actual_position.y))){
init_prev_position = true;
/Serial.println("******************** prev position init **********************************"); */
/ previous_position = actual_position;
}
*/

Serial.print(" a distance: ");
Serial.print(A_distance.toFloat());
Serial.print(", b distance: ");
Serial.print(B_distance.toFloat());
Serial.print(", actual_position.x : ");
Serial.print(actual_position.x);
Serial.print(", actual_position.y: ");
Serial.print(actual_position.y);
Serial.print(", previous_position.x : ");
Serial.print(previous_position.x);
Serial.print(", previous_position.y: ");
Serial.println(previous_position.y);

if(Automatic_mode){

  cutter_data.Source = "Mov_Ctrl";
  cutter_data.Data = "Cutter_Start";  
  /*If the automatic mode is activated the cutter shall be started. */
  xQueueSend(cutter_ctrl_queue, &cutter_data, portMAX_DELAY);

  int i = 1;
  FILE_name="P_";
  FILE_name+=String(i);
  FILE_name+=".txt";


  FILE_name_NA = "NA_";
  FILE_name_NA+=String(i++);
  FILE_name_NA+=".txt";


    switch(movement_plan_state){
      case Initializing: {

    // identifiying the perimeter numbers
          // it is stored at the end of the file, so it shall be opened for writing
            myFile = SD.open(FILE_name, FILE_WRITE);
            char Perimeter_points_array[text_len];
            String Perimeter_points;
            act_file_pos = myFile.position();
            act_file_pos-=(text_len+2);
            myFile.seek(act_file_pos); 
            myFile.read(Perimeter_points_array,text_len);
            Perimeter_points = Perimeter_points_array;
            number_of_Perimeter_points = Perimeter_points.toInt();                
            myFile.close();

            // the NA points shall also be readed
            myFile_NA = SD.open(FILE_name_NA, FILE_WRITE);
            char Perimeter_points_NA_array[text_len]; 
            String Perimeter_points_NA;
            act_file_pos = myFile_NA.position();
            act_file_pos-=(text_len + 2);
            myFile_NA.seek(act_file_pos); 
            myFile_NA.read(Perimeter_points_NA_array,text_len);
            Perimeter_points_NA = Perimeter_points_NA_array;
            numberofNApoints = Perimeter_points_NA.toInt();
            myFile_NA.close();

            // changing the state to movement plan counting
            movement_plan_state = Movement_plan_counting;
            // the first lines init bit shall be false             

            Serial.print("Number of perimenter points: ");
            Serial.println(number_of_Perimeter_points);
        }
    break;

    case Movement_plan_counting: {
      // in the first step the Xmin shall be found in the Perimeter coordinates
      // the file is given, and the Perimeter numbers are known too
      float X_min = 10000.0, X_max = 0.0, Y_min = 10000.0, Y_max = 0.0, float_ptr[10];
      struct P_cord Act_Perimeter_Point, Act_Plan_Point;

      int File_pos = 0, File_pos_NA = 0;
      char Perimeter_point_data_array[text_len], NA_Point_data_array[text_len]; 
      String Perimeter_point_data, NA_Point_data;

      // opeing the file for reading
      myFile = SD.open(FILE_name);
      for(int i = 0; i < number_of_Perimeter_points; i++){
        // getting the actual position
        File_pos = myFile.position();
        // stepping over the GPS coordinate to get the first perimeter data
        File_pos+=(text_len - 1);
        //myFile.seek(File_pos);
        myFile.read(Perimeter_point_data_array,text_len);
        Perimeter_point_data = Perimeter_point_data_array;
        Act_Perimeter_Point = Coord_from_String(Perimeter_point_data);

        Serial.print(" i value: ");
        Serial.print(i);
        Serial.print(", point's x: ");
        Serial.print(Act_Perimeter_Point.x);
        Serial.print(", point's y: ");
        Serial.println(Act_Perimeter_Point.y);



        if(Act_Perimeter_Point.x < X_min){
          X_min = Act_Perimeter_Point.x;
        } else if (X_max < Act_Perimeter_Point.x){
          X_max = Act_Perimeter_Point.x;
        }

        if(Act_Perimeter_Point.y < Y_min){
          Y_min = Act_Perimeter_Point.y;
        } else if (Y_max < Act_Perimeter_Point.y){
          Y_max = Act_Perimeter_Point.y;
        }
      } 
      myFile.close();

      Serial.print(" X min: ");
      Serial.print(X_min);
      Serial.print(", X max: ");
      Serial.print(X_max);
      Serial.print(" Y min: ");
      Serial.print(Y_min);
      Serial.print(", Y max: ");
      Serial.println(Y_max);


      // If the Act_Plan_Point.x is selected there will be two y value belonging to this x value. These shall be searched and saved.
      // In the search for the coordinate the Coordinate_tolerance shall also be taken in consideration, because there might be a chance 
      // that no point can be found with the exact coordinates.

      // width shall be defined in meters 
      // starting with X min coordinate
      // The starting point X shall be increased by the half of the width!                     
      for(Act_Plan_Point.x=(X_min + (Mower_Width / 2)); Act_Plan_Point.x <= X_max; Act_Plan_Point.x+=(Mower_Width / 2)){

        Serial.print("For's act plan points, x: ");
        Serial.println(Act_Plan_Point.x);

        // the number of the coordinates shall be stored in 10 element array
        // the array shall be reserved here
        int y_coord_number = 0;

        // Opening the file
        myFile = SD.open(FILE_name);
        myFile_NA = SD.open(FILE_name_NA);


        // initializing the array
        for(int i = 0; i < 10; i++){
          float_ptr[i] = 0.0;
        }

        // stepping over the GPS coordinate to get the first perimeter data
        File_pos = myFile.position();
        File_pos+=text_len;
        //myFile.seek(File_pos);
        char Perimeter_point_array[text_len], NA_Point_data_array[text_len];



        // checking the perimeter points for the Y cordinates at the given X
        for(int i = 0; i < number_of_Perimeter_points; i++){
Serial.print(" i val: ");
Serial.println(i);

          /* the first point shall be saved to use it with the planning of the last point
           * The second coordinate shall be read once, since it changes the file pointer's value.*/
          if((Act_Plan_Point.x == (X_min + (Mower_Width / 2)))&&(i == 0)){
                /* reading the first point of the line */                         
                myFile.read(Perimeter_point_array,text_len);
                Perimeter_point_data = Perimeter_point_array;
                planning_point_X = Coord_from_String(Perimeter_point_data);

                myFile.read(Perimeter_point_array,text_len);
                Perimeter_point_data = Perimeter_point_array;
                planning_point_X_1 = Coord_from_String(Perimeter_point_data);

                first_point = Act_Perimeter_Point;
                Serial.println("First point saved");
          } else {

                planning_point_X_1 = planning_point_X;

                myFile.read(Perimeter_point_array,text_len);
                Perimeter_point_data = Perimeter_point_array;
                planning_point_X = Coord_from_String(Perimeter_point_data);

                /* in case of the last point, the first shall be pair */
                 if(i == (number_of_Perimeter_points - 1)){
                    planning_point_X_1 = first_point;
                    Serial.println("First point added to the last point counting");
                 }
          }
Serial.print("X1 coordinate: ");
Serial.print(planning_point_X.x);
Serial.print(", Y1 coordinate: ");
Serial.print(planning_point_X.y);
Serial.print(", X coordinate: ");
Serial.print(Act_Plan_Point.x);
Serial.print(", X2 coordinate: ");
Serial.print(planning_point_X_1.x);
Serial.print(", Y2 coordinate: ");
Serial.println(planning_point_X_1.y);

/* first, before the straight line counting check if the actual X is between the selected point's x /
if(((abs(planning_point_X.x) <= abs(Act_Plan_Point.x))&&(abs(Act_Plan_Point.x) <= abs(planning_point_X_1.x)))||((abs(planning_point_X.x) >= abs(Act_Plan_Point.x))&&( abs(Act_Plan_Point.x) >= abs(planning_point_X_1.x)))){
/ counting the straight line's parameters /
Serial.print("m: ");
float m = ((planning_point_X_1.y - planning_point_X.y)/(planning_point_X_1.x - planning_point_X.x));
Serial.print(m);
Serial.print(", b: ");
float b = -1 * m * planning_point_X.x + planning_point_X.y;
Serial.println(b);
/ if m is zero then no crossing point exsits since they are parallel */
if(m == 0){
Serial.println(" m is zero ");
continue;
}

                /* if the m is not a number, then the straight line is parallel to x axis, in this case the y value is equal to the y value */
                if(isnan(m)){
Serial.println(" m is nan ");
float_ptr[y_coord_number++] = planning_point_X.y;
continue;
}

                /* if m is a number and >0 then the y value an be counted according to y = m * x + b; */
                float_ptr[y_coord_number++] = m * Act_Plan_Point.x + b;
                Serial.print("counted value: ");
                Serial.println(m * Act_Plan_Point.x + b);
          }
        }




        // The array shall be in min order.
        for(int i = 0; i < 10; i++){
          float float_save = 0.0;
          for(int j = i; j < 10; j++){
            if((float_ptr[i] > float_ptr[j])&&(float_ptr[i] != 0)&&(float_ptr[j] != 0)){
              float_save = float_ptr[i];
              float_ptr[i] = float_ptr[j];
              float_ptr[j] = float_save;
            }
          }
        }
        Serial.println("Found y coordinates: ");
        for(int i = 0; i < 10; i++){
          Serial.println(float_ptr[i]);
        }


        // when the search for the perimeter points is over, the lines to be executed shall be created from the found points
        //if(y_coord_number > 0){
        //  y_coord_number--; // it shall be decreased, because during the counting the coordinates it was incremented: y_coord_number++.
        //} It shall not be decreased, because this way next while will be okay.

        // the first movement plan line shall be initialized too
        // when any coordinate can be found at x min
        // the case when no line can be found at X_min shall also be considered.
        // for this a new variable shall be used
        int i = 0;
        struct P_cord A_end;
        struct P_cord B_end;
        movement_plan actually_counted_line;
Serial.print(" the number of found y coordinates: ");
Serial.println(y_coord_number);

        // counting the coordinates
        while(i < y_coord_number){
          actually_counted_line.A_end.x = Act_Plan_Point.x;
          actually_counted_line.A_end.y = float_ptr[i++];
          actually_counted_line.B_end.x = Act_Plan_Point.x;
          actually_counted_line.B_end.y = float_ptr[i++];

          // movement plan pointer shall be increased too
          MovPln[MovPln_pointer++] = actually_counted_line;
Serial.print("act point's A.x: ");
Serial.print(A_end.x);
Serial.print("act point's A.y: ");
Serial.print(A_end.y);
Serial.print("act point's B.x: ");
Serial.print(B_end.x);
Serial.print("act point's B.y: ");
Serial.println(B_end.y);
Serial.println("creating new line");

Serial.println("new line created");

}

        // closing the used file
        myFile.close();
        myFile_NA.close();

      }
Serial.println("The created lines: ");

for(int g = 0; g < MovPln_pointer; g++){
Serial.print("number: ");
Serial.print(g);
Serial.print(", A end x: ");
Serial.print(MovPln[g].A_end.x);
Serial.print(", A end y: ");
Serial.print(MovPln[g].A_end.y);
Serial.print(", B end x: ");
Serial.print(MovPln[g].B_end.x);
Serial.print(", B end y: ");
Serial.println(MovPln[g].B_end.y);
}

      // when the counting of the movement plan is ready, then set the state to Movement_plan_exectuion
      movement_plan_state = Movement_plan_exectuion;

      MovPln_execution_ptr = 0;

      // opeing the file for reading, Shall be done here, in this case, the actual file position
      // shall not be registered
      myFile = SD.open(FILE_name); 
    }  
    break;


    case Movement_plan_exectuion:{
      /* the first perimeter point is read in the previous status. Here the heading and turn shall be counted and executed and then moving the mower to the
       * first perimeter point via a straight line.
       * */
       //Serial.println("Execution phase");
      // In the first step the perimeter shall be executed and mowed
      switch(Mov_plan_exec_state){
        case Perimeter_Reading:{                
            char Perimeter_points_array[text_len];
            String Perimeter_points;                                             
            myFile.read(Perimeter_points_array,text_len);
            Perimeter_points = Perimeter_points_array;
            next_point = Coord_from_String(Perimeter_points);
Serial.print("Next point's x: ");
Serial.print(next_point.x);
Serial.print(", next point's y: ");
Serial.println(next_point.y);

            Mov_plan_exec_state = Line_execution;
            act_perimeter++;
          }
          break;


        case Line_reading:{
            /* when a line gets executed, the closest shall be selected and executed.
              * Exept in the case when the first line is to be executed.  
              */

              if(!MovPln[MovPln_execution_ptr].A_end_reached && A_point_selected){
                next_point = MovPln[MovPln_execution_ptr].A_end;
                Mov_plan_exec_state = Line_execution;


              } else if(!MovPln[MovPln_execution_ptr].B_end_reached && B_point_selected){
                next_point = MovPln[MovPln_execution_ptr].B_end;
                Mov_plan_exec_state = Line_execution;


              } else if(MovPln[MovPln_execution_ptr].A_end_reached && MovPln[MovPln_execution_ptr].B_end_reached){
                // searching for the closest point
                float distance_to_A, distance_to_B, min_distance = 10000.0;
                int closest_point_ptr = 0;
                for(int LineToBeCompared = ++MovPln_execution_ptr; LineToBeCompared <= MovPln_pointer; LineToBeCompared++){
                  distance_to_A = sqrt((pow((MovPln[LineToBeCompared].B_end.x - MovPln[MovPln_execution_ptr].A_end.x),2)) + (pow((MovPln[LineToBeCompared].B_end.y - MovPln[MovPln_execution_ptr].A_end.y),2)));
                  distance_to_B = sqrt((pow((MovPln[LineToBeCompared].B_end.x - MovPln[MovPln_execution_ptr].B_end.x),2)) + (pow((MovPln[LineToBeCompared].B_end.y - MovPln[MovPln_execution_ptr].B_end.y),2)));

                  if(min_distance > distance_to_A){
                    min_distance = distance_to_A;
                    // setting the actual lines execution pointer to the closest point                    
                    next_point = MovPln[LineToBeCompared].A_end;
                    Mov_plan_exec_state = Line_execution;
                    A_point_selected = true; 
                    B_point_selected = false;
                    closest_point_ptr = LineToBeCompared;                     
                  }

                  if(min_distance > distance_to_B){
                    min_distance = distance_to_B;
                    // setting the actual lines execution pointer to the closest point                    
                    next_point = MovPln[LineToBeCompared].B_end;
                    Mov_plan_exec_state = Line_execution;
                    A_point_selected = false;
                    B_point_selected = true;
                    closest_point_ptr = LineToBeCompared;

                  }                      
                }
              }       
          }
          break;



        case Line_execution:{
            //Serial.println("Line execution");
            // if an object is ahead, then start the avoiding movement

            object_ahead = false;

            if(object_ahead){
              switch(object_avoiding_state){
                case LIDAR_resuming:{
                    // the LIDAR and the object avoidance shall only be started if the object is closer than the next point
                    struct P_cord obj_coordinate;
                    // initialize the obj coordinate with act position
                    obj_coordinate = actual_position;
                    // then overwrite the y position
                    obj_coordinate.y = actual_position.y + 0.4;
                    if((sqrt((pow((actual_position.x - next_point.x),2)) + (pow((actual_position.y - next_point.y),2)))) > (sqrt((pow((actual_position.x - obj_coordinate.x),2)) + (pow((actual_position.y - obj_coordinate.y),2))))){                        
                      //vTaskResume(xHandleLIDAR);
                      bool LIDAR = true;
                      object_avoiding_state = Correction;
                    } else {
                      object_ahead = false;
                    }
                  }
                  break;

                case Correction: {
                    // if the correction value is received, then the correction shall be counted and then set the state machine to the
                    // next state
                    if(abs(correction_value) > 0.0){
                      next_to_be_saved = next_point;

                      // the correction point shall be initialized with the actual position
                      correction_point = actual_position;

                      // if the previous point y is less than the actual, then the mower is heading "upward".
                      // otherwise "downward"
                      // + correction value is right turn, - correction value is left turn. This shall change according to the heading(upward/downward) of the mower
                      if(actual_position.y > previous_position.y){
                        correction_point.x = correction_point.x + correction_value;
                        correction_point.y = correction_point.y + object_distance_ahead;
                        next_point = correction_point;

                      } else {
                        correction_point.x = correction_point.x - correction_value;
                        correction_point.y = correction_point.y - object_distance_ahead;
                        next_point = correction_point;
                      }

                      object_avoiding_state = Avoiding_Object;
                    }

                  }
                  break;

                case Avoiding_Object: {
                    // if the mower is next to the object, the correction point is reached, then the LIDAR shall be activated again. If the received distance is 1, then the reutn point can be calculated
                    float act_correction_distance = sqrt((pow((actual_position.x - correction_point.x),2)) + (pow((actual_position.y - correction_point.y),2)));
                    if(Coordinate_tolerance < act_correction_distance){
                      //vTaskResume(xHandleLIDAR);
                      bool LIDAR = true;
                    }

                    // if the object end is 1, then the mower is next to the object and it can return to the original path: return_point shall be counted
                    // only the correction data shall be differently added
                    if(object_end == 1.0){
                      return_point = actual_position;
                      if(actual_position.y > previous_position.y){
                        return_point.x = return_point.x - correction_value;
                        return_point.y = return_point.y + object_distance_ahead;
                        next_point = return_point;

                      } else {
                        return_point.x = return_point.x + correction_value;
                        return_point.y = return_point.y - object_distance_ahead;
                        next_point = return_point;
                      }
                    }

                    object_end = 0.0;
                    correction_value = 0.0;
                    object_avoiding_state = Returning_to_P;
                  }
                  break;

                case Returning_to_P:{
                    // if the return point is reached, then the original point shall be aimed again
                    float act_return_point_distance = sqrt((pow((actual_position.x - return_point.x),2)) + (pow((actual_position.y - return_point.y),2)));
                    if(Coordinate_tolerance < act_return_point_distance){
                      next_point = next_to_be_saved;
                      object_avoiding_state = LIDAR_resuming;
                      object_ahead = false;
                    }
                  }
                  break;

              }

            }



            // if the Perimeter point is reached, with a tolerance, then the next point shall be selected, only if the last
            // perimeter point is not reached. If the last is reached, the movement plan shall be executed
            float act_point_distance = sqrt((pow((actual_position.x - next_point.x),2)) + (pow((actual_position.y - next_point.y),2)));
            if(Coordinate_tolerance >= act_point_distance){
              Serial.println("coordinate reached");                  
              if(act_perimeter < number_of_Perimeter_points){
                Mov_plan_exec_state = Perimeter_Reading;
                // the file shall be closed, if all the perimeteres are executed
                Serial.println("State is back to Perimeter reading");
                Point_reached = true;
              } else {
                // state to the next line reading. 
                // selecting the A or B of the next line
                // if the A point is the next to be reached, then no next point shall be selected and reached, just the actual's B.
                // If B is selected, and reached then new point shall be selected.

                if(first_execution){
                    if(A_point_selected){
                      MovPln[MovPln_execution_ptr].A_end_reached = true;
                      A_point_selected = false;
                      Point_reached = true;

                      // if the another end is not yet reached then it shall be selected
                      if(!MovPln[MovPln_execution_ptr].B_end_reached){
                        B_point_selected = true;
                      }


                    } else if(B_point_selected){
                      MovPln[MovPln_execution_ptr].B_end_reached = true;
                      B_point_selected = false;
                      Point_reached = true;

                      // if the another end is not yet reached then it shall be selected
                      if(!MovPln[MovPln_execution_ptr].A_end_reached){
                        A_point_selected = true;
                      }
                    }
                   }

                 // In the first execution, First Line's A point shall be selected.
                 if(!first_execution){
                  A_point_selected = true;
                  first_execution = true;
                 }

                    /* if the last planned line was executed, then finish the execution */ 
                    if((MovPln_execution_ptr == (MovPln_pointer - 1))&&(MovPln[MovPln_execution_ptr].A_end_reached)&&(MovPln[MovPln_execution_ptr].B_end_reached)){

                       Automatic_mode = false;
                       Mov_plan_exec_state = 0;
                       movement_plan_state = 0;

                       MovPln_pointer = 0;
                       MovPln_execution_ptr = 0;
                       myFile.close();
                       /*If the execution is over, the cutter shall be stopped. */ 
                       cutter_data.Source = "Mov_Ctrl";
                       cutter_data.Data = "Cutter_Stop"; 
                       /*If the automatic mode is activated the cutter shall be started. */
                       xQueueSend(cutter_ctrl_queue, &cutter_data, portMAX_DELAY);

                    }                       

                Mov_plan_exec_state = Line_reading;
              }
            }

            time_N = millis();
            if(Automatic_mode_activated){
                time_N_1 = time_N;
            }

            switch(SP_Counting_state){
                  /* after startup, set 100 setpoint */ 
                  case 0:{
                          if(!SP_to_100_exec){
                              /* saving the actual position, when the 100.0 % SP shall be set */
                              previous_position = actual_position;                                  
                              SP_to_100_exec = true;
Serial.println("Saving previous position ");

}
Right_SP = Left_SP = SP_value;
Serial.print("SP 100.0%");
/* counting the SP for the drives
* If there is a difference between the left and the right, start a timer, set a new state.
* If the time is elapsed, then the state machine shall return to the original state and set SP_max again.*/
Serial.print(", time diff: ");
Serial.println(time_N - time_N_1);

if((time_N - time_N_1) > 2000){
SP_Counting_state = 1;

time_N_1 = time_N;
SP_to_100_exec = false;
Serial.println("State to SP counting");

}

}
break;

                  /* counting the setpoints. If one of them is different than 100, then change the state to different SP. */
                  case 1:{
                      if(!count_SP){
                          SP_counting(actual_position, previous_position, next_point, &Right_SP, &Left_SP, SP_value);
                          count_SP = true;
                      }
Serial.print("Counted Right SP: ");
Serial.print(Right_SP);
Serial.print(", Left SP: ");
Serial.print(Left_SP);
Serial.print(", next_point.x : ");
Serial.print(next_point.x);
Serial.print(", next_point.y: ");
Serial.print(next_point.y);
Serial.print(", actual_position.x : ");
Serial.print(actual_position.x);
Serial.print(", actual_position.y: ");
Serial.print(actual_position.y);
Serial.print(", previous_position.x : ");
Serial.print(previous_position.x);
Serial.print(", previous_position.y: ");
Serial.print(previous_position.y);
Serial.print(", time diff: ");
Serial.println(time_N - time_N_1);

                        if((time_N - time_N_1) > 500){
                            if((Right_SP != SP_value)||(Left_SP != SP_value)){
                                  SP_Counting_state = 2;
                                  time_N_1 = time_N;
                                  count_SP = false;
                            } else {
                                  SP_Counting_state = 0;
                                  time_N_1 = time_N;
                                  count_SP = false;
                            }
                        }
                      }
                      break;

                  /* after 1 s, outputting the counted SP, the state shall be back to SP max. */
                  case 2:{
                        if((time_N - time_N_1) > 1){
                              SP_Counting_state = 0;
                              time_N_1 = time_N;
Serial.print("State to 100% SP");
Serial.print(", time diff: ");
Serial.println(time_N - time_N_1);

}

}
break;
}

                Automatic_mode_activated = false;
                Trigger_signal_for_SP_counting = false;
                Point_reached = false;                                                   

          }
          break;
      }
    }
    break;
    }            
}

// creating the setpoints and distances
local_data.source = "Mov_Ctrl";
local_data.Right_SP = Right_SP;
local_data.Left_SP = Left_SP;


/*Serial.print("Sent SP:");
Serial.println(Serial_data);*/

  // sending the setpoints and the distances to CAN Master
  xQueueSend(mot_ctrl_queue, &local_data, portMAX_DELAY);   
}
}

void SP_counting(struct P_cord M_N, struct P_cord M_N_1, struct P_cord P_next, float *Right, float *Left, float SP_in_percent){

const float PI_const =  3.1415926;
// m_n vector couting between M_N and M_N_1
struct P_cord m_n;
m_n.x = M_N.x - M_N_1.x;
m_n.y = M_N.y - M_N_1.y;

// m_n_1 vector couting between M_N_1 and P
struct P_cord m_n_1;
m_n_1.x = P_next.x - M_N_1.x;
m_n_1.y = P_next.y - M_N_1.y;

// p vector counting: difference between m_n  and m_n_1
struct P_cord p;
p.x = m_n_1.x - m_n.x;
p.y = m_n_1.y - m_n.y;

// counting the alpha angle: m_n's angle
float alpha = abs(atan(m_n.y / m_n.x)*180/PI_const);

// counting the beta angle: p's angle
float beta = abs(atan(p.y / p.x)*180/PI_const);

// counting the gamma angle: alpha and beta difference
float gamma = acos(((m_n.x * p.x) + (m_n.y * p.y))/((sqrt(pow(m_n.x,2) + pow(m_n.y,2)))*((sqrt(pow(p.x,2) + pow(p.y,2))))))*180/PI_const;

// deciding in which quarter the m vector is in
int m_quarter = 0;
if((m_n.x >= 0) && (m_n.y >= 0)){
    m_quarter = 1;
} else if((m_n.x < 0) && (m_n.y >= 0)){
    m_quarter = 2;
} else if((m_n.x < 0) && (m_n.y < 0)){
    m_quarter = 3;
} else if((m_n.x >= 0) && (m_n.y < 0)){
    m_quarter = 4;
}

// deciding in which quarter the p vector is in
int p_quarter = 0;
if((p.x >= 0) && (p.y >= 0)){
    p_quarter = 1;
} else if((p.x < 0) && (p.y >= 0)){
    p_quarter = 2;
} else if((p.x < 0) && (p.y < 0)){
    p_quarter = 3;
} else if((p.x >= 0) && (p.y < 0)){
    p_quarter = 4;
}
// counting the parameters of the SP and gamma function
/* It is a linear function. Described by a SP = a * gamma + b formula.
 * If the angle difference is 0, then no change in SP shall follow.
 * At 90 degree difference, the SP shall be 0. At 180 degree max SP in different direction.
 * 
 * = SP_max_percent
 */
 float a, b = SP_in_percent, c = 90.0;
 a = -(b / c);

// deciding the two vectors position to each-other
switch(m_quarter){
    // m is in the first quarter
    case 1:
            switch(p_quarter){
                // p is in the first quarter
                case 1:
                    // if both are in the first quarter, difference between the alpha's and beta's angles will determine the turn of the angle
                    // if alpha is less, then left turn, otherwise right turn
                    if(alpha <= beta){
                        *Left = a * gamma + b;
                        *Right = SP_in_percent;
                    } else {
                        *Left = SP_in_percent;
                        *Right = a * gamma + b;
                    }
                    break;

                // p is in the second quarter
                // this case left turn
                case 2:
                        *Left = a * gamma + b;
                        *Right = SP_in_percent;
                    break;

               // p is in the third quarter
               //difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then right turn, otherwise left turn
                case 3:
                    if(alpha <= beta){
                        *Left = SP_in_percent;
                        *Right = a * gamma + b;
                    } else {
                        *Left = a * gamma + b;
                        *Right = SP_in_percent;
                    }

                    break;

               // p is in the foruth quarter
               // right turn
                case 4:
                        *Left = SP_in_percent;
                        *Right = a * gamma + b;
                    break;
            }
        break;


    // m is in the second quarter
    case 2:
        switch(p_quarter){
                // p is in the first quarter
                // right turn
                case 1:
                        *Left = SP_in_percent;
                        *Right = a * gamma + b;
                    break;

                // p is in the second quarter
                //difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then right turn, otherwise left turn
                case 2:
                        if(alpha <= beta){
                            *Left = SP_in_percent;
                            *Right = a * gamma + b;
                        } else {
                            *Left = a * gamma + b;
                            *Right = SP_in_percent;
                        }
                    break;

               // p is in the third quarter
               // left turn
                case 3:
                        *Left = a * gamma + b;
                        *Right = SP_in_percent;
                    break;

               // p is in the foruth quarter
               //difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then left turn, otherwise right turn
                case 4:
                        if(alpha <= beta){
                            *Left = a * gamma + b;
                            *Right = SP_in_percent;
                        } else {
                            *Left = SP_in_percent;
                            *Right = a * gamma + b;
                        }
                    break;
            }
        break;


    // m is in the third quarter
    case 3:
        switch(p_quarter){
                // p is in the first quarter
                // difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then right turn, otherwise left turn
                case 1:
                        if(alpha <= beta){
                            *Left = SP_in_percent;
                            *Right = a * gamma + b;
                        } else {
                            *Left = a * gamma + b;
                            *Right = SP_in_percent;
                        }
                    break;

                // p is in the second quarter
                // right turn
                case 2:
                        *Left = SP_in_percent;
                        *Right = a * gamma + b;
                    break;

               // p is in the third quarter
               // difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then left turn, otherwise right turn
                case 3:
                        if(alpha <= beta){
                            *Left = a * gamma + b;
                            *Right = SP_in_percent;
                        } else {
                            *Left = SP_in_percent;
                            *Right = a * gamma + b;
                        }
                    break;

               // p is in the foruth quarter
               // left turn
                case 4:
                        *Left = a * gamma + b;
                        *Right = SP_in_percent;
                    break;
            }

        break;


    // m is in the fourth quarter
    case 4:
        switch(p_quarter){
                // p is in the first quarter
                // left turn
                case 1:
                        *Left = a * gamma + b;
                        *Right = SP_in_percent;
                    break;

                // p is in the second quarter
                // difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then left turn, otherwise right turn
                case 2:                       
                        if(alpha <= beta){
                            *Left = a * gamma + b;
                            *Right = SP_in_percent;
                        } else {
                            *Left = SP_in_percent;
                            *Right = a * gamma + b;
                        }
                    break;

               // p is in the third quarter
               // right turn
                case 3:
                        *Left = SP_in_percent;
                        *Right = a * gamma + b;
                    break;

               // p is in the foruth quarter
               // difference between the alpha's and beta's angles will determine the turn of the angle
                // if alpha is less, then left turn, otherwise right tur
                case 4:
                        if(alpha <= beta){
                            *Left = SP_in_percent;
                            *Right = a * gamma + b;
                        } else {
                            *Left = a * gamma + b;
                            *Right = SP_in_percent;
                        }
                    break;
            }
        break;
}
}

struct P_cord Count_Coordinate(float AB_distance, float AC_distance, float BC_distance, float A_distance, float B_distance, float C_distance){
struct P_cord local_data;

const float PI_const = 3.1416;

// in the first step based on the cosinus (tétel) the alpha angle shall be counted
// if angle is needed then *180/PI shall be added, otherwise the result is in radian.

/Serial.print(", A dist: ");
Serial.print(A_distance);
Serial.print(", B dist: ");
Serial.print(B_distance);
Serial.print(", C dist: ");
Serial.print(C_distance);
Serial.print(", AB dist: ");
Serial.print(AB_distance);
Serial.print(", AC dist: ");
Serial.print(AC_distance);
Serial.print(", BC dist: ");
Serial.print(BC_distance);/

float alpha = acos(((A_distance * A_distance) + (AB_distance * AB_distance) - (B_distance * B_distance))/(2 * A_distance * AB_distance));
if(isnan(alpha)){
local_data.valid = false;
} else {
local_data.valid = true;
}

local_data.x = A_distance * cos(alpha);
local_data.y = A_distance * sin(alpha);

/Serial.print(", X coord: ");
Serial.print(local_data.x);
Serial.print(", Y coord: ");
Serial.print(local_data.y);
Serial.print("alpha: ");
Serial.println(alpha);/

return local_data;
}

struct P_cord Coord_from_String(String Perimeter_point_data){
// getting the coordinates from the saved string data
String Comma = ",";
struct P_cord local_data;
String local_string = Perimeter_point_data, X, Y;

X = local_string.substring(0, local_string.indexOf(Comma));
local_data.x = X.toFloat();
//removing the value
local_string.remove(0, local_string.indexOf(Comma) + 1);

Y = local_string.substring(0, local_string.indexOf(Comma));
local_data.y = Y.toFloat();

return local_data;
}

/****************** end of Movement Control task **********************************************************************************************************/

/******************************* Motor control task. ******************************************************************************************************
* The motor control shall write the received SP to the output of the right motor. If the received SP is <0, then the direction shall change.
*/
void mot_ctrl(void *pvParameters)
{
(void) pvParameters;

struct Data_struct received_data;

float Right_SP = 100.0, Left_SP = 100.0;

for (;;)
{
if (xQueueReceive(mot_ctrl_queue, &received_data, portMAX_DELAY) == pdPASS){

  if(received_data.source == "Mov_Ctrl"){
    if(!isnan(received_data.Right_SP)){
      /* in case of sign change the direction of the motor shall change*/
      if(received_data.Right_SP >= 0){
        digitalWrite(MotorRightPin_1, LOW);
        digitalWrite(MotorRightPin_2, HIGH);

      } else {
        digitalWrite(MotorRightPin_1, HIGH);
        digitalWrite(MotorRightPin_2, LOW);
      }
  /*    Serial.print("Right motor: ");
      Serial.print(received_data.Right_SP);*/
      analogWrite(MotorRightPin_PWM, map(abs(received_data.Right_SP),0,100,0,255));
    } else {
      analogWrite(MotorRightPin_PWM, map(abs(Right_SP),0,100,0,255));
    }

    if(!isnan(received_data.Left_SP)){  
      if(received_data.Left_SP >= 0){
        digitalWrite(MotorLeftPin_1, LOW);
        digitalWrite(MotorLeftPin_2, HIGH);

      } else {
        digitalWrite(MotorLeftPin_1, HIGH);
        digitalWrite(MotorLeftPin_2, LOW);
      }
      /*Serial.print(", Left motor: ");
      Serial.print(received_data.Left_SP);*/
      analogWrite(MotorLeftPin_PWM, map(abs(received_data.Left_SP),0,100,0,255));
    } else {
      analogWrite(MotorRightPin_PWM, map(abs(Left_SP),0,100,0,255)); 
    }
  }
}
}
}
/******************************* end of Motor control task. **************************************************************************************************/

/******************************* I2C connection to MPU6050 *************************************************************************************************
* MPU6050 provides the acceleration value in a possible turnover. If that happens all the drives shall stop immediately.
* Probably the acceleration value shall be Kalman filtered.
*/
void MPU6050_task(void *pvParameters)
{
(void) pvParameters;

bool i2c_connection = false;
unsigned long myTime, myTime_N1;
struct Mov_Ctrl_Data local_data;
int16_t ax, ay, az, limit_ax , limit_ay, limit_az;
int16_t gx, gy, gz;

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

accelgyro.initialize();

if(accelgyro.testConnection()){
i2c_connection = true;
} else {
i2c_connection = false;
}

for (;;)
{
if (i2c_connection) {
accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

/*if the to be deined acceleration value exeeds a limit, a data shall be sent to the motor control
 * to let it know an E-stop shall be done.
 */
 if(limit_ax < ax , limit_ay < ay, limit_az < az){
   local_data.Source = "MPU6050";
   local_data.Data = "E-Stop";
   xQueueSend(mot_ctrl_queue, &local_data, portMAX_DELAY);
 }

} else {
/* if the communication is broken, let the motor control and the LCD know */

    local_data.Source = "MPU6050";
    local_data.Data = "Error";

    xQueueSend(mot_ctrl_queue, &local_data, portMAX_DELAY);    
}

vTaskDelay( 70 / portTICK_PERIOD_MS );
}
}
/******************************* end of I2C connection to MPU6050 ****************************************************************/

/******************************* Bluetooth connection *************************************************************************/
void Bluetooth(void *pvParameters)
{
/ All the messqages shall have theri own type otherwise, if only the local_data is used, they get overwritten. */
struct Mov_Ctrl_Data local_data, cutter_data, perimeter_teaching_data, automatic_mode_data, na_teaching_data;
bool UP_cmd_registered = false, DOWN_cmd_registered = false, LEFT_cmd_registered = false, RIGHT_cmd_registered = false, NA_teaching = false, Automatic_mode = false, Perimeter_teaching = false, cutter_ON = false, cutter_CMD_registered = false, Perimeter_teaching_CMD_registered = false, Automatic_mode_CMD_registered = false, NA_teaching_CMD_registered = false;
String cmd;

float SP_min = -100.0, SP_left = 100, SP_left_to_be_Sent = 0.0, SP_right = 100, SP_right_to_be_Sent = 0.0, SP_Left_percentage = 0.0, SP_Right_percentage = 0.0, SP_decreased = 10.0;

for( ;; ) {

while(Serial2.available() > 0){

  char inputByte = Serial2.read();
  Serial.println(inputByte);


  // Right command
  if (inputByte == 'R'){
    RIGHT_cmd_registered = true;          
  }
  if (RIGHT_cmd_registered && (inputByte == '0') && UP_cmd_registered){
    RIGHT_cmd_registered = false; 
    // when the button is released, the SP shall be the normal SP
    SP_left_to_be_Sent = SP_left;
    SP_right_to_be_Sent = SP_right;

  }


  // Left command
  if (inputByte == 'L'){
    LEFT_cmd_registered = true;         
  }
  if (LEFT_cmd_registered && (inputByte == '0') && UP_cmd_registered){
    LEFT_cmd_registered = false;  
    // when the button is released, the SP shall be the normal SP
    SP_left_to_be_Sent = SP_left;
    SP_right_to_be_Sent = SP_right;    
  }


  // Up command
  if (inputByte == 'U'){
    UP_cmd_registered = true;
    // when up command is registered, then the SP shall be set.
    SP_Left_percentage = SP_left_to_be_Sent = SP_left;
    SP_Right_percentage = SP_right_to_be_Sent = SP_right; 

    }

  /*if (UP_cmd_registered && (inputByte == '0')){
    UP_cmd_registered = false;
  }*/

  // Down command
  if (inputByte == 'D'){
    DOWN_cmd_registered = true;
    UP_cmd_registered = false;
    // when up command is registered, then the SP shall be set.
    SP_left_to_be_Sent = 0.0;
    SP_right_to_be_Sent = 0.0;   

    }
  if (DOWN_cmd_registered && (inputByte == '0')){
    DOWN_cmd_registered = false;
  }



  // NA teaching
  if ((inputByte == 'X')&&(!NA_teaching_CMD_registered)){
    NA_teaching = true;
    na_teaching_data.Source = "Manual_NA";
    na_teaching_data.Data = "Start"; 
    xQueueSend(mov_ctrl_queue, &na_teaching_data, portMAX_DELAY);
  }

  if (NA_teaching && (inputByte == '0')){
    NA_teaching_CMD_registered = true;
  }

  if(NA_teaching_CMD_registered && (inputByte == 'X')){
    NA_teaching = false;
    NA_teaching_CMD_registered = false;
    na_teaching_data.Source = "Manual_NA";
    na_teaching_data.Data = "Finish"; 
    xQueueSend(mov_ctrl_queue, &na_teaching_data, portMAX_DELAY);
  }



  // automatic mode
  if((inputByte == 'A')&&(!Automatic_mode_CMD_registered)){
    Automatic_mode = true;
    automatic_mode_data.Source = "Manual_Automatic";
    automatic_mode_data.Data = "ON"; 
    xQueueSend(mov_ctrl_queue, &automatic_mode_data, portMAX_DELAY);
  }

  if (Automatic_mode && (inputByte == 'M')){
    Automatic_mode_CMD_registered = true;
  }

  if(Automatic_mode_CMD_registered && (inputByte == 'A')){
    Automatic_mode_CMD_registered = false;
    Automatic_mode = false;
    automatic_mode_data.Source = "Manual_Automatic";
    automatic_mode_data.Data = "OFF"; 
    xQueueSend(mov_ctrl_queue, &automatic_mode_data, portMAX_DELAY);
  }
