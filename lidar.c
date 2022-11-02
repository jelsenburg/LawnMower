#include <TimerOne.h>
#include<mcp2515.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L1X.h>


Servo scanning;  
Servo vertical;

VL53L1X sensor;

struct can_frame Received_data, Sent_data;
MCP2515 mcp2515(10);                // the pin for the CAN shall be defined

float LIDAR_distance = 0.0, LIDAR_angle = 0.0;
byte Life = 0, Scanning = 0, Output_1 = 0, Output_2 = 0, Output_3 = 0;
int act_scan_angle = 0, scanning_step_angle = 10, Scanning_min = 30, Scanning_max = 150, Vertical_Scan_pos = 90, Vertical_banked_pos = 150, act_distance = 0, detection_limit = 430;



void setup(){
    scanning.attach(40);
    vertical.attach(41);

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    sensor.setTimeout(500);
    sensor.init();
    delay(200);
    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
  
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor.startContinuous(50);
    
    Timer1.initialize(100000);         // initialize timer1 in microseconds
    Timer1.attachInterrupt(timer_interrupt);  // attaches callback() as a timer overflow interrupt

    Serial.begin(115200);
  
  // initializing the CAN message
    Received_data.can_id  = 0x051;    // CAN ID
    Received_data.can_dlc = 8;      // CAN DLC
    Received_data.data[0] = 0x00;   // Speed value byte 0
    Received_data.data[1] = 0x00;   // Speed value byte 1
    Received_data.data[2] = 0x00;   // Speed value byte 2
    Received_data.data[3] = 0x00;   // Speed value byte 3
    Received_data.data[4] = 0x00;   // Empty byte
    Received_data.data[5] = 0x00;   // Empty byte
    Received_data.data[6] = 0x00;   // Empty byte
    Received_data.data[7] = 0x00;   // Life singal.


    Sent_data.can_id  = 0x061;    // CAN ID
    Sent_data.can_dlc = 8;      // CAN DLC
    Sent_data.data[0] = 0x00;   // Speed value byte 0
    Sent_data.data[1] = 0x00;   // Speed value byte 1
    Sent_data.data[2] = 0x00;   // Speed value byte 2
    Sent_data.data[3] = 0x00;   // Speed value byte 3
    Sent_data.data[4] = 0x00;   // Empty byte
    Sent_data.data[5] = 0x00;   // Empty byte
    Sent_data.data[6] = 0x00;   // Empty byte
    Sent_data.data[7] = 0x00;   // Life singal.
  
  
  // starting the CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();

  
}

void loop(){
  if (mcp2515.readMessage(&Received_data) == MCP2515::ERROR_OK){
    if(Received_data.can_id == 0x061){
      Scanning = Received_data.data[0];
      Output_1 = Received_data.data[1];
      Output_2 = Received_data.data[2];
      Output_3 = Received_data.data[3];
          
    }
    
    Serial.print(Received_data.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(Received_data.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<Received_data.can_dlc; i++)  {  // print the data
      Serial.print(Received_data.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }

  if(Output_1){
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }

  if(Output_2){
    digitalWrite(3, HIGH);
  } else {
    digitalWrite(3, LOW);
  }

  if(Output_3){
    digitalWrite(4, HIGH);
  } else {
    digitalWrite(4, LOW);
  }
}


void timer_interrupt(void){
  if(Scanning){

    if(act_scan_angle >= Scanning_max){
      act_scan_angle = Scanning_min;
    }
    
    scanning.write(act_scan_angle);
    act_scan_angle+=scanning_step_angle;
    vertical.write(Vertical_Scan_pos);

    LIDAR_distance = sensor.read();
    LIDAR_angle = act_scan_angle;
    
  } else {
    scanning.write((Scanning_min + Scanning_max)/2);
    vertical.write(Vertical_Scan_pos);
    act_scan_angle = Scanning_min;
  }

  Sent_data.data[0] = ((uint8_t*)&LIDAR_distance)[0];    // Speed value byte 0
  Sent_data.data[1] = ((uint8_t*)&LIDAR_distance)[1];    // Speed value byte 1
  Sent_data.data[2] = ((uint8_t*)&LIDAR_angle)[0];       // Speed value byte 2
  Sent_data.data[3] = ((uint8_t*)&LIDAR_angle)[1];       // Speed value byte 3
  Sent_data.data[7] = Life++;
    
  mcp2515.sendMessage(&Sent_data);
  Serial.println("CAN msg sent");
}