
// CAN
#include<mcp2515.h>

// Usonic
#include <Usonic.h>

struct can_frame Usonic_data;
MCP2515 mcp2515(10);								// the pin for the CAN shall be defined

int distance_1 = 0, distance_2 = 0;


void setup() {
	
	Timer1.initialize(100000);         // initialize timer1 in microseconds
    Timer1.attachInterrupt(timer_interrupt);  // attaches callback() as a timer overflow interrupt
	
	Usonic_setup();
	Serial.begin(57600);
  
  // initializing the CAN message
    Usonic_data.can_id  = 0x041;    // CAN ID
    Usonic_data.can_dlc = 8;      // CAN DLC
    Usonic_data.data[0] = 0x00;   // Speed value byte 0
    Usonic_data.data[1] = 0x00;   // Speed value byte 1
    Usonic_data.data[2] = 0x00;   // Speed value byte 2
    Usonic_data.data[3] = 0x00;   // Speed value byte 3
    Usonic_data.data[4] = 0x00;   // Empty byte
    Usonic_data.data[5] = 0x00;   // Empty byte
    Usonic_data.data[6] = 0x00;   // Empty byte
    Usonic_data.data[7] = 0x00;   // Life singal.

  
  
	// starting the CAN
	mcp2515.reset();
	mcp2515.setBitrate(CAN_1000KBPS);
	mcp2515.setNormalMode();

  
}

void loop() {
	Usonic_loop();
	
	distance_1 = Usonic_1_distance();
	distance_2 = Usonic_2_distance();	
	
	Serial.print("Usonic 1: ");
	Serial.print(distance_1);
	Serial.print(", Usonic 2: ");
	Serial.print(distance_2);
  
}


void timer_interrupt(){
  // sending the CAN message
  
	Usonic_data.data[0] = ((uint8_t*)&distance_1)[0];    // Speed value byte 0
    Usonic_data.data[1] = ((uint8_t*)&distance_1)[1];    // Speed value byte 1
    Usonic_data.data[2] = ((uint8_t*)&distance_2)[0];    // Speed value byte 2
    Usonic_data.data[3] = ((uint8_t*)&distance_2)[1];    // Speed value byte 3
  
    mcp2515.sendMessage(&Usonic_data);
}
