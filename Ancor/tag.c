
#include <SPI.h>
#include "DW1000Ranging.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 7; // spi select pin

int array_lenght = 40;
bool a_active = false, b_active = false, c_active = false;
float a_distance = 1.0, b_distance = 1.0, c_distance = 1.0;
String a_string = "1.0",b_string = "1.0", c_string = "1.0", final_string = "1.0,1.0,1.0,";
unsigned int current_time = 0, last_time = 0;
float a_arr[40], b_arr[40], c_arr[40];
float a_avg = 1.0, b_avg = 1.0, c_avg = 1.0;


void setup() {
  
  for(int i = 0; i < array_lenght; i++){
    a_arr[i] = 1.0;
    b_arr[i] = 1.0;
    c_arr[i] = 1.0;
  }
  
  Serial.begin(115200);
  delay(1000);
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  //Serial.println("started");
  
  //we start the module as a tag
  DW1000Ranging.startAsTag("0T:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop(){
  DW1000Ranging.loop();
  //Serial.println("loop");
  current_time = millis();
  if(current_time - last_time > 500)
  {
    a_avg = 0; b_avg = 0; c_avg = 0;
    
    for(int j = 0; j < array_lenght; j++)
    {
      a_avg+= a_arr[j];
      b_avg+= b_arr[j];
      c_avg+= c_arr[j];
    }
    
    a_avg = a_avg / array_lenght;
    b_avg = b_avg / array_lenght;
    c_avg = c_avg / array_lenght;
    
  if(a_active){
    a_string = String(a_avg);
  } else {
    a_string = "Err";
  }
  
  if(b_active){
    b_string = String(b_avg);
  } else {
    b_string = "Err";
  } 
    
  if(c_active){
    c_string = String(c_avg);
  } else {
    c_string = "Err";
  }
    
    
    
    final_string = a_string + "," + b_string + "," + c_string + "\n";
    Serial.print(final_string);
    last_time = current_time;
  }
}

void newRange() {
  uint16_t id = DW1000Ranging.getDistantDevice()->getShortAddress();
  float current_distance = DW1000Ranging.getDistantDevice()->getRange();
  if (id == 10){update_a();   a_arr[array_lenght - 1] = current_distance; }
  if (id == 11){update_b();   b_arr[array_lenght - 1] = current_distance; }
  if (id == 12){update_c();   c_arr[array_lenght - 1] = current_distance; }
  
}


void update_a()
{
  for (int i = 1; i < array_lenght; i++)
  {
    a_arr[i-1] = a_arr[i];
  }  
}

void update_b()
{
  for (int i = 1; i < array_lenght; i++)
  {
    b_arr[i-1] = b_arr[i];
  }  
}

void update_c()
{
  for (int i = 1; i < array_lenght; i++)
  {
    c_arr[i-1] = c_arr[i];
  }  
}




void newDevice(DW1000Device* device){
  int address = device->getShortAddress();
  if(address == 10){
    a_active = true;
  } else if(address == 11){
    b_active = true;
  } else if(address == 12){
    c_active = true;
  }
}

void inactiveDevice(DW1000Device* device){
  int address = device->getShortAddress();
  if(address == 10){
    a_active = false;
  } else if(address == 11){
    b_active = false;
  } else if(address == 12){
    c_active = false;
  }
  
}
