#include <Arduino.h>
#include <OneWire.h>

#define HEATPIN 5
#define COOLPIN 3
#define COOLLIGHT 9
#define HEATLIGHT 10

OneWire  ds(2);  // on pin 10 (a 4.7K resistor is necessary)
uint32_t time_now = millis();
uint32_t last_temp_time = 0;
uint32_t last_correction = 0;
float setpoint = 20.0; 
float old_setpoint = 21.0;
bool first = true;
bool heating = false;
bool cooling = false;


void setup(void) {
  Serial.begin(9600);
}

void start_conversion(byte addr[8]){
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
}

float get_data(byte addr[8]){
  float celsius;
  byte i;
  byte data[9];
  byte present = 0;
  byte type_s;
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
   
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  } 
  
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
    
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  celsius = (float)raw / 16.0;
  return celsius;
}

void heaton(){
  heating = true;
  digitalWrite(HEATPIN, HIGH);
  digitalWrite(HEATLIGHT, HIGH);
}

void heatoff(){
  heating = false;
  digitalWrite(HEATPIN, LOW);
  digitalWrite(HEATLIGHT, LOW);
}


void coolon(){
  cooling = true;
  digitalWrite(COOLPIN, HIGH);
  digitalWrite(COOLLIGHT, HIGH);
}

void cooloff(){
  cooling = false;
  digitalWrite(COOLPIN, LOW);
  digitalWrite(COOLLIGHT, LOW);
}

void stop_h(){
  cooling = false;
  heating = false;
  digitalWrite(HEATPIN, LOW);
  digitalWrite(COOLPIN, LOW);
  digitalWrite(HEATLIGHT, LOW);
  digitalWrite(COOLLIGHT, LOW);
}


void loop(void) {
  byte i;
  byte data[9];
  byte addr[8] = {0x28, 0xB8, 0xEB, 0xFC, 0x0D, 0x00, 0x00, 0x61,};
  float celsius, fahrenheit;
  time_now = millis();

  if (Serial.available() > 0) {
      setpoint = Serial.parseFloat(SKIP_ALL, '\n');
      first = true;
      if(setpoint > old_setpoint){
        
      }
  }
  
  start_conversion(addr);
  delay(1000);
  celsius = get_data(addr);

  Serial.print("Temp:");
  Serial.print(celsius);
  Serial.print(" ");
  Serial.print("Setpoint:");
  Serial.print(setpoint);
  Serial.print(" Heating:");
  Serial.print(heating);
  Serial.print(" Cooling:");
  Serial.println(cooling);
  
  if(setpoint < celsius-3){
    heatoff();
    coolon();
  }
  else{
    if((time_now - last_temp_time) > 60000){
      last_temp_time = millis();
      cooloff();
    }
  }
  
  if(setpoint > celsius){
    cooloff();
    heaton();
  }
  else{
    heatoff();
  }
 
  
  
}