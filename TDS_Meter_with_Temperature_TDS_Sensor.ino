
#include <LiquidCrystal.h> //Libraries
LiquidCrystal lcd(2, 3, 4, 5, 6, 7); //Arduino pins to lcd

#define tds_sensor A0
#define DS18B20_PIN  A5 // for Arduino microcontroller

#define G_led 8 
#define R_led 9 
#define buzzer 13

float aref = 4.3;
float ecCalibration = 1;

float ec = 0;
unsigned int tds = 0;

int raw_temp;
float waterTemp = 0;

void setup(){
pinMode(tds_sensor, INPUT);

pinMode(R_led,OUTPUT); // declare Red LED as output
pinMode(G_led,OUTPUT); // declare Green LED as output
pinMode(buzzer,OUTPUT); // declare Buzzer as output 

lcd.begin(16, 2); // Configura lcd numero columnas y filas
lcd.clear();
lcd.setCursor (0,0);
lcd.print("   Welcome To   ");
lcd.setCursor (0,1);
lcd.print("TDS & Temp Meter"); 
delay(2000);
lcd.clear();
}
 
void loop(){

if(ds18b20_read(&raw_temp)) {
waterTemp = (float)raw_temp / 16;     // Convert temperature raw value into degree Celsius (temp in °C = raw/16)
}
float rawEc = analogRead(tds_sensor) * aref / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
float temperatureCoefficient = 1.0 + 0.02 * (waterTemp - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
ec = (rawEc / temperatureCoefficient) * ecCalibration; // temperature and calibration compensation
tds = (133.42 * pow(ec, 3) - 255.86 * ec * ec + 857.39 * ec) * 0.5; //convert voltage value to tds value

lcd.clear();
lcd.setCursor(0,0);
lcd.print("TDS   EC   Temp");
lcd.setCursor(0,1); 
lcd.print(tds); 
lcd.setCursor(5,1); 
lcd.print(ec, 2); 
lcd.setCursor(11,1); 
lcd.print(waterTemp,2); 

if(tds<50 || tds>700){ 
digitalWrite(buzzer, HIGH);
digitalWrite(G_led, LOW); // Turn LED off.   
digitalWrite(R_led, HIGH);  // Turn LED on.
delay(300);
}else{  
digitalWrite(G_led, HIGH); // Turn LED on.  
digitalWrite(R_led, LOW);  // Turn LED off.
}
  
digitalWrite(buzzer, LOW);
delay(500); 
}
 
bool ds18b20_start(){bool ret = 0;
  digitalWrite(DS18B20_PIN, LOW);     // Send reset pulse to the DS18B20 sensor
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(500);             // Wait 500 us
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(100);             //wait to read the DS18B20 sensor response
  if (!digitalRead(DS18B20_PIN)) {
    ret = 1;                          // DS18B20 sensor is present
    delayMicroseconds(400);           // Wait 400 us
  }
  return(ret);
}

void ds18b20_write_bit(bool value){
  digitalWrite(DS18B20_PIN, LOW);
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(2);
  digitalWrite(DS18B20_PIN, value);
  delayMicroseconds(80);
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(2);
}

void ds18b20_write_byte(byte value){
  byte i;
  for(i = 0; i < 8; i++)
    ds18b20_write_bit(bitRead(value, i));
}

bool ds18b20_read_bit(void) {
  bool value;
  digitalWrite(DS18B20_PIN, LOW);
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(2);
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(5);
  value = digitalRead(DS18B20_PIN);
  delayMicroseconds(100);
  return value;
}

byte ds18b20_read_byte(void) {
  byte i, value;
  for(i = 0; i  <8; i++)
    bitWrite(value, i, ds18b20_read_bit());
  return value;
}

bool ds18b20_read(int *raw_temp_value) {
  if (!ds18b20_start())                     // Send start pulse
    return(0);                              // Return 0 if error
  ds18b20_write_byte(0xCC);                 // Send skip ROM command
  ds18b20_write_byte(0x44);                 // Send start conversion command
  while(ds18b20_read_byte() == 0);          // Wait for conversion complete
  if (!ds18b20_start())                     // Send start pulse
    return(0);                              // Return 0 if error
  ds18b20_write_byte(0xCC);                 // Send skip ROM command
  ds18b20_write_byte(0xBE);                 // Send read command
  *raw_temp_value = ds18b20_read_byte();    // Read temperature LSB byte and store it on raw_temp_value LSB byte
  *raw_temp_value |= (unsigned int)(ds18b20_read_byte() << 8);     // Read temperature MSB byte and store it on raw_temp_value MSB byte
  return(1);                                // OK --> return 1
}
