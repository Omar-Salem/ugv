// /*
//   Arduino DC Voltage Demo 1
//   dc-voltage-demo.ino
//   Use Arduino A/D converter to measure voltage
//   Use external voltage divider with 30k & 7.5k resistors
//   Results displayed on Serial Monitor

//   DroneBot Workshop 2021
//   https://dronebotworkshop.com
// */
// #include <Wire.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>

// // Define analog input
// #define ANALOG_IN_PIN 13

// #define OLED_WIDTH 128
// #define OLED_HEIGHT 64
// #define OLED_ADDR   0x3C

// // Floats for ADC voltage & Input voltage
// float adc_voltage = 0.0;
// float in_voltage = 0.0;

// // Floats for resistor values in divider (in ohms)
// float R1 = 30000.0;
// float R2 = 7500.0; 

// // Float for Reference Voltage
// float ref_voltage = .89;

// // Integer for ADC value
// int adc_value = 0;

// Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);
// void setup(){

//   display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
// }

// void loop(){
//    // Read the Analog Input
//    adc_value = analogRead(ANALOG_IN_PIN);
   
//    // Determine voltage at ADC input
//    adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
   
//    // Calculate voltage at divider input
//    in_voltage = adc_voltage / (R2/(R1+R2)); 
   
//   display.clearDisplay();

//   display.setTextSize(1);
//   display.setTextColor(WHITE);
//   display.setCursor(0, 0);
//   display.print("Voltage:");
//   display.println(in_voltage);

//   display.display();
  
//   // Short delay
//   delay(500);
// }
