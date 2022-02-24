#include <Arduino.h>

#include <TFT_eSPI.h>
#include <SPI.h>


TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
//135x240 display


// Bluetooth
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;



const byte SwitchPin  = 13 ;
const byte VinPin = 32;
const byte OnOffPin = 33;

const float Ratio = 0.00476190476  ;

float Vin = 0 ;
float OnOff = 0;
const float OffV = 10.0 ;
const float OnVin = 11.0;
const float Trigger = 1600;

bool Sensing = false ;
bool Switch = false;
bool Switching = false;

//millis
const long SensingTime = 1000 ;
const long SwitchingTime = 500;

unsigned long CurrentTime = 0 ;
unsigned long SensingStartTime = 0 ;
unsigned long SwitchStartTime = 0;

unsigned long test = 0;
static long int timestamp = 0;
static long int timestamp1 = 0;
static long int timestamp2 = 0;
static long int timestamp3 = 0;


//Smoothing,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
float VinSmooth = 0;
int inputPin = 32;


//Table De Configuration,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
float x=0;
float voltage=0;
// Takes ADC input, returns voltage (float) by looking up calibration table
//example: voltage = adc_to_v(2755); 

// Function prototypes
float adc_to_v(uint16_t);
void Bluetooth();
void BTWrite();
void Smoothing();
void InverterControl();


// Function definition
float adc_to_v(uint16_t x){     //10.07kh and 2.169kh resistors for ESP32-TTGO
  static const double xs[] = {0,1650,1709,1765,1819,1875,1931,1986,2044, 2100, 2157, 2212, 2267, 2321, 2378, 2434, 2488, 2544, 2607, 2663, 2718, 2774, 2831, 2890, 2950, 3010, 3070, 3135, 3200, 3264};
  static const double ys[] = {0,8.00,8.25,8.50,8.75,9.00,9.25,9.50,9.75,10.00,10.25,10.50,10.75,11.00,11.25,11.50,11.75,12.00,12.25,12.50,12.75,13.00,13.25,13.50,13.75,14.00,14.25,14.50,14.75,15.00};                                                               
    static const int count = sizeof(xs) / sizeof(xs[0]);   // number of elements in the array
  int i;
  double dx, dy;
  if (x < xs[0]){                     // x is less than the minimum element
                                      // * handle error here if you want 
    return ys[0];                     // return minimum element
  }
  if (x > xs[count - 1]){
    return ys[count - 1]; // return maximum
  }
   for (i = 0; i < count - 1; i++){        // find i, such that xs[i] <= x < xs[i+1]
      if (xs[i + 1] > x) {
          break;
         }
  }
    dx = xs[i + 1] - xs[i];                   // interpolate
  dy = ys[i + 1] - ys[i];
  return ys[i] + (x - xs[i]) * dy / dx;
};

//...................................................................................
void setup() {
     Serial.begin(9600);
      pinMode(SwitchPin, OUTPUT);
      pinMode (VinPin, INPUT);
      pinMode (OnOffPin, INPUT);

      //Screen
      tft.init();
      tft.setRotation(3);                   //Screen orientation
      tft.fillScreen(TFT_BLACK);
      tft.setSwapBytes(true);


      //Blutetooth
      SerialBT.begin("TheRazzInverterControler"); //Bluetooth device name
      Serial.println("The device started, now you can pair it with bluetooth!");
      
     }
//...................................................................................
void loop() {
  
     
    if (millis() - timestamp > 1){
      Smoothing();
      float voltage =  adc_to_v(average);          //Configuration Chart
      InverterControl();
      VinSmooth = average * Ratio ;
      
      timestamp = millis();
      }
    if (millis() - timestamp1 > 100){
      
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    //  tft.setTextColor(TFT_ORANGE);
      tft.drawString("The Razz",0,0,4);     //Font #4
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
     // tft.setTextColor(TFT_CYAN);
      String VoltageDisplay = "Voltage : " + String(VinSmooth) + "v" ;
      tft.drawString(VoltageDisplay, 20, 60,4 );
      if (OnOff >= 200.0){tft.drawString("Inverter Off", 20, 90, 4);}
      else {tft.drawString("Inverter On", 20, 90, 4); }
      timestamp1 = millis();
      }

      //Bluetooth
      if (millis() - timestamp2 > 20){
        Bluetooth();
        timestamp2 = millis();
        }

        if (millis() - timestamp3 > 1000){
          BTWrite();
          timestamp3 = millis();
          
          }
}

//-----------------------------------------------------------------------------------------------
void Bluetooth() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  
}
void BTWrite (){
SerialBT.print  (Vin, 2); SerialBT.println("v");
  SerialBT.print ("ADC Value Vin   : "); SerialBT.println (analogRead(VinPin));
  SerialBT.print ("ADC Value OnOff : "); SerialBT.println(analogRead(OnOffPin));
  SerialBT.print ("CurrentTime         : "); SerialBT.println (CurrentTime);
  SerialBT.print ("StartSwitchTime     : "); SerialBT.println (SwitchStartTime);
  test = CurrentTime - SwitchStartTime;
  SerialBT.print ("TimeSinceSwitchTime : "); SerialBT.println (test);
  SerialBT.print("SwitchPin  : ");SerialBT.println (digitalRead(SwitchPin));
  SerialBT.print("Switching  : "); SerialBT.println (Switching);
  SerialBT.print("Sensing    : "); SerialBT.println(Sensing);
  
  SerialBT.println ("...");
}

//-------------------------------------------------------------------------------------------------
void Smoothing(){
    total = total - readings[readIndex];           // subtract the last reading:
    readings[readIndex] = analogRead(inputPin);      // read from the sensor:
    total = total + readings[readIndex];              // add the reading to the total:
    readIndex = readIndex + 1;                        // advance to the next position in the array:

  if (readIndex >= numReadings) {              // if we're at the end of the array...
        readIndex = 0;                          // ...wrap around to the beginning:
  }

  average = total / numReadings;               // calculate the average:
  Serial.print("average         : ");Serial.println(average); 
  Serial.print(average * Ratio);Serial.println("v");  
}

//-----------------------------------------------------------------------------------
void InverterControl() {
  CurrentTime = millis();
  Vin = analogRead(VinPin)  * Ratio;
  OnOff = analogRead(OnOffPin);

  if (Switching == false && Sensing == false && Vin <= OffV && OnOff <= Trigger ){
       SensingStartTime = millis() ;
       Sensing = true ;
       Serial.println ("Sensing");
      }
if ( Sensing == true && CurrentTime - SensingStartTime >= SensingTime){
  if ( Vin <= OffV && OnOff <= Trigger ) {
       SwitchStartTime = millis() ;
       digitalWrite(SwitchPin, HIGH) ;
       Serial.println ("Switching Off");
       Sensing = false ;
       Switching = true;
      }
  else {
       Sensing = false ;
       }
   }

if (Switching == false && Vin >= OnVin && OnOff >= Trigger) {
    SwitchStartTime =millis() ;
    digitalWrite(SwitchPin, HIGH) ;
    Serial.println ("Switching On");
    Switching = true;
               }

if( Switching == true && CurrentTime - SwitchStartTime >= SwitchingTime ) {
    digitalWrite ( SwitchPin, LOW) ;
    Switching = false;
     }

  Serial.print  (Vin, 2); Serial.println("v");
  Serial.print ("ADC Value Vin   : "); Serial.println (analogRead(VinPin));
  Serial.print ("ADC Value OnOff : "); Serial.println(analogRead(OnOffPin));
  Serial.print ("CurrentTime         : "); Serial.println (CurrentTime);
  Serial.print ("StartSwitchTime     : "); Serial.println (SwitchStartTime);
  test = CurrentTime - SwitchStartTime;
  Serial.print ("TimeSinceSwitchTime : "); Serial.println (test);
  Serial.print("SwitchPin  : ");Serial.println (digitalRead(SwitchPin));
  Serial.print("Switching  : "); Serial.println (Switching);
  Serial.print("Sensing    : "); Serial.println(Sensing);
  
  Serial.println ("...");
}
