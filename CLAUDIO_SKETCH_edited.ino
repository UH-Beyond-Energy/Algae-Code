#include <DFRobot_PH.h>
#include <EEPROM.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

/////////Pin Assignment
#define PH_PIN A8
#define MG_PIN A9     //define which analog input channel you are going to use
#define DHTPIN 7     // what pin we're connected to
//#define BOOL_PIN 2  //Not in use
#define ONE_WIRE_BUS 6 // water temp sensor pin

#define DHTTYPE DHT22   // DHT 22  (AM2302)
DFRobot_PH ph;
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino


//water temp sensor header
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);




//pinMode(BOOL_PIN, INPUT);             ///Not in use
//digitalWrite(BOOL_PIN, HIGH);        ///Not in use

//////Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value
float voltage;
float phValue;
float temperature=25;
int percentage;
float volts;  
float Celcius=0;
float Fahrenheit=0;
int relayAgitatorpin = 3;

///////Constants
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in
                                                     //normal operation
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.220) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
float CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
                                                //two points are taken from the curve.
                                                //with these two points, a line is formed which is
                                                //"approximately equivalent" to the original curve.
                                                //data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280)
                                                //slope = ( reaction voltage ) / (log400 –log1000)
//-------







float CO2Read(int mg_pin)
{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;
}

int  CO2Percentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}

String TempAlarm(float temp)
{
    String warning;

    if (temp > 30 ) {
        warning = "WARNING HIGH" ;
    } 
    else if (temp< 26 ){
        warning = "WARNING LOW";
    }
    return warning;
}


String WaterTempAlarm(float Fahrenheit)
{
    String warning;

    if (Fahrenheit > 78 ) {
        warning = "WARNING HIGH" ;
    } 
    else if (Fahrenheit <76  ){
        warning = "WARNING LOW";
    }
    return warning;
}
String phAlarm(float phValue)
{
    String warning;

    if (phValue > 8.99 ) {
        warning = "WARNING HIGH" ;
    } 
    else if (phValue < 7.99 ){
        warning = "WARNING LOW";
    }
    return warning;
}

String HumAlarm(float hum)
{
    String warning;

    if (hum > 95 ) {
        warning = " WARNING HIGH" ;
    } 
    else if (hum < 50 ){
        warning = " WARNING LOW";
    }
    return warning;
}

String CO2Check(float percent)
{
    String value;

     if (percentage == -1) {
        value = "<400";
    } else {
        value = percentage;
    }  

    return value;
}

void setup()
{
  ph.begin();
  dht.begin();
  
  Serial.begin(9600);
  delay(200);
  Serial.print("");

  Serial.begin(9600);
  sensors.begin();

  pinMode(relayAgitatorpin, OUTPUT);
  digitalWrite(relayAgitatorpin, HIGH);

 
}

void loop()

     {static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
        timepoint = millis();
        voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
        phValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation
        Serial.println("pH:");
        Serial.print (phValue,2);
         Serial.print(" ");
        Serial.print( phAlarm(phValue));// this function warns if ph is low or high
    }
    ph.calibration(voltage,temperature);  // calibration process by Serail CMD


    delay(1000);
  

    //Print humidity value to serial monitor 
    hum = dht.readHumidity();
    Serial.println("Humidity:");
    Serial.print(hum);
    Serial.print("% ");
    Serial.println(HumAlarm(hum));// this function checks if humidity is within desired range


    delay(1000); //Delay 1 sec.


    //Print temperature value to serial monitor
    temp= dht.readTemperature();
    Serial.print(" Air Temp: ");
    Serial.print(temp);
    Serial.println(" Celsius");
    Serial.print(TempAlarm(temp));// this function checks if temperature is within desired range
    

    delay(1000); //Delay 1 sec.


    volts = CO2Read(MG_PIN);
    //Serial.print(volts);   testing
    //Serial.print( "V" );   testing
    percentage = CO2Percentage(volts,CO2Curve);
    Serial.print("CO2:");
    Serial.print(CO2Check(percentage));// Check that percentage reading is in formula range: percent not = to -1
    Serial.println( "ppm" );
    Serial.print("\n");

    delay(1000); // Loop Delay

  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  Fahrenheit=sensors.toFahrenheit(Celcius);

  Serial.print(" Water Temp F=  ");
  Serial.print(Fahrenheit);
   Serial.print(" ");
  Serial.print(WaterTempAlarm(Fahrenheit));
  
  delay(1000);

digitalWrite(relayAgitatorpin, HIGH);// THIS FUNCTION IS TO START 30MIN AGITATOR CYCLES
  Serial.print("AGITATOR ON FOR 30 MIN");
  delay(5000);
  digitalWrite(relayAgitatorpin, LOW);
  Serial.print("AGITATOR OFF FOR 30 MIN");
  delay(5000);
  

}
