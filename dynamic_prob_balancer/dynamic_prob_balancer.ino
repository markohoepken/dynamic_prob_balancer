/***************************************************
Dynamic rc copter or plane balancing project.
The prob is mounted to a motor, that is controlled by an ESC.

The motor is mounted on a camera damper.
This allows to  "wobble" a little on bad (most cheap props are bad) probeller.

The movement is dynamically measured by  MPU-6050 sensor.
The motor position is monitored by a optical sensor watching to the motor.

When tbe motor is turning the software measures the missbalance and shows on what side
of the propeller the missalignment is.

By adding tape on the other blade, the balancing takes place.
To fix the balance run the process interative, until the balance is done.

The advantage if this device is that you get real measures and you know the SIDE where
to add weight.

Each run takes just 3 seconds. 

Each round you get a good indicator to add or remove weight.

Now prob balanceing is fun an save (no longer "touch" the motor to "feel" the missbalnace.) 
 
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>             // required for the display
#include <Servo.h>           // required for the ESC
#include<Wire.h>             // required for the MPU-6050
const int MPU=0x68;          // I2C address of the MPU-6050

// pinning of display
#define TFT_CS     10
#define TFT_RST    8  
#define TFT_DC     9
#define TFT_SCLK 13   
#define TFT_MOSI 11   

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);


// keyboard
#define KEY1 A0
#define KEY2 A1
#define KEY3 A2
#define KEY4 A3
#define PRESS 0

#define POTI A7
#define MOTOR 2     // pulse output to controll the ESC
#define POSITION 7
#define POSITION_REF_OUT 5
uint16_t rotation=0;
unsigned long timeold;

// each ESC does have different min / max position.
// here you can tweak to your ESC
#define MOTOR_MIN 33 //  1ms position
#define MOTOR_TEST 50 // test speed
#define MOTOR_MAX 55 //  max save speed to balance
//#define MOTOR_MAX 119 // 2ms position

int poti=0;
Servo  motor;



void setup(void) {

  // keyboard
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);
  pinMode(KEY3, INPUT);
  pinMode(KEY4, INPUT);  
  // pull up for keyboard
  digitalWrite(KEY1, HIGH);
  digitalWrite(KEY2, HIGH);
  digitalWrite(KEY3, HIGH);
  digitalWrite(KEY4, HIGH);
  // position sensor
  //pinMode(POSITION, INPUT);  
  
  // Display
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextWrap(false); // Allow text to run off right edge
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(2);
  // ESC Motor controller
  motor.attach(MOTOR);
  // Acc meter MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // NOTE: 400 kHz is not required, the sensor is not fast enough to get more data.
  // set i2c to 400KHz
  //Wire.setclock(400000L);
  // TWBR = 24;  
  // RPM measure / position
  //attachInterrupt(1, count_rotation, FALLING);
  
  // setup analog compare
  ACSR = 
  (0<<ACD) |   // Analog Comparator: Enabled
  (0<<ACBG) |   // Analog Comparator Bandgap Select: AIN0 is applied to the positive input
  (0<<ACO) |   // Analog Comparator Output: Off
  (0<<ACI) |   // Analog Comparator Interrupt Flag: Clear Pending Interrupt
  (1<<ACIE) |   // Analog Comparator Interrupt: Enabled
  (0<<ACIC) |   // Analog Comparator Input Capture: Disabled
  (1<<ACIS1) | (0<<ACIS0);   // Analog Comparator Interrupt Mode: Comparator Interrupt on Rising Output Edge

   // set reference to 50%
   analogWrite(POSITION_REF_OUT, 128);
  
  rotation=0;
  // debug display
  Serial.begin(115200);
}

uint16_t last_poti=65535;
uint16_t servo_pos=0;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int16_t AcY_max=0;
int16_t AcY_min=0;
int8_t balance_side=0; // -1=left 0=undefined  +1=right
int16_t rpm=0;
bool motor_pulse=false;
#define UPDATE_SCREEN 50
uint8_t update_screen=UPDATE_SCREEN;

uint16_t measure_index=0;
uint16_t reference_position=0;
#define MEASURE_ARRAY 100
int16_t AcY_measure[MEASURE_ARRAY];
uint8_t index_pos[MEASURE_ARRAY];

int32_t current_time = 0;
int32_t last_time = 0;
int32_t next_trigger = 0;
#define POSTITION_FILTER 5

int32_t rotation_time = 0;
int32_t test_rotation_time = 0;

void loop(void) {

    // reset value
    if((digitalRead(KEY1) == PRESS))
    {
        AcY_max=0;
        AcY_min=0;
        rotation=0;
    }
    if((digitalRead(KEY2) == PRESS)) // dump results on COM for debugging
    {
        for(uint8_t index=0;index<MEASURE_ARRAY;index++)
        {
            Serial.print(index); 
            Serial.print(":"); 
            Serial.println(AcY_measure[index]);
        }
        // wait until release
        while((digitalRead(KEY2) != PRESS));
    }        
    if((digitalRead(KEY3) == PRESS)) // run test with ESC controll
    {
        measure_index=0; // reset test counter
        // run test
        motor.write(MOTOR_TEST); 
        // wait until motor is fully running
        delay(3000);
        //AcY_measure[measure_index]=read_y_sensor();
        //delay(1000);

        // check RPM
        test_rotation_time=rotation_time;

        // start probing
        for(uint8_t index=0;index<MEASURE_ARRAY;index++)
        {
            AcY_measure[measure_index]=read_y_sensor();
            if(motor_pulse){
                index_pos[measure_index]=1;
                motor_pulse=false;                
            }
            else
            {
                index_pos[measure_index]=0;
            }
            measure_index++;  
            //delay(10);
        }
        #if 1
        for(uint8_t index=0;index<MEASURE_ARRAY;index++)
        {
            Serial.print(index); 
            Serial.print(";"); 
            Serial.print(AcY_measure[index]);            
            Serial.print(";");                        
            Serial.println(index_pos[index]);
        }        
        #endif
        motor.write(MOTOR_MIN); 
        // measurements done
        // do anaysis, find min and max and polarity
        AcY_max=AcY_measure[0];
        AcY_min=AcY_measure[0];
        uint8_t marker_side=0;
        uint8_t not_marker_side=0;
        for(uint8_t index=0;index<MEASURE_ARRAY;index++)
        {
            if(AcY_max<AcY_measure[index])
            {
                AcY_max=AcY_measure[index];
            }
            if(AcY_min>AcY_measure[index])
            {
                AcY_min=AcY_measure[index];
            }            
       
            // check for polarity on index

            if(index_pos[index]) 
            {
                if(AcY_measure[index]>0)
                {
                    marker_side++;
                }
                else
                {
                not_marker_side++; 
                }
            }
        }
        // summary
        if(marker_side && not_marker_side)
        {
            balance_side=0; // undefined
        }
        if(marker_side)
        {
            balance_side=1;
        }
        else
        {
            balance_side=-1;
        }
    }
    else
    {
        poti=analogRead(POTI);
        servo_pos = map(poti, 0, 1023, MOTOR_MIN, MOTOR_MAX);     // scale it to use it with the servo (value between 0 and 180) 
        motor.write(servo_pos);     
    }
    
    // update from time to time, and on Keypress
    if((update_screen--==0) || (digitalRead(KEY4) == PRESS))
//    if(digitalRead(KEY4) == PRESS)
    {
        update_screen=UPDATE_SCREEN;
        uint8_t y_pos=0;
        #define SIZE 20
        //tft.fillRect(0, 0, 127, 100, ST7735_BLACK); // clear
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, y_pos);
        y_pos+=SIZE;
        tft.print("Servo:");
        tft.print(servo_pos);       
        tft.setCursor(0, y_pos);        
        y_pos+=SIZE;  
        tft.print("val:");
        tft.print(read_y_sensor());       
        tft.setCursor(0, y_pos);        
        y_pos+=SIZE;        
        tft.print("Min:");
        tft.print(AcY_min);       
        tft.setCursor(0, y_pos);        
        y_pos+=SIZE;
        tft.print("Max:");
        tft.print(AcY_max);      
        tft.setCursor(0, y_pos);        
        y_pos+=SIZE;        
        tft.print("ROT:");
        tft.print(rotation);         
        tft.setCursor(0, y_pos);        
        y_pos+=SIZE;        
        tft.print("TIME:");
        //rotation_time=current_time-last_time;
        tft.print(test_rotation_time);  
        tft.setCursor(0, y_pos);  
        y_pos+=SIZE;        
        tft.print("SIDE:");
        tft.print(balance_side);       
    }


}

int16_t read_y_sensor()
{
    int16_t value;
    // sensor stuff
    Wire.beginTransmission(MPU);
    //    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.write(0x3D);  // starting with register 0x3D (ACCEL_YOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,2,true);  // request a total of 2 registers
    value=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_yOUT_H) & 0x3e (ACCEL_OUT_L)   
    return(value);
}

ISR(ANALOG_COMP_vect )
{
    current_time=millis();
    if(current_time > next_trigger)
    {
        rotation++;
        motor_pulse=true;
        next_trigger=current_time+POSTITION_FILTER; // some ms delay as filter
        rotation_time=current_time-last_time;
        last_time=current_time;
    }

}

