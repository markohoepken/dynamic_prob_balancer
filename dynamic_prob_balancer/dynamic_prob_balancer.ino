/***************************************************
 
 
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <Servo.h> 
#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050


#define TFT_CS     10
#define TFT_RST    8  
#define TFT_DC     9
#define TFT_SCLK 13   
#define TFT_MOSI 11   

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

#define KEY1 A0
#define KEY2 A1
#define KEY3 A2
#define KEY4 A3
#define PRESS 0

#define POTI A7
#define MOTOR 2
#define POSITION 3
uint16_t rotation=0;
unsigned long timeold;

#define MOTOR_MIN 33 //  1ms position
#define MOTOR_TEST 50 // test speed
#define MOTOR_MAX 55 //  max save speed
//#define MOTOR_MAX 119 // 2ms position

int poti=0;
Servo  motor;



void setup(void) {

  // keyboard
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);
  pinMode(KEY3, INPUT);
  pinMode(KEY4, INPUT);  
  // pull up
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
  // set i2c to 400KHz
  //Wire.setclock(400000L);
  TWBR = 24;  
  // RPM measure / position
  attachInterrupt(1, count_rotation, FALLING);
  rotation=0;
  // debug display
  Serial.begin(115200);
}

uint16_t last_poti=65535;
uint16_t servo_pos=0;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int16_t AcY_max=0;
int16_t AcY_min=0;
int16_t rpm=0;
bool motor_pulse=false;
#define UPDATE_SCREEN 50
uint8_t update_screen=UPDATE_SCREEN;

uint16_t measure_index=0;
uint16_t reference_position=0;
#define MEASURE_ARRAY 100
int16_t AcY_measure[MEASURE_ARRAY];
uint8_t index_pos[MEASURE_ARRAY];

int16_t current_time = 0;
int16_t last_time = 0;

unsigned long rotation_time = 0;

void loop(void) {


    if(AcY_max<AcY)
    {
        AcY_max=AcY;
    }
    if(AcY_min>AcY)
    {
        AcY_min=AcY;
    }    
    //Serial.println("AcX = "); Serial.print(AcX);    
    #if 0
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)    
    #endif
    
    
    #if 0

    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
    #endif 
    //delay(333);
    
    if((digitalRead(KEY2) == PRESS)) // dump results on COM
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
    
    
    if((digitalRead(KEY3) == PRESS)) // run test
    {
        // run test
        motor.write(MOTOR_TEST); 
        delay(1000);
        AcY_measure[measure_index]=read_y_sensor();
        delay(1000);

        // check RPM
        rotation_time=current_time-last_time;
        // wait for sensor index marker
        motor_pulse=false;
        //while(!motor_pulse)
        //{};
        motor_pulse=false;
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
        for(uint8_t index=0;index<MEASURE_ARRAY;index++)
        {
            Serial.print(index); 
            Serial.print(";"); 
            Serial.print(AcY_measure[index]);            
            Serial.print(";");                        
            Serial.println(index_pos[index]);
        }        
        
        motor.write(MOTOR_MIN); 
        // measurements done
        
        // wait until release
        //while((digitalRead(KEY3) != PRESS));  
        // motor off
        motor.write(MOTOR_MIN);        
    }
    else
    {
        poti=analogRead(POTI);
        servo_pos = map(poti, 0, 1023, MOTOR_MIN, MOTOR_MAX);     // scale it to use it with the servo (value between 0 and 180) 
        motor.write(servo_pos);     
    }
    
    // update on changed poti or key 4
//    if((update_screen--==0) || (digitalRead(KEY4) == PRESS))
    if(digitalRead(KEY4) == PRESS)
    {
        uint8_t y_pos=0;
        #define SIZE 20
        tft.fillRect(0, 0, 127, 100, ST7735_BLACK); // clear
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
        rotation_time=current_time-last_time;
        tft.print(rotation_time);         
    }
    // reset value
    if((digitalRead(KEY1) == PRESS))
    {
        AcY_max=0;
        AcY_min=0;
        rotation=0;
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

void count_rotation()
{
    rotation++;
    motor_pulse=true;
    last_time=current_time;
    current_time=millis();
}
