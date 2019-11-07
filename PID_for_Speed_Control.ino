% PID Speed Control for 6 DC Motors

#define InA1            24                      // INA motor 1 pin
#define InB1            26                      // INB motor 1 pin
#define InA2            22                       // INA motor 2 pin
#define InB2            28                       // INA motor 2 pin
#define PWM1            4                       // PWM motor 1 pin
#define PWM2            5                       // PWM motor 2 pin
#define encodPinA1      2                       // encoder A pin motor 1
#define encodPinB1      29                       // encoder B pin motor 1
#define encodPinA2      3                       // encoder A pin motor 2
#define encodPinB2      31                       // encoder A pin motor 2
#define InA3            42                     // INA motor 1 pin
#define InB3            48                    // INB motor 1 pin
#define PWM3           	9


#define Vpin            0                       // battery monitoring analog pin
#define Apin            1                       // motor current monitoring analog pin


#define CURRENT_LIMIT   100000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

int readings[NUMREADINGS];
unsigned long lasttime=0;
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint1 = 0;               // loop timing
unsigned long lastMilliPrint2 = 0; 
int speed_req1 =-10; // speed (Set Point) positive value is down (-50:-5::5:50 at max)
int y_direc=-1;
int speed_act1 = 0;                              // speed (actual value)
int speed_act2=0;
int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2=0; 
int PWM_val_load = 0; 
int load_req = 20;
float load_act = 0;                       
int voltage = 0;                                // in mV
int current = 0;                                // in mA
int comma1_index;
int comma2_index;
int comma3_index;
int comma4_index;
int contactTime = 0;
int y_velocity = 0;
int y_position = 0;
volatile long count1 = 0;                        // rev counter
volatile long count2 =0;
float Kp1 =   0.4;                                // PID proportional control Gain
float Kd1 =    1;                                // PID Derivitave control gain
float Kp2 =  0.4;                                // PID proportional control Gain
float Kd2 =    1; 
float Ki2 =    0;
float Kp3 =  0.4;                                // PID proportional control Gain
float Kd3 =    0.5; 
float Kp4 =  0.015;                                // PID proportional control Gain
float Kd4 =  0.0015; 

bool doCommand1 = true;
bool doCommand3 = true;

String string1;
String string2;
String initialParameters;
String setpoint_load_string;
String z_velocity_string;
String y_velocity_string;
String y_position_string;
String contactTime_string;
String content;

void setup() {
 analogReference(EXTERNAL);                            // Current external ref is 3.3V
 Serial.begin(115200);
 Serial.setTimeout(1);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT);
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(0, rencoder1, FALLING);
//myPID.SetMode(AUTOMATIC);
 pinMode(InA2, OUTPUT);
 pinMode(InB2, OUTPUT);
 pinMode(PWM2, OUTPUT);
 pinMode(InA3, OUTPUT);
 pinMode(InB3, OUTPUT);
 pinMode(PWM3, OUTPUT);
 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(1, rencoder2, FALLING);
// delay(15000);
 for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0
if (speed_req1<=0){
 analogWrite(PWM1, PWM_val1);
 digitalWrite(InA1, HIGH);
 digitalWrite(InB1, LOW);
  analogWrite(PWM2, PWM_val2);
 digitalWrite(InA2, HIGH);
 digitalWrite(InB2, LOW);
}
else
{
  analogWrite(PWM1, PWM_val1);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, HIGH);
  analogWrite(PWM2, PWM_val2);
 digitalWrite(InA2, LOW);
 digitalWrite(InB2, HIGH);
}

}

void loop() {
 
if (doCommand1 == true) {

 content = Serial.readStringUntil('#');
  while (content.indexOf('#') == -1) {
    content = Serial.readStringUntil('#');  
 }
  initialParameters = Serial.read();
  comma1_index = initialParameters.indexOf(',');
  comma2_index = initialParameters.indexOf(',', comma1_index + 1);
  comma3_index = initialParameters.indexOf(',', comma2_index + 1);
  comma4_index = initialParameters.indexOf(',', comma3_index + 1);
  setpoint_load_string = initialParameters.substring(0,comma1_index - 1);
 load_req = setpoint_load_string.toInt();
  z_velocity_string = initialParameters.substring(comma1_index + 1, comma2_index - 1);
  speed_req1 = z_velocity_string.toInt();
  y_velocity_string = initialParameters.substring(comma2_index + 1, comma3_index - 1);
  y_velocity = z_velocity_string.toInt();
  y_position_string = initialParameters.substring(comma3_index + 1, comma4_index - 1);
  y_position = y_position_string.toInt();
  contactTime_string = initialParameters.substring(comma4_index + 1);
  contactTime = contactTime_string.toInt();
  
  string2 = "Load Setpoint: " + String(load_req) + "Z Velocity: " + String(speed_req1);
  Serial.println(string2);
  
  doCommand1 = false;
}    

 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
  load_act = Serial.parseInt(); 
  load_act = Serial.parseFloat(); // when above line is uncommented and this line commented code runs
  string1 = String(millis()) + "\t" + String(load_act) + "\t" + String(PWM_val_load) + "\t" + String(speed_act1) + "\t" + String(speed_act2);
  Serial.println(string1);
  lastMilli = millis();
    if (load_act < 3) // Christian edit: maybe edit this to be if load_act < load_req
{
      if(doCommand3 == true) {
       getMotorData1();                                                           // calculate speed, volts and Amps
       PWM_val1= updatePid1(PWM_val1, speed_req1, speed_act1);                        // compute PWM value
       analogWrite(PWM1, PWM_val1);                                               // send PWM to motor
       getMotorData2();
       PWM_val2=updatePid2(PWM_val2, speed_req1, speed_act2);
       analogWrite(PWM2, PWM_val2);
       lasttime=millis();
      PWM_val_load = (PWM_val2 + PWM_val1) / 2;                        // compute PWM value
      }
    }
   else {
     doCommand3 = false;
     
     getMotorData1();                                                           // calculate speed, volts and Amps
     getMotorData2();
     Serial.println((millis()-lasttime));
    if((millis()-lasttime)<500)
    {
   
           if(y_direc<0)
           {
           digitalWrite(InA3, LOW);
               digitalWrite(InB3, HIGH);
           analogWrite(PWM3,50);
           delay(1000);
           analogWrite(PWM3,0);
           }
            else
            {
              digitalWrite(InA3, HIGH);
               digitalWrite(InB3, LOW);
           analogWrite(PWM3,50);
           delay(1000);
           analogWrite(PWM3,0);
            }
            
    }    
      else
       {
        
      analogWrite(PWM3,0);
       }
       
     PWM_val_load = updateLoadPid(PWM_val_load, load_req, load_act);                        // compute PWM value
     if (PWM_val_load>=0){
         analogWrite(PWM1, PWM_val1);
         digitalWrite(InA1, LOW);
         digitalWrite(InB1, HIGH);
         analogWrite(PWM2, PWM_val2);
         digitalWrite(InA2, LOW);
         digitalWrite(InB2, HIGH);
         analogWrite(PWM1, PWM_val_load); // send PWM to motor
         analogWrite(PWM2, PWM_val_load); 
        }
        else
        {
         // analogWrite(PWM1, PWM_val1);
         digitalWrite(InA1, HIGH);
         digitalWrite(InB1, LOW);
          analogWrite(PWM2, PWM_val2);
         digitalWrite(InA2, HIGH);
         digitalWrite(InB2, LOW);
        analogWrite(PWM1, PWM_val_load); // send PWM to motor
        analogWrite(PWM2, PWM_val_load);  
        }
   }
    printMotorInfo1();                                                           // display data
    printMotorInfo2();
 }
}

void getMotorData1()  {                                                        // calculate speed, volts and Amps
static long countAnt1 = 0;                                                   // last count
 speed_act1 = ((count1 - countAnt1)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt1 = count1;                  
 voltage = int(analogRead(Vpin) * 3.22 * 12.2/2.2);                          // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
 current = int(analogRead(Apin) * 3.22 * .77 *(1000.0/132.0));               // motor current - output: 130mV per Amp
current = digital_smooth(current, readings);                                // remove signal noise
}

void getMotorData2()  {                                                        // calculate speed, volts and Amps
static long countAnt2 = 0;                                                   // last count
 speed_act2 = ((count2 - countAnt2)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt2 = count2;                  
 voltage = int(analogRead(Vpin) * 3.22 * 12.2/2.2);                          // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
 current = int(analogRead(Apin) * 3.22 * .77 *(1000.0/132.0));               // motor current - output: 130mV per Amp
 //current = digital_smooth(current, readings);                                // remove signal noise
}
int updatePid1(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error1=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp1 * error) + (Kd1 * (error - last_error1));                            
 last_error1 = error;
 
 return constrain(command + int(pidTerm), 0,255);
 
}

int correct=0;

int updatePid2(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error2=0;                             
 error = abs(targetValue) - abs(currentValue); 
 
 
 pidTerm = (Kp2 * error) + (Kd2 * (error - last_error2)) + Ki2*(-Ki2*last_error2+error)*LOOPTIME;                            
 last_error2 = error;
 correct = updatePid2error();
 return constrain(command + int(pidTerm)+correct, 0, 255);
 
}

int updatePid2error(){
  float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error3=0;                             
 error = abs(count1) - abs(count2); 
 
 
 pidTerm = (Kp3 * error) + (Kd3 * (error - last_error3)) ;                            
 last_error3 = error;
 return constrain(int(pidTerm), -3, 3);
}

int updateLoadPid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error4=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp4 * error) + (Kd4 * (error - last_error4));                            
 last_error4 = error;
 
 return constrain(command + int(pidTerm), -255,255);
 
}

void printMotorInfo1()  {                                                      // display data
 if((millis()-lastMilliPrint1) >= 50)   {                     
   lastMilliPrint1 = millis();
   Serial.print("SP:");             Serial.print(speed_req1);  
   Serial.print("  RPM:");          Serial.print(speed_act1);
   Serial.print("  PWM:");          Serial.print(PWM_val1);  
   Serial.print("  count1:");          Serial.print(count1); 
  Serial.print("  V:");            Serial.print(float(voltage)/1000,1);
  Serial.print("  mA:");           Serial.println(current);

   if (current > CURRENT_LIMIT)               Serial.println("*** CURRENT_LIMIT ***");                
   if (voltage > 1000 && voltage < LOW_BAT)   Serial.println("*** LOW_BAT ***");                
 }
}

void printMotorInfo2()  {                                                      // display data
 if((millis()-lastMilliPrint2) >= 50)   {                     
   lastMilliPrint2 = millis();
  Serial.print("SP2:");             Serial.print(speed_act1);  
  Serial.print("  RPM2:");          Serial.print(speed_act2);
   Serial.print("  PWM2:");          Serial.print(PWM_val2); 
    Serial.print("  count2:");          Serial.print(count2);  
   Serial.print("  V2:");            Serial.print(float(voltage)/1000,1);
   Serial.print("  mA2:");           Serial.println(current);

   if (current > CURRENT_LIMIT)               Serial.println("*** CURRENT_LIMIT ***");                
   if (voltage > 1000 && voltage < LOW_BAT)   Serial.println("*** LOW_BAT ***");                
 }
}
void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
  if(digitalRead(encodPinB1)==HIGH)   count1 ++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
 else                      count1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
if(digitalRead(encodPinB2)==HIGH)   count2 ++;               // if(digitalRead(encodPinB1)==HIGH)   count ++;
 else                      count2--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

  int getParam()  {
  char param, cmd;
   if(!Serial.available())    return 0;
  delay(10);                  
   param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 0;
   cmd = Serial.read();                                // get command byte
   Serial.flush();
  switch (param) {
    case 'v':                                         // adjust speed
       if(cmd=='+')  {
         speed_req1 += 20;
        if(speed_req1>400)   speed_req1=400;
      }
       if(cmd=='-')    {
         speed_req1 -= 20;
        if(speed_req1<0)   speed_req1=0;
      }
      break;
    case 's':                                        // adjust direction
       if(cmd=='+'){
         digitalWrite(InA1, LOW);
         digitalWrite(InB1, HIGH);
      }
       if(cmd=='-')   {
         digitalWrite(InA1, HIGH);
         digitalWrite(InB1, LOW);
      }
       break;
     case 'o':                                        // user should type "oo"
      digitalWrite(InA1, LOW);
       digitalWrite(InB1, LOW);
       speed_req1 = 0;
       break;
     default: 
       Serial.println("???");
     }
  }

int digital_smooth(int value, int *data_array)  {    // remove signal noise
static int ndx=0;                                                         
static int count3=0;                          
static int total=0;                          
 total -= data_array[ndx];               
 data_array[ndx] = value;                
 total += data_array[ndx];               
 ndx = (ndx+1) % NUMREADINGS;                                
 if(count1 < NUMREADINGS)      count1++;
 return total/count1;
}
