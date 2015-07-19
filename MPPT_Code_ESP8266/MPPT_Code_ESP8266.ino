//----------------------------------------------------------------------------------------------------
// ARDUINO MPPT SOLAR CHARGE CONTROLLER (Version-3) 
//  Author: Debasish Dutta/deba168
//          www.opengreenenergy.in
//
//  This code is for an arduino Nano based Solar MPPT charge controller.
//  This code is a modified version of sample code from www.timnolan.com
//  updated 06/07/2015
//
//  Mods by Aplavins 19/06/2015 

////  Specifications :  //////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                                                                            //
//    1.Solar panel power = 50W                                            
                                                                                                                            //
//    2.Rated Battery Voltage= 12V ( lead acid type )

//    3.Maximum current = 5A                                                                                                //

//    4.Maximum load current =10A                                                                                            //

//    5. In put Voltage = Solar panel with Open circuit voltage from 17 to 25V                                               //

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




#include "TimerOne.h"                // using Timer1 library from http://www.arduino.cc/playground/Code/Timer1
#include <LiquidCrystal_I2C.h>      // using the LCD I2C Library from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#include <Wire.h>  
#include <SoftwareSerial.h>         // using the Software Serial library Ref : http://www.arduino.cc/en/Reference/SoftwareSerialConstructor
//----------------------------------------------------------------------------------------------------------
 
//////// Arduino pins Connections//////////////////////////////////////////////////////////////////////////////////

// A0 - Voltage divider (solar)
// A1 - ACS 712 Out
// A2 - Voltage divider (battery)
// A4 - LCD SDA
// A5 - LCD SCL
// D2 - ESP8266 Tx
// D3 - ESP8266 Rx through the voltage divider
// D5 - LCD back control button
// D6 - Load Control 
// D8 - 2104 MOSFET driver SD
// D9 - 2104 MOSFET driver IN  
// D11- Green LED
// D12- Yellow LED
// D13- Red LED

// Full scheatic is given at http://www.instructables.com/files/orig/F9A/LLR8/IAPASVA1/F9ALLR8IAPASVA1.pdf

///////// Definitions /////////////////////////////////////////////////////////////////////////////////////////////////


// Turn this on to use the ESP8266 chip. If you set this to 0, the periodic updates will not happen
#define ENABLE_DATALOGGER 0

// Load control algorithm
// 0 - NIGHT LIGHT: Load ON when there is no solar power and battery is above LVD (low voltage disconnect)
// 1 - POWER DUMP: Load ON when there is solar power and the battery is above BATT_FLOAT (charged)
#define LOAD_ALGORITHM 0

#define SOL_AMPS_CHAN 1                // Defining the adc channel to read solar amps
#define SOL_VOLTS_CHAN 0               // defining the adc channel to read solar volts
#define BAT_VOLTS_CHAN 2               // defining the adc channel to read battery volts
#define AVG_NUM 8                      // number of iterations of the adc routine to average the adc readings

// ACS 712 Current Sensor is used. Current Measured = (5/(1024 *0.185))*ADC - (2.5/0.185) 

#define SOL_AMPS_SCALE  0.026393581        // the scaling value for raw adc reading to get solar amps   // 5/(1024*0.185)
#define SOL_VOLTS_SCALE 0.029296875        // the scaling value for raw adc reading to get solar volts  // (5/1024)*(R1+R2)/R2 // R1=100k and R2=20k
#define BAT_VOLTS_SCALE 0.029296875        // the scaling value for raw adc reading to get battery volts 

#define PWM_PIN 9                    // the output pin for the pwm (only pin 9 avaliable for timer 1 at 50kHz)
#define PWM_ENABLE_PIN 8            // pin used to control shutoff function of the IR2104 MOSFET driver (hight the mosfet driver is on)
#define PWM_FULL 1023                // the actual value used by the Timer1 routines for 100% pwm duty cycle
#define PWM_MAX 100                  // the value for pwm duty cyle 0-100%
#define PWM_MIN 60                  // the value for pwm duty cyle 0-100% (below this value the current running in the system is = 0)
#define PWM_START 90                // the value for pwm duty cyle 0-100%
#define PWM_INC 1                    //the value the increment to the pwm value for the ppt algorithm

#define TRUE 1
#define FALSE 0
#define ON TRUE
#define OFF FALSE

#define TURN_ON_MOSFETS digitalWrite(PWM_ENABLE_PIN, HIGH)      // enable MOSFET driver
#define TURN_OFF_MOSFETS digitalWrite(PWM_ENABLE_PIN, LOW)      // disable MOSFET driver

#define ONE_SECOND 50000            //count for number of interrupt in 1 second on interrupt period of 20us

#define LOW_SOL_WATTS 5.00          //value of solar watts // this is 5.00 watts
#define MIN_SOL_WATTS 1.00          //value of solar watts // this is 1.00 watts
#define MIN_BAT_VOLTS 11.00         //value of battery voltage // this is 11.00 volts          
#define MAX_BAT_VOLTS 14.10         //value of battery voltage// this is 14.10 volts
#define BATT_FLOAT 13.60            // battery voltage we want to stop charging at
#define HIGH_BAT_VOLTS 13.00        //value of battery voltage // this is 13.00 volts 
#define LVD 11.5                    //Low voltage disconnect setting for a 12V system
#define OFF_NUM 9                   // number of iterations of off charger state
  
//------------------------------------------------------------------------------------------------------
//Defining led pins for indication
#define LED_RED 11
#define LED_GREEN 12
#define LED_YELLOW 13
//-----------------------------------------------------------------------------------------------------
// Defining load control pin
#define LOAD_PIN 6       // pin-2 is used to control the load
  
//-----------------------------------------------------------------------------------------------------
// Defining lcd back light pin
#define BACK_LIGHT_PIN 5       // pin-5 is used to control the lcd back light

// ---------------------------For ESP8266--------------------------------------------------------------

// replace with your channel's thingspeak API key
String apiKey = "DPK8RMTFY2B1XCAF";
// connect 2 to TX of Serial USB
// connect 3 to RX of serial USB

SoftwareSerial ser(2,3); // RX, TX
//---------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------
/////////////////////////////////////////BIT MAP ARRAY//////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------

byte battery_icons[6][8]=
{{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
}};
#define SOLAR_ICON 6
byte solar_icon[8] = //icon for termometer
{
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b00000
};
#define PWM_ICON 7
byte _PWM_icon[8]=
{
  0b11101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10111,
};
byte backslash_char[8]= 
{
  0b10000,
  0b10000,
  0b01000,
  0b01000,
  0b00100,
  0b00100,
  0b00010,
  0b00010,
};
//-------------------------------------------------------------------------------------------------------

// global variables

float sol_amps;                       // solar amps 
float sol_volts;                      // solar volts 
float bat_volts;                      // battery volts 
float sol_watts;                      // solar watts
float old_sol_watts = 0;              // solar watts from previous time through ppt routine 
unsigned int seconds = 0;             // seconds from timer routine
unsigned int prev_seconds = 0;        // seconds value from previous pass
unsigned int interrupt_counter = 0;   // counter for 20us interrrupt
unsigned long time = 0;               // variable to store time the back light control button was pressed in millis
int delta = PWM_INC;                  // variable used to modify pwm duty cycle for the ppt algorithm
int pwm = 0;                          // pwm duty cycle 0-100%
int back_light_pin_State = 0;         // variable for storing the state of the backlight button
boolean load_status = false;          // variable for storing the load output state (for writing to LCD)
  
enum charger_mode {off, on, bulk, bat_float} charger_state;    // enumerated variable that holds state for charger state machine
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


//------------------------------------------------------------------------------------------------------
// This routine is automatically called at powerup/reset
//------------------------------------------------------------------------------------------------------

void setup()                           // run once, when the sketch starts
{
  pinMode(PWM_ENABLE_PIN, OUTPUT);     // sets the digital pin as output
  TURN_OFF_MOSFETS;                    // turn off MOSFET driver chip
  charger_state = off;                 // start with charger state as off
  lcd.begin(20,4);                     // initialize the lcd for 16 chars 2 lines, turn on backlight

  // create the LCD special characters. Characters 0-5 are the various battery fullness icons
  // icon 7 is for the PWM icon, and icon 8 is for the solar array
  lcd.backlight();
  for (int batchar = 0; batchar <   6; ++batchar) {
    lcd.createChar(batchar, battery_icons[batchar]);
  }
  lcd.createChar(PWM_ICON,_PWM_icon);
  lcd.createChar(SOLAR_ICON,solar_icon);
  lcd.createChar('\\', backslash_char);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  Timer1.initialize(20);               // initialize timer1, and set a 20uS period
  Timer1.pwm(PWM_PIN, 0);              // setup pwm on pin 9, 0% duty cycle
  Timer1.attachInterrupt(callback);    // attaches callback() as a timer overflow interrupt
  Serial.begin(9600);                  // open the serial port at 9600 bps:
  ser.begin(9600);                     // enable software serial
  ser.println("AT+RST");               // reset ESP8266
  pwm = PWM_START;                     //starting value for pwm  
  pinMode(BACK_LIGHT_PIN, INPUT);
  pinMode(LOAD_PIN,OUTPUT);
  digitalWrite(LOAD_PIN,LOW);          // default load state is OFF
  digitalWrite(BACK_LIGHT_PIN,LOW);    //  default LCd back light is OFF

  // display the constant stuff on the LCD
  lcd.setCursor(0, 0);
  lcd.print("SOL");
  lcd.setCursor(4, 0);
  lcd.write(SOLAR_ICON);
  lcd.setCursor(8, 0);
  lcd.print("BAT");
}

//------------------------------------------------------------------------------------------------------
// Main loop
//------------------------------------------------------------------------------------------------------
void loop()
{
  read_data();                         // read data from inputs
  run_charger();                       // run the charger state machine
  print_data();                        // print data
  load_control();                      // control the connected load
  led_output();                        // led indication
  lcd_display();                       // lcd display
#if ENABLE_DATALOGGER
  wifi_datalog();                    // sends data to thingspeak
#endif
}

//------------------------------------------------------------------------------------------------------
// This routine reads and averages the analog inputs for this system, solar volts, solar amps and 
// battery volts. 
//------------------------------------------------------------------------------------------------------
int read_adc(int channel){
  
  int sum = 0;
  int temp;
  int i;
  
  for (i=0; i<AVG_NUM; i++) {          // loop through reading raw adc values AVG_NUM number of times  
    temp = analogRead(channel);        // read the input pin  
    sum += temp;                       // store sum for averaging
    delayMicroseconds(50);             // pauses for 50 microseconds  
  }
  return(sum / AVG_NUM);               // divide sum by AVG_NUM to get average and return it
}

//------------------------------------------------------------------------------------------------------
// This routine reads all the analog input values for the system. Then it multiplies them by the scale
// factor to get actual value in volts or amps. 
//------------------------------------------------------------------------------------------------------
void read_data(void) {
  
  sol_amps = (read_adc(SOL_AMPS_CHAN) * SOL_AMPS_SCALE -13.51);    //input of solar amps
  sol_volts = read_adc(SOL_VOLTS_CHAN) * SOL_VOLTS_SCALE;          //input of solar volts 
  bat_volts = read_adc(BAT_VOLTS_CHAN) * BAT_VOLTS_SCALE;          //input of battery volts 
  sol_watts = sol_amps * sol_volts ;                               //calculations of solar watts                  
}

//------------------------------------------------------------------------------------------------------
// This is interrupt service routine for Timer1 that occurs every 20uS.
//
//------------------------------------------------------------------------------------------------------
void callback()
{
  if (interrupt_counter++ > ONE_SECOND) {        // increment interrupt_counter until one second has passed
    interrupt_counter = 0;                       // reset the counter
    seconds++;                                   // then increment seconds counter
  }
}

//------------------------------------------------------------------------------------------------------
// This routine uses the Timer1.pwm function to set the pwm duty cycle.
//------------------------------------------------------------------------------------------------------
void set_pwm_duty(void) {

  if (pwm > PWM_MAX) {					   // check limits of PWM duty cyle and set to PWM_MAX
    pwm = PWM_MAX;		
  }
  else if (pwm < PWM_MIN) {				   // if pwm is less than PWM_MIN then set it to PWM_MIN
    pwm = PWM_MIN;
  }
  if (pwm < PWM_MAX) {
    Timer1.pwm(PWM_PIN,(PWM_FULL * (long)pwm / 100), 20);  // use Timer1 routine to set pwm duty cycle at 20uS period
    //Timer1.pwm(PWM_PIN,(PWM_FULL * (long)pwm / 100));
  }												
  else if (pwm == PWM_MAX) {				   // if pwm set to 100% it will be on full but we have 
    Timer1.pwm(PWM_PIN,(PWM_FULL - 1), 20);                // keep switching so set duty cycle at 99.9% 
    //Timer1.pwm(PWM_PIN,(PWM_FULL - 1));              
  }												
}	

//------------------------------------------------------------------------------------------------------
// This routine is the charger state machine. It has four states on, off, bulk and float.
// It's called once each time through the main loop to see what state the charger should be in.
// The battery charger can be in one of the following four states:
// 
//  On State - this is charger state for MIN_SOL_WATTS < solar watts < LOW_SOL_WATTS. In this state isthe solar 
//      watts input is too low for the bulk charging state but not low enough to go into the off state. 
//      In this state we just set the pwm = 99.9% to get the most of low amount of power available.

//  Bulk State - this is charger state for solar watts > MIN_SOL_WATTS. This is where we do the bulk of the battery
//      charging and where we run the Peak Power Tracking alogorithm. In this state we try and run the maximum amount
//      of current that the solar panels are generating into the battery.

//  Float State - As the battery charges it's voltage rises. When it gets to the MAX_BAT_VOLTS we are done with the 
//      bulk battery charging and enter the battery float state. In this state we try and keep the battery voltage
//      at MAX_BAT_VOLTS by adjusting the pwm value. If we get to pwm = 100% it means we can't keep the battery 
//      voltage at MAX_BAT_VOLTS which probably means the battery is being drawn down by some load so we need to back
//      into the bulk charging mode.

//  Off State - This is state that the charger enters when solar watts < MIN_SOL_WATTS. The charger goes into this
//      state when there is no more power being generated by the solar panels. The MOSFETs are turned
//      off in this state so that power from the battery doesn't leak back into the solar panel. 
//------------------------------------------------------------------------------------------------------
void run_charger(void) {
  
  static int off_count = OFF_NUM;

  switch (charger_state) {
    case on:                                        
      if (sol_watts < MIN_SOL_WATTS) {                      // if watts input from the solar panel is less than
        charger_state = off;                                // the minimum solar watts then 
        off_count = OFF_NUM;                                // go to the charger off state
        TURN_OFF_MOSFETS; 
      }
      else if (bat_volts > (BATT_FLOAT - 0.1)) {            // else if the battery voltage has gotten above the float
        charger_state = bat_float;                          // battery float voltage go to the charger battery float state
      }
      else if (sol_watts < LOW_SOL_WATTS) {                 // else if the solar input watts is less than low solar watts
        pwm = PWM_MAX;                                      // it means there is not much power being generated by the solar panel
        set_pwm_duty();			                    // so we just set the pwm = 100% so we can get as much of this power as possible
      }                                                     // and stay in the charger on state
      else {                                          
        pwm = ((bat_volts * 10) / (sol_volts / 10)) + 5;    // else if we are making more power than low solar watts figure out what the pwm
        charger_state = bulk;                               // value should be and change the charger to bulk state 
      }
      break;
    case bulk:
      if (sol_watts < MIN_SOL_WATTS) {                      // if watts input from the solar panel is less than
        charger_state = off;                                // the minimum solar watts then it is getting dark so
        off_count = OFF_NUM;                                // go to the charger off state
        TURN_OFF_MOSFETS; 
      }
      else if (bat_volts > BATT_FLOAT) {                 // else if the battery voltage has gotten above the float
        charger_state = bat_float;                          // battery float voltage go to the charger battery float state
      }
      else if (sol_watts < LOW_SOL_WATTS) {                 // else if the solar input watts is less than low solar watts
        charger_state = on;                                 // it means there is not much power being generated by the solar panel
        TURN_ON_MOSFETS;                                    // so go to charger on state
      }
      else {                                                // this is where we do the Peak Power Tracking ro Maximum Power Point algorithm
        if (old_sol_watts >= sol_watts) {                   // if previous watts are greater change the value of
          delta = -delta;			            // delta to make pwm increase or decrease to maximize watts
        }
        pwm += delta;                                       // add delta to change PWM duty cycle for PPT algorythm (compound addition)
        old_sol_watts = sol_watts;                          // load old_watts with current watts value for next time
        set_pwm_duty();				            // set pwm duty cycle to pwm value
      }
      break;
    case bat_float:

      if (sol_watts < MIN_SOL_WATTS) {                      // if watts input from the solar panel is less than
        charger_state = off;                                // the minimum solar watts then it is getting dark so
        off_count = OFF_NUM;                                // go to the charger off state
        TURN_OFF_MOSFETS; 
        set_pwm_duty();					
      }
      else if (bat_volts > BATT_FLOAT) {                    // If we've charged the battery abovethe float voltage                
        TURN_OFF_MOSFETS;                                   // turn off MOSFETs instead of modiflying duty cycle
        pwm = PWM_MAX;                                      // the charger is less efficient at 99% duty cycle
        set_pwm_duty();                                     // write the PWM
      }
      else if (bat_volts < BATT_FLOAT) {                    // else if the battery voltage is less than the float voltage - 0.1
        pwm = PWM_MAX;                                      
        set_pwm_duty();	                                    // start charging again
        TURN_ON_MOSFETS; 		
        if (bat_volts < (BATT_FLOAT - 0.1)) {               // if the voltage drops because of added load,
        charger_state = bulk;                               // switch back into bulk state to keep the voltage up
        }
      }
      break;
    case off:                                               // when we jump into the charger off state, off_count is set with OFF_NUM
      TURN_OFF_MOSFETS;
      if (off_count > 0) {                                  // this means that we run through the off state OFF_NUM of times with out doing
        off_count--;                                        // anything, this is to allow the battery voltage to settle down to see if the  
      }                                                     // battery has been disconnected
      else if ((bat_volts > BATT_FLOAT) && (sol_volts > bat_volts)) {
        charger_state = bat_float;                          // if battery voltage is still high and solar volts are high
        TURN_ON_MOSFETS;
      }    
      else if ((bat_volts > MIN_BAT_VOLTS) && (bat_volts < BATT_FLOAT) && (sol_volts > bat_volts)) {
        charger_state = bulk;
        TURN_ON_MOSFETS;
      }
      break;
    default:
      TURN_OFF_MOSFETS; 
      break;
  }
}

//----------------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////LOAD CONTROL/////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------  

void load_control(){
#if LOAD_ALGORITHM == 0
  // turn on loads at night when the solar panel is not producing power
  // as long as the battery voltage is above LVD
  load_on(sol_watts < MIN_SOL_WATTS && bat_volts > LVD);
#else
  // dump excess solar energy into the load circuit
  load_on(sol_watts > MIN_SOL_WATTS && bat_volts > BATT_FLOAT);
#endif
}

void load_on(boolean new_status) {
  if (load_status != new_status) {
    load_status = new_status;
    digitalWrite(LOAD_PIN, new_status ? HIGH : LOW);
  }
}

//------------------------------------------------------------------------------------------------------
// This routine prints all the data out to the serial port.
//------------------------------------------------------------------------------------------------------
void print_data(void) {
  
  Serial.print(seconds,DEC);
  Serial.print("      ");

  Serial.print("Charging = ");
  if (charger_state == on) Serial.print("on   ");
  else if (charger_state == off) Serial.print("off  ");
  else if (charger_state == bulk) Serial.print("bulk ");
  else if (charger_state == bat_float) Serial.print("float");
  Serial.print("      ");

  Serial.print("pwm = ");
  if(charger_state == off)
  Serial.print(0,DEC);
  else
  Serial.print(pwm,DEC);
  Serial.print("      ");

  Serial.print("Current (panel) = ");
  Serial.print(sol_amps);
  Serial.print("      ");

  Serial.print("Voltage (panel) = ");
  Serial.print(sol_volts);
  Serial.print("      ");

  Serial.print("Power (panel) = ");
  Serial.print(sol_volts);
  Serial.print("      ");

  Serial.print("Battery Voltage = ");
  Serial.print(bat_volts);
  Serial.print("      ");

  Serial.print("\n\r");
  //delay(1000);
}

//-------------------------------------------------------------------------------------------------
//---------------------------------Led Indication--------------------------------------------------
//-------------------------------------------------------------------------------------------------
// light an individual LED
// we remember which one was on before in last_lit and turn it off if different
void light_led(char pin)
{
  static char last_lit;
  if (last_lit == pin)
      return;
  if (last_lit != 0)
      digitalWrite(last_lit, LOW);
  digitalWrite(pin, HIGH);
  last_lit = pin;
}
// display the current state via LED as follows:
// YELLOW means overvoltage (over 14.1 volts)
// RED means undervoltage (under 11.9 volts)
// GREEN is between 11.9 and 14.1 volts
void led_output(void)
{
  static char last_lit;
  if(bat_volts > 14.1 )
      light_led(LED_YELLOW);
  else if(bat_volts > 11.9)
      light_led(LED_GREEN);
  else
      light_led(LED_RED);
}

//------------------------------------------------------------------------------------------------------
//-------------------------- LCD DISPLAY --------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void lcd_display()
{
  static bool current_backlight_state = -1;
  back_light_pin_State = digitalRead(BACK_LIGHT_PIN);
  if (current_backlight_state != back_light_pin_State) {
    current_backlight_state = back_light_pin_State;
    if (back_light_pin_State == HIGH)
      lcd.backlight();// finish with backlight on
    else
      lcd.noBacklight();
  }

  if (back_light_pin_State == HIGH)
  {
    time = millis();                        // If any of the buttons are pressed, save the time in millis to "time"
  }
 
 lcd.setCursor(0, 1);
 lcd.print(sol_volts);
 lcd.print("V ");
 lcd.setCursor(0, 2);
 lcd.print(sol_amps);
 lcd.print("A");  
 lcd.setCursor(0, 3);
 lcd.print(sol_watts);
 lcd.print("W "); 
 lcd.setCursor(8, 1);
 lcd.print(bat_volts);
 lcd.setCursor(8,2);

 if (charger_state == on) 
 lcd.print("on   ");
 else if (charger_state == off)
 lcd.print("off  ");
 else if (charger_state == bulk)
 lcd.print("bulk ");
 else if (charger_state == bat_float)
 {
 lcd.print("     ");
 lcd.setCursor(8,2);
 lcd.print("float");
 }
 
 //-----------------------------------------------------------
 //--------------------Battery State Of Charge ---------------
 //-----------------------------------------------------------
 int pct = 100.0*(bat_volts - 11.3)/(12.7 - 11.3);
 if (pct < 0)
     pct = 0;
 else if (pct > 100)
     pct = 100;

 lcd.setCursor(12,0);
 lcd.print((char)(pct*5/100));

 lcd.setCursor(8,3);
 pct = pct - (pct%10);
 lcd.print(pct);
 lcd.print("%  ");
 
//--------------------------------------------------------------------- 
//------------------Duty Cycle-----------------------------------------
//---------------------------------------------------------------------
 lcd.setCursor(15,0);
 lcd.print("PWM");
 lcd.setCursor(19,0);
 lcd.write(PWM_ICON);
 lcd.setCursor(15,1);
 lcd.print("   ");
 lcd.setCursor(15,1);
 if( charger_state == off)
 lcd.print(0);
 else
 lcd.print(pwm); 
 lcd.print("% ");
 //----------------------------------------------------------------------
 //------------------------Load Status-----------------------------------
 //----------------------------------------------------------------------
 lcd.setCursor(15,2);
 lcd.print("Load");
 lcd.setCursor(15,3);
 if (load_status)
 {
    lcd.print("On  ");
 }
 else
 {
   lcd.print("Off ");
 } 
 spinner();
 backLight_timer();                      // call the backlight timer function in every loop 
}

void backLight_timer(){
  if((millis() - time) <= 15000)         // if it's been less than the 15 secs, turn the backlight on
      lcd.backlight();                   // finish with backlight on  
  else 
      lcd.noBacklight();                 // if it's been more than 15 secs, turn the backlight off
}
void spinner(void) {
  static int cspinner;
  static char spinner_chars[] = { '*','*', '*', ' ', ' '};
  cspinner++;
  lcd.print(spinner_chars[cspinner%sizeof(spinner_chars)]);
}

//-------------------------------------------------------------------------
//----------------------------- ESP8266 WiFi ------------------------------
//--------------------------Plot System data on thingspeak.com-------------
//-------------------------------------------------------------------------
void wifi_datalog()
{
  // thingspeak needs 15 sec delay between updates
  static int lastlogged;
  if ( seconds - lastlogged < 16 )
      return;
  lastlogged = seconds;
 // convert to string
  char buf[16];
  String strTemp = dtostrf( sol_volts, 4, 1, buf);
  
  Serial.println(strTemp);
  
  // TCP connection
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += "184.106.153.149"; // api.thingspeak.com
  cmd += "\",80";
  ser.println(cmd);
   
  if(ser.find((char *)"Error")){
    Serial.println("AT+CIPSTART error");
    return;
  }
  
  // prepare GET string
  String getStr = "GET /update?api_key=";
  getStr += apiKey;
  getStr +="&field1=";
  getStr += String(strTemp);
  getStr += "\r\n\r\n";

  // send data length
  cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  ser.println(cmd);

  if(ser.find((char *)">")){
    ser.print(getStr);
  }
  else{
    ser.println("AT+CIPCLOSE");
    // alert user
    Serial.println("AT+CIPCLOSE");
  } 
}
