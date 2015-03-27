

// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// SDA....>A4
// SCL....>A5
#include <LiquidCrystal_I2C.h>
#include <Wire.h>  
byte solar[8] = //icon for termometer
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

byte battery[8]=
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};

byte pwm [8]=
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

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
int backlight_Pin = 5;
int backlight_State = 0;
void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);  // Used to type in characters
  pinMode(backlight_Pin, INPUT);
  lcd.begin(20,4);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.noBacklight(); 
  lcd.createChar(1,solar);
  lcd.createChar(2, battery);
  lcd.createChar(3, pwm);
  
//-------- Write characters on the display ------------------
// NOTE: Cursor Position: (CHAR, LINE) start at 0  
 
  lcd.clear();

}/*--(end setup )---*/


void loop()  
{
  symbol();
 backlight_State = digitalRead(backlight_Pin);
  if ( backlight_State == HIGH)
  {
    lcd.backlight();// finish with backlight on 
// Wait for 10 seconds and then turn off the display and backlight.
    delay(10000); // adjust back light on time
    lcd.noBacklight();
  }
}
void symbol() 

{
 lcd.setCursor(0, 0);
 lcd.print("SOL");
 lcd.setCursor(4, 0);
 lcd.write(1);
 lcd.setCursor(0, 1);
 lcd.print("16.45V");
 lcd.setCursor(0, 2);
 lcd.print("1.03A"); 
 lcd.setCursor(0, 3);
 lcd.print("16.94W"); 
 lcd.setCursor(8, 0);
 lcd.print("BAT");
 lcd.setCursor(12, 0);
 lcd.write(2);
 lcd.setCursor(8, 1);
 lcd.print("12.35V");
 lcd.setCursor(8,2);
 lcd.print("bulk");
 lcd.setCursor(8,3);
 lcd.print( "70%");
 
 lcd.setCursor(15,0);
 lcd.print("PWM");
 lcd.setCursor(19,0);
 lcd.write(3);
 lcd.setCursor(15,1);
 lcd.print("95%"); 
 
 lcd.setCursor(15,2);
 lcd.print("Load");
 lcd.setCursor(15,3);
 lcd.print("Off");
}
