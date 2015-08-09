/*********************
Analog Slow Cooker Controller
Dan Ternes
www.maniacallabs.com

Controls a pair of relays that toggle on/off buttons on
a wireless outlet remote control. LCD display allows time
and "temperature" control. Temperature is controlled by
adjusting the "duty cycle" of a 60 second period.

Adafruit RGB LCD required:
https://www.adafruit.com/products/716

Based on example code for the Adafruit RGB Character LCD Shield and Library
and the Adafruit SousViduino project
https://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino/sous-vide

To use:
From starting screen:
RIGHT: Enter Set Time/Temp mode
SELECT: Reset timer to zero

From Set Time:
LEFT: exit set mode
UP/DOWN: Increase/decrease time in 30 min steps
RIGHT: Skip to Set Temp mode, time isn't changed
SELECT: Confirm set time, proceed to Set Temp mode

From Set Temp:
LEFT: Return to Set Time, ignoring changes
UP/DOWN: Cycle through temperature settings
RIGHT: Return to Off/Run state, ignoring changes
SELECT: Confirm set temperature and enter Run mode

From Running state:
LEFT: Enter Off state (timer stops, heater turns off)
RIGHT: Enter Set Time/Temp mode
SELECT: Reset timer to zero
When Timer Expires: Enter Warming mode

From Warming mode:
Heater remains on, temp set to 'Warm'
L/R/SEL: Turn off heater, back to Off state
**********************/

// ************************************************
// Adafruit RGB LCD includes/defines
// ************************************************
// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SET_TIME, SET_TEMP, RUN, WARM};
operatingState opState = OFF;

// ************************************************
// Defines and Globals
// ************************************************
#define PULSE_WIDTH 550 //How long in ms to "push" wireless remote button

#define TEMP_WARM 1 //value coresponding to the 'Warm' temp setting
#define TEMP_LOW 2 //value coresponding to the 'Low' temp setting
#define TEMP_MED 3 //value coresponding to the 'Med' temp setting
#define TEMP_HIGH 4 //value coresponding to the 'High' temp setting

#define TIME_INC 1800 //how much to incr/decr setTime (in seconds)

#define OFF_PIN 6 //GPIO Pin connected to Off relay control
#define ON_PIN 7 //GPIO Pin connected to On relay control

//Percent of the time during the Window (see below) that the heater is on
//You may need to tweak these for your cooker
#define WARM_DUTY_CYCLE 10 
#define LOW_DUTY_CYCLE 30 
#define MED_DUTY_CYCLE 60 
#define HIGH_DUTY_CYCLE 100 

// 60 second Time Proportional Output window
unsigned long WindowSize = 60000; // Seconds * 1000 (has to be in millis)
unsigned long windowStartTime; //used in DriveOutput()
unsigned long onTime = 0; //used in DriveOutput(). How much time (in millis) heater is on during Window

unsigned long timeLeft = 0;  //time remaining (in seconds)
unsigned long timeSet = 0;  //used when setting new time (in seconds)

int temp = 1; //used to determine onTime (see defines above and DriveOutput() below)
int tempSet = 1; //used when setting temperature

boolean heaterEnable = false; //heater won't turn on if this is false
boolean heaterOn = false; //also serves as an "Is it running?" flag

int OffPin = OFF_PIN; //connected to Off button relay
int OnPin = ON_PIN; //connected to On button relay

void setup() {
  // Debugging output
  Serial.begin(9600);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  lcd.print(F("  Analog  Slow  "));
  lcd.setCursor(0, 1);
  lcd.print(F(" Cooker Control"));
  delay(2500);

  pinMode(OffPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(OffPin, HIGH);  // make sure it is off to start
  pinMode(OnPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(OnPin, HIGH);  // make sure it is off to start

  digitalWrite(OffPin, LOW);  // make sure relay is off
  delay(PULSE_WIDTH);
  digitalWrite(OffPin, HIGH);

  //--Configure Timer for 1 sec interval--
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = 15624; //timer overflows this many times per 1 second
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
ISR(TIMER1_COMPA_vect)
{
  if ((opState == OFF) && heaterOn)
  {
    digitalWrite(OffPin, LOW);  // make sure relay is off
    delay(PULSE_WIDTH);
    digitalWrite(OffPin, HIGH);
    //delay(5000);
    heaterOn = false;
  }
  else
  {
    if (heaterEnable == true) { //if the device is running, always be counting down time
      if (timeLeft > 0) {
        timeLeft--;
      }
      DriveOutput(); //wireless remote activation and "temperature control" logic
    }
  }
}


// ************************************************
// Main loop just runs the state machine
// ************************************************
void loop() {
  // wait for button release before changing state
  while (lcd.readButtons() != 0) {}

  lcd.clear();

  switch (opState)
  {
    case OFF:
      Off();
      break;
    case SET_TIME:
      SetTime();
      break;
    case SET_TEMP:
      SetTemp();
      break;
    case RUN:
      Run();
      break;
    case WARM:
      Warm();
      break;
  }
}


// ************************************************
// State machine functions
// ************************************************
void Off()
{
  //turn the LCD off
  lcd.setBacklight(0);

  //disable heater (heaterEnabled)
  heaterEnable = false;
  
  delay(PULSE_WIDTH); //saftey margin to ensure the wireless remote relay is
                      //triggered correctly. Had timing issues during testing
  
  //turn off the heater (heaterOn)
  digitalWrite(OffPin, LOW);  // make sure relay is off
  delay(PULSE_WIDTH);
  digitalWrite(OffPin, HIGH);
  heaterOn = false;

  //print text
  lcd.setCursor(0, 0);
  lcd.print(F("TIME        TEMP"));
  lcd.setCursor(0, 1);
  lcd.print(timeLeft / 3600);
  if ((timeLeft / 60) < 10)
  {
    lcd.setCursor(1, 1);
    lcd.print(F(" "));
  }
  lcd.setCursor(3, 1);
  lcd.print((timeLeft / 60) % 60);
  if (((timeLeft / 60) % 60) < 10)
  {
    lcd.setCursor(4, 1);
    lcd.print(F(" "));
  }
  lcd.setCursor(6, 1);
  lcd.print(timeLeft % 60);
  if ((timeLeft % 60) < 10)
  {
    lcd.setCursor(7, 1);
    lcd.print(F(" "));
  }
  lcd.setCursor(12, 1);
  lcd.print(F("OFF"));

  uint8_t buttons = 0;
  while (true)
  {
    buttons = lcd.readButtons();

    if (buttons & BUTTON_SELECT)
    {
      //sel resets time (timeLeft = 0)
      timeLeft = 0;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      //right goes to set time
      opState = SET_TIME;
      return;
    }
  }
}

void SetTime()
{

  lcd.setCursor(0, 0);
  lcd.print(F("Set Time:"));
  lcd.setCursor(10, 1);
  lcd.print(F("hr/min"));
  lcd.setBacklight(BLUE);

  delay(200); //helps with debouncing

  //start with (timeSet) = (timeLeft)
  timeSet = timeLeft;

  //use up/down arrows to inc/dec time by half-hour
  uint8_t buttons = 0;
  while (true)
  {
    buttons = lcd.readButtons();

    if (buttons & BUTTON_UP)
    {
      timeSet += TIME_INC;
//      if (timeSet >= 34200)
//      {
//        timeSet = 34200;
//      }
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      timeSet -= TIME_INC;
      if (timeSet < 0)
      {
        timeSet = 0;
      }
      delay(200);
    }
    if (buttons & BUTTON_SELECT)
    {
      //sel accepts new time, goes to SET_TEMP
      timeLeft = timeSet;
      opState = SET_TEMP;
      return;
    }
    if (buttons & BUTTON_LEFT)
    {
      //left goes to off, canceling changes, but if heater is enabled (heaterEnabled), go to run
      if (heaterEnable)
      {
        opState = RUN;
      }
      else
      {
        opState = OFF;
      }
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      //right goes to set temp, doesn't change time
      opState = SET_TEMP;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(timeSet / 3600);
    lcd.setCursor(3, 1);
    lcd.print((timeSet / 60) % 60);
    if (((timeSet / 60) % 60) < 10)
    {
      lcd.setCursor(5, 1);
      lcd.print(F(" "));
    }
    lcd.setCursor(5, 1);
  }
}

void SetTemp()
{
  lcd.setCursor(0, 0);
  lcd.print(F("Set Temp:"));
  lcd.setBacklight(YELLOW);

  //start at current temp
  tempSet = temp;

  delay(200); //helps with debouncing

  //use up/down arrows to change temp
  uint8_t buttons = 0;
  while (true)
  {
    buttons = lcd.readButtons();

    //use up/down arrows to change temp
    if (buttons & BUTTON_UP)
    {
      tempSet += 1;
      if (tempSet > TEMP_HIGH)
      {
        tempSet = TEMP_HIGH;
      }
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      tempSet -= 1;
      if (tempSet < TEMP_WARM)
      {
        tempSet = TEMP_WARM;
      }
      delay(200);
    }
    if (buttons & BUTTON_SELECT)
    {
      //sel goes to run, starting the timer and heater control cycle
      temp = tempSet;
      windowStartTime = millis();
      opState = RUN;
      return;
    }
    if (buttons & BUTTON_LEFT)
    {
      //left goes to set time
      opState = SET_TIME;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      //right goes to off, saving changes, but if heater is enabled (heaterEnabled), go to run
      if (heaterEnable)
      {
        opState = RUN;
      }
      else
      {
        opState = OFF;
      }
      return;
    }
    lcd.setCursor(0, 1);
    printTemp(tempSet);
    lcd.print(" ");
  }
}

void Run()
{
  //display temp and remaining time
  lcd.setCursor(0, 0);
  lcd.print(F("TIME        TEMP"));
  lcd.setBacklight(GREEN);

  //sets the temp control duty cycle
  setTempDutyCycle(temp);

  //enable the heater (heaterEnable)
  heaterEnable = true;

  uint8_t buttons = 0;
  while (true)
  {

    //if no time left (timeLeft), go to WARM
    if (timeLeft <= 0)
    {
      temp = TEMP_WARM;
      setTempDutyCycle(temp);
      windowStartTime = millis();
      opState = WARM;
      return;
    }
    buttons = lcd.readButtons();

    if (buttons & BUTTON_LEFT)
    {
      //left goes to Off
      opState = OFF;
      delay(PULSE_WIDTH); //saftey margin to ensure the wireless remote relay is
                          //triggered correctly. Had timing issues during testing
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      //right goes to set time
      opState = SET_TIME;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(timeLeft / 3600);
    if ((timeLeft / 60) < 10)
    {
      lcd.setCursor(1, 1);
      lcd.print(F(" "));
    }
    lcd.setCursor(3, 1);
    lcd.print((timeLeft / 60) % 60);
    if (((timeLeft / 60) % 60) < 10)
    {
      lcd.setCursor(4, 1);
      lcd.print(F(" "));
    }
    lcd.setCursor(6, 1);
    lcd.print(timeLeft % 60);
    if ((timeLeft % 60) < 10)
    {
      lcd.setCursor(7, 1);
      lcd.print(F(" "));
    }
    lcd.setCursor(12, 1);
    printTemp(temp);
  }
}


void Warm()
{
  //LCD color to Yellow
  lcd.setBacklight(YELLOW);

  //print text
  lcd.setCursor(0, 0);
  lcd.print(F("TIME        TEMP"));
  lcd.setCursor(0, 1);
  lcd.print(F("DONE!"));
  lcd.setCursor(12, 1);
  printTemp(temp);

  uint8_t buttons = 0;
  while (true)
  {
    buttons = lcd.readButtons();
    
    //hit any button to turn off heater
    if (buttons & BUTTON_SELECT)
    {
      opState = OFF;
      return;
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = OFF;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = OFF;
      return;
    }
  }
}

// ************************************************
// Helper Functions
// ************************************************
void DriveOutput()
{

  //see if heater is enabled (heaterEnable), if not, do nothing
  //duty cycle logic (onTime) based on temp, set in Run
  //onTime determines for how much of WindowSize (e.g. 60sec) the
  //heater remains on. Lower temp settings means the heater
  //is on for shorter amounts of the WindowSize, higher temps
  //mean it is on for longer

//  Serial.print("onTime: ");
//  Serial.println(onTime, DEC);
//  Serial.print("windowStartTime: ");
//  Serial.println(windowStartTime, DEC);
//  Serial.print("windowSize: ");
//  Serial.println(WindowSize, DEC);

  if (heaterEnable) {
    long now = millis();
//    Serial.print("now: ");
//    Serial.println(now, DEC);
    if (now - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
//      Serial.println(onTime, DEC);
    }
    if ((onTime > PULSE_WIDTH) && (onTime > (now - windowStartTime)))
    {
      if (!heaterOn)
      {
        digitalWrite(OnPin, LOW);
        delay(PULSE_WIDTH * 30);
        digitalWrite(OnPin, HIGH);
        heaterOn = true;
      }
    }
    else
    {
      if (heaterOn)
      {
        digitalWrite(OffPin, LOW);
        delay(PULSE_WIDTH * 30);
        digitalWrite(OffPin, HIGH);
        heaterOn = false;
      }
    }
  }
}

void printTemp(int temp) {

  switch (temp)
  {
    case TEMP_WARM:
      lcd.print(F("WARM"));
      break;
    case TEMP_LOW:
      lcd.print(F("LOW"));
      break;
    case TEMP_MED:
      lcd.print(F("MED"));
      break;
    case TEMP_HIGH:
      lcd.print(F("HIGH"));
      break;
  }
  return;
}

void setTempDutyCycle(int temp) {

  switch (temp)
  {
    case TEMP_WARM:
      onTime = (WindowSize/100) * long(WARM_DUTY_CYCLE);
      break;
    case TEMP_LOW:
      onTime = (WindowSize/100) * long(LOW_DUTY_CYCLE);
      break;
    case TEMP_MED:
      onTime = (WindowSize/100) * long(MED_DUTY_CYCLE);
      break;
    case TEMP_HIGH:
      onTime = (WindowSize/100) * long(HIGH_DUTY_CYCLE);
      break;
  }
  return;
}
