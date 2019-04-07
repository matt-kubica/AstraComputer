#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <LiquidCrystal.h>
#include <string.h>
#include <avr/wdt.h>



//constans
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define INJECTOR_PIN 13
#define IMPULSATOR_PIN 2
#define UP_BUTTON_PIN 1
#define DOWN_BUTTON_PIN 0

#define IMPULSATOR_INTERRUPT 1
#define UP_BUTTON_PIN_INTERRUPT 3 
#define DOWN_BUTTON_PIN_INTERRUPT 2 

#define MAX_FUEL_CONSUMPTION 36 //liter per hour
#define IMPULSES_PER_METER 16.85

#define UPPER_LINE 0
#define LOWER_LINE 1
#define OPTIONS_AMMOUNT 5

#define SCREEN_WIDTH 16
#define SCREEN_HEIGHT 2

#define AVG_CONSUMPTION_SUM_EEPROM_ADDRESS 0
#define AVG_CONSUMPTION_DIVIDER_EEPROM_ADDRESS 4
#define AVG_SPEED_SUM_EEPROM_ADDRESS 8
#define AVG_SPEED_DIVIDER_EEPROM_ADDRESS 12
#define IMPULSE_COUNTER_EEPROM_ADDRESS 16
//----------------------------------------------------------------------------------------------------------------------------------------------------



//global variables and declarations
//----------------------------------------------------------------------------------------------------------------------------------------------------
//lcd and timer objects declaration
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); //TODO: definitions


//fuel consumption global variables
volatile uint16_t startingEdge = 0; 
volatile uint16_t endingEdge = 0; 
volatile uint16_t currentOverflowCounter = 0;
volatile uint16_t startOverflowCounter = 0;
volatile uint32_t timer3Clocks = 0;
volatile uint64_t overallTimer3Clocks = 0;
uint16_t injectorWorkingTimePerSecond = 0;
double consumptionPerHour = 0, consumptionPer100KM = 0;
double averageConsumptionPer100KM = 0;
uint32_t averageConsumptionSum = 0;
uint32_t averageConsumptionDivider = 0;


//speed global variables
volatile uint16_t impulsesPerSecond = 0;
uint32_t overallImpulseCounter = 0;
uint8_t speedOfVehicle = 0;
uint8_t averageSpeed = 0;
uint32_t averageSpeedSum = 0;
uint32_t averageSpeedDivider = 0;


//distance global variables
uint32_t overallDistanceMeters = 0;
double overallDistanceKilometers = 0;


//buttons and display global variables
bool upButtonPressed = false;
bool downButtonPressed = false;
uint8_t upLineCounter = 0;
uint8_t downLineCounter = 2;


//EEPROM
uint16_t drivingTime = 0;


//general global variables
volatile bool reload = false;
volatile bool screenReload = false;
uint16_t seconds = 0;


//methods declarations
void calculateSpeed();
void calculateConsumption();
void calculateDistance();
void displayInfo();
void incrementImpulsesAmmountISR();
//----------------------------------------------------------------------------------------------------------------------------------------------------



//setup
//----------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {

	//LCD init
  lcd.begin(SCREEN_WIDTH, SCREEN_HEIGHT);
  lcd.setCursor(0,0);
  lcd.print("  ONBOARD COMP  ");
  lcd.setCursor(0,1);
  lcd.print("  ASTRA FX14XE  ");
  delay(5000);

  noInterrupts();

	//PIN's configuration  
  pinMode(INJECTOR_PIN, INPUT);
  pinMode(IMPULSATOR_PIN, INPUT);
  pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);

	//interrupts configuration
	attachInterrupt(IMPULSATOR_INTERRUPT, incrementImpulsesAmmountISR, FALLING);
  attachInterrupt(UP_BUTTON_PIN_INTERRUPT, upButtonISR, FALLING);
  attachInterrupt(DOWN_BUTTON_PIN_INTERRUPT, downButtonISR, FALLING);

  //T1 registers configuration
  //responsible for reloading whole programm every 1000[ms]
  //-----------------------------------------------------------------------------------------------
  TCCR1A = 0;

  TCCR1B |= (1 << WGM12); //CTC mode
  TCCR1B &= ~(1 << WGM13);

  TCCR1B &= ~(1 << CS10); // prescaler = 256
  TCCR1B &= ~(1 << CS11);
  TCCR1B |= (1 << CS12);

  TCNT1 = 0; // initialize counter 

  TIMSK1 |= (1 << OCIE1A); //enable output capture interrupt

  OCR1A = 62499;
  //-----------------------------------------------------------------------------------------------


  //T3 registers configuration
  //responsible for measuring injector impulse length
  //-----------------------------------------------------------------------------------------------
  TCCR3A = 0;
  
  TCCR3B |= (1 << CS30); // prescaler = 0
  TCCR3B &= ~(1 << CS31);
  TCCR3B &= ~(1 << CS32);
  
  TCCR3B &= ~(1 << ICES3); //input capture on falling edge
  
  TCNT3 = 0; // initialize counter  

  TIMSK3 |= (1 << ICIE3)|(1 << TOIE3); //enable input capture interrupt, enable timer ovf interrupt
  //-----------------------------------------------------------------------------------------------
  
  wdt_enable(WDTO_2S); //wdt init
  loadEEPROM();        //EEPROM init
  interrupts();
}
//----------------------------------------------------------------------------------------------------------------------------------------------------



//loop
//----------------------------------------------------------------------------------------------------------------------------------------------------
void loop() { //TODO: improve this
  if(screenReload and reload) {
    //reload every 1000 [ms]
    if(reload) {
      calculateSpeed();
      calculateConsumption();
      calculateAverageSpeed();
      calculateAverageConsumption();

      //calculate distance every 10 second
      if((seconds % 10) == 0) 
        calculateDistance();
    
      reload = false;
    
      saveEEPROM();
      isCarDriving();
      wdt_reset();
    }
    displayInfo();
    screenReload = false;
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------



//ISR methods
//----------------------------------------------------------------------------------------------------------------------------------------------------
void upButtonISR() {
  upButtonPressed = true;
  screenReload = true;
}

void downButtonISR() {
  downButtonPressed = true;
  screenReload = true;
}

void incrementImpulsesAmmountISR() {
  impulsesPerSecond++;
}

//T1 1000ms interrupt
ISR (TIMER1_COMPA_vect) {
  noInterrupts();
  reload = true;
  screenReload = true;
  seconds++;
  interrupts();
}

//T3 overflow interrupt
ISR (TIMER3_OVF_vect) {
  //ovf per 65536 timer clocks (4.096 [ms])
  currentOverflowCounter++;
}

//interrupt on pin 13
ISR (TIMER3_CAPT_vect) {
  noInterrupts();
  
  //checking state of ICES3bit (ICES3 == 1 -> input capture on rising edge, ICES3 == 0 -> input capture on falling edge)
  if(((TCCR3B & (1 << ICES3)) >> ICES3) == 0) {           //if falling edge occurs
    startingEdge = ICR3;                                  //saving state(value) of ICR3 register
    startOverflowCounter = currentOverflowCounter;        //saving ovf counter state
  } else {                                                //if rising enge occurs
    endingEdge = ICR3;                                    //saving state(value) of ICR3 register

    //calculating how many timer3 clocks had place during injector impulse    
    timer3Clocks = (uint16_t)endingEdge + ((uint16_t)currentOverflowCounter * 65536) - ((uint16_t)startingEdge + (startOverflowCounter * 65536));
    
    overallTimer3Clocks += timer3Clocks;                  //adding to overall result
    currentOverflowCounter = 0;                           //clearing variable for next impulse
  }
  TCCR3B ^= (1 << ICES3);                                 //toggle ICES3 bit, preparing for next PWM's edge
  
  interrupts();
}
//----------------------------------------------------------------------------------------------------------------------------------------------------



//calculating methods
//----------------------------------------------------------------------------------------------------------------------------------------------------
void calculateSpeed() {
  speedOfVehicle = (impulsesPerSecond * 36) / (IMPULSES_PER_METER * 10.00); //km/h (3600/1000) -> (36/10)
  overallImpulseCounter += impulsesPerSecond;
  impulsesPerSecond = 0;
}

void calculateAverageSpeed() {
  if(speedOfVehicle > 0) {
    averageSpeedSum += speedOfVehicle;
    averageSpeedDivider++;
    averageSpeed = averageSpeedSum / averageSpeedDivider; 
  }
}

void calculateConsumption() {
  injectorWorkingTimePerSecond = overallTimer3Clocks / 16000; //result in milliseconds
  overallTimer3Clocks = 0;                                    //reseting timer3's clocks counter

  consumptionPerHour = MAX_FUEL_CONSUMPTION * injectorWorkingTimePerSecond / 1000.0;
  consumptionPer100KM = MAX_FUEL_CONSUMPTION * injectorWorkingTimePerSecond / (speedOfVehicle * 10.0);
}

void calculateAverageConsumption() {
  if(speedOfVehicle > 5) {
    averageConsumptionSum += ((uint32_t)round(consumptionPer100KM));
    averageConsumptionDivider++;
    averageConsumptionPer100KM = averageConsumptionSum / (averageConsumptionDivider * 1.0);
  }
}

void calculateDistance() {
  overallDistanceMeters = overallImpulseCounter / IMPULSES_PER_METER;
  overallDistanceKilometers = overallDistanceMeters / 1000.0;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------



//buttons methods
//----------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t lineCounterOverflower(uint8_t counter) {
  if(counter > OPTIONS_AMMOUNT - 1) {
    counter = 0;
  }
  return counter;
}

void checkForButtons() {
  if(upButtonPressed) {
    upLineCounter = lineCounterOverflower(++upLineCounter);
    upButtonPressed = false;
  }
    
  if(downButtonPressed) {
    downLineCounter = lineCounterOverflower(++downLineCounter);
    downButtonPressed = false;
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------




//display methods
//----------------------------------------------------------------------------------------------------------------------------------------------------
void displayTempSpeed(uint8_t line) {
  String toDisplay = String((int)speedOfVehicle); 
  
  lcd.setCursor(0, line);
  lcd.print("                ");
  lcd.setCursor(0, line);
  lcd.print("SPEED: ");
  lcd.setCursor(SCREEN_WIDTH - toDisplay.length(), line);
  lcd.print(toDisplay);
}

void displayAvgSpeed(uint8_t line) {
  String toDisplay = String((int)averageSpeed); 
  
  lcd.setCursor(0, line);
  lcd.print("                ");
  lcd.setCursor(0, line);
  lcd.print("AVG SPEED: ");
  lcd.setCursor(SCREEN_WIDTH - toDisplay.length(), line);
  lcd.print(toDisplay);
}

void displayTempConsumption(uint8_t line) {
  String toDisplay;
  
  if(speedOfVehicle > 5)
    toDisplay = String((double)consumptionPer100KM, 1); 
  else
    toDisplay = String((double)consumptionPerHour, 1);
  
  lcd.setCursor(0, line);
  lcd.print("                ");
  lcd.setCursor(0, line);
  lcd.print("CONS: ");
  lcd.setCursor(SCREEN_WIDTH - toDisplay.length(), line);
  lcd.print(toDisplay);
}

void displayAvgConsumption(uint8_t line) {
  String toDisplay = String((double)averageConsumptionPer100KM, 1);
  
  lcd.setCursor(0, line);
  lcd.print("                ");
  lcd.setCursor(0, line);
  lcd.print("AVG CONS: ");
  lcd.setCursor(SCREEN_WIDTH - toDisplay.length(), line);
  lcd.print(toDisplay);
}

void displayDistance(uint8_t line) {
  String toDisplay = String((double)overallDistanceKilometers, 1); 
  
  lcd.setCursor(0, line);
  lcd.print("                ");
  lcd.setCursor(0, line);
  lcd.print("ODO: ");
  if(overallDistanceKilometers < 10.0)
    lcd.setCursor(SCREEN_WIDTH - toDisplay.length(), line);
  else
    lcd.setCursor(SCREEN_WIDTH - toDisplay.length() + 2, line);
  lcd.print(toDisplay);
}

void displayInfo() { 
  checkForButtons();
  
  switch(upLineCounter) {
    case 0: displayTempConsumption(UPPER_LINE); 
        break;
    case 1: displayAvgConsumption(UPPER_LINE); 
        break;
    case 2: displayTempSpeed(UPPER_LINE); 
        break;
    case 3: displayAvgSpeed(UPPER_LINE); 
        break;
    case 4: displayDistance(UPPER_LINE);
        break;
  }

  switch(downLineCounter) {
    case 0: displayTempConsumption(LOWER_LINE); 
        break;
    case 1: displayAvgConsumption(LOWER_LINE); 
        break;
    case 2: displayTempSpeed(LOWER_LINE); 
        break;
    case 3: displayAvgSpeed(LOWER_LINE); 
        break;
    case 4: displayDistance(LOWER_LINE);
        break;
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------



//EEPROM manipulation methods
//----------------------------------------------------------------------------------------------------------------------------------------------------
void isCarDriving() {
  if(speedOfVehicle > 0)
    drivingTime++;
   else drivingTime = 0;
}


void saveEEPROM() {
  if(speedOfVehicle == 0 and drivingTime >= 30 and EEPROM.isReady()) {
    Serial.println("save!");
    EEPROM.writeLong(AVG_CONSUMPTION_SUM_EEPROM_ADDRESS, averageConsumptionSum);
    EEPROM.writeLong(AVG_CONSUMPTION_DIVIDER_EEPROM_ADDRESS, averageConsumptionDivider);
    EEPROM.writeLong(AVG_SPEED_SUM_EEPROM_ADDRESS, averageSpeedSum);
    EEPROM.writeLong(AVG_SPEED_DIVIDER_EEPROM_ADDRESS, averageSpeedDivider);
    EEPROM.writeLong(IMPULSE_COUNTER_EEPROM_ADDRESS, overallImpulseCounter);
  }
}

void loadEEPROM() {
  if(EEPROM.isReady()) {
    averageConsumptionSum = EEPROM.readLong(AVG_CONSUMPTION_SUM_EEPROM_ADDRESS);
    averageConsumptionDivider = EEPROM.readLong(AVG_CONSUMPTION_DIVIDER_EEPROM_ADDRESS);
    averageSpeedSum = EEPROM.readLong(AVG_SPEED_SUM_EEPROM_ADDRESS);
    averageSpeedDivider = EEPROM.readLong(AVG_SPEED_DIVIDER_EEPROM_ADDRESS);
    overallImpulseCounter = EEPROM.readLong(IMPULSE_COUNTER_EEPROM_ADDRESS);
    
    Serial.print(averageConsumptionSum);
    Serial.print("\t");
    Serial.print(averageConsumptionDivider);
    Serial.print("\t");
    Serial.print(averageSpeedSum);
    Serial.print("\t");
    Serial.print(averageSpeedDivider);
    Serial.print("\t");
    Serial.println(overallImpulseCounter);
  }    
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
