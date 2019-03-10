#include <LiquidCrystal.h>

//lcd and timer objects declaration
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

uint8_t inputCapturePin = 13;

volatile uint16_t startingEdge = 0; 
volatile uint16_t endingEdge = 0; 
volatile uint16_t currentOverflowCounter = 0;
volatile uint16_t startOverflowCounter = 0;
volatile uint32_t timer3Clocks = 0;
volatile uint64_t overallTimer3Clocks = 0;

volatile bool reload = false;
volatile uint16_t seconds = 0;


void setup() {
  
  Serial.begin(9600);

  lcd.begin(16,2);
  lcd.setCursor(1,0);
  lcd.print("IMPULSE LENGTH");
  lcd.setCursor(3,1);
  lcd.print("MEASURMENT");
  delay(3000);
  
  pinMode(13, INPUT);
  noInterrupts();

  //T1 registers configuration
  TCCR1A = 0;

  TCCR1B |= (1 << WGM12); //CTC mode
  TCCR1B &= ~(1 << WGM13);

  TCCR1B &= ~(1 << CS10); // prescaler = 256
  TCCR1B &= ~(1 << CS11);
  TCCR1B |= (1 << CS12);

  TCNT1 = 0; // initialize counter 

  TIMSK1 |= (1 << OCIE1A); //enable output capture interrupt

  OCR1A = 62499;


  //T3 registers configuration
  TCCR3A = 0;
  
  TCCR3B |= (1 << CS30); // prescaler = 0
  TCCR3B &= ~(1 << CS31);
  TCCR3B &= ~(1 << CS32);
  
  TCCR3B &= ~(1 << ICES3); //input capture on falling edge
  

  TCNT3 = 0; // initialize counter  

  TIMSK3 |= (1 << ICIE3)|(1 << TOIE3); //enable input capture interrupt, enable timer ovf interrupt
  interrupts();
}


//loop method
void loop() {
  
  if(reload) {
    int result = overallTimer3Clocks / 16000; //result in milliseconds
    lcd.clear();
    lcd.print(result);
    lcd.setCursor(0,1);
    lcd.print(seconds);
    Serial.println(result);
    seconds++;
    
    //Serial.print(seconds);
    reload = false;
    overallTimer3Clocks = 0;
  }
  
}

//T1 1000ms interrupt
ISR (TIMER1_COMPA_vect) {
  reload = true;
}

ISR (TIMER3_OVF_vect) {
  //ovf per 65536 timer clocks (4.096 [ms])
  currentOverflowCounter++;
}

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
