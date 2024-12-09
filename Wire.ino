#include <Wire.h>
//#include <led.h>
#define SLAVE_ADDRESS 0x09
#define READLENGTH 4
#define RECEIVELENGTH 2
#define TYPE 3 //WIRE
#define POLARISATION 0


#define RED1 PD3
#define BLUE1 PD2
#define BLUE2 PD4
#define BLUE3 PD5
#define BLUE4 PD6
#define BLUE5 PD7
#define BLUE6 PB0
#define BLUE7 PB6



uint8_t memory[128]; // Simulate internal registers/memory
uint8_t currentInternalAddress = 0;

const unsigned long interval = 500; // Time between LED transitions (in ms)

const int ledPins[] = {1, 2, 3};  // Define the pins for the LEDs
const int numLeds = 3; // Number of LEDs Actually 6

unsigned long previousMillis = 0; // Stores the last time an LED state changed
int currentLed = 0;               // Tracks the currently active LED


void setup() {
    // Initialize simulated memory with some values
    for (uint8_t i = 0; i < 128; i++) {
        memory[i] = i;
    }
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(9600);

    led_setup();
}

void loop() {
    // No continuous work needed; all actions are event-driven
    // red_on();
    // delay(2000);
    // red_off();
    // delay(2000);
    led_cycle(-1);
  
}



// Handle receiving data from the master
void receiveEvent(int bytes) {
    if (bytes == 1) {
        //THIS IS A REQUEST
        currentInternalAddress = Wire.read(); // First byte is internal address

        Serial.print("Internal address set to: ");
        Serial.println(currentInternalAddress);
    }
    if (bytes == 2) {
      //THIS IS A WRITE
      Serial.println("WRITE");
      currentInternalAddress = Wire.read();       // First byte: internal address
      Serial.println(currentInternalAddress);
      uint8_t value = Wire.read();         // Second byte: value to write
      Serial.println(value);
      if (currentInternalAddress < sizeof(memory)) {      // Ensure address is within bounds
          memory[currentInternalAddress] = value;         // Write value to memory
      } else {
           Serial.println("Error: Address out of bounds");
      }
    }
    
}

// Handle data request from the master
void requestEvent() {
    Wire.write(&memory[currentInternalAddress], 4); // Send 4 bytes starting from the current internal address
}





void led_setup() {
  // // Initialize the LED pins as outputs and turn them off
  // for (int i = 0; i < numLeds; i++) {
  //   pinMode(ledPins[i], OUTPUT);
  //   digitalWrite(ledPins[i], HIGH); // Turn off the LED (active low)
  // }

  DDRD |= (1 << RED1);
  DDRD |= (1 << BLUE1);
  DDRD |= (1 << BLUE2);
  DDRD |= (1 << BLUE3);
  DDRD |= (1 << BLUE4);
  DDRD |= (1 << BLUE5);
  DDRB |= (1 << BLUE6);
  DDRB |= (1 << BLUE7);
  

  // INITIAL LED OFF: Active Low
  PORTD |= (1 << RED1);
  PORTD |= (1 << BLUE1);
  PORTD |= (1 << BLUE2);
  PORTD |= (1 << BLUE3);
  PORTD |= (1 << BLUE4);
  PORTD |= (1 << BLUE5);
  PORTB |= (1 << BLUE6);
  PORTB |= (1 << BLUE7);

  // // Turn on all LEDs
  // for (int i = 0; i < numLeds; i++) {
  //   digitalWrite(ledPins[i], LOW); // Turn on the LED (active low)
  // }

  
}

void all_leds_on(){
  PORTD &= ~(1 << RED1);
  PORTD &= ~(1 << BLUE1);
  PORTD &= ~(1 << BLUE2);
  PORTD &= ~(1 << BLUE3);
  PORTD &= ~(1 << BLUE4);
  PORTD &= ~(1 << BLUE5);
  PORTB &= ~(1 << BLUE6);
  PORTB &= ~(1 << BLUE7);
}

void all_leds_off(){
  PORTD |= (1 << RED1);
  PORTD |= (1 << BLUE1);
  PORTD |= (1 << BLUE2);
  PORTD |= (1 << BLUE3);
  PORTD |= (1 << BLUE4);
  PORTD |= (1 << BLUE5);
  PORTB |= (1 << BLUE6);
  PORTB |= (1 << BLUE7);
}

void red_on(){
  PORTD &= ~(1 << RED1);
}

void red_off(){
  PORTD |= (1 << RED1);
}

void led_pair_on(int pair){
  if (pair == 0){
    PORTD &= ~(1 << BLUE2);
    PORTD &= ~(1 << BLUE5);
  }
  if (pair == 1){
    PORTD &= ~(1 << BLUE3);
    PORTB &= ~(1 << BLUE6);
  }
  if (pair == 2){
    PORTD &= ~(1 << BLUE4);
    PORTB &= ~(1 << BLUE7);
  }
}

void led_pair_off(int pair){
  if (pair == 0){
    PORTD |= (1 << BLUE2);
    PORTD |= (1 << BLUE5);
  }
  if (pair == 1){
    PORTD |= (1 << BLUE3);
    PORTB |= (1 << BLUE6);
  }
  if (pair == 2){
    PORTD |= (1 << BLUE4);
    PORTB |= (1 << BLUE7);
  }
}

void led_cycle(int direction){
  unsigned long currentMillis = millis();

  // Check if it's time to switch to the next LED
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Turn off the current LED
    led_pair_off(currentLed);
    Serial.println(currentLed);
    // Move to the next LED, flipping direction if needed
    currentLed += direction;

     // Handle wrapping around
    if (currentLed >= numLeds) {
      currentLed = 0; // Wrap to the first LED
    } else if (currentLed < 0) {
      currentLed = numLeds - 1; // Wrap to the last LED
    }

    // Turn on the next LED
    led_pair_on(currentLed);
  }
}
