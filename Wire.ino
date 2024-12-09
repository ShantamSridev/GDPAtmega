#include <Wire.h>

#define SLAVE_ADDRESS 0x09
#define READLENGTH 4
#define RECEIVELENGTH 2
#define TYPE 3 //WIRE
#define POLARISATION 0



uint8_t memory[128]; // Simulate internal registers/memory
uint8_t currentInternalAddress = 0;



void setup() {
    // Initialize simulated memory with some values
    for (uint8_t i = 0; i < 128; i++) {
        memory[i] = i;
    }
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(9600);
}

void loop() {
    // No continuous work needed; all actions are event-driven
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
