#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>

// --- SPI Pins from the document ---
#define W5500_SCK  4
#define W5500_MISO 5
#define W5500_MOSI 6
#define W5500_CS   7
// --- Interrupt Pin requested ---
#define W5500_INT_PIN 20

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

volatile bool w5500Event = false;
void IRAM_ATTR w5500IRQ() { 
  w5500Event = true; 
}

void setup() {
  Serial.begin(115200);
  delay(3000); 
  Serial.println("Starting W5500 Test...");

  // Set up the interrupt pin
  pinMode(W5500_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(W5500_INT_PIN), w5500IRQ, FALLING);

  // Initialize custom SPI pins for ESP32-C3
  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);
  Ethernet.init(W5500_CS);

  // --- NEW DIAGNOSTIC CHECK ---
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("ERROR: W5500 not found! Check your SPI wiring (MISO, MOSI, SCK, CS).");
    while (true) { delay(1); } // Stop here
  } else {
    Serial.println("SUCCESS: W5500 chip detected!");
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("ERROR: Ethernet cable is disconnected!");
  }

  // Set a shorter timeout for DHCP (15 seconds instead of 60)
  Serial.println("Starting DHCP...");
  if (Ethernet.begin(mac, 15000)) {
    Serial.print("Successfully connected! IP Address: ");
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("Failed to get an IP address from the router.");
  }
}

void loop() {
  // If the interrupt pin triggers, acknowledge it
  if (w5500Event) {
    w5500Event = false;
    Serial.println("Ethernet Interrupt triggered!");
  }
  
  // Let the Ethernet library maintain the DHCP lease
  Ethernet.maintain();
}