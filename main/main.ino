#include "basicHardware.h"
#include "wateringHardware.h"

// MCP_23017* _mcp;
// Switch* button;
// Pin* pin;
Waterman* waterman;

Plant plants[] = {
  { { 200, 660 }, Thirstiness::LOW_THIRST },
  { { -200, 700 }, Thirstiness::HIGH_THIRST },
  { { 0, 850 }, Thirstiness::MEDIUM_THIRST }
};

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);

  // pin = new PinExtender(mcp, GPA3);
  // // pin = new PinNative(D6);
  // button = new Switch("button", pin);

  // pinMode(LED_BUILTIN, OUTPUT);


  waterman = new Waterman();
  waterman->resetPosition();
  delay(1000);
  waterman->waterPlants(plants, 3);


}

// the loop function runs over and over again forever
void loop() {
  // if (button->isPressed()) {
  //   Serial.println("ON");
  // } else {
  //   Serial.println("off");
  // }



  // if (button->isPressed() == true) {
  //   digitalWrite(LED_BUILTIN, LOW);
  // } else {
  //   digitalWrite(LED_BUILTIN, HIGH);
  // }
  // waterman->resetPosition();  
  delay(5000);
}
