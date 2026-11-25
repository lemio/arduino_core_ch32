/*
The built-in LED on pin PB12 on the CH32X035 blinks shortly (100 ms) when turning on.
If the requested voltage is available it then remains on; if the requested voltage
is not available it stays off.
*/

#include <USBPD.h>

void setup() {
  pinMode(PB12, OUTPUT);
  digitalWrite(PB12, HIGH);
  delay(100);
  digitalWrite(PB12, LOW);
  // Ask the USB-C source for 9 V and at least 0.2 A.
  USBPD.requestVoltage(9, 0.2);
}

void loop() {
  if (USBPD.isReady()) {
    // The source accepted the request. This is the contract voltage.
    float voltage = USBPD.getVoltage();
    digitalWrite(PB12, HIGH);

    (void)voltage;
  } else {
    delay(100);
    // The requested power is not available yet. Check getStatus() for details.
  }
}
