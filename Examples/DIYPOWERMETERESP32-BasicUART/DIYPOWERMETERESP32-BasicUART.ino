#include "DIYPowerMeterESP32.h"

DIYPOWERMETERESP32 PowerMeter(POWERMETER_SERIAL_INTERFACE);

unsigned long lastMillis;
enumButtonState lastButtonState = NOT_PRESSED;

void setup()
{
  Serial.begin(115200);
  PowerMeter.setRelay(ON);
  PowerMeter.setLED(LED_RLY, ON);

  // Check Power Meter Module Configuration and try to restore the parameter if not matched
  PowerMeter.checkConfig();

  // Reset Energy (kWh) Value when BUTTON1 Pressed During Boot Up
  if (PowerMeter.getButton(BUTTON1) == PRESSED)
    PowerMeter.resetEnergy();
}

void loop()
{
  // Read Measurement every 1000ms / 1s
  if (millis() - lastMillis >= 1000)
  {
    lastMillis = millis();
    PowerMeter.setLED(LED_ACT, ON);
    Serial.print("\nRead Measurement...");
    if (PowerMeter.readMeasurement())
    {
      Serial.println("SUCCESS");
      float voltage = PowerMeter.voltage();
      Serial.print("Voltage: ");
      Serial.print(voltage);
      Serial.println(" V");

      float current = PowerMeter.current();
      Serial.print("Current: ");
      Serial.print(current);
      Serial.println(" A");

      float power = PowerMeter.power();
      Serial.print("Power: ");
      Serial.print(power);
      Serial.println(" W");

      float energy = PowerMeter.energy();
      Serial.print("Energy: ");
      Serial.print(energy);
      Serial.println(" kWh");

      float frequency = PowerMeter.frequency();
      Serial.print("Frequency: ");
      Serial.print(frequency);
      Serial.println(" Hz");

      float pf = PowerMeter.pf();
      Serial.print("PF: ");
      Serial.println(pf);

      float temperature = PowerMeter.temperature();
      Serial.print("Core Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");
    }
    else
    {
      Serial.println("FAILED");
    }
    Serial.println("-------------------------------");
  }

  PowerMeter.setLED(LED_ACT, OFF);

  /*
      Button Check Routine
      Button1 / SW1 : Turn OFF Relay
      Button2 / SW2 : Turn ON Relay
  */
  if (lastButtonState == NOT_PRESSED)
  {
    if (PowerMeter.getButton(BUTTON1) == PRESSED)
    {
      lastButtonState = PRESSED;
      PowerMeter.setRelay(OFF);
      PowerMeter.setLED(LED_RLY, OFF);
      PowerMeter.setBuzzer(ON);
      delay(100);
      PowerMeter.setBuzzer(OFF);
    }
    else if (PowerMeter.getButton(BUTTON2) == PRESSED)
    {
      lastButtonState = PRESSED;
      PowerMeter.setRelay(ON);
      PowerMeter.setLED(LED_RLY, ON);
      PowerMeter.setBuzzer(ON);
      delay(100);
      PowerMeter.setBuzzer(OFF);
    }
  }
  else if (PowerMeter.getButton(BUTTON1) == NOT_PRESSED && PowerMeter.getButton(BUTTON2) == NOT_PRESSED)
  {
    lastButtonState = NOT_PRESSED;
  }
}
