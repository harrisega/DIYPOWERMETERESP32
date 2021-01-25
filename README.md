# DIYPOWERMETERESP32
Library for DIY Power Meter ESP32 Board by Axial Labs

***

This library add functionality for reading measurement data from power meter module and GPIO function (LED Indicator, Buttons, and Latch Relay)

#### Common issue:
Make sure the device is connected to the AC power! Please be caution, AC is dangerous. You are responsible for your own action.

### Specifications

| Function      | Measuring range      | Resolution      | Accuracy | 
|---------------|----------------------|-----------------|----------|
| Voltage       | 80 ~ 250VAC          | 0.0001V         | 1%       |
| Current       | 10mA ~ 10A           | 0.0001A         | 1%       |
| Active power  | 0 ~ 2500W            | 0.0001W         | 1%       |
| Active energy | 0 ~ 999999.999kWh    | 0.001kWh        | 1%       |
| Frequency     | 45 ~ 65Hz            | 0.1Hz           | 1%       |
| Power factor  | 0.00~1.00            | 0.01            | 1%       |

Maximum Load 10A (Resistive Load)

### Example
```c++
#include <DIYPOWERMETERESP32.h>

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
      if (!isnan(voltage))
      {
        Serial.print("Voltage: ");
        Serial.print(voltage);
        Serial.println(" V");
      }
      else
      {
        Serial.println("Error reading Voltage");
      }

      float current = PowerMeter.current();
      if (!isnan(current))
      {
        Serial.print("Current: ");
        Serial.print(current);
        Serial.println(" A");
      }
      else
      {
        Serial.println("Error reading Current");
      }

      float power = PowerMeter.power();
      if (!isnan(power))
      {
        Serial.print("Power: ");
        Serial.print(power);
        Serial.println(" W");
      }
      else
      {
        Serial.println("Error reading Power");
      }

      float energy = PowerMeter.energy();
      if (!isnan(energy))
      {
        Serial.print("Energy: ");
        Serial.print(energy);
        Serial.println(" kWh");
      }
      else
      {
        Serial.println("Error reading Energy");
      }

      float frequency = PowerMeter.frequency();
      if (!isnan(frequency))
      {
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.println(" Hz");
      }
      else
      {
        Serial.println("Error reading Frequency");
      }

      float pf = PowerMeter.pf();
      if (!isnan(pf))
      {
        Serial.print("PF: ");
        Serial.println(pf);
      }
      else
      {
        Serial.println("Error reading PF");
      }

      float temperature = PowerMeter.temperature();
      if (!isnan(temperature))
      {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println("°C");
      }
      else
      {
        Serial.println("Error reading Temperature");
      }
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
```
# Installation instructions
You should be able to install the library from the Library Manager in the Arduino IDE. You can also download the ZIP of this repository and install it manually. A guide on how to do that is over here: [https://www.arduino.cc/en/guide/libraries](https://www.arduino.cc/en/guide/libraries) 

***
Thanks to [@olehs](https://github.com/olehs) and [@mandulaj](https://github.com/mandulaj) for inspiring this library.
