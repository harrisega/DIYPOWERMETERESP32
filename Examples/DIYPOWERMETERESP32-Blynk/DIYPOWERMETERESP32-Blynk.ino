#include "DIYPowerMeterESP32.h"
#include "BlynkSimpleEsp32.h"

DIYPOWERMETERESP32 PowerMeter(POWERMETER_SERIAL_INTERFACE);

unsigned long lastMillis;
enumButtonState lastButtonState = NOT_PRESSED;

#define BLYNK_VIRTUAL_PIN_VOLTAGE V0
#define BLYNK_VIRTUAL_PIN_CURRENT V1
#define BLYNK_VIRTUAL_PIN_POWER V2
#define BLYNK_VIRTUAL_PIN_ENERGY V3
#define BLYNK_VIRTUAL_PIN_FREQUENCY V4
#define BLYNK_VIRTUAL_PIN_PF V5
#define BLYNK_VIRTUAL_PIN_CORETEMPERATURE V6
#define BLYNK_VIRTUAL_PIN_RELAY V7

char auth[] = "AUTH ID";
char ssid[] = "SSID";
char pass[] = "PASSWORD";

// Sync Blynk when connected
BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

// Relay Control from Apps
BLYNK_WRITE(BLYNK_VIRTUAL_PIN_RELAY)
{
  int relayState = param.asInt();
  if (relayState == 1)
  {
	// Relay ON
    PowerMeter.setRelay(ON);
    PowerMeter.setLED(LED_RLY, ON);
  }
  else if (relayState == 2)
  {
	// Relay OFF
    PowerMeter.setRelay(OFF);
    PowerMeter.setLED(LED_RLY, OFF);
  }
}

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

  // Start Blynk
  Blynk.begin(auth, ssid, pass, IPAddress(192, 168, 1, 200), 8080);
  // Blynk.begin(auth, ssid, pass);

  Serial.println("Blynk Started... !");
}

void loop()
{
  // Read Measurement every 1000ms / 1s
  if (millis() - lastMillis >= 1000)
  {
    lastMillis = millis();

    // Check Connection to Blynk server , Show Status via NET LED
    if (Blynk.connected())
    {
      PowerMeter.setLED(LED_NET, ON); // Device connected to server
    }
    else
    {
      PowerMeter.setLED(LED_NET, OFF);
    }

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

      float coreTemperature = PowerMeter.coreTemperature();

      Serial.print("Core Temperature: ");
      Serial.print(coreTemperature);
      Serial.println("°C");

	  // Send values to Blynk
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_VOLTAGE, voltage);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_CURRENT, current);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_POWER, power);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_ENERGY, energy);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_FREQUENCY, frequency);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_PF, pf);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_CORETEMPERATURE, coreTemperature);
    }
    else
    {
      Serial.println("FAILED");
    }
    Serial.println("-------------------------------");
    PowerMeter.setLED(LED_ACT, OFF);
  }

  Blynk.run();

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
