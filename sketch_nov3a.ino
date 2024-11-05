#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHTPIN PA1
#define DHTTYPE DHT11
#define LED_PIN PA0      // External LED connected to PA0
#define BUZZER_PIN PA9   // Buzzer connected to PA9
#define SWITCH_PIN PA7   // Switch connected to PA7

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);
bool buzzerState = false;         // Track if the buzzer has been triggered
volatile bool bulbState = false;  // Track the LED bulb state (toggle state)

void setup() {
  Serial.begin(9600);  // Initialize serial monitor
  lcd.init();
  lcd.backlight();

  // Initialize the LED, Buzzer, and Switch pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // Switch with internal pull-up resistor
  digitalWrite(LED_PIN, LOW);        // Ensure LED is off initially
  digitalWrite(BUZZER_PIN, LOW);     // Ensure Buzzer is off initially

  // Display "Laboratory Alarm System" on the LCD
  lcd.setCursor(0, 0);
  lcd.print("Laboratory Alarm");
  lcd.setCursor(0, 1);
  lcd.print("System");
  delay(2000);  // Display for 2 seconds

  // Display "STM32 with DHT11" on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STM32 with DHT11");
  delay(2000);  // Display for 2 seconds

  dht.begin();
  delay(1000);  // Extra delay for sensor startup

  // Attach an interrupt to the switch pin
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), toggleBulb, FALLING);
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check if any reads failed and print error
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DHT Error!");
    digitalWrite(LED_PIN, LOW);   // Turn off the LED
    digitalWrite(BUZZER_PIN, LOW); // Turn off the Buzzer
    buzzerState = false;           // Reset buzzer state
    delay(2000);                   // Wait before retrying
    return;                        // Skip the rest of the loop
  }

  // Range check for realistic humidity and temperature values
  if (h < 0 || h > 100 || t < -40 || t > 80) { // Assuming DHT11 operating range
    Serial.println("Sensor values out of range!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Out of range!");
    digitalWrite(LED_PIN, LOW);   // Turn off the LED
    digitalWrite(BUZZER_PIN, LOW); // Turn off the Buzzer
    buzzerState = false;           // Reset buzzer state
    delay(2000);
    return;
  }

  // Display on Serial Monitor
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" C");

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Humid: ");
  lcd.print(h);
  lcd.print(" %");

  // Check humidity and control the LED
  if (h > 85) {
    digitalWrite(LED_PIN, HIGH); // Turn on the LED if humidity > 85%
  } else {
    digitalWrite(LED_PIN, LOW);  // Turn off the LED if humidity <= 85%
  }

  // Check temperature and control the Buzzer
  if (t > 32) {
    if (!buzzerState) {
      digitalWrite(BUZZER_PIN, HIGH); // Turn on the Buzzer if temperature > 31°C
      buzzerState = true;             // Mark buzzer as triggered
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);    // Turn off the Buzzer if temperature <= 31°C
    buzzerState = false;              // Reset buzzer state for next activation
  }

  delay(1000); // Update every 1 second
}

// Interrupt Service Routine to toggle the bulb state
void toggleBulb() {
  bulbState = !bulbState;           // Toggle the bulb state
  digitalWrite(LED_PIN, bulbState); // Set LED_PIN based on new bulb state
}
