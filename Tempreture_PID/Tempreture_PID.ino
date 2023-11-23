#include <max6675.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display


int thermoDO = 2;
int thermoCS = 3;
int thermoCLK = 4;
int relayPin = 5;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// PID parameters
double Kp = 5.0;   // Proportional gain
double Ki = 0.2;   // Integral gain
double Kd = 0.1;   // Derivative gain

double setpoint = 50.0;
double output, error, lastError, integral;

void setup() {
  pinMode(relayPin, OUTPUT);
  lcd.init();
   lcd.backlight();
  Serial.begin(9600);
}

void loop() {
  // Read temperature in Celsius
  double temperatureC = thermocouple.readCelsius();

  // Check if the thermocouple reading is valid
  if (isnan(temperatureC)) {
    Serial.println("Error reading temperature!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" °C");

    // Calculate the error
    error = setpoint - temperatureC;

    // Calculate the integral
    integral += error;

    // Calculate the derivative
    double derivative = error - lastError;

    // Calculate the PID output
    output = Kp * error + Ki * integral + Kd * derivative;

    // Update the relay based on the PID output
    if (output > 0) {
      digitalWrite(relayPin, LOW); // Turn on the external device (heater)
    } else {
      digitalWrite(relayPin, HIGH); // Turn off the external device
    }

    // Save the current error for the next iteration
    lastError = error;
  }
  lcd.setCursor(0,0);
  lcd.print("set: ");
  lcd.print(output);
  lcd.setCursor(1,1);
  lcd.print("Temp: ");
  lcd.print(temperatureC);
  delay(100); // Adjust the delay as needed
}
