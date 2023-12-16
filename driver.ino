//Name: Maya Price
//Date: 12/09/23
//Assignment: Final Project

// Define macros for register and pin positions
#define DDR(x) (*(&x - 1)) // Data Direction Register
#define PORT(x) (*(&x))    // Port Register
#define PIN(x) (*(&x - 2))  // Pin Register

// Define pin numbers
#define waterLevelPin A0      // Water level sensor pin
#define tempHumidityPin 2     // DHT11 sensor pin
#define fanMotorPin 3         // Fan motor pin
#define stepperMotorPin1 4    // Stepper motor pin 1
#define stepperMotorPin2 5    // Stepper motor pin 2
#define stepperMotorPin3 6    // Stepper motor pin 3
#define stepperMotorPin4 7    // Stepper motor pin 4
#define startButtonPin 8      // Start button pin
#define stopButtonPin 9       // Stop button pin
#define enableButtonPin 10    // Enable/disable button pin
#define yellowLEDPin 11       // Yellow LED pin
#define greenLEDPin 12        // Green LED pin
#define redLEDPin 13          // Red LED pin
#define blueLEDPin A1         // Blue LED pin

// State constants
#define DISABLED 0
#define IDLE 1
#define ERROR_STATE 2
#define RUNNING 3

// Stepper motor configuration
#define STEPS_PER_REV 2048
#define HIGH_TEMPERATURE 30 // Example high temperature threshold

#include <dht11.h>
#include <Wire.h>
#include <RTClib.h>

dht11 DHT;
RTC_DS3231 rtc;

// Global variables
volatile uint8_t currentState = DISABLED;
volatile uint8_t fanRunning = 0; // 0: OFF, 1: ON
volatile uint8_t stepperPosition = 0;

void setup() 
{
  // Set pin modes
  DDR(PORT(yellowLEDPin)) |= (1 << (yellowLEDPin % 8)); // Set YELLOW LED pin as output
  DDR(PORT(greenLEDPin)) |= (1 << (greenLEDPin % 8));   // Set GREEN LED pin as output
  DDR(PORT(redLEDPin)) |= (1 << (redLEDPin % 8));       // Set RED LED pin as output
  DDR(PORT(blueLEDPin)) |= (1 << (blueLEDPin % 8));     // Set BLUE LED pin as output

  // Set initial LED states
  PORT(PORT(yellowLEDPin)) |= (1 << (yellowLEDPin % 8)); // YELLOW LED ON
  PORT(PORT(greenLEDPin)) &= ~(1 << (greenLEDPin % 8));  // GREEN LED OFF
  PORT(PORT(redLEDPin)) &= ~(1 << (redLEDPin % 8));      // RED LED OFF
  PORT(PORT(blueLEDPin)) &= ~(1 << (blueLEDPin % 8));    // BLUE LED OFF

  // Set pin modes for buttons
  DDR(PORT(startButtonPin)) &= ~(1 << (startButtonPin % 8)); // Set START button as input
  PORT(PORT(startButtonPin)) |= (1 << (startButtonPin % 8)); // Enable internal pull-up resistor

  DDR(PORT(stopButtonPin)) &= ~(1 << (stopButtonPin % 8)); // Set STOP button as input
  PORT(PORT(stopButtonPin)) |= (1 << (stopButtonPin % 8)); // Enable internal pull-up resistor

  DDR(PORT(enableButtonPin)) &= ~(1 << (enableButtonPin % 8)); // Set ENABLE button as input
  PORT(PORT(enableButtonPin)) |= (1 << (enableButtonPin % 8)); // Enable internal pull-up resistor

  // Initialize stepper motor
  DDR(PORT(stepperMotorPin1)) |= (1 << (stepperMotorPin1 % 8)); // Set stepper motor pins as output
  DDR(PORT(stepperMotorPin2)) |= (1 << (stepperMotorPin2 % 8));
  DDR(PORT(stepperMotorPin3)) |= (1 << (stepperMotorPin3 % 8));
  DDR(PORT(stepperMotorPin4)) |= (1 << (stepperMotorPin4 % 8));

  // Initialize interrupt for START button
  EICRA |= (1 << ISC01); // Falling edge triggers interrupt
  EIMSK |= (1 << INT1);  // Enable external interrupt INT1

  // Set initial state and log event
  currentState = DISABLED;
  logEvent("System initialized");
}

void loop() 
{
  switch (currentState) 
  {
    case DISABLED:
      disabledState();
      break;

    case IDLE:
      idleState();
      break;

    case ERROR_STATE:
      errorState();
      break;

    case RUNNING:
      runningState();
      break;
  }
}

void disabledState() 
{
  PORT(PORT(yellowLEDPin)) |= (1 << (yellowLEDPin % 8)); // YELLOW LED ON

  // Check for start button press
  if (!(PIN(PORT(startButtonPin)) & (1 << (startButtonPin % 8)))) 
  {
    currentState = IDLE;
    PORT(PORT(yellowLEDPin)) &= ~(1 << (yellowLEDPin % 8)); // YELLOW LED OFF
    logEvent("System enabled");
  }
}

void idleState() 
{
  PORT(PORT(greenLEDPin)) |= (1 << (greenLEDPin % 8)); // GREEN LED ON

  // Check for stop button press
  if (!(PIN(PORT(stopButtonPin)) & (1 << (stopButtonPin % 8)))) 
  {
    currentState = DISABLED;
    PORT(PORT(greenLEDPin)) &= ~(1 << (greenLEDPin % 8)); // GREEN LED OFF
    stopFan();
    logEvent("System disabled");
    return;
  }

  // Check water level
  if (isWaterLow()) 
  {
    currentState = ERROR_STATE;
    PORT(PORT(greenLEDPin)) &= ~(1 << (greenLEDPin % 8)); // GREEN LED OFF
    logEvent("Water level low");
    return;
  }

  // Monitor and display temperature/humidity
  monitorTempHumidity();
}

void errorState() 
{
  PORT(PORT(redLEDPin)) |= (1 << (redLEDPin % 8)); // RED LED ON

   Serial.print("Error State!");

  // Check for reset button press
  if (!(PIN(PORT(startButtonPin)) & (1 << (startButtonPin % 8)))) 
  {
    currentState = IDLE;
    PORT(PORT(redLEDPin)) &= ~(1 << (redLEDPin % 8)); // RED LED OFF
    logEvent("Error reset");
  }
}

void runningState() 
{
  PORT(PORT(blueLEDPin)) |= (1 << (blueLEDPin % 8)); // BLUE LED ON

  // Check water level
  if (isWaterLow()) 
  {
    currentState = ERROR_STATE;
    PORT(PORT(blueLEDPin)) &= ~(1 << (blueLEDPin % 8)); // BLUE LED OFF
    stopFan();
    logEvent("Water level low during operation");
    return;
  }

  // Turn on fan if temperature is too high
  if (getCurrentTemperature() > HIGH_TEMPERATURE && !fanRunning) 
  {
    startFan();
    logEvent("Fan started");
  }

  // Turn off fan if temperature is within range
  if (getCurrentTemperature() <= HIGH_TEMPERATURE && fanRunning) 
  {
    stopFan();
    logEvent("Fan stopped");
  }

  // Monitor and display temperature/humidity
  monitorTempHumidity();
}

void startFan() 
{
  PORT(PORT(fanMotorPin)) |= (1 << (fanMotorPin % 8));
  fanRunning = 1;
}

void stopFan() 
{
  PORT(PORT(fanMotorPin)) &= ~(1 << (fanMotorPin % 8));
  fanRunning = 0;
}

bool isWaterLow() 
{
  // Read analog value from water level sensor
  int waterLevelReading = analogRead(waterLevelPin);

  // Define the threshold for low water level (adjust as needed)
  int lowWaterThreshold = 200;

  // Check if water level is below the threshold
  if (waterLevelReading < lowWaterThreshold) 
  {
    return true;  // Water is low
  } 
  else 
  {
    return false; // Water level is okay
  }
}

float getCurrentTemperature() 
{
  // Read temperature from DHT11 sensor
  int chk = DHT.read(tempHumidityPin);

  // Check if the reading was successful
  if (chk == DHTLIB_OK) 
  {
    // Temperature is in Celsius
    float temperature = DHT.temperature;
    return temperature;
  } 
  else 
  {
    // Error reading temperature, return a default value
    return -1.0;
  }
}

void monitorTempHumidity() 
{
  // Read temperature and humidity
  float temperature = readTemperature();
  float humidity = readHumidity();

  // Display on LCD
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print("C  Humidity: ");
  Serial.println(humidity);
  delay(60000); // Update every minute
}

float readTemperature() 
{
  // Read temperature from DHT11 sensor
  int chk = DHT.read(tempHumidityPin);

  // Check if the reading was successful
  if (chk == DHTLIB_OK) 
  {
    // Temperature is in Celsius
    float temperature = DHT.temperature;
    return temperature;
  } 
  else 
  {
    // Error reading temperature, return a default value
    return -1.0;
  }
}

float readHumidity() 
{
  // Read humidity from DHT11 sensor
  int chk = DHT.read(tempHumidityPin);

  // Check if the reading was successful
  if (chk == DHTLIB_OK) 
  {
    // Humidity is a percentage
    float humidity = DHT.humidity;
    return humidity;
  } 
  else 
  {
    // Error reading humidity, return a default value
    return -1.0;
  }
}

void logEvent(const char* event) 
{
  // Get current time from RTC
  DateTime now = rtc.now();

  // Print event over Serial with timestamp
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" - ");
  Serial.println(event);
}

char* getCurrentTime() 
{
  // Get current time from RTC
  DateTime now = rtc.now();

  // Convert time to a string format
  static char currentTime[20];
  sprintf(currentTime, "%04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  return currentTime;
}
