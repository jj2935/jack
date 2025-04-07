// PROJECT  :JumpHeightTracker
// PURPOSE  :To calculate the height of a persons vertical jump through an acceleromter
// COURSE   :ICS3U-E2
// AUTHOR   :Jack Smith
// DATE     :2025 04 05
// MCU      :328P (Nano)
// STATUS   :Working
// REFERENCE: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/#google_vignette

#include <Wire.h>               // Include Wire library for I2C communication
#include <Adafruit_MPU6050.h>   // Include Adafruit library for MPU6050 sensor
#include <Adafruit_Sensor.h>    // Include generic sensor library from Adafruit
#include <LiquidCrystal_I2C.h>  // Include LiquidCrystal I2C library for LCD display

Adafruit_MPU6050 mpu;                // Create an MPU6050 object named 'mpu'
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Create an LCD object at I2C address 0x27 with 20 columns and 4 rows

#define DATA_POINTS 200       // Define a constant for the number of data points in the circular buffer (200 points)
float jumpData[DATA_POINTS];  // Declare an array to store the last 200 Z acceleration values
int index = 0;                // Initialize an index variable for the circular buffer

uint8_t inJump = 0;               // Flag to indicate whether a jump is occurring (0 = no jump, 1 = jump happening)
unsigned long jumpStartTime = 0;  // Variable to store the time when a jump starts
unsigned long jumpEndTime = 0;    // Variable to store the time when a jump ends
float jumpHeight = 0;             // Variable to store the calculated jump height

void setup() {                   // Begin the setup function, runs once at start
  Serial.begin(115200);          // Initialize Serial communication at 115200 baud
  lcd.init();                    // Initialize the LCD
  lcd.backlight();               // Turn on the LCD backlight
  lcd.setCursor(0, 0);           // Set the LCD cursor to the first column of the first row
  lcd.print("MPU6050 Init...");  // Print initialization message on the LCD

  if (!mpu.begin()) {                      // Try to initialize the MPU6050 sensor, check if it fails
    lcd.setCursor(0, 1);                   // Set the LCD cursor to the first column of the second row
    lcd.print("ERROR!");                   // Print error message on the LCD
    Serial.println("MPU6050 not found!");  // Output error message to the Serial monitor
    while (1)
      ;  // Enter an infinite loop if the sensor is not found
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // Set the accelerometer's range to ±2G
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // Set the gyroscope's range to ±250 degrees per second
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set the sensor's filter bandwidth to 21 Hz

  lcd.clear();  // Clear the LCD screen
}

void display(float x, float y, float z, float height) {  // Define a function to display sensor values and jump height on the LCD
  lcd.setCursor(0, 0);                                   // Set the LCD cursor to the first column of the first row
  lcd.print("X: ");                                      // Print label for X acceleration
  lcd.print(x, 2);                                       // Print X acceleration value with 2 decimal places

  lcd.setCursor(0, 1);  // Set the LCD cursor to the first column of the second row
  lcd.print("Y: ");     // Print label for Y acceleration
  lcd.print(y, 2);      // Print Y acceleration value with 2 decimal places

  lcd.setCursor(8, 1);  // Set the LCD cursor to the 9th column of the second row
  lcd.print("Z: ");     // Print label for Z acceleration
  lcd.print(z, 2);      // Print Z acceleration value with 2 decimal places

  lcd.setCursor(0, 2);   // Set the LCD cursor to the first column of the third row
  lcd.print("Jump: ");   // Print label for jump height
  lcd.print(height, 2);  // Print jump height value with 2 decimal places
  lcd.print("m ");       // Print "m" to indicate meters
}

void loop() {                            // Begin the loop function, runs repeatedly
  static unsigned long prevTime = 0;     // Declare a static variable to keep track of previous time
  unsigned long currentTime = millis();  // Get the current time in milliseconds

  if (currentTime - prevTime >= 50) {  // Check if 50ms have passed since the last update
    prevTime = currentTime;            // Update prevTime with the current time

    sensors_event_t a, g, temp;   // Declare sensor event structures for acceleration, gyroscope, and temperature
    mpu.getEvent(&a, &g, &temp);  // Retrieve sensor data from the MPU6050

    float zAcc = a.acceleration.z;  // Store the Z-axis acceleration value

    // Store Z acceleration
    jumpData[index] = zAcc;             // Save the current Z acceleration into the buffer
    index = (index + 1) % DATA_POINTS;  // Increment the buffer index and wrap around if necessary

    // Detect Jump Start
    if (!inJump && zAcc > 12.0) {        // If not currently in a jump and Z acceleration exceeds 12.0 (indicative of takeoff)
      inJump = 1;                        // Set the jump flag to indicate a jump has started
      jumpStartTime = millis();          // Record the time when the jump started
      Serial.println("Jump detected!");  // Print a message to the Serial monitor indicating a jump
    }

    // Detect Jump End
    if (inJump && zAcc < 9.5) {                                // If in a jump and Z acceleration drops below 9.5 (indicative of landing)
      inJump = 0;                                              // Reset the jump flag as the jump has ended
      jumpEndTime = millis();                                  // Record the time when the jump ended
      float airTime = (jumpEndTime - jumpStartTime) / 1000.0;  // Calculate the duration of the jump in seconds

      // Calculate Jump Height
      jumpHeight = (9.81 * airTime * airTime) / 8.0;  // Compute the jump height based on air time
      Serial.print("Jump Height: ");                  // Print label for jump height on the Serial monitor
      Serial.print(jumpHeight, 2);                    // Print the calculated jump height with 2 decimal places
      Serial.println(" m");                           // Print "m" to indicate meters and move to next line
    }

    Serial.print("X: ");                // Print label for X acceleration on Serial monitor
    Serial.print(a.acceleration.x, 2);  // Print X acceleration with 2 decimal places
    Serial.print(" Y: ");               // Print label for Y acceleration
    Serial.print(a.acceleration.y, 2);  // Print Y acceleration with 2 decimal places
    Serial.print(" Z: ");               // Print label for Z acceleration
    Serial.print(zAcc, 2);              // Print Z acceleration with 2 decimal places
    Serial.print(" Jump Height: ");     // Print label for jump height
    Serial.print(jumpHeight, 2);        // Print jump height with 2 decimal places
    Serial.println("m");                // Print "m" to indicate meters and move to next line

    display(a.acceleration.x, a.acceleration.y, zAcc, jumpHeight);  // Call the display function to show values on the LCD
  }
}
