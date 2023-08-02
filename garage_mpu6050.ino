#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Define the states
enum State{
  CLOSED,
  OPENED,
  OPENING,
  OPENING_TIMEOUT,
  ROTATED,
  ROTATED_TIMEOUT
};

State currentState = CLOSED;
unsigned long stateStartTime = 0;
float openingTotalRadians = 0;
const unsigned long OPENING_TIMEOUT_DURATION = 4000; 
const unsigned long ROTATED_TIMEOUT_DURATION = 4000; 
const unsigned long OPENED_TIMEOUT_DURATION = 14000;
const float Y_ACCELERATION_THRESHOLD = -0.4;

// Define the events
enum Event {
  RESET_ALL,
  MOTION_TIMEOUT,
  ROTATION_COMPLETE
};

Event currentEvent = RESET_ALL;

void resetState() {
  currentState = CLOSED;
  currentEvent = RESET_ALL;
  stateStartTime = millis();
  openingTotalRadians = 0.0;
}

void handleEvent(Event event) {
  currentEvent = event;
}

void transitionTo(State newState) {
  currentState = newState;
  stateStartTime = millis();
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 

  stateStartTime = millis();

  Serial.println("");
  delay(100);
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //debug
  Serial.println("Got new data!");

  // Check for rotation in the X axis
  bool rotating = false;
  float rotationSpeedX = 0.0;
  if (g.gyro.x > 0.2) {
    rotating = true;
    rotationSpeedX = g.gyro.x;
    openingTotalRadians += rotationSpeedX * 0.5;
  }

  // Check if acceleration in the Y axis is increasing
  float accY = a.acceleration.x;  //for now, it is .x - change it according to sensor orientation
  static float prevAccY = 0.0;
  bool accYDecreasing = (accY - prevAccY < Y_ACCELERATION_THRESHOLD);
  prevAccY = accY;

  // State machine
  switch (currentState) {
    case CLOSED:
      if (rotating) {
        transitionTo(OPENING);
      }
      break;

    case OPENING:
      if (openingTotalRadians > 2.0) {
        transitionTo(ROTATED);
      } else if (!rotating) {
        transitionTo(OPENING_TIMEOUT);
      }
      break;

    case OPENING_TIMEOUT:
      if (millis() - stateStartTime >= OPENING_TIMEOUT_DURATION) {
        resetState();
      } else if (rotating) {
        transitionTo(OPENING);
      }
      break;

    case ROTATED:
      if (!rotating && !accYDecreasing) {
        transitionTo(ROTATED_TIMEOUT);
      } else if (accYDecreasing) {
        transitionTo(OPENED);
      }
      break;

    case ROTATED_TIMEOUT:
      if (millis() - stateStartTime >= ROTATED_TIMEOUT_DURATION) {
        resetState();
      } else if (accYDecreasing) {
        transitionTo(OPENED);
      } else if (rotating) {
        transitionTo(ROTATED);
      }
      break;

    case OPENED:
      if (millis() - stateStartTime >= OPENED_TIMEOUT_DURATION) {
        transitionTo(CLOSED);
      }
      break;

    default:
      break;
  }

  Serial.print("State: ");
  switch (currentState) {
    case CLOSED:
      Serial.println("Closed");
      break;
    case OPENED:
      Serial.println("Opened");
      break;
    case OPENING:
      Serial.println("Opening");
      break;
    case OPENING_TIMEOUT:
      Serial.println("Opening Timeout");
      break;
    case ROTATED:
      Serial.println("Rotated");
      break;
    case ROTATED_TIMEOUT:
      Serial.println("Rotated Timeout");
      break;
  }

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  
  Serial.println("");
  delay(500);
}

