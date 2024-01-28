

// Pin assignments
const int measurePin = A0;
const int digitalOutPin1 = 2;
const int digitalOutPin2 = 5;
const int digitalInPin = 10;

// Reflow process parameters
const double RampToSoakRate = 1.0; // °C/s
const int PreHeatTime = 150; // seconds between 150 and 190 degree
const double RampToPeakRate = 1.0;
const int ReflowTime = 70; // plus cooling time
const double SupplyVoltage = 4.96;
const double ReferenceResistance = 0.997; // kOhm
const double HeatingHysteresis = 0.5; //Hysteresis for teh bang-bang controller

int toggleStatusLED = 0;

// Timing and temperature variables
volatile double Time = 0;
volatile double CurrentTime = 0;
volatile double TimeAtStateStart = 0;
volatile double TimeAbove217 = 0;
volatile double Temp = 30;
volatile int State = 0; // Ramp to soak

const double Temp2Resistance[] = { //From datasheet
    27280, 26135, 25045, 24005, 23014, 22070, 21169, 20309, 19489, 18707,
    17960, 17246, 16564, 15912, 15290, 14695, 14127, 13583, 13063, 12566,
    12091, 11635, 11199, 10782, 10383, 10000, 9633.4, 9282.0, 8945.3, 8622.5,
    8313.0, 8016.1, 7731.3, 7458.1, 7195.9, 6944.2, 6702.6, 6470.6, 6247.7, 6033.7,
    5828.0, 5630.4, 5440.4, 5257.7, 5082.1, 4913.2, 4750.8, 4594.5, 4444.1, 4299.4,
    4160.0, 4025.9, 3896.7, 3772.3, 3652.5, 3537.0, 3426.2, 3319.4, 3216.3, 3117.0,
    3021.2, 2928.7, 2839.5, 2753.5, 2670.4, 2590.3, 2512.9, 2438.1, 2366.0, 2296.3,
    2229.0, 2164.0, 2101.2, 2040.4, 1981.8, 1925.1, 1870.2, 1817.2, 1766.0, 1716.4,
    1668.4, 1622.0, 1577.2, 1533.7, 1491.7, 1451.0, 1411.6, 1373.5, 1336.5, 1300.8,
    1266.1, 1232.6, 1200.1, 1168.6, 1138.0, 1108.5, 1079.8, 1052.0, 1025.0, 998.90,
    973.50, 948.92, 925.07, 901.93, 879.47, 857.66, 836.50, 815.96, 796.01, 776.64,
    757.82, 739.55, 721.79, 704.55, 687.79, 671.50, 655.67, 640.29, 625.33, 610.79,
    596.65, 582.90, 569.53, 556.52, 543.87, 531.56, 519.58, 507.93, 496.59, 485.55,
    474.80, 464.34, 454.15, 444.23, 434.57, 425.17, 416.00, 407.08, 398.38, 389.91,
    381.65, 373.61, 365.77, 358.12, 350.67, 343.40, 336.32, 329.41, 322.68, 316.11,
    309.70, 303.44, 297.33, 291.37, 285.56, 279.88, 274.35, 268.94, 263.66, 258.51,
    253.48, 248.57, 243.77, 239.08, 234.51, 230.04, 225.67, 221.41, 217.24, 213.17,
    209.19, 205.30, 201.49, 197.78, 194.15, 190.59, 187.12, 183.73, 180.41, 177.16,
    173.98, 170.88, 167.84, 164.87, 161.96, 159.11, 156.33, 153.60, 150.94, 148.33,
    145.77, 143.27, 140.82, 138.42, 136.08, 133.78, 131.53, 129.32, 127.17, 125.05,
    122.98, 120.95, 118.96, 117.02, 115.11, 113.24, 111.41, 109.61, 107.85, 106.13,
    104.44, 102.78, 101.15, 99.56, 98.00, 96.46, 94.960, 93.490, 92.040, 90.630,
    89.240, 87.870, 86.530, 85.220, 83.930, 82.670, 81.430, 80.210, 79.020, 77.850,
    76.700, 75.570, 74.460, 73.370, 72.300, 71.250, 70.220, 69.210, 68.220, 67.240,
    66.280, 65.340, 64.420, 63.510, 62.620, 61.740, 60.880, 60.040, 59.200, 58.390,
    57.590, 56.800, 56.020, 55.260, 54.510, 53.780, 53.050, 52.340, 51.640, 50.950,
    50.280, 49.610, 48.960, 48.320, 47.690, 47.060, 46.450, 45.850, 45.260, 44.680,
    44.110, 43.540, 42.990, 42.440, 41.910, 41.380, 40.860, 40.350, 39.850, 39.350,
    38.870, 38.390, 37.910, 37.450, 36.990, 36.540, 36.100, 35.660, 35.230, 34.810,
    34.390, 33.980, 33.580, 33.180, 32.790, 32.400, 32.020, 31.650, 31.280, 30.920,
    30.560
};


// Function Prototypes
void setupTimer();
void controlHeatingElement(double currentTemperature);
void updateTemperature(double &temp);
double calculateCurrentTemperature(double measuredVoltage);
void printDebugInfo(double measuredVoltage, double thermistorResistance, double currentTemperature);


//2. Setup Function
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(digitalOutPin1, OUTPUT);
  pinMode(digitalOutPin2, OUTPUT);
  pinMode(digitalInPin, INPUT);

  setupTimer();
}

void setupTimer() {
  cli(); // stop interrupts
  // Use Timer/Counter TCB3 using a TOP of 10k and prescaler of /2 to run at 1kHz
  TCB3.CCMP = 20000;
  TCB3.INTCTRL = (1 << TCB_CAPT_bp); // Enable interrupts
  TCB3.INTCTRL |= (1 << TCB_CAPT_bp); // Enable interrupts
  // |= TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // Enable timer with prescaler of /2 to start with 10MHz frequency.
  sei(); // allow interrupts
}

//3. Interrupt Service Routine (ISR)
ISR(TIMER2_COMPA_vect) {
  static int itrCounter = 0;
  itrCounter++;
  Time += 0.0005;
  CurrentTime = Time - TimeAtStateStart;

  double measuredVoltage = analogRead(measurePin) * (5.0 / 1023.0);
  double currentTemperature = calculateCurrentTemperature(measuredVoltage);

  double tempCopy = Temp;  // Make a copy of the volatile variable
  updateTemperature(tempCopy);  // Pass the copy to the function
  Temp = tempCopy;  // Write the result back if the function modifies the value

  controlHeatingElement(currentTemperature);

  if (itrCounter % 20 == 0) {
    double thermistorResistance = (SupplyVoltage - measuredVoltage) / measuredVoltage * ReferenceResistance;
    printDebugInfo(currentTemperature);
    itrCounter = 0;
  }
}


//4. Helper Functions
/*The updateTemperature function manages the state transitions of the reflow process and updates the target temperature accordingly. 
To improve the readability and maintainability of the code, it's a good idea to separate the logic for each state into its own function. 
Below is the filled-in updateTemperature function along with auxiliary functions for each state:
*/
// Function prototypes for state management
void handleRampToSoak(double &temp, double &timeAtStateStart, double &time);
void handlePreheatAndSoak(double &temp, double &timeAtStateStart, double &time);
void handleRampToPeak(double &temp, double &timeAbove217, double &time);
void handleCooling(double &temp);

void updateTemperature(double &temp) {
  double timeCopy = Time; // Make a copy of the volatile variable
  double timeAtStateStartCopy = TimeAtStateStart; // Make a copy of the volatile variable
  double timeAbove217Copy = TimeAbove217; // Make a copy of the volatile variable

  switch (State) {
    case 0: // Ramp to Soak
      handleRampToSoak(temp, timeAtStateStartCopy, timeCopy);
      break;
    case 1: // Preheat and Soak
      handlePreheatAndSoak(temp, timeAtStateStartCopy, timeCopy);
      break;
    case 2: // Ramp to Peak
      handleRampToPeak(temp, timeAbove217Copy, timeCopy);
      break;
    case 3: // Cooling
      handleCooling(temp);
      break;
    default:
      // Handle unexpected state
      Serial.println("Error: Unknown state!");
      break;
  }

  // If these functions modify these variables, write the changes back to the original variables.
  Time = timeCopy;
  TimeAtStateStart = timeAtStateStartCopy;
  TimeAbove217 = timeAbove217Copy;
}


void handleRampToSoak(double &temp, double &timeAtStateStart, double &time) {
  if (temp < 150) {
    temp += RampToSoakRate / 2000.0;
  } else {
    State = 1; // Transition to Preheat and Soak
    timeAtStateStart = time;
    Serial.print("Transition to Preheat and Soak, time is: ");
    Serial.println(time, 10);
  }
}

void handlePreheatAndSoak(double &temp, double &timeAtStateStart, double &time) {
  double currentTime = time - timeAtStateStart;
  if (temp < 190) {
    temp += (190 - temp + 1) / (PreHeatTime - currentTime) * 1 / 2000.0;
  } else {
    State = 2; // Transition to Ramp to Peak
    timeAtStateStart = time;
    Serial.print("Transition to Ramp to Peak, time is: ");
    Serial.println(time, 10);
  }
}

void handleRampToPeak(double &temp, double &timeAbove217, double &time) {
  if (temp < 245) {
    temp += RampToPeakRate / 2000.0;
  }
  if (temp > 217) {
    timeAbove217 += 1 / 2000.0;
  }
  if (timeAbove217 > ReflowTime) { // Transition to Cooling
    State = 3;
    TimeAtStateStart = time; // Use the global variable directly if appropriate
  }
}


void handleCooling(double &temp) {
  // Implement cooling logic here
  // For example, you might want to decrease the temperature gradually
  temp -= 1.0 / 2000.0; // Example cooling rate, adjust as necessary
  if (temp <= 30) { // Example: Reset to initial state if temp <= 30
    State = 3; //Keep in Cooling, do not start again
    temp = 30;
    Serial.println("Transition to Initial State");
  }
}


double calculateCurrentTemperature(double measuredVoltage) {
  double thermistorResistance = (SupplyVoltage - measuredVoltage) / measuredVoltage * ReferenceResistance * 1000; // Convert to Ohms
  double currentTemperature = 0;
  int left = 0;
  int right = sizeof(Temp2Resistance)/sizeof(Temp2Resistance[0]) - 1;

  while (left <= right) {
    int mid = left + (right - left) / 2;
    if (mid == 0 || mid == right || (thermistorResistance <= Temp2Resistance[mid - 1] && thermistorResistance >= Temp2Resistance[mid])) {
      // Linear interpolation for temperature
      double tempDiff = Temp2Resistance[mid - 1] - Temp2Resistance[mid];
      double resistanceDiff = thermistorResistance - Temp2Resistance[mid];
      currentTemperature = (mid - 1) + resistanceDiff / tempDiff;
      break;
    } else if (thermistorResistance < Temp2Resistance[mid]) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }

  return currentTemperature;
}


void controlHeatingElement(double currentTemperature) {
  // Check if the current temperature is below the target temperature minus a threshold
  if (currentTemperature < Temp - HeatingHysteresis) {
    // If it's too cold, turn the heating elements ON
    digitalWrite(digitalOutPin1, HIGH);
    digitalWrite(digitalOutPin2, HIGH);
    digitalWrite(LED_BUILTIN, HIGH); // Optional: turn on built-in LED to indicate heating
  } 
  // Check if the current temperature is above the target temperature plus a threshold
  else if (currentTemperature > Temp + HeatingHysteresis) {
    // If it's too hot, turn the heating elements OFF
    digitalWrite(digitalOutPin1, LOW);
    digitalWrite(digitalOutPin2, LOW);
    digitalWrite(LED_BUILTIN, LOW); // Optional: turn off built-in LED to indicate not heating
  }
  else if (toggleStatusLED){
    digitalWrite(digitalOutPin1, LOW);
    digitalWrite(digitalOutPin2, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  else {
    digitalWrite(digitalOutPin1, LOW);
    digitalWrite(digitalOutPin2, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }

  toggleStatusLED = (toggleStatusLED + 1) % 2; //toggle LED between the two endpoint
  // Optional: Handle the case where the temperature is within the acceptable range (hysteresis)
  // This prevents rapid switching of the heating elements near the target temperature
  // else {
    // Code to maintain the current state of the heating elements
  // }
}


void printDebugInfo(double currentTemperature) {
    double timeCopy = Time; // Make a copy of the volatile variable Time

    // Sending two values - timeCopy and currentTemperature
    //Serial.print(timeCopy);
    //Serial.print(" ");
    //Serial.println(currentTemperature);
}


//5. Loop Function
void loop() {
  // The main loop remains empty as functionality is interrupt-driven
/*  Serial.print("Temperature:");
  Serial.print(Temp);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(10);*/
}