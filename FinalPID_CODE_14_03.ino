#include <TimerOne.h>  // Timer library for TRIAC control This code is perfect only sudden drop of power when target temperature met 19/03/2025
#include <math.h>

// ******************** Pin Definitions ********************
const int sensorPin = A0;     // Analog pin for temperature sensor input
const int triacPin = 3;       // Output pin to control the TRIAC (heater)
const int zeroCrossPin = 2;   // Input pin for zero-cross detection

// ******************** Temperature Sensor Constants ********************
const float Rref = 467.0;     // Reference resistance for PT100
const float R0 = 500.0;       // Resistance at 0°C
const float A = 3.9083e-3;    // Callendar-Van Dusen equation coefficient A
const float B = -5.775e-7;    // Callendar-Van Dusen equation coefficient B
#define VREF 4.95             // Reference voltage for ADC
const int numSamples = 20;    // Number of samples for moving average filter

// ******************** AC Power Control Constants ********************
#define MIN_DELAY 10          // Minimum delay before TRIAC fires (full power)
#define AC_FREQUENCY 50       
#define MAX_DELAY 10500

// ******************** Control Variables ********************
volatile boolean cruce_cero = false;  // Zero-cross detection flag
volatile float temperature = 0.0;     // Stores current temperature reading
float targetTemperature = 50.0;       // Desired temperature setpoint
unsigned long lastTempReadTime = 0;   // Last time temperature was read
unsigned long previousMicros = 0;     // Last time dimmer function was called

// ******************** PID Control Variables ********************
float prevError = 0;   // Stores previous error value for derivative calculation
float integral = 0;    // Stores accumulated error for integral term
float dim = MAX_DELAY; // Initial power level (lowest power)
float lastDim = MAX_DELAY; // Stores previous dim value


// PID tuning parameters
const float Kp = 3.0;  // Proportional gain (faster response)
const float Ki = 0.0; // Integral gain (eliminates steady-state error)
const float Kd = 0.0;  // Derivative gain (prevents overshooting)

void setup() {
    Serial.begin(9600);               // Initialize serial communication for debugging
    pinMode(triacPin, OUTPUT);        // Set TRIAC pin as output
    attachInterrupt(digitalPinToInterrupt(zeroCrossPin), deteccion_Cruce_cero, RISING); // Attach zero-cross detection interrupt
}

// ******************** Interrupt Service Routine (ISR) for Zero-Cross Detection ********************
void deteccion_Cruce_cero() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = micros();

    if (interruptTime - lastInterruptTime > 4000) { // 4ms debounce to filter noise
        cruce_cero = true;
    }
    
    lastInterruptTime = interruptTime;
    digitalWrite(triacPin, LOW);
}

// ******************** Read Temperature from Sensor ********************
float readTemperature() {
    float analogValue = readSensor();
    float voltage = analogValue * (VREF / 1023.0); // Convert ADC value to voltage 
    if (voltage >= 4.9) {
        return NAN;
    } 
    float Rpt500 = Rref * voltage / (VREF - voltage);
    return calculateTemperature(Rpt500);
}

// ******************** Read Sensor Value with Moving Average Filter ********************
float readSensor() {
    static float readings[numSamples] = {0}; // Array to store previous readings
    static int index = 0;                    // Index for circular buffer
    static float sum = 0;                     // Sum of readings for averaging

    sum -= readings[index];                  
    readings[index] = analogRead(sensorPin);
    sum += readings[index];                   
    index = (index + 1) % numSamples;         

    return sum / numSamples;                  
}

// ******************** Calculate Temperature using Callendar-Van Dusen Equation ********************
float calculateTemperature(float R) {
    float discriminant = A * A - 4 * B * (1.0 - (R / R0)); 
    if (discriminant < 0) {
        return NAN;
    }   
    return (-A + sqrt(discriminant)) / (2 * B);
}

// ******************** PID Control for TRIAC Firing Angle with Power Limit ********************
void updatePhaseAngle() {
    float error = targetTemperature - temperature;

    // Reset integral if error is small to prevent wind-up
    if (abs(error) < 1) {
        integral = 0;
    } else {
        integral += error;
    }

    float derivative = error - prevError;
    prevError = error;

    // Compute PID output
    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Convert PID output to dimming value
    int newDim = constrain(map(pidOutput, -50, 100, MAX_DELAY, MIN_DELAY), MIN_DELAY, MAX_DELAY * 0.9);

    static float prevTemperature = temperature;
    static int prevDim = dim;

    // Small random fluctuation for power to prevent constant power level
    int fluctuation = random(-3, 3); // Adds small variations to power changes

    if (temperature < prevTemperature || (temperature > targetTemperature && error < 2)) {
        if (dim < newDim) {
            dim += 1 + fluctuation;  // Gradually increase power with small variations
        }
    } 
    else if (dim > newDim) {  
        dim -= max(3, (dim - newDim) / 5) + fluctuation; // Gradual power decrease with variations
    } 
    else {
        dim = newDim + fluctuation; // Keep power fluctuating slightly even when stable
    }

    // Prevent power from stabilizing at a fixed value
    if (abs(dim - prevDim) < 2) {
        dim += random(-2, 2); // Force minor variations to avoid a constant line in the graph
    }

    prevTemperature = temperature;
    prevDim = dim;

    // ******************** Debugging Output ********************
    Serial.print(millis()); 
    Serial.print(",");
    Serial.print(temperature); 
    Serial.print(",");
    Serial.println(calculatePowerPercentage()); 
    delay(100);
}



// ******************** Calculate Power Percentage ********************
// Function to compute actual power percentage
float calculatePowerPercentage() {
    if (dim >= MAX_DELAY) return 0.0;  // Ensure power shows 0% correctly

    float firing_angle = (float)dim / MAX_DELAY * 180.0;
    float power_percentage = (2 / M_PI) * (cos(radians(firing_angle)) - cos(radians(180.0))) * 100.0;

    // Add 20% correction to compensate for TRIAC non-linearity
    power_percentage += 15;

    // Ensure it does not exceed 100%
    return constrain(power_percentage, 0, 100);
}

// ******************** TRIAC Dimming Function ********************
void Dimer() {
    if (cruce_cero) {  
        if (dim < MAX_DELAY) {  // Only fire TRIAC if dim is not at max delay (0% power)
            delayMicroseconds(dim);
            digitalWrite(triacPin, HIGH);
            delayMicroseconds(2000);
            digitalWrite(triacPin, LOW);
        }
        cruce_cero = false;
    }
}

// ******************** Main Loop ********************
void loop() {
    if (millis() - lastTempReadTime >= 50) {
        lastTempReadTime = millis();
        temperature = readTemperature();
        if (!isnan(temperature)) {
            updatePhaseAngle();  
        }
    }

    if (cruce_cero) {  
        Dimer();  
    }
}
