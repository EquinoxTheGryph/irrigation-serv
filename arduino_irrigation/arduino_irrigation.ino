// Libraries needed: ArduinoJson, DRV8833? (https://github.com/Racoun/DRV8833), Adafruit_ADS1015
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

/*
 * Modes:
 *   0  just controlling the valve
 *   1  controlling the valve based on flow limit
 *
 * JSON format: (<0-x> means an integer range)
 *   Input:
 *      {
 *          "mode": <0-1>,
 *          "targetValvePos": <0-255>,
 *          "flowLimit": <0-x>,
 *          "reportInterval": <1000-x>  // Under 1000 will reset the interval to C_REPORT_INTERVAL
 *      }
 *   Output:
 *      {
 *          "timeStamp": <0-x>,
 *          "soilHumidity": [
 *              <0-x>,
 *              <0-x>,
 *              ...
 *          ],
 *          "flowRate": [
 *              <0-x>,
 *              <0-x>
 *          ],
 *          "airTemperature": <0-x>,
 *          "enclosureTemperature": <0-x>,
 *          "airHumidity": <0-x>,
 *          "movementDetected": <true or false>,
 *          "targetValvePos": <0-x>,
 *
 *              // Also report back the current input values
 *          "mode": <0-1>,
 *          "targetValvePos": <0-255>,
 *          "flowLimit": <0-x>,
 *      }
 *
 * TODO:
 *      get sensor data
 *      Maybe report an average soilHumidity value (To make automation easier)
 *
 * IDEAS:
 *      Maybe implement a RTC for an internet-less clock
 */


// DEFINITIONS //

    // Define output pins for the valve
#define PIN_VALVE_MOTOR_A0  8
#define PIN_VALVE_MOTOR_A1  9
#define PIN_VALVE_MOTOR_B0  10
#define PIN_VALVE_MOTOR_B1  11
    // Define input pins for the valve state
#define PIN_VALVE_OPEN_0    4
#define PIN_VALVE_CLOSE_0   5
#define PIN_VALVE_OPEN_1    6
#define PIN_VALVE_CLOSE_1   7
    // Define input pin for the flow meter sensor (Must be an interrupt)
#define PIN_FLOW_METER_0    2
#define PIN_FLOW_METER_1    3
    // Define output pin for a simple status LED
#define PIN_STATUS          13
    // Define Other sensors
#define PIN_DOOR            A3
#define PIN_TEMP_INT        A0
#define PIN_TEMP_EXT        A2
    // Define I2C pins
#define PIN_SDA             A4
#define PIN_SCL             A5

    // Define Constants
#define C_REPORT_INTERVAL   4000    // The default amount of time (ms) between sending sensor data via serial
#define C_VALVE_OPEN_TIME   3000    // The amount of time (ms) it takes to open the valve
#define C_VALVE_CLOSE_TIME  5000    // The amount of time (ms) it takes to close the valve
#define C_BAUD_RATE         9600    // Sets the serial Baud rate
#define C_ADC_BASE_ADDR     0x48    // Sets the base I2C address of the ADS1115
#define C_MOIST_SENS_AMOUNT 16      // Sets the amount of moisture sensors connected via the external ADC's
#define C_FLOW_SENS_AMOUNT  2       // Sets the amount of flow sensors connected
#define C_MAX_FLOW_RATE     35      // FLOW SENSOR Maximum flow rate (in L/min)
#define C_FLOW_CALIBRATION  7.71    // FLOW SENSOR Pulses per second per litre/minute of flow. (Pulses per liter / 60)

    // Define Maximum Array Sizes
#define C_SIZE_INPUT        100
#define C_SIZE_MSG          100
#define C_SIZE_OUTPUT       400

    // Define helper functions
#define ARRAYSIZE(x)        (sizeof(x)/sizeof(*x))         // define a macro to calculate array size
#define SOILCALIBRATION(x)  (map(x, 15000, 9000, 0, 100))  // this function is used to calibrate the soil sensors


// GLOBAL VARIABLES //

    // Input (From Serial)
uint8_t mode = 0;          // Modes: 0 = just controlling the valve, 1 = controlling the valve based on flow limit
uint8_t targetValvePos = 0;
float flowLimit = 0;        // Maximum flow rate in L/min
uint16_t reportInterval = C_REPORT_INTERVAL;

    // Output (To Serial)
uint16_t soilHumidity[C_MOIST_SENS_AMOUNT];
uint16_t avgSoilHumidity;
uint16_t airHumidity = 0;
float flowRate[2];         // Currently picked up flow rate in L/min
float airTemperature = 0;
float enclosureTemperature = 0; // Temperature inside the enclosure
bool movementDetected = false;

    // Don't change these values outside their intended functions!
uint8_t currentValvePos = 0;
unsigned long lastCheckTime = 0;
int delta = 0;

volatile byte pulseCount0 = 0;
volatile byte pulseCount1 = 0;


// CLASS DEFINITIONS //

Adafruit_ADS1115 Adc0(C_ADC_BASE_ADDR);
Adafruit_ADS1115 Adc1(C_ADC_BASE_ADDR + 1);
Adafruit_ADS1115 Adc2(C_ADC_BASE_ADDR + 2);
Adafruit_ADS1115 Adc3(C_ADC_BASE_ADDR + 3);


// TOP FUNCTIONS //

void setup() {
    // Initialize serial port
    Serial.begin(C_BAUD_RATE);
    while (!Serial) continue;

    // Initialize pins
    pinMode(PIN_STATUS, OUTPUT);
    pinMode(PIN_FLOW_METER_0, INPUT);
    pinMode(PIN_FLOW_METER_1, INPUT);

    // Initialize ADCs
    Adc0.begin();
    Adc1.begin();
    Adc2.begin();
    Adc3.begin();

    // Attach interrupts for the water flow sensors
    setInterrupts(true);
}

void loop() {
    recieveStates();

    // Perform functions at set intervals
    unsigned long currentCheckTime = millis();
    delta = currentCheckTime - lastCheckTime;
    if (currentCheckTime - lastCheckTime >= reportInterval) {
        lastCheckTime = currentCheckTime;

        // Timout reached, Perform these functions below
        updateSensorValues();
        sendStates();
    }
}


// EXTRA FUNCTIONS //

// Write Sensor data to the Serial port
void sendStates() {
    StaticJsonDocument<C_SIZE_OUTPUT> jbuf;

    // Add time stamp, to make sure the output is unique
    jbuf["timeStamp"] = millis();

    // Build subarray containing the soil humidity values
    JsonArray arr0 = jbuf.createNestedArray("soilHumidity");
    for (int i = 0; i < C_MOIST_SENS_AMOUNT; i++) {
        arr0.add(soilHumidity[i]);
    }

    // Build subarray containing the flow rate values
    JsonArray arr1 = jbuf.createNestedArray("flowRate");
    for (int i = 0; i < C_FLOW_SENS_AMOUNT; i++) {
    arr1.add(flowRate[i]);
    }

    // Add additional values
    jbuf["airHumidity"] = airHumidity;
    /* jbuf["flowRate"] = flowRate; */
    jbuf["airTemperature"] = airTemperature;
    jbuf["enclosureTemperature"] = enclosureTemperature;
    jbuf["movementDetected"] = movementDetected;
    jbuf["currentValvePos"] = currentValvePos;
    jbuf["avgSoilHumidity"] = avgSoilHumidity;

    jbuf["targetValvePos"] = targetValvePos;
    jbuf["mode"] = mode;
    jbuf["flowLimit"] = flowLimit;

    // Output data to serial
    serializeJson(jbuf, Serial);

    // Make sure to print a new line
    Serial.println();
    Serial.flush();
}


// Read Serial data and parse values
void recieveStates() {

    if (Serial.available()) {

        StaticJsonDocument<C_SIZE_INPUT> jsonInput;
        DeserializationError err = deserializeJson(jsonInput, Serial);

        // Test if parsing succeeds.
        if (!err == DeserializationError::Ok) {
            printError(err.c_str());
            return;
        }

        // Parsing succeeds! Set variables.
        // [ EXPRESSION ? A : B ] -> IF result of EXPRESSION is TRUE, Return A, otherwise Return B
        mode = (jsonInput["mode"] >= 0) ? jsonInput["mode"] : mode;
        targetValvePos = (jsonInput["targetValvePos"] >= 0) ? jsonInput["targetValvePos"] : targetValvePos;
        flowLimit = (jsonInput["flowLimit"] >= 0) ? jsonInput["flowLimit"] : flowLimit;
        if (jsonInput["reportInterval"] >= 0) {
            reportInterval = (jsonInput["reportInterval"] >= 1000) ? jsonInput["reportInterval"] : C_REPORT_INTERVAL;
        }

        // TEST Set LED to mode status
        digitalWrite(PIN_STATUS, mode);

        // Wait until the remaining data has been recieved and then make sure the serial buffer is empty.
        delay(10);
        while (Serial.available() > 0) {
            char t = Serial.read();
        }
    }
}

// Read all sensors and update global values TODO: Currently these are just random values until i get some acual sensors!
void updateSensorValues(){
    // TODO
    /* UPDATE THESE SENSORS
        soilHumidity[C_MOIST_SENS_AMOUNT]
        airHumidity
        flowRate       // Currently picked up flow rate in L/min
        airTemperature
        enclosureTemperature // Temperature inside the enclosure
        movementDetected
    */

    //for(int i = 0; i < C_MOIST_SENS_AMOUNT; i++) {
    //    soilHumidity[i] = random(1, 1023);
    //}

    airHumidity = random(1, 1023);
    //flowRate = random(1, 1023);       // Currently picked up flow rate in L/min
    airTemperature = random(1, 30);
    enclosureTemperature = random(1, 50); // Temperature inside the enclosure
    movementDetected = (random(0, 10) >= 5);

    // COMPLETED SENSORS
    updateSoilHumidity();
    updateFlowRate();
}

// Turn the valve until it (predictivly) reaches the desired state. (0-255)
void setValve(uint8_t amount) {
    // TODO
}

// Read all I2C ADC values (Including the average value), used for soil sensors.
// Sets: soilHumidity, avgSoilHumidity
void updateSoilHumidity(){
    // Get individual values and apply calibrations
    for(int i; i < C_MOIST_SENS_AMOUNT / 4; i++) {
        soilHumidity[i]    = SOILCALIBRATION(Adc0.readADC_SingleEnded(i));
        soilHumidity[i+4]  = SOILCALIBRATION(Adc1.readADC_SingleEnded(i));
        soilHumidity[i+8]  = SOILCALIBRATION(Adc2.readADC_SingleEnded(i));
        soilHumidity[i+12] = SOILCALIBRATION(Adc3.readADC_SingleEnded(i));
    }

    // Calculate the average soil humidity
    uint16_t currentAvg = soilHumidity[0];
    for (int i = 1; i < C_MOIST_SENS_AMOUNT; i++) {
        currentAvg = (currentAvg + soilHumidity[i]) / 2;
    }
    avgSoilHumidity = currentAvg;
}

// Calculate the current flow rate of the flow sensors
// Sets: flowRate
void updateFlowRate(){
    // disable interrupts
    setInterrupts(false);

    // Convert counted pulses to Liter per Minute
    flowRate[0] = ((1000.0 / delta) * pulseCount0) / C_FLOW_CALIBRATION;
    flowRate[1] = ((1000.0 / delta) * pulseCount1) / C_FLOW_CALIBRATION;

    // Reset pulse counters
    pulseCount0 = 0;
    pulseCount1 = 0;

    // enable interrupts
    setInterrupts(true);
}

// Attach or detach interrupts for the water flow sensors
void setInterrupts(bool state) {
    if (state) {
        attachInterrupt(digitalPinToInterrupt(PIN_FLOW_METER_0), pulseInterrupt0, FALLING);
        attachInterrupt(digitalPinToInterrupt(PIN_FLOW_METER_1), pulseInterrupt1, FALLING);
    } else {
        detachInterrupt(pulseInterrupt0);
        detachInterrupt(pulseInterrupt1);
    }
}

// Interrupt function for water flow sensor pulses
void pulseInterrupt0() {
    pulseCount0++;
}

// Interrupt function for water flow sensor pulses
void pulseInterrupt1() {
    pulseCount1++;
}

// Print a simple error message in json form
void printError(char* errorMsg) {
    StaticJsonDocument<C_SIZE_MSG> jbuf;
    jbuf["error"] = errorMsg;
    // Send data
    serializeJson(jbuf, Serial);
    // Make sure to print a new line
    Serial.println();
    //Serial.flush();
}

// Print a simple debug message in json form
void printDebug(char* debugMsg) {
    StaticJsonDocument<C_SIZE_MSG> jbuf;
    jbuf["debug"] = debugMsg;
    // Send data
    serializeJson(jbuf, Serial);
    // Make sure to print a new line
    Serial.println();
    //Serial.flush();
}
