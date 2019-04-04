// Libraries needed: ArduinoJson, DRV8833 (https://github.com/Racoun/DRV8833), ADS1115
#include <ArduinoJson.h>

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
 *          "flowRate": <0-x>,
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
#define PIN_VALVE_MOTOR_A1  0
#define PIN_VALVE_MOTOR_A2  0
    // Define input pins for the valve state
#define PIN_VALVE_CLOSE     0
#define PIN_VALVE_OPEN      0
    // Define input pin for the flow meter sensor (Must be an interrupt)
#define PIN_FLOW_METER      2
    // TEST Define output pin for a simple status LED
#define PIN_STATUS          13
    // Define I2C pins
#define PIN_SDA             A4
#define PIN_SCL             A5

    // Define Constants
#define C_REPORT_INTERVAL   10000    // The default amount of time (ms) between sending sensor data via serial
#define C_VALVE_OPEN_TIME   3000    // The amount of time (ms) it takes to open the valve
#define C_VALVE_CLOSE_TIME  5000    // The amount of time (ms) it takes to close the valve
#define C_MAX_MOTOR_SPEED   254     // The maximum speed of the motor
#define C_BAUD_RATE         9600    // Sets the serial Baud rate
#define C_ADC_BASE_ADR      0x48    // Sets the base I2C address of the ADS1115
#define C_MOIST_SENS_AMOUNT 16      // Sets the amount of moisture sensors connected via the external ADC's
#define C_MAX_FLOW_RATE     45      // Maximum flow rate handled by the sensor (in L/min)

    // Define Maximum Array Sizes
#define C_SIZE_INPUT        100
#define C_SIZE_MSG          100
#define C_SIZE_OUTPUT       400


// GLOBAL VARIABLES //

    // Input (From Serial)
uint8_t mode = 0;          // Modes: 0 = just controlling the valve, 1 = controlling the valve based on flow limit
uint8_t targetValvePos = 0;
float flowLimit = 0;        // Maximum flow rate in L/min
uint16_t reportInterval = C_REPORT_INTERVAL;

    // Output (To Serial)
uint16_t soilHumidity[C_MOIST_SENS_AMOUNT];
uint16_t airHumidity = 0;
float flowRate = 0;         // Currently picked up flow rate in L/min
float airTemperature = 0;
float enclosureTemperature = 0; // Temperature inside the enclosure
bool movementDetected = false;

    // Don't change these values outside their intended functions!
uint8_t currentValvePos = 0;
unsigned long lastCheckTime = 0;


// CLASS DEFINITIONS //

// TOP FUNCTIONS //

void setup() {
    // Initialize serial port
    Serial.begin(C_BAUD_RATE);
    while (!Serial) continue;
    "mode": <0-1>,
 *          "targetValvePos": <0-255>,
 *          "flowLimit": <0-x>,
    // Initialize pins
    pinMode(PIN_STATUS, OUTPUT);
}

void loop() {
    recieveStates();
    
    // Perform functions at set intervals
    unsigned long currentCheckTime = millis();
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
    JsonArray arr = jbuf.createNestedArray("soilHumidity");
    for (int i = 0; i < C_MOIST_SENS_AMOUNT; i++) {
        arr.add(soilHumidity[i]);
    }
    
    // Add additional values
    jbuf["airHumidity"] = airHumidity;
    jbuf["flowRate"] = flowRate;
    jbuf["airTemperature"] = airTemperature;
    jbuf["enclosureTemperature"] = enclosureTemperature;
    jbuf["movementDetected"] = movementDetected;
    jbuf["currentValvePos"] = currentValvePos;
    jbuf["avgSoilHumidity"] = getAverageSoilHumidity();
    
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
    
    for(int i = 0; i < C_MOIST_SENS_AMOUNT; i++) {
        soilHumidity[i] = random(1, 1023);
    }
    airHumidity = random(1, 1023);
    flowRate = random(1, 1023);       // Currently picked up flow rate in L/min
    airTemperature = random(1, 30);
    enclosureTemperature = random(1, 50); // Temperature inside the enclosure
    movementDetected = (random(0, 10) >= 5);
}

// Turn the valve until it (predictivly) reaches the desired state. (0-255)
void setValve(uint8_t amount) {
    // TODO
}

//
int getAverageSoilHumidity() {
    int output = 0;
    for (int i = 0; i < C_MOIST_SENS_AMOUNT; i++) {
        output += soilHumidity[i];
    }
    return output / C_MOIST_SENS_AMOUNT;
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
