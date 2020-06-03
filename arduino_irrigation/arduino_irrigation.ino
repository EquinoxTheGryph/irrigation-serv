// Libraries needed: ArduinoJson, Adafruit_ADS1015, Adafruit_DHT (Temperature and humidty sensor)
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <DHT.h>

/*
 * Modes:
 *   0  just controlling the valve
 *   1  controlling the valve based on flow limit
 *
 * JSON format: (<0-x> means an integer range)
 *   Input:
 *      {
 *          "targetValvePos0": <0-100>,
 *          "targetValvePos1": <0-100>,
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
 *          "targetValvePos": [
 *              <0-100>,
 *              <0-100>
 *          ],
 *          "airTemperature": <0-x>,
 *          "enclosureTemperature": <0-x>,
 *          "airHumidity": <0-x>,
 *          "doorOpen": <true or false>,
 *          "avgSoilHumidity": <0-x>,
 *          "valve0FullyOpen": bool,
 *          "valve0FulllClosed": bool,
 *          "valve1FullyOpen": bool,
 *          "valve1FullyClosed": bool,
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
#define PIN_VALVE_MOTOR_A0  9
#define PIN_VALVE_MOTOR_A1  8
#define PIN_VALVE_MOTOR_B0  11
#define PIN_VALVE_MOTOR_B1  10
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
#define C_REPORT_INTERVAL   5000    // The default amount of time (ms) between sending sensor data via serial
#define C_VALVE_OPEN_TIME   4100    // The amount of time (ms) it takes to open the valve
#define C_VALVE_CLOSE_TIME  4100    // The amount of time (ms) it takes to close the valve
#define C_BAUD_RATE         9600    // Sets the serial Baud rate
#define C_ADC_BASE_ADDR     0x48    // Sets the base I2C address of the ADS1115
#define C_MOIST_SENS_AMOUNT 4       // Sets the amount of moisture sensors connected via the external ADC's
#define C_FLOW_SENS_AMOUNT  2       // Sets the amount of flow sensors connected
#define C_MAX_FLOW_RATE     35      // FLOW SENSOR Maximum flow rate (in L/min)
#define C_FLOW_CALIBRATION  7.71    // FLOW SENSOR Pulses per second per litre/minute of flow. (Pulses per liter / 60)
#define C_DHT_TYPE          DHT22   // Set DHT (Temperature and Humidity sensor) type

    // Enable/disable Extra Components, uncomment to enable
//#define ENABLE_MOISTURE_1           // Enables extra moisture sensors
//#define ENABLE_MOISTURE_2
//#define ENABLE_MOISTURE_3

    // Define Maximum Array Sizes
#define C_SIZE_INPUT        100
#define C_SIZE_MSG          100
#define C_SIZE_OUTPUT       400

    // Define helper functions
#define ARRAYSIZE(x)        (sizeof(x)/sizeof(*x))         // define a macro to calculate array size

    // These functions is used to calibrate the soil sensors individually
    // Set the minimum value to the value when using the sensor in very humid soil (Soil soaked in water)
    // Set the maximum value to the value when using the sensor in air (Sensor dried off)
    // Syntax: map(x, Max value, Min value, 0%, 100%)
#define SOILCAL_00(x)  (map(x, 15300, 9150, 0, 100))
#define SOILCAL_01(x)  (map(x, 15300, 8475, 0, 100))
#define SOILCAL_02(x)  (map(x, 15240, 8550, 0, 100))
#define SOILCAL_03(x)  (map(x, 14740, 8200, 0, 100))

#ifdef ENABLE_MOISTURE_1
  #define SOILCAL_10(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_11(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_12(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_13(x)  (map(x, 15000, 9000, 0, 100))
#endif
#ifdef ENABLE_MOISTURE_2
  #define SOILCAL_20(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_21(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_22(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_23(x)  (map(x, 15000, 9000, 0, 100))
#endif
#ifdef ENABLE_MOISTURE_3
  #define SOILCAL_30(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_31(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_32(x)  (map(x, 15000, 9000, 0, 100))
  #define SOILCAL_33(x)  (map(x, 15000, 9000, 0, 100))
#endif


// GLOBAL VARIABLES //

    // Input (From Serial)
int targetValvePos[2];
uint16_t reportInterval = C_REPORT_INTERVAL;

    // Output (To Serial)
uint16_t soilHumidity[C_MOIST_SENS_AMOUNT];
uint16_t avgSoilHumidity;
uint16_t airHumidity;
float flowRate[2];         // Currently picked up flow rate in L/min
float airTemperature;
float enclosureTemperature; // Temperature inside the enclosure
bool doorOpen;

// bool valve0FullyOpen;
// bool valve0FullyClosed;
// bool valve1FullyOpen;
// bool valve1FullyClosed;

    // Don't change these values outside their intended functions!
long targetValvePosMsec[2];
unsigned long lastCheckTime;
unsigned long lastTime;
int reportDelta = 0;
int delta = 0;
int motorState[2]; // 0 = stop, -1 = closing, 1 = opening

volatile byte pulseCount0;
volatile byte pulseCount1;


// CLASS DEFINITIONS //

Adafruit_ADS1115 Adc0(C_ADC_BASE_ADDR);

#ifdef ENABLE_MOISTURE_1
  Adafruit_ADS1115 Adc1(C_ADC_BASE_ADDR + 1);
#endif
#ifdef ENABLE_MOISTURE_2
  Adafruit_ADS1115 Adc2(C_ADC_BASE_ADDR + 2);
#endif
#ifdef ENABLE_MOISTURE_3
  Adafruit_ADS1115 Adc3(C_ADC_BASE_ADDR + 3);
#endif

DHT Dht0(PIN_TEMP_EXT, C_DHT_TYPE);
DHT Dht1(PIN_TEMP_INT, C_DHT_TYPE);


// TOP FUNCTIONS //

void setup() {
    // Initialize serial port
    Serial.begin(C_BAUD_RATE);
    while (!Serial) continue;

    // Initialize pins
    pinMode(PIN_STATUS, OUTPUT);
    pinMode(PIN_FLOW_METER_0, INPUT);
    pinMode(PIN_FLOW_METER_1, INPUT);
    pinMode(PIN_DOOR, INPUT_PULLUP);

    pinMode(PIN_VALVE_OPEN_0, INPUT_PULLUP);
    pinMode(PIN_VALVE_CLOSE_0, INPUT_PULLUP);
    pinMode(PIN_VALVE_OPEN_1, INPUT_PULLUP);
    pinMode(PIN_VALVE_CLOSE_1, INPUT_PULLUP);

    pinMode(PIN_VALVE_MOTOR_A0, OUTPUT);
    pinMode(PIN_VALVE_MOTOR_A1, OUTPUT);
    pinMode(PIN_VALVE_MOTOR_B0, OUTPUT);
    pinMode(PIN_VALVE_MOTOR_B1, OUTPUT);

    // Initialize ADCs
    Adc0.begin();
    
    #ifdef ENABLE_MOISTURE_1
      Adc1.begin();
    #endif
    #ifdef ENABLE_MOISTURE_2
      Adc2.begin();
    #endif
    #ifdef ENABLE_MOISTURE_3
      Adc3.begin();
    #endif

    // Initialize DHTs
    Dht0.begin();
    Dht1.begin();

    // Attach interrupts for the water flow sensors
    setInterrupts(true);

    // Move valves to home position
    targetValvePosMsec[0] = C_VALVE_CLOSE_TIME;
    targetValvePosMsec[1] = C_VALVE_CLOSE_TIME;
    setMotor(0, -1);
    setMotor(1, -1);
}

void loop() {
    recieveStates();

    // Perform functions at set intervals
    unsigned long currentCheckTime = millis();
    delta = millis() - lastTime;
    lastTime = currentCheckTime;
    reportDelta = currentCheckTime - lastCheckTime;
    if (currentCheckTime - lastCheckTime >= reportInterval) {
        lastCheckTime = currentCheckTime;

        // Timout reached, Perform these functions below
        updateSensorValues();
        sendStates();
    }

    checkValves();
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

    // Build subarray containing the target valve positions
    JsonArray arr2 = jbuf.createNestedArray("targetValvePos");
    for (int i = 0; i < ARRAYSIZE(targetValvePos); i++) {
        arr2.add(targetValvePos[i]);
    }

    
    jbuf["valve0FullyOpen"]   = digitalRead(PIN_VALVE_OPEN_0);
    jbuf["valve0FulllClosed"] = digitalRead(PIN_VALVE_CLOSE_0);
    jbuf["valve1FullyOpen"]   = digitalRead(PIN_VALVE_OPEN_1);
    jbuf["valve1FullyClosed"] = digitalRead(PIN_VALVE_CLOSE_1);


    // TODO: Maybe add current valve positions

    // Add additional values
    jbuf["airHumidity"] = airHumidity;
    jbuf["airTemperature"] = airTemperature;
    jbuf["enclosureTemperature"] = enclosureTemperature;
    jbuf["doorOpen"] = doorOpen;
    jbuf["avgSoilHumidity"] = avgSoilHumidity;

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
        int valvePos0 = getJsonKeyValueAsInt(jsonInput, "targetValvePos0", targetValvePos[0]);
        int valvePos1 = getJsonKeyValueAsInt(jsonInput, "targetValvePos1", targetValvePos[1]);
        reportInterval = getJsonKeyValueAsInt(jsonInput, "reportInterval", reportInterval);

        // Set reportInterval to default if below 1000 ms (Avoids spamming)
        reportInterval = (reportInterval >= 1000) ? reportInterval : C_REPORT_INTERVAL;

        // Move valves if requested
        if (valvePos0 != targetValvePos[0])
            setValve(0, valvePos0);
        if (valvePos1 != targetValvePos[1])
            setValve(1, valvePos1);

        // Wait until the remaining data has been recieved and then make sure the serial buffer is empty.
        delay(10);
        while (Serial.available() > 0) {
            char t = Serial.read();
        }

        // Publish the current values
        publishArray("targetValvePos", targetValvePos, ARRAYSIZE(targetValvePos));
        publish("reportInterval", reportInterval);
    }
}

// Read all sensors and update global values
void updateSensorValues(){
    updateSoilHumidity();
    updateFlowRate();
    updateTemperatureHumidity();

    doorOpen = digitalRead(PIN_DOOR);
}

// Turn the valve until it (predictivly) reaches the desired state. (0-100)
void setValve(byte which, uint8_t percentage) {
    int turnLength;
    int direction;
    unsigned long * selectedMsec;
    int * selectedPerc;

    // Set references to the global variables
    selectedMsec = &targetValvePosMsec[which];
    selectedPerc = &targetValvePos[which];

    // Set the direction which the valve should move to
    if (percentage > *selectedPerc)
        direction = 1;
    else if (percentage < *selectedPerc)
        direction = -1;
    else
        direction = 0;

    // Get the opening or closing time, depending on the direction
    turnLength = (direction == -1) ? C_VALVE_CLOSE_TIME : C_VALVE_OPEN_TIME;
    int percentageChange = abs(*selectedPerc - percentage);

    // Determine for how long the valve should turn
    switch(percentage) {
        case 0:
            *selectedMsec = C_VALVE_CLOSE_TIME + 2000; // Add some time to ensure proper sealing
            break;
        case 100:
            *selectedMsec = C_VALVE_OPEN_TIME + 2000;
            break;
        default:
            *selectedMsec = (turnLength / 100) * percentageChange;
            break;
    }

    // Update target percentage
    *selectedPerc = percentage;

    // Move the motor in the desired direction
    setMotor(which, direction);
}

// Check wheter a valve should be stopped
void checkValves() {
    // Uses delta
    // should we publish an updated value?
    bool shouldPublish = false;

    // Count down currently moving motors, stop them if they have reached their target
    for (int i; i < ARRAYSIZE(targetValvePos); i++) {
        if (targetValvePosMsec[i] > 0 && motorState[i] != 0) {
            targetValvePosMsec[i] -= delta;
        } else if (targetValvePosMsec[i] <= 0 && motorState[i] != 0) {
            setMotor(0, 0);
        }
    }

    // If a valve touched a limit switch, stop that valve.
    if (!digitalRead(PIN_VALVE_CLOSE_0) && motorState[0] == -1) {
        setMotor(0, 0);
        targetValvePosMsec[0] = 0;
        targetValvePos[0] = 0;
        shouldPublish = true;
    }

    if (!digitalRead(PIN_VALVE_OPEN_0) && motorState[0] == 1) {
        setMotor(0, 0);
        targetValvePosMsec[0] = 0;
        targetValvePos[0] = 100;
        shouldPublish = true;
    }

    if (!digitalRead(PIN_VALVE_CLOSE_1) && motorState[1] == -1) {
        setMotor(1, 0);
        targetValvePosMsec[1] = 0;
        targetValvePos[1] = 0;
        shouldPublish = true;
    }

    if (!digitalRead(PIN_VALVE_OPEN_1) && motorState[1] == 1) {
        setMotor(1, 0);
        targetValvePosMsec[1] = 0;
        targetValvePos[1] = 100;
        shouldPublish = true;
    }

    if (shouldPublish) {
        publishArray("targetValvePos", targetValvePos, 2);
    }
}

// Turn or stop valve motor. direction: 0 = Stop, -1 = Close, 1 = Open
void setMotor(int which, int direction) {
    int outPin0;
    int outPin1;

    // Determine which valve to turn
    switch(which) {
    case 0:
        outPin0 = PIN_VALVE_MOTOR_A0;
        outPin1 = PIN_VALVE_MOTOR_A1;
        motorState[0] = direction;
        break;
    case 1:
        outPin0 = PIN_VALVE_MOTOR_B0;
        outPin1 = PIN_VALVE_MOTOR_B1;
        motorState[1] = direction;
        break;
    }

    // Move the valves accodingly
    switch(direction) {
    case 0: // STOP
        digitalWrite(outPin0, 0);
        digitalWrite(outPin1, 0);
        break;
    case -1: // CLOSE
        digitalWrite(outPin0, 0);
        digitalWrite(outPin1, 1);
        break;
    case 1: // OPEN
        digitalWrite(outPin0, 1);
        digitalWrite(outPin1, 0);
        break;
    }
}

// Read all I2C ADC values (Including the average value), used for soil sensors.
// Sets: soilHumidity, avgSoilHumidity
void updateSoilHumidity(){
    // Get individual values and apply calibrations
//    for(int i = 0; i < C_MOIST_SENS_AMOUNT; i++) {
//        if(i < 4) {
//          soilHumidity[i]    = Adc0.readADC_SingleEnded(i);//SOILCALIBRATION(Adc0.readADC_SingleEnded(i));
//        }
//        #ifdef ENABLE_MOISTURE_1
//          if(i >= 4 and i < 8) {
//            soilHumidity[i]  = SOILCALIBRATION(Adc1.readADC_SingleEnded(i - 4));
//          }
//        #endif
//        #ifdef ENABLE_MOISTURE_2
//          if(i >= 8 and i < 12) {
//            soilHumidity[i]  = SOILCALIBRATION(Adc2.readADC_SingleEnded(i - 8));
//          }
//        #endif
//        #ifdef ENABLE_MOISTURE_3
//          if(i >= 12 and i < 16) {
//            soilHumidity[i] = SOILCALIBRATION(Adc3.readADC_SingleEnded(i - 12));
//          }
//        #endif
//    }
    soilHumidity[0] =       SOILCAL_00(Adc0.readADC_SingleEnded(0));
    soilHumidity[1] =       SOILCAL_01(Adc0.readADC_SingleEnded(1));
    soilHumidity[2] =       SOILCAL_02(Adc0.readADC_SingleEnded(2));
    soilHumidity[3] =       SOILCAL_03(Adc0.readADC_SingleEnded(3));
    
    #ifdef ENABLE_MOISTURE_1
      soilHumidity[4] =     SOILCAL_10(Adc1.readADC_SingleEnded(0));
      soilHumidity[5] =     SOILCAL_11(Adc1.readADC_SingleEnded(1));
      soilHumidity[6] =     SOILCAL_12(Adc1.readADC_SingleEnded(2));
      soilHumidity[7] =     SOILCAL_13(Adc1.readADC_SingleEnded(3));
    #endif
    #ifdef ENABLE_MOISTURE_2
      soilHumidity[8]  =    SOILCAL_20(Adc2.readADC_SingleEnded(0));
      soilHumidity[9]  =    SOILCAL_21(Adc2.readADC_SingleEnded(1));
      soilHumidity[10] =    SOILCAL_22(Adc2.readADC_SingleEnded(2));
      soilHumidity[11] =    SOILCAL_23(Adc2.readADC_SingleEnded(3));
    #endif
    #ifdef ENABLE_MOISTURE_3
      soilHumidity[12] =    SOILCAL_30(Adc3.readADC_SingleEnded(0));
      soilHumidity[13] =    SOILCAL_31(Adc3.readADC_SingleEnded(1));
      soilHumidity[14] =    SOILCAL_32(Adc3.readADC_SingleEnded(2));
      soilHumidity[15] =    SOILCAL_33(Adc3.readADC_SingleEnded(3));
    #endif
   

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
    flowRate[0] = ((1000.0 / reportDelta) * pulseCount0) / C_FLOW_CALIBRATION;
    flowRate[1] = ((1000.0 / reportDelta) * pulseCount1) / C_FLOW_CALIBRATION;

    // Reset pulse counters
    pulseCount0 = 0;
    pulseCount1 = 0;

    // enable interrupts
    setInterrupts(true);
}

// Get temperature and humidity, internal and external
// Sets: enclosureTemperature, airTemperature, airHumidity
// TODO Note: Might return NaN if dht is not found
void updateTemperatureHumidity() {
    enclosureTemperature = Dht1.readTemperature();
    airTemperature = Dht0.readTemperature();
    airHumidity = Dht0.readHumidity();
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

// Publish a value now
void publish(String key, int value) {
    StaticJsonDocument<C_SIZE_MSG> jbuf;
    jbuf[key] = value;
    // Send data
    serializeJson(jbuf, Serial);
    // Make sure to print a new line
    Serial.println();
    //Serial.flush();
}

// Publish a value as array now
void publishArray(String key, int values[], int size) {
    StaticJsonDocument<C_SIZE_MSG> jbuf;

    // Put convert the array to a json array
    JsonArray arr0 = jbuf.createNestedArray(key);
    for (int i = 0; i < size; i++) {
        arr0.add(values[i]);
    }

    // Send data
    serializeJson(jbuf, Serial);
    // Make sure to print a new line
    Serial.println();
    //Serial.flush();
}

// Get a json key value, use defaultVal if key is not found
// NOTE: I'm not sure if using StaticJsonDocument for this is right, but it works...
int getJsonKeyValueAsInt(StaticJsonDocument<C_SIZE_INPUT> doc, String key, int defaultVal) {
    JsonVariant outVariant = doc.getMember(key);
    int out = (!outVariant.isNull()) ? outVariant.as<int>() : defaultVal;
    return out;
}
