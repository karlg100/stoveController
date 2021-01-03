
#include <pid.h>
#include <Adafruit_Si7021.h>
//include <Grove_4Digit_Display.h>

//Adafruit_Si7021 sensor = Adafruit_Si7021();
//Adafruit_Si7021 sensor = Adafruit_Si7021(&Wire1);
Adafruit_Si7021 sensor(&Wire1);

#define MIN_TEMP 30
#define MAX_TEMP 100
#define DEFAULT_TARGETTEMP 80
#define DEFAULT_TARGETMODE 0

// High/low mode
#define COAL_HIGH_F 100
#define COAL_LOW_F 0

// Temp mode
//define COAL_HIGH_F 460
//define COAL_LOW_F 200

// ** PID Setup
//Define Variables we'll be connecting to
double Setpoint, Input, Output, OutputTemp;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, PID::DIRECT);

// time tracking vars
size_t nextReport = 0;
size_t nextPID = 0;

// working vars
double temp = MIN_TEMP;
int currentHeatingCoolingState;
double currentTemp;

#define CHECK_DELAY 5000
#define PUB_DELAY 20000

bool heating = false;


// EEPROM state struct and handlers
// Struct for storing state in RAM and EEPROM
struct CCState {
    
    int magicNumber = 0xC10001; // this should change when struct changes

    // Target temp to maintain output at
    int targetTemp;

    int targetHeatingCoolingState;

    // checkum value
    int checkSum = 0;
} cfgData;

//  EEPROM handing functions
#define CFG_BASE 0
bool readCfg()
{
  uint8_t *_data = (uint8_t*) &cfgData;
  //uint8_t* _data     = &cfgData;
  int      _chkSum   = 0;

  for (int i = 0; i < sizeof(cfgData); i++)
  {
    _data[i] = EEPROM.read(CFG_BASE + i);
    if (i < sizeof(cfgData) - sizeof(_chkSum))
      _chkSum += (_data[i] << i);
  }
  return (_chkSum == cfgData.checkSum);
}

int writeCfg()
{
  uint8_t *_data = (uint8_t*) &cfgData;
  //uint8_t* _data = &cfgData;
  cfgData.checkSum = 0;

  for (int i = 0; i < sizeof(cfgData); i++)
  {
    EEPROM.write(CFG_BASE + i, _data[i]);
    if (i < sizeof(cfgData) - sizeof(cfgData.checkSum))
      cfgData.checkSum += (_data[i] << i);
  }
  return cfgData.checkSum;
}

bool validCfg() {
    if (cfgData.targetTemp < MIN_TEMP)
        return false;
    if (cfgData.targetTemp > MAX_TEMP)
        return false;
    if (cfgData.targetHeatingCoolingState < 0)
        return false;
    if (cfgData.targetHeatingCoolingState > 2)
        return false;

    // everything looks good!  We can use these values
    return true;
}

int initCfg() {
    cfgData.targetTemp = DEFAULT_TARGETTEMP;
    cfgData.targetHeatingCoolingState = DEFAULT_TARGETMODE;
    writeCfg();
    Particle.publish("EEPROM", "New state initialized and written");
}

int setTargetTemp(String tempValue) {
    cfgData.targetTemp = tempValue.toInt();
    writeCfg();
    return cfgData.targetTemp;
}

int setTargetHeatingCoolingState(String stateValue) {
    cfgData.targetHeatingCoolingState = stateValue.toInt();
    writeCfg();
    currentHeatingCoolingState = cfgData.targetHeatingCoolingState;
    return cfgData.targetHeatingCoolingState;
}

void setupCfg() {
    // retrieve values from EEPROM or init if needed
    if ( !readCfg() ) {
        Particle.publish("EEPROM", "State failed checksum from EEPROM");
        initCfg();
    } else
        Particle.publish("EEPROM", "State successfully loaded from EEPROM");
    if ( !validCfg() ) {
        Particle.publish("EEPROM", "State values are out of range");
        initCfg();
    } else
        Particle.publish("EEPROM", "State has valid values");
}

bool sensorPresent = false;

void setup() {

    if (!sensor.begin())
        Particle.publish("Status", "Did not find Si7021 sensor!");
    else
        sensorPresent = true;

    // read/setup cfg    
    setupCfg();

    // register cloud functions and vars
    Particle.function("setMode", setTargetHeatingCoolingState);
    Particle.function("setTargetTemp", setTargetTemp);
    Particle.variable("getTargetTemp", &cfgData.targetTemp, INT);
    Particle.variable("getCurrentTemp", &currentTemp, DOUBLE);
    Particle.variable("getCurrentState", &currentHeatingCoolingState, INT);

    // setup status output LED
    pinMode(D7, OUTPUT);
    
    // Init telemetry
    getTelem();

    currentHeatingCoolingState = 0;
    
    // PID setup
    //initialize the variables we're linked to
    Input = (double)temp;
    Setpoint = (double)cfgData.targetTemp;
    myPID.SetMode(PID::AUTOMATIC);

    myPID.SetOutputLimits(0, 100);
    //myPID.SetOutputLimits(COAL_LOW_F, COAL_HIGH_F);
//    myPID.SetTunings(aggKp, aggKi, aggKd);

    // Calibrate/preseed PID output
    myPID.SetSampleTime(CHECK_DELAY);
    myPID.Compute();
}

void getTelem() {
    // get temp
    if (nextPID < millis()) {
        digitalWrite(D7, HIGH);
        temp = sensor.readTemperature() * 1.8 + 32.0;
        currentTemp = temp;
        digitalWrite(D7, LOW);
    }
}

// decide if we need to be on or off
// decide if we are manual override, or back to homeKit set temp
void checkTemp() {
    if (nextPID < millis()) {
        Input = (double)temp;
        Setpoint = (double)cfgData.targetTemp;
        myPID.Compute();

        if (cfgData.targetHeatingCoolingState == 0)
            Output = 0;

        nextPID = millis() + CHECK_DELAY;
    }
}

int pubToggle = 0;

void publishTelem() {
    if (nextReport < millis()) {
//        OutputTemp = MIN_TEMP + ((MAX_TEMP - MIN_TEMP) * Output/100);
        OutputTemp = COAL_LOW_F + ((COAL_HIGH_F - COAL_LOW_F) * Output/100);
        //OutputTemp = COAL_LOW_F + ((COAL_HIGH_F - COAL_LOW_F));
        if (Output > 0)
          currentHeatingCoolingState = 1;
        else
          currentHeatingCoolingState = 0;

        if (pubToggle == 0) {
            Particle.publish("Humidity", String(sensor.readHumidity()));
            Particle.publish("Temperature", String(temp));
            Particle.publish("StoveTargetTemp", String(Output));
            pubToggle = 1;
        } else {
            Particle.publish("TargetTemp", String(cfgData.targetTemp));
            Particle.publish("StoveTargetTemp", String(OutputTemp));
            Particle.publish("targetHeatingCoolingState", String(cfgData.targetHeatingCoolingState));
            Particle.publish("currentHeatingCoolingState", String(currentHeatingCoolingState));
            pubToggle = 0;
        }

        nextReport = millis() + PUB_DELAY;
    }
}

void loop() {
    if (sensorPresent == true) {
        // retrieve telementry
        getTelem();

        // check temp, adjust servo
        checkTemp();

        // publish telementry
        publishTelem();
    }
}
