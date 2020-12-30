// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_Si7021.h>

// This #include statement was automatically added by the Particle IDE.
#include <pid.h>

// This #include statement was automatically added by the Particle IDE.
//include <ArduinoJson.h>

// This #include statement was automatically added by the Particle IDE.
//include <WebServer.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_Si7021.h>

// This #include statement was automatically added by the Particle IDE.
//include <Grove_4Digit_Display.h>

// 2020-11-15 - I2C bus on Mesh gateway died. D0 seems to be dead.  Moved to second i2c bus

Adafruit_Si7021 sensor = Adafruit_Si7021(&Wire1);

#define MIN_TEMP 30
#define MAX_TEMP 100
#define DEFAULT_TARGETTEMP 80
#define DEFAULT_TARGETMODE 0

#define COAL_HIGH_F 460
#define COAL_LOW_F 200

// ** PID Setup
//Define Variables we'll be connecting to
double Setpoint, Input, Output, OutputTemp;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, PID::DIRECT);

// LCD display
#define CLK D2//pins definitions for TM1637 and can be changed to other ports
#define DIO D3
//TM1637 tm1637(CLK,DIO);

// time tracking vars
size_t nextReport = 0;
size_t nextPID = 0;
size_t overrideTimer = 0;

// manual override
#define ROTARY_ANGLE_SENSOR A0
#define ROTARY_DEBOUNCE 5
int lastManualTemp = 0;
int lastRotary = 0;
#define OVERRIDE_TIME 60000

// working vars
double temp = MIN_TEMP;
int manualTemp;
int targetTemp;
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

bool sensorPresent = false;

void setup() {

    if (!sensor.begin())
        Particle.publish("Status", "Did not find Si7021 sensor!");
    else
        sensorPresent = true;
    
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

    Particle.function("setMode", setTargetHeatingCoolingState);
    Particle.function("setTargetTemp", setTargetTemp);
    Particle.variable("getTargetTemp", &targetTemp, INT);
    Particle.variable("getCurrentTemp", &currentTemp, DOUBLE);
    Particle.variable("getCurrentState", &currentHeatingCoolingState, INT);

    // Angle sensor
    //pinMode(ROTARY_ANGLE_SENSOR, INPUT);
    //lastManualTemp = MIN_TEMP + int((MAX_TEMP + 1 - MIN_TEMP) * analogRead(ROTARY_ANGLE_SENSOR)/4095);
    overrideTimer = 0;

    // init the display
//    tm1637.init();
//    tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
//    tm1637.point(POINT_OFF);
    
    pinMode(D7, OUTPUT);
    
    // boost BLE transmit power
    //BLE.setTxPower(8);
    
    
    // Init telemetry
    getTelem();

    manualTemp = cfgData.targetTemp;
    lastManualTemp = cfgData.targetTemp;
    targetTemp = cfgData.targetTemp;
    currentHeatingCoolingState = cfgData.targetHeatingCoolingState;
    
    // PID setup
    //initialize the variables we're linked to
    Input = (double)temp;
    Setpoint = (double)cfgData.targetTemp;
    myPID.SetMode(PID::AUTOMATIC);

    myPID.SetOutputLimits(0, 100);
    //myPID.SetOutputLimits(COAL_LOW_F, COAL_HIGH_F);
//    myPID.SetTunings(aggKp, aggKi, aggKd);

    // Calibrate/preseed PID output
    myPID.SetSampleTime(0);
    for (int x = 0; x < 100000; ++x) {
        myPID.Compute();
    }
    myPID.SetSampleTime(CHECK_DELAY);
}

void getTelem() {
/*
    // get rotary pos
    if (analogRead(ROTARY_ANGLE_SENSOR) <= lastRotary + ROTARY_DEBOUNCE && analogRead(ROTARY_ANGLE_SENSOR) >= lastRotary - ROTARY_DEBOUNCE)
        manualTemp = MIN_TEMP + int((MAX_TEMP + 1 - MIN_TEMP) * (4095-lastRotary)/4095);
    else {
        lastRotary = analogRead(ROTARY_ANGLE_SENSOR);
        manualTemp = MIN_TEMP + int((MAX_TEMP + 1 - MIN_TEMP) * (4095-analogRead(ROTARY_ANGLE_SENSOR))/4095);
    }
*/
    // get temp
    if (nextPID < millis()) {
        temp = sensor.readTemperature() * 1.8 + 32.0;
        currentTemp = temp;
    }
}

// decide if we need to be on or off
// decide if we are manual override, or back to homeKit set temp
void checkTemp() {
    if (manualTemp != lastManualTemp) {
        lastManualTemp = manualTemp;
        overrideTimer = millis() + OVERRIDE_TIME;
    }

    if (nextPID < millis()) {
        if (overrideTimer > millis()) {
            targetTemp = manualTemp;
//        tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
        } else {
            targetTemp = cfgData.targetTemp;
//        tm1637.set(BRIGHT_DARKEST);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
        }

        Input = (double)temp;
        Setpoint = (double)targetTemp;
        myPID.Compute();

        if (currentHeatingCoolingState == 0)
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
        if (Output == 0)
          int currentHeatingCoolingState = 0;
        else
          int currentHeatingCoolingState = 1;

        if (pubToggle == 0) {
            Particle.publish("Humidity", String(sensor.readHumidity()));
            Particle.publish("Temperature", String(temp));
            pubToggle = 1;
        } else {
            Particle.publish("TargetTemp", String(targetTemp));
            Particle.publish("StoveTargetTemp", String(OutputTemp));
            Particle.publish("targetHeatingCoolingState", String(cfgData.targetHeatingCoolingState));
            Particle.publish("currentHeatingCoolingState", String(currentHeatingCoolingState));
            pubToggle = 0;
        }

        //Particle.publish("PID_raw", String(Output));
        //Mesh.publish("CoalSetPt", String(OutputTemp));
        nextReport = millis() + PUB_DELAY;
    }

/*    
    tm1637.display(2,String(String(targetTemp).charAt(0)).toInt());
    tm1637.display(3,String(String(targetTemp).charAt(1)).toInt());
    tm1637.display(0,String(temp.charAt(0)).toInt());
    tm1637.display(1,String(temp.charAt(1)).toInt());
*/
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
