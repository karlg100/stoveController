// This #include statement was automatically added by the Particle IDE.
#include "DallasTemperature.h"

// This #include statement was automatically added by the Particle IDE.
#include <Debounce.h>

// This #include statement was automatically added by the Particle IDE.
#include <pid.h>

// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h>

// Thermocouple
//OneWire oneWire(D3);
//DallasTemperature dallas(&oneWire);
DallasTemperature dallas(new OneWire(D3));
DeviceAddress addr;

// anything less than this temp is invalid.  set to the lowest possible temp for the environemnt
#define MIN_TEMP 0
#define MAX_TEMP 512
#define TEMP_OUT_OF_RANGE 500

//  number of bad readings in a row to be considered out of range and go to safe mode
#define BAD_READING_LIMIT 10
int badReadings = 0;

#define BUTTON A4
#define BUTTON_OVERRIDE_DURATION 60000
#define BUTTON_OVERRIDE_MAX 600000
#define BUTTON_PRESS_TIME 1000
size_t buttonOverrideTimer = 0;
size_t buttonPressTimer = 0;
Debounce buttonDebouncer = Debounce();

// LED Pin
#define LED A0

// Speaker Pin
#define BUZZER A1

// Actuator control and params
Servo myservo;
#define SERVO_PIN D2
//define SERVO_MIN 1050
//define SERVO_MAX 2000

// min/max for the coal stove

// Viglant II
#define SERVO_MIN 1200
#define SERVO_MAX 1700
//define SERVO_MAX 1500

// Serdiac settins
//define SERVO_MIN 1220
//define SERVO_MAX 1420


// step size for each time we adust the servo
#define STEP_SIZE 20

// temp swing + or - the set temp to start adjusting the servo
#define TEMP_SWING 1

// sleep time between doing a step in or out when trying to reach out setpoint
#define STEP_SLEEP 60000

// sleep time between publishing telementry
#define TELEM_SLEEP 8000

// Servo Control
#define DEFAULT_SERVO SERVO_MAX
//define DEFAULT_SERVO (SERVO_MIN+SERVO_MAX)/2
#define DEFAULT_TARGETTEMP 200
#define DEFAULT_MODE 0

#define MODE_AUTO 0
#define MODE_MANUAL 1

// time tracking vars
size_t lastReport = 0;
size_t lastStep = 0;

// for particle cloud vars
double tempC = 0.0;
double tempF = 0.0;
double servo = 0.0;
char outstr[30];
double c, f;

// PID setup
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
double aggKp=4, aggKi=1, aggKd=0;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, PID::DIRECT);


// EEPROM state struct and handlers
// Struct for storing state in RAM and EEPROM
struct CCState {
    int magicNumber = 0xC10001; // this should change when struct changes

    // Servo Control
    int servoState;

    // Servo Control
    int mode;

    // Target temp to maintain output at
    int targetTemp;

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
    if ( cfgData.servoState < SERVO_MIN || cfgData.servoState > SERVO_MAX )
        return false;
    if (cfgData.targetTemp < MIN_TEMP)
        return false;
    if (cfgData.targetTemp > MAX_TEMP)
        return false;
    if (cfgData.mode < MODE_AUTO || cfgData.mode > MODE_MANUAL)
        return false;

    // everything looks good!  We can use these values
    return true;
}

int initCfg() {
    cfgData.servoState = DEFAULT_SERVO;
    cfgData.targetTemp = DEFAULT_TARGETTEMP;
    cfgData.mode = DEFAULT_MODE;
    writeCfg();
    Particle.publish("EEPROM", "New state initialized and written");
}

void setup() {
    // init servo
    myservo.attach(SERVO_PIN);   // attach the servo on the D0 pin to the servo object

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

    myservo.writeMicroseconds(cfgData.servoState);    // test the servo by moving it to 25°
    servo = cfgData.servoState;
    lastStep = millis();
    lastReport = millis();
    delay(1000);
    
    // init sensor
    dallas.begin();
    dallas.setResolution(12);
    
    // Register a Particle variable here
    Particle.variable("C", &c, DOUBLE);
    Particle.variable("F", &f, DOUBLE);
    Particle.variable("Input", &Input, DOUBLE);
    Particle.variable("Output", &Output, DOUBLE);
    Particle.variable("cfgData.servoState", &cfgData.servoState, INT);

    //Mesh.subscribe("CoalSetPt", partileSetTargetTemp);
    Particle.subscribe("StoveTargetTemp", partileSetTargetTemp);

    Particle.function("stepOut", servoOut);
    Particle.function("stepIn", servoIn);
    Particle.function("setMode", setMode);
    Particle.function("targetTemp", setTargetTemp);

    pinMode(D7, OUTPUT);  // set D7 as an output so we can flash the onboard LED

    // Init telemetry
    getTelem();

    // PID setup
    //initialize the variables we're linked to
    Input = (double)f;
    Setpoint = (double)cfgData.targetTemp;

    myPID.SetOutputLimits(0, 100);
    //myPID.SetOutputLimits(SERVO_MIN, SERVO_MAX);

    myPID.SetTunings(aggKp, aggKi, aggKd);

    //turn the PID on
    myPID.SetMode(PID::AUTOMATIC);
    
    // setup button
    buttonDebouncer.attach(BUTTON, INPUT_PULLUP);
    buttonDebouncer.interval(20);
    
    // setup LED
    pinMode(LED, OUTPUT);  // set D7 as an output so we can flash the onboard LED
    digitalWrite(LED, LOW);

    // setup Speaker
    pinMode(BUZZER, OUTPUT);  // set D7 as an output so we can flash the onboard LED
    digitalWrite(BUZZER, LOW);
}

int servoOut(String stepValue) {
    if ( cfgData.servoState + stepValue.toInt() <= SERVO_MAX )
        cfgData.servoState += stepValue.toInt();
    return cfgData.servoState;
}

int servoIn(String stepValue) {
    if ( cfgData.servoState - stepValue.toInt() >= SERVO_MIN )
        cfgData.servoState -= stepValue.toInt();
    return cfgData.servoState;
}

int setMode(String mode) {
    if (mode.toInt() >= MODE_AUTO && mode.toInt() <= MODE_MANUAL)
        cfgData.mode = mode.toInt();
    writeCfg();
    return cfgData.mode;
}


int setTargetTemp(String tempValue) {
    cfgData.targetTemp = tempValue.toInt();
    //writeCfg();
    return cfgData.targetTemp;
}

void partileSetTargetTemp(const char *event, const char *data)
{
    setTargetTemp(data);
//    Particle.publish("newSetPt", String(cfgData.targetTemp));
}

void buttonPress() {
    if (buttonOverrideTimer < millis())
        buttonOverrideTimer = millis();
    if (buttonOverrideTimer-millis()+BUTTON_OVERRIDE_DURATION > BUTTON_OVERRIDE_MAX)
        return;
    buttonOverrideTimer += buttonOverrideTimer + BUTTON_OVERRIDE_DURATION;
}

// calculate new servo position
void checkTemp() {
    // Output is a range of 0-100%
    
    // inverse control - 100% is SERVO_MIN, 0% is SERVO_MAX
    //cfgData.servoState = SERVO_MIN + round((SERVO_MAX - SERVO_MIN) * ((100 - Output)/100));

    // direct control - 100% is SERVO_MAX, 0% is SERVO_MIN
    if (buttonOverrideTimer > millis())
        cfgData.servoState = SERVO_MAX;
    else if (cfgData.mode == MODE_AUTO)
        cfgData.servoState = SERVO_MIN + round((SERVO_MAX - SERVO_MIN) * (Output/100));

/*
    // check to see if we're still sleeping
    if (lastStep + STEP_SLEEP > millis())
        return;
      
    if (f < MIN_TEMP)
        return;
      
    // check for temp too low, if so, step in to open damper
    if (f < cfgData.targetTemp-TEMP_SWING && cfgData.servoState - STEP_SIZE >= SERVO_MIN ) {
        cfgData.servoState -= STEP_SIZE;
        Particle.publish("status", "temp too low, stepping in");
        lastStep = millis();
    }
    // cehck for temp too high, if so, step out to close door
    else if ( f > cfgData.targetTemp+TEMP_SWING && cfgData.servoState + STEP_SIZE <= SERVO_MAX ) {
        cfgData.servoState += STEP_SIZE;
        Particle.publish("status", "temp too high, stepping out");
        lastStep = millis();
    }
    */
}

// get telementry from probe and feed it to the PID
void getTelem() {
    if (lastReport + TELEM_SLEEP > millis())
        return;
    digitalWrite(D7, HIGH);
    dallas.requestTemperatures();
    digitalWrite(D7, LOW);
    delay(1000);
    c = dallas.getTempCByIndex(0);

    digitalWrite(D7, HIGH);
    dallas.requestTemperatures();
    digitalWrite(D7, LOW);
    delay(1000);
    float fTemp = dallas.getTempFByIndex(0);

   // PID routine
    if (fTemp <= 0) {
        badReadings += 1;
        Particle.publish("Error", String(c));
    }
    else
        badReadings = 0;

    if (badReadings > BAD_READING_LIMIT)
        f = TEMP_OUT_OF_RANGE;

    if (fTemp > 0)
        f = fTemp;

    Input = (double)f;

    Setpoint = (double)cfgData.targetTemp;

/*
    double gap = abs(Setpoint-Input); //distance away from setpoint

    if(gap<=1)
    {  //we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
        //we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggKp, aggKi, aggKd);
    }
*/

    // compute new PID value
    myPID.Compute();
}

void publishTelem() {
    // time to publish?
    if (lastReport + TELEM_SLEEP > millis())
        return;
    
    // publish device count
    //sprintf(outstr,"%d", dallas.getDeviceCount());
    //Particle.publish("DallasCount", outstr);

    // publish Ferenheight
    if (f > 0) {
        //sprintf(outstr,"%f", f);
        //Particle.publish("C", String(c));
        Particle.publish("F", String(f));
        //Mesh.publish("F", outstr);
    }

    // publish current temperature setpoint
    //sprintf(outstr,"%d", cfgData.targetTemp);
    Particle.publish("targetTemp", String(cfgData.targetTemp));

    //sprintf(outstr,"%f", Output);
    Particle.publish("PID", String(Output));

    // adjust servo
    sprintf(outstr,"%d / %d", cfgData.servoState, myservo.read());
    Particle.publish("servo", outstr);

    if (buttonOverrideTimer > millis()) {
        //sprintf(outstr,"%d", buttonOverrideTimer-millis());
        Particle.publish("Override", String(buttonOverrideTimer-millis()));
    } else if (buttonOverrideTimer > 0)
        buttonOverrideTimer = 0;

    lastReport = millis();
}

// adjust the servo to the calculated set point
void setServo() {
    myservo.writeMicroseconds(cfgData.servoState);
}

void loop() {
    // service Button
    buttonDebouncer.update();
    if (buttonDebouncer.read() == LOW) {
        if (buttonPressTimer < millis()) {
            buttonPressTimer = millis() + BUTTON_PRESS_TIME;
            buttonPress();
        }
    } else {
        buttonPressTimer = 0;
    }
    
    // retrieve telementry
    getTelem();

    // check temp, adjust servo
    checkTemp();

    // update the servo
    setServo();

    // publish telementry
    publishTelem();
}


