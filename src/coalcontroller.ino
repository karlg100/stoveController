
#include "DallasTemperature.h"
#include <Debounce.h>
#include <pid.h>
#include <OneWire.h>

// Thermocouple
DallasTemperature dallas(new OneWire(D3));
DeviceAddress addr;

// anything less than this temp is invalid.  set to the lowest possible temp for the environemnt
#define MIN_TEMP 0

// Max temp.  We should chirp when we get this high
#define MAX_TEMP 500

// Max temp.  Start freaking out if we go above this
#define MAX_TEMP_PANIC 600

#define TEMP_OUT_OF_RANGE 500

// temperture when we should start flashing for coal
#define TEMP_NEED_COAL 200

//  number of bad readings in a row to be considered out of range and go to safe mode
#define BAD_READING_LIMIT 10
int badReadings = 0;

// Button
#define BUTTON A4
#define BUTTON_OVERRIDE_DURATION 60000
#define BUTTON_OVERRIDE_MAX 600000
#define BUTTON_PRESS_TIME 1000
size_t buttonOverrideTimer = 0;
size_t buttonPressTimer = 0;
Debounce buttonDebouncer = Debounce();

// LED
#define LED A0

// Speaker
#define BUZZER A1
#define ALARM_SLEEP 60000;
#define ALARM_SLEEP_PANIC 30000;

#define ALARM_NONE 0
#define ALARM_WARN 1
#define ALARM_PANIC 2

bool alarmMute = false;
int alarmStage = 0;
size_t nextAlarm = 0;


// Actuator control and params
Servo myservo;
#define SERVO_PIN D2

// Absolute min and max range of the servo
//define SERVO_MIN 1050
//define SERVO_MAX 2000

// min/max for the coal stove

// Viglant II - manual
//define SERVO_MIN 1200
//define SERVO_MAX 1700

// Viglant II - auto (with coal)
#define SERVO_MIN 1400
#define SERVO_MAX 1700


// Serdiac settins
//define SERVO_MIN 1220
//define SERVO_MAX 1420

// Timing
// sleep time between publishing telementry
#define TELEM_SLEEP 8000

// Defaults for configuation
#define DEFAULT_SERVO SERVO_MAX
//define DEFAULT_SERVO (SERVO_MIN+SERVO_MAX)/2
#define DEFAULT_TARGETTEMP 200
#define DEFAULT_MODE 0

// Modes
// Manual - do nothing, servo is set manually using remote fuctions
// Auto - adjust the servo/output of the stove as set by the remote thermostat (it does all the thinking)
// Auto_PID - set a target temp for the stove, and try to keep it there
#define MODE_MANUAL 0
#define MODE_AUTO 1
#define MODE_AUTO_PID 2

#define MODE_MIN MODE_MANUAL
#define MODE_MAX MODE_AUTO_PID

// time tracking vars
size_t lastReport = 0;

// for particle cloud vars
double tempC = 0.0;
double tempF = 0.0;
char outstr[30];
double c, f;
double modeOutput;      // Output to Servo, depending on mode
double targetTemp = DEFAULT_TARGETTEMP;

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
    int magicNumber = 0xC10003; // this should change when struct changes

    // Servo Control
    int servoState;

    // Servo Control
    int mode;

    // checkum value
    int checkSum = 0;
} cfgData;


//  EEPROM handing functions
#define CFG_BASE 0
bool readCfg()
{
  //uint8_t* _data     = &cfgData;
  uint8_t *_data = (uint8_t*) &cfgData;
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
    if (cfgData.mode < MODE_MIN || cfgData.mode > MODE_MAX)
        return false;

    // everything looks good!  We can use these values
    return true;
}

int initCfg() {
    cfgData.servoState = DEFAULT_SERVO;
    cfgData.mode = DEFAULT_MODE;
    writeCfg();
    Particle.publish("EEPROM", "New state initialized and written");
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

void LEDFlash(int duration) {
    digitalWrite(LED, HIGH);
    delay(duration);
    digitalWrite(LED, LOW);
}

void LEDhandler() {
    if (alarmStage == ALARM_WARN && !alarmMute) {
        LEDFlash(20);
        return;
    }
    
    if (alarmStage == ALARM_PANIC && !alarmMute) {
        LEDFlash(50);    delay(100);
        LEDFlash(50);    delay(100);
        LEDFlash(50);    delay(300);
        LEDFlash(100);    delay(100);
        LEDFlash(100);    delay(100);
        LEDFlash(100);    delay(300);
        LEDFlash(50);    delay(100);
        LEDFlash(50);    delay(100);
        LEDFlash(50);    delay(100);
        return;
    }
    
    if (f < TEMP_NEED_COAL) {
        digitalWrite(LED, LOW);
        delay(200);
        analogWrite(LED, 50);
    } else {
        digitalWrite(LED, LOW);
    }
}

Timer LEDTimer(5000, LEDhandler);

void setup() {
    // init servo
    myservo.attach(SERVO_PIN);   // attach the servo on the D0 pin to the servo object

    setupCfg();

    myservo.writeMicroseconds(cfgData.servoState);    // test the servo by moving it to 25°
    lastReport = millis();

    // init sensor
    dallas.begin();
    dallas.setResolution(10);  // Hack - we're really reading 12 bit, but we're turning down the coversion time with the MAX31850
    
    // Register a Particle variable here
    Particle.variable("C", &c, DOUBLE);
    Particle.variable("F", &f, DOUBLE);
    Particle.variable("Input", &Input, DOUBLE);
    Particle.variable("Output", &Output, DOUBLE);
    Particle.variable("Mode", &cfgData.mode, INT);
    Particle.variable("servoState", &cfgData.servoState, INT);
    Particle.variable("targetTemp", &targetTemp, DOUBLE);

    Particle.subscribe("StoveTargetTemp", particleSetTargetTemp);

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
    Setpoint = targetTemp;

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
    LEDTimer.start();

    // setup Speaker
    pinMode(BUZZER, OUTPUT);  // set D7 as an output so we can flash the onboard LED
    digitalWrite(BUZZER, LOW);
}

int servoOut(String stepValue) {
    if ( cfgData.servoState + stepValue.toInt() <= SERVO_MAX )
        cfgData.servoState += stepValue.toInt();
    writeCfg();
    return cfgData.servoState;
}

int servoIn(String stepValue) {
    if ( cfgData.servoState - stepValue.toInt() >= SERVO_MIN )
        cfgData.servoState -= stepValue.toInt();
    writeCfg();
    return cfgData.servoState;
}

int setMode(String mode) {
    if (mode.toInt() >= MODE_MIN && mode.toInt() <= MODE_MAX)
        cfgData.mode = mode.toInt();
    writeCfg();
    return cfgData.mode;
}

int setTargetTemp(String tempValue) {
    targetTemp = (double)tempValue.toFloat();
    return targetTemp;
}

void particleSetTargetTemp(const char *event, const char *data)
{
    setTargetTemp(data);
}

void buttonPress() {
    if (alarmStage > ALARM_NONE && !alarmMute) {
        alarmMute = true;
        return;
    }
    
    buttonOverrideTimer += buttonOverrideTimer + BUTTON_OVERRIDE_DURATION;
}

// calculate new servo position
void checkTemp() {
    if (lastReport + TELEM_SLEEP > millis())
        return;
    
    // Output is a range of 0-100%
        // inverse control - 100% is SERVO_MIN, 0% is SERVO_MAX
    //cfgData.servoState = SERVO_MIN + round((SERVO_MAX - SERVO_MIN) * ((100 - Output)/100));

    // Decide witch output to use
    if (cfgData.mode == MODE_AUTO_PID)      // Second PID Control
        modeOutput = double(Output);
    else if (cfgData.mode == MODE_AUTO)     // Direct Control
        modeOutput = targetTemp;

    // direct control - 100% is SERVO_MAX, 0% is SERVO_MIN
    if (buttonOverrideTimer > millis())
        cfgData.servoState = SERVO_MAX;
    else if (cfgData.mode == MODE_AUTO || cfgData.mode == MODE_AUTO_PID)     // Direct Control
        cfgData.servoState = SERVO_MIN + round((SERVO_MAX - SERVO_MIN) * (modeOutput/100));
}

// get telementry from probe and feed it to the PID
void getTelem() {
    if (lastReport + TELEM_SLEEP > millis())
        return;
    digitalWrite(D7, HIGH);
    dallas.requestTemperatures();
    digitalWrite(D7, LOW);
//    delay(1000);
    c = dallas.getTempCByIndex(0);

//    digitalWrite(D7, HIGH);
//    dallas.requestTemperatures();
//    digitalWrite(D7, LOW);
//    delay(1000);
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

    Setpoint = targetTemp;

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
    
    checkTemp();
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
        //Particle.publish("C", String(c));
        Particle.publish("F", String(f));
    }

    // publish current temperature setpoint
    if (cfgData.mode == MODE_AUTO_PID)
        Particle.publish("targetTemp", String(targetTemp));

    Particle.publish("PID", String(modeOutput));

//    Particle.publish("Debug", String(Output)+", "+String(targetTemp)+", "+String(modeOutput));

    // adjust servo
    sprintf(outstr,"%d / %d", cfgData.servoState, myservo.read());
    Particle.publish("servo", outstr);

    if (buttonOverrideTimer > millis()) {
        Particle.publish("Override", String(buttonOverrideTimer-millis()));
    } else if (buttonOverrideTimer > 0)
        buttonOverrideTimer = 0;

    lastReport = millis();
}

// adjust the servo to the calculated set point
void setServo() {
    myservo.writeMicroseconds(cfgData.servoState);
}

void soundAlarm1() {
    Particle.publish("Alarm", "Alarm1");
    digitalWrite(BUZZER, HIGH);
    delay(10);
    digitalWrite(BUZZER, LOW);
    delay(500);
    digitalWrite(BUZZER, HIGH);
    delay(30);
    digitalWrite(BUZZER, LOW);
//    delay(100);
//    digitalWrite(BUZZER, HIGH);
//    delay(10);
//    digitalWrite(BUZZER, LOW);
}

void soundAlarm2() {
    Particle.publish("Alarm", "Alarm2");
    for (int x = 0; x <= 255; x++) {
        analogWrite(BUZZER, x);
        delay(5);
    }    
    analogWrite(BUZZER, 0);
}


void checkAlarms() {
    // If our temp dropped below threadhold, downgrade and reset
    if (f < MAX_TEMP && alarmStage == ALARM_WARN) {
        alarmStage = ALARM_NONE;
        alarmMute = false;
        nextAlarm = 0;
    }
    else if (f < MAX_TEMP_PANIC && alarmStage == ALARM_PANIC) {
        alarmStage = ALARM_WARN;
    }

    // Check if our temp is above threadhold, and upgrade
    if (f > MAX_TEMP_PANIC*1.01 && alarmStage < ALARM_PANIC) {
        alarmStage = ALARM_PANIC;
        nextAlarm = 0;
        alarmMute = false;
    } 
    else if (f > MAX_TEMP*1.01 && alarmStage < ALARM_WARN) {
        alarmStage = ALARM_WARN;
        alarmMute = false;
        nextAlarm = 0;
    }

    // sound the alarm!
    if (alarmStage > ALARM_NONE && nextAlarm < millis() && !alarmMute) {
        if (alarmStage == ALARM_WARN) {
            if (!alarmMute) soundAlarm1();
            nextAlarm = millis() + ALARM_SLEEP;
        }       
        
        else if (alarmStage == ALARM_PANIC) {
            if (!alarmMute) soundAlarm2();
            nextAlarm = millis() + ALARM_SLEEP_PANIC;
        }
    }
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
    //checkTemp();

    // update the servo
    setServo();

    // publish telementry
    publishTelem();
    
    // sound the alarms
    checkAlarms();
}
