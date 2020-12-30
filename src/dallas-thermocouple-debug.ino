// This #include statement was automatically added by the Particle IDE.
#include "DallasTemperature.h"

// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h>

DallasTemperature dallas(new OneWire(D3));

void setup() {
    Serial.begin(9600);
    dallas.begin();
    dallas.setResolution(9);
    Serial.println("Resolution: "+String(dallas.getResolution()));
}

float c1, f1, c2, f2;

void loop() {

    // get temps    
    digitalWrite(D7, HIGH);
    dallas.requestTemperatures();
    digitalWrite(D7, LOW);
//    delay(1000);
    c1 = dallas.getTempCByIndex(0);
//    c2 = dallas.getTempCByIndex(1);

    //digitalWrite(D7, HIGH);
    //dallas.requestTemperatures();
    //digitalWrite(D7, LOW);
    //delay(1000);
    f1 = dallas.getTempFByIndex(0);
//    f2 = dallas.getTempFByIndex(1);


    Serial.println("  Probe 1 Temperature = ");
    Serial.println(String(c1)+" C, "+String(f1)+" F\n");
//    Serial.println("  Probe 2 Temperature = ");
//    Serial.println(String(c2)+" C, "+String(f2)+" F\n");
    Particle.publish("F1", String(f1));
    delay(1000);
    Particle.publish("C1", String(c1));
    delay(1000);
//    Particle.publish("F2", String(f2));
//    delay(1000);
//    Particle.publish("C2", String(c2));
}