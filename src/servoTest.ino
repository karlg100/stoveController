
Servo myservo;

int servoState = 0;

void setup() {
    myservo.attach(D2);   // attach the servo on the D0 pin to the servo object
    Particle.publish("status", "init servo to 0");
    myservo.write(0);    // test the servo by moving it to 25°
    delay(1000);
    pinMode(D7, OUTPUT);  // set D7 as an output so we can flash the onboard LED
}

char outstr[30];

void loop() {
    
    servoState = random(950)+1050;
    myservo.writeMicroseconds(servoState);
    sprintf(outstr,"%d", servoState);
    Particle.publish("servo", outstr);
    delay(5000);

/*

    //myservo.write(random(100));    // test the servo by moving it to 25°
    Particle.publish("status", "servo to 1000");
//    myservo.write(0);    // test the servo by moving it to 25°
    myservo.writeMicroseconds(1050);
    delay(5000);

    Particle.publish("status", "servo to 1500");
    myservo.writeMicroseconds(1500);
    delay(5000);

    Particle.publish("status", "servo to 2000");
//    myservo.write(90);    // test the servo by moving it to 25°
    myservo.writeMicroseconds(2000);
    delay(5000);

    Particle.publish("status", "servo to 1500 (rev)");
    myservo.writeMicroseconds(1500);
    delay(5000);
    
    */
    
    
}