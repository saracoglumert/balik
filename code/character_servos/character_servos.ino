#include <Servo.h>

Servo table;
Servo wristPitch;
Servo wristRoll;
Servo lid;
Servo eyePan;
Servo eyeTilt;

String input = "T000WP000WR000L000EP000ET000";

int tableInput = 0;
int wristPitchInput = 0;
int wristRollInput = 0;
int lidInput = 0;
int eyePanInput = 0;
int eyeTiltInput = 0;

int tablePin = 3;
int wristPitchPin = 5;
int wristRollPin= 6;
int lidPin = 11;
int eyePanPin = 9;
int eyeTiltPin = 10;


int tablemin = 0;
int tablemax = 180;
int wristPitchmin = 75;
int wristPitchmax = 140;
int wristRollmin = 0;
int wristRollmax = 90;
int lidmin = 0;
int lidmax = 180;
int eyePanmin = 0;
int eyePanmax = 30;
int eyeTiltmin = 0;
int eyeTiltmax = 70;

void setup()

{
  
  table.attach(tablePin);
  wristPitch.attach(wristPitchPin);
  wristRoll.attach(wristRollPin);
  lid.attach(lidPin);
  eyePan.attach(eyePanPin);
  eyeTilt.attach(eyeTiltPin);

  table.write(10); 

 
  Serial.begin(9600);  
}

void loop() {
  if (Serial.available() > 0) { //T000WP000WR000L000EP000ET000
    
    input = Serial.readString();
    Serial.read();

    tableInput = (input.substring(1,4)).toInt();
    wristPitchInput = (input.substring(6,9)).toInt();
    wristRollInput = (input.substring(11,14)).toInt();
    lidInput = (input.substring(15,18)).toInt();
    eyePanInput = (input.substring(20,23)).toInt();
    eyeTiltInput = (input.substring(25,28)).toInt();

    
    if (tableInput < tablemin)
      tableInput = tablemin;
    else if (tableInput > tablemax)
      tableInput = tablemax;

    if (wristPitchInput < wristPitchmin)
      wristPitchInput = wristPitchmin;
    else if (wristPitchInput > wristPitchmax)
      wristPitchInput = wristPitchmax;
      
    if (wristRollInput < wristRollmin)
      wristRollInput = wristRollmin;
    else if (wristRollInput > wristRollmax)
      wristRollInput = wristRollmax;

    if (lidInput < lidmin)
      lidInput = lidmin;
    else if (lidInput > lidmax)
      lidInput = lidmax;

    if (eyePanInput < eyePanmin)
      eyePanInput = eyePanmin;
    else if (eyePanInput > eyePanmax)
      eyePanInput = eyePanmax;
      
    if (eyeTiltInput < eyeTiltmin)
      eyeTiltInput = eyeTiltmin;
    else if (eyeTiltInput > eyeTiltmax)
      eyeTiltInput = eyeTiltmax;

    Serial.println(tableInput);
    Serial.println(wristPitchInput);
    Serial.println(wristRollInput);
    Serial.println(lidInput);
    Serial.println(eyePanInput); 
    Serial.println(eyeTiltInput);

    table.write(tableInput);
    wristPitch.write(wristPitchInput);
    wristRoll.write(wristRollInput);
    lid.write(lidInput);
    eyePan.write(eyePanInput);
    eyeTilt.write(eyeTiltInput);


  }
  delay (10);
}
