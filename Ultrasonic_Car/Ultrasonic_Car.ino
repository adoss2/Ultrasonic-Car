#include <Servo.h>
#include <Stepper.h>
  //sensor macros
  #define TRIG_PIN (A5)
  #define ECHO_PIN (A4)
  #define SERVO_PIN (3)
  //Wheel macros
  #define ENA (5)
  #define IN1 (7)
  #define IN2 8
  #define IN3 9
  #define IN4 11
  #define ENB 6
  //Stepper macros
  /*
   * Curtis is testing with the stepper motor, the code will go here when he integrates it
  */
  //#define STEPS 256
  
  //Gyro macros
  /*
   * Curtist is testing with gyro, code will go here when he integrates it.
  */
  
  int distance; //in cm
  int index = 0; //index for the array
  bool goBack = false;
  int index2 = 0; //index for the servo
  int tooClose = 30; //in cm
  int upTime = 20; //in milliseconds
  int currentMap[32];
  int savedMap[32];
  int tolerance = 10; //in cm
  //Stepper myStepper(STEPS, );
  Servo myServo;
  
  //graph stuff
  
void setup() {
  //Sensor setup
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  Serial.begin(9600);
  //Wheel setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  //Stepper Setup

  //Servo Setup
  myServo.attach(SERVO_PIN);
  //Gyro Setup

  //Graph Setup
  
}

void loop() {
  //This is to detect from the sensor
  distance = getDistance();
  if(distance != -1){
    //Make new array if it's full
    if(index >32){
      //move elements back one
      for(index = 0; index < 31; index++){
        currentMap[index] = currentMap[index+1];
      }
      index = 32;
    }
    Serial.print("Distance in CM: ");
    Serial.println(distance);
    Serial.print(" ");
    //Write to Array
    currentMap[index] = distance;
    index++;
    
    //Turn Servo
    myServo.write((int) index*(180/32));
    delay(75);
    index2++;
    if(index2 > 32){
      index2 =0;
      myServo.write(1);
      delay(1800);
    }
  }
  
  //delay before doing it again
  delay(upTime);
  
  //drive the car
  if(distance < tooClose){
      stopCar();
      /*
      Here we will put the code to accept an array of values, then decide which way to turn based on the array.
      As it stands right now, it simply does what the original car does, turns until it will not hit a wall.
      */
      //Planned code: take most recent saved array from ultrasound, find the index of the max, match that to an angle to turn to, then match that angle with the gyroscope
      //Will look like:
      /* angle = getArrayMaxIndex(currentMap);
       *  (code to get the angle from index)
       *  switch to left or right to get to angle
      */
      distance = getDistance();
        while(!withinRange(getArrayMax(currentMap), distance, tolerance)){
        distance = getDistance();
        turnRight(); //This will be replaced
        delayMicroseconds(10);
        stopCar();
      }
    } else{
      moveForward();
    }
  
  
  //this moves the servo 
  //We're switching to a stepper so that'll be used later
}

int getDistance(){
  int distance;
  //Sets to low
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  //pulse from senstor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  //it takes 58 microseconds per cm for the signal to come back
  distance = (int) pulseIn(ECHO_PIN, HIGH)/58;
  if(distance < 0){
      distance = -1;
      Serial.println("Out of range!!");
    }
  return distance; //returns in cm
}

void moveForward(void){
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar(void){
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveBackward(void){
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft(void){
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight(void){
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

int getArrayMax(int myArray[]){
  //returns index of maximum of an array
  int index = 0;
  int current;
  int high = myArray[0];
  int result = 0;
  while(myArray[index] != NULL){
    current = myArray[index];
    if(current > high){
      high = current;
      result = index;
    }
  }
  return high;
}

int getArrayMaxIndex(int myArray[]){
  //returns index of maximum of an array
  int index = 0;
  int current;
  int high = myArray[0];
  int result = 0;
  while(myArray[index] != NULL){
    current = myArray[index];
    if(current > high){
      high = current;
      result = index;
    }
  }
  return result;
}

bool withinRange(int firstVal, int secondVal, int precision){
  bool ans;
  if(abs(firstVal-secondVal) < precision){
    ans = true;
  } else{
    ans = false;
  }
  return ans;
}
