#include <Servo.h>
#include <Stepper.h>
  //sensor macros
  #define TRIG_PIN (A5)
  #define ECHO_PIN (A4)
  //#define SERVO_PIN (3)
  //Wheel macros
  #define ENA (5)
  #define IN1 (7)
  #define IN2 8
  #define IN3 9
  #define IN4 11
  #define ENB 6
  //Stepper macros - note, 8 through 11 are already used by wheel
  #define ST1 1
  #define ST2 2
  #define ST3 3
  #define ST4 4
  #define STEPS_PER_REVOLUTION 64
  
  
  int distance; //in cm
  int index = 0; //index for the array
  bool goBack = false;
  int index2 = 0; //index for the servo
  int tooClose = 30; //in cm
  int upTime = 20; //in milliseconds
  int currentMap[32];
  //int savedMap[32];
  int tolerance = 10; //in cm
  //Stepper myStepper(STEPS, );
  Servo myServo;
  Stepper myStepper(STEPS_PER_REVOLUTION, ST1, ST2, ST3, ST4);
  //graph stuff
  
void setup() {
  //Sensor setup
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  //Serial setup
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
  myStepper.setSpeed(120);
  
  //Fill in array with zeros
  for(index = 0; index < 32 ; index++){
    currentMap[index] = 0;
  }
}

void loop() {
  //This is to detect from the sensor
  distance = getDistance();
  if(distance != -1){
    //if array is full, move all elements back one and add new value to the end
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
  }
    //Turn Stepper
    myStepper.step(1);
    delay(100);
    index2++; //index is the current step that the stepper is on.
    if(index2 >= 32){
      myStepper.step(-32);
      delay(300);
      
    }
    
  //delay before doing it again
  delay(upTime);

  if(distance < tooClose){
      stopCar();
      
     // Here we have put the code to accept an array of values, then turn until it sees the furthest distance from where it currently is from the array
      
      //initialize value
      distance = getDistance();
      //Make the ultrasound face forward, which it does at point 16
      myStepper.step(16-index2);
      delay(300);
      //while loop that runs until it gets the best distance
        while(distance < getArrayMax(currentMap)){
        distance = getDistance();
        turnRight();
        delay(500);
        stopCar();
        delay(1000);
      }
      
    } else{
      //moveForward();
    }
    
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
  while((myArray[index] != NULL) || (myArray[index] != 0)){
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
