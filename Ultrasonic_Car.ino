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
  #define ST1 4
  #define ST2 10
  #define ST3 12
  #define ST4 13
  #define STEPS_PER_REVOLUTION 2048
  //There are 2048 steps, only want 180 degrees
  //from 0 to 1024 in 32 degree increments
  #define A_STEP 32
  
  
  int distance; //in cm
  int index = 0; //index for the array
  bool goBack = false;
  int index2 = 0; //index for the servo
  int tooClose = 30; //in cm
  int upTime = 20; //in milliseconds
  int currentMap[32];
  //int savedMap[32];
  int tolerance = 10; //in cm
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
  //see if the distance is too close
  if(distance < tooClose){
      stopCar();
      //rotate 360 degrees, taking measurements, then turn to the farthest point
      //turning clockwise
      if(goBack)
        for(index = 0; index < 32 ; index += 1){
          currentMap[index] = getDistance();
          myStepper.step(index*A_STEP);
          delay(200);
          stopStepper();
        } else{ //turning counterclockwise
          for(index = 32; index > 0 ; index -= 1){
          currentMap[index] = getDistance();
          myStepper.step(index*A_STEP);
          delay(200);
          stopStepper();
        }
      
      //reset the stepper to default position

      delay(500);
      //Then, turn the car
      if(goBack){ //car turns clockwise
        turnRight();
        while(getDistance() < getArrayMax(currentMap)){
          delay(1); //just turns the car slowly to match furthest distance
        }
        stopCar(); //brief pause
      } else{
        turnLeft();
        while(getDistance() < getArrayMax(currentMap)){
          delay(1); //just turns the car slowly to match furthest distance
        }
        stopCar(); //brief pause
      }
      //after spinning 360 degrees, change direction of stepper
      goBack = !goBack;
        //then go
        moveForward();
      }
    } else{
      moveForward();
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
  digitalWrite(ENA, 127);  
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight(void){
  digitalWrite(ENA, 127);  
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

void stopStepper(){
  digitalWrite(ST1, LOW);
  digitalWrite(ST2,LOW);
  digitalWrite(ST3,LOW);
  digitalWrite(ST4,LOW);
}
