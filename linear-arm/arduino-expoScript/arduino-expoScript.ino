#include <Servo.h>
#define NUM_SERVOS 3
#define NUM_AXES 4
#define ZERO_LIMIT 4
#define MAX_LIMIT 2
#define DC_IN1 9
#define DC_IN2 10
#define DC_EN 11
#define ENC_A 12 // DATA
#define ENC_B 13 // CLK
// Assume all non servo axes are the lowest indexed
const int NON_SERVOS = NUM_AXES - NUM_SERVOS;
Servo servos[NUM_SERVOS];
int axesPins[NUM_SERVOS] = {3, 5, 6};
int constraints[NUM_SERVOS][2] = {{40, 145}, {30, 180}, {70, 155}};
// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 10;
const unsigned int railSpeed = 200;
int encPosition = 0;
int previousStateEncA;
int previousStateEncB;
const int railLength = 1478; // assuming railSpeed 200
int targetRailPosition = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  pinMode(DC_EN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ZERO_LIMIT, INPUT_PULLUP);
  pinMode(MAX_LIMIT, INPUT_PULLUP);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(axesPins[i]);
  }
  Serial.begin(115200);
  // calibrate
//  digitalWrite(DC_IN1, HIGH);
//  digitalWrite(DC_IN2, LOW);
//  analogWrite(DC_EN, 160);
  
  // Read the initial state of encoder
   previousStateEncA = digitalRead(ENC_A);
   previousStateEncB = digitalRead(ENC_B);
}

// here to process incoming serial data after a terminator received
void process_data (const char * data, int dataLen)
  {
    if (dataLen>=2){
      int intData = atoi(data);
      int axis = (int) (intData/pow(10, dataLen-1));
      if (axis >= NUM_AXES) {
        // invalid
        return;
      }
      if (axis >= NON_SERVOS) {
        int angle = intData % ((int) ceil(pow(10, dataLen-1)));
        servos[axis-NON_SERVOS].write(map(constrain(angle, 0, 180), 0, 180, constraints[axis-NON_SERVOS][0], constraints[axis-NON_SERVOS][1]));
      }
      else {
        targetRailPosition = intData % ((int) ceil(pow(10, dataLen-1)));
      }
    }
    
  }  // end of process_data

void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
    {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line, input_pos);

      // reset buffer for next time
      input_pos = 0;  
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch

  } // end of processIncomingByte  

void loop() {
  // if serial data available, process it
  while (Serial.available () > 0)
    processIncomingByte (Serial.read ());
  
  updateEncoder();
    
  if (digitalRead(ZERO_LIMIT) == LOW) {
//    Serial.println("ZERO");
    targetRailPosition=15;
    encPosition = 0;
  }
  if (digitalRead(MAX_LIMIT) == LOW) {
//    Serial.println("MAX");
    targetRailPosition=railLength-15;
    encPosition = railLength;
  }
  // Update linear motor stuff
  int deadband = 10;
  if (encPosition - targetRailPosition > deadband) {
    // move towards motor
    digitalWrite(DC_IN1, HIGH);
    digitalWrite(DC_IN2, LOW);
    analogWrite(DC_EN, railSpeed);
  }else if (targetRailPosition - encPosition > deadband) {
    // move away from motor
    digitalWrite(DC_IN1, LOW);
    digitalWrite(DC_IN2, HIGH);
    analogWrite(DC_EN, railSpeed);
  }else {
    digitalWrite(DC_EN, LOW);
  }
  
 }  // end of loop

// kinda bad practice but just changes globals
 void updateEncoder() {
  int currentStateEncB = digitalRead(ENC_B);
   
   // If the previous and the current state of the inputCLK are different then a pulse has occurred
   if (currentStateEncB != previousStateEncB){
       
     // If the inputDT state is different than the inputCLK state then
     // the encoder is rotating counterclockwise
     if (digitalRead(ENC_A) != currentStateEncB) {
       encPosition--;
       
     } else {
       // Encoder is rotating clockwise
       encPosition++;
       
     }
   }
   previousStateEncB = currentStateEncB;
//   Serial.println(encPosition);
 }
