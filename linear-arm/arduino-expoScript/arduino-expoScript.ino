#include <Servo.h>
#define NUM_SERVOS 5
#define NUM_AXES 5
Servo servos[NUM_SERVOS];
int axesPins[NUM_SERVOS] = {5, 6, 9, 10, 11};
const unsigned int MAX_INPUT = 10;

/*
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo grip;
*/

void setup() {
  /*
  base.attach(5);
  shoulder.attach(6);
  elbow.attach(9);
  wrist.attach(10);
  grip.attach(11);
  */
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(axesPins[i]);
  }
  Serial.begin(115200);
}

void process_data (const char * data, int dataLen)
  {
    if (dataLen>=2){
      int intData = atoi(data);
      int axis = (int) (intData/pow(10, dataLen-1));
      if (axis >= NUM_AXES) {
        // invalid
        return;
      }
      int angle = intData % ((int) ceil(pow(10, dataLen-1)));
      servos[axis].write(angle);
    }
  }

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

  }
void loop() {
  while (Serial.available () > 0)
    processIncomingByte (Serial.read ());
}
