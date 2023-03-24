#include "communication.h"
#include "mbed.h"
#include <cstdio>
#include "robot.h"



BufferedSerial serial(USBTX, USBRX, 115200);
Communicator *communicator = new Communicator();

FileHandle *mbed::mbed_override_console(int fd) { return &serial; }

Robot *robot = new Robot();


// main() runs in its own thread in the OS
int main() {
  
  robot->move_forward(2.0);
  










  // states and actual state for state machine
   /* const int State_forward_both_wheels= 0;
    const int State_forward_front_wheels= 0;
    const int State_forward_back_wheels= 0;    
    const int State_hochklappen_front= 0;    
    const int State_hochklappen_back= 0; 
    const int State_runterklappen_front= 0; 
    const int State_runterklappen_back= 0;
    const int State_hochklappen_both= 0; 
    const int State_runterklappen_both= 0; 


    const int ROBOT_STATE_FORWARD  = 1;
    const int ROBOT_STATE_BACKWARD = 2;
    const int ROBOT_STATE_SLEEP    = 3;
*/
  while (true) {
    ThisThread::sleep_for(1s);
  }
}
