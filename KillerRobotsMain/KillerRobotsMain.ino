/*
 * Filename:
 * Purpose:
 * 
 * Contributors:
 * Date:
 */

//Libraries



//Pins


//Objects


//Variables
int mode = 0;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  //Declare variables

  //Switch between different modes
  switch(mode) {
    case 1:   //Standby
      //scan and detect
      break;
    case 2:   //Chase or Attack
      //PID
      //Alert function
      break;
    case 3:   //Only Alert
      //Alert only, arrived at obstacle or cannot reach
      break;
    case 4:   //Withdraw
      //After 5s or so, withdraw from target
      break;
    default:  //Idle (all operations suspended)
      //All operations off and safe
    break;
  }

}

void persue {
  
}
