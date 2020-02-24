

//Skeleton/Pseudo code for final robot




//Variables
//----------------------------

//State of the robot i.e. current objective
int state = 0; // 0 = line following, 1 = sweeping ,2 = ....

//Line following
bool sensor_l;      // 1 = white, 0 = black
bool sensor_r;
bool speed_l;       // 1 = faster, 0 = slower
bool speed_r;


//Parameters
//----------------------------



//Functions
//----------------------------


//Update motor speed
void update_motor_speed() {
}

//Slightly rotate the robot to a direction
void nudge(bool dir){
    //1 = nudge right, 0 = nudge left
}

//Rotate direction (approx)
void rotate(float angle){
    //
}





//Actual code in robot
//----------------------------
void setup()
{
  Serial.begin(9600);
  //Establish connection with laptop
  //Other important initialising
}

void loop()
{
  switch(state){
    
    //State 0: Line following
    case 0:
      //Line following code here
      state = 1
    break;
    
    
    //State 1: Sweeping to find robot
    case 1:
      //Rotate 90 degrees left
      //Rotate 180 degrees right
      //Determine robot location
      //Send data to laptop
      state = 2
    break;
    
    
    //State 2: Pathfinding to target
    case 2:
      //Rotate towards target
      //Work with CV
      if target_correctype {
        state = 3;
      } else {
        //Pathfind to next target  
      }
    break;
    
    
    //State 3: Picking up target
    case 3:
      //Pick up robot
      state = 4
    break;
    
    
    //State 4: Returning to tunnel entrance
    case 4:
      //Pathfind back to tunnel entrance
      state = 5
    break;
    
    
    //State 5: Line following to dumping ground
    case 5:
      //Line following code keep right
      state = 6
    break;
    
    
    //State 6; Dump target
    case 6:
      //Dump target
      state = 7
      
    break;
    
    
    //State 7: Turn around and follow line to tunnel
    case 7:
      state = 1
    break;
}










