

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
//Rotate direction (approx)
void rotate(float angle){
    //
}

//Follow Line
void follow_line(bool keepRight, int count){
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT); //Side sensor as input
  while 1{
    //Sensor readings
    sensor_s_prev = sensor_s;
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol) ? 1 : 0;
    //Determine motor speeds via boolean logic
    speed_L = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    speed_R = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    //Sensor_S
    if (sensor_s && !sensor_s_prev){
      count--;
      if (count <= 0){
        motor_L(0);
        motor_R(0);
        return
      }
    }
    //Update speeds
    motor_L(speed_L ? v : 0);
    motor_R(speed_R ? v : 0);
    delay(50);
  }
}




//Actual code in robot
//----------------------------
void setup()
{
  Serial.begin(9600);
  //Establish connection with laptop
  //Other important initialising
}

void loop(){
  switch(state){
    
    //State 0: Line following
    case 0:
      follow_line(1,1);  //keep right and stop on 1st instance sensor_s = 1
      state = 1
    break;
    
    //State 1: Pathfinding to target
    case 1:
      //Rotate towards target
      //Work with CV
      if target_correctype {
        state = 2;
      } else {
        //Pathfind to next target  
      }
    break;
    
    
    //State 2: Picking up target
    case 2:
      //Pick up robot
      state = 3
    break;
}










