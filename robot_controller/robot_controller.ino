#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

int turn=NULL;
int dist=NULL;

// centered position of the arm
const prog_uint16_t Center[] PROGMEM ={4, 2048, 2048, 2048, 2048};


void setup(){
  pinMode(0,OUTPUT);
  Serial.begin(28800);

  delay(500);

  // go to the initial position
  MoveCenter();
  vertical();
}




void loop(){

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {

      case 'P': // send the position of the motors
        delay(200);
        send_position();
        break;

      case 'C': // move the robot to the center circle position
        delay(200);
        turn = Serial.read();
        delay(10);
        dist = Serial.read();
        correct_turn(turn);
        if (turn == 0){
          correct_dist(dist);
        }
        break;


      case 'M': // move the robots to the desired position
        delay(100);
        int * p = receive_next_pos();
        delay(100);
        move_to_position(p);
        break;
    }
  }
}

void vertical(){
  // move the head facing down (vertical)
  int pos2 = ax12GetRegister(2, 36, 2);
  int pos3 = ax12GetRegister(3, 36, 2);
  int pos4 = ax12GetRegister(4, 36, 2);
  int position_target = 1024 + pos2 - pos3;
  if(position_target >= 1024 && position_target <= 3076){
    int erreur= abs(position_target - pos4);
    if (erreur>10){
      SetPosition(4, position_target);
      delay(10);
    }
  }
}

void MoveCenter(){
  delay(100);
  bioloid.loadPose(Center);
  bioloid.readPose();
  delay(1000);
  bioloid.interpolateSetup(1000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
}

void send_position(){
  for (int i = 1;i<=4;i++){
    Serial.println(ax12GetRegister(i, 36, 2));
    delay(300);
  }
}

void correct_turn(int turn){
  if (turn==1){
    turn_left(30);
  }
  else if (turn==2){
    turn_right(30);
  }
  else if (turn==3){
    turn_little_left();
  }
  else if (turn==4){
    turn_little_right();
  }
}

void correct_dist(int dist){
  if (dist==1){
    go_forward(10);
  }
  else if (dist==2){
    go_backward(30);
  }
}

void turn_left(int increment){
  int pos1 = ax12GetRegister(1, 36, 2);
  int position_target=pos1+increment;
  if(position_target<4096){
    SetPosition(1,position_target);
  }
}
void turn_little_left(){
  int pos1 = ax12GetRegister(1, 36, 2);
  int position_target=pos1-40;
  SetPosition(1,position_target);
  delay(50);
  position_target=pos1+35;
  SetPosition(1,position_target);
}

void turn_right(int increment){
  int pos1 = ax12GetRegister(1, 36, 2);
  int position_target=pos1-increment;
  if(position_target>0){
    SetPosition(1,position_target);
  }
}

void turn_little_right(){
  int pos1 = ax12GetRegister(1, 36, 2);
  int position_target=pos1+40;
  SetPosition(1,position_target);
  delay(50);
  position_target=pos1-35;
  SetPosition(1,position_target);
}

void go_forward(int increment){
  int pos2 = ax12GetRegister(2, 36, 2);
  int position_target=pos2+increment;
  if(position_target<3072){
    while(pos2 <= position_target){
      pos2 = pos2+1;
      SetPosition(2, pos2);
      SetPosition(3, pos2);
      delay(10);
    }
  }
}

void go_backward(int increment){
  int pos2 = ax12GetRegister(2, 36, 2);
  int position_target=pos2-increment;
  if(position_target>1024){
    while(pos2 >= position_target){
      pos2 = pos2-1;
      SetPosition(2, pos2);
      SetPosition(3, pos2);
      delay(10);
    }
  }
}

int * receive_next_pos(){
  static int pos[5];
  // going to receive ";xxxx;xxx;xxxx;xxxx;"
  delay(500);
  char caract='z';
  while(caract != ';') {
    caract = Serial.read();
  }
  String t="";
  for (int i = 0; i<5; i++){
    delay(100);
    while (caract!=';'){
      t += caract;
      caract = Serial.read();
      delay(100);
    }
    pos[i] = t.toInt();
    t="";
    if (i!=4){
      caract = Serial.read();
    }
    delay(100);
  }
  return pos;
}

void move_to_position(int * pos){

  for (int i = 0 ; i < 4 ; i++){
    SetPosition(i+1, *(pos+i));
    delay(100);
  }
}
