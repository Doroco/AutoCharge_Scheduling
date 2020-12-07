// C Robot
// 한칸 당 시간 1.6s
#include <DynamixelMotor.h>
#include <Encoder.h>
#define R_en_A 2
#define R_en_B 3
#define L_en_A 18
#define L_en_B 19
#define A_PWM 9 //analogWrite 이용
#define B_PWM 8 //analogWrite 이용
#define A1_dir 7 //INT4
#define A2_dir 6 //INT3
#define B1_dir 5 //INT2
#define B2_dir 4 //INT1
#define LED 53

typedef enum {
  GO_F = 1, // forward 앞으로 가는 것
  GO_R, // right 오른쪽으로 가는 것
  GO_B, // backward 뒤쪽으로 가는 것
  GO_L, // left 왼쪽으로 가는 것
  STOP,
  PAUSE,
  ROTATE_180, // 180도 회전 시키기
  LIFT_UP,
  LIFT_DOWN
}CAR_CONTROL_DIR;

//Dynamixel 관련
int id1 = 1, id2 = 2, speed = 512;
const long unsigned int baudrate = 1000000;
HardwareDynamixelInterface interface(Serial);
DynamixelMotor motor1(interface, id1);
DynamixelMotor motor2(interface, id2); //다이나믹셀 용

//속도관련 PID
int L_new_en_pos = 0, L_old_en_pos, R_new_en_pos , R_old_en_pos, L_en_cnt = 0, R_en_cnt = 0, L_A_en = LOW, L_B_en = LOW, R_A_en = LOW, R_B_en = LOW; // 엔코더
int PWM_A, PWM_B;                    
int v_time_cur, v_time_pre=0;
float v_dt , PPR = 6;                          //PID
double R_v, L_v, v_goal = 1.4, R_v_gain, L_v_gain;
long double R_v_error_pre , R_v_error_cur , R_v_error_sum, R_v_error_dif, L_v_error_pre , L_v_error_cur , L_v_error_sum, L_v_error_dif;               //PID      

//adc   
int IR[6] = {A0, A1, A2, A3, A4, A5}; // 6개 IR센서값 저장    
int f_adc[6] = {0, }; // 정규화 데이터 저장
int total_adc, flag, R_cur_cnt, L_cur_cnt;
volatile int adc[6] = {0, }; //adc 값 저장
double weight_adc[6] = {-15, -5, -0.4, 0.4, 5, 15};
volatile double i_sum;
double i_time_pre, i_time_cur,i_v, i_dt, i_error_cur, i_error_pre, i_gain, adc_goal = 0;
double Kp = 5, Kd = 3.5;

int cnt = 0, crs = 0, zero_cnt = 0;
int route[600] = {1, 8, 7, 1, 4, 1, 4, 1, 1, 1, 4, 1, 4, 9, 7, 2, 1, 4, 1, 1, 1, 1, 4, 1, 1, 4, 1, 8, 7, 2, 2, 9, 7, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 8, 7, 1, 4, 1, 4, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 1, 4, 1, 9, 7, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 4, STOP, 7, 2, 1, 4, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 4, 1, 8, 7, 1, 4, 4, 1, 9, 7, 2, 1, 2, 1, 1, 2, 4, 1, 8, 7, 2, 1, 4, 1, 1, 1, 1, 4, 1, 4, 1, 9, 7, 2, 2, 1, 1, 2, 2, 8, 7, 1, 1, 9, 7, 2, 1, 4, 1, 1, 1, 1, 4, 1, 1, 4, 1, 8, 7, 1, 4, 1, 4, 1, 9, 7, 2, 1, 1, 1, 1, 4, 4, 1, 1, 1, 1, 1, 1, 1, 4, 4, STOP, STOP};
int routeSize = 0;

Encoder knobLeft(19, 18);
Encoder knobRight(3, 2);

void setup() {
  pinMode(L_en_A, INPUT);
  pinMode(L_en_B, INPUT);
  pinMode(R_en_A, INPUT);
  pinMode(R_en_B, INPUT);
  pinMode(A1_dir, OUTPUT);
  pinMode(A2_dir, OUTPUT);
  pinMode(B1_dir, OUTPUT);
  pinMode(B2_dir, OUTPUT);
  pinMode(LED, OUTPUT);
  for(int i = 0; i < 6; i++){
    pinMode(IR[i], INPUT);
  }
  interface.begin(baudrate);
  motor1.jointMode(204, 820);
  motor1.speed(speed);
  motor2.jointMode(204, 820);
  motor2.speed(speed);
  motor1.enableTorque();
  motor2.enableTorque();
  D_init();
  delay(5000);
}

void loop() {
  makeRoute(route, 300); //성공!!!!
  delay(100);
}

/*
//엔코더 카운트 함수
void L_doEncoA(){
  L_A_en = digitalRead(L_en_A);
  L_B_en = digitalRead(L_en_B);
  if(L_A_en == HIGH){
    if(L_B_en == LOW){
      L_en_cnt += 1;
    }
    else{
      L_en_cnt -= 1;
    }
  }
  else{
    if(L_B_en == HIGH){
     L_en_cnt += 1; 
    }
    else{
      L_en_cnt -= 1;
    }
  }
}
void L_doEncoB(){
  L_A_en = digitalRead(L_en_A);
  L_B_en = digitalRead(L_en_B);
  if(L_B_en == HIGH){
    if(L_A_en == HIGH){
      L_en_cnt += 1;
    }
    else{
      L_en_cnt -= 1;
    }
  }
  else{
    if(L_A_en == LOW){
     L_en_cnt += 1; 
    }
    else{
      L_en_cnt -= 1;
    }
  }
}

//엔코더 카운트 함수
void R_doEncoA(){
  R_A_en = digitalRead(R_en_A);
  R_B_en = digitalRead(R_en_B);
  if(R_A_en == HIGH){
    if(R_B_en == LOW){
      R_en_cnt += 1;
    }
    else{
      R_en_cnt -= 1;
    }
  }
  else{
    if(R_B_en == HIGH){
     R_en_cnt += 1; 
    }
    else{
      R_en_cnt -= 1;
    }
  }
}
void R_doEncoB(){
  R_A_en = digitalRead(R_en_A);
  R_B_en = digitalRead(R_en_B);
  if(R_B_en == HIGH){
    if(R_A_en == HIGH){
      R_en_cnt += 1;
    }
    else{
      R_en_cnt -= 1;
    }
  }
  else{
    if(R_A_en == LOW){
     R_en_cnt += 1; 
    }
    else{
      R_en_cnt -= 1;
    }
  }
}*/
//속도제어 PID제어기
void v_PID(double Kp, double Ki, double Kd){
  v_dt = 1;
  i_v = Linetrace(adc_goal, 5, 4.5);
  L_en_cnt = knobLeft.read();
  R_en_cnt = knobRight.read();
  L_new_en_pos = L_en_cnt;
  L_v = (((L_new_en_pos - L_old_en_pos)/PPR)*TWO_PI)/v_dt;
  if(L_v < 0){  L_v = -L_v; }
  L_old_en_pos = L_new_en_pos;

  R_new_en_pos = R_en_cnt;
  R_v = (((R_new_en_pos - R_old_en_pos)/PPR)*TWO_PI)/v_dt;
  if(R_v < 0){  R_v = -R_v; }
  R_old_en_pos = R_new_en_pos;
  
  R_v_error_cur = (v_goal - (i_v/ 150)) - R_v; //속도 제어 변화시 수식 계산해서 바꿀 것
  R_v_error_sum += R_v_error_cur;
  R_v_error_dif = R_v_error_cur - R_v_error_pre;
  
  R_v_gain = (Kp * R_v_error_cur) + (Ki * R_v_error_sum * v_dt) + ((Kd * R_v_error_dif)/v_dt);
  R_v_error_pre = R_v_error_cur;
  R_v_gain = constrain(R_v_gain, 0, 255);

  L_v_error_cur = (v_goal + (i_v / 150)) - L_v;
  L_v_error_sum += L_v_error_cur;
  L_v_error_dif = L_v_error_cur - L_v_error_pre;
  
  L_v_gain = (Kp * L_v_error_cur) + (Ki * L_v_error_sum * v_dt) + ((Kd * L_v_error_dif)/v_dt);
  L_v_error_pre = L_v_error_cur;
  L_v_gain = constrain(L_v_gain, 0, 255);
  
  
  analogWrite(A_PWM, R_v_gain);
  analogWrite(B_PWM, L_v_gain);
}
// 회전 속도 (좌회전, 우회전, 180도 회전)전용 PID
void turn_v_PID(double Kp, double Ki, double Kd){
  v_dt = 1;

  L_en_cnt = knobLeft.read();
  R_en_cnt = knobRight.read();
  L_new_en_pos = L_en_cnt;
  L_v = (((L_new_en_pos - L_old_en_pos)/PPR)*TWO_PI)/v_dt;
  if(L_v < 0){  L_v = -L_v; }
  L_old_en_pos = L_new_en_pos;

  R_new_en_pos = R_en_cnt;
  R_v = (((R_new_en_pos - R_old_en_pos)/PPR)*TWO_PI)/v_dt;
  if(R_v < 0){  R_v = -R_v; }
  R_old_en_pos = R_new_en_pos;
  
  R_v_error_cur = 0.2 - R_v; //속도 제어 변화시 수식 계산해서 바꿀 것
  R_v_error_sum += R_v_error_cur;
  R_v_error_dif = R_v_error_cur - R_v_error_pre;
  
  R_v_gain = (Kp * R_v_error_cur) + (Ki * R_v_error_sum * v_dt) + ((Kd * R_v_error_dif)/v_dt);
  R_v_error_pre = R_v_error_cur;
  R_v_gain = constrain(R_v_gain, 0, 255);

  L_v_error_cur = 0.2 - L_v;
  L_v_error_sum += L_v_error_cur;
  L_v_error_dif = L_v_error_cur - L_v_error_pre;
  
  L_v_gain = (Kp * L_v_error_cur) + (Ki * L_v_error_sum * v_dt) + ((Kd * L_v_error_dif)/v_dt);
  L_v_error_pre = L_v_error_cur;
  L_v_gain = constrain(L_v_gain, 0, 255);
  
  
  analogWrite(A_PWM, R_v_gain);
  analogWrite(B_PWM, L_v_gain);
}
//다이나믹셀 초기화 함수
void dynamixel_init(){
  interface.begin(baudrate);
  motor1.jointMode(204, 820);
  motor1.speed(speed);
  motor2.jointMode(204, 820);
  motor2.speed(speed);
  motor1.enableTorque();
  motor2.enableTorque();
}

//위치제어 PD제어기
double Linetrace(double adc_goal, double Kp, double Kd){
  i_sum = 0;
  i_dt = 1;
  for(int j = 0; j < 6; j++){
    f_adc[j] = (int)(analogRead(IR[j]) / 900);// 0 또는 1로만 출력
    //total_adc += f_adc[j];
  }
   
  for(int k = 0; k < 6; k++){
    i_sum += f_adc[k] * weight_adc[k];
  }
  i_error_cur = adc_goal - i_sum;
  i_gain = (Kp * i_error_cur) + (Kd * (i_error_cur - i_error_pre));
  i_error_pre = i_error_cur;
  //Serial.println(i_gain);
  return i_gain;
}
void D_init(){// 초기 및 물건 내릴때
  motor1.goalPosition(512);
  motor2.goalPosition(512); // 위치
  digitalWrite(LED, LOW);
}
void D_up(){ //물건 올릴때
  motor1.goalPosition(412);
  motor2.goalPosition(612); // 위치
  digitalWrite(LED, HIGH);
}
//십자인식 함수
int getFlag(){
  total_adc = 0;
  for(int k = 0; k < 6; k++){
    f_adc[k] = (int)(analogRead(IR[k]) / 900);// 0 또는 1로만 출력
    total_adc += f_adc[k];
    if(total_adc >= 4){
      flag = 1;
      zero_cnt = 0;
    }
    else{
      flag = 0;
      zero_cnt ++;
    }
  }
  return flag;
}

// 이동관련 함수들
void Stop(){ //정지
  analogWrite(A_PWM, 0);
  analogWrite(B_PWM, 0);
}
void go(int n){ //전진
  switch(n){
    case 0:
      digitalWrite(A1_dir, HIGH);
      digitalWrite(A2_dir, LOW); // 
      digitalWrite(B1_dir, HIGH);
      digitalWrite(B2_dir, LOW); //
      v_PID(5, 3, 5);
      
      break;
    
    case 1:
      analogWrite(A_PWM, 0);
      analogWrite(B_PWM, 0);
      break; 
        
    default :
      analogWrite(A_PWM, 0);
      analogWrite(B_PWM, 0);
      break;
  }
}
void t_right(){ // 우회전
  analogWrite(A_PWM, 0);
  analogWrite(B_PWM, 0);
  delay(100);

  //L_en_cnt = knobLeft.read();
  //R_en_cnt = knobRight.read();
  R_cur_cnt = knobRight.read();
  L_cur_cnt = knobLeft.read();
 // Serial.println(R_en_cnt);
  while(abs(knobRight.read() - R_cur_cnt) < 330){
    digitalWrite(A1_dir, HIGH);
    digitalWrite(A2_dir, LOW); // 방향 다시 설정할 것
    digitalWrite(B1_dir, HIGH);
    digitalWrite(B2_dir, LOW);
    turn_v_PID(5, 3, 5);
  }
  R_cur_cnt = 0;
     
  R_cur_cnt = knobRight.read();
  //Serial.println(R_en_cnt);
  while(abs(knobRight.read() - R_cur_cnt) <= 310){ // 우회전 
    digitalWrite(A1_dir, LOW);
    digitalWrite(A2_dir, HIGH); 
    digitalWrite(B1_dir, HIGH);
    digitalWrite(B2_dir, LOW);
    turn_v_PID(5, 3, 5);
  }

  delay(50);
  //L_en_cnt = L_cur_cnt - 730;
}

void go_back(int n){ //후진
  switch(n){
    case 0:
      digitalWrite(A1_dir, LOW);
      digitalWrite(A2_dir, HIGH); // 
      digitalWrite(B1_dir, LOW);
      digitalWrite(B2_dir, HIGH); //
      v_PID(5, 3, 5);
      
      break;
    
    case 1:
      analogWrite(A_PWM, 0);
      analogWrite(B_PWM, 0);
      break; 
        
    default :
      analogWrite(A_PWM, 0);
      analogWrite(B_PWM, 0);
      break;
  }
}

void t_left(){ //좌회전
  analogWrite(A_PWM, 0);
  analogWrite(B_PWM, 0);
  delay(100);
  
  //L_en_cnt = knobLeft.read();
  //R_en_cnt = knobRight.read();
  R_cur_cnt = knobRight.read();
  L_cur_cnt = knobLeft.read();
 // Serial.println(L_en_cnt);
  while(abs(knobLeft.read() - L_cur_cnt) <  310){
    digitalWrite(A1_dir, HIGH);
    digitalWrite(A2_dir, LOW); 
    digitalWrite(B1_dir, HIGH);
    digitalWrite(B2_dir, LOW);
    turn_v_PID(5, 3, 5);
  }
  L_cur_cnt = 0;
      
  L_cur_cnt = knobLeft.read();
  //Serial.println(L_en_cnt);
  while(abs(knobLeft.read() - L_cur_cnt) < 320){ 
    digitalWrite(A1_dir, HIGH);
    digitalWrite(A2_dir, LOW); 
    digitalWrite(B1_dir, LOW);
    digitalWrite(B2_dir, HIGH);
    turn_v_PID(5, 3, 5);
  }
       
  //Serial.println(R_en_cnt);
  //R_en_cnt = R_cur_cnt + 710;
}
void t_180(){ //180도 회전
  analogWrite(A_PWM, 0);
  analogWrite(B_PWM, 0);
  delay(100);
    
  //L_en_cnt = knobLeft.read();
  //R_en_cnt = knobRight.read();
  R_cur_cnt = knobRight.read();
  L_cur_cnt = knobLeft.read();
  //Serial.println(R_en_cnt);
  while(abs(knobLeft.read() - L_cur_cnt) < 130){
    digitalWrite(A1_dir, HIGH);
    digitalWrite(A2_dir, LOW); 
    digitalWrite(B1_dir, HIGH);
    digitalWrite(B2_dir, LOW);
    turn_v_PID(5, 3, 5);
  }
  L_cur_cnt = 0;
  
  L_cur_cnt = knobLeft.read();
 // Serial.println(R_en_cnt);
  while(abs(knobLeft.read() - L_cur_cnt) < 710){
    digitalWrite(A1_dir, HIGH);
    digitalWrite(A2_dir, LOW); 
    digitalWrite(B1_dir, LOW);
    digitalWrite(B2_dir, HIGH);
    turn_v_PID(5, 3, 5);
  }
  L_cur_cnt = 0;

  delay(100);

  L_cur_cnt = knobLeft.read();
  while(abs(knobLeft.read() - L_cur_cnt) < 130){
    digitalWrite(A1_dir, HIGH);
    digitalWrite(A2_dir, LOW); 
    digitalWrite(B1_dir, HIGH);
    digitalWrite(B2_dir, LOW);
    turn_v_PID(5, 3, 5);
  }
  L_cur_cnt = 0;
  
  delay(50);
  //R_en_cnt = R_cur_cnt + 1080;
}

//입력받은 경로 구현하는 함수
void makeRoute(int *m_route, int m_routeSize){  
  int route = 0;
  for(int i=0; i<m_routeSize; i++){
    int route = 0;
    route = *(m_route+i);
    
    switch(route){
      case GO_F:
        while(getFlag() == 1){
          LED_state(cnt);
          go(0);
        }
        while(getFlag() != 1){
          LED_state(cnt);
          go(0);
        }
       // Serial.println(cnt);
        delay(100);
        break;
        
      case GO_R:
        LED_state(cnt); 
        t_right();
        delay(100);
        
        while(getFlag() != 1){
          go(0);
        }
      //  Serial.println(cnt);
        delay(100);
        break;
      
      case GO_B:
        LED_state(cnt); 
        while(getFlag() != 1){
          go_back(0);
        }
        //Serial.println(cnt);
        delay(100);
        break;
        
      case GO_L:
        LED_state(cnt);
        t_left();
        delay(100);
        while(getFlag() != 1){
          go(0);
        }
        //Serial.println(cnt);
        delay(100); 
        break;

      case STOP:
        LED_state(cnt); 
        Stop();
        delay(62400);
        break;
        
      case PAUSE: 
        LED_state(cnt);
        Stop();
        delay(1000);
        break;
     
      case ROTATE_180: 
        LED_state(cnt);
        t_180();
        delay(100);
        while(getFlag() != 1){
          go(0);
        }
        //Serial.println(cnt);
        delay(100);
        break;
        
      case LIFT_UP:
        cnt = 1;
        LED_state(cnt);
        analogWrite(A_PWM, 0);
        analogWrite(B_PWM, 0);
        motor1.goalPosition(412);
        motor2.goalPosition(612); // 위치
        delay(1000);
        break;
        
      case LIFT_DOWN:
        cnt = 0;
        LED_state(cnt);
        analogWrite(A_PWM, 0);
        analogWrite(B_PWM, 0);
        motor1.goalPosition(512);
        motor2.goalPosition(512); // 위치
        delay(1000);
        break;
    }
  }  
}

int sizecnt(int *a){
  return sizeof(a)/sizeof(a[0]);
}
void LED_state(int flag){
  if(flag == 1)
    digitalWrite(LED, HIGH);
  else
    digitalWrite(LED, LOW);
}
