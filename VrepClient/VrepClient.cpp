#include<Windows.h>
#include <iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <thread>
#include <vector>
#include <fstream> 
#include<queue>
#include<iomanip>
#include<fstream>
#include<cmath>
#include"tracemap.h"
#include"Linetracer.h"
#include <chrono>

extern "C" {
#include "extApi.h"
};

using namespace std;
#define PI 3.14
#define RED_VALUE 11
#define GREEN_VALUE 12
#define BLUE_VALUE 13


Map map; // 전체 맵에 대한 것 선언
static Car lineTracer[CARNUM]; //차 네대 생성

//전방참조 방지

//------------------------------------------AGV 의 회전상태를 나타내기 위한 허수 ------------------------------------------------------------------------
class My_COMPLEX
{
public:
	int real, imag;
	My_COMPLEX(int R = 0.0, int I = 0.0) {
		real = R; imag = I;
		//printf("Init R=%f, I=%f\n",R,I);
	}
	My_COMPLEX const& operator=(My_COMPLEX const& other) {
		real = other.real;
		imag = other.imag;
		return *this;
	};
	bool operator==(My_COMPLEX const& other) {
		if (real == other.real && imag == other.imag) {
			return true;
		}
		else
			return false;
	};
	int ComplexToMode() {
		if (this->real == 1 && this->imag == 0)
			return 1;
		//x + 1
		if (this->real == 0 && this->imag == 1)
			return 2;
		//y - 1
		if (this->real == -1 && this->imag == 0)
			return 3;
		//x - 1
		if (this->real == 0 && this->imag == -1)
			return 4;
		// y + 1
	}
	friend My_COMPLEX operator+(My_COMPLEX&, My_COMPLEX&);
	friend My_COMPLEX operator-(My_COMPLEX&, My_COMPLEX&);
	friend My_COMPLEX operator*(My_COMPLEX&, My_COMPLEX&);
};

My_COMPLEX operator+(My_COMPLEX& x, My_COMPLEX& y) {
	My_COMPLEX t(x.real, x.imag);
	t.real += y.real; t.imag += y.imag;
	return t;
};
My_COMPLEX operator-(My_COMPLEX& pt1, My_COMPLEX& pt2) {
	My_COMPLEX t(pt1.real, pt1.imag);
	t.real -= pt2.real; t.imag -= pt2.imag;
	return t;
};
My_COMPLEX operator*(My_COMPLEX& pt1, My_COMPLEX& pt2) {
	My_COMPLEX t;
	t.real = (pt1.real * pt2.real) - (pt1.imag * pt2.imag);
	t.imag = (pt1.real * pt2.imag) + (pt1.imag * pt2.real);
	return t;
};

//------------------------------------------ ------------------------------------------------------------------------


typedef struct positionState {
	//where are you?
	int x_pos = 0;
	int y_pos = 0;
} Where;

//Color_value of VisionSensor
typedef struct color {
	double red;
	double green;
	double blue;
} RGB;
//======================================나중에 정리할 것
enum {
	NOMAL_SENSING = 0,
	LEFT,
	RIGHT,
	REVERSE,
	CONVEY,
	RELEASE,
	BACK,
	LINKED_BACK,
	LINKED_REVERSE,
	STOP34,
	STOP39,
	STOP47,
	STOPP,
	INITIALIZE
};

void printmove(int i) {
	switch (i) {
	case NOMAL_SENSING:
		printf("%s\t", "NOMAL_SENSING");
		break;
	case LEFT:
		printf("%s\t", "LEFT");
		break;
	case RIGHT:
		printf("%s\t", "RIGHT");
		break;
	case REVERSE:
		printf("%s\t", "REVERSE");
		break;
	case CONVEY:
		printf("%s\t", "CONVEY");
		break;
	case RELEASE:
		printf("%s\t", "RELEASE");
		break;
	case STOPP:
		printf("%s\t", "STOP");
		break;
	}
}

string printmoves(int i) {
	string move;
	switch (i) {
	case NOMAL_SENSING:
		move = "NOMAL_SENSING";
		break;
	case LEFT:
		move = "LEFT";
		break;
	case RIGHT:
		move = "RIGHT";
		break;
	case REVERSE:
		move = "REVERSE";
		break;
	case CONVEY:
		move = "CONVEY";
		break;
	case RELEASE:
		move = "RELEASE";
		break;
	case STOPP:
		move = "STOP";
		break;
	}

	return move;
}

//typedef enum {
//	GO_F = 0, // forward 앞으로 가는 것
//	GO_R, // right 오른쪽으로 가는 것
//	GO_B, // backward 뒤쪽으로 가는 것
//	GO_L, // left 왼쪽으로 가는 것
//	CHARGE_OUT, //충전하고 나갈 때,
//	ROTATE_180, // 180도 회전하고 나가는 것까지
//	LIFT_UP,
//	LIFT_DOWN,
//	STOP // 멈춤
//}CAR_CONTROL_DIR;

int CartoAGVEnum(int carE) {
	int AGVE = 0;

	switch (carE) {
	case GO_F:
		AGVE = NOMAL_SENSING;
		break;
	case GO_R:
		AGVE = RIGHT;
		break;
	case GO_B:
		AGVE = BACK;
		break;
	case GO_L:
		AGVE = LEFT;
		break;
	case CHARGE_OUT:
		AGVE = REVERSE;
		break;
	case ROTATE_180:
		AGVE = REVERSE;
		break;
	case LIFT_UP:
		AGVE = CONVEY;
		break;
	case LIFT_DOWN:
		AGVE = RELEASE;
		break;
	case STOP:
		AGVE = STOPP;
		break;
	default:
		break;
	}

	return AGVE;
}

//AGV handle Class(차량 운반로봇 클래스)
class AGV {
private:

	int flag = 1;
	Where car_pos;
	//AGV의 진행정보
	My_COMPLEX state;
	int State_mode = 4;

	My_COMPLEX right_s = My_COMPLEX(0, -1);
	My_COMPLEX left_s = My_COMPLEX(0, 1);
	My_COMPLEX turn_s = My_COMPLEX(-1, 0);

	//예상상태 of AGV
	Where expected_car_pos;

	//RXR 비젼센서
	simxUChar LL_vision_state;
	simxFloat* LL_auxValues;
	simxInt* LL_auxValuesCount;

	simxUChar L_vision_state;
	simxFloat* L_auxValues;
	simxInt* L_auxValuesCount;

	simxUChar M_vision_state;
	simxFloat* M_auxValues;
	simxInt* M_auxValuesCount;

	simxUChar MR_vision_state;
	simxFloat* MR_auxValues;
	simxInt* MR_auxValuesCount;

	simxUChar R_vision_state;
	simxFloat* R_auxValues;
	simxInt* R_auxValuesCount;

	simxUChar RR_vision_state;
	simxFloat* RR_auxValues;
	simxInt* RR_auxValuesCount;

	//비젼센서센서값
	RGB LL_detectedColor;
	RGB L_detectedColor;
	RGB M_detectedColor;
	RGB MR_detectedColor;
	RGB R_detectedColor;
	RGB RR_detectedColor;

	ofstream out;
	ifstream in;

	int CarHandle = 0;
	int clientID = 0;
	int cnt = 0;
	int cnt_zero = 0;


	//Sensors
	int leftSensor_L = 0;
	int leftSensor = 0;
	int middleSensor = 0;
	int middleSensor_R = 0;
	int rightSensor = 0;
	int rightSensor_R = 0;

	//Joint Handle
	int leftMotorHandle = 0;
	int rightMotorHandle = 0;

	//컨베이어 조인트
	int C_L_joint = 0;
	int C_L_joint2 = 0;
	int C_R_joint = 0;
	int C_R_joint2 = 0;

	//Car_Speed
	float Speed_Nomal = 2.5f;

	//Error_Code
	int result = 0;

	//Thread Object
	int Thread_obj_num = 0;

	//베터리량 -----> 윈도우폼으로부터 스트리밍으로 베터리 퍼센테이지를 받아옴
	float BatteryPercent;

	//Woking State
	bool working = false;

	//FileStream Enable
	int IPCMODE = 0;

	//Signal
	bool SensorChecking = true;
	double check = 9.0;
	double check2 = 9.0;
	double check_p = 9.0;
	int Mode = INITIALIZE;
	bool Checking_Enable = true;

	//File Stream
	string s;
	string batteryStatePath;
	string Bbuffer;

	//Path Finder
	queue <int> path;

	//Encoder
	float L_angle = 0.0;
	float R_angle = 0.0;

	int r_count = 0;
	int l_count = 0;

	double L_angle_org = 0.0;
	double R_angle_org = 0.0;

	bool R_Enable = true;
	bool L_Enable = true;

	double R_temp = 0;
	double L_temp = 0;

	int L_cnt = 1;
	int R_cnt = 1;

	//script저장된곳
	int scriptHandle_make = 0;
	int scriptHandle_delete = 0;

	//들고 있는 물건정보
	simxFloat obj = 0;

	//impormation for script
	simxFloat  sending[2] = { 0, };


	//AGV의 허수 상태는 차의 앞범퍼를 기준으로 한다.------------------------------------------------------------------------------------------------
	//---------클라이언트 아이디 --IR센서--IPC를 위한 파일 스트림 --- 집게 부품들 ---AGV의 현재 위치정보------------------------------------------------
public:
	//communiation with others
	AGV(int clientID, const simxChar* leftSensor_L, const simxChar* leftSensor, const simxChar* middleSensor, const simxChar* middleSensor_R,
		const simxChar* rightSensor, const simxChar* rightSensor_R, const simxChar* leftMotorHandle, const simxChar* rightftMotorHandle
		, const string FileStream, const string batteryStatePath, const simxChar* letfJoint, const simxChar* letfJoint2,
		const simxChar* rightJoint, const simxChar* rightJoint2, simxFloat objNumber, const simxChar* scriptHandle_make, const simxChar* scriptHandle_delete,
		int Current_x = 0, int Current_Y = 0, int Current_real = 0, int Current_img = -1)
	{

		//Set ClientID
		this->clientID = clientID;
		this->Thread_obj_num = Thread_obj_num;
		//ViSion Receive Vision Values 
		this->result = simxGetObjectHandle(clientID, leftSensor_L, &this->leftSensor_L, simx_opmode_oneshot_wait) +
			simxGetObjectHandle(clientID, leftSensor, &this->leftSensor, simx_opmode_oneshot_wait) +
			simxGetObjectHandle(clientID, middleSensor, &this->middleSensor, simx_opmode_oneshot_wait) +
			simxGetObjectHandle(clientID, middleSensor_R, &this->middleSensor_R, simx_opmode_oneshot_wait) +
			simxGetObjectHandle(clientID, rightSensor, &this->rightSensor, simx_opmode_oneshot_wait) +
			simxGetObjectHandle(clientID, rightSensor_R, &this->rightSensor_R, simx_opmode_oneshot_wait);

		//Motor_obj Connection
		simxGetObjectHandle(clientID, leftMotorHandle, &this->leftMotorHandle, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, rightftMotorHandle, &this->rightMotorHandle, simx_opmode_oneshot_wait);

		//Convey_obj Connection
		simxGetObjectHandle(clientID, letfJoint, &this->C_L_joint, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, letfJoint2, &this->C_L_joint2, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, rightJoint, &this->C_R_joint, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, rightJoint2, &this->C_R_joint2, simx_opmode_oneshot_wait);

		//Setting Obj
		this->obj = objNumber;

		//script setting
		simxGetObjectHandle(clientID, scriptHandle_make, &this->scriptHandle_make, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, scriptHandle_delete, &this->scriptHandle_delete, simx_opmode_oneshot_wait);

		//AGV의 위치 정보
		this->car_pos.x_pos = Current_x;
		this->car_pos.y_pos = Current_Y;
		this->state.real = Current_real;
		this->state.imag = Current_img;

		//파일스트림 경로
		this->s = FileStream;
		this->batteryStatePath = batteryStatePath;

		if (result != 0) {  //if Connection Failed
			cout << "Error Happend!" << endl;
			simxFinish(clientID);
		}
		printf("Vision Sensor has Connected!!!\n");

		simxStartSimulation(clientID, simx_opmode_oneshot);
		printf("Program is Runnning\n");

		//C#과의 주고받기를 위한 파일입출력 스트림 설정
		//out.open(s);

		//visionSensor Sensoring
		simxReadVisionSensor(clientID, this->leftSensor_L, NULL, NULL, NULL, simx_opmode_streaming);
		simxReadVisionSensor(clientID, this->leftSensor, NULL, NULL, NULL, simx_opmode_streaming);
		simxReadVisionSensor(clientID, this->middleSensor, NULL, NULL, NULL, simx_opmode_streaming);
		simxReadVisionSensor(clientID, this->middleSensor_R, NULL, NULL, NULL, simx_opmode_streaming);
		simxReadVisionSensor(clientID, this->rightSensor, NULL, NULL, NULL, simx_opmode_streaming);
		simxReadVisionSensor(clientID, this->rightSensor_R, NULL, NULL, NULL, simx_opmode_streaming);

		//Encoder Initialize
		simxSetJointPosition(clientID, this->leftMotorHandle, 0.0, simx_opmode_oneshot_wait);
		simxSetJointPosition(clientID, this->rightMotorHandle, 0.0, simx_opmode_oneshot_wait);

		//Encoder Value Output
		simxGetJointPosition(clientID, this->leftMotorHandle, &L_angle, simx_opmode_streaming);
		simxGetJointPosition(clientID, this->rightMotorHandle, &R_angle, simx_opmode_streaming);
	};
	void gogo();
	void getPath(int instruction);
	void pathCheck();
	int getInstruction();
	int getCarPosX();
	int getGarPosY();
	int getFlag();
	void setFlag(int flag);
	float getBattery();
	void ClearQueue(std::queue<int>& someQueue);
	void putMode(int mode);

	/*void showPath();*/
};
void AGV::gogo() {
	while (true)
	{
		if (path.empty())
		{
			flag = 1;
		}

		//VisionSensor FeedBack
		simxReadVisionSensor(clientID, leftSensor_L, &LL_vision_state, &LL_auxValues, &LL_auxValuesCount, simx_opmode_buffer);
		simxReadVisionSensor(clientID, leftSensor, &L_vision_state, &L_auxValues, &L_auxValuesCount, simx_opmode_buffer);
		simxReadVisionSensor(clientID, middleSensor, &M_vision_state, &M_auxValues, &M_auxValuesCount, simx_opmode_buffer);
		simxReadVisionSensor(clientID, middleSensor_R, &MR_vision_state, &MR_auxValues, &MR_auxValuesCount, simx_opmode_buffer);
		simxReadVisionSensor(clientID, rightSensor, &R_vision_state, &R_auxValues, &R_auxValuesCount, simx_opmode_buffer);
		simxReadVisionSensor(clientID, rightSensor_R, &RR_vision_state, &RR_auxValues, &RR_auxValuesCount, simx_opmode_buffer);

		//-----------Car LineTracing-----------------------------------------------------------------------------------

		//이제 맨끝에 가중치를 곱하여 줄 것 입니다!

		//Car LL Sensor Value
		LL_detectedColor.red = LL_auxValues[RED_VALUE];
		LL_detectedColor.green = LL_auxValues[GREEN_VALUE];
		LL_detectedColor.blue = LL_auxValues[BLUE_VALUE];
		double LeftSensorValue_L = LL_detectedColor.blue + LL_detectedColor.green + LL_detectedColor.red;

		//Car L Sensor Value
		L_detectedColor.red = L_auxValues[RED_VALUE];
		L_detectedColor.green = L_auxValues[GREEN_VALUE];
		L_detectedColor.blue = L_auxValues[BLUE_VALUE];
		double LeftSensorValue = L_detectedColor.blue + L_detectedColor.green + L_detectedColor.red;

		//Car M Sensor Value
		M_detectedColor.red = M_auxValues[RED_VALUE];
		M_detectedColor.green = M_auxValues[GREEN_VALUE];
		M_detectedColor.blue = M_auxValues[BLUE_VALUE];
		double MiddleSensorValue = M_detectedColor.blue + M_detectedColor.green + M_detectedColor.red;

		//Car MR Sensor Value
		MR_detectedColor.red = MR_auxValues[RED_VALUE];
		MR_detectedColor.green = MR_auxValues[GREEN_VALUE];
		MR_detectedColor.blue = MR_auxValues[BLUE_VALUE];
		double MiddleSensorValue_R = MR_detectedColor.blue + MR_detectedColor.green + MR_detectedColor.red;

		//Car R Sensor Value
		R_detectedColor.red = R_auxValues[RED_VALUE];
		R_detectedColor.green = R_auxValues[GREEN_VALUE];
		R_detectedColor.blue = R_auxValues[BLUE_VALUE];
		double RightSensorValue = R_detectedColor.blue + R_detectedColor.green + R_detectedColor.red;

		//Car RR Sensor Value
		RR_detectedColor.red = RR_auxValues[11];
		RR_detectedColor.green = RR_auxValues[12];
		RR_detectedColor.blue = RR_auxValues[13];
		double RightSensorValue_R = RR_detectedColor.blue + RR_detectedColor.green + RR_detectedColor.red;


		//-------------------------------센서값 받기 ----------------------------------------------------------------------------------------------------------

		double L_motor_speed = (MiddleSensorValue * 1.0 + LeftSensorValue * 2.1 + LeftSensorValue_L * 4.0) / 2.0;    //2.0

		double R_motor_speed = (MiddleSensorValue_R * 1.0 + RightSensorValue * 2.1 + RightSensorValue_R * 4.0) / 2.0;      //2.0

		double Sensor = RightSensorValue_R + RightSensorValue + MiddleSensorValue_R + LeftSensorValue_L + LeftSensorValue + MiddleSensorValue;

		in.open(batteryStatePath);

		if (in.is_open()) {
			in >> Bbuffer;
			BatteryPercent = atof(Bbuffer.c_str());
		}
		in.close();

		//--모드 0: 일반라인트레이싱 1: 좌회전  2: 우회전 ---------------------------------------------------------------------------------------
		switch (Mode)
		{
		case NOMAL_SENSING:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, L_motor_speed + Speed_Nomal, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, R_motor_speed + Speed_Nomal, simx_opmode_streaming);

			if (Sensor != 0)
				cnt_zero++;

			if ((Sensor < 0.1) && (Sensor != check_p) && cnt_zero > 3) {
				////쓰레드시 넣을 시 고민
				simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_oneshot_wait);
				simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_oneshot_wait);
				Mode = getInstruction();

				//printf("id : %d", clientID);
				//printmove(Mode);

				State_mode = state.ComplexToMode();
				//--------------허수를 이용한 좌표표현-------------------------------------------------------------------------------------------------------------------
				switch (State_mode)
				{
				case 1:
					car_pos.x_pos += 1;
					flag = 1;
					break;
				case 2:
					car_pos.y_pos -= 1;
					flag = 1;
					break;
				case 3:
					car_pos.x_pos -= 1;
					flag = 1;
					break;
				case 4:
					car_pos.y_pos += 1;
					flag = 1;
					break;
				default:
					break;
				}
				check_p = 9.0;
				cnt_zero = 0;
				/*cout << "AGV  :" << obj << " : "<<setw(8) << car_pos.x_pos << "  " << car_pos.y_pos << setfill(' ') << endl;*/
			}
			check_p = Sensor;
			break;
		case RIGHT:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, Speed_Nomal + L_motor_speed + 1.5, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, Speed_Nomal, simx_opmode_streaming);

			if (MiddleSensorValue_R != 0)
				cnt_zero++;

			if ((MiddleSensorValue_R < 0.1) && SensorChecking)
			{
				cnt_zero = 0;
				cnt++;
				SensorChecking = false;
			}

			if (MiddleSensorValue_R != check && cnt_zero > 3) {
				SensorChecking = true;
				cnt_zero = 0;
			}

			check = MiddleSensorValue_R;

			if (cnt == 2) {
				cnt = 0;
				cnt_zero = 0;
				Mode = NOMAL_SENSING;
				check = 15.0;
				SensorChecking = true;
				state = state * right_s;
			}
			break;
		case LEFT:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, Speed_Nomal, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, Speed_Nomal + R_motor_speed + 1.5, simx_opmode_streaming);

			if (MiddleSensorValue != 0)
				cnt_zero++;

			if ((MiddleSensorValue < 0.1) && SensorChecking)
			{
				cnt_zero = 0;
				cnt++;
				SensorChecking = false;
			}

			if (MiddleSensorValue != check && cnt_zero > 3) {
				SensorChecking = true;
				cnt_zero = 0;
			}

			check = MiddleSensorValue;

			if (cnt == 2) {
				cnt = 0;
				cnt_zero = 0;
				Mode = NOMAL_SENSING;
				check = 15.0;
				SensorChecking = true;
				state = state * left_s;
			}
			break;
		case REVERSE:
			//if ((car_pos.x_pos == 10 && car_pos.y_pos == 2) || (car_pos.x_pos == 10 && car_pos.y_pos == 5))
			//{

			//}

			simxSetJointTargetVelocity(clientID, leftMotorHandle, -(Speed_Nomal + R_motor_speed + 1.5) / 3.0, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, (Speed_Nomal + R_motor_speed + 1.5) / 3.0, simx_opmode_streaming);

			if (LeftSensorValue != 0)
				cnt_zero++;

			if ((LeftSensorValue < 0.1) && SensorChecking)
			{
				cnt_zero = 0;
				cnt++;
				SensorChecking = false;
			}

			if (LeftSensorValue != check && cnt_zero > 3) {
				SensorChecking = true;
				cnt_zero = 0;
			}

			//--------------------------------센서값 튀는 거 확인하도록 하시오!--------------------------------------------------------------------------

			//cout << "LeftSensorValue " << setw(8) << LeftSensorValue << setfill('0') << "  ";
			//cout << "check2: " << setw(8) << check2 << setfill('0') << "  ";
			//cout << "cnt2: " << setw(8) << cnt << setfill('0') << endl;

			//------------------------------------------------------------------------------------------------------------------------------------------

			check = LeftSensorValue;

			if (cnt == 2) {
				cnt = 0;
				cnt_zero = 0;
				Mode = NOMAL_SENSING;
				check = 15.0;
				SensorChecking = true;
				state = state * turn_s;
				//cout << "-----------------BBB--------------------------------------------------------------------" << endl;
			}
			break;
			//CONVEY이의 경우 다시 라인센싱으로 갈 경우 좌표가 The Blue로 더해질수가 있음.
		case CONVEY:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_oneshot_wait);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint, 0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint2, 0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint, 0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint2, 0 * (PI / 180), simx_opmode_oneshot_wait);
			sending[0] = obj;
			sending[1] = car_pos.x_pos * 10 + car_pos.y_pos;
			simxCallScriptFunction(clientID, "Section1", sim_scripttype_childscript, "make_obj", 1, &scriptHandle_make, 2, sending, 0, NULL,
				NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
			//while (path.empty()) {
			//	std::this_thread::sleep_for(std::chrono::seconds(10));
			//}
			Mode = getInstruction();
			IPCMODE = 1;
			break;
		case RELEASE:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_oneshot_wait);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint, -20.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint2, -20.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint, 20.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint2, 20.0 * (PI / 180), simx_opmode_oneshot_wait);
			sending[0] = obj;
			sending[1] = car_pos.x_pos * 10 + car_pos.y_pos;
			simxCallScriptFunction(clientID, "Section2", sim_scripttype_childscript, "delete_obj", 1, &scriptHandle_delete, 2, sending, 0, NULL,
				NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
			IPCMODE = 0;
			//while (path.empty()) {
			//	std::this_thread::sleep_for(std::chrono::seconds(10));
			//}
			Mode = getInstruction();
			break;
			//case BACK:
			//	if (R_Enable && L_Enable)
			//	{
			//		//Encoder Value Output
			//		simxGetJointPosition(clientID, leftMotorHandle, &L_angle, simx_opmode_streaming);
			//		simxGetJointPosition(clientID, rightMotorHandle, &R_angle, simx_opmode_streaming);

			//		R_angle_org = R_angle;
			//		L_angle_org = L_angle;
			//		R_Enable = false;
			//		L_Enable = false;
			//		R_temp = R_angle_org + 0.5;
			//		L_temp = L_angle_org + 0.5;
			//		if (R_temp > 3.14)
			//		{
			//			R_temp -= 6.28;
			//		}
			//		if (L_temp > 3.14)
			//		{
			//			L_temp -= 6.28;
			//		}
			//	}
			//	else
			//	{
			//		//Encoder Value Output
			//		simxGetJointPosition(clientID, leftMotorHandle, &L_angle, simx_opmode_streaming);
			//		simxGetJointPosition(clientID, rightMotorHandle, &R_angle, simx_opmode_streaming);

			//		simxSetJointTargetVelocity(clientID, leftMotorHandle, -Speed_Nomal, simx_opmode_streaming);
			//		simxSetJointTargetVelocity(clientID, rightMotorHandle, -Speed_Nomal, simx_opmode_streaming);


			//		if ((L_angle >= L_temp) && (R_angle >= R_temp))
			//		{
			//			Mode = LINKED_REVERSE;
			//			R_Enable = true;
			//			L_Enable = true;
			//			R_cnt = 1;
			//			L_cnt = 1;
			//			R_angle = 0;
			//			L_angle = 0;
			//		}
			//	}
			//	break;
			//case LINKED_REVERSE:
			//	simxSetJointTargetVelocity(clientID, leftMotorHandle, -(Speed_Nomal + R_motor_speed + 1.5) / 3.2, simx_opmode_streaming);
			//	simxSetJointTargetVelocity(clientID, rightMotorHandle, (Speed_Nomal + R_motor_speed + 1.5) / 3.2, simx_opmode_streaming);

			//	if (MiddleSensorValue != 0)
			//		cnt_zero++;

			//	if ((MiddleSensorValue < 0.1) && SensorChecking)
			//	{
			//		cnt_zero = 0;
			//		cnt++;
			//		SensorChecking = false;
			//	}

			//	if (MiddleSensorValue != check && cnt_zero > 3) {
			//		SensorChecking = true;
			//		cnt_zero = 0;
			//	}

			//	//--------------------------------센서값 튀는 거 확인하도록 하시오!--------------------------------------------------------------------------

			//	//cout << "LeftSensorValue " << setw(8) << LeftSensorValue << setfill('0') << "  ";
			//	//cout << "check2: " << setw(8) << check2 << setfill('0') << "  ";
			//	//cout << "cnt2: " << setw(8) << cnt << setfill('0') << endl;

			//	//------------------------------------------------------------------------------------------------------------------------------------------

			//	check = MiddleSensorValue;

			//	if (cnt == 2) {
			//		cnt = 0;
			//		cnt_zero = 0;
			//		Mode = NOMAL_SENSING;
			//		check = 15.0;
			//		SensorChecking = true;
			//		state = state * turn_s;
			//		//cout << "-----------------BBB--------------------------------------------------------------------" << endl;
			//	}
			//	break;
		case STOP34:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetPosition(clientID, C_L_joint, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint2, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint2, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			//노가다의 흔적
			//for (int i = 0; i < 6528000; i++)
			//	cout << "Hello World!" << endl;
				//for (int j = 0; j < 100000; j++)
					//int k = 0;
			out.open(s);
			if (out.is_open()) {
				out << "C";
			}
			out.close();
			std::this_thread::sleep_for(std::chrono::seconds(380));
			Mode = getInstruction();
			out.open(s);
			if (out.is_open()) {
				out << "A";
			}
			out.close();
			break;
		case STOP39:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetPosition(clientID, C_L_joint, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint2, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint2, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			//for (int i = 0; i < 7488000; i++)
			//	cout << "Hello World!" << endl;
			//for (int j = 0; j < 100000; j++)
				//int k = 0;
			out.open(s);
			if (out.is_open()) {
				out << "C";
			}
			out.close();
			std::this_thread::sleep_for(std::chrono::seconds(436));
			Mode = getInstruction();
			out.open(s);
			if (out.is_open()) {
				out << "A";
			}
			out.close();
			break;
		case STOP47:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetPosition(clientID, C_L_joint, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint2, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint2, 0.0 * (PI / 180), simx_opmode_oneshot_wait);
			//for (int i = 0; i < 9024000; i++)
			//	cout << "Hello World!" << endl;
			//for (int j = 0; j < 100000; j++)
				//int k = 0;
			out.open(s);
			if (out.is_open()) {
				out << "C";
			}
			out.close();
			std::this_thread::sleep_for(std::chrono::seconds(510));
			Mode = getInstruction();
			break;
		case STOPP:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetPosition(clientID, C_L_joint, 3.884e-04 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_L_joint2, -8.862e-03 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint, +2.756e-03 * (PI / 180), simx_opmode_oneshot_wait);
			simxSetJointTargetPosition(clientID, C_R_joint2, +5.017e-03 * (PI / 180), simx_opmode_oneshot_wait);
			//for (int i = 0; i < 9024000; i++)
			//	cout << "Hello World!" << endl;
			//for (int j = 0; j < 100000; j++)
				//int k = 0;
			if ((car_pos.x_pos == 10 && car_pos.y_pos == 2) || (car_pos.x_pos == 10 && car_pos.y_pos == 5))
			{
				out.open(s);
				if (out.is_open()) {
					out << "C";
				}
				out.close();
				//ClearQueue(this->path);
				while (path.empty())
				{
					in.open(batteryStatePath);

					if (in.is_open()) {
						in >> Bbuffer;
						BatteryPercent = atof(Bbuffer.c_str());
					}
					in.close();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
				std::this_thread::sleep_for(std::chrono::seconds(10));
				Mode = getInstruction();
			}
			else {
				out.open(s);
				if (out.is_open()) {
					out << "A";
				}
				out.close();
				while (path.empty())
				{
					in.open(batteryStatePath);

					if (in.is_open()) {
						in >> Bbuffer;
						BatteryPercent = atof(Bbuffer.c_str());
					}
					in.close();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
				std::this_thread::sleep_for(std::chrono::seconds(5));
				Mode = getInstruction();
			}

			break;
		case INITIALIZE:
			simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
			Mode = getInstruction();
			break;
		default:
			break;
		}
		//----------------------------------------------------------------------------------------------------------------------


		//----------------센싱값 확인 코드----------------------
		//cout << Thread_obj_num << " ";
		//cout << "Left L " << setw(8) << LeftSensorValue_L << setfill('0') << "  ";
		//cout << "Left: " << setw(8) << LeftSensorValue << setfill('0') << "  ";
		//cout << "Middle: " << setw(8) << MiddleSensorValue << setfill('0') << " ";
		//cout << "Middle R: " << setw(8) << MiddleSensorValue_R << setfill('0') << " ";
		//cout << "Right: " << setw(8) << RightSensorValue << setfill('0') << " ";
		//cout << "Right R: " << setw(8) << RightSensorValue_R << setfill('0') << endl;
		//----------------센싱값 확인 코드----------------------
		//simxGetJointPosition(clientID, leftMotorHandle, &L_angle, simx_opmode_buffer);
		//simxGetJointPosition(clientID, rightMotorHandle, &R_angle, simx_opmode_buffer);
		//cout << "왼쪽각도: " << setw(8) << L_angle << setfill('0') << " ";
		//cout << "오른쪽각도: " << setw(8) << R_angle << setfill('0') << endl;

		//--------IPC__MailSlot---------------------------------------------0:NOLMAL(NOT WORKING) 1:CARRYIGN 2:Chaging
		out.open(s);

		switch (IPCMODE) {
		case 0:
			if (out.is_open()) {
				out << "A";
				break;
			}
			break;
		case 1:
			if (out.is_open()) {
				out << "B";
				break;
			}
			break;
		case 2:
			if (out.is_open()) {
				out << "C";
				break;
			}
			break;
		default:
			break;
		}

		out.close();

		in.open(batteryStatePath);

		if (in.is_open()) {
			in >> Bbuffer;
			BatteryPercent = atof(Bbuffer.c_str());
		}
		in.close();

		//동기화 진행
		simxSynchronousTrigger(clientID); // Trigger next simulation ste
	}

}

void AGV::getPath(int instruction) {
	this->path.push(instruction);
}

void AGV::pathCheck() {
	this->path.pop();
}

int AGV::getInstruction() {
	if (path.empty())
	{
		ClearQueue(this->path);
		return STOPP;
	}
	int temp = this->path.front();
	pathCheck();
	printf("lineTracer%d, position : (%d, %d), bat : %.2f, remain_bat : %.2f,  relMove : %s\n", clientID + 1, lineTracer[clientID].carPos.first, lineTracer[clientID].carPos.second, lineTracer[clientID].bat, lineTracer[clientID].remain_bat, printmoves(CartoAGVEnum(lineTracer[clientID].route[0])));
	//printmove(CartoAGVEnum(lineTracer[clientID].route[0]));
	//printf("\n");

	return temp;
}

int AGV::getCarPosX() {
	return this->car_pos.x_pos;
}

int AGV::getGarPosY() {
	return this->car_pos.y_pos;
}

int AGV::getFlag() {
	return this->flag;
}

void AGV::setFlag(int flag) {
	this->flag = flag;
}

float AGV::getBattery() {
	return this->BatteryPercent;
}

void AGV::ClearQueue(std::queue<int>& someQueue)
{
	std::queue<int> empty;
	std::swap(someQueue, empty);
}

void AGV::putMode(int mode) {
	this->Mode = mode;
}


//점검용 Function 
//void AGV::showPath()
//{
//	for (int i = 0; i <= path.size(); i++)
//	{
//		if (path.empty())
//		{
//			cout << i + 1 << "번쨰 경로 " << setw(2) << "NO" << setfill('0') << " ";
//			break;
//		}
//		cout << i + 1 << "번쨰 경로 " << setw(2) << path[i] << setfill('0') << " ";
//	}
//	cout << endl;
//	cout << "좌표 값 " << setw(5) << car_pos.x_pos << "  " << car_pos.y_pos << setfill(' ') << endl;
//}

//int stop(int &clientID) {
//	string instruction;
//
//	cin >> instruction;
//	if (instruction == "stop") {
//		//thread1.join();
//		//thread2.join();
//		//thread3.join();
//		//thread4.join();
//		//simxFinish(clientID);
//		cout << "BYE Bye" << endl;
//		return clientID;
//	}
//}


void putRoot(int caridx) {

	if (lineTracer[caridx].relPointer + 1 == lineTracer[caridx].realpathLength()) {

		lineTracer[caridx].route[0] = lineTracer[caridx].realpath[lineTracer[caridx].relPointer];
		lineTracer[caridx].route[1] = STOP;
		lineTracer[caridx].addAbsPointer(); //절대 좌표값을 증가시키기!
		lineTracer[caridx].relPointer++; //상대 좌표값을 증가시키기!

		return;
	}
	else {

		int myNextX = lineTracer[caridx].path[lineTracer[caridx].absPointer + 1][0];
		int myNextY = lineTracer[caridx].path[lineTracer[caridx].absPointer + 1][1];

		//충돌하려 한다면
		for (int i = 0; i < 4; i++) {
			if (i != caridx) {

				if (lineTracer[caridx].route[0] == LIFT_UP) {
					//int myNextX2 = lineTracer[caridx].path[lineTracer[caridx].absPointer + 2][0];
					//int myNextY2 = lineTracer[caridx].path[lineTracer[caridx].absPointer + 2][1];

					if (lineTracer[i].carPos.first == lineTracer[caridx].route[0] && lineTracer[i].carPos.second == lineTracer[caridx].route[1]) {

						//정지해랏!!!
						lineTracer[caridx].route[0] = STOP;
						lineTracer[caridx].route[1] = STOP;
						cout << "================" << caridx + 1 << "linetracer charge STOP" << "================\n";

						return;

					}
				}

				if (lineTracer[i].carPos.first == myNextX && lineTracer[i].carPos.second == myNextY) {

					//정지해랏!!!
					lineTracer[caridx].route[0] = STOP;
					lineTracer[caridx].route[1] = STOP;
					cout << "================" << caridx + 1 << "linetracer STOP" << "================\n";

					return;
				}
			}
		}

		lineTracer[caridx].route[0] = lineTracer[caridx].realpath[lineTracer[caridx].relPointer];
		lineTracer[caridx].route[1] = lineTracer[caridx].realpath[lineTracer[caridx].relPointer + 1];
	}

	lineTracer[caridx].addAbsPointer(); //절대 좌표값을 증가시키기!
	lineTracer[caridx].relPointer++; //상대 좌표값을 증가시키기!

}

int bat_order = 0;
int temp_return = 0;
#define B_minus 9.4	//일의 최대량 최소량 평균의 70% 수치임.
#define C_minus 5.5	//충전소로 복귀하기까지의 최대수치임.
static float min_rule[CARNUM] = { C_minus + B_minus, C_minus + B_minus, C_minus + 2 * B_minus ,C_minus + 2 * B_minus };
//static float min_rule[CARNUM] = { C_minus + B_minus, C_minus + B_minus, C_minus + B_minus ,C_minus + B_minus };

static float max_rule = 75.0;
static int working_robot_num = CARNUM;
int getout = 0;

int charge_decision(int idx) {

	bat_order = 0;

	//몇번째로 배터리가 작은 녀석인가?
	for (int i = 0; i < CARNUM; i++) {
		if (lineTracer[idx].remain_bat < lineTracer[i].remain_bat)
			bat_order++;
	}

	//일이 완전히 끝났다면
	int posX = lineTracer[idx].carPos.first;
	int posY = lineTracer[idx].carPos.second;

	//일하는 수가 부족하면 일해라(배터리 적어도 강제로 일 시킴)
	int tmpX, tmpY, tmpBat = 0, highestBat = 0, highestindex = 0;
	for (int i = 0; i < 4; i++) {
		tmpX = lineTracer[i].carPos.first;
		tmpY = lineTracer[i].carPos.second;

		if ((tmpX == 10 && tmpY == 2) || (tmpX == 10 && tmpY == 5)) {
			tmpBat = lineTracer[i].remain_bat;
			if (tmpBat > highestBat) {
				highestBat = tmpBat;
				highestindex = i;
			}
		}
	}
	if ((getout > 0) && (idx == highestindex))
	{
		getout -= 1;
		temp_return = GOING_WORK;
		return temp_return;
	}

	if (lineTracer[idx].realpath.size() == 0) {
		return GOING_WORK;
	}

	//일 하나가 끝난것.
	if (lineTracer[idx].relPointer == (lineTracer[idx].realpath.size()))
	{
		if ((!(posX == 10 && posY == 2)) && (!(posX == 10 && posY == 5))) {
			if (min_rule[bat_order] < lineTracer[idx].remain_bat)  //배터리가 15 15 30 30보다 위에 있는가?
			{
				lineTracer[idx].remain_bat -= float(map.findShortroute(lineTracer[idx].carPos.first, lineTracer[idx].carPos.second)) * BATSUB;
				temp_return = GOING_WORK;    //그렇다면 한번 더 GOING_WORK.
				return temp_return;
			}
		}
		else {
			//충전소에 누군가 있다면 나가라고 해라
			if ((map.batLoc[0] == false) && (map.batLoc[1] == false)) {

				getout += 1;
				cout << "======================" << idx << "linetracer charging wait======================\n";
				return WORK_WAIT;
			}

			temp_return = GOING_CHARGE;   //아니라면 충전소로 가라.
			return temp_return;
		}
	}

	//풀충전이면 일해라
	if (lineTracer[idx].remain_bat >= max_rule) {
		lineTracer[idx].remain_bat -= float(map.findShortroute(lineTracer[idx].carPos.first, lineTracer[idx].carPos.second)) * BATSUB;
		temp_return = GOING_WORK;    //그렇다면 한번 더 GOING_WORK.
		return temp_return;                 //lineTracer[idx].work_bat = temp;
	}



	return GOING_CHARGE;
}

int main(int argc, char* argv[])
{
	srand(time(NULL));

	//라인트레이서 초기위치 및 배터리 잔량 설정
	lineTracer[0].carPos.first = 2;
	lineTracer[0].carPos.second = 4;
	lineTracer[0].bat = 83.7;
	lineTracer[1].carPos.first = 3;
	lineTracer[1].carPos.second = 2;
	lineTracer[1].bat = 76.5;
	lineTracer[2].carPos.first = 7;
	lineTracer[2].carPos.second = 3;
	lineTracer[2].bat = 92.5;
	lineTracer[3].carPos.first = 10;
	lineTracer[3].carPos.second = 5;
	lineTracer[3].bat = 57.7;

	int clientID = 0;
	int clientID2 = 0;
	int clientID3 = 0;
	int clientID4 = 0;

	string instruction;

	float Speed_Nomal = 2.0f;

	int num1 = 1;
	int num2 = 2;


	//initialization Associated Value
	int result;

	RGB colorOK;
	colorOK.red = 0;
	colorOK.green = 1;
	colorOK.blue = 0;


	//Remove previous Thread
	simxFinish(-1);
	simxFinish(-1);
	simxFinish(-1);
	simxFinish(-1);

	//Tcp/IP Connection with V-rep
	clientID = simxStart((simxChar*)"127.0.0.1", 20003, true, true, 2000, 5);
	clientID2 = simxStart((simxChar*)"127.0.0.1", 20002, true, true, 2000, 5);
	clientID3 = simxStart((simxChar*)"127.0.0.1", 20001, true, true, 2000, 5);
	clientID4 = simxStart((simxChar*)"127.0.0.1", 20000, true, true, 2000, 5);

	//Sleep(1000);

	if (clientID != -1)
	{
		simxAddStatusbarMessage(clientID, "Connected Completely!", simx_opmode_oneshot);
		cout << "Connection status to V REP : Success 서버연결 성공" << endl;

		//Synchronous Mode Enalble
		simxSynchronous(clientID, 1);
		simxSynchronous(clientID2, 1);
		simxSynchronous(clientID3, 1);
		simxSynchronous(clientID4, 1);
	}
	else {
		cout << "Connection status to V REP : Falled" << endl;
		simxFinish(clientID);
		return clientID;
	}


	//생성자-> 클라이언트아이디, 센서값 좌에서 우로, 모터돌릴 휠 좌 우,일하는 상태를 받는 입출력경로, 베터리량 을받는 입 출력 경로, 어디로 가는중인지(허수정보로 나타냄),현재의 위치정보
	AGV agv1 = AGV(clientID, "LeftSensorL1", "LeftSensor1", "MiddleSensor1", "MiddleSensorR1", "RightSensor1",
		"RightSensorR1", "DynamicLeftJoint1", "DynamicRightJoint1", "C:/Users/이동규/Desktop/embedSW/State1.txt"
		, "C:/Users/이동규/Desktop/embedSW/Battery1.txt", "K3_gripper_leftFingerJoint9", "K3_gripper_leftFingerJoint11"
		, "K3_gripper_rightFingerJoint9", "K3_gripper_rightFingerJoint11", 1, "Section1", "Section2", 2, 4, 1, 0);

	AGV agv2 = AGV(clientID2, "LeftSensorL2", "LeftSensor2", "MiddleSensor2", "MiddleSensorR2", "RightSensor2",
		"RightSensorR2", "DynamicLeftJoint2", "DynamicRightJoint2", "C:/Users/이동규/Desktop/embedSW/State2.txt"
		, "C:/Users/이동규/Desktop/embedSW/Battery2.txt", "K3_gripper_leftFingerJoint13", "K3_gripper_leftFingerJoint15"
		, "K3_gripper_rightFingerJoint13", "K3_gripper_rightFingerJoint15", 2, "Section1", "Section2", 3, 2, -1, 0);

	AGV agv3 = AGV(clientID3, "LeftSensorL3", "LeftSensor3", "MiddleSensor3", "MiddleSensorR3", "RightSensor3",
		"RightSensorR3", "DynamicLeftJoint3", "DynamicRightJoint3", "C:/Users/이동규/Desktop/embedSW/State3.txt"
		, "C:/Users/이동규/Desktop/embedSW/Battery3.txt", "K3_gripper_leftFingerJoint17", "K3_gripper_leftFingerJoint19"
		, "K3_gripper_rightFingerJoint17", "K3_gripper_rightFingerJoint19", 3, "Section1", "Section2", 7, 3, -1, 0);

	AGV agv4 = AGV(clientID4, "LeftSensorL4", "LeftSensor4", "MiddleSensor4", "MiddleSensorR4", "RightSensor4",
		"RightSensorR4", "DynamicLeftJoint4", "DynamicRightJoint4", "C:/Users/이동규/Desktop/embedSW/State4.txt"
		, "C:/Users/이동규/Desktop/embedSW/Battery4.txt", "K3_gripper_leftFingerJoint21", "K3_gripper_leftFingerJoint23"
		, "K3_gripper_rightFingerJoint22", "K3_gripper_rightFingerJoint24", 4, "Section1", "Section2", 10, 5, 1, 0);

	//--------------LED Shape------------------
	//int led = 0;
	//
	//simxGetObjectHandle(clientID, "sphere", &led, simx_opmode_oneshot_wait);


	std::thread thread1(&AGV::gogo, &agv1);
	std::thread thread2(&AGV::gogo, &agv2);
	std::thread thread3(&AGV::gogo, &agv3);
	std::thread thread4(&AGV::gogo, &agv4);

	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(CONVEY);
	//agv1.getPath(REVERSE);
	//agv1.getPath(LEFT);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(LEFT);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(RIGHT);
	//agv1.getPath(RIGHT);
	//agv1.getPath(RELEASE);
	//agv1.getPath(REVERSE);
	//agv1.getPath(LEFT);
	//agv1.getPath(LEFT);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(LEFT);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(NOMAL_SENSING);
	//agv1.getPath(LEFT);

	////---센싱값 확인

	////while (true) {
	////	agv4.showPath();
	////	Sleep(500);
	////}

	////simxFinish(clientID);
	////simxFinish(clientID2);
	////simxFinish(clientID3);
	////simxFinish(clientID);

	int lineState = WORK_WAIT;
	int posX, posY = 0;
	vector<vector<int>> tmproute;

	AGV* agv[CARNUM] = { &agv1, &agv2, &agv3, &agv4 };

	Sleep(500);

	for (int i = 0; i < CARNUM; i++) {
		lineTracer[i].bat = (*agv)[i].getBattery();
		lineTracer[i].remain_bat = lineTracer[i].bat;
	}

	while (1) {

		for (int lineIdx = 0; lineIdx < CARNUM; lineIdx++) {

			lineTracer[lineIdx].bat = (*agv)[lineIdx].getBattery();

			posX = lineTracer[lineIdx].carPos.first;
			posY = lineTracer[lineIdx].carPos.second;

			//물건들 위치 갱신하는 부분
			if (!(lineTracer[lineIdx].realpath.empty()) && !(lineTracer[lineIdx].relPointer == (lineTracer[lineIdx].realpath.size()))) {
				if (lineTracer[lineIdx].realpath[lineTracer[lineIdx].relPointer] == LIFT_DOWN) {
					int axis = map.tranRealLocTOStuffLoc(make_pair(posX, posY)); //실제좌표에서 물건 표시하는 숫자로 변환
					map.stuffLoc[axis] = YES;
				}
				if (lineTracer[lineIdx].realpath[lineTracer[lineIdx].relPointer] == LIFT_UP) {
					int axis = map.tranRealLocTOStuffLoc(make_pair(posX, posY));//실제좌표에서 물건 표시하는 숫자로 변환
					map.stuffLoc[axis] = NO;
				}
			}

			//충전기 위치에 있다면(충전 중이라면)
			if (((posX == 10 && posY == 2) || (posX == 10 && posY == 5)) && !(lineTracer[lineIdx].relPointer == 0)) {

				//충전알고리즘 실행 후 어떻게 해야할지 나옴	(GOING_WORK or GOING_CHARGE)
				lineState = charge_decision(lineIdx);

				if (lineTracer[lineIdx].batflag == 1) {
					lineTracer[lineIdx].batflag = 0;
					(*agv)[lineIdx].getPath(CartoAGVEnum(lineTracer[lineIdx].route[0]));
				}

				if (lineState == WORK_WAIT) {
					continue;
				}
				else if (lineState == GOING_WORK) {
					tmproute = map.makeroute(posX, posY, lineState); //일할 경로 생성
					lineTracer[lineIdx].putPath(tmproute);//일할 경로 삽입
					//putRoot(lineIdx);

					cout << lineIdx + 1 << "linetracer Path" << endl;
					for (int i = 0; i < lineTracer[lineIdx].realpath.size(); i++) {
						if (i % 5 == 0) {
							printf("\n");
						}
						printmove(CartoAGVEnum(lineTracer[lineIdx].realpath[i]));

					}
					printf("\n");

					if (lineTracer[lineIdx].route[0] != STOP) {
						if ((posX == 10 && posY == 2))
							map.batLoc[0] = true;
						else if ((posX == 10 && posY == 5))
							map.batLoc[1] = true;
					}
					lineTracer[lineIdx].batflag = 1;

					cout << lineIdx << "linetracer is out!!  " << "position : (" << lineTracer[lineIdx].carPos.first << ", " << lineTracer[lineIdx].carPos.second << ")\n";

				}
				lineTracer[lineIdx].bat = (*agv)[lineIdx].getBattery();
				lineTracer[lineIdx].remain_bat = lineTracer[lineIdx].bat;


			}
			//일하는 중이라면
			else {

				//일이 끝났는 지 통신
				lineTracer[lineIdx].routeIdx = (*agv)[lineIdx].getFlag();

				//첫 동작이 하나 끝났다면!
				if (lineTracer[lineIdx].routeIdx == 1) {

					//마지막 경로까지 간거라면!
					if (lineTracer[lineIdx].relPointer == (lineTracer[lineIdx].realpath.size())) {
						//충전알고리즘 실행 후 어떻게 해야할지 나옴	(GOING_WORK or GOING_CHARGE)-------------------재성이 파트
						lineState = charge_decision(lineIdx);


						if (lineState == WORK_WAIT) {
							continue;
						}
						tmproute = map.makeroute(posX, posY, lineState); //일할 경로 생성
						lineTracer[lineIdx].putPath(tmproute);//일할 경로 삽입

						cout << lineIdx + 1 << "linetracer Path" << endl;
						for (int i = 0; i < lineTracer[lineIdx].realpath.size(); i++) {
							if (i % 5 == 0) {
								printf("\n");
							}
							printmove(CartoAGVEnum(lineTracer[lineIdx].realpath[i]));

						}
						printf("\n");

					}
					else {
						//충돌방지 알고리즘 삽입

						(*agv)[lineIdx].getPath(CartoAGVEnum(lineTracer[lineIdx].route[0]));
						(*agv)[lineIdx].setFlag(0);
						putRoot(lineIdx);
					}

				}
				else {} //첫 동작이 아직 안끝났다면

			}

			lineTracer[lineIdx].bat = (*agv)[lineIdx].getBattery();

		}

	}

	thread1.join();
	thread2.join();
	thread3.join();
	thread4.join();


	return clientID;
}


/*
#include<iomanip>
라인트레이서 값확인
			cout << Thread_obj_num << " ";
			cout << "Left L " << setw(8) << LeftSensorValue_L << setfill('0') << "  ";
			cout << "Left: " << setw(8) << LeftSensorValue << setfill('0') << "  ";
			cout << "Middle: " << setw(8) << MiddleSensorValue << setfill('0') << " ";
			cout << "Middle R: " << setw(8) << MiddleSensorValue_R << setfill('0') << " ";
			cout << "Right: " << setw(8) << RightSensorValue << setfill('0') << " ";
			cout << "Right R: " << setw(8) << RightSensorValue_R << setfill('0') << endl;
*/


//목표 스레드 균할 분배 혹은 작업순서 지정
/*
OpenMP로 루프를 병렬 처리 하는 것은 매우 간단하지만 몇 가지 조건이 만족되어야 한다. 런타임에 OpenMP가 루프의 시작과 끝이 언제인지 알 수 있어야 하기 때문이다.



1. for 루프만 병렬화 가능하고 형태는 다음과 같아야 한다 for( 초기화 ; 조건; 증감)

2. 조건문의 카운터 변수는 int, unsigned int, C 포인터, C++ iterator 만 가능

3. 조건문의 카운터 변수는 병렬화할 코드의 작 또는 끝 부분에서 초기화 되어야한다.

4. 루프 내 벼변수는 루프 조건문의 카운터와 함께 증가 또는 감소되어야 한다.

5. 루프 조건문은 루프 카운터 변수와 >, >=, <, <= 중 하나를 이용한 비교문이어야 한다.



이러한 조건이 만족되지 않더라도 컴파일, 또는 런타임에 오류가 발생하지는 않지만 병렬 처리가 되지 않는다


*/


