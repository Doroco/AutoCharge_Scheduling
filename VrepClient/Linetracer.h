#pragma once
#include "tracemap.h"

///////////////////////////////////////////////////충전알고리즘에서 사용하는 부분
#define BATADD 1.1
#define BATSUB 0.3
#define CARNUM 4

static int carID = 0; //차에 아이디 부여하는 변수

//차 조종하기 위한 방향
typedef enum {
	GO_F = 0, // forward 앞으로 가는 것
	GO_R, // right 오른쪽으로 가는 것
	GO_B, // backward 뒤쪽으로 가는 것
	GO_L, // left 왼쪽으로 가는 것
	CHARGE_OUT, //충전하고 나갈 때,
	ROTATE_180, // 180도 회전하고 나가는 것까지
	LIFT_UP,
	LIFT_DOWN,
	STOP, // 멈춤
}CAR_CONTROL_DIR;

class Car {

private:
	const int id = carID++; // 차 아이디 부여하기
	//bool chargeFlag = false; //충전해야하는 놈인지 아닌 지를 표현하는 변수
	//<int, int> carDestin; //차가 마지막에 가려고 하는 곳 -- 쓸 일 없음(carpos사용하면 됌)

public:

	pair<int, int> carPos; // 차가 현재 있는 위치

	vector<vector<int>> path; //가야하는 전체 경로를 표현(절대경로)
	vector<int> realpath; // 차가 실제로 가는 경로(상대경로)

	int routeIdx = 1; //route를 가르키는 index (0,1을 사용) -- 초기에 1이어야 일을 받을 수 있음
	int route[2] = { STOP, STOP }; //두칸씩 저장하고 있는 경로

	int batflag = 1; //배터리 경로로 왔을 때 한번더 움직이게 한다.
	int batflag2 = 1; //충전소에서 나갈때 0을 만들어 준다.


	int absPointer = 0; //경로까지 진행상황, 절대 좌표에서의 포인터
	int relPointer = 0; //경로까지 진행상황, 상대 좌표에서의 포인터

	float bat; // 배터리 잔량 표시 변수
	float remain_bat;	//현재 하고있는 일 한 후의 배터리
	float work_bat; //현재 하고있는 일의 거리에 해당하는 배터리가 저장		//bat- work_bat = remain_bat
	//float remainChrgBat; //충전소까지 가는 거리에 해당하는 배터리가 저장

	

	Car() {
		//chargeFlag = false;
	}

		///나중에 초기화 해주기========================================
	Car(int x, int y) {
		carPos = make_pair(x, y);
	}

	//relPointer증가 시키기 전에 사용해야한다! -- 항상 증가하는 것이 아니기에 함수로 증가시키도록 만듬
	void addAbsPointer() {

		int carDir = realpath[relPointer];

		if ((carDir == LIFT_DOWN) || (carDir == STOP)) {}
		else {
			absPointer++;
			carPos.first = path[absPointer][0];
			carPos.second =path[absPointer][1];
		}
	}

	//배터리순 AGV 정렬
	bool comp_by_bat(static Car& a, static Car& b) {			//배터리순으로 정렬하기 위한 함수
		return a.bat < b.bat;
	}

	//id순 AGV 정렬
	bool comp_by_id(static Car& a, static Car& b) {				//id순으로 정렬하기 위한 함수
		return a.id < b.id;
	}

	//차가 가야하는 절대 경로를 넣음 - 거꾸로 들어온 경로를 뒤집어 넣음으로써 정상적인 순서가 된다.
	bool putPath(vector<vector<int>> path) {

		//이전에 저장되었던 경로를 다 초기화시켜준다.
		this->path.clear();
		this->realpath.clear();
		this->absPointer = 0;
		this->relPointer = 0;
		this->work_bat = path.size() - 1;
		this->remain_bat = this->bat - this->work_bat * BATSUB;

		if (path.size() == 0)
			return false;
		else {

			for (int i = 0; i < path.size(); i++) {
				this->path.push_back(path[path.size() - 1 - i]);
			}

			putRealPath(); //상대경로를 만드는 소스
			return true;
		}
	}

	//경로 중 원하는 부분 반환(absPointer를 인자로 넣는다.)
	vector<int> getPath(int i) {
		if (i > pathLength()) {
			cerr << "wrong size input!!!" << endl;
		}
		else {
			//나중에 지워야할 부분!!!-----------------------------------------------------------------------------
			cout << "x : " << path[i][0] << ", y : " << path[i][1] << ", depth : "
				<< path[i][2] << ", id : " << path[i][3] << ", parent : " << path[i][4] << endl;
			return path[i];
		}

	}

	//차를 조종하기 위한 실제 경로를 만들어줌(차 입장)
	bool putRealPath();

	//절대 경로의 길이 반환(pc입장)
	int pathLength() {
		return path.size();
	}

	//상대 경로의 명령어길이 반환(차입장)
	int realpathLength() {
		return realpath.size(); // 실제 가는 길이는 처음 시작하는 경로를 빼야하기 때문에...
	}

	//남은 상대 경로 길이 반환(차입장)
	int remainPathLength() {
		return realpathLength() - relPointer - 1; // 실제 가는 길이는 처음 시작하는 경로를 빼야하기 때문에...
	}

	//차의 ID 받기
	int getCarID() {
		return id;
	}

	//route index를 바꿔주는 함수
	void putRouteIdx(int i) {
		routeIdx = i;
	}

	//route index를 받아오는 함수
	int getRouteIdx() {
		return routeIdx;
	}

	//route 정보를 받아온다.
	int* getRoute() {
		return route;
	}

};


