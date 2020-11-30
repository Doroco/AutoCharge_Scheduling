#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <time.h>

using namespace std;

#define MAP_ROW 7 // 배열기준(0부터 시작)
#define MAP_COL 10 // 배열기준(0부터 시작)

#define STUFFNUM 16 //물건놓는 곳의 갯수
#define BATTERYNUM 2 //충전소 갯수
#define STORENUM 4 //저장하고 있는 일의 갯수

//차의 상태를 표현하는 변수
typedef enum {
	GOING_WORK = 0,
	WORK_WAIT,
	GOING_CHARGE,
	//CHARGING,
	//CANTCHARGE // 충전하러 가야하지만 충전소가 꽉찬상태
}CAR_STATUS;

//direction
typedef enum {
	NODIR = 0, //0          1
	L, U, R, D, //1~4       4
	LU, UR, RD, DL, //5~8   4
	LUR, URD, RDL, //9~11   3
	LURD //12               1
} MAP_DIR;

typedef enum {
	YES,
	FLOATING,
	NO,
	CHOSEN
}ISSTUFF;

class Map {

private:
	int pos[MAP_ROW + 1][MAP_COL + 1]; // 좌표에 따른 갈 수 있는 곳 확인
	bool ischecked[MAP_ROW + 1][MAP_COL + 1]; // 갔던 곳은 체크해놔서 더이상 못가게 하기

public:

	int stuffLoc[STUFFNUM] = { YES, NO, YES, NO,
								YES, YES, YES, NO,
								NO, NO, NO, YES,
								NO, YES, YES, NO }; // 왼쪽 좌측부터 오른쪽으로 세기 시작

	vector<pair<int, int>> storedWork; //저장한 일들(x,y) //외부벡터, 일의 갯수

	bool batLoc[BATTERYNUM] = {true, true}; // 배터리 충전 공간이 비었는지 안비었는지 알수 있는 변수, true 가 빈 것

	Map();

	void makeStoredWork(int order); //저장할 일을 만드는 함수

	pair<int, int> tranStuffLocTORealLoc(int order); //stuffLoc을 실제 좌표계로 변환 시켜주는 작업을 하는 함수
	int tranRealLocTOStuffLoc(pair<int, int> axis);

	int getPos(int x_col, int y_row) {
		return pos[y_row][x_col];
	}

	bool getCheck(int x_col, int y_row) {
		return ischecked[y_row][x_col];
	}

	void putCheck(int x_col, int y_row) {
		ischecked[y_row][x_col] = false;
	}

	void initCheck() {
		for (int row = 0; row <= MAP_ROW; row++)
			for (int col = 0; col <= MAP_COL; col++) {
				ischecked[row][col] = true;
			}
	}

	//숫자로 표현된 경로를 넣으면 실제로 무슨 방향으로 가는지 알려줌
	void printRealpath(int realpath) {
		switch (realpath) {
		case 0:
			cout << "GO_F" << "\t";
			break;
		case 1:
			cout << "GO_R" << "\t";
			break;
		case 3:
			cout << "GO_L" << "\t";
			break;
		case 4:
			cout << "CHARGE_OUT" << "\t";
			break;
		case 5:
			cout << "ROTATE_180" << "\t";
			break;
		case 6:
			cout << "LIFT_UP" << "\t";
			break;
		case 7:
			cout << "LIFT_DOWN" << "\t";
			break;
		case 8:
			cout << "STOP" << "\t";
			break;
		}
	}

	//좌표값 반환
	vector<vector<int>> BFS(int s_x, int s_y, int e_x, int e_y);

	//일중에서 가장 짧은 거리를 찾아서 반환해줌
	int findShortroute(int x, int y);
	
	//업무를 만들어서 줌, 로봇이 일이 끝난 후 있을 위치, 상태(일하러 가야하는지, 충전하러 가야하는지)
	vector<vector<int>> makeroute(int x, int y, int status);
	
};