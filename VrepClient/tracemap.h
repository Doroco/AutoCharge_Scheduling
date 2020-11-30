#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <time.h>

using namespace std;

#define MAP_ROW 7 // �迭����(0���� ����)
#define MAP_COL 10 // �迭����(0���� ����)

#define STUFFNUM 16 //���ǳ��� ���� ����
#define BATTERYNUM 2 //������ ����
#define STORENUM 4 //�����ϰ� �ִ� ���� ����

//���� ���¸� ǥ���ϴ� ����
typedef enum {
	GOING_WORK = 0,
	WORK_WAIT,
	GOING_CHARGE,
	//CHARGING,
	//CANTCHARGE // �����Ϸ� ���������� �����Ұ� ��������
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
	int pos[MAP_ROW + 1][MAP_COL + 1]; // ��ǥ�� ���� �� �� �ִ� �� Ȯ��
	bool ischecked[MAP_ROW + 1][MAP_COL + 1]; // ���� ���� üũ�س��� ���̻� ������ �ϱ�

public:

	int stuffLoc[STUFFNUM] = { YES, NO, YES, NO,
								YES, YES, YES, NO,
								NO, NO, NO, YES,
								NO, YES, YES, NO }; // ���� �������� ���������� ���� ����

	vector<pair<int, int>> storedWork; //������ �ϵ�(x,y) //�ܺκ���, ���� ����

	bool batLoc[BATTERYNUM] = {true, true}; // ���͸� ���� ������ ������� �Ⱥ������ �˼� �ִ� ����, true �� �� ��

	Map();

	void makeStoredWork(int order); //������ ���� ����� �Լ�

	pair<int, int> tranStuffLocTORealLoc(int order); //stuffLoc�� ���� ��ǥ��� ��ȯ �����ִ� �۾��� �ϴ� �Լ�
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

	//���ڷ� ǥ���� ��θ� ������ ������ ���� �������� ������ �˷���
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

	//��ǥ�� ��ȯ
	vector<vector<int>> BFS(int s_x, int s_y, int e_x, int e_y);

	//���߿��� ���� ª�� �Ÿ��� ã�Ƽ� ��ȯ����
	int findShortroute(int x, int y);
	
	//������ ���� ��, �κ��� ���� ���� �� ���� ��ġ, ����(���Ϸ� �����ϴ���, �����Ϸ� �����ϴ���)
	vector<vector<int>> makeroute(int x, int y, int status);
	
};