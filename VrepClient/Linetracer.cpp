#include "Linetracer.h"

/*
	왼쪽 방향으로 있을 경우
	- [2~5][3, 7] / [2, 10], [5, 10]
	오른쪽 방향으로 있을 경우
	- [2~5][2, 6]

	-1, 0 -> 왼쪽으로 가기
	0, 1 -> 아래로 가기
	0, -1 -> 위로 가기
	1, 0 -> 오른쪽으로 가기

	차가 오른쪽 방향으로 있을 경우
		위로가면 좌회전
		아래로가면 우회전

		차가 왼쪽 방향으로 있을 경우
		위로 가면 우회전
		아래로 가면 좌회전
*/

bool Car::putRealPath() {
	//차의 입장에서 보는 경로를 표현해야함
	int now_x = 0 , now_y = 0, next_x = 0, next_y = 0;
	int pre_gap_x = 0, pre_gap_y = 0; //현재값 - 이전값
	int now_gap_x = 0, now_gap_y = 0; // 다음값 - 현재값
	int mode = 0; // 정방향인지, 역방향인지

	int count = 0; // 회전 값
	int result = 0;
	int find_x = pre_gap_x, find_y = pre_gap_y;

	int workFlag = 0;

	//절대경로에서 상대경로로 바꿀 때 한개가 소실된다.
	for (int i = 0; i < pathLength()-1; i++) {
		pre_gap_x = now_gap_x; pre_gap_y = now_gap_y; // 이전 경로 저장
		
		now_x = path[i][0]; next_x = path[i + 1][0];
		now_y = path[i][1]; next_y = path[i + 1][1];
		now_gap_x = next_x - now_x; now_gap_y = next_y - now_y;

		//같은 좌표가 나올때 -- 다음경로가 추가가 될 때...
		if (now_gap_x == 0 && now_gap_y == 0) {
			realpath.push_back(LIFT_UP);
			workFlag = 1;
			continue;
		}

		/*마지막에 충전기 들어가기 직전이라면*/
		if ((i == pathLength()-2) && ((now_x == 9 && now_y == 2) || (now_x == 9 && now_y == 5))) {
			mode = 1; //충전소 가기 직전
		}
		else if ((now_y == 2) || (now_y == 3) || (now_y == 4) || (now_y == 5)) {
			//충전기쪽에 있을 때
			if ((now_x == 10)) {
				realpath.push_back(CHARGE_OUT);
				mode = 0; // 정방향
				continue;
			}

			//화물쪽에 있을 때
			else if ((now_x == 2) || (now_x == 3) || (now_x == 6) || (now_x == 7)) {
				//처음에 나오게 되면 뒷꽁무니로 나오므로 처리를 해줘야한다.
				realpath.push_back(ROTATE_180);
				continue;
			}
		}

		count = 0; // 회전 값
		result = 0;
		find_x = pre_gap_x, find_y = pre_gap_y;

		//값이 같아질 때까지 계속 진행(복소수 개념)
		while (!((find_x == now_gap_x) && (find_y == now_gap_y))) {
			if (find_x == 1) {
				find_x = 0; find_y = 1;
			}
			else if (find_y == 1) {
				find_x = -1; find_y = 0;
			}
			else if (find_x == -1) {
				find_x = 0; find_y = -1;
			}
			else if (find_y == -1) {
				find_x = 1; find_y = 0;
			}
			count++;
		}

		//모드에 따른 경로 삽입
		switch (mode) {
		case 0:
			result = count % 4;
			realpath.push_back(result);
			break;
		case 1:
			result = count % 4;
			if (result == GO_F) {
				realpath.push_back(GO_F);
				realpath.push_back(STOP);
			}
			else {
				realpath.push_back(GO_R);
				realpath.push_back(ROTATE_180);
			}

			mode = 0;
			break;

		default:
			cerr << "error" << endl;
		}
	}

	if (workFlag == 1) {
		realpath.push_back(LIFT_DOWN);
	}

	return true;
}