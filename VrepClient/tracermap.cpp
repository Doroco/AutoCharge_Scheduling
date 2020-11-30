#include "tracemap.h"

Map::Map() {

	
	//일 만들어 놓는 부분
	storedWork.resize(STORENUM); //몇칸 저장할 지 할당
	for (int i = 0; i < STORENUM; i++) {
		makeStoredWork(i);
	}

	//배터리 초기화 부분
	for (int i = 0; i < BATTERYNUM; i++) {
		batLoc[i] = true;
	}

	//맵 설정 부분
	{
		pos[0][0] = D; pos[MAP_ROW][0] = R; pos[0][MAP_COL] = L; pos[MAP_ROW][MAP_COL] = U; //corner - 4
		for (int row = 2; row <= MAP_ROW - 2; row++) {
			pos[row][2] = L; pos[row][3] = R; pos[row][6] = L; pos[row][7] = R; //load -16
			if (row == 2 || row == 5) pos[row][10] = L; //charger - 2
		}

		pos[0][MAP_COL] = NODIR; pos[1][MAP_COL] = NODIR; pos[3][MAP_COL] = NODIR; //charger illustion - 6
		pos[4][MAP_COL] = NODIR; pos[6][MAP_COL] = NODIR; pos[7][MAP_COL] = NODIR; //charger illustion - 6

		//column line 6*6 - 8 = 28
		for (int row = 1; row <= (MAP_ROW - 1); row++) {
			pos[row][0] = RD;
			pos[row][1] = LUR; //except for [1,1], [6,1]
			pos[row][4] = RDL; //except for [1,4], [6,4]
			pos[row][5] = LUR; //except for [1,5], [6,5]
			pos[row][8] = RDL; //except for [1,8], [6,8]
			pos[row][9] = LU; // except for [2,9], [5,10]
		}
		pos[1][1] = R; pos[6][1] = U; pos[1][4] = RD; pos[6][4] = DL; pos[1][5] = UR; pos[6][5] = LU; pos[1][8] = D; pos[6][8] = L;  pos[2][9] = LUR; pos[5][9] = LUR;//exception - 10

		//first row & last row 8*2 - 4 = 12
		for (int col = 1; col <= MAP_COL - 1; col++) {
			pos[0][col] = L; pos[MAP_ROW][col] = R; //except for [0,4], [0,5], [7,4], [7,5]
		}
		pos[0][4] = DL; pos[0][5] = L; pos[7][4] = R; pos[7][5] = UR; //exception -4

		//useless - 8
		pos[1][2] = R; pos[1][3] = R; pos[1][6] = R; pos[1][7] = R;
		pos[6][2] = L; pos[6][3] = L; pos[6][6] = L; pos[6][7] = L;


		//중간 부분 끊기
		pos[2][4] = RDL;
		pos[3][4] = DL;
		pos[4][4] = RDL;
		pos[5][4] = DL;
		pos[2][5] = UR;
		pos[3][5] = LUR;
		pos[4][5] = UR;
		pos[5][5] = LUR;

		//충전소 부분 끊기
		pos[3][8] = DL;
		pos[4][8] = DL;
		pos[2][9] = UR;
		pos[5][9] = UR;
		pos[7][9] = U;

		pos[3][9] = LU;
		pos[4][9] = LU;

		pos[2][9] = UR;
		pos[5][9] = UR;
		
		pos[2][0] = D;
		pos[4][0] = D;
		pos[3][1] = UR;
		pos[5][1] = UR;


		pos[1][0] = D;
		pos[1][9] = U;
		pos[6][0] = D;
		pos[6][9] = U;



		//ischeck TRUE로 초기화
		for (int row = 0; row <= MAP_ROW; row++)
			for (int col = 0; col <= MAP_COL; col++) {
				ischecked[row][col] = true;
			}

	}
}

void Map::makeStoredWork(int order) {
	int yes[STUFFNUM] = {}; //물건이 있는 곳
	int no[STUFFNUM] = {}; //물건이 없는 곳
	int k1 = 0, k2 = 0; //물건이 있는 갯수, 없는 갯수

	for (int i = 0; i < STUFFNUM; i++)
	{
		if (stuffLoc[i] == YES) {
			yes[k1] = i;
			k1++; //물건이 있는 것의 개수
		}
		else if (stuffLoc[i] == NO) {
			no[k2] = i;
			k2++; //물건이 없는 것의 갯수
		}
	}

	//위치를 랜덤으로 넣어주는 부분

	int stuffPickLoc, stuffLeaveLoc; //물건 드는 위치, 내리는 위치
	int tmp; //임시 변수

	tmp = rand() % k1; //물건이 있는 것보다 작은 수가 랜덤으로 생성
	stuffPickLoc = yes[tmp];
	stuffLoc[yes[tmp]] = CHOSEN; //선택된 것으로 정해줌
	tmp = rand() % k2;
	stuffLeaveLoc = no[tmp];
	stuffLoc[no[tmp]] = CHOSEN; //선택된 것으로 정해줌

	storedWork[order].first = stuffPickLoc;
	storedWork[order].second = stuffLeaveLoc;
}

pair<int, int> Map::tranStuffLocTORealLoc(int order){

	pair<int, int> realcoordin;
	switch (order) {
	case 0:
		realcoordin.first = 2;
		realcoordin.second = 2;
		break;
	case 1:
		realcoordin.first = 3;
		realcoordin.second = 2;
		break;
	case 2:
		realcoordin.first = 6;
		realcoordin.second = 2;
		break;
	case 3:
		realcoordin.first = 7;
		realcoordin.second = 2;
		break;
	case 4:
		realcoordin.first = 2;
		realcoordin.second = 3;
		break;
	case 5:
		realcoordin.first = 3;
		realcoordin.second = 3;
		break;
	case 6:
		realcoordin.first = 6;
		realcoordin.second = 3;
		break;
	case 7:
		realcoordin.first = 7;
		realcoordin.second = 3;
		break;
	case 8:
		realcoordin.first = 2;
		realcoordin.second = 4;
		break;
	case 9:
		realcoordin.first = 3;
		realcoordin.second = 4;
		break;
	case 10:
		realcoordin.first = 6;
		realcoordin.second = 4;
		break;
	case 11:
		realcoordin.first = 7;
		realcoordin.second = 4;
		break;
	case 12:
		realcoordin.first = 2;
		realcoordin.second = 5;
		break;
	case 13:
		realcoordin.first = 3;
		realcoordin.second = 5;
		break;
	case 14:
		realcoordin.first = 6;
		realcoordin.second = 5;
		break;
	case 15:
		realcoordin.first = 7;
		realcoordin.second = 5;
		break;
	
	}

	return realcoordin;
}

int Map::tranRealLocTOStuffLoc(pair<int, int> axis) {
	if (axis == make_pair(2, 2)) {
		return 0;
	}
	else if (axis == make_pair(3, 2)) {
		return 1;
	}
	else if (axis == make_pair(6, 2)) {
		return 2;
	}
	else if (axis == make_pair(7, 2)) {
		return 3;
	}
	else if (axis == make_pair(2, 3)) {
		return 4;
	}
	else if (axis == make_pair(3, 3)) {
		return 5;
	}
	else if (axis == make_pair(6, 3)) {
		return 6;
	}
	else if (axis == make_pair(7, 3)) {
		return 7;
	}
	else if (axis == make_pair(2, 4)) {
		return 8;
	}
	else if (axis == make_pair(3, 4)) {
		return 9;
	}
	else if (axis == make_pair(6, 4)) {
		return 10;
	}
	else if (axis == make_pair(7, 4)) {
		return 11;
	}
	else if (axis == make_pair(2, 5)) {
		return 12;
	}
	else if (axis == make_pair(3, 5)) {
		return 13;
	}
	else if (axis == make_pair(6, 5)) {
		return 14;
	}
	else if (axis == make_pair(7, 5)) {
		return 15;
	}
}

vector<vector<int>> Map::BFS(int s_x, int s_y, int e_x, int e_y)
{
	//cout << "BFS route!!!!" << endl;

	int depth = 0; //깊이
	int dir = 0; // 갈 수 있는 방향을 알려주는 변수
	int parent = -1;
	int id = 0;
	

	//초기화 단계
	queue<vector<int>> que; //(x, y, depth, parent)의 정보를 담는다.

	vector<int> pre_data = { s_x, s_y, depth, id, parent}; //뺄때 데이터 건드리는 부분
	vector<int> now_data; // 담을 데이터들 건드리는 부분

	vector<vector<int>> all_data; // 모든 경로 저장
	vector<vector<int>> path; // 실제 경로 표현
	
	que.push(pre_data);

	//bfs 실행
	while (!que.empty()) {

		//이전 값 불러오고 빼내기
		pre_data = que.front();
		putCheck(pre_data[0], pre_data[1]);
		dir = getPos(pre_data[0], pre_data[1]);
		depth = pre_data[2] + 1;
		parent = pre_data[3];

		////안내용 --------------------------------------------------------------------------------------------나중에 지워야지 빨라짐!!
		//cout << "x : " << pre_data[0] << ", y : " << pre_data[1] << ", depth : "
		//	<< pre_data[2] << ", id : " << pre_data[3] << ", parent : " << pre_data[4] << endl;

		//목표위치를 찾으면 탈출
		if (pre_data[0] == e_x && pre_data[1] == e_y) {
			path.push_back(pre_data); // 맨처음 데이터 저장



			//실제 경로 담기 위한 부분
			vector<int> tmp = pre_data;
			for (int i = depth-1; i > 0; i--) {
				for (int j = 0; j < all_data.size(); j++) {
					if (all_data[j][3] == tmp[4]) {
						tmp = all_data[j];
						path.push_back(tmp);
					}
				}
			}
			initCheck();
			//cout << "============================================" << endl;
			return path;
		}

		//모든 경로 기록들을 저장하기
		all_data.push_back(pre_data);

		que.pop();

		//다음  depth의 갈 수 있는 곳들을 추가하기
		if (dir == NODIR) {
			continue;
		}
		else if (dir == L || dir == LU || dir == LUR) {

			//갈 수 있는 경로 추가하기
			if (getCheck(pre_data[0] - 1, pre_data[1])) {
				//putCheck(pre_data[0] - 1, pre_data[1]);
				now_data = { pre_data[0] - 1, pre_data[1], depth, ++id, parent };
				que.push(now_data);
			}


			if (dir == LU || dir == LUR) {
				if (getCheck(pre_data[0], pre_data[1] - 1)) {
					//putCheck(pre_data[0], pre_data[1] - 1);
					now_data = { pre_data[0], pre_data[1] - 1, depth, ++id, parent };
					que.push(now_data);
				}

				if (dir == LUR) {
					if (getCheck(pre_data[0] + 1, pre_data[1])) {
						//putCheck(pre_data[0] + 1, pre_data[1]);
						now_data = { pre_data[0] + 1, pre_data[1], depth, ++id, parent };
						que.push(now_data);

					}
				}
			}
		}
		else if (dir == U || dir == UR || dir == URD) {
			if (getCheck(pre_data[0], pre_data[1] - 1)) {
				//putCheck(pre_data[0], pre_data[1] - 1);
				now_data = { pre_data[0], pre_data[1] - 1, depth, ++id, parent };
				que.push(now_data);

			}

			if (dir == UR || dir == URD) {
				if (getCheck(pre_data[0] + 1, pre_data[1])) {
					//putCheck(pre_data[0] + 1, pre_data[1]);
					now_data = { pre_data[0] + 1, pre_data[1], depth, ++id, parent };
					que.push(now_data);

				}

				if (dir == URD) {
					if (getCheck(pre_data[0], pre_data[1] + 1)) {
						//putCheck(pre_data[0], pre_data[1] + 1);
						now_data = { pre_data[0], pre_data[1] + 1, depth, ++id, parent };
						que.push(now_data);

					}

				}
			}
		}
		else if (dir == R || dir == RD || dir == RDL) {
			if (getCheck(pre_data[0] + 1, pre_data[1])) {
				//putCheck(pre_data[0] + 1, pre_data[1]);
				now_data = { pre_data[0] + 1, pre_data[1], depth, ++id, parent };
				que.push(now_data);


			}

			if (dir == RD || dir == RDL) {
				if (getCheck(pre_data[0], pre_data[1] + 1)) {
					//putCheck(pre_data[0], pre_data[1] + 1);
					now_data = { pre_data[0], pre_data[1] + 1, depth, ++id, parent };
					que.push(now_data);

				}

				if (dir == RDL) {
					if (getCheck(pre_data[0] - 1, pre_data[1])) {
						//putCheck(pre_data[0] - 1, pre_data[1]);
						now_data = { pre_data[0] - 1, pre_data[1], depth, ++id, parent };
						que.push(now_data);

					}

				}
			}
		}
		else if (dir == D || dir == DL) {
			if (getCheck(pre_data[0], pre_data[1] + 1)) {
				//putCheck(pre_data[0], pre_data[1] + 1);
				now_data = { pre_data[0], pre_data[1] + 1, depth, ++id, parent };
				que.push(now_data);


			}

			if (dir == DL) {
				if (getCheck(pre_data[0] - 1, pre_data[1])) {
					//putCheck(pre_data[0] - 1, pre_data[1]);
					now_data = { pre_data[0] - 1, pre_data[1], depth, ++id, parent };
					que.push(now_data);


				}

			}
		}

	}



	return vector<vector<int>>();
}

int Map::findShortroute(int x, int y) {
	int shortSize = 9999; //가장 짧은 거리를 저장
	int sCoordin, eCoordin; //시작좌표 끝좌표를 저장

	for (int i = 0; i < STORENUM; i++) {
		sCoordin = storedWork[i].first;

		pair<int, int> realSCoordin = tranStuffLocTORealLoc(sCoordin);

		vector<vector<int>> tmproute = BFS(x, y, realSCoordin.first, realSCoordin.second);

		int tmp = tmproute.size(); //차의 위치에서 일하러 가는 곳까지의 거리 계산

								   //제일 짧은 거리를 찾아낸다.
		if (tmp < shortSize) {
			shortSize = tmp;
		}
	}

	return shortSize;
}

vector<vector<int>> Map::makeroute(int car_x, int car_y, int status) {

	vector<vector<int>> route, tmproute, swap;

	if (status == GOING_WORK) {
		int shortestSize = 9999; //가장 짧은 거리를 저장
		int shortidx; //가장 짧은 거리의 index를 저장
		int sCoordin, eCoordin; //시작좌표 끝좌표를 저장

		for (int i = 0; i < STORENUM; i++) {
			sCoordin = storedWork[i].first;

			pair<int, int> realSCoordin = tranStuffLocTORealLoc(sCoordin);

			vector<vector<int>> tmproute = BFS(car_x, car_y, realSCoordin.first, realSCoordin.second);

			int tmp = tmproute.size(); //차의 위치에서 일하러 가는 곳까지의 거리 계산

									   //제일 짧은 거리를 찾아낸다.
			if (tmp < shortestSize) {
				shortestSize = tmp;
				shortidx = i;
				route.swap(tmproute);
			}
		}

		for (int i = route.size() - 1; i >= 0; i--) {
			swap.push_back(route[i]);
		}

		route.swap(swap);
		swap.clear();


		//물건 집는 곳에서 물건 놓는 곳까지의 일을 부여한다.
		sCoordin = storedWork[shortidx].first;
		eCoordin = storedWork[shortidx].second;

		//물건을 들고 놓을 예정인 곳은 FLOATING 상태로 만든다.
		stuffLoc[sCoordin] = FLOATING;
		stuffLoc[eCoordin] = FLOATING;

		//그 자리에 새로운 일 부여
		makeStoredWork(shortidx);

		pair<int, int> realSCoordin = tranStuffLocTORealLoc(sCoordin);
		pair<int, int> realECoordin = tranStuffLocTORealLoc(eCoordin);


		tmproute = BFS(realSCoordin.first, realSCoordin.second, realECoordin.first, realECoordin.second);

		for (int i = tmproute.size() - 1; i >= 0; i--) {
			swap.push_back(tmproute[i]);
		}

		tmproute.swap(swap);

		for (int i = 0; i < tmproute.size(); i++)
			route.push_back(tmproute[i]);
	}
	else if (status == GOING_CHARGE) {

		if (batLoc[0] == true) {
			route = BFS(car_x, car_y, 10, 2);

			for (int i = route.size() - 1; i >= 0; i--) {
				swap.push_back(route[i]);
			}

			route.swap(swap);
			batLoc[0] = false;
		}
		else if (batLoc[1] == true) {
			route = BFS(car_x, car_y, 10, 5);

			for (int i = route.size() - 1; i >= 0; i--) {
				swap.push_back(route[i]);
			}

			route.swap(swap);
			batLoc[1] = false;
		}
	}

	swap.clear();
	for (int i = route.size() - 1; i >= 0; i--) {
		swap.push_back(route[i]);
	}

	route.swap(swap);

	return route;
}