#include "tracemap.h"

Map::Map() {

	
	//�� ����� ���� �κ�
	storedWork.resize(STORENUM); //��ĭ ������ �� �Ҵ�
	for (int i = 0; i < STORENUM; i++) {
		makeStoredWork(i);
	}

	//���͸� �ʱ�ȭ �κ�
	for (int i = 0; i < BATTERYNUM; i++) {
		batLoc[i] = true;
	}

	//�� ���� �κ�
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


		//�߰� �κ� ����
		pos[2][4] = RDL;
		pos[3][4] = DL;
		pos[4][4] = RDL;
		pos[5][4] = DL;
		pos[2][5] = UR;
		pos[3][5] = LUR;
		pos[4][5] = UR;
		pos[5][5] = LUR;

		//������ �κ� ����
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



		//ischeck TRUE�� �ʱ�ȭ
		for (int row = 0; row <= MAP_ROW; row++)
			for (int col = 0; col <= MAP_COL; col++) {
				ischecked[row][col] = true;
			}

	}
}

void Map::makeStoredWork(int order) {
	int yes[STUFFNUM] = {}; //������ �ִ� ��
	int no[STUFFNUM] = {}; //������ ���� ��
	int k1 = 0, k2 = 0; //������ �ִ� ����, ���� ����

	for (int i = 0; i < STUFFNUM; i++)
	{
		if (stuffLoc[i] == YES) {
			yes[k1] = i;
			k1++; //������ �ִ� ���� ����
		}
		else if (stuffLoc[i] == NO) {
			no[k2] = i;
			k2++; //������ ���� ���� ����
		}
	}

	//��ġ�� �������� �־��ִ� �κ�

	int stuffPickLoc, stuffLeaveLoc; //���� ��� ��ġ, ������ ��ġ
	int tmp; //�ӽ� ����

	tmp = rand() % k1; //������ �ִ� �ͺ��� ���� ���� �������� ����
	stuffPickLoc = yes[tmp];
	stuffLoc[yes[tmp]] = CHOSEN; //���õ� ������ ������
	tmp = rand() % k2;
	stuffLeaveLoc = no[tmp];
	stuffLoc[no[tmp]] = CHOSEN; //���õ� ������ ������

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

	int depth = 0; //����
	int dir = 0; // �� �� �ִ� ������ �˷��ִ� ����
	int parent = -1;
	int id = 0;
	

	//�ʱ�ȭ �ܰ�
	queue<vector<int>> que; //(x, y, depth, parent)�� ������ ��´�.

	vector<int> pre_data = { s_x, s_y, depth, id, parent}; //���� ������ �ǵ帮�� �κ�
	vector<int> now_data; // ���� �����͵� �ǵ帮�� �κ�

	vector<vector<int>> all_data; // ��� ��� ����
	vector<vector<int>> path; // ���� ��� ǥ��
	
	que.push(pre_data);

	//bfs ����
	while (!que.empty()) {

		//���� �� �ҷ����� ������
		pre_data = que.front();
		putCheck(pre_data[0], pre_data[1]);
		dir = getPos(pre_data[0], pre_data[1]);
		depth = pre_data[2] + 1;
		parent = pre_data[3];

		////�ȳ��� --------------------------------------------------------------------------------------------���߿� �������� ������!!
		//cout << "x : " << pre_data[0] << ", y : " << pre_data[1] << ", depth : "
		//	<< pre_data[2] << ", id : " << pre_data[3] << ", parent : " << pre_data[4] << endl;

		//��ǥ��ġ�� ã���� Ż��
		if (pre_data[0] == e_x && pre_data[1] == e_y) {
			path.push_back(pre_data); // ��ó�� ������ ����



			//���� ��� ��� ���� �κ�
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

		//��� ��� ��ϵ��� �����ϱ�
		all_data.push_back(pre_data);

		que.pop();

		//����  depth�� �� �� �ִ� ������ �߰��ϱ�
		if (dir == NODIR) {
			continue;
		}
		else if (dir == L || dir == LU || dir == LUR) {

			//�� �� �ִ� ��� �߰��ϱ�
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
	int shortSize = 9999; //���� ª�� �Ÿ��� ����
	int sCoordin, eCoordin; //������ǥ ����ǥ�� ����

	for (int i = 0; i < STORENUM; i++) {
		sCoordin = storedWork[i].first;

		pair<int, int> realSCoordin = tranStuffLocTORealLoc(sCoordin);

		vector<vector<int>> tmproute = BFS(x, y, realSCoordin.first, realSCoordin.second);

		int tmp = tmproute.size(); //���� ��ġ���� ���Ϸ� ���� �������� �Ÿ� ���

								   //���� ª�� �Ÿ��� ã�Ƴ���.
		if (tmp < shortSize) {
			shortSize = tmp;
		}
	}

	return shortSize;
}

vector<vector<int>> Map::makeroute(int car_x, int car_y, int status) {

	vector<vector<int>> route, tmproute, swap;

	if (status == GOING_WORK) {
		int shortestSize = 9999; //���� ª�� �Ÿ��� ����
		int shortidx; //���� ª�� �Ÿ��� index�� ����
		int sCoordin, eCoordin; //������ǥ ����ǥ�� ����

		for (int i = 0; i < STORENUM; i++) {
			sCoordin = storedWork[i].first;

			pair<int, int> realSCoordin = tranStuffLocTORealLoc(sCoordin);

			vector<vector<int>> tmproute = BFS(car_x, car_y, realSCoordin.first, realSCoordin.second);

			int tmp = tmproute.size(); //���� ��ġ���� ���Ϸ� ���� �������� �Ÿ� ���

									   //���� ª�� �Ÿ��� ã�Ƴ���.
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


		//���� ���� ������ ���� ���� �������� ���� �ο��Ѵ�.
		sCoordin = storedWork[shortidx].first;
		eCoordin = storedWork[shortidx].second;

		//������ ��� ���� ������ ���� FLOATING ���·� �����.
		stuffLoc[sCoordin] = FLOATING;
		stuffLoc[eCoordin] = FLOATING;

		//�� �ڸ��� ���ο� �� �ο�
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