#include "Linetracer.h"

/*
	���� �������� ���� ���
	- [2~5][3, 7] / [2, 10], [5, 10]
	������ �������� ���� ���
	- [2~5][2, 6]

	-1, 0 -> �������� ����
	0, 1 -> �Ʒ��� ����
	0, -1 -> ���� ����
	1, 0 -> ���������� ����

	���� ������ �������� ���� ���
		���ΰ��� ��ȸ��
		�Ʒ��ΰ��� ��ȸ��

		���� ���� �������� ���� ���
		���� ���� ��ȸ��
		�Ʒ��� ���� ��ȸ��
*/

bool Car::putRealPath() {
	//���� ���忡�� ���� ��θ� ǥ���ؾ���
	int now_x = 0 , now_y = 0, next_x = 0, next_y = 0;
	int pre_gap_x = 0, pre_gap_y = 0; //���簪 - ������
	int now_gap_x = 0, now_gap_y = 0; // ������ - ���簪
	int mode = 0; // ����������, ����������

	int count = 0; // ȸ�� ��
	int result = 0;
	int find_x = pre_gap_x, find_y = pre_gap_y;

	int workFlag = 0;

	//�����ο��� ����η� �ٲ� �� �Ѱ��� �ҽǵȴ�.
	for (int i = 0; i < pathLength()-1; i++) {
		pre_gap_x = now_gap_x; pre_gap_y = now_gap_y; // ���� ��� ����
		
		now_x = path[i][0]; next_x = path[i + 1][0];
		now_y = path[i][1]; next_y = path[i + 1][1];
		now_gap_x = next_x - now_x; now_gap_y = next_y - now_y;

		//���� ��ǥ�� ���ö� -- ������ΰ� �߰��� �� ��...
		if (now_gap_x == 0 && now_gap_y == 0) {
			realpath.push_back(LIFT_UP);
			workFlag = 1;
			continue;
		}

		/*�������� ������ ���� �����̶��*/
		if ((i == pathLength()-2) && ((now_x == 9 && now_y == 2) || (now_x == 9 && now_y == 5))) {
			mode = 1; //������ ���� ����
		}
		else if ((now_y == 2) || (now_y == 3) || (now_y == 4) || (now_y == 5)) {
			//�������ʿ� ���� ��
			if ((now_x == 10)) {
				realpath.push_back(CHARGE_OUT);
				mode = 0; // ������
				continue;
			}

			//ȭ���ʿ� ���� ��
			else if ((now_x == 2) || (now_x == 3) || (now_x == 6) || (now_x == 7)) {
				//ó���� ������ �Ǹ� �޲ǹ��Ϸ� �����Ƿ� ó���� ������Ѵ�.
				realpath.push_back(ROTATE_180);
				continue;
			}
		}

		count = 0; // ȸ�� ��
		result = 0;
		find_x = pre_gap_x, find_y = pre_gap_y;

		//���� ������ ������ ��� ����(���Ҽ� ����)
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

		//��忡 ���� ��� ����
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