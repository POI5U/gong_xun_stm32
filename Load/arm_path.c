#include "arm_path.h"
#include "act.h"
#include "FreeRTOS.h"

int64_t arm_move_start[2][5] = {
	{-116451 , -4161 , 26001 , 0, 20000},
	{-148012 , 37418 , 5537 , 1, 20000},
};


int64_t arm_move_state[1][5] = {
	{-116451 , -4161 , 26001 , 0, 20000},
};



//����̨ץȡ����
int64_t arm_look[1][5] = {
	{ 60220 , -2102 , 29566, 0 , 10000},	//����̨�Ͽ�����λ��
};

int64_t mid_get[4][5] = {
	{58615 , 31658 , 29461, 0, 30000},		//ץ��ǰһ��
	{58615 , 51526 , 4110, 0, 10000},		//ץ��λ��,δץ
	{58615 , 51526 , 4110, 1, 10000},		//ץ����
	{ 58615 , -5021 , 29666, 1, 50000},		//ץ��̧����
};

int64_t mid_put[4][5] = {
	{240508 , -11916 , 20852, 1,50000},		//�ŵ�ǰһ��,���Ϸ�
	{239626 , 11697 , 21852, 1, 10000},		//�ŵ�λ��,δ��
	{239626 , 11697 , 21852, 0, 10000},		//����
	{241569 , -6789 , 25904, 0, 30000},		//����̧����
};


int64_t left_get[4][5] = {
	{74805 , 39918 , -36515, 0, 30000},
	{74805 , 27919 , -47441, 0, 10000},
	{74805 , 27919 , -47441, 1, 10000},
	{74805 , 13807 , 7899, 1, 50000},
};

int64_t left_put[4][5] = {
	{212962 , -3896 , 11569, 1, 50000},
	{212075 , 14473 , 11682, 1, 10000},
	{212075 , 14473 , 11682, 0, 10000},
	{212235 , 11 , 13707, 0, 30000},
	
	
};



int64_t right_get[4][5] = {
	{46608 , 36970 , -38067, 0, 30000},
	{46608 , 27962 , -49080 , 0, 10000},
	{46608 , 27962 , -49080 , 1, 10000},
	{ 46608 , -7052 , 11398, 1, 50000},
};

int64_t right_put[4][5] = {
	{ -91712 , 2452 , 11564, 1, 50000},
	{-91880 , 14761 , 10714, 1, 10000},
	{-91880 , 14761 , 10714, 0, 10000},
	{ -90637 , -2810 , 13016, 0, 30000},
};


//Բ��λ�÷��ö���

int64_t p_arm_high_look[1][5] = {
	{-84889 , -19706 , 20532, 0, 20000},	//ת��ȥ֮ǰ̧��һ��
};


int64_t p_arm_look[1][5] = {
	{ 60800 , 19037 , -477, 0 , 10000},		//��Բ����λ��
};


//�ӳ���ץ���ϵĶ���,�м�Բ��(��ɫ)�Ķ���
int64_t p_mid_get[4][5] = {
	{241043 , -15679 , 21570, 0, 40000},    //ץ��ǰһ��
	{240224 , 12506 , 21619, 0, 10000},		//ץ��λ��,����δץ
	{240224 , 12506 , 21619, 1, 10000},		//ץȡ
	{ 241065 , -28501 , 20289, 1, 30000},	//ץ��̧����
	
};



//��Բ���ŵ�λ��
int64_t p_mid_put_before[2][5] = {
	{60305 , -11041 , 29732, 1, 50000},		//ת������λ��
	{60305 , 87361 , -47676, 1, 4000},		//���·�
};

int64_t p_mid_put_after[3][5] = {
	{60479 , 88795 ,  -51572, 1, 6000},		//����λ��,����δ��
	{60479 , 88795 ,  -51572, 0, 10000},	//����
	{60479 ,  37090 , -19520, 0, 20000},	//̧����
};

//����������ھ��ӹ��������(��ɫ)
int64_t p_mid_put[4][5] = {
	{59913 , -11041 , 29732, 1, 20000},		//����ǰһ��,���Ϸ�
	{59913 , 56440 , -16101 , 1, 10000},	//����λ��,צ��û���ſ�
	{59913 , 56440 , -16101 , 0, 10000},	//����,�ſ�צ��
	{59913 ,  37090 , -19520, 0, 20000},	//���˹���̧����
};

//���Բ��(��ɫ)�Ķ���
int64_t p_left_get[4][5] = {
	{211525 ,  -6387 , 14143, 0, 40000},
	{ 212188 , 15829 , 10546, 0, 10000},
	{ 212188 , 15829 , 10546, 1, 10000},
	{213111 , -24986 , 6630, 1, 50000},
};

int64_t p_left_put_before[2][5] = {
	{ 93058 , -8556 , 29537,1, 20000},
	{93058 , 70043 , -50393,1, 4000},	//
};

int64_t p_left_put_after[3][5] = {
	{92635 , 73344 , -54616, 1, 6000},
	{92635 , 73344 , -54616, 0, 10000},
	{92635 ,  37090 , -19520, 0, 20000},
	
};
//����������ھ��ӹ��������(��ɫ)
int64_t p_left_put[4][5] = {
	{ 92472 , -22660 , 29522, 1, 20000},
	{92472 , 49039 , -27867 , 1, 10000},
	{92472 , 49039 , -27867 , 0, 10000},
	{92472 ,  37090 , -19520, 0, 20000},
};

//�ұ�Բ��(��ɫ)�Ķ���
int64_t p_right_get[4][5] = {
	{ -91031 , -12553 , 10940, 0, 40000},
	{ -91637 , 15480 , 10755, 0, 10000},
	{ -91637 , 15480 , 10755, 1, 10000},
	{ -90934 , -23748 , 7910, 1, 50000},
};

int64_t p_right_put_before[2][5] = {
	{ 28262 , -22660 , 29522, 1, 20000},
	{28262 , 71457 , -51053, 1, 4000},
	
};
int64_t p_right_put_after[3][5] = {
	{28055 , 73062 , -53420, 1, 6000},
	{28055 , 73062 , -53420, 0, 10000},
	{28055 ,  37090 , -19520, 0, 20000},
};

//����������ھ��ӹ��������(��ɫ)
int64_t p_right_put[4][5] = {
	{ 27783 , -22660 , 29522, 1, 20000},
	{27783 , 49039 , -27867 , 1, 10000},
	{27783 , 49039 , -27867 , 0, 10000},
	{27783 ,  37090 , -19520, 0, 20000},
};




//�ּӹ���ץȡ����,�ž���ǰ���ץ�Ķ�������,put_before��put_after����һ�𵹷����put,ǰ���put���ų�get�Ķ���
int64_t pd_mid_put[4][5] = {
	{ 241065 , -23165 , 18045, 1, 20000}, 
	{240224 , 12506 , 21619, 1, 10000},
	{240224 , 12506 , 21619, 0, 10000},
	{241043 , -15679 , 21570, 0, 20000},  
};

int64_t pd_mid_get[5][5] = {
	
	{60522 , 64248 , -20023, 0, 20000},
	{ 60522 , 90730 , -51279, 0, 10000},
	{ 60522 , 90730 , -51279, 1, 10000},
	{ 60522 , 79321 , -35201, 1, 50000},
	{65075 , -11041 , 29732, 1,20000},
};



int64_t pd_left_put[4][5] = {
	{213111 , -24986 , 6630, 1, 20000},
	{ 212188 , 15829 , 10546, 1, 10000},
	{ 212188 , 15829 , 10546, 0, 10000},
	{211525 ,  -6387 , 14143, 0, 10000},
	
};

int64_t pd_left_get[5][5] = {
	{93559 , 75652 , -36226, 0, 20000},
	{ 93559 , 74228 , -53485, 0, 10000},
	{ 93559 , 74228 , -53485, 1, 10000},
	{93559 , 66438 , -42232,1, 20000},
	{ 96469 , -8556 , 29537,1, 20000},
};


int64_t pd_right_put[4][5] = {
	{ -90934 , -23748 , 7910, 1, 20000},
	{ -91637 , 15480 , 10755, 1, 10000},
	{ -91637 , 15480 , 10755, 0, 10000},
	{ -91031 , -12553 , 10940, 0, 20000},
};

int64_t pd_right_get[5][5] = {
	
	{27722 , 49605 , -27592, 0, 20000},
	{27722, 74198, -53736, 0, 10000},
	{27722, 74198, -53736, 1, 10000},
	{27722, 65565, -42912 , 1, 20000},
	{30591 , -22660 , 29522, 1, 20000},
};