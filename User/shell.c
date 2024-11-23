#include "shell.h"


static ShellCmd_t REGD_CMD[MAX_CMD_NUM] = { 0 };


static void Shell_Calc(char* recv_cmd)
{	
	int param_num = 0;		//包含命令名
	char* ptr = NULL;
	char* param[MAX_PARAM_NUM] = { 0 };
	ptr = strtok(recv_cmd, USE_CUT_CMD);
	if(ptr == NULL){
		print("\r\n>>");
		return ;
	}
	
	while( ptr != NULL) {
		param[param_num++] = ptr;
		ptr = strtok(NULL, USE_CUT_CMD);
	}
	print("\r\n");
	for( int i = 0; i < MAX_CMD_NUM; i++) {
		if( strcmp( REGD_CMD[i].name, param[0]) == 0 ){		// 确认找到了对的命令
			if( param_num == 1){
				print("%s", REGD_CMD[i].help);
			}else{
				REGD_CMD[i].cmd_fun(param_num, param);
				return;
			}
			break;
		}
		if( i + 1 == MAX_CMD_NUM) {		// 未能找到
			print("cmd: /%s/ does not exist!!\r\n", param[0]);
		}
	}
	print(">>");
}


void ShellCmdRegister(char* cmdName, char* cmdHelp, cmd_func_t fun)
{
	static uint8_t cmd_num = 0;
	
	if( strlen(cmdName) > MAX_CMD_NAME_LONG ||
		strlen(cmdHelp) > MAX_HELP_LONG ){
		print("error: cmd len over limit!!!\r\n");
		return;
	}
	
	strcpy(REGD_CMD[cmd_num].name, cmdName);
	strcpy(REGD_CMD[cmd_num].help, cmdHelp);
	REGD_CMD[cmd_num].cmd_fun = fun;
	
	cmd_num++;
}



void UART_Recv_Callback(uint8_t recv_char)
{
	static uint8_t CMD_Buff[MAX_SHELL_CHAR] = { 0 };
	static uint32_t recv_cont = 0;
	
	if( recv_cont >= MAX_SHELL_CHAR){
		recv_cont = 0;
		return;
	}
	switch( recv_char) {
		case '\b':		// 退格键
		case 0x7F:		// 删除键
			if( recv_cont > 0){
				recv_cont --;
				print("\b \b");
			}
			break;
		case '\r':
		case '\n':
			CMD_Buff[recv_cont] = 0;
			recv_cont = 0;				// 为字符串添加结束标志
			// 进入处理函数
			Shell_Calc((char*)CMD_Buff);
			break;
		default:		// 一般字符
			CMD_Buff[recv_cont++] = recv_char;
			print("%c", recv_char);
	}
}



