#ifndef __SHELL_H__
#define __SHELL_H__

#include "stdint.h"
#include "system.h"

#define MAX_CMD_NUM			20			// 最多支持命令数量
#define MAX_SHELL_CHAR		50			// 一行命令最大长度
#define MAX_CMD_NAME_LONG	10			// 命令名字最大长度
#define MAX_HELP_LONG		180			// 帮助文本最大长度
#define USE_CUT_CMD			" "			// 分割一行内命令之间的符号, 一般只用空格就够了
#define MAX_PARAM_NUM		5			// 一行命令最大支持的参数个数(包含命令名)


typedef void(*cmd_func_t)(int argc, char**argv);


typedef struct ShellCmd_t {				/* 定义命令结构体 */
	char name[MAX_CMD_NAME_LONG]; 			/* 命令的名字 */
	cmd_func_t cmd_fun; 					/* 执行函数 */
	char help[MAX_HELP_LONG]; 				/* 帮助文字 */
}ShellCmd_t;



void UART_Recv_Callback(uint8_t recv_char);
void ShellCmdRegister(char* cmdName, char* cmdHelp, cmd_func_t fun);



#endif


