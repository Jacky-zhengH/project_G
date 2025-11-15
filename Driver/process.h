#ifndef __PROCESS_H
#define __PROCESS_H

void HMI_Process_Init(void);
void HMI_Send_Cmd(const char *cmd_string);
void HMI_Process_Task_Poll(void);

#endif