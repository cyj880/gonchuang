#include "can.h"

/**********************************************************
***	ZDT_X57步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

__IO CAN_t can = {0};

/**
	* @brief   CAN2_RX0接收中断
	* @param   无
	* @retval  无
	*/
void CAN2_RX0_IRQHandler(void)
{
	// 接收一包数据
	CAN_Receive(CAN2, CAN_FIFO0, (CanRxMsg *)(&can.CAN_RxMsg));
	
	// 一帧数据接收完成，置位帧标志位
	can.rxFrameFlag = true;
}

/**
	* @brief   CAN发送多个字节（带发送完成等待与超时取消）
	* @param   cmd 命令缓存  len 长度
	* @retval  1=发送成功  0=发送失败（无节点ACK导致超时，已取消释放邮箱）
	* @note    总线上不存在目标地址的电机时，帧得不到ACK会一直自动重传，
	*          若不等待/取消会把3个发送邮箱全部卡死，导致后续命令全部发不出去！
	*/
uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
	uint32_t txMailbox, timeout;
	uint32_t rqcp_flag;

	// 除去ID地址和功能码后的数据长度
	j = len - 2;

	// 发送数据
	while(i < j)
	{
		// 数据个数
		k = j - i;

		// 填充缓存
		can.CAN_TxMsg.StdId = 0x00;
		can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		can.CAN_TxMsg.Data[0] = cmd[1];
		can.CAN_TxMsg.IDE = CAN_Id_Extended;
		can.CAN_TxMsg.RTR = CAN_RTR_Data;

		// 小于8字节命令
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = k + 1;
		}
		// 大于8字节命令，分包发送，每包数据最多发送8个字节
		else
		{
			for(l=0; l < 7; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = 8;
		}

		// 发送数据
		txMailbox = CAN_Transmit(CAN2, (CanTxMsg *)(&can.CAN_TxMsg));
		if (txMailbox == CAN_TxStatus_NoMailBox) return 0;

		// 选择对应邮箱的请求完成标志（RQCP：发送成功或失败均会置位）
		switch (txMailbox)
		{
			case 0:  rqcp_flag = CAN_FLAG_RQCP0; break;
			case 1:  rqcp_flag = CAN_FLAG_RQCP1; break;
			default: rqcp_flag = CAN_FLAG_RQCP2; break;
		}

		// 等待这一帧发送结束，约50ms超时
		timeout = 300000U;
		while (--timeout)
		{
			if (CAN_GetFlagStatus(CAN2, rqcp_flag) != RESET) break;
		}

		if (timeout == 0)
		{
			// 总线上无节点应答(无ACK)，取消发送释放邮箱，避免卡死
			CAN_CancelTransmit(CAN2, txMailbox);
			CAN_GetFlagStatus(CAN2, rqcp_flag);		// 清除标志
			return 0;
		}

		// 记录发送的第几包的数据
		++packNum;
	}
	return 1;
}
