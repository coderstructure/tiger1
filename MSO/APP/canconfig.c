#include "can.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "canconfig.h"
#include "main.h"
//#include "bsp_can.h"
extern int v=8000;

motor_info_t motor_yaw_info;
uint16_t can_cnt;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  if(hcan->Instance == CAN1)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  switch(rx_header.StdId)
	{
	  case 0x205:
	{
    motor_yaw_info.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_yaw_info.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_yaw_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_yaw_info.temp           =   rx_data[6];
		break;
	}
	}
	
//	printf("机械角度: %d\r\n", motor_yaw_info.rotor_angle);
//    printf("转速: %d\r\n",rx_data[2]<<8|rx_data[3]);
//    printf("扭矩电流: %x\r\n",rx_data[4]<<8|rx_data[5]);
//    printf("温度: %d\r\n",rx_data[6]);
//	
  }
}

void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1)
{
  CAN_TxHeaderTypeDef tx_header;
  uint16_t   tx_data[8] = {0};
  
  tx_header.StdId = 0x1FF;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
	 
//  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
   
  if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*)tx_data,(uint32_t*)CAN_TX_MAILBOX0))
    {
//      printf("CAN1 Send data Err!!\r\n");
    }
    else
    {
//      printf("CAN1 Send data Success!!\r\n");
    }
}

//16位掩码过滤器
uint8_t bsp_can1_filter_config_16(void)
{
    //初始化筛选器
    CAN_FilterTypeDef filter = {0};
    //筛选器编号
    filter.FilterBank = 0;
    //存储FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    //ID组1
//    uint16_t  msk = (0x721 ^ (~0x722));
    //掩码
    filter.FilterIdLow =0x000;
    //ID
    filter.FilterMaskIdLow = 0x000;
    
//    //ID组2
//    uint16_t  msk1 = 0;
    filter.FilterIdHigh = 0x000;
    filter.FilterMaskIdHigh = 0x000;
    
    //是否使能当前筛选器
    filter.FilterActivation = ENABLE;
    //筛选器模式     掩码模式
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    //筛选器长度     16位
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    //配置筛选器
	HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
 //CAN_IT_RX_FIFO0_MSG_PENDING是为FIFO0配置挂起中断，有消息传至该缓存区则触发中断回调函数

     //过滤器配置失败
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    return 1;

}
                       
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    uint16_t i = 0;
//    uint8_t buf[8] = {0};
//    CAN_RxHeaderTypeDef rxHeader = {0};
//    if(hcan->Instance == CAN1)
//    {
//	printf("Recv via STM32F429 Interrupt\r\n");
//	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, buf);
//	printf("Recv Data: ");
//	for(i = 0; i < rxHeader.DLC; i++)
//               {
//                 printf("%x ",buf[i]);
//               }
//	printf("\r\n");
//    }
//}


                       

													 
//配置CAN过滤器
uint8_t bsp_can1_filter_config(void)
{
    //初始化筛选器
    CAN_FilterTypeDef filter = {0};
    //筛选器编号
    filter.FilterBank = 0;
    //存储FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    //筛选标准ID高位
    filter.FilterIdHigh = 0;
    //筛选标准ID低位
    filter.FilterIdLow = 0;
    //标准ID掩码高位
    filter.FilterMaskIdHigh = 0;
    //标准ID掩码低位
    filter.FilterMaskIdLow = 0;
    //是否使能当前筛选器
    filter.FilterActivation = ENABLE;
    //筛选器模式     掩码模式
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    //筛选器长度     32位
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    //配置筛选器
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    //过滤器配置失败
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    else
    {
      printf("CAN1 ConfigFilter SUCCESS!!\r\n");
    }
    return 1;

}	

//列表模式过滤器
uint8_t bsp_can1_filter_config_list(void)
{
   //初始化筛选器
	CAN_FilterTypeDef filter ={0};
	//筛选器变编号
  filter.FilterBank =0;
	//选择Fifo0
		filter.FilterFIFOAssignment = CAN_FilterFIFO0;
		filter.FilterIdHigh = 0x723<<5;
		filter.FilterIdLow = 0;
		
		filter.FilterMaskIdHigh = 0x724<<5;
		filter.FilterMaskIdLow = 0;
	
	  //筛选器使能
    filter.FilterActivation  =ENABLE;
	  //筛选器模式  列表
	  filter.FilterMode =CAN_FILTERMODE_IDLIST;
	  //筛选器长度  32
	  filter.FilterScale =CAN_FILTERSCALE_32BIT;
	  //配置筛选器
	  HAL_CAN_ConfigFilter(&hcan1,&filter);
	   //过滤器配置失败
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    return 1;

	
}



uint8_t bsp_can1_filter_config_16_list(void)
{
  //初始化筛选器
    CAN_FilterTypeDef filter = {0};
    //筛选器编号
    filter.FilterBank = 0;
    //存储FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    //ID1
    filter.FilterIdLow = (0x729 << 5) & 0xFFFF;
    //ID2
    filter.FilterMaskIdLow = (0x741 << 5) & 0xFFFF;
    //ID3
    filter.FilterIdHigh = (0x755 << 5) & 0xFFFF;
    //ID4
    filter.FilterMaskIdHigh = (0x769 << 5) & 0xFFFF;
    //是否使能当前筛选器
    filter.FilterActivation = ENABLE;
    //筛选器模式     掩码模式
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    //筛选器长度     16位
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    //配置筛选器
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    return 1;

}

//配置CAN过滤器
//uint8_t bsp_can2_filter_config(void)
//{
//   //初始化筛选器
//    CAN_FilterTypeDef filter = {0};
//    //筛选器编号
//    filter.FilterBank = 14;
//    //存储FIFO    FIFO0
//    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    //筛选标准ID高位
//    //filter.FilterIdHigh = 0x702 << 5;
//    filter.FilterIdHigh = 0;
//    //扩展ID高位
//    //filter.FilterIdHigh = (0x1839f101 >> 13) & 0xFFFF;
//    //筛选标准ID低位
//    filter.FilterIdLow = 0;
//    //扩展ID低位
//    //filter.FilterIdLow = ((0x1839f101 << 3)| CAN_ID_EXT) & 0xFFFF;
//    //筛选ID掩码高位          存放寄存器的高16位
//    //filter.FilterMaskIdHigh = 0xFFFFFFFF;
//    //计算掩码
//    //uint16_t  msk = (0x701 ^ (~0x702));
//    //uint16_t  msk = (0x1839f101 ^ (~0x1839f102));
//    //标准ID掩码高位
//    //filter.FilterMaskIdHigh = msk << 5;
//    filter.FilterMaskIdHigh = 0;
//    //扩展ID掩码低位
//    //filter.FilterMaskIdHigh = (msk >> 16) & 0xFFFF;
//    //标准ID掩码低位
//    filter.FilterMaskIdLow = 0;
//    //扩展ID掩码低位
//    //filter.FilterMaskIdLow = msk & 0xFFFF;
//    //是否使能当前筛选器
//    filter.FilterActivation = ENABLE;
//    //筛选器模式     掩码模式
//    filter.FilterMode = CAN_FILTERMODE_IDMASK;
//    //筛选器长度     32位
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;
//    //配置筛选器
//    HAL_CAN_ConfigFilter(&hcan2, &filter);
//    //过滤器配置失败
//    if(HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
//    {
//      printf("CAN2 ConfigFilter Fail!!\r\n");
//    } 
//		else{
//		  printf("CAN2 ConfigFilter SUCCESS!!\r\n");
//		}
//    return 1;


//}
//	

	
//can1 接收数据
void Can1Receive(void)
{
	 //配置CAN过滤器
  bsp_can1_filter_config_16();
  //配置32位列表模式
  //bsp_can1_filter_config_list();
  //配置16位掩码模式
  //bsp_can1_filter_config_16();
  //16位列表模式
  //bsp_can1_filter_config_16_list();
  //初始话CAN接收结构体
  CAN_RxHeaderTypeDef  rceStu = {0};
  uint8_t data[8] = {0};
  //判断FIFO中是否有数据
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0) != 0)
  {
    printf("CAN1 have data!!\r\n");
    //接收CAN数据信息
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rceStu, data);
    printf("rceStu.DLC: %d\r\n",rceStu.DLC);
    printf("rceStu.ExtId: %d\r\n",rceStu.ExtId);
    printf("rceStu.StdId: %x\r\n",rceStu.StdId);
    printf("rceStu.Timestamp: %d\r\n",rceStu.Timestamp);
    for(uint8_t i = 0;i<rceStu.DLC;i++)
    {
      printf(" %x",data[i]);
    }
     printf("\r\n");
  }
  else
  {
    printf("No CAN1 INFO!\r\n");
  }

}


//can2 接收数据
//void Can2Receive(void)
//{
//    //配置CAN过滤器
//  bsp_can2_filter_config();
//  //初始话CAN接收结构体
//  CAN_RxHeaderTypeDef  rceStu = {0};
//  uint8_t data[8] = {0};
//  //判断FIFO中是否有数据
//  if(HAL_CAN_GetRxFifoFillLevel(&hcan2,CAN_RX_FIFO0) != 0)
//  {
//    printf("CAN2 have data!!\r\n");
//    //接收CAN数据信息
//    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rceStu, data);
//    //打印有效数据长度
//    printf("rceStu.DLC: %d\r\n",rceStu.DLC);
//    //打印扩展ID
//   printf("rceStu.ExtId: %d\r\n",rceStu.ExtId);
//    //打印标准ID
//    printf("rceStu.StdId: %x\r\n",rceStu.StdId);
//    //打印接收时间
//    printf("rceStu.Timestamp: %d\r\n",rceStu.Timestamp);
//    for(uint8_t i = 0;i<rceStu.DLC;i++)
//    {
//      printf(" %x",data[i]);
//    }
//     printf("\r\n");
//  }
//  else
//  {
//    printf("No CAN2 INFO!\r\n");
//  }

//}

 //can1 发送数据
void can1_transmit(uint32_t id_type,uint32_t basic_id,uint32_t ex_id,uint8_t *data,uint32_t data_len)
{   

	 
   //初始化发送结构体
	  CAN_TxHeaderTypeDef send_msg_hdr ={0};
		int8_t index = 0;
    uint32_t msg_box = CAN_TX_MAILBOX0;   
    uint8_t send_buf[8] = {0};
		//标识符ID
    send_msg_hdr.StdId = basic_id;
    //扩展标识符ID
    send_msg_hdr.ExtId = 0;
    //扩展标志（传输格式：标准帧/扩展帧）
    //send_msg_hdr.IDE = CAN_ID_EXT;
    send_msg_hdr.IDE = CAN_ID_STD;  
    //远程控制标志  ：传输数据格式：数据帧/遥控帧
    send_msg_hdr.RTR = CAN_RTR_DATA;
    //有效数据位长度
    send_msg_hdr.DLC = data_len;
    //是否使能捕获时间戳计数器
    send_msg_hdr.TransmitGlobalTime = DISABLE;
    
    for(index = 0; index < data_len; index++)
        {
          send_buf[index] = data[index];
        }
    if((HAL_CAN_AddTxMessage(&hcan1,&send_msg_hdr,send_buf,&msg_box))!= HAL_OK)
    {
      printf("CAN1 Send data Err!!\r\n");
    }
    else
    {
      printf("CAN1 Send data Success!!\r\n");
    }
                          
}
	

 //can2 发送数据
//void can2_transmit(uint32_t id_type,uint32_t basic_id,uint32_t ex_id,uint8_t *data,uint32_t data_len)
//{
//   //初始化发送结构体
//	  CAN_TxHeaderTypeDef send_msg_hdr ={0};
//		int8_t index = 0;
//    uint32_t msg_box = 0;
//    uint8_t send_buf[8] = {0};
//		//标识符ID
//    send_msg_hdr.StdId = basic_id;
//    //扩展标识符ID
//    send_msg_hdr.ExtId = 0x1839f101;
//    //扩展标志
//    //send_msg_hdr.IDE = CAN_ID_EXT;
//    send_msg_hdr.IDE = CAN_ID_STD;
//    //远程控制标志
//    send_msg_hdr.RTR = CAN_RTR_DATA;
//    //有效数据位长度
//    send_msg_hdr.DLC = data_len;
//    //是否使能捕获时间戳计数器
//    send_msg_hdr.TransmitGlobalTime = DISABLE;
//    for(index = 0; index < data_len; index++)
//        {
//          send_buf[index] = data[index];
//        }
//    if((HAL_CAN_AddTxMessage(&hcan2,&send_msg_hdr,send_buf,&msg_box))!= HAL_OK)
//    {
//      printf("CAN2 Send data Err!!\r\n");
//    }
//    else
//    {
//      printf("CAN2 Send data Success!!\r\n");
//    }
//                          
//}
	

//void Can1AndCan2CommunicationTest(void)
//{
//  //CAN发送数据
//    uint8_t data[10] = {0x66,0x64,0x55,0x33,0x66,0x33,0x44,0x55};
//    //发送CAN数据信息
//    can1_transmit(CAN_ID_STD,0x729,2,data,8);
//    //发送CAN数据信息
//   can2_transmit(CAN_ID_STD,0x723,2,data,8);
//    //CAN1接收数据
//    Can1Receive();
//    //CAN2接收数据
//    Can1Receive();
//}


//can1 接收数据
void Can1Receive_motor(void)
{
	 //配置CAN过滤器
  bsp_can1_filter_config();
  //配置32位列表模式
  //bsp_can1_filter_config_list();
  //配置16位掩码模式
  //bsp_can1_filter_config_16();
  //16位列表模式
  //bsp_can1_filter_config_16_list();
  //初始话CAN接收结构体
  CAN_RxHeaderTypeDef  rceStu = {0};
  uint8_t data[8] = {0};
  //判断FIFO中是否有数据
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0) != 0)
  {
    printf("CAN1 have data!!\r\n");
    //接收CAN数据信息
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rceStu, data);
    printf("rceStu.DLC: %d\r\n",rceStu.DLC);
    printf("rceStu.ExtId: %d\r\n",rceStu.ExtId);
    printf("rceStu.StdId: %x\r\n",rceStu.StdId);
    printf("rceStu.Timestamp: %d\r\n",rceStu.Timestamp);
    for(uint8_t i = 0;i<rceStu.DLC;i++)
    {
      printf(" %x",data[i]);
    }
     printf("\r\n");
  }
  else
  {
    printf("No CAN1 INFO!\r\n");
  }

}
