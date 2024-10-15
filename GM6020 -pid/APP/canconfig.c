#include "can.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "canconfig.h"


//��׼CAN ID 
uint16_t StdIdArray[10] = {  0x7e0,0x7e1,0x7e2,0x7e3,0x7e4,
                          0x7e5,0x7e6,0x7e7,0x7e8,0x7e9};

//��չID
							
uint32_t ExtIdArray[10] = {0x1839f101,0x1839f102,0x1839f111,0x1839f107,0x1839f1f1,
                           0x183Af101,0x183Af102,0x183Af103,0x183Af104,0x183Af105};

	
                 

													 
//����CAN������
uint8_t bsp_can1_filter_config(void)
{
    //��ʼ��ɸѡ��
    CAN_FilterTypeDef filter = {0};
    //ɸѡ�����
    filter.FilterBank = 0;
    //�洢FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    //ɸѡ��׼ID��λ
    filter.FilterIdHigh = 0;
    //ɸѡ��׼ID��λ
    filter.FilterIdLow = 0;
    //��׼ID�����λ
    filter.FilterMaskIdHigh = 0;
    //��׼ID�����λ
    filter.FilterMaskIdLow = 0;
    //�Ƿ�ʹ�ܵ�ǰɸѡ��
    filter.FilterActivation = ENABLE;
    //ɸѡ��ģʽ     ����ģʽ
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    //ɸѡ������     32λ
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
	    
    //����ɸѡ��
    HAL_CAN_ConfigFilter(&hcan1, &filter);
	
    //����������ʧ��
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
//      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    else
    {
//      printf("CAN1 ConfigFilter SUCCESS!!\r\n");
    }
    return 1;

}	

//�б�ģʽ������
uint8_t bsp_can1_filter_config_list(void)
{
   //��ʼ��ɸѡ��
	CAN_FilterTypeDef filter ={0};
	//ɸѡ������
  filter.FilterBank =0;
	//ѡ��Fifo0
		filter.FilterFIFOAssignment = CAN_FilterFIFO0;
		filter.FilterIdHigh = 0x723<<5;
		filter.FilterIdLow = 0;
		
		filter.FilterMaskIdHigh = 0x724<<5;
		filter.FilterMaskIdLow = 0;
	
	  //ɸѡ��ʹ��
    filter.FilterActivation  =ENABLE;
	  //ɸѡ��ģʽ  �б�
	  filter.FilterMode =CAN_FILTERMODE_IDLIST;
	  //ɸѡ������  32
	  filter.FilterScale =CAN_FILTERSCALE_32BIT;
	  //����ɸѡ��
	  HAL_CAN_ConfigFilter(&hcan1,&filter);
	   //����������ʧ��
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    return 1;

	
}
//16λ���������
uint8_t bsp_can1_filter_config_16(void)
{
    //��ʼ��ɸѡ��
    CAN_FilterTypeDef filter = {0};
    //ɸѡ�����
    filter.FilterBank = 0;
    //�洢FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    //ID��1
//    uint16_t  msk = (0x721 ^ (~0x722));
    //����
    filter.FilterIdLow =0x000;
    //ID
    filter.FilterMaskIdLow = 0x000;
    
//    //ID��2
//    uint16_t  msk1 = 0;
    filter.FilterIdHigh = 0x000;
    filter.FilterMaskIdHigh = 0x000;
    
    //�Ƿ�ʹ�ܵ�ǰɸѡ��
    filter.FilterActivation = ENABLE;
    //ɸѡ��ģʽ     ����ģʽ
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    //ɸѡ������     16λ
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    //����ɸѡ��
    HAL_CAN_ConfigFilter(&hcan1, &filter);
     //����������ʧ��
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    return 1;

}


uint8_t bsp_can1_filter_config_16_list(void)
{
  //��ʼ��ɸѡ��
    CAN_FilterTypeDef filter = {0};
    //ɸѡ�����
    filter.FilterBank = 0;
    //�洢FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    //ID1
    filter.FilterIdLow = (0x729 << 5) & 0xFFFF;
    //ID2
    filter.FilterMaskIdLow = (0x741 << 5) & 0xFFFF;
    //ID3
    filter.FilterIdHigh = (0x755 << 5) & 0xFFFF;
    //ID4
    filter.FilterMaskIdHigh = (0x769 << 5) & 0xFFFF;
    //�Ƿ�ʹ�ܵ�ǰɸѡ��
    filter.FilterActivation = ENABLE;
    //ɸѡ��ģʽ     ����ģʽ
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    //ɸѡ������     16λ
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    //����ɸѡ��
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
      printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    return 1;

}

//����CAN������
uint8_t bsp_can2_filter_config(void)
{
   //��ʼ��ɸѡ��
    CAN_FilterTypeDef filter = {0};
    //ɸѡ�����
    filter.FilterBank = 14;
    //�洢FIFO    FIFO0
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    //ɸѡ��׼ID��λ
    //filter.FilterIdHigh = 0x702 << 5;
    filter.FilterIdHigh = 0;
    //��չID��λ
    //filter.FilterIdHigh = (0x1839f101 >> 13) & 0xFFFF;
    //ɸѡ��׼ID��λ
    filter.FilterIdLow = 0;
    //��չID��λ
    //filter.FilterIdLow = ((0x1839f101 << 3)| CAN_ID_EXT) & 0xFFFF;
    //ɸѡID�����λ          ��żĴ����ĸ�16λ
    //filter.FilterMaskIdHigh = 0xFFFFFFFF;
    //��������
    //uint16_t  msk = (0x701 ^ (~0x702));
    //uint16_t  msk = (0x1839f101 ^ (~0x1839f102));
    //��׼ID�����λ
    //filter.FilterMaskIdHigh = msk << 5;
    filter.FilterMaskIdHigh = 0;
    //��չID�����λ
    //filter.FilterMaskIdHigh = (msk >> 16) & 0xFFFF;
    //��׼ID�����λ
    filter.FilterMaskIdLow = 0;
    //��չID�����λ
    //filter.FilterMaskIdLow = msk & 0xFFFF;
    //�Ƿ�ʹ�ܵ�ǰɸѡ��
    filter.FilterActivation = ENABLE;
    //ɸѡ��ģʽ     ����ģʽ
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    //ɸѡ������     32λ
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    //����ɸѡ��
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    //����������ʧ��
    if(HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
    {
      printf("CAN2 ConfigFilter Fail!!\r\n");
    } 
		else{
		  printf("CAN2 ConfigFilter SUCCESS!!\r\n");
		}
    return 1;


}
	
	
//can1 ��������
void Can1Receive(void)
{
	 //����CAN������
  bsp_can1_filter_config_16();
  //����32λ�б�ģʽ
  //bsp_can1_filter_config_list();
  //����16λ����ģʽ
  //bsp_can1_filter_config_16();
  //16λ�б�ģʽ
  //bsp_can1_filter_config_16_list();
  //��ʼ��CAN���սṹ��
  CAN_RxHeaderTypeDef  rceStu = {0};
  uint8_t data[8] = {0};
  //�ж�FIFO���Ƿ�������
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0) != 0)
  {
    printf("CAN1 have data!!\r\n");
    //����CAN������Ϣ
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


//can2 ��������
void Can2Receive(void)
{
    //����CAN������
  bsp_can2_filter_config();
  //��ʼ��CAN���սṹ��
  CAN_RxHeaderTypeDef  rceStu = {0};
  uint8_t data[8] = {0};
  //�ж�FIFO���Ƿ�������
  if(HAL_CAN_GetRxFifoFillLevel(&hcan2,CAN_RX_FIFO0) != 0)
  {
    printf("CAN2 have data!!\r\n");
    //����CAN������Ϣ
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rceStu, data);
    //��ӡ��Ч���ݳ���
    printf("rceStu.DLC: %d\r\n",rceStu.DLC);
    //��ӡ��չID
   printf("rceStu.ExtId: %d\r\n",rceStu.ExtId);
    //��ӡ��׼ID
    printf("rceStu.StdId: %x\r\n",rceStu.StdId);
    //��ӡ����ʱ��
    printf("rceStu.Timestamp: %d\r\n",rceStu.Timestamp);
    for(uint8_t i = 0;i<rceStu.DLC;i++)
    {
      printf(" %x",data[i]);
    }
     printf("\r\n");
  }
  else
  {
    printf("No CAN2 INFO!\r\n");
  }

}

 //can1 ��������
void can1_transmit(uint32_t id_type,uint32_t basic_id,uint32_t ex_id,uint8_t *data,uint32_t data_len)
{   

	 
   //��ʼ�����ͽṹ��
	  CAN_TxHeaderTypeDef send_msg_hdr ={0};
		int8_t index = 0;
    uint32_t msg_box = CAN_TX_MAILBOX0;   
    uint8_t send_buf[8] = {0};
		//��ʶ��ID
    send_msg_hdr.StdId = basic_id;
    //��չ��ʶ��ID
    send_msg_hdr.ExtId = 0;
    //��չ��־�������ʽ����׼֡/��չ֡��
    //send_msg_hdr.IDE = CAN_ID_EXT;
    send_msg_hdr.IDE = CAN_ID_STD;  
    //Զ�̿��Ʊ�־  ���������ݸ�ʽ������֡/ң��֡
    send_msg_hdr.RTR = CAN_RTR_DATA;
    //��Ч����λ����
    send_msg_hdr.DLC = data_len;
    //�Ƿ�ʹ�ܲ���ʱ���������
    send_msg_hdr.TransmitGlobalTime = DISABLE;
    
    for(index = 0; index < data_len; index++)
        {
          send_buf[index] = data[index];
        }
				HAL_CAN_AddTxMessage(&hcan1,&send_msg_hdr,send_buf,&msg_box);
    if((HAL_CAN_AddTxMessage(&hcan1,&send_msg_hdr,send_buf,&msg_box))!= HAL_OK)
    {
//      printf("CAN1 Send data Err!!\r\n");
    }
    else
    {
//      printf("CAN1 Send data Success!!\r\n");
    }
                          
}
	

 //can2 ��������
void can2_transmit(uint32_t id_type,uint32_t basic_id,uint32_t ex_id,uint8_t *data,uint32_t data_len)
{
   //��ʼ�����ͽṹ��
	  CAN_TxHeaderTypeDef send_msg_hdr ={0};
		int8_t index = 0;
    uint32_t msg_box = 0;
    uint8_t send_buf[8] = {0};
		//��ʶ��ID
    send_msg_hdr.StdId = basic_id;
    //��չ��ʶ��ID
    send_msg_hdr.ExtId = 0x1839f101;
    //��չ��־
    //send_msg_hdr.IDE = CAN_ID_EXT;
    send_msg_hdr.IDE = CAN_ID_STD;
    //Զ�̿��Ʊ�־
    send_msg_hdr.RTR = CAN_RTR_DATA;
    //��Ч����λ����
    send_msg_hdr.DLC = data_len;
    //�Ƿ�ʹ�ܲ���ʱ���������
    send_msg_hdr.TransmitGlobalTime = DISABLE;
    for(index = 0; index < data_len; index++)
        {
          send_buf[index] = data[index];
        }
    if((HAL_CAN_AddTxMessage(&hcan2,&send_msg_hdr,send_buf,&msg_box))!= HAL_OK)
    {
      printf("CAN2 Send data Err!!\r\n");
    }
    else
    {
      printf("CAN2 Send data Success!!\r\n");
    }
                          
}
	

void Can1AndCan2CommunicationTest(void)
{
  //CAN��������
    uint8_t data[10] = {0x66,0x64,0x55,0x33,0x66,0x33,0x44,0x55};
    //����CAN������Ϣ
    can1_transmit(CAN_ID_STD,0x729,2,data,8);
    //����CAN������Ϣ
   can2_transmit(CAN_ID_STD,0x723,2,data,8);
    //CAN1��������
    Can1Receive();
    //CAN2��������
    Can1Receive();
}
  uint8_t data[8] = {0};  

 motor_info_t motor_yaw_info; 
//can1 ��������
void Can1Receive_motor(void)
{   
	//��ʼ��CAN���սṹ��
  CAN_RxHeaderTypeDef  rceStu = {0};
  uint8_t data[8] = {0};  
	//�ṹ���������
   motor_info_t motor_yaw_info;
	 motor_yaw_info.rotor_angle =(data[0] <<8  | data[1]);
	 motor_yaw_info.rotor_speed=((data[2] <<8    | data[3]));
	 motor_yaw_info.torque_current=((data[4] <<8 | data[5]));
	 motor_yaw_info.temp = data[6];
	
	 //����CAN������
  bsp_can1_filter_config();
  //����32λ�б�ģʽ
  //bsp_can1_filter_config_list();
  //����16λ����ģʽ
  //bsp_can1_filter_config_16();
  //16λ�б�ģʽ
  //bsp_can1_filter_config_16_list();
  
  //�ж�FIFO���Ƿ�������
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0) != 0)
  {
    printf("CAN1 have data!!\r\n");
    //����CAN������Ϣ
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rceStu, data);
   
    for(uint8_t i = 0;i<rceStu.DLC;i++)
    {
      printf(" %x",data[i]);
    }
     printf("\r\n");
	  printf("��е�Ƕ�: %d\r\n",data[0]<<8|data[1]);	//     ��ӡ�����¶ȶ����������
    printf("ת��: %d\r\n",data[2]<<8|data[3]);
    printf("Ť�ص���: %x\r\n",data[4]<<8|data[5]);
    printf("�¶�: %d\r\n",data[6]);
  }
  else
  {
    printf("No CAN1 INFO!\r\n");
  }

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{ 
	
//	bsp_can1_filter_config();
  CAN_RxHeaderTypeDef  rceStu = {0};
	rceStu.DLC=0x08;
	rceStu.FilterMatchIndex=0xff;
	rceStu.IDE=CAN_ID_STD;
	rceStu.RTR=CAN_RTR_DATA;
	rceStu.StdId=0x204;
  uint8_t rx_data[8] = {0};
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0) != 0)
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rceStu, rx_data);
	}
    motor_yaw_info.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_yaw_info.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_yaw_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_yaw_info.temp           =   rx_data[6];
//		printf("��е�Ƕ�: %d\r\n",rx_data[0]<<8|rx_data[1]);	//     ��ӡ�����¶ȶ����������
//    printf("ת��: %d\r\n",rx_data[2]<<8|rx_data[3]);
//    printf("Ť�ص���: %x\r\n",rx_data[4]<<8|rx_data[5]);
//    printf("�¶�: %d\r\n",rx_data[6]);
	//	break;
//	}
	//}
  }




//void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1)
//{
//  CAN_TxHeaderTypeDef tx_header;
//  uint8_t             tx_data[8] = {0};
//    
//  tx_header.StdId = 0x1FF;
//  tx_header.IDE   = CAN_ID_STD;
//  tx_header.RTR   = CAN_RTR_DATA;
//  tx_header.DLC   = 8;
// 
//  tx_data[0] = (v1>>8)&0xff;
//  tx_data[1] =    (v1)&0xff;
//	 
//  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//}

