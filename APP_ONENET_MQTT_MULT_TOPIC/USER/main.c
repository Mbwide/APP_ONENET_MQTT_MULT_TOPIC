/*------------------------------------------------------*/
/*                                                      */
/*            ����main��������ں���Դ�ļ�               */
/*                                                      */
/*------------------------------------------------------*/

#include <stdlib.h>
#include "sys.h"
#include "delay.h"	     //������Ҫ��ͷ�ļ�
#include "usart1.h"      //������Ҫ��ͷ�ļ�
#include "usart2.h"      //������Ҫ��ͷ�ļ�
#include "timer3.h"      //������Ҫ��ͷ�ļ�
#include "timer4.h"      //������Ҫ��ͷ�ļ�

#include "FreeRTOS.h"	 //FreeRTOS����ͷ�ļ�
#include "semphr.h" 	 //�ź���
#include "queue.h"		 //����
#include "event_groups.h"//�¼���־��

#include "wifi.h"	     //������Ҫ��ͷ�ļ�
#include "mqtt.h"        //������Ҫ��ͷ�ļ�
#include "control.h"     //������Ҫ��ͷ�ļ� ����ģ��������ݷ��͸�������
#include "led.h"	     //������Ҫ��ͷ�ļ� LED
#include "dht11.h"       //������Ҫ��ͷ�ļ� ������ʪ��
#include "bh1750.h"      //������Ҫ��ͷ�ļ� ���մ�����

/*-------------------------------------------------------------*/
/*          	WIFI������ONENET���ã����ã�			      	   */
/*-------------------------------------------------------------*/
const char SSID[] 			 = "PPP";           //·��������
const char PASS[] 			 = "qaz123qaz";     //·��������

const char PRODUCTID[] 	     = "394499";  	    //��ƷID
const char DEVICEID []	     = "661126800";     //�豸ID  
const char AUTHENTICATION[]  = "123456";        //��Ȩ��Ϣ  
const char SERVER_IP[]	     = "183.230.40.39"; //��ŷ�����IP�����������øģ�
const int  SERVER_PORT 		 = 6002;		    //��ŷ������Ķ˿ںţ����øģ�

/*-------------------------------------------------------------*/
/*          			topic���⣨���ã�			      	   */
/*-------------------------------------------------------------*/
const char TEMP_TOPIC[]   = "temp";  //�¶�topic
const char HUM_TOPIC[] 	  = "hum";	 //ʪ��topic
const char SUN_TOPIC[]    = "sun";	 //����topic

const char APP_TOPIC[]    = "app";   //Ҫ����app������tpoic
const char LOCAL_TOPIC[]  = "local"; //�������ؿ���ģ��״̬topic
/*-------------------------------------------------------------*/
/*          ���������Լ�����ģ���ʼ״̬���ã����ã�		   	   */
/*-------------------------------------------------------------*/

/* state[0]:LED1״̬��O(������)Ϊ������XΪ�ر�
 * state[1]:LED2����״̬��OΪ���������ִ�У���XΪ���𣨹رգ�����
 * state[2] - state[4]: ����
 */
char state[5] = "XOXXX";			  		//�����͵��豸״̬����

const char *CMD_APP_GET_STATE  = "DEVSTA";  //APP���͵��豸״̬��ȡ����

const char *CMD_LED1ON  = "LED1ON";   		//LED1������
const char *CMD_LED1OFF = "LED1OFF";  		//LED1�ر�����

const char *CMD_LED2ON  = "LED2ON";   		//LED2������������
const char *CMD_LED2OFF = "LED2OFF";  		//LED2�����������


/*-------------------------------------------------------------*/
/*               freerto����ͨ�ſ��ƣ��̶���			      	   */
/*-------------------------------------------------------------*/

/*	��ֵ�ź������                         
 *	���ã����ڿ���MQTT����崦��������MQTT���ݽ��շ��ͻ��崦�������з���
 *		  ����������յ���������ʱ�������ź���		 
 */
SemaphoreHandle_t BinarySemaphore;
	
/*	�¼���־��                         
 *	���ã���־WIFI���ӣ�PING����������ģʽ����wifi�Ƿ��������ӣ��Ƿ������ݣ��������Ƿ����� 
 *  ���壺1.�¼���־��λ1Ϊ0��λ0Ϊ1ʱ����0x03��0000 0001����wifi������������ʱλ0��λ1����ʱconnect���Ļ�δ���͡� 
 *		  2.�¼���־��λ1Ϊ1��λ0Ϊ1ʱ����0x03��0000 0011����connect���ķ��ͣ��������ӳɹ�����ʱλ1��λ1��PING��
 *			��������30s����ģʽ�������������������ݿ�ʼ�ϴ����豸Զ�̿��ƣ�LED���ƣ����ܿ����� 
 */
EventGroupHandle_t Event_Handle = NULL;      //�¼���־�飨λ0��WIFI����״̬ λ1��PING������2S���ٷ���ģʽ��
const int WIFI_CONECT = (0x01 << 0);         //�����¼������λ 0������������ģʽ��ֵ1��ʾ�Ѿ����ӣ�0��ʾδ����
const int PING_MODE   = (0x01 << 1);         //�����¼������λ 1��PING����������ģʽ��1��ʾ����30S����ģʽ��0��ʾδ�������ͻ���2S���ٷ���ģʽ

/*	���������ݷ�����Ϣ����                         
 *	���ã��������������ݷ��͵���������Ϣ����  
 */
QueueHandle_t Message_Queue;		 		 //��Ϣ���о��  
const UBaseType_t MESSAGE_DATA_TX_NUM = 10;	 //��Ϣ���������Ϣ��Ŀ  
const UBaseType_t MESSAGE_DATA_TX_LEN = 30;  //��Ϣ���е�Ԫ��С����λΪ�ֽ�  

/*-------------------------------------------------------------*/
/*               ������������������1�����ã�		      	   */
/*-------------------------------------------------------------*/
//��ʼ����
TaskHandle_t StartTask_Handler;
void my_start_task(void *pvParameters);
//LED���� 
TaskHandle_t Led2_Task_Handler;
void my_led2_task(void *pvParameters);
//DHT11���� ��ʪ�ȴ�����
TaskHandle_t DHT11_Task_Handler;
void my_dht11_task(void *pvParameters);
//SUN���񣬹��մ�����
TaskHandle_t SUN_Task_Handler;
void my_sun_task(void *pvParameters);
//MQTT����崦������
TaskHandle_t MQTT_Cmd_Task_Handler;
void my_mqtt_buffer_cmd_task(void *pvParameters);

//�����ջ��С��������
//TaskHandle_t STACK_Task_Handler;
//void stack_task(void *pvParameters);

/*-------------------------------------------------------------*/
/*               ������������������2���̶���		      	   */
/*-------------------------------------------------------------*/
//WIFI����
TaskHandle_t WIFI_Task_Handler;
void wifi_task(void *pvParameters);
//MQTT���ݽ��շ��ͻ��崦������
TaskHandle_t MQTT_RxTx_Task_Handler;
void mqtt_buffer_rx_tx_task(void *pvParameters);
//���������ݴ������񣬴�������͵Ĵ��������ݣ�.����MQTT���ݷ��ͻ�����
TaskHandle_t DATA_TX_Task_Handler;
void data_tx_to_buffer_task(void *pvParameters);

/*---------------------------------------------------------------*/
/*��������int main()                                             */
/*��  �ܣ�������							                         */
/*		  1.��ʼ��������ģ��  				     				 */
/*		  2.������ʼ�����ڿ�ʼ�����ﴴ��������������           */
/*		  3.�����������				       			 		     */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	delay_init();	       //��ʱ������ʼ��
	usart1_init(115200);   //����1���ܳ�ʼ����������115200���봮������ͨ��		
	usart2_init(115200);   //����2���ܳ�ʼ����������115200��wifiͨ��	
	tim4_init(500,7200);   //TIM4��ʼ������ʱʱ�� 500*7200*1000/72000000 = 50ms	
	led_init();		  	   //��ʼ��LED
	dht11_init();  		   //��ʼ��DHT11 ��ʪ��
	iic_by30_init();       //��ʼ��IIC�ӿ� ����ǿ��
	
	wifi_reset_io_init();  //��ʼ��esp8266
	IoT_parameter_init();  //��ʼ��OneNETƽ̨MQTT�������Ĳ���	
	
	//������ʼ����
	xTaskCreate((TaskFunction_t	) my_start_task,		//������
			    (const char* 	)"my_start_task",		//��������
				(uint16_t 		) 128,				  	//�����ջ��С
				(void* 		  	) NULL,				 	//���ݸ��������Ĳ���
				(UBaseType_t 	) 1, 				  	//�������ȼ�
				(TaskHandle_t*  ) &StartTask_Handler);	//������ƿ� 
			
	vTaskStartScheduler();  							//�����������
}

/*---------------------------------------------------------------*/
/*��������void my_start_task(void *pvParameters)                 */
/*��  �ܣ���ʼ�������ã�							             */
/*		  1.�����ź�������Ϣ���е�����ͨ�ŷ�ʽ   				     */
/*		  2.������������       			 						 */
/*		  3.ɾ������       			 		    				 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
void my_start_task(void *pvParameters)
{
	taskENTER_CRITICAL(); //�����ٽ���
	
	//������ֵ�ź���
	BinarySemaphore = xSemaphoreCreateBinary();	
	//�¼���־�飬���ڱ�־wifi����״̬�Լ�ping����״̬
	Event_Handle = xEventGroupCreate(); 
	//������������Ϣ����Ϣ����
	Message_Queue = xQueueCreate(MESSAGE_DATA_TX_NUM, MESSAGE_DATA_TX_LEN); 
	
	//���񴴽�����������1.������ 2.�������� 3.�����ջ��С 3.���ݸ��������Ĳ��� 4.�������ȼ�����ֵԽ�����ȼ�Խ�� 8>2�� 5.������ƿ飨�����
	//����WIFI����
    xTaskCreate(wifi_task, 				"wifi_task", 				128, NULL, 8, &WIFI_Task_Handler); 			
	//����MQTT����崦������
    xTaskCreate(my_mqtt_buffer_cmd_task,"my_mqtt_buffer_cmd_task",  128, NULL, 7, &MQTT_Cmd_Task_Handler); 			
	//����MQTT���ݽ��շ��ͻ��崦������
    xTaskCreate(mqtt_buffer_rx_tx_task, "mqtt_buffer_rx_tx_task", 	200, NULL, 6, &MQTT_RxTx_Task_Handler); 
	//����led��������
	xTaskCreate(my_led2_task, 			"my_led2_task",				128, NULL, 4, &Led2_Task_Handler);  
    //����DHT11������ʪ�ȴ�����
    xTaskCreate(my_dht11_task, 			"my_dht11_task", 			128, NULL, 3, &DHT11_Task_Handler);
    //����SUN���񣬹��մ�����
    xTaskCreate(my_sun_task, 			"my_sun_task",        		128, NULL, 3, &SUN_Task_Handler);	
	//�������������ݴ������񣬴�������͵Ĵ��������ݣ�����MQTT���ݷ��ͻ�����
    xTaskCreate(data_tx_to_buffer_task, "data_tx_to_buffer_task", 	256, NULL, 2, &DATA_TX_Task_Handler); 

	//��ջ����
	//xTaskCreate(stack_task, 			"stack_task", 				256, NULL, 2, &STACK_Task_Handler); 
	
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

/*---------------------------------------------------------------*/
/*��������void my_mqtt_buffer_cmd_task(void *pvParameters)       */
/*��  �ܣ�MQTT����崦���������ã�							 */
/*		  1.MQTT�������������ִ����Ӧ����     				 */
/*		  2.������ִ�н�����͸�������       			 		 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ������ȡ����ֵ�ź���ʱִ�У��յ�APP��������ִ�У�    		 */
/*---------------------------------------------------------------*/
void my_mqtt_buffer_cmd_task(void *pvParameters)	
{
	while(1)
	{
		xSemaphoreTake(BinarySemaphore, portMAX_DELAY);	//��ȡ�ź�������ȡ���ź���������ִ�У������������̬���ȴ�ִ��
		while (MQTT_CMDOutPtr != MQTT_CMDInPtr)			//ѭ����������ָ��	
		{                            		       
			printf("����:%s\r\n", &MQTT_CMDOutPtr[2]);              	   
			
			if(!memcmp(&MQTT_CMDOutPtr[2], CMD_APP_GET_STATE, strlen(CMD_APP_GET_STATE))) //�豸״̬�ж�
			{    
				if (led1_state())   //led1״̬
					state[0] = 'O'; //����
				else
					state[0] = 'X';	//�ر�
				
				if (eTaskGetState(Led2_Task_Handler) <= eBlocked)  //led2��������״̬������̬������̬������̬Ϊled2������״̬������״̬Ϊ�ر�״̬������̬��ɾ����
					state[1] = 'O'; //����
				else
					state[1] = 'X'; //�ر�
			}
			else if(!memcmp(&MQTT_CMDOutPtr[2], CMD_LED1ON, strlen(CMD_LED1ON)))
			{                                              
				led1_on();  	    //LED1����
				state[0]  = 'O';
			}
			else if(!memcmp(&MQTT_CMDOutPtr[2], CMD_LED1OFF, strlen(CMD_LED1OFF)))
			{                                           
				led1_off(); 	   //LED1�ر�
				state[0]  = 'X';
			}
			else if(!memcmp(&MQTT_CMDOutPtr[2], CMD_LED2ON, strlen(CMD_LED2ON)))
			{                                            
				vTaskResume(Led2_Task_Handler);  //LED2�����ɹ���̬תΪ����̬��LED2��������
				state[1]  = 'O';
			}
			else if(!memcmp(&MQTT_CMDOutPtr[2], CMD_LED2OFF, strlen(CMD_LED2OFF)))
			{                                      
				vTaskSuspend(Led2_Task_Handler); //LED2�����ɾ���̬������̬��תΪ����̬��LED2�������ֹͣ��  
				state[1]  = 'X';
			}					
		
			else printf("δָ֪��\r\n");	
					
			send_device_state(LOCAL_TOPIC, state);  //���͸������������豸״̬��APP���գ�
			printf("devsta:%s\r\n", state);
			
			MQTT_CMDOutPtr += CBUFF_UNIT;			//ָ������
			if(MQTT_CMDOutPtr == MQTT_CMDEndPtr)    //���ָ�뵽������β����
			MQTT_CMDOutPtr = MQTT_CMDBuf[0];        //ָ���λ����������ͷ	
			delay_ms(10); 
		}
		delay_ms(100);	  
	}
}

/*---------------------------------------------------------------*/
/*��������void my_dht11_task(void *pvParameters)                 */
/*��  �ܣ�DHT11���� ��ʪ�ȴ����������ã�							 */
/*		  1.��⻷����ʪ������      							     */
/*		  2.��������ʪ�����ݷ��봫����������Ϣ����       			 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ���������������Լ�PING������30S����ģʽ�¼�����ʱִ�д�����*/
/*		  �����������   									     */
/*---------------------------------------------------------------*/
// ��topic�������ݷ��͵���Ϣ���У���Ϣ���м��з��͵�������(���ַ�ʽ)		
//1. sprintf(date_of_senor, "%d", temperature); //�������͸���������ģ�δ���ԣ�
//   send_data_to_queue_type(TEMP_TOPIC, date_of_senor); 
		
//2. send_data_to_queue_int(TEMP_TOPIC, date_of_senor); int��char���ַ������ַ�����ʹ��

void my_dht11_task(void *pvParameters)
{
	while(1)
	{
		char humidity;		   //����һ������������ʪ��ֵ
		char temperature;	   //����һ�������������¶�ֵ	
		//char date_of_senor[20] = {0};
		
		//�����������Լ�PING������30S����ģʽ�¼�����ʱִ�д����񣬷����������
		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
							(EventBits_t		)PING_MODE,
							(BaseType_t			)pdFALSE,				
							(BaseType_t			)pdTRUE,
							(TickType_t			)portMAX_DELAY);
		
		dht11_read_data(&temperature, &humidity);//��ȡ��ʪ��ֵ	
		
		//��topic�������ݷ��͵���Ϣ���У���Ϣ���м��з��͵�������
	
		send_data_to_queue_int(HUM_TOPIC, humidity);
		delay_ms(500);		//��һ�����ͼ��0.5s
		send_data_to_queue_int(TEMP_TOPIC, temperature);
		
		delay_ms(10 * 1000);//���10s + 0.5s�ɼ�һ������
	}
} 

/*---------------------------------------------------------------*/
/*��������void my_led2_task(void *pvParameters)                  */
/*��  �ܣ�LED�������ã�									     */
/*		  1.LED2����ִ��       							         */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ���������������Լ�ping������30S����ģʽ�¼�����ʱִ�д�����*/
/*		  �����������   									     */
/*---------------------------------------------------------------*/
void my_led2_task(void *pvParameters)
{
	while(1)
	{
		//�����������Լ�ping������30S����ģʽ�¼�����ʱִ�д����񣬷����������
		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
							(EventBits_t		)PING_MODE,
							(BaseType_t			)pdFALSE,				
							(BaseType_t			)pdTRUE,
							(TickType_t			)portMAX_DELAY);
		led2_on();
		delay_ms(500);	//��ʱ500ms
		led2_off();
		delay_ms(500);	//��ʱ500ms
	}
}

/*---------------------------------------------------------------*/
/*��������void my_sun_task(void *pvParameters)                   */
/*��  �ܣ�SUN���񣬹��մ����������ã�							 */
/*		  1.������ǿ��       							         */
/*		  2.������ǿ�����ݷ��봫����������Ϣ����       			 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ���������������Լ�PING������30S����ģʽ�¼�����ʱִ�д�����*/
/*		  �����������   									     */
/*---------------------------------------------------------------*/
void my_sun_task(void *pvParameters)
{
	while(1)
	{
		int sun_light;	//����һ���������������ǿ��
		
		//�����������Լ�ping������30S����ģʽ�¼�����ʱִ�д����񣬷����������
		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
							(EventBits_t		)PING_MODE,
							(BaseType_t			)pdFALSE,				
							(BaseType_t			)pdTRUE,
							(TickType_t			)portMAX_DELAY);	
	
		sun_light = get_sunlight_value();		
		//��topic�������ݷ��͵���Ϣ���У���Ϣ���м��з��͵�������
		send_data_to_queue_int(SUN_TOPIC, sun_light);
			
		delay_ms(10 * 1000);	    //��ʱ10s
	}
} 

/*---------------------------------------------------------------*/
/*��������void wifi_task(void *pvParameters)                     */
/*��  �ܣ�WIFI���񣨹̶���										 */
/*		  1.����wifi�Լ��Ʒ�����       							 */
/*		  2.��������        									     */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ����1.����������ǰ�رշ���ping���Ķ�ʱ��3������¼���־λ	 */
/*		  2.�����������ӣ��׳��¼���־�������Լ����������̬		 */
/*		  3.����������wifi�ѶϿ�������¼���־������ִ�б��������� */
/*			����	 											 */
/*---------------------------------------------------------------*/
void wifi_task(void *pvParameters)
{
	while(1)
	{ 
		printf("��Ҫ���ӷ�����\r\n");                 
		TIM_Cmd(TIM4, DISABLE);                       //�ر�TIM4 
		TIM_Cmd(TIM3, DISABLE);                       //�ر�TIM3
		xEventGroupClearBits(Event_Handle, PING_MODE);//�رշ���PING���Ķ�ʱ��3������¼���־λ
		WiFi_RxCounter = 0;                           //WiFi������������������                        
		memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);     //���WiFi���ջ����� 
		if(WiFi_Connect_IoTServer() == 0)			  //���WiFi�����Ʒ�������������0����ʾ��ȷ������if
		{   			     
			printf("����TCP���ӳɹ�\r\n");            
			WiFi_RxCounter = 0;                       //WiFi������������������                        
			memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //���WiFi���ջ����� 
			MQTT_Buff_Init();                         //��ʼ�����ͻ�����  
			
			xEventGroupSetBits(Event_Handle, WIFI_CONECT);  //�����������ӣ��׳��¼���־
			vTaskSuspend(NULL);	    						//�����������ӣ������Լ����������̬�������ɹ���תΪ����̬ʱ�������ִ����ȥ��
			xEventGroupClearBits(Event_Handle, WIFI_CONECT);//����������wifi�ѶϿ�������¼���־������ִ�б������������� 
			xEventGroupClearBits(Event_Handle, PING_MODE);  //�رշ���PING���Ķ�ʱ��3������¼���־λ
		}
		delay_ms(10);	    //��ʱ10ms
	}
}

/*---------------------------------------------------------------*/
/*��������void data_tx_to_buffer_task(void *pvParameters)        */
/*��  �ܣ�������������Ϣ���д������񣨹̶���			     		 */			
/*		  1.��������͵Ĵ��������ݣ�����MQTT���ݷ��ͻ�����         */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ���������������Լ�ping������30S����ģʽ�¼�����ʱִ�д�����*/
/*		  �����������   									     */
/*---------------------------------------------------------------*/
void data_tx_to_buffer_task(void *pvParameters)
{
	while(1)
	{			

		char data_buffer[100] = {0};//���ݰ�������,��ʼ��Ϊ0
		//�����������Լ�ping������30S����ģʽ�¼�����ʱִ�д����񣬷����������
		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
							(EventBits_t		)PING_MODE,
							(BaseType_t			)pdFALSE,				
							(BaseType_t			)pdTRUE,
							(TickType_t			)portMAX_DELAY);
		
		/* ����data_buff�Զ������ݰ���
		 * 	    topic:����
		 * ʾ����
		 *	    temp:21
		 */
		while (xQueueReceive(Message_Queue, data_buffer, portMAX_DELAY))
		{
			int  data_len = 0;			
			int  topic_len = 0;
			while(data_buffer[topic_len] != ':') //����topic����
			{
				topic_len++;
			}
			data_buffer[topic_len] = '\0'; 		 //Ϊ�˼���topic��С
			data_len = strlen(data_buffer + topic_len + 1);
			
			taskENTER_CRITICAL(); //�����ٽ�������ֹ�жϴ��
			MQTT_PublishQs0(data_buffer, data_buffer + topic_len + 1, data_len);//������ݣ�������������	
			taskEXIT_CRITICAL();  //�˳��ٽ���
			
			printf("topic:%s value:%s length:%d\r\n", data_buffer, data_buffer + topic_len + 1, data_len);
			delay_ms(500);		 //��ʱ500ms
		}	
		delay_ms(100);	 		 
	}
}

/*---------------------------------------------------------------*/
/*��������void mqtt_buffer_rx_tx_task(void *pvParameters)        */
/*��  �ܣ�MQTT���շ��ʹ������񣨹̶���							 */
/*		  1.�����ͻ���������       							 */
/*		  2.������ջ��������ݣ������Ը�����ס�ֽ��յ����ݣ������� */
/*		    �������з���������������������				     */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ����1.�����������¼�����ִ�д����񣬷������				 */
/*		  2.���յ�����������ʱ������ֵ�ź���				 		 */
/*		  3.CONNECT���ĳɹ�������30s��PING��ʱ���������¼���־λ	 */
/*		  4.PING���Ŀ��ٷ���ģʽ��2s���յ��ظ�������30s��ping��ʱ	 */
/*			���������¼���־λ			 						 */
/*		  5.CONNECT����ʧ�ܣ�WIFI���ӷ����������ɹ���̬תΪ����̬��*/
/*			��������												 */
/*---------------------------------------------------------------*/
void mqtt_buffer_rx_tx_task(void *pvParameters)
{
	while(1)
	{
		//�����������¼�����ִ�д����񣬷������
		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
							(EventBits_t		)WIFI_CONECT,
							(BaseType_t			)pdFALSE,				
							(BaseType_t			)pdTRUE,
							(TickType_t			)portMAX_DELAY);
		/*-------------------------------------------------------------*/
		/*                     �����ͻ���������					   */
		/*-------------------------------------------------------------*/
		if(MQTT_TxDataOutPtr != MQTT_TxDataInPtr) //if�����Ļ���˵�����ͻ�������������
		{                
			//�������ݻ���
			if(MQTT_TxDataOutPtr[2] == 0x30) 
			{	
				printf("��������:0x30\r\n");
			}
			else
			{  
				printf("��������:0x%x\r\n", MQTT_TxDataOutPtr[2]);
			}
			
			MQTT_TxData(MQTT_TxDataOutPtr);					
			MQTT_TxDataOutPtr += TBUFF_UNIT;				
			if(MQTT_TxDataOutPtr == MQTT_TxDataEndPtr)		
			{ 
				MQTT_TxDataOutPtr = MQTT_TxDataBuf[0];	
			}			
		}					
		/*-------------------------------------------------------------*/
		/*                     ������ջ���������                       */
		/*-------------------------------------------------------------*/
		if(MQTT_RxDataOutPtr != MQTT_RxDataInPtr) //if�����Ļ���˵�����ջ�������������	
		{		
			printf("���յ�����:");

			//�����һ���ֽ���0x30����ʾ�յ����Ƿ�������������������
			//��ȡ��������
			if((MQTT_RxDataOutPtr[2] == 0x30))
			{ 
				printf("�������ȼ�0����\r\n"); 		   	 //���������Ϣ
				MQTT_DealPushdata_Qs0(MQTT_RxDataOutPtr);//����ȼ�0��������
				xSemaphoreGive(BinarySemaphore);	     //������ֵ�ź���������MQTT����崦������ִ��
			}			
			//if�жϣ������һ���ֽ���0x20����ʾ�յ�����CONNACK����
			//��������Ҫ�жϵ�4���ֽڣ�����CONNECT�����Ƿ�ɹ�
			else if(MQTT_RxDataOutPtr[2] == 0x20)
			{             			
				switch(MQTT_RxDataOutPtr[5])
				{					   
					case 0x00: printf("CONNECT���ĳɹ�\r\n");				//CONNECT���ĳɹ�					   
							   TIM3_ENABLE_30S();				 			//����30s��PING��ʱ��	
							   xEventGroupSetBits(Event_Handle, PING_MODE); //����30s��PING��ʱ���������¼���־λ
							   break;													                                         
					case 0x01: printf("�����Ѿܾ�����֧�ֵ�Э��汾��w	׼������\r\n");       
							   vTaskResume(WIFI_Task_Handler);				//WIFI���ӷ����������ɹ���̬תΪ����̬����������
							   break;														
					case 0x02: printf("�����Ѿܾ������ϸ�Ŀͻ��˱�ʶ����׼������\r\n");   
							   vTaskResume(WIFI_Task_Handler);              //WIFI���ӷ����������ɹ���̬תΪ����̬����������
							   break; 														
					case 0x03: printf("�����Ѿܾ�������˲����ã�׼������\r\n");	    
							   vTaskResume(WIFI_Task_Handler);				//WIFI���ӷ����������ɹ���̬תΪ����̬����������
							   break;														
					case 0x04: printf("�����Ѿܾ�����Ч���û��������룬׼������\r\n");	   
							   vTaskResume(WIFI_Task_Handler);				//WIFI���ӷ����������ɹ���̬תΪ����̬����������					
							   break;														
					case 0x05: printf("�����Ѿܾ���δ��Ȩ��׼������\r\n");				   
							   vTaskResume(WIFI_Task_Handler);				//WIFI���ӷ����������ɹ���̬תΪ����̬����������					
							   break;																
					default  : printf("�����Ѿܾ���δ֪״̬��׼������\r\n");		     
							   vTaskResume(WIFI_Task_Handler);				//WIFI���ӷ����������ɹ���̬תΪ����̬����������			
							   break;																						
				}				
			}
			//if�жϣ���һ���ֽ���0xD0����ʾ�յ�����PINGRESP����
			else if(MQTT_RxDataOutPtr[2] == 0xD0)
			{ 
				printf("PING���Ļظ�\r\n");                       
				if(pingFlag == 1)
				{                   						     //���pingFlag=1����ʾ��һ�η���
					pingFlag = 0;    				       		 //Ҫ���pingFlag��־
				}
				else if(pingFlag > 1)	
				{ 				 								 //���pingFlag>1����ʾ�Ƕ�η����ˣ�������2s����Ŀ��ٷ���
					pingFlag = 0;     				      		 //Ҫ���pingFlag��־
					TIM3_ENABLE_30S(); 				      		 //PING��ʱ���ػ�30s��ʱ��
					xEventGroupSetBits(Event_Handle, PING_MODE); //30s��PING��ʱ���������¼���־λ
				}				
			}	
			//if�жϣ���һ���ֽ���0x90����ʾ�յ�����SUBACK����
			//��������Ҫ�ж϶��Ļظ��������ǲ��ǳɹ�
			if(MQTT_RxDataOutPtr[2]==0x90)
			{ 
				switch(MQTT_RxDataOutPtr[6])
				{					
					case 0x00 :
					case 0x01 : printf("���Ŀ��Ʊ��ĳɹ�\r\n");					
								break;                                                                         
					default   : printf("����ʧ��\r\n");   
								break;                                								
				}					
			}			
			
			MQTT_RxDataOutPtr += RBUFF_UNIT;                //ָ������	
			if(MQTT_RxDataOutPtr == MQTT_RxDataEndPtr)      //���ָ�뵽������β����
			{
				MQTT_RxDataOutPtr = MQTT_RxDataBuf[0];      //ָ���λ����������ͷ              
			}		          
		}			
		delay_ms(100);//��ʱ10ms
	}
}

/*---------------------------------------------------------------*/
/*��������void stack_task(void *pvParameters)                    */
/*��  �ܣ������ջ��С���ԣ��̶���							     */			
/*		  1.�鿴��������ʱ��ջ��С�����ڵ���          			 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
//void stack_task(void *pvParameters)
//{
//	TaskHandle_t TaskHandle;	
//	TaskStatus_t TaskStatus;
//	int i = 0;
//	while(1)
//	{
////		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
////							(EventBits_t		)WIFI_CONECT|PING_MODE,
////							(BaseType_t			)pdFALSE,				
////							(BaseType_t			)pdTRUE,
////							(TickType_t			)portMAX_DELAY);
////		LED_On();
////		delay_ms(500);			//��ʱ0.5s
////		LED_Off();
////		delay_ms(500);			//��ʱ0.5s
//	
//		for(i = 0; i < 5; i++)
//		{
//			if (i == 0)
//			{
//				TaskHandle = WIFI_Task_Handler;			//������������ȡ��������
//			}
//			else if (i == 1)
//			{
//				TaskHandle = MQTT_Cmd_Task_Handler;		//������������ȡ��������
//			}
//			else if (i == 2)
//			{
//				TaskHandle = MQTT_RxTx_Task_Handler;	//������������ȡ��������
//			}	
//			else if (i == 3)
//			{
//				TaskHandle = DHT11_Task_Handler;		//������������ȡ��������
//			}	
//			else if (i == 4)
//			{
//				TaskHandle = DATA_TX_Task_Handler;		//������������ȡ��������
//			}				
//			
//			//��ȡ������Ϣ
//			vTaskGetInfo((TaskHandle_t	)TaskHandle, 	//������
//						 (TaskStatus_t*	)&TaskStatus, 	//������Ϣ�ṹ��
//						 (BaseType_t	)pdTRUE,		//����ͳ�������ջ��ʷ��Сʣ���С
//						 (eTaskState	)eInvalid);		//�����Լ���ȡ��������׳̬
//			//ͨ�����ڴ�ӡ��ָ��������й���Ϣ��
//			printf("������:                %s\r\n",TaskStatus.pcTaskName);
//			printf("������:              %d\r\n",(int)TaskStatus.xTaskNumber);
//			printf("����׳̬:              %d\r\n",TaskStatus.eCurrentState);
//			printf("����ǰ���ȼ�:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
//			printf("��������ȼ�:          %d\r\n",(int)TaskStatus.uxBasePriority);
//			printf("�����ջ����ַ:        %#x\r\n",(int)TaskStatus.pxStackBase);
//			printf("�����ջ��ʷʣ����Сֵ:%d\r\n",TaskStatus.usStackHighWaterMark);
//		}
//		delay_ms(10 * 1000);	    //��ʱ10s

//	}
//}
