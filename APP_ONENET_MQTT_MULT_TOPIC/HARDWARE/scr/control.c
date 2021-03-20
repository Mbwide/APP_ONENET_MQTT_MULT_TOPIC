/*-------------------------------------------------*/
/*                                                 */
/* 		�����������ݷ��������м������豸״̬      */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"
#include "control.h"
#include "usart1.h"
#include "mqtt.h"
   
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"	
	
extern QueueHandle_t Message_Queue;	//������������Ϣ���о�� 

/*---------------------------------------------------------------*/
/*��������send_data_to_queue_int(const char *laber, char *flag)  */
/*��  �ܣ�����Ϣ���з��ʹ��������ݣ����ɴ�����������Ϣ���д������� */
/*		  �񣨹̶������д���     								 */
/*��  ����1.const char *topic_name : topic���� 					 */
/*        2.int value :topic���ݣ����������ݣ�char��int���ͣ�     */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
void send_data_to_queue_int(const char *topic_name, int value)
{	
	char data_of_sensor[40] = {0};
	sprintf(data_of_sensor, "%s:%d", topic_name, value);
	xQueueSend(Message_Queue, &data_of_sensor, portMAX_DELAY);  //����Ϣ���з�����Ϣ�壬�����������ݷ�������Ϣ����
}

/*---------------------------------------------------------------*/
/*��������send_data_to_queue_int(const char *laber, char *flag)  */
/*��  �ܣ�����Ϣ���з��ʹ��������ݣ����ɴ�����������Ϣ���д������� */
/*		  �񣨹̶������д���δ���ԣ�     						 */
/*��  ����1.const char *topic_name : topic���� 					 */
/*        2.char *value :topic���ݣ����������ݣ�char*���������ͣ� */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
void send_data_to_queue_type(const char *topic_name, char *value)
{	
	char data_of_sensor[40] = {0};
	sprintf(data_of_sensor, "%s:%s", topic_name, value);
	xQueueSend(Message_Queue, &data_of_sensor, portMAX_DELAY);  //����Ϣ���з�����Ϣ�壬�����������ݷ�������Ϣ����
}

/*---------------------------------------------------------------*/
/*��������send_device_state(const char *topic_name, char *state) */
/*��  �ܣ���������͵Ŀ����豸���ݣ��̶���					     */
/*		  1.��������͵Ŀ����豸���ݣ�����MQTT���ݷ��ͻ�����       */
/*��  ����1.const char *topic_name :topic���� LOCAL_TOPIC		 */
/*			����												 */
/*        2.char *value :topic���ݣ��豸״̬��state��       		 */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
void send_device_state(const char *topic_name, char *state)
{
	int  data_len = 0;			
	
	data_len = strlen(state);
	
	taskENTER_CRITICAL(); //�����ٽ�������ֹ�жϴ��
	MQTT_PublishQs0(topic_name, state, data_len);//��ӿ����豸topic�Լ�״̬���ݣ�ֱ�ӷ�����������	
	taskEXIT_CRITICAL();  //�˳��ٽ���
	
	//char data_of_sensor[30] = {0};
	//sprintf(data_of_sensor, "%s:%s", topic_name, state);
	//xQueueSend(Message_Queue, &data_of_sensor, portMAX_DELAY);  //����Ϣ���з�����Ϣ�壬��������ݷ�������Ϣ����
}



