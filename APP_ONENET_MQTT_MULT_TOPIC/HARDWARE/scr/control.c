/*-------------------------------------------------*/
/*                                                 */
/* 		处理传感器数据发送至队列及发送设备状态      */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"
#include "control.h"
#include "usart1.h"
#include "mqtt.h"
   
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"	
	
extern QueueHandle_t Message_Queue;	//传感器数据消息队列句柄 

/*---------------------------------------------------------------*/
/*函数名：send_data_to_queue_int(const char *laber, char *flag)  */
/*功  能：向消息队列发送传感器数据，将由传感器数据消息队列处理理任 */
/*		  务（固定）集中处理     								 */
/*参  数：1.const char *topic_name : topic主题 					 */
/*        2.int value :topic内容（传感器数据，char和int类型）     */
/*返回值：无                                       			     */
/*---------------------------------------------------------------*/
void send_data_to_queue_int(const char *topic_name, int value)
{	
	char data_of_sensor[40] = {0};
	sprintf(data_of_sensor, "%s:%d", topic_name, value);
	xQueueSend(Message_Queue, &data_of_sensor, portMAX_DELAY);  //向消息队列发送消息体，将传感器数据发送至消息队列
}

/*---------------------------------------------------------------*/
/*函数名：send_data_to_queue_int(const char *laber, char *flag)  */
/*功  能：向消息队列发送传感器数据，将由传感器数据消息队列处理理任 */
/*		  务（固定）集中处理（未测试）     						 */
/*参  数：1.const char *topic_name : topic主题 					 */
/*        2.char *value :topic内容（传感器数据，char*和其他类型） */
/*返回值：无                                       			     */
/*---------------------------------------------------------------*/
void send_data_to_queue_type(const char *topic_name, char *value)
{	
	char data_of_sensor[40] = {0};
	sprintf(data_of_sensor, "%s:%s", topic_name, value);
	xQueueSend(Message_Queue, &data_of_sensor, portMAX_DELAY);  //向消息队列发送消息体，将传感器数据发送至消息队列
}

/*---------------------------------------------------------------*/
/*函数名：send_device_state(const char *topic_name, char *state) */
/*功  能：处理待发送的控制设备数据（固定）					     */
/*		  1.处理待发送的控制设备数据，移入MQTT数据发送缓冲区       */
/*参  数：1.const char *topic_name :topic主题 LOCAL_TOPIC		 */
/*			名称												 */
/*        2.char *value :topic内容（设备状态，state）       		 */
/*返回值：无                                       			     */
/*---------------------------------------------------------------*/
void send_device_state(const char *topic_name, char *state)
{
	int  data_len = 0;			
	
	data_len = strlen(state);
	
	taskENTER_CRITICAL(); //进入临界区，防止中断打断
	MQTT_PublishQs0(topic_name, state, data_len);//添加控制设备topic以及状态数据，直接发布给服务器	
	taskEXIT_CRITICAL();  //退出临界区
	
	//char data_of_sensor[30] = {0};
	//sprintf(data_of_sensor, "%s:%s", topic_name, state);
	//xQueueSend(Message_Queue, &data_of_sensor, portMAX_DELAY);  //向消息队列发送消息体，将相关数据发送至消息队列
}



