
#ifndef __CONTROL_H
#define __CONTROL_H

void send_data_to_queue_int(const char *topic_name, int value);
void send_data_to_queue_type(const char *topic_name, char *value);
void send_device_state(const char *topic_name, char *state);
#endif


