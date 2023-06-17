#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <geometry_msgs/msg/twist.h>

/**
 * @brief Subscriber callback function declaration
 * 
 * @param msgin 
 */
void subscriber_callback(const void * msgin);

#endif // SUBSCRIBER_H