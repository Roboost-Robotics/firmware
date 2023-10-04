/**
 * @file conf_network_example.h
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the network parameters for the micro-ROS
 * communication.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CONF_NETWORK_H
#define CONF_NETWORK_H

/**
 * @brief Definition of the network parameters
 *
 */
const char* SSID = "your_ssid";
const char* SSID_PW = "your_password";
const uint16_t AGENT_PORT = 8888; // AGENT port number
#define AGENT_IP 0, 0, 0, 0       // Change to IP of the ROS2 messages recceiver

#endif