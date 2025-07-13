/*
 * Copyright 2025 NXP
 * 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <openthread/coap.h>
#include <openthread/cli.h>
#include "LED.h"
#include "Temp_sensor.h"
#include <stdio.h>
#include <stdlib.h>

#include "Coap_Server.h"
#include <ctype.h>
#include <string.h>

otInstance *instance_g;


void handle_led_request(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    char payload[10];
    int length = otMessageRead(aMessage, otMessageGetOffset(aMessage), payload, sizeof(payload) - 1);
    payload[length] = '\0';

    if (payload[0] == '1')
    {
        // Turn LED on
        otCliOutputFormat("Payload Recived: %s\r\n", payload);
        otCliOutputFormat("LED On \r\n");
        LED_ON(0);

    }
    else if (payload[0] == '0')
    {
        // Turn LED off
        otCliOutputFormat("Payload Recived: %s\r\n", payload);
        otCliOutputFormat("LED Off \r\n");
        LED_OFF(0);
    }

    //Send response
    otMessage *response = otCoapNewMessage(instance_g, NULL);
    otCoapMessageInitResponse(response, aMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
    otCoapSendResponse(instance_g, response, aMessageInfo);
}

char str_default[20] = "Sin nombre";
char str_nombre[20] = "Sin nombre";

void handle_nombre_request(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    otMessage *response;

    if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
    {
        response = otCoapNewMessage(instance_g, NULL);
        otCliOutputFormat("GET\r\n");

        if (response != NULL)
        {
            otCoapMessageInitResponse(response, aMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CONTENT);
            
            otCoapMessageSetPayloadMarker(response);
                       
            otCliOutputFormat("payload: %s\r\n", str_nombre);

            otMessageAppend(response, str_nombre, strlen(str_nombre));

            otCoapSendResponse(instance_g, response, aMessageInfo);
        }
    }
    else if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_PUT)
    {
        char payload[20];
        int length = otMessageRead(aMessage, otMessageGetOffset(aMessage), payload, sizeof(payload) - 1);
        payload[length] = '\0';

        response = otCoapNewMessage(instance_g, NULL);
        otCliOutputFormat("PUT\r\n");

        strcpy(&str_nombre[0],&payload[0]);

        otMessage *response = otCoapNewMessage(instance_g, NULL);
        otCoapMessageInitResponse(response, aMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
        otCoapSendResponse(instance_g, response, aMessageInfo);
    }
    else if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_DELETE)
    {
        response = otCoapNewMessage(instance_g, NULL);

        strcpy(&str_nombre[0],&str_default[0]);

        otCliOutputFormat("DELETE\r\n");

        otMessage *response = otCoapNewMessage(instance_g, NULL);
        otCoapMessageInitResponse(response, aMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
        otCoapSendResponse(instance_g, response, aMessageInfo);
    }
}

void handle_sensor_request(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    static double temp_value = 0;
    temp_value = Get_Temperature();

    otMessage *response;

    if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)  
    {
        response = otCoapNewMessage(instance_g, NULL);
        otCliOutputFormat("GET\r\n");

        if (response != NULL)
        {
            otCoapMessageInitResponse(response, aMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CONTENT);
            
            otCoapMessageSetPayloadMarker(response);
           
            char sensorData[50] = {"0"};
            
            snprintf(sensorData, sizeof(sensorData), "%d", (int)temp_value);
            otCliOutputFormat("payload: %s\r\n", sensorData);

            otMessageAppend(response, sensorData, strlen(sensorData));

            otCoapSendResponse(instance_g, response, aMessageInfo);
        }
    }
}


void init_coap_server(otInstance *aInstance)
{
    I2C2_InitPins();
    LED_INIT();
    Temp_Sensor_start();

    instance_g = aInstance;
    
    static otCoapResource coapResource_led;
    static otCoapResource coapResource_sensor;
    static otCoapResource coapResource_nombre;
    
    coapResource_led.mUriPath = "led";
    coapResource_led.mHandler = handle_led_request;
    coapResource_led.mContext = NULL;
    coapResource_led.mNext = NULL;

    otCoapAddResource(aInstance, &coapResource_led);

    coapResource_sensor.mUriPath = "sensor";
    coapResource_sensor.mHandler = handle_sensor_request;
    coapResource_sensor.mContext = NULL;
    coapResource_sensor.mNext = NULL;

    otCoapAddResource(aInstance, &coapResource_sensor);

    coapResource_sensor.mUriPath = "nombre";
    coapResource_sensor.mHandler = handle_nombre_request;
    coapResource_sensor.mContext = NULL;
    coapResource_sensor.mNext = NULL;

    otCoapAddResource(aInstance, &coapResource_nombre);
}
