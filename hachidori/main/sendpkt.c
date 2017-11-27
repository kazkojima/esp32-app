/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_heap_caps.h"
#include "nvs.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

extern int sockfd;
extern SemaphoreHandle_t send_sem;
extern xQueueHandle pkt_queue;

void pkt_task(void *arg)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

#if CONFIG_PACKET_BUNDLE
    // Bundle upto 6 B3packets into one udp packet.
    struct B3packet pkts[6];
    while(1) {
        if (xQueueReceive(pkt_queue, &pkts[0], portMAX_DELAY) != pdTRUE)
            continue;
        int len = sizeof(struct B3packet);
        if (xQueueReceive(pkt_queue, &pkts[1], 0) == pdTRUE) {
            len += sizeof(struct B3packet);
            if (xQueueReceive(pkt_queue, &pkts[2], 0) == pdTRUE) {
                len += sizeof(struct B3packet);
                if (xQueueReceive(pkt_queue, &pkts[3], 0) == pdTRUE) {
                    len += sizeof(struct B3packet);
                    if (xQueueReceive(pkt_queue, &pkts[4], 0) == pdTRUE) {
                        len += sizeof(struct B3packet);
                        if (xQueueReceive(pkt_queue, &pkts[5], 0) == pdTRUE) {
                            len += sizeof(struct B3packet);
                        }
                    }
                }
            }
        }
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, pkts, len, 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }            
#else
    struct B3packet pkt;
    while(1) {
        if (xQueueReceive(pkt_queue, &pkt, portMAX_DELAY) != pdTRUE)
            continue;
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
#endif
}
