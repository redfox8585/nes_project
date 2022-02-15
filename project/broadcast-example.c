/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         Testing the broadcast layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */
#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>
#include <stdlib.h>
#include "net/rime/timesynch.h"
#include "sys/energest.h"

/*---------------------------------------------------------------------------*/
PROCESS(example_broadcast_process, "Broadcast example");
PROCESS(energest_thread, "Energest Thread");
AUTOSTART_PROCESSES(&example_broadcast_process, &energest_thread);

#define DEBUG 0
#define ENERGY 1
// change this define to change the print mode
#define PRINT_MODE DEBUG

/*int printf2(int mode, const char* format, ...){
    va_list args;
    if(PRINT_MODE == mode)
        return printf(format, args);
    return 0;
}*/

typedef struct {
    linkaddr_t addr;
} Path;

typedef struct {
	uint8_t type;
    linkaddr_t sender;
    uint8_t packetID;
    uint8_t hops;
    clock_time_t timestamp;
}Packet;

/*void add_to_path(Packet* packet, linkaddr_t addr){
    Path** p = &(packet->path);
    while(*p != NULL)
        p = &(p->next);
    
    *p = (Path*)malloc(sizeof(Path));
    (*p)->addr = addr;
    (*p)->next = NULL;

    packet->size++;
}

void deletePacket(Packet* packet){
    Path* next = packet->path;
    while(next != NULL){
        free(packet->path);
        packet->path = next;
        next = next->next;
    }
}*/

typedef struct PLE{
    Packet* packet;
    struct PLE* next;
}PacketListElement;

PacketListElement* packetList = NULL;
uint8_t packetIdCnt = 0;

static void addToPacketList(PacketListElement** list, Packet* packet){
    while(*list != NULL)
        list = &((*list)->next);

    *list = (PacketListElement*) malloc(sizeof(PacketListElement));
    (*list)->packet = (Packet*) malloc(sizeof(Packet));
    memcpy((void*)((*list)->packet), (void*)packet, sizeof(Packet));
}

static uint8_t isPacketInList(PacketListElement** list, Packet* packet){
    while(*list != NULL){
        if(linkaddr_cmp(&((*list)->packet->sender), &(packet->sender)) && (*list)->packet->packetID == packet->packetID)
            return 1;

        list = &((*list)->next);
    }

    return 0;
}

static void sendAlert(struct broadcast_conn* broadcast){
        /* Copy data to the packet buffer */
        Packet packet;
        packet.type = 0;
        linkaddr_copy(&(packet.sender), &linkaddr_node_addr);
        packet.packetID = packetIdCnt++;
        packet.hops = 0;
        packet.timestamp = clock_time();
        packetbuf_copyfrom((void*)&packet, sizeof(Packet));
        
        addToPacketList(&packetList, &packet);

        /* Send broadcast packet */
        broadcast_send(broadcast);
        
        printf("%d, %d sent broadcast packet\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}

/*---------------------------------------------------------------------------*/
/* 
 * Define callbacks function
 * Called when a packet has been received by the broadcast module
 */
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
    /* 
     * Function: void* packetbuf_dataptr()
     * Get a pointer to the data in the packetbuf
     * Using appropriate casting operator for data
     */
    Packet* packet = (Packet*) packetbuf_dataptr();
    packet->hops++;

    if(packet->type == 0){

        //printf("time: %d ms", (int)(clock_time() - packet->timestamp) * 1000/ CLOCK_SECOND );

        printf("broadcast message received from %d.%d: packed type %d, hops %d\n", 
            from->u8[0], from->u8[1], packet->type, packet->hops);

        if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0){
            printf("sink received packet from %d, %d\n", packet->sender.u8[0], packet->sender.u8[1]);
            return;
        }

    }

    /* transmit packet only if it has not already been sent */
    if(isPacketInList(&packetList, packet))
        return;

    packetbuf_copyfrom((void*)packet, sizeof(Packet));
        
    addToPacketList(&packetList, packet);
    
    /* Send broadcast packet */
    broadcast_send(c);
    
    printf("%d, %d sent broadcast packet\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}

/*static void initTime(struct broadcast_conn* broadcast){

    timesynch_init();

    if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0)
        timesynch_set_authority_level(2);
    else
        timesynch_set_authority_level(1);

    //Copy data to the packet buffer
    Packet packet;
    packet.type = 1;
    linkaddr_copy(&(packet.sender), &linkaddr_node_addr);
    packet.packetID = packetIdCnt++;
    packet.hops = 0;
    packet.timestamp = clock_time();
    packetbuf_copyfrom((void*)&packet, sizeof(Packet));
    
    addToPacketList(&packetList, &packet);

    // Send broadcast packet
    broadcast_send(broadcast);
    
    printf("%d, %d sent broadcast packet\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

}*/

static const struct broadcast_callbacks broadcast_cb = {broadcast_recv};

/*---------------------------------------------------------------------------*/
/* Broadcast connection */
static struct broadcast_conn broadcast;

PROCESS_THREAD(example_broadcast_process, ev, data) {
    /*static struct etimer et;*/
    
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);

    /* 
     * Set up a broadcast connection
     * Arguments: channel (129) and callbacks function
     */
    broadcast_open(&broadcast, 129, &broadcast_cb);

    //initTime(&broadcast);
    
    /* Send broadcast packet in loop */
    while(1) {
        /* Delay 2-4 seconds*/
        /*unsigned long ticks = CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2);
        
        etimer_set(&et, ticks);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));*/

        PROCESS_YIELD_UNTIL(ev == sensors_event && data == &button_sensor);
        printf("================\n");
        printf(" button clicked \n");
        printf("================\n");
        sendAlert(&broadcast);

    }
    
    PROCESS_END();
}


PROCESS_THREAD(energest_thread, ev, data){

    static struct etimer periodicTimer;

    PROCESS_BEGIN();
    
    energest_init();

    ENERGEST_ON(ENERGEST_TYPE_CPU);
    ENERGEST_ON(ENERGEST_TYPE_LPM);
    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);

    while(1){
        etimer_set(&periodicTimer, CLOCK_SECOND * 1);
        PROCESS_YIELD_UNTIL(etimer_expired(&periodicTimer));
        //etimer_reset(&periodicTimer);
        energest_flush();
        printf("cpu: %4lu", energest_type_time(ENERGEST_TYPE_CPU));
        printf("lpm: %4lu", energest_type_time(ENERGEST_TYPE_LPM));
        printf("transmit: %4lu", energest_type_time(ENERGEST_TYPE_TRANSMIT));
        printf("listen: %4lu\n", energest_type_time(ENERGEST_TYPE_LISTEN));
    }

    PROCESS_END();

}