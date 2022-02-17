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

#define NO_OUTPUT 0
#define DEBUG 1
#define ENERGY 2
#define HOPS 4
#define LATENCY 8

// change this define to change the print mode
#define PRINT_MODE (LATENCY | HOPS)

static int isPrintMode(int mode){
    return (PRINT_MODE & mode) > 0;
}

// address of the reporting point
#define REPORTING_POINT_ADDR0 1
#define REPORTING_POINT_ADDR1 0

// packet types
#define ALERT 0
#define ACK 1

typedef struct {
	uint8_t type;
    linkaddr_t sender;
    uint8_t packetID;
    uint8_t hops;
    linkaddr_t receiver;
}Packet;


typedef struct PLE{
    Packet* packet;
    struct PLE* next;
}PacketListElement;

static PacketListElement* packetList = NULL;

static uint8_t lastPacketIdSend = 0;
static uint8_t lastPacketIdAck = 0;

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
        packet.type = ALERT;
        linkaddr_copy(&(packet.sender), &linkaddr_node_addr);
        packet.packetID = ++lastPacketIdSend;
        packet.hops = 0;

        linkaddr_copy(&(packet.receiver), &linkaddr_null);
        packet.receiver.u8[0] = REPORTING_POINT_ADDR0;
        packet.receiver.u8[1] = REPORTING_POINT_ADDR1;

        packetbuf_copyfrom((void*)&packet, sizeof(Packet));
        
        addToPacketList(&packetList, &packet);

        /* Send broadcast packet */
        broadcast_send(broadcast);
        
        if(isPrintMode(DEBUG))
            printf("%d, %d sent broadcast packet\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}

static void sendAck(struct broadcast_conn* broadcast, linkaddr_t* sender){
        /* Copy data to the packet buffer */
        Packet packet;
        packet.type = ACK;
        
        linkaddr_copy(&(packet.sender), &linkaddr_null);
        packet.sender.u8[0] = REPORTING_POINT_ADDR0;
        packet.sender.u8[1] = REPORTING_POINT_ADDR1;
        packet.packetID = ++lastPacketIdSend;
        packet.hops = 0;

        linkaddr_copy(&(packet.receiver), sender);

        packetbuf_copyfrom((void*)&packet, sizeof(Packet));
        
        addToPacketList(&packetList, &packet);

        /* Send broadcast packet */
        broadcast_send(broadcast);
        
        if(isPrintMode(DEBUG))
            printf("%d, %d sent ack broadcast packet\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}


static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {


    Packet* packet = (Packet*) packetbuf_dataptr();
    packet->hops++;

    if(isPrintMode(DEBUG))
            printf("broadcast message received from %d.%d: packed type %d, hops %d\n", from->u8[0], from->u8[1], packet->type, packet->hops);

    // check if packet reached destination
    if(linkaddr_cmp(&linkaddr_node_addr, &packet->receiver)){

        if(packet->type == ALERT){

            if(linkaddr_node_addr.u8[0] == REPORTING_POINT_ADDR0 && linkaddr_node_addr.u8[1] == REPORTING_POINT_ADDR1){
                
                if(isPrintMode(LATENCY))
                    printf("sink received packet from %d, %d\n", packet->sender.u8[0], packet->sender.u8[1]);

                if(isPrintMode(HOPS))
                    printf("number of hops: %d\n", packet->hops);

                // reporting point received packet: send acknowledgement back to sender
                sendAck(c, &packet->sender);

                return;
            }

        } else if (packet->type == ACK) {

            if(isPrintMode(LATENCY))
                printf("source received ack packet from %d, %d\n", packet->sender.u8[0], packet->sender.u8[1]);

            // source of alert received ackonwledgement: stop retransmission of packet
            lastPacketIdAck = lastPacketIdSend;
            
            return;
        }
    }

    /* transmit packet only if it has not already been sent */
    if(isPacketInList(&packetList, packet))
        return;


    // wait random time to avoid collision
    /*static struct etimer et;
    unsigned long ticks = CLOCK_SECOND / 1000 + random_rand() % (CLOCK_SECOND / 500);
    etimer_set(&et, ticks);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));*/

    packetbuf_copyfrom((void*)packet, sizeof(Packet));
        
    addToPacketList(&packetList, packet);
    
    /* Send broadcast packet */
    broadcast_send(c);
    
    if(isPrintMode(DEBUG))
        printf("%d, %d sent broadcast packet\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}

static const struct broadcast_callbacks broadcast_cb = {broadcast_recv};

/*---------------------------------------------------------------------------*/
/* Broadcast connection */
static struct broadcast_conn broadcast;

PROCESS_THREAD(example_broadcast_process, ev, data) {
    static struct etimer et;
    
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);

    broadcast_open(&broadcast, 129, &broadcast_cb);
    
    while(1) {


        PROCESS_YIELD_UNTIL(ev == sensors_event && data == &button_sensor);

        if(isPrintMode(LATENCY)) {
            printf("================\n");
            printf(" button clicked \n");
            printf("================\n");
        }

        sendAlert(&broadcast);


        while(1){
            unsigned long ticks = CLOCK_SECOND * 10;
            etimer_set(&et, ticks);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

            // if packet has not been acknowledged after 10 seconds: retransmit packet
            if(lastPacketIdSend == lastPacketIdAck){
                break;
            }

            sendAlert(&broadcast);
        }

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

        energest_flush();

        if(isPrintMode(ENERGY)) {
            printf("cpu: %4lu ", energest_type_time(ENERGEST_TYPE_CPU));
            printf("lpm: %4lu ", energest_type_time(ENERGEST_TYPE_LPM));
            printf("transmit: %4lu ", energest_type_time(ENERGEST_TYPE_TRANSMIT));
            printf("listen: %4lu\n", energest_type_time(ENERGEST_TYPE_LISTEN));
        }
    }

    PROCESS_END();

}