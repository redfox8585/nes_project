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

/*---------------------------------------------------------------------------*/
PROCESS(example_broadcast_process, "Broadcast example");
AUTOSTART_PROCESSES(&example_broadcast_process);

/*---------------------------------------------------------------------------*/
/* 
 * Define callbacks function
 * Called when a packet has been received by the broadcast module
 */

float neighbors_ids[4] = {0};
int neighbors_id[4][2] = {0};

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
    /* 
     * Function: void* packetbuf_dataptr()
     * Get a pointer to the data in the packetbuf
     * Using appropriate casting operator for data
     */
    printf("broadcast message received from %d.%d: '%s'\n", 
           from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
    add_neighbor(from->u8[0],from->u8[1]);
}

void add_neighbor(int addr1, int addr2){
	int n=0;
	for (n=0,n<5,++n){
		if ((neighbors_id[n][0]==addr1)&&(neighbors_id[n][1]==addr2)){
			printf("neighbor already known\n");
			break;
		} else if ((neighbors_id[n][0]==0)&&(neighbors_id[n][1]==0)){
			neighbors_id[n][0]==addr1;
			neighbors_id[n][1]==addr2;
			printf("added neighbor: %d.%d\n",addr1,addr2);
                        break;
		}
	}
}

static const struct broadcast_callbacks broadcast_cb = {broadcast_recv};

/*---------------------------------------------------------------------------*/
/* Broadcast connection */
static struct broadcast_conn broadcast;

PROCESS_THREAD(example_broadcast_process, ev, data) {
    static struct etimer et;
    
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();
    
    /* 
     * Set up a broadcast connection
     * Arguments: channel (129) and callbacks function
     */
    broadcast_open(&broadcast, 129, &broadcast_cb);
    
    /* Send broadcast packet in loop */
    while(1) {
        /* Delay 2-4 seconds*/
        unsigned long ticks = CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2);
        
        etimer_set(&et, ticks);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
        
        /* Copy data to the packet buffer */
        packetbuf_copyfrom("Hello", 6);
        
        /* Send broadcast packet */
        broadcast_send(&broadcast);
        
        printf("broadcast packet sent\n");
    }
    
    PROCESS_END();
}