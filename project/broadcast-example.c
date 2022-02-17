#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>
#include <stdlib.h>
#include "net/rime/timesynch.h"
#include "sys/energest.h"

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
static const linkaddr_t REPORTING_POINT_ADDR = { {1,0,0,0, 0,0,0,0} };

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

        linkaddr_copy(&(packet.receiver), &REPORTING_POINT_ADDR);

        packetbuf_copyfrom((void*)&packet, sizeof(Packet));
        
        addToPacketList(&packetList, &packet);

        /* Send broadcast packet */
        broadcast_send(broadcast);
        
        if(isPrintMode(LATENCY | DEBUG))
            printf("%d sent alert packet [%d, %d]\n", linkaddr_node_addr.u8[0], packet.sender.u8[0], packet.packetID);
}

static void sendAck(struct broadcast_conn* broadcast, linkaddr_t* sender){
        /* Copy data to the packet buffer */
        Packet packet;
        packet.type = ACK;
        
        linkaddr_copy(&(packet.sender), &REPORTING_POINT_ADDR);
        packet.packetID = ++lastPacketIdSend;
        packet.hops = 0;

        linkaddr_copy(&(packet.receiver), sender);

        packetbuf_copyfrom((void*)&packet, sizeof(Packet));
        
        addToPacketList(&packetList, &packet);

        /* Send broadcast packet */
        broadcast_send(broadcast);
        
        if(isPrintMode(LATENCY | DEBUG))
            printf("%d sent ack packet [%d, %d]\n", linkaddr_node_addr.u8[0], packet.sender.u8[0], packet.packetID);
}


static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {


    Packet* packet = (Packet*) packetbuf_dataptr();
    packet->hops++;

    if(linkaddr_cmp(&linkaddr_node_addr, &packet->receiver)){
        if(isPrintMode(LATENCY | DEBUG))
            printf("%s received %s packet [%d, %d] from %d\n", packet->type == ACK ? "source" : "sink", packet->type == ACK ? "ack" : "alert", packet->sender.u8[0], packet->packetID, packet->sender.u8[0]);

        if(isPrintMode(HOPS))
            printf("number of hops: %d\n", packet->hops);
    }

    /* transmit packet only if it has not already been sent */
    if(isPacketInList(&packetList, packet))
        return;
    packetbuf_copyfrom((void*)packet, sizeof(Packet));
    addToPacketList(&packetList, packet);


    if(isPrintMode(DEBUG))
            printf("broadcast message received from %d: packed type %d, hops %d\n", from->u8[0], packet->type, packet->hops);

    // check if packet reached destination
    if(linkaddr_cmp(&linkaddr_node_addr, &packet->receiver)){

        if(packet->type == ALERT){

            if(linkaddr_cmp(&linkaddr_node_addr, &REPORTING_POINT_ADDR)){

                // reporting point received packet: send acknowledgement back to sender
                sendAck(c, &packet->sender);

                return;
            }

        } else if (packet->type == ACK) {

            // source of alert received ackonwledgement: stop retransmission of packet
            lastPacketIdAck = lastPacketIdSend;
            
            return;
        }
    }
    
    /* Send broadcast packet */
    broadcast_send(c);
    
    if(isPrintMode(DEBUG))
        printf("%d sent packet\n", linkaddr_node_addr.u8[0]);
}

static const struct broadcast_callbacks broadcast_cb = {broadcast_recv};

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