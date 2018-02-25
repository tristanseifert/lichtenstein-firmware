/*
 * ARP.cpp
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */
#define LOG_MODULE "ARP"

#include "ARP.h"
#include "ARPPrivate.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstring>

// log caching
#define LOG_CACHING							1

namespace ip {

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _ARPTaskTrampoline(void *ctx) {
	(static_cast<ARP *>(ctx))->taskEntry();
}

/**
 * Initializes the ARP handler.
 */
ARP::ARP(Stack *s) : stack(s) {
	// clear ARP cache
	size_t cacheLength = sizeof(this->cache);
	memset(this->cache, 0, cacheLength);

	LOG(S_INFO, "ARP cache has %u entries", ARP::cacheEntries);

	// set up the message queue
	BaseType_t ok;

	// create the queue
	this->messageQueue = xQueueCreate(ARP::messageQueueSize, sizeof(arp_task_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue!");
	}

	// now, create the task
	ok = xTaskCreate(_ARPTaskTrampoline, "ARP", ARP::TaskStackSize,
					 this, ARP::TaskPriority, &this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}
}

/**
 * Stops the ARP task.
 */
ARP::~ARP() {
	// kill the task and queue
	if(this->task) {
		vTaskDelete(this->task);
	}

	if(this->messageQueue) {
		vQueueDelete(this->messageQueue);
	}
}



/**
 * Returns the number of available entries in the cache.
 */
int ARP::numFreeCacheEntries(void) {
	int free = 0;

	// iterate over each entry
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// if its valid bit is not set, that entry is free
		if(this->cache[i].valid == false) {
			free++;
		}
	}

	return free;
}

/**
 * Discards the oldest entry in the ARP cache.
 */
void ARP::discardOldestEntry(void) {
	uint32_t oldestAge = 0xFFFFFFFF;
	size_t oldestIndex = 0;

	// iterate over the entire cache to find the oldest entry
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// is this entry's age lower than the current oldest?
		if(this->cache[i].age < oldestAge) {
			oldestAge = this->cache[i].age;
			oldestIndex = i;
		}
	}

	// discard the oldest entry
#if LOG_CACHING
	int age = xTaskGetTickCount() - oldestAge;

	char ipStr[16];
	Stack::ipToString(this->cache[oldestIndex].ip, ipStr, 16);

	LOG(S_DEBUG, "Discarding ARP cache entry for %s (%d ticks old)", ipStr, age);
#endif

	this->cache[oldestIndex].valid = false;
}

/**
 * Inserts a new IP address into the ARP cache.
 */
void ARP::insertAddress(stack_mac_addr_t mac, stack_ipv4_addr_t addr) {
	// free up an entry in the cache if it's full
	if(this->numFreeCacheEntries() == 0) {
		this->discardOldestEntry();
	}

	// check if we have an entry for this IP, and update it
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// if the IP matches, we're done
		if(this->cache[i].ip == addr) {
			this->cache[i].mac = mac;

#if LOG_CACHING
	char ipStr[16];
	Stack::ipToString(addr, ipStr, 16);

	char macStr[18];
	Stack::macToString(mac, macStr, 18);

	LOG(S_DEBUG, "Updated cache entry: %s = %s", macStr, ipStr);
#endif

			return;
		}
	}

	// if we get here, we're going to need to insert a new entry
#if LOG_CACHING
	char ipStr[16];
	Stack::ipToString(addr, ipStr, 16);

	char macStr[18];
	Stack::macToString(mac, macStr, 18);

	LOG(S_DEBUG, "Inserting into cache: %s = %s", macStr, ipStr);
#endif

	// find the first free entry and insert it
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// is this entry free?
		if(this->cache[i].valid == false) {
			// if so, insert it
			this->cache[i].age = xTaskGetTickCount();

			this->cache[i].ip = addr;
			this->cache[i].mac = mac;

			// check whether a task was waiting on this IP address
			// TODO: implement task IP checking

			// we can exit now
			return;
		}
	}

	// we shouldn't get down here
	LOG(S_ERROR, "Couldn't insert into ARP cache: no space in cache (wtf)");
}



/**
 * Handles a received ARP frame.
 */
void ARP::handleARPFrame(void *p) {
	stack_rx_packet_t *packet = (stack_rx_packet_t *) p;

	// first, swap all the multi-byte fields
	arp_ipv4_packet_t *arp = (arp_ipv4_packet_t *) packet->payload;

	arp->hwType = __builtin_bswap16(arp->hwType);
	arp->protoType = __builtin_bswap16(arp->protoType);
	arp->op = __builtin_bswap16(arp->op);

	// handle requests
	if(arp->op == ARP_OP_REQUEST) {
		this->handleARPRequest(arp);
	}
	// and handle replies
	else if(arp->op == ARP_OP_REPLY) {
		this->handleARPReply(arp);
	}

	// we've handled the entire packet so release it
	this->stack->doneWithRxPacket(packet);
}

/**
 * Transmits a gratuitous ARP for our MAC address.
 */
void ARP::sendGratuitousARP(void) {

}

/**
 * Clears the ARP cache.
 */
void ARP::clearARPCache(void) {
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		this->cache[i].valid = false;
	}
}



/**
 * Handles a received ARP request. This does two things:
 *
 * - If the target and sender protocol addresses are the same, we received a
 *   gratuitous ARP request from another device on the network: unless another
 *   task is waiting for this IP address, we ignore it as to not clutter up the
 *   ARP cache.
 *
 * - If the target protocol address is the same as our IPv4 address, we will
 *   respond to this request with our unicast MAC address.
 */
void ARP::handleARPRequest(void *_packet) {
	arp_ipv4_packet_t *arp = (arp_ipv4_packet_t *) _packet;

	// was it a gratuitous ARP?
	if(arp->targetIP == arp->senderIP) {
		// XXX: debugging
		char ip[16];
		Stack::ipToString(arp->targetIP, ip, 16);
		LOG(S_DEBUG, "Received gratuitous ARP for address %s", ip);

		// if so, let the ARP task handle it
		this->sendReceivedPacketToTask(arp, 0);
		return;
	}

	// XXX: debugging
	char ip[16];
	Stack::ipToString(arp->targetIP, ip, 16);
	LOG(S_DEBUG, "Who has %s?", ip);

	// is this a request for our MAC address?
	if(arp->targetIP == this->stack->getIPAddress()) {
		LOG(S_DEBUG, "Received request for our IP");

		// learn the MAC/IP of the node requesting our IP
		this->sendReceivedPacketToTask(arp, 0);

		// send a reply
		this->sendResponseRequestToTask(arp, 0);
	}
}

/**
 * Handles an ARP reply - these _should_ be sent via unicast to our node, but
 * some devices use the "ARP Reply" method of sending gratuitous ARPs, in which
 * case we simply ignore it (target MAC = source MAC)
 */
void ARP::handleARPReply(void *_packet) {
	arp_ipv4_packet_t *arp = (arp_ipv4_packet_t *) _packet;

	// ignore gratuitious ARP
	if(arp->senderMAC == arp->targetMAC) {
		// XXX: debugging
		char ip[16];
		Stack::ipToString(arp->targetIP, ip, 16);
		LOG(S_DEBUG, "Ignoring gratuitous ARP for address %s", ip);

		return;
	}

	// process the received frame later
	this->sendReceivedPacketToTask(arp, 0);
}




/**
 * Entry point for the ARP message task.
 */
void ARP::taskEntry(void) {
	BaseType_t ok;
	arp_task_message_t msg;

	// message loop
	while(1) {
		// attempt to receive a message
		ok = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG(S_ERROR, "Error reading from queue");
			continue;
		}

		// parse it and act upon it
		switch(msg.type) {
			// processes a received ARP message (a reply or gratuitous)
			case kARPMessageReceived:
				this->insertAddress(msg.packet.senderMAC, msg.packet.senderIP);
				break;

			// send a reply to a request for our MAC
			case kARPMessageSendReply:
				this->taskGenerateResponse(&msg);
				break;
		}
	}
}

/**
 * Generate a response to a received ARP request.
 */
void ARP::taskGenerateResponse(void *_msg) {
	stack_tx_packet_t *tx;
	arp_task_message_t *msg = (arp_task_message_t *) _msg;

	// request a transmit buffer
	const size_t payloadLength = sizeof(arp_ipv4_packet_t);
	tx = (stack_tx_packet_t *) this->stack->getTxPacket(payloadLength);

	if(tx == nullptr) {
		LOG(S_ERROR, "Couldn't get TX buffer for ARP response!");
		return;
	}

	// populate the payload
	arp_ipv4_packet_t *request = &msg->packet;
	arp_ipv4_packet_t *reply = (arp_ipv4_packet_t *) tx->payload;

	// XXX: debugging
	char mac[18];
	Stack::macToString(request->senderMAC, mac, 18);
	LOG(S_DEBUG, "Sending ARP reply to %s", mac);

	reply->hwType = ARP_HW_ETHERNET;
	reply->protoType = kProtocolIPv4;

	reply->hwAddressLength = sizeof(stack_mac_addr_t);
	reply->protoAddressLength = sizeof(stack_ipv4_addr_t);

	reply->op = ARP_OP_REPLY;

	// target MAC/IP are that of the original requestor
	reply->targetMAC = request->senderMAC;
	reply->targetIP = request->senderIP;

	// sender MAC/IP are OUR values
	reply->senderMAC = this->stack->mac;
	reply->senderIP = this->stack->getIPAddress();

	// byteswap all multibyte fields
	reply->hwType = __builtin_bswap16(reply->hwType);
	reply->protoType = __builtin_bswap16(reply->protoType);
	reply->op = __builtin_bswap16(reply->op);

	// transmit the buffer
	this->stack->sendTxPacket(tx, request->senderMAC, kProtocolARP);
}



/**
 * Sends a received packet to the task so that it can add the MAC address to
 * the global ARP cache.
 */
void ARP::sendReceivedPacketToTask(void *_packet, int timeout) {
	arp_ipv4_packet_t *packet = (arp_ipv4_packet_t *) _packet;

	// prepare the message
	arp_task_message_t msg;

	msg.type = kARPMessageReceived;
	memcpy(&msg.packet, packet, sizeof(arp_ipv4_packet_t));

	// send the message
	if(!this->postMessageToTask(&msg, timeout)) {
		LOG(S_ERROR, "Couldn't process received ARP packet");
	}
}

/**
 * Requests that the ARP task generates a reply to this ARP request.
 */
void ARP::sendResponseRequestToTask(void *_packet, int timeout) {
	arp_ipv4_packet_t *packet = (arp_ipv4_packet_t *) _packet;

	// prepare the message
	arp_task_message_t msg;

	msg.type = kARPMessageSendReply;
	memcpy(&msg.packet, packet, sizeof(arp_ipv4_packet_t));

	// send the message
	if(!this->postMessageToTask(&msg, timeout)) {
		LOG(S_ERROR, "Couldn't process received ARP packet");
	}
}

/**
 * Posts a task message to the ARP task.
 */
bool ARP::postMessageToTask(void *msg, int timeout) {
	BaseType_t ok;

	ok = xQueueSendToBack(this->messageQueue, msg, timeout);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't send message to task: %u", ok);
		return false;
	}

	return true;
}

} /* namespace ip */
