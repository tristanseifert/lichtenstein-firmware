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

// produce logging output for received ARP messages
#define LOG_RECEIVED_ARP						0
// produce logging output when we send an ARP request/reply
#define LOG_SENT_ARP							1
// log caching
#define LOG_CACHING							1
// produce logging output for when a task is notified
#define LOG_RESOLVE_NOTIFICATIONS			0

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
	BaseType_t ok;

	// clear ARP cache
	size_t cacheLength = sizeof(this->cache);
	memset(this->cache, 0, cacheLength);

	LOG(S_INFO, "ARP cache has %u entries", ARP::cacheEntries);

	// clear notifications list
	cacheLength = sizeof(this->notifications);
	memset(this->notifications, 0, cacheLength);

	LOG(S_INFO, "Notifications list has %u entries", ARP::notificationsEntries);


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
 * Dumps the ARP cache.
 */
void ARP::dbgDumpCache(void) {
	char ipStr[16], macStr[18];

	// iterate over each element
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// ignore invalid entries
		if(this->cache[i].valid == false) {
			continue;
		}

		// convert the IP and MAC
		Stack::ipToString(this->cache[i].ip, ipStr, 16);
		Stack::macToString(this->cache[i].mac, macStr, 18);

		LOG(S_DEBUG, "Entry %u: %s = %s, age %u", i, ipStr, macStr,
				this->cache[i].age);
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
#if LOG_CACHING || LOG_RESOLVE_NOTIFICATIONS
	char ipStr[16];
#endif
#if LOG_CACHING
	char macStr[18];
#endif

	// check if we have an entry for this IP, and update it
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// if the IP matches, we're done
		if(this->cache[i].valid && this->cache[i].ip == addr) {
			this->cache[i].mac = mac;

#if LOG_CACHING
			Stack::ipToString(addr, ipStr, 16);
			Stack::macToString(mac, macStr, 18);

			LOG(S_DEBUG, "Updated cache entry: %s = %s", macStr, ipStr);
#endif

			// notify tasks and exit
			goto notify;
		}
	}

	// if we get here, we're going to need to insert a new entry
#if LOG_CACHING
	Stack::ipToString(addr, ipStr, 16);
	Stack::macToString(mac, macStr, 18);

	LOG(S_DEBUG, "Inserting into cache: %s = %s", ipStr, macStr);
#endif

	// free up an entry in the cache if it's full
	if(this->numFreeCacheEntries() == 0) {
		this->discardOldestEntry();
	}

	// find the first free entry and insert it
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		// is this entry free?
		if(this->cache[i].valid == false) {
			// if so, insert it
			this->cache[i].age = xTaskGetTickCount();

			this->cache[i].ip = addr;
			this->cache[i].mac = mac;

			this->cache[i].valid = true;

			// notify tasks and exit
			goto notify;
		}
	}

	// we shouldn't get down here
	LOG(S_ERROR, "Couldn't insert into ARP cache: no space in cache (wtf)");
	return;

	// notify any waiting tasks that the IP is available
notify: ;
	for(size_t i = 0; i < ARP::notificationsEntries; i++) {
		// is this entry valid?
		if(this->notifications[i].valid) {
			// does the IP match?
			if(this->notifications[i].address == addr) {
				// logging
#if LOG_RESOLVE_NOTIFICATIONS
				Stack::ipToString(this->notifications[i].address, ipStr, 16);
				LOG(S_DEBUG, "Notifying task waiting on %s: 0x%x", ipStr,
						this->notifications[i].completion);
#endif

				// give semaphore and mark the entry as invalid
				xSemaphoreGive(this->notifications[i].completion);


				// clear this entry
				this->notifications[i].valid = false;

				this->notifications[i].completion = nullptr;
				this->notifications[i].address = kIPv4AddressZero;
			}
		}
	}
}



/**
 * Handles a received ARP frame.
 */
void ARP::handleARPFrame(void *p) {
	stack_rx_packet_t *packet = (stack_rx_packet_t *) p;

	// first, swap all the multi-byte fields
	arp_ipv4_packet_t *arp = (arp_ipv4_packet_t *) packet->payload;
	this->packetNetworkToHost(arp);

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
	arp_task_message_t msg;

	msg.type = kARPMessageSendGratuitous;

	// send the message
	if(!this->postMessageToTask(&msg, 0)) {
		LOG(S_ERROR, "Couldn't request ARP resolution");
	}
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
#if LOG_RECEIVED_ARP
		// debug logging
		char ip[16];
		Stack::ipToString(arp->targetIP, ip, 16);
		LOG(S_DEBUG, "Received gratuitous ARP for address %s", ip);
#endif

		// if so, let the ARP task handle it
		this->sendReceivedPacketToTask(arp, 0);
		return;
	}

#if LOG_RECEIVED_ARP
	// debug logging
	char ip[16];
	Stack::ipToString(arp->targetIP, ip, 16);
	LOG(S_DEBUG, "Who has %s?", ip);
#endif

	// is this a request for our MAC address, and our IP config is valid?
	if(this->stack->isIPv4ConfigValid &&
			arp->targetIP == this->stack->getIPAddress()) {
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

/*	// ignore gratuitious ARP
	if(arp->senderMAC == arp->targetMAC) {
#if LOG_RECEIVED_ARP
		// debug logging
		char ip[16];
		Stack::ipToString(arp->targetIP, ip, 16);
		LOG(S_DEBUG, "Ignoring gratuitous ARP for address %s", ip);
#endif

		return;
	}*/

#if LOG_RECEIVED_ARP
	// debug logging
	char ip[16];
	Stack::ipToString(arp->senderIP, ip, 16);
	LOG(S_DEBUG, "Received ARP Reply from address %s", ip);
#endif

	// process the received frame later
	this->sendReceivedPacketToTask(arp, 0);
}



/**
 * Resolves an IPv4 address to a MAC address. When this method is called, it
 * will:
 *
 * 1. Check the ARP cache for the IP address. If the IP is in there, return
 *    immediately.
 * 2. If the IP could not be found in the cache, send an ARP request for that
 *    IP address and wait up to timeout ticks for a response. If no response
 *    is received, assume the host doesn't exist.
 */
bool ARP::resolveIPv4(stack_ipv4_addr_t addr, stack_mac_addr_t *result, int timeout) {
//	this->dbgDumpCache();

	// is it in the cache?
	for(size_t i = 0; i < ARP::cacheEntries; i++) {
		if(this->cache[i].valid && this->cache[i].ip == addr) {
			*result = this->cache[i].mac;
			return true;
		}
	}

	// if not, prepare to resole the IP: check if the notify queue has space
	bool notificationsSpaceAvailable = false;

	for(size_t i = 0; i < ARP::notificationsEntries; i++) {
		if(this->notifications[i].valid == false) {
			notificationsSpaceAvailable = true;
			break;
		}
	}

	if(!notificationsSpaceAvailable) {
		LOG(S_ERROR, "Can't resolve IP: no notifications space available");
		return false;
	}

	// allocate a mutex to signal and build the message
	SemaphoreHandle_t completion = xSemaphoreCreateBinary();

	if(completion == nullptr) {
		LOG(S_ERROR, "Couldn't allocate completion semaphore");
		return false;
	}


	arp_task_message_t msg;

	msg.type = kARPMessageResolveIP;
	msg.payload.request.address = addr;
	msg.payload.request.completion = completion;

	// send the message
	if(!this->postMessageToTask(&msg, timeout)) {
		LOG(S_ERROR, "Couldn't request ARP resolution");
	}

	// wait on the mutex
#if LOG_RESOLVE_NOTIFICATIONS
	LOG(S_DEBUG, "Waiting on semaphore 0x%x", completion);
#endif

	if(xSemaphoreTake(completion, timeout) == pdTRUE) {
#if LOG_RESOLVE_NOTIFICATIONS
		char ipStr[16];
		Stack::ipToString(addr, ipStr, 16);

		LOG(S_DEBUG, "ARP request for %s complete", ipStr);
#endif

		// we can delete the semaphore now
		vSemaphoreDelete(completion);

		// find the address in the cache
		for(size_t i = 0; i < ARP::cacheEntries; i++) {
			if(this->cache[i].ip == addr) {
				*result = this->cache[i].mac;
				return true;
			}
		}

		// we shouldn't get here... but hey, who knows
		LOG(S_ERROR, "ARP request completed but MAC wasn't found");
		return false;
	} else {
		// we timed out: remove ourselves from the notification queue
		for(size_t i = 0; i < ARP::notificationsEntries; i++) {
			// does the IP match?
			if(this->notifications[i].address == addr) {
				// mark entry as invalid and deallocate semaphore
				this->notifications[i].valid = false;
				vSemaphoreDelete(completion);

				// early return
				return false;
			}
		}

		LOG(S_ERROR, "Wait timed out but we're not in the notification queue");

		// return error
		return false;
	}

	// we should NEVER get down here
	return false;
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
				this->insertAddress(msg.payload.learn.mac,
						msg.payload.learn.address);
				break;

			// send a reply to a request for our MAC
			case kARPMessageSendReply:
				this->taskGenerateResponse(&msg);
				break;

			// request the MAC address for an IP
			case kARPMessageResolveIP:
				this->taskResolveIP(&msg);
				break;

			// transmit a gratuitous ARP for our address
			case kARPMessageSendGratuitous:
				this->taskSendGratuitous(&msg);
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
	arp_ipv4_packet_t *reply = (arp_ipv4_packet_t *) tx->payload;

#if LOG_SENT_ARP
	// debugging
	char mac[18];
	Stack::macToString(msg->payload.sendResponse.mac, mac, 18);
	LOG(S_DEBUG, "Sending ARP reply to %s", mac);
#endif

	reply->hwType = ARP_HW_ETHERNET;
	reply->protoType = kProtocolIPv4;

	reply->hwAddressLength = sizeof(stack_mac_addr_t);
	reply->protoAddressLength = sizeof(stack_ipv4_addr_t);

	reply->op = ARP_OP_REPLY;

	// target MAC/IP are that of the original requestor
	reply->targetMAC = msg->payload.sendResponse.mac;
	reply->targetIP = msg->payload.sendResponse.address;

	// sender MAC/IP are OUR values
	reply->senderMAC = this->stack->mac;
	reply->senderIP = this->stack->getIPAddress();

	// byteswap all multibyte fields and transmit
	this->packetHostToNetwork(reply);
	this->stack->sendTxPacket(tx, msg->payload.sendResponse.mac, kProtocolARP);
}

/**
 * Sends an ARP request for the given IP address.
 */
void ARP::taskResolveIP(void *_msg) {
	stack_tx_packet_t *tx;
	arp_task_message_t *msg = (arp_task_message_t *) _msg;

	// request a transmit buffer
	const size_t payloadLength = sizeof(arp_ipv4_packet_t);
	tx = (stack_tx_packet_t *) this->stack->getTxPacket(payloadLength);

	if(tx == nullptr) {
		LOG(S_ERROR, "Couldn't get TX buffer for ARP request!");
		return;
	}

	// try to insert the caller into the notify queue
	bool addedToNotifyQueue = false;

	for(size_t i = 0; i < ARP::notificationsEntries; i++) {
		// is this entry invalid?
		if(this->notifications[i].valid == false) {
			// if so, populate it
			this->notifications[i].valid = true;
			this->notifications[i].address = msg->payload.request.address;
			this->notifications[i].completion = msg->payload.request.completion;

			addedToNotifyQueue = true;
			break;
		}
	}

	if(!addedToNotifyQueue) {
		LOG(S_ERROR, "Couldn't add to notify queue: insufficient space");

		// notify caller
		xSemaphoreGive(msg->payload.request.completion);

		// discard the packet
		this->stack->discardTXPacket(tx);
		return;
	}


	// populate the payload
	arp_ipv4_packet_t *reply = (arp_ipv4_packet_t *) tx->payload;

#if LOG_SENT_ARP
	// debugging
	char ipStr[16];
	Stack::ipToString(msg->payload.request.address, ipStr, 16);
	LOG(S_DEBUG, "Sending ARP request for %s", ipStr);
#endif

	reply->hwType = ARP_HW_ETHERNET;
	reply->protoType = kProtocolIPv4;

	reply->hwAddressLength = sizeof(stack_mac_addr_t);
	reply->protoAddressLength = sizeof(stack_ipv4_addr_t);

	reply->op = ARP_OP_REQUEST;

	// target IP is the IP we want, MAC is ignored
	memset(&reply->targetMAC, 0, sizeof(decltype(reply->targetMAC)));
	reply->targetIP = msg->payload.request.address;

	// sender MAC/IP are OUR values
	reply->senderMAC = this->stack->mac;
	reply->senderIP = this->stack->getIPAddress();

	// byteswap all multibyte fields and transmit
	this->packetHostToNetwork(reply);
	this->stack->broadcastTxPacket(tx, kProtocolARP);
}

/**
 * Transmits a gratuitous ARP. This uses the "ARP Request" method, which is
 * better supported with other devices.
 */
void ARP::taskSendGratuitous(void *_msg) {
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
	arp_ipv4_packet_t *reply = (arp_ipv4_packet_t *) tx->payload;

#if LOG_SENT_ARP
	LOG(S_DEBUG, "Sending gratuitous ARP");
#endif

	reply->hwType = ARP_HW_ETHERNET;
	reply->protoType = kProtocolIPv4;

	reply->hwAddressLength = sizeof(stack_mac_addr_t);
	reply->protoAddressLength = sizeof(stack_ipv4_addr_t);

	reply->op = ARP_OP_REQUEST;

	// target and sender protocol address are our addresses
	reply->targetIP = this->stack->getIPAddress();
	reply->senderIP = reply->targetIP;

	// sender hardware address is our MAC, target is empty
	reply->targetMAC = kMACAddressInvalid;
	reply->senderMAC = this->stack->mac;

	// byteswap all multibyte fields and transmit
	this->packetHostToNetwork(reply);
	this->stack->broadcastTxPacket(tx, kProtocolARP);
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

	msg.payload.learn.mac = packet->senderMAC;
	msg.payload.learn.address = packet->senderIP;

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

	msg.payload.sendResponse.mac = packet->senderMAC;
	msg.payload.sendResponse.address = packet->senderIP;

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



/**
 * Converts the byte order of a packet.
 */
void ARP::convertPacketByteOrder(void *_arp) {
	arp_ipv4_packet_t *arp = (arp_ipv4_packet_t *) _arp;

	arp->hwType = __builtin_bswap16(arp->hwType);
	arp->protoType = __builtin_bswap16(arp->protoType);
	arp->op = __builtin_bswap16(arp->op);
}

} /* namespace ip */
