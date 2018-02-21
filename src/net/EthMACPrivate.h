/*
 * EthMACPrivate.h
 *
 *  Created on: Feb 21, 2018
 *      Author: tristan
 */

#ifndef NET_ETHMACPRIVATE_H_
#define NET_ETHMACPRIVATE_H_

#include <cstdint>

namespace net {

/**
 * Defines a TX DMA buffer descriptor.
 */
typedef struct __attribute__((__packed__)) {
	// status bit: see defines below
	volatile uint32_t status;

	// buffer size (buf 2 is in the high word, buf 1 in the low)
	volatile uint32_t bufSz;

	// address of buffer 1
	volatile uint32_t buf1Address;
	// address of buffer 2
	volatile uint32_t buf2Address;
} mac_tx_dma_descriptor_t;

// set when the DMA owns the buffer
#define TX_STATUS_DMA_OWNS_BUFFER			(1 << 31)

// generate an interrupt when this descriptor has been transmitted
#define TX_STATUS_DMA_IRQ_ON_COMPLETE		(1 << 30)

// this buffer contains the last segment of a frame
#define TX_STATUS_DMA_LAST_SEGMENT			(1 << 29)
// this buffer contains the first segment of a frame
#define TX_STATUS_DMA_FIRST_SEGMENT			(1 << 28)

// prevent the MAC from adding a CRC to this frame
#define TX_STATUS_DMA_NO_CRC					(1 << 27)
// disable padding the frame to the minimum length of 64 bytes
#define TX_STATUS_DMA_NO_PAD					(1 << 26)

// set the time stamp in the descriptor after transmission (IEEE1588)
#define TX_STATUS_DMA_IEEE1588				(1 << 25)

// checksum insertion control
#define TX_STATUS_DMA_CIC_MASK				((1 << 23) | (1 << 22))
// insert IP header and payload checksums, and also pseudo-header checksums
#define TX_STATUS_DMA_CIC_ALL				((1 << 23) | (1 << 22))

// indicates the end of the descriptor list
#define TX_STATUS_DMA_END_OF_LIST			(1 << 21)
// when set, the next descriptor is pointed to by the second buffer
#define TX_STATUS_DMA_NEXT_CHAINED			(1 << 20)

// when set, the MAC captured a time stamp and wrote it to the pointer fields
#define TX_STATUS_DMA_TIMESTAMP_CAPTURED		(1 << 17)
// indicates that an IP header error occurred
#define TX_STATUS_DMA_IP_HEADER_ERR			(1 << 16)

// indicates that an error occurred sending this frame
#define TX_STATUS_DMA_ERR					(1 << 15)
// MAC experienced a jabber timeout
#define TX_STATUS_DMA_ERR_JABBER				(1 << 14)
// DMA flushed the frame due to a software flush command from the CPU
#define TX_STATUS_DMA_ERR_FLUSH				(1 << 13)
// An error was detected in the TCP/UDP/ICMP packet
#define TX_STATUS_DMA_ERR_IP_PAYLOAD			(1 << 12)
// The Ethernet carrier was lost during transmission
#define TX_STATUS_DMA_ERR_CARRIER_LOST		(1 << 11)
// The Ethernet carrier was not asserted (i.e. there is no carrier)
#define TX_STATUS_DMA_ERR_NO_CARRIER			(1 << 10)
// A late collision occurred (after 64 byte times)
#define TX_STATUS_DMA_ERR_LATE_COL			(1 << 9)
// More than 16 collisions occurred while sending this frame
#define TX_STATUS_DMA_ERR_EXCESS_COL			(1 << 8)
// mask for the collision counter
#define TX_STATUS_DMA_ERR_COL_COUNTER		(0x78)
// Transmission was deferred for over 24,288 bit times
#define TX_STATUS_DMA_ERR_EXCESS_DEFER		(1 << 2)
// MAC aborted frame transmission because data was not available quickly enough
#define TX_STATUS_DMA_ERR_UNDERFLOW			(1 << 1)
// MAC deferred the transmission because a carrier is present in half-duplex
#define TX_STATUS_DMA_ERR_TX_DEFERRED		(1 << 0)

// indicates that the transmitted frame had a VLAN tag
#define TX_STATUS_DMA_VLAN_TAG				(1 << 7)



/**
 * Defines an RX DMA buffer descriptor.
 */
typedef struct __attribute__((__packed__)) {
	// status bit: see defines below
	volatile uint32_t status;

	// buffer size (buf 2 is in the high word, buf 1 in the low)
	volatile uint32_t bufSz;

	// address of buffer 1
	volatile uint32_t buf1Address;
	// address of buffer 2
	volatile uint32_t buf2Address;
} mac_rx_dma_descriptor_t;

// set when the DMA owns the buffer
#define RX_STATUS_DMA_OWNS_BUFFER			(1 << 31)
// if the frame did not pass destination address filtering, this bit is set
#define RX_STATUS_DMA_DA_FAIL				(1 << 30)


// frame length mask
#define RX_STATUS_DMA_FRAME_LENGTH_MASK		(0x3FFF0000)
// bits to shift the frame length
#define RX_STATUS_DMA_FRAME_LENGTH_SHIFT		(0x10)

// set if an error occurred during reception
#define RX_STATUS_DMA_ERR					(1 << 15)
// descriptor error: usually indicates that the buffer was too small
#define RX_STATUS_DMA_ERR_TRUNCATED			(1 << 14)
// indicates that the source address filter failed
#define RX_STATUS_DMA_ERR_SA_FAIL			(1 << 13)
// indicates that the length of the frame is incorrect
#define RX_STATUS_DMA_ERR_LENGTH_MISMATCH	(1 << 12)
// received frame was damaged due to buffer overflow
#define RX_STATUS_DMA_ERR_OVERFLOW			(1 << 11)
// indicates the IP header checksum is incorrect
#define RX_STATUS_DMA_ERR_IP_CHECKSUM		(1 << 7)
// receive watchdog expired during reception; frame is truncated
#define RX_STATUS_DMA_ERR_WATCHDOG			(1 << 4)
// the PHY asserted the RX_ERR signal during reception
#define RX_STATUS_DMA_ERR_PHY				(1 << 3)
// dribble error (received frame has an odd number of nybbles)
#define RX_STATUS_DMA_ERR_DRIBBLE			(1 << 2)
// received CRC is invalid
#define RX_STATUS_DMA_ERR_CRC				(1 << 1)
// payload checksum (TCP/UDP/ICMP) is incorrect
#define RX_STATUS_DMA_ERR_PAYLOAD_CHECKSUM	(1 << 0)

// the frame has been VLAN tagged
#define RX_STATUS_DMA_VLAN_TAGGED			(1 << 10)

// set if this is the first descriptor for this frame
#define RX_STATUS_DMA_FIRST					(1 << 9)
// set if this is the last descriptor for this frame
#define RX_STATUS_DMA_LAST					(1 << 8)

// indicates that the received frame is an Ethernet frame (LT >= 0x0600)
#define RX_STATUS_DMA_FT						(1 << 5)


// set in the size buf to inhibit receive interrupts
#define RX_BUFSZ_DISABLE_IRQ					(1 << 31)
// indicates the end of the descriptor list
#define RX_BUFSZ_END_OF_LIST					(1 << 15)
// indicates the buffer 2 pointer is the address of the next descriptor
#define RX_BUFSZ_NEXT_CHAINED				(1 << 14)
}

#endif /* NET_ETHMACPRIVATE_H_ */
