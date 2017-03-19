/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#ifndef _USB_REG_MAP_H_
#define _USB_REG_MAP_H_

#include <usb.h>
#include <usb_type.h>
#include <stm32f10x.h>

/* TODO:
 * - Pick one of "endp", "ep" "endpt"
 */

/*
 * Register map and base pointer
 */

#define USB_NR_EP_REGS                  8

/** USB register map type */
typedef struct usb_reg_map {
    __io uint32_t EP[USB_NR_EP_REGS]; /**< Endpoint registers */
    const uint32_t RESERVED[8];       /**< Reserved */
    __io uint32_t CNTR;               /**< Control register */
    __io uint32_t ISTR;               /**< Interrupt status register */
    __io uint32_t FNR;                /**< Frame number register */
    __io uint32_t DADDR;              /**< Device address */
    __io uint32_t BTABLE;             /**< @brief Buffer table address
                                     *
                                     * Address offset within the USB
                                     * packet memory area which points
                                     * to the base of the buffer
                                     * descriptor table.  Must be
                                     * aligned to an 8 byte boundary.
                                     */
} usb_reg_map;

/** USB register map base pointer */
#define USB_BASE                        ((struct usb_reg_map*)0x40005C00)

/*
 * Register bit definitions
 */

/* Endpoint registers (USB_EPnR) */

#define USB_EP_CTR_RX_BIT              15
#define USB_EP_DTOG_RX_BIT             14
#define USB_EP_SETUP_BIT               11
#define USB_EP_EP_KIND_BIT             8
#define USB_EP_CTR_TX_BIT              7
#define USB_EP_DTOG_TX_BIT             6

#define USB_EP_CTR_RX                  BIT(USB_EP_CTR_RX_BIT)
#define USB_EP_DTOG_RX                 BIT(USB_EP_DTOG_RX_BIT)
#define USB_EP_STAT_RX                 (0x3 << 12)
#define USB_EP_STAT_RX_DISABLED        (0x0 << 12)
#define USB_EP_STAT_RX_STALL           (0x1 << 12)
#define USB_EP_STAT_RX_NAK             (0x2 << 12)
#define USB_EP_STAT_RX_VALID           (0x3 << 12)
#define USB_EP_SETUP                   BIT(USB_EP_SETUP_BIT)
#define USB_EP_EP_TYPE                 (0x3 << 9)
#define USB_EP_EP_TYPE_BULK            (0x0 << 9)
#define USB_EP_EP_TYPE_CONTROL         (0x1 << 9)
#define USB_EP_EP_TYPE_ISO             (0x2 << 9)
#define USB_EP_EP_TYPE_INTERRUPT       (0x3 << 9)
#define USB_EP_EP_KIND                 BIT(USB_EP_EP_KIND_BIT)
#define USB_EP_EP_KIND_DBL_BUF         (0x1 << USB_EP_EP_KIND_BIT)
#define USB_EP_CTR_TX                  BIT(USB_EP_CTR_TX_BIT)
#define USB_EP_DTOG_TX                 BIT(USB_EP_DTOG_TX_BIT)
#define USB_EP_STAT_TX                 (0x3 << 4)
#define USB_EP_STAT_TX_DISABLED        (0x0 << 4)
#define USB_EP_STAT_TX_STALL           (0x1 << 4)
#define USB_EP_STAT_TX_NAK             (0x2 << 4)
#define USB_EP_STAT_TX_VALID           (0x3 << 4)
#define USB_EP_EA                      0xF

/*
 * Register convenience routines
 */

#define __EP_CTR_NOP                    (USB_EP_CTR_RX | USB_EP_CTR_TX)
#define __EP_NONTOGGLE                  (USB_EP_CTR_RX | USB_EP_SETUP |    \
                                         USB_EP_EP_TYPE | USB_EP_EP_KIND | \
                                         USB_EP_CTR_TX | USB_EP_EA)

static inline void usb_clear_ctr_rx(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    USB_BASE->EP[ep] = epr & ~USB_EP_CTR_RX & __EP_NONTOGGLE;
}

static inline void usb_clear_ctr_tx(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    USB_BASE->EP[ep] = epr & ~USB_EP_CTR_TX & __EP_NONTOGGLE;
}

static inline uint32_t usb_get_ep_dtog_tx(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    return epr & USB_EP_DTOG_TX;
}

static inline uint32_t usb_get_ep_dtog_rx(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    return epr & USB_EP_DTOG_RX;
}

static inline uint32_t usb_get_ep_tx_sw_buf(uint8_t ep) {
    return usb_get_ep_dtog_rx(ep);
}

static inline uint32_t usb_get_ep_rx_sw_buf(uint8_t ep) {
    return usb_get_ep_dtog_tx(ep);
}

static inline void usb_toggle_ep_dtog_tx(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    epr &= __EP_NONTOGGLE;
    epr |= USB_EP_DTOG_TX;
    USB_BASE->EP[ep] = epr;
}

static inline void usb_toggle_ep_dtog_rx(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    epr &= __EP_NONTOGGLE;
    epr |= USB_EP_DTOG_RX;
    USB_BASE->EP[ep] = epr;
}

static inline void usb_clear_ep_dtog_tx(uint8_t ep) {
    if (usb_get_ep_dtog_tx(ep) != 0) {
        usb_toggle_ep_dtog_tx(ep);
    }
}

static inline void usb_clear_ep_dtog_rx(uint8_t ep) {
    if (usb_get_ep_dtog_rx(ep) != 0) {
        usb_toggle_ep_dtog_rx(ep);
    }
}

static inline void usb_set_ep_dtog_tx(uint8_t ep) {
    if (usb_get_ep_dtog_tx(ep) == 0) {
        usb_toggle_ep_dtog_tx(ep);
    }
}

static inline void usb_set_ep_dtog_rx(uint8_t ep) {
    if (usb_get_ep_dtog_rx(ep) == 0) {
        usb_toggle_ep_dtog_rx(ep);
    }
}

static inline void usb_toggle_ep_rx_sw_buf(uint8_t ep) {
    usb_toggle_ep_dtog_tx(ep);
}

static inline void usb_toggle_ep_tx_sw_buf(uint8_t ep) {
    usb_toggle_ep_dtog_rx(ep);
}

static inline void usb_clear_ep_rx_sw_buf(uint8_t ep) {
    usb_clear_ep_dtog_tx(ep);
}

static inline void usb_clear_ep_tx_sw_buf(uint8_t ep) {
    usb_clear_ep_dtog_rx(ep);
}

static inline void usb_set_ep_rx_sw_buf(uint8_t ep) {
    usb_set_ep_dtog_tx(ep);
}

static inline void usb_set_ep_tx_sw_buf(uint8_t ep) {
    usb_set_ep_dtog_rx(ep);
}

static inline void usb_set_ep_rx_stat(uint8_t ep, uint32_t status) {
    uint32_t epr = USB_BASE->EP[ep];
    epr &= ~(USB_EP_STAT_TX | USB_EP_DTOG_RX | USB_EP_DTOG_TX);
    epr |= __EP_CTR_NOP;
    epr ^= status;
    USB_BASE->EP[ep] = epr;
}

static inline void usb_set_ep_tx_stat(uint8_t ep, uint32_t status) {
    uint32_t epr = USB_BASE->EP[ep];
    epr &= ~(USB_EP_STAT_RX | USB_EP_DTOG_RX | USB_EP_DTOG_TX);
    epr |= __EP_CTR_NOP;
    epr ^= status;
    USB_BASE->EP[ep] = epr;
}

static inline void usb_set_ep_type(uint8_t ep, uint32_t type) {
    uint32_t epr = USB_BASE->EP[ep];
    epr &= ~USB_EP_EP_TYPE & __EP_NONTOGGLE;
    epr |= type;
    USB_BASE->EP[ep] = epr;
}

static inline void usb_set_ep_kind(uint8_t ep, uint32_t kind) {
    uint32_t epr = USB_BASE->EP[ep];
    epr &= ~USB_EP_EP_KIND & __EP_NONTOGGLE;
    epr |= kind;
    USB_BASE->EP[ep] = epr;
}

static inline uint32_t usb_get_ep_type(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    return epr & USB_EP_EP_TYPE;
}

static inline uint32_t usb_get_ep_kind(uint8_t ep) {
    uint32_t epr = USB_BASE->EP[ep];
    return epr & USB_EP_EP_TYPE;
}


static inline void usb_clear_status_out(uint8_t ep) {
    usb_set_ep_kind(ep, 0);
}

/*
 * Packet memory area (PMA) base pointer
 */

/**
 * @brief USB packet memory area (PMA) base pointer.
 *
 * The USB PMA is SRAM shared between USB and CAN.  The USB peripheral
 * accesses this memory directly via the packet buffer interface.  */
#define USB_PMA_BASE                    ((__io void*)0x40006000)

/*
 * PMA conveniences
 */
/*
void usb_copy_to_pma(const uint8_t *buf, uint16_t len, uint16_t pma_offset);
void usb_copy_from_pma(uint8_t *buf, uint16_t len, uint16_t pma_offset);
*/
static inline uint32_t * usb_pma_ptr(uint32_t offset) {
    return (uint32_t*)(USB_PMA_BASE + 2 * offset);
}

/*
 * BTABLE
 */

/* (Forward-declared) BTABLE entry.
 *
 * The BTABLE can be viewed as an array of usb_btable_ent values;
 * these vary in structure according to the configuration of the
 * endpoint.
 */
union usb_btable_ent;

/* Bidirectional endpoint BTABLE entry */
typedef struct usb_btable_bidi {
    __io uint16_t addr_tx;     const uint16_t PAD1;
    __io uint16_t count_tx;    const uint16_t PAD2;
    __io uint16_t addr_rx;     const uint16_t PAD3;
    __io uint16_t count_rx;    const uint16_t PAD4;
} usb_btable_bidi;

/* Unidirectional receive-only endpoint BTABLE entry */
typedef struct usb_btable_uni_rx {
    __io uint16_t empty1;      const uint16_t PAD1;
    __io uint16_t empty2;      const uint16_t PAD2;
    __io uint16_t addr_rx;     const uint16_t PAD3;
    __io uint16_t count_rx;    const uint16_t PAD4;
} usb_btable_uni_rx;

/* Unidirectional transmit-only endpoint BTABLE entry */
typedef struct usb_btable_uni_tx {
    __io uint16_t addr_tx;     const uint16_t PAD1;
    __io uint16_t count_tx;    const uint16_t PAD2;
    __io uint16_t empty1;      const uint16_t PAD3;
    __io uint16_t empty2;      const uint16_t PAD4;
} usb_btable_uni_tx;

/* Double-buffered transmission endpoint BTABLE entry */
typedef struct usb_btable_dbl_tx {
    __io uint16_t addr_tx0;     const uint16_t PAD1;
    __io uint16_t count_tx0;    const uint16_t PAD2;
    __io uint16_t addr_tx1;     const uint16_t PAD3;
    __io uint16_t count_tx1;    const uint16_t PAD4;
} usb_btable_dbl_tx;

/* Double-buffered reception endpoint BTABLE entry */
typedef struct usb_btable_dbl_rx {
    __io uint16_t addr_rx0;     const uint16_t PAD1;
    __io uint16_t count_rx0;    const uint16_t PAD2;
    __io uint16_t addr_rx1;     const uint16_t PAD3;
    __io uint16_t count_rx1;    const uint16_t PAD4;
} usb_btable_dbl_rx;

/* TODO isochronous endpoint entries */

/* Definition for above forward-declared BTABLE entry. */
typedef union usb_btable_ent {
    usb_btable_bidi   bidi;
    usb_btable_uni_rx u_rx;
    usb_btable_uni_tx u_tx;
    usb_btable_dbl_tx d_tx;
    usb_btable_dbl_rx d_rx;
} usb_btable_ent;

/*
 * BTABLE conveniences
 */

/* TODO (?) Convert usages of the many (and lengthily-named)
 * accessors/mutators below to just manipulating usb_btable_entry
 * values.  */

static inline uint32_t* usb_btable_ptr(uint32_t offset) {
    return (uint32_t*)usb_pma_ptr(USB_BASE->BTABLE + offset);
}

/* TX address */

static inline uint32_t* usb_ep_tx_addr_ptr(uint8_t ep) {
    return usb_btable_ptr(ep * 8);
}

static inline uint16_t usb_get_ep_tx_addr(uint8_t ep) {
    return (uint16_t)*usb_ep_tx_addr_ptr(ep);
}

static inline void usb_set_ep_tx_addr(uint8_t ep, uint16_t addr) {
    volatile uint32_t *tx_addr = usb_ep_tx_addr_ptr(ep);
    *tx_addr = addr & ~0x1;
}

/* RX address */

static inline uint32_t* usb_ep_rx_addr_ptr(uint8_t ep) {
    return usb_btable_ptr(ep * 8 + 4);
}

static inline uint16_t usb_get_ep_rx_addr(uint8_t ep) {
    return (uint16_t)*usb_ep_rx_addr_ptr(ep);
}

static inline void usb_set_ep_rx_addr(uint8_t ep, uint16_t addr) {
    volatile uint32_t *rx_addr = usb_ep_rx_addr_ptr(ep);
    *rx_addr = addr & ~0x1;
}

/* TX count (doesn't cover double-buffered and isochronous in) */

static inline uint32_t* usb_ep_tx_count_ptr(uint8_t ep) {
    return usb_btable_ptr(ep * 8 + 2);
}

static inline uint16_t usb_get_ep_tx_count(uint8_t ep) {
    /* FIXME: this is broken somehow; calling it seems to
     * confuse/crash the chip. */
    return (uint16_t)(*usb_ep_tx_count_ptr(ep) & 0x3FF);
}

static inline void usb_set_ep_tx_count(uint8_t ep, uint16_t count) {
    volatile uint32_t *txc = usb_ep_tx_count_ptr(ep);
    *txc = count;
}

/* RX count */

static inline uint32_t* usb_ep_rx_count_ptr(uint8_t ep) {
    return usb_btable_ptr(ep * 8 + 6);
}

static inline uint16_t usb_get_ep_rx_count(uint8_t ep) {
    return (uint16_t)*usb_ep_rx_count_ptr(ep) & 0x3FF;
}

void usb_set_ep_rx_count(uint8_t ep, uint16_t count);

/* double buffer definitions */
static inline uint32_t* usb_get_ep_tx_buf0_addr_ptr(uint8_t ep) {
    return usb_ep_tx_addr_ptr(ep);
}

static inline uint16_t usb_get_ep_tx_buf0_addr(uint8_t ep) {
    return usb_get_ep_tx_addr(ep);
}

static inline void usb_set_ep_tx_buf0_addr(uint8_t ep, uint16_t addr) {
    usb_set_ep_tx_addr(ep, addr);
}

static inline uint32_t* usb_get_ep_tx_buf1_addr_ptr(uint8_t ep) {
    return usb_ep_rx_addr_ptr(ep);
}

static inline uint16_t usb_get_ep_tx_buf1_addr(uint8_t ep)  {
    return usb_get_ep_rx_addr(ep);
}

static inline void usb_set_ep_tx_buf1_addr(uint8_t ep, uint16_t addr) {
    usb_set_ep_rx_addr(ep, addr);
}

static inline uint32_t* usb_ep_tx_buf0_count_ptr(uint8_t ep) {
    return usb_ep_tx_count_ptr(ep);
}

static inline uint16_t usb_get_ep_tx_buf0_count(uint8_t ep) {
    return usb_get_ep_tx_count(ep);
}

static inline void usb_set_ep_tx_buf0_count(uint8_t ep, uint16_t count) {
    usb_set_ep_tx_count(ep, count);
}

static inline uint32_t* usb_ep_tx_buf1_count_ptr(uint8_t ep) {
    return usb_ep_rx_count_ptr(ep);
}

static inline uint16_t usb_get_ep_tx_buf1_count(uint8_t ep) {
    return usb_get_ep_rx_count(ep);
}

static inline void usb_set_ep_tx_buf1_count(uint8_t ep, uint16_t count) {
    usb_set_ep_rx_count(ep, count);
}
static inline uint32_t* usb_get_ep_rx_buf0_addr_ptr(uint8_t ep) {
    return usb_ep_tx_addr_ptr(ep);
}

static inline uint16_t usb_get_ep_rx_buf0_addr(uint8_t ep) {
    return usb_get_ep_tx_addr(ep);
}

static inline void usb_set_ep_rx_buf0_addr(uint8_t ep, uint16_t addr) {
    usb_set_ep_tx_addr(ep, addr);
}

static inline uint32_t* usb_get_ep_rx_buf1_addr_ptr(uint8_t ep) {
    return usb_ep_rx_addr_ptr(ep);
}

static inline uint16_t usb_get_ep_rx_buf1_addr(uint8_t ep) {
    return usb_get_ep_rx_addr(ep);
}

static inline void usb_set_ep_rx_buf1_addr(uint8_t ep, uint16_t addr) {
    usb_set_ep_rx_addr(ep, addr);
}

static inline uint32_t* usb_ep_rx_buf0_count_ptr(uint8_t ep) {
    return usb_ep_tx_count_ptr(ep);
}

static inline uint16_t usb_get_ep_rx_buf0_count(uint8_t ep) {
    return usb_get_ep_tx_count(ep);
}

//void usb_set_ep_rx_buf0_count(uint8_t ep, uint16_t count);

static inline uint32_t* usb_ep_rx_buf1_count_ptr(uint8_t ep) {
    return usb_ep_rx_count_ptr(ep);
}

static inline uint16_t usb_get_ep_rx_buf1_count(uint8_t ep) {
    return usb_get_ep_rx_count(ep);
}

static inline void usb_set_ep_rx_buf1_count(uint8_t ep, uint16_t count) {
    usb_set_ep_rx_count(ep, count);
}

/*
 * Misc. types
 */

typedef enum usb_ep {
    USB_EP0,
    USB_EP1,
    USB_EP2,
    USB_EP3,
    USB_EP4,
    USB_EP5,
    USB_EP6,
    USB_EP7,
} usb_ep;

typedef enum usb_ep_type {
    USB_EP_T_CTL   = USB_EP_EP_TYPE_CONTROL,
    USB_EP_T_BULK  = USB_EP_EP_TYPE_BULK,
    USB_EP_T_INT   = USB_EP_EP_TYPE_INTERRUPT,
    USB_EP_T_ISO   = USB_EP_EP_TYPE_ISO
} usb_ep_type;

typedef enum usb_ep_stat {
    USB_EP_ST_RX_DIS = USB_EP_STAT_RX_DISABLED,
    USB_EP_ST_RX_STL = USB_EP_STAT_RX_STALL,
    USB_EP_ST_RX_NAK = USB_EP_STAT_RX_NAK,
    USB_EP_ST_RX_VAL = USB_EP_STAT_RX_VALID,
    USB_EP_ST_TX_DIS = USB_EP_STAT_TX_DISABLED,
    USB_EP_ST_TX_STL = USB_EP_STAT_TX_STALL,
    USB_EP_ST_TX_NAK = USB_EP_STAT_TX_NAK,
    USB_EP_ST_TX_VAL = USB_EP_STAT_TX_VALID
} usb_ep_stat;

#endif
