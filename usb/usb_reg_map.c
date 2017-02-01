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

#include <usb_reg_map.h>

/* TODO these could use some improvement; they're fairly
 * straightforward ports of the analogous ST code.  The PMA blit
 * routines in particular are obvious targets for performance
 * measurement and tuning.
 */
 
static void usb_set_ep_rx_count_common(uint32_t *rxc, uint16_t count) {
    uint16_t nblocks;
    if (count > 62) {
        /* use 32-byte memory block size */
        nblocks = count >> 5;
        if ((count & 0x1F) == 0) {
            nblocks--;
        }
        *rxc = (nblocks << 10) | 0x8000;
    } else {
        /* use 2-byte memory block size */
        nblocks = count >> 1;
        if ((count & 0x1) != 0) {
            nblocks++;
        }
        *rxc = nblocks << 10;
    }
}

void usb_set_ep_rx_count(uint8_t ep, uint16_t count) {
    uint32_t *rxc = usb_ep_rx_count_ptr(ep);
    usb_set_ep_rx_count_common(rxc, count);
}
