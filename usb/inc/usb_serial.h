/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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

/**
 * @brief Wirish USB virtual serial port (SerialUSB).
 */

#ifndef _WIRISH_USB_SERIAL_H_
#define _WIRISH_USB_SERIAL_H_

#include <stm32f10x.h>

/**
 * @brief Virtual serial terminal.
 */
class USBSerial {
public:
    USBSerial(void);

    void begin(void);
    void end(void);

    virtual int available(void);// Changed to virtual

    uint32_t read(uint8_t * buf, uint32_t len);

	// Roger Clark. added functions to support Arduino 1.0 API
    virtual int peek(void);
    virtual int read(void);
    int availableForWrite(void);
    virtual void flush(void);
	
    uint32_t write(uint8_t);
    uint32_t write(const uint8_t*, uint32_t);

    uint8_t getRTS();
    uint8_t getDTR();
    uint8_t isConnected();
    uint8_t pending();
};

extern USBSerial usbserial;

#endif

