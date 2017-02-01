/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010, 2011, 2012 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

/*
 * NOTE: This API is _unstable_ and will change drastically over time.
 */

#ifndef _LIBMAPLE_USB_H_
#define _LIBMAPLE_USB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <usb_type.h>
#include <stm32f10x.h>

#define BIT(shift)                     (1UL << (shift))

/*
 * Descriptors and other paraphernalia
 */

/* Descriptor types */

#define USB_DESCRIPTOR_TYPE_DEVICE        0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION 0x02
#define USB_DESCRIPTOR_TYPE_STRING        0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE     0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT      0x05

/* Descriptor structs and declaration helpers */

#define USB_DESCRIPTOR_STRING_LEN(x) (2 + (x << 1))

#define USB_DESCRIPTOR_STRING(len)              \
  struct {                                      \
      uint8_t_t bLength;                            \
      uint8_t bDescriptorType;                    \
      uint16_t bString[len];                      \
  } __packed

typedef struct usb_descriptor_device {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __packed usb_descriptor_device;

typedef struct usb_descriptor_config_header {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} __packed usb_descriptor_config_header;

typedef struct usb_descriptor_interface {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __packed usb_descriptor_interface;

typedef struct usb_descriptor_endpoint {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __packed usb_descriptor_endpoint;

typedef struct usb_descriptor_string {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bString[];
} usb_descriptor_string;

/* Common values that go inside descriptors */

#define USB_CONFIG_ATTR_BUSPOWERED        0b10000000
#define USB_CONFIG_ATTR_SELF_POWERED      0b11000000

#define USB_EP_TYPE_INTERRUPT             0x03
#define USB_EP_TYPE_BULK                  0x02

#define USB_DESCRIPTOR_ENDPOINT_IN        0x80
#define USB_DESCRIPTOR_ENDPOINT_OUT       0x00

/*
 * USB module core
 */

#ifndef USB_ISR_MSK
/* Handle CTRM, WKUPM, SUSPM, ERRM, SOFM, ESOFM, RESETM */
#define USB_ISR_MSK 0xBF00
#endif

typedef enum usb_dev_state {
    USB_UNCONNECTED,
    USB_ATTACHED,
    USB_POWERED,
    USB_SUSPENDED,
    USB_ADDRESSED,
    USB_CONFIGURED
} usb_dev_state;

/* Encapsulates global state formerly handled by usb_lib/ */
typedef struct usblib_dev {
    uint32_t irq_mask;
    void (**ep_int_in)(void);
    void (**ep_int_out)(void);
    usb_dev_state state;
    usb_dev_state prevState;
    //rcc_clk_id clk_id;
} usblib_dev;

extern usblib_dev *USBLIB;

void usb_init_usblib(usblib_dev *dev,
                     void (**ep_int_in)(void),
                     void (**ep_int_out)(void));

static inline uint8_t usb_is_connected(usblib_dev *dev) {
    return dev->state != USB_UNCONNECTED;
}

static inline uint8_t usb_is_configured(usblib_dev *dev) {
    return dev->state == USB_CONFIGURED;
}

#ifdef __cplusplus
}
#endif

#endif
