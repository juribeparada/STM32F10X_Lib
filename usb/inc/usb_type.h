/**
  ******************************************************************************
  * @file    usb_type.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Type definitions used by the USB Library
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_TYPE_H
#define __USB_TYPE_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef void (*voidFuncPtr)(void);
typedef void (*voidArgumentFuncPtr)(void *);

#define __io volatile
#define __attr_flash __attribute__((section (".USER_FLASH")))
#define __packed __attribute__((__packed__))
#define __deprecated __attribute__((__deprecated__))
#define __weak __attribute__((weak))
#ifndef __always_inline
#define __always_inline __attribute__((always_inline))
#endif
#ifndef __unused
#define __unused __attribute__((unused))
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef offsetof
#define offsetof(type, member) __builtin_offsetof(type, member)
#endif

/* Exported constants --------------------------------------------------------*/
#ifndef NULL
#define NULL ((void *)0)
#endif

typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
USB_bool;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/

#endif /* __USB_TYPE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
