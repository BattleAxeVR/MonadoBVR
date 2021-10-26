// Copyright 2020-2021, N Madsen.
// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Defines and constants related to WMR driver code.
 * @author nima01 <nima_zero_one@protonmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_wmr
 */

#pragma once


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Defines for the WMR driver.
 *
 * @ingroup drv_wmr
 * @{
 */

#define MS_HOLOLENS_MANUFACTURER_STRING "Microsoft"
#define MS_HOLOLENS_PRODUCT_STRING "HoloLens Sensors"

#define MICROSOFT_VID 0x045e
#define HOLOLENS_SENSORS_PID 0x0659

#define HP_VID 0x03f0
#define REVERB_G1_PID 0x0c6a
#define REVERB_G2_PID 0x0580

#define LENOVO_VID 0x17ef
#define EXPLORER_PID 0xb801

/*!
 * @}
 */


#ifdef __cplusplus
}
#endif
