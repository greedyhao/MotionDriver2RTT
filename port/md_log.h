/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-15     SummerGift   first version
 */

/*
 * NOTE: DO NOT include this file on the header file.
 */

#ifndef LOG_TAG
#define DBG_TAG               "md"
#else
#define DBG_TAG               LOG_TAG
#endif /* LOG_TAG */

#ifdef MD_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif /* FRM_DEBUG */

#include <rtdbg.h>
