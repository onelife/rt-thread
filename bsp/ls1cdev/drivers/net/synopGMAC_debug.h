/*
 * File      : synopGMAC_debug.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) chinesebear
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-08-24     chinesebear  first version
 */


#ifndef __DEBUG_H__
#define __DEBUG_H__

//#define GMAC_DEBUG
#include <rtthread.h>
#ifdef GMAC_DEBUG	
#define DEBUG_MES	rt_kprintf
#else
#define DEBUG_MES(...)
#endif

#endif /*__DEBUG_H__*/
