/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/


#ifdef BUILD_LK
#include "platform/mt_gpio.h"
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM8018B	0x8009

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

int LCM_DETECT = 0;

static void OTM8018B_set_reset_pin(int high){
	mt_set_gpio_mode(GPIO_DISP_LRSTB_PIN, GPIO_MODE_GPIO);
	if(1 == high)
		mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
}
//#define SET_RESET_PIN(v)    (OTM8018B_set_reset_pin(v))

static struct LCM_setting_table lcm_initialization_setting[] = {
//YT50F126H0     ??¨º¨®?¨¢-CPT2¡ê¨¢¡ì  ID low
    {0x00,1,	{0x00}},
    {0xFF,3,	{0x80,0x09,0x01}},

    {0x00,1,	{0x80}},
    {0xFF,2,	{0x80,0x09}},
    //RGB SPI Read Function
    {0x00,1,	{0x03}},
    {0xFF,1,	{0x01}},

    {0x00,1,	{0x8B}},
    {0xB0,1,	{0x40}},

    {0x00,1,	{0x90}},
    {0xB3,1,	{0x02}},

    {0x00,1,	{0x92}},
    {0xB3,1,	{0x45}},

    {0x00,1,	{0x90}},
    {0xC0,6,	{0x00,0x44,0x00,0x00,0x00,0x03}},

    {0x00,1,	{0xA3}},
    {0xC0,1,	{0x00}},

    {0x00,1,	{0xB4}},
    {0xC0,1,	{0x50}},

    {0x00,1,	{0x80}},
    {0xd6,1,	{0x88}},//color enhance

    {0x00,1,	{0x81}},
    {0xC1,1,	{0x66}},

    {0x00,1,	{0xA0}},
    {0xC1,2,	{0xEA,0x0E}},

    {0x00,1,	{0xA6}},
    {0xC1,3,	{0x01,0x00,0x00}},

    {0x00,1,	{0x81}},
    {0xC4,1,	{0x83}},

    {0x00,1,	{0x89}},
    {0xC4,1,	{0x08}},

    {0x00,1,	{0x82}},
    {0xC5,1,	{0xA3}},

    {0x00,1,	{0x90}},
    {0xC5,4,	{0x96,0xA6,0x01,0x03}},

    {0x00,1,	{0xB1}},
    {0xC5,1,	{0xA9}},

    {0x00,1,	{0xC0}},
    {0xC5,2,	{0x00,0xC0}},

    {0x00,1,	{0xB2}},
    {0xF5,4,	{0x15,0x00,0x15,0x00}},

    {0x00,1,	{0x80}},
    {0xCE,6,	{0x87,0x03,0x00,0x86,0x02,0x00}},
    {0x00,1,	{0x90}},
    {0xCE,6,	{0x33,0x54,0x14,0x33,0x55,0x14}},                                                                                                                                                           
    {0x00,1,	{0xa0}},
    {0xCE,14,	{0x38,0x03,0x03,0x58,0x00,0x00,0x00,0x38,0x02,0x03,0x59,0x00,0x00,0x00}},                                                                                                                                                    
    {0x00,1,	{0xb0}},
    {0xCE,14,	{0x38,0x01,0x03,0x5A,0x00,0x00,0x00,0x38,0x00,0x03,0x5B,0x00,0x00,0x00}},
    {0x00,1,	{0xC0}},
    {0xCE,14,	{0x30,0x00,0x03,0x5C,0x00,0x00,0x00,0x30,0x01,0x03,0x5D,0x00,0x00,0x00}},
    {0x00,1,	{0xD0}},
    {0xCE,14,	{0x38,0x05,0x03,0x5E,0x00,0x00,0x00,0x38,0x04,0x03,0x5F,0x00,0x00,0x00}},

    {0x00,1,	{0xC6}},
    {0xCF,4,	{0x01,0x80,0x00,0x09}},

    {0x00,1,	{0xC1}},
    {0xCB,4,	{0x04,0x04,0x04,0x04}},
    {0x00,1,	{0xC7}},
    {0xCB,4,	{0x04,0x04,0x04,0x04}},
    {0x00,1,	{0xD6}},
    {0xCB,4,    {0x04,0x04,0x04,0x04}},
    {0x00,1,	{0xDC}},
    {0xCB,3,	{0x04,0x04,0x04}},
    {0x00,1,	{0xE0}},
    {0xCB,1,	{0x04}},

    {0x00,1,	{0x81}},
    {0xCC,4,	{0x26,0x25,0x02,0x06}},
    {0x00,1,	{0x87}},
    {0xCC,3,	{0x0A,0x0E,0x0C}},
    {0x00,1,	{0x90}},
    {0xCC,1,	{0x10}},
    {0x00,1,	{0x9B}},
    {0xCC,4,	{0x26,0x25,0x01,0x05}},
    {0x00,1,	{0xA2}},
    {0xCC,4,	{0x09,0x0D,0x0B,0x0F}},
    {0x00,1,	{0xB1}},
    {0xCC,4,	{0x25,0x26,0x05,0x01}},
    {0x00,1,	{0xB7}},
    {0xCC,3,	{0x0D,0x09,0x0B}},
    {0x00,1,	{0xC0}},
    {0xCC,1,	{0x0F}},
    {0x00,1,	{0xCB}},
    {0xCC,4,	{0x25,0x26,0x06,0x02}},
    {0x00,1,	{0xD2}},
    {0xCC,4,	{0x0E,0x0A,0x0C,0x10}},

    //GVDD/NGVDD Setting
    {0x00,1,	{0x00}},
    {0xD8,2,	{0x77,0x77}},

    {0x00,1,	{0x00}},
    {0xD9,1,	{0x1F}},

    //Gamma 2.2
    {0x00,1,	{0x00}},
    {0xE1,16,	{0x02,0x08,0x10,0x0F,0x09,0x1B,0x0E,0x0D,0x00,0x05,0x03,0x08,0x10,0x20,0x1C,0x08}},
    {0x00,1,	{0x00}},
    {0xE2,16,	{0x02,0x08,0x10,0x0F,0x09,0x1B,0x0E,0x0D,0x00,0x05,0x03,0x08,0x10,0x20,0x1C,0x08}},

    {0x00,1,	{0x00}},
    {0xFF,3,	{0xFF,0xFF,0xFF}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 50, {}},
};

static struct LCM_setting_table lcm_initialization_setting1[] = {
//¦Ì??¡ì?¨¢-HSD2¡ê¨¢¡ìDJ   ID  hight
//gamma2.5
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0x80,0x09,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x80,0x09}},
	{0x00,	1,	{0x03}},
	{0xff,	1,	{0x01}},
	{0x00,	1,	{0xb4}},
	{0xc0,	1,	{0x50}},   //0x10
	{0x00,	1,	{0x82}},
	{0xc5,	1,	{0xa3}},
	{0x00,	1,	{0x90}},
	{0xc5,	2,	{0xd6,0x76}},   //0x96,0x76

    {0x00,	1,	{0x91}},
    {0xc0,	1,	{0xFF}},//0x61

    {0x00,	1,	{0x80}},
    {0xd6,	1,	{0x88}},//color enhance

	{0x00,	1,	{0x00}},
	{0xd8,	2,	{0x97,0x97}},  //5.0V
	{0x00,	1,	{0x00}},
	{0xd9,	1,	{0x3d}},   //39
	{0x00,	1,	{0x00}},
	{0xe1,	16,	{0x09,0x0c,0x12,0x0e,0x08,0x1c,0x0e,0x0E,0x00,0x04,0x02,0x07,0x0F,0x21,0x1C,0x0f}},
	{0x00,	1,	{0x00}},
	{0xe2,	16,	{0x09,0x0c,0x12,0x0e,0x08,0x1c,0x0e,0x0E,0x00,0x04,0x02,0x07,0x0E,0x20,0x1D,0x0f}},
	{0x00,	1,	{0x81}},
	{0xc1,	1,	{0x77}},
	{0x00,	1,	{0xa1}},
	{0xc1,	1,	{0x0e}},

	{0x00,	1,	{0xa0}},
	{0xc1,	1,	{0xea}},//MIPI I/F

	{0x00,	1,	{0x89}},
	{0xc4,	1,	{0x08}},
	{0x00,	1,	{0xa2}},
	{0xc0,	3,	{0x04,0x00,0x02}},
	{0x00,	1,	{0x80}},
	{0xc4,	1,	{0x30}},
	{0x00,	1,	{0x8a}},
	{0xc4,	1,	{0x40}},
	{0x00,	1,	{0xa6}},
	{0xc1,	1,	{0x01}},
	{0x00,	1,	{0xc0}},
	{0xc5,	1,	{0x00}},
	{0x00,	1,	{0x8b}},
	{0xb0,	1,	{0x40}},
	{0x00,	1,	{0xb2}},
	{0xf5,	4,	{0x15,0x00,0x15,0x00}},
	{0x00,	1,	{0x93}},
	{0xc5,	1,	{0x03}},
	{0x00,	1,	{0x81}},
	{0xc4,	1,	{0x83}},
	{0x00,	1,	{0x92}},
	{0xc5,	1,	{0x01}},
	{0x00,	1,	{0xb1}},
	{0xc5,	1,	{0xa9}},
	{0x00,	1,	{0x92}},
	{0xb3,	1,	{0x45}},
	{0x00,	1,	{0x90}},
	{0xb3,	1,	{0x02}},
	{0x00,	1,	{0x80}},
	{0xc0,	5,	{0x00,0x58,0x00,0x14,0x16}},
	{0x00,	1,	{0x90}},
	{0xc0,	6,	{0x00,0x56,0x00,0x00,0x00,0x03}},
	{0x00,	1,	{0xa6}},
	{0xc1,	3,	{0x01,0x00,0x00}},
	{0x00,	1,	{0x80}},
	{0xce,	12,	{0x87,0x03,0x00,0x85,0x03,0x00,0x86,0x03,0x00,0x84,0x03,0x00}},
	{0x00,	1,	{0xa0}},
	{0xce,	14,	{0x38,0x03,0x03,0x58,0x00,0x00,0x00,0x38,0x02,0x03,0x59,0x00,0x00,0x00}},
	{0x00,	1,	{0xb0}},
	{0xce,	14,	{0x38,0x01,0x03,0x5a,0x00,0x00,0x00,0x38,0x00,0x03,0x5b,0x00,0x00,0x00}},
	{0x00,	1,	{0xc0}},
	{0xce,	14,	{0x30,0x00,0x03,0x5c,0x00,0x00,0x00,0x30,0x01,0x03,0x5d,0x00,0x00,0x00}},
	{0x00,	1,	{0xd0}},
	{0xce,	14,	{0x30,0x02,0x03,0x5e,0x00,0x00,0x00,0x30,0x03,0x03,0x5f,0x00,0x00,0x00}},
	{0x00,	1,	{0xc7}},
	{0xcf,	1,	{0x00}},
	{0x00,	1,	{0xc9}},
	{0xcf,	1,	{0x00}},
	{0x00,	1,	{0xc4}},
	{0xcb,	6,	{0x04,0x04,0x04,0x04,0x04,0x04}},
	{0x00,	1,	{0xd9}},
	{0xcb,	6,	{0x04,0x04,0x04,0x04,0x04,0x04}},
	{0x00,	1,	{0x84}},
	{0xcc,	6,	{0x0c,0x0a,0x10,0x0e,0x03,0x04}},
	{0x00,	1,	{0x9e}},
	{0xcc,	1,	{0x0b}},
	{0x00,	1,	{0xa0}},
	{0xcc,	5,	{0x09,0x0f,0x0d,0x01,0x02}},
	{0x00,	1,	{0xb4}},
	{0xcc,	6,	{0x0d,0x0f,0x09,0x0b,0x02,0x01}},
	{0x00,	1,	{0xce}},
	{0xcc,	1,	{0x0e}},
	{0x00,	1,	{0xd0}},
	{0xcc,	7,	{0x10,0x0a,0x0c,0x04,0x03,0x05,0x85}},
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0xff,0xff,0xff}},

    {0x35,	1,	{0x00}},
    {0x36,	1,	{0x00}},//0x08
    {0x3A,	1,	{0x77}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 50, {}},
};

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	//{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
   memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
    // enable tearing-free
    params->dbi.te_mode				= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity	= LCM_POLARITY_RISING;
    
    params->dsi.mode   = BURST_VDO_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	    = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=854;

    params->dsi.vertical_sync_active				= 4;//3;
    params->dsi.vertical_backporch					= 8;//12;
    params->dsi.vertical_frontporch					= 8;//2;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 6;//10;
    params->dsi.horizontal_backporch				= 37;//50;
    params->dsi.horizontal_frontporch				= 37;//50;
    params->dsi.horizontal_blanking_pixel			= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    
    // Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4
    params->dsi.fbk_sel=0;	 	// fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =8;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =31;//30;//20;//17;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
#endif

    /* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
    params->dsi.lcm_int_te_monitor = TRUE; 
    params->dsi.lcm_int_te_period = 1; // Unit : frames 

    // Need longer FP for more opportunity to do int. TE monitor applicably. 
    if(params->dsi.lcm_int_te_monitor) 
    	params->dsi.vertical_frontporch *= 2; 

    // Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
    params->dsi.lcm_ext_te_monitor = FALSE; 
    // Non-continuous clock 
    params->dsi.noncont_clock = FALSE; 
    params->dsi.noncont_clock_period = 2; // Unit : frames
}

#ifndef BUILD_LK 
//extern int lcd_firmware_version[2];
#endif
static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;
	int i;
    int uiHighCnt, uiLowCnt; 
	uiHighCnt = 0;  
	uiLowCnt  = 0;
#if 0    
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;
	
    #ifdef BUILD_LK
		printf("OTM8018B uboot %s \n", __func__);
	    printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM8018B kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
#endif
	mt_set_gpio_mode(GPIO21, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO21, GPIO_DIR_IN);
	for (i = 0; i < 6; i++)  
	{      
	    if (mt_get_gpio_in(GPIO21))        //LCM ID Pin:21        
	    {            
	        uiHighCnt++;     
	    }        
	    else        
	    {          
	        uiLowCnt++;       
	    }    
	}
	if (uiHighCnt > uiLowCnt)  
	{
		LCM_DETECT = 1;
    #ifndef BUILD_LK    
     //   lcd_firmware_version[1] = 0x22;
    #endif    
	} 
	else
	{
		LCM_DETECT = 0;
    #ifndef BUILD_LK    
     //   lcd_firmware_version[1] = 0x11;
    #endif    
	}
	  
	return 1;
}

static void lcm_init(void)
{
	unsigned int data_array[16];
    unsigned int i;
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(120);

	i = lcm_compare_id();

    if(LCM_DETECT ==0)
  	{
	    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
  	}	
    else if(LCM_DETECT ==1)
  	{
  	    push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1);
  	}
}

static unsigned int lcm_esd_check(void)
{
    #ifndef BUILD_LK    
	if(lcm_esd_test)  
	{
    	lcm_esd_test = FALSE; 
    	return TRUE;   
	}
	unsigned char buffer[1];    
	unsigned int array[16]; 
	#if 0    
	array[0] = 0x00013700;// read id return two byte,version and id    dsi_set_cmdq(array, 1, 1);  
	//  id = read_reg(0xF4);   
	read_reg_v2(0x0A, buffer, 1);
	#else
	array[0] = 0x00013700;// read id return two byte,version and id	
	dsi_set_cmdq(array, 1, 1);      
	read_reg_v2(0x0A, buffer, 1);    
	if(buffer[0] != 0x9C)		
		return  TRUE;      
	read_reg_v2(0x0D, buffer, 1);    
	if(buffer[0] != 0x00)		
		return  TRUE;     
	read_reg_v2(0x0E, buffer, 1);     
	if(buffer[0] != 0x80)		
		return  TRUE;		
	#endif	
	#if defined(BUILD_LK)  
	printf("lcm_esd_check  0x0A = %x\n",buffer[0]);
	#else  
	printk("lcm_esd_check  0x0A = %x\n",buffer[0]);
	#endif
	//    if(buffer[0] != 0x9C) 
	//   {
	//        return TRUE;
	//   }
	#endif
	return FALSE;
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK   
    unsigned int data_array[16];  
#if defined(BUILD_LK) 
    printf("lcm_esd_recover enter \n");
#else   
    printk("lcm_esd_recover enter \n");
#endif      
    lcm_init();   
    data_array[0]=0x00110500;  
    dsi_set_cmdq(&data_array, 1, 1);  
    MDELAY(50);    
    data_array[0]=0x00290500; 
    dsi_set_cmdq(&data_array, 1, 1);   
    data_array[0]= 0x00023902; 
    data_array[1]= 0xFF51;  
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(10);
#endif   

    return TRUE;
}

static unsigned char s_ucColorMode = 1;
static void lcm_setColorMode(long lmode);
static void lcm_suspend(void)
{
/*	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
*/
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
/*
    lcm_init();
    lcm_setColorMode(s_ucColorMode);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
*/
}

static void lcm_resume(void)
{
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
//	lcm_init();
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/

//Add for color enhance
static void lcm_setColorMode(long lmode)
{
#if 0
    unsigned int array1 = 0x80001500;
    unsigned int array2 = 0x08d61500;

    dsi_set_cmdq(&array1, 1, 1); 
    MDELAY(5);
    array2 += (lmode<<31);//28
    dsi_set_cmdq(&array2, 1, 1); 
#else
	struct LCM_setting_table ce_start[] = {{0x00,	1,	{0x00}},
                                             {0xff,	3,	{0x80,0x09,0x01}},
                                             {0x00,	1,	{0x80}},
                                             {0xff,	2,	{0x80,0x09}},
                                             {0x00,	1,	{0x03}},
                                             {0xff,	1,	{0x01}},};
	struct LCM_setting_table ce_disable[] = {{0x00, 1,  {0x80}},
                                             {0xd6,	1,	{0x00}},
											 {REGFLAG_DELAY, 10, {}},
											 // Setting ending by predefined flag
											 {REGFLAG_END_OF_TABLE, 0x00, {}},};
	struct LCM_setting_table ce_low[] = {{0x00, 1, {0x80}},
                                             {0xd6,	1,	{0x28}},
											 {REGFLAG_DELAY, 10, {}},
											 // Setting ending by predefined flag
											 {REGFLAG_END_OF_TABLE, 0x00, {}},};
	struct LCM_setting_table ce_medium[] = {{0x00,  1,  {0x80}},
                                            {0xd6,	1,	{0x58}},
											{REGFLAG_DELAY, 10, {}},
											// Setting ending by predefined flag
											{REGFLAG_END_OF_TABLE, 0x00, {}},};
	struct LCM_setting_table ce_high[] = {{0x00, 1, {0x80}},
                                             {0xd6,	1,	{0x88}},
											 {REGFLAG_DELAY, 10, {}},
											 // Setting ending by predefined flag
											 {REGFLAG_END_OF_TABLE, 0x00, {}},};									  	
    push_table(ce_start, sizeof(ce_start)/sizeof(struct LCM_setting_table), 1);
    switch (lmode) {
		case 0:	//color enhanse disable
			push_table(ce_disable, sizeof(ce_disable)/sizeof(struct LCM_setting_table), 1);
			break;
/*			
		case 1:
			push_table(ce_low, sizeof(ce_low)/sizeof(struct LCM_setting_table), 1);
			break;
		case 2:
			push_table(ce_medium, sizeof(ce_medium)/sizeof(struct LCM_setting_table), 1);
			break;
		case 3:
			push_table(ce_high, sizeof(ce_high)/sizeof(struct LCM_setting_table), 1);
			break;
*/
		default:
			push_table(ce_high, sizeof(ce_high)/sizeof(struct LCM_setting_table), 1);
			break;
	}
#endif

    s_ucColorMode = (unsigned char)lmode;
}

static int lcm_getColorMode(unsigned char *pValue)
{
    if (pValue == NULL) {
        return -1;
    }
    *pValue = s_ucColorMode;
    
    return 1;
}
//Add for color enhance end
 
LCM_DRIVER otm8018b_dsi_vdo_lcm_drv = 
{
    .name			= "otm8018b_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,	
	.esd_check      = lcm_esd_check,      //only for command mode, no use in video mode    
	.esd_recover    = lcm_esd_recover,    //only for command mode, no use in video mode
#if (LCM_DSI_CMD_MODE)
	//.set_backlight  = lcm_setbacklight,
    //.update         = lcm_update,
#endif
    //Add for color enhance
	.setColorMode = lcm_setColorMode,
	.getColorMode = lcm_getColorMode,
	//Add for color enhance end
};

