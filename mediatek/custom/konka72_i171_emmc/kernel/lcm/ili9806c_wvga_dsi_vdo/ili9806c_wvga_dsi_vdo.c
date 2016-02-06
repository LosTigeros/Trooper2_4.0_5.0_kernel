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

#define FRAME_WIDTH                                         (480)
#define FRAME_HEIGHT                                        (854)

#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_ILI9806C 0x9816

#define LCM_DSI_CMD_MODE                                    0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static unsigned int lcm_esd_test = TRUE;      ///only for ESD test
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//I171 TNÆÁ
static struct LCM_setting_table lcm_initialization_setting_TN[] = {
    {0xff,5,{0xff,0x98,0x06,0x04,0x01}},
    {0x08,1,{0x10}},
    {0x21,1,{0x01}},
    {0x30,1,{0x01}},
    {0x31,1,{0x02}},
    {0x40,1,{0x15}},
    {0x41,1,{0x33}},
    {0x42,1,{0x03}},
    {0x43,1,{0x09}},
    {0x44,1,{0x09}},
    {0x50,1,{0x78}},
    {0x51,1,{0x66}},
    {0x60,1,{0x07}},
    {0x61,1,{0x00}},
    {0x62,1,{0x08}},
    {0x63,1,{0x00}},
    {0x52,1,{0x00}},
    {0x53,1,{0x46}},
    {0x57,1,{0x50}},

    {0xa0,1,{0x00}},
    {0xa1,1,{0x06}},
    {0xa2,1,{0x00}},
    {0xa3,1,{0x13}},
    {0xa4,1,{0x10}},
    {0xa5,1,{0x1c}},
    {0xa6,1,{0x0f}},
    {0xa7,1,{0x0f}},
    {0xa8,1,{0x01}},
    {0xa9,1,{0x06}},
    {0xaa,1,{0x04}},
    {0xab,1,{0x08}},
    {0xac,1,{0x0e}},
    {0xad,1,{0x33}},
    {0xae,1,{0x31}},
    {0xaf,1,{0x00}},
    {0xc0,1,{0x00}},
    {0xc1,1,{0x01}},
    {0xc2,1,{0x0c}},
    {0xc3,1,{0x0a}},
    {0xc4,1,{0x01}},
    {0xc5,1,{0x14}},
    {0xc6,1,{0x06}},
    {0xc7,1,{0x03}},
    {0xc8,1,{0x04}},
    {0xc9,1,{0x08}},
    {0xca,1,{0x07}},
    {0xcb,1,{0x01}},
    {0xcc,1,{0x08}},
    {0xcd,1,{0x28}},
    {0xce,1,{0x1f}},
    {0xcf,1,{0x00}},
    {0xff,5,{0xff,0x98,0x06,0x04,0x06}},
    {0x00,1,{0x21}},
    {0x01,1,{0x06}},
    {0x02,1,{0x00}},
    {0x03,1,{0x00}},
    {0x04,1,{0x01}},
    {0x05,1,{0x01}},
    {0x06,1,{0x80}},
    {0x07,1,{0x02}},
    {0x08,1,{0x05}},
    {0x09,1,{0x00}},
    {0x0a,1,{0x00}},
    {0x0b,1,{0x00}},
    {0x0c,1,{0x01}},
    {0x0d,1,{0x01}},
    {0x0e,1,{0x00}},
    {0x0f,1,{0x00}},
    {0x10,1,{0xf0}},
    {0x11,1,{0xf4}},
    {0x12,1,{0x00}},
    {0x13,1,{0x00}},
    {0x14,1,{0x00}},
    {0x15,1,{0xc0}},
    {0x16,1,{0x08}},
    {0x17,1,{0x00}},
    {0x18,1,{0x00}},
    {0x19,1,{0x00}},
    {0x1a,1,{0x00}},
    {0x1b,1,{0x00}},
    {0x1c,1,{0x00}},
    {0x1d,1,{0x00}},
    {0x20,1,{0x02}},
    {0x21,1,{0x13}},
    {0x22,1,{0x45}},
    {0x23,1,{0x67}},
    {0x24,1,{0x01}},
    {0x25,1,{0x23}},
    {0x26,1,{0x45}},
    {0x27,1,{0x67}},
    {0x30,1,{0x13}},
    {0x31,1,{0x22}},
    {0x32,1,{0x22}},
    {0x33,1,{0x22}},
    {0x34,1,{0x22}},
    {0x35,1,{0xbb}},
    {0x36,1,{0xaa}},
    {0x37,1,{0xdd}},
    {0x38,1,{0xcc}},
    {0x39,1,{0x66}},
    {0x3a,1,{0x77}},
    {0x3b,1,{0x22}},
    {0x3c,1,{0x22}},
    {0x3d,1,{0x22}},
    {0x3e,1,{0x22}},
    {0x3f,1,{0x22}},
    {0x40,1,{0x22}},
    {0x52,1,{0x10}},
    {0x53,1,{0x10}},
    {0xff,5,{0xff,0x98,0x06,0x04,0x07}},
    {0x17,1,{0x22}},
    {0x02,1,{0x77}},
    {0xe1,1,{0x79}},
    {0xff,5,{0xff,0x98,0x06,0x04,0x00}},
    {0x55,1,{0x80}},
	
	{0x11,1,{0x00}},
    {REGFLAG_DELAY, 120, {}},
       
    {0x29,1,{0x00}},
    {REGFLAG_DELAY, 50, {}},

    {0xFF,5,{0xFF,0x98,0x06,0x04,0x08}},   // Change to Page 8 CMD for Normal command

	{REGFLAG_END_OF_TABLE, 0x00, {}}      
};

//I171 IPSÆÁ
static struct LCM_setting_table lcm_initialization_setting_IPS[] = { 
    //****************************************************************************// 
    //****************************** Page 1 Command ******************************// 
    //****************************************************************************// 
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1 
    {0x08,1,{0x10}},                 // output SDA 
    {0x21,1,{0x01}},                 // DE = 1 Active 
    {0x30,1,{0x01}},                 // 480 X 854 
    {0x31,1,{0x02}},                 // 2-dot Inversion 
    {0x60,1,{0x07}},                 // SDTI 
    {0x61,1,{0x00}},                // CRTI 
    {0x62,1,{0x08}},                 // EQTI 
    {0x63,1,{0x00}},                // PCTI 
    {0x40,1,{0x10}},                // BT  +2.5/-2.5 pump for DDVDH-L 
    {0x41,1,{0x33}},                 // DVDDH DVDDL clamp   
    {0x42,1,{0x02}},                 // VGH/VGL 
    {0x43,1,{0x09}}, 
    {0x44,1,{0x09}},                 // VGH/VGL 
    {0x50,1,{0x60}},                 // VGMP 
    {0x51,1,{0x60}},                // VGMN 
    {0x52,1,{0x00}},                   //Flicker 
    {0x53,1,{0x39}},                  //Flicker  0x53
    {0x57,1,{0x50}},
 
    //++++++++++++++++++ Gamma Setting ++++++++++++++++++// 
    {0xA0,1,{0x00}},  // Gamma 255 
    {0xA1,1,{0x11}}, // Gamma 251 
    {0xA2,1,{0x18}}, // Gamma 247 
    {0xA3,1,{0x0D}},  // Gamma 239 
    {0xA4,1,{0x06}},  // Gamma 231 
    {0xA5,1,{0x09}},  // Gamma 203 
    {0xA6,1,{0x08}},  // Gamma 175 
    {0xA7,1,{0x06}},  // Gamma 147 
    {0xA8,1,{0x08}},  // Gamma 108 
    {0xA9,1,{0x0A}},  // Gamma 80 
    {0xAA,1,{0x13}},  // Gamma 52 
    {0xAB,1,{0x08}},  // Gamma 24 
    {0xAC,1,{0x0E}},  // Gamma 16 
    {0xAD,1,{0x17}},  // Gamma 8 
    {0xAE,1,{0x0D}},  // Gamma 4 
    {0xAF,1,{0x00}},  // Gamma 0 
    ///==============Nagitive 
    {0xC0,1,{0x00}},  // Gamma 255 
    {0xC1,1,{0x11}},  // Gamma 251 
    {0xC2,1,{0x18}},  // Gamma 247 
    {0xC3,1,{0x0D}},  // Gamma 239 
    {0xC4,1,{0x06}}, // Gamma 231 
    {0xC5,1,{0x09}},  // Gamma 203 
    {0xC6,1,{0x08}},  // Gamma 175 
    {0xC7,1,{0x06}},  // Gamma 147 
    {0xC8,1,{0x08}},  // Gamma 108 
    {0xC9,1,{0x0A}},  // Gamma 80 
    {0xCA,1,{0x13}},  // Gamma 52 
    {0xCB,1,{0x08}},  // Gamma 24 
    {0xCC,1,{0x0E}},  // Gamma 16 
    {0xCD,1,{0x17}},  // Gamma 8 
    {0xCE,1,{0x0D}},  // Gamma 4 
    {0xCF,1,{0x00}},  // Gamma 0 
 
    //+++++++++++++++++++++++++++++++++++++++++++++++++++// 
 
    //****************************************************************************// 
    //****************************** Page 6 Command ******************************// 
    //****************************************************************************// 
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6 
    {0x00,1,{0x20}}, 
    {0x01,1,{0x04}}, 
    {0x02,1,{0x00}},     
    {0x03,1,{0x00}}, 
    {0x04,1,{0x01}}, 
    {0x05,1,{0x01}}, 
    {0x06,1,{0x88}},     
    {0x07,1,{0x04}}, 
    {0x08,1,{0x01}}, 
    {0x09,1,{0x90}},     
    {0x0A,1,{0x03}},     
    {0x0B,1,{0x01}},     
    {0x0C,1,{0x01}}, 
    {0x0D,1,{0x01}}, 
    {0x0E,1,{0x00}}, 
    {0x0F,1,{0x00}}, 
    {0x10,1,{0x55}}, 
    {0x11,1,{0x53}}, 
    {0x12,1,{0x01}}, 
    {0x13,1,{0x0D}}, 
    {0x14,1,{0x0D}}, 
    {0x15,1,{0x43}}, 
    {0x16,1,{0x0B}}, 
    {0x17,1,{0x00}}, 
    {0x18,1,{0x00}}, 
    {0x19,1,{0x00}}, 
    {0x1A,1,{0x00}}, 
    {0x1B,1,{0x00}}, 
    {0x1C,1,{0x00}}, 
    {0x1D,1,{0x00}}, 

    {0x20,1,{0x01}}, 
    {0x21,1,{0x23}}, 
    {0x22,1,{0x45}}, 
    {0x23,1,{0x67}}, 
    {0x24,1,{0x01}}, 
    {0x25,1,{0x23}}, 
    {0x26,1,{0x45}}, 
    {0x27,1,{0x67}}, 

    {0x30,1,{0x02}}, 
    {0x31,1,{0x22}}, 
    {0x32,1,{0x11}}, 
    {0x33,1,{0xAA}}, 
    {0x34,1,{0xBB}}, 
    {0x35,1,{0x66}}, 
    {0x36,1,{0x00}}, 
    {0x37,1,{0x22}}, 
    {0x38,1,{0x22}}, 
    {0x39,1,{0x22}}, 
    {0x3A,1,{0x22}}, 
    {0x3B,1,{0x22}}, 
    {0x3C,1,{0x22}}, 
    {0x3D,1,{0x22}}, 
    {0x3E,1,{0x22}}, 
    {0x3F,1,{0x22}}, 
    {0x40,1,{0x22}}, 
    {0x52,1,{0x10}}, 
    {0x53,1,{0x10}}, 

    //****************************************************************************// 
    //****************************** Page 7 Command ******************************// 
    //****************************************************************************// 
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7 
    {0x17,1,{0x22}}, 
    {0xE1,1,{0x79}}, 
    {0x02,1,{0x77}}, 
    {0x06,1,{0x13}}, 
    
    //****************************************************************************// 
    //****************************** Page 0 Command ******************************// 
    //****************************************************************************// 
    {REGFLAG_DELAY,10, {}}, 
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 0 
    {0x55,1, {0xB0}},//color enhance
    
    //{0x35,1,{0x00}},   //TE=ON 
        
    {0x11,1,{0x00}}, 
    {REGFLAG_DELAY, 120, {}}, 
        
    {0x29,1,{0x00}}, 
    {REGFLAG_DELAY, 50, {}}, 

    {0xFF,5,{0xFF,0x98,0x06,0x04,0x08}},   // Change to Page 8 CMD for Normal command 

    {REGFLAG_END_OF_TABLE, 0x00, {}}       
} ;

/*

static struct LCM_setting_table lcm_set_window[] = {
    {0x2A,  4,  {0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
    {0x2B,  4,  {0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

*/


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x08}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
    {REGFLAG_DELAY, 1, {}}, 
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x01}},
    {0x58, 1, {0x91}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*
static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

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
    
#if 0 //mt6589 and mt6572 no need
    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;
#endif

#if (LCM_DSI_CMD_MODE)
        params->dsi.mode   = CMD_MODE;
#else
        params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting       
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;   //DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=854;

    params->dsi.vertical_sync_active                = 4;//3;
    params->dsi.vertical_backporch                  = 16;//12;  16
    params->dsi.vertical_frontporch                 = 4;//2;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active              = 6;//10;
    params->dsi.horizontal_backporch                = 37;//50;
    params->dsi.horizontal_frontporch               = 37;//50;
    params->dsi.horizontal_blanking_pixel           = 60;
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
    
    // Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2;     // div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2;     // div2=0,1,2,3;div1_real=1,2,4,4
    params->dsi.fbk_sel=0;      // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =8;     // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
    params->dsi.pll_div1=1;     // div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=1;     // div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_sel=1;       // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =31;//17;       // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)    20  
#endif
#if 1
    // ESD or noise interference recovery For video mode LCM only. // Send TE packet to LCM in a period of n frames and check the response. 
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
#endif
}

#ifndef BUILD_LK 
extern int lcd_firmware_version[2];
#endif

static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;
    int i;
    int uiHighCnt=0, uiLowCnt=0; 

    mt_set_gpio_mode(GPIO21, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO21, GPIO_DIR_IN);
	for (i = 0; i < 6; i++)  
	{      
	    if (mt_get_gpio_in(GPIO21))        //LCM ID Pin:119       
	    {            
	        uiHighCnt++;     
	    }        
	    else        
	    {          
	        uiLowCnt++;       
	    }    
	}
	if (uiHighCnt > uiLowCnt)//IPSÆÁ
	{
		return 1;
	} 
	else//TNÆÁ
	{
		return 0;
	}
}

static void lcm_init(void)
{
    unsigned int data_array[16];
    unsigned int id = 0;
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(120);

    id = lcm_compare_id();

    #ifdef BUILD_LK
        printf("ILI9806C lk %s id=%d\n", __func__,id);
    #else
        printk("ILI9806C kernel %s id=%d\n", __func__,id);
    #endif

    if(0 == id)
        push_table(lcm_initialization_setting_TN, sizeof(lcm_initialization_setting_TN) / sizeof(struct LCM_setting_table), 1);
    else
        push_table(lcm_initialization_setting_IPS, sizeof(lcm_initialization_setting_IPS) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_esd_check(void)
{
    unsigned char buffer[3] = {0};
    unsigned int array[3] = {0};

    UDELAY(600);

    array[0] = 0x00063902;
    array[1] = 0x0698ffff;
    array[2] = 0x00000004;
    dsi_set_cmdq(array, 3, 1);
    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------

    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x0A, &buffer[0], 1);
#if defined(BUILD_LK)  
//    printf("lcm_esd_check 0x0A=%x\n",buffer[0]);
#else  
//    printk("lcm_esd_check 0x0A=%x\n",buffer[0]);
#endif

    array[0] = 0x00063902;
    array[1] = 0x0698ffff;
    array[2] = 0x00000804;
    dsi_set_cmdq(array, 3, 1);

    if(0x9c == buffer[0])
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK   
    
    printk("lcm_esd_recover enter \n");
       
    lcm_init(); 
#endif  
     
    return TRUE;
}

static void lcm_suspend(void)
{
#if 0
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);
#else
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    //lcm_init();
    //push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}


static void lcm_resume(void)
{
    lcm_init();    
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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

//Add for color enhance start
static unsigned char s_ucColorMode = 1;
static void lcm_setColorMode(long lmode)
{
	//struct LCM_setting_table ce_start[] = {{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},};
	struct LCM_setting_table ce_disable[] = {{0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},//page0
	                                         {0x55, 1,  {0x00}},
											 {REGFLAG_DELAY, 10, {}},
											 {0xFF, 5, {0xFF,0x98,0x06,0x04,0x08}},//page8
											 // Setting ending by predefined flag
											 {REGFLAG_END_OF_TABLE, 0x00, {}},};
	struct LCM_setting_table ce_high[] = {{0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
                                             {0x55, 1, {0xB0}},
											 {REGFLAG_DELAY, 10, {}},
											 {0xFF, 5, {0xFF,0x98,0x06,0x04,0x08}},
											 // Setting ending by predefined flag
											 {REGFLAG_END_OF_TABLE, 0x00, {}},};									  	
    //push_table(ce_start, sizeof(ce_start)/sizeof(struct LCM_setting_table), 1);
    switch (lmode) {
		case 0:
			push_table(ce_disable, sizeof(ce_disable)/sizeof(struct LCM_setting_table), 1);
			break;
		default:
			push_table(ce_high, sizeof(ce_high)/sizeof(struct LCM_setting_table), 1);
			break;
	}

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
 
LCM_DRIVER ili9806c_wvga_dsi_vdo_lcm_drv = 
{
    .name           = "ili9806c_dsi_vdo",
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

