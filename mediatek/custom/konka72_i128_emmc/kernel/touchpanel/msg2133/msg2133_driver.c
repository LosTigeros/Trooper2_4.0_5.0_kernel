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


#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <linux/dma-mapping.h>

#include "cust_gpio_usage.h"

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
//#include "tpd_custom_msg2033.h"
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif

#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif

#ifdef MT6572
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include <linux/wakelock.h>

extern struct tpd_device *tpd;

static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);


static void tpd_eint_interrupt_handler(void);


#ifdef MT6575 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#else

	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif


static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

//#define TPD_X_INVERT      //__M8522__

#define TPD_RES_X                480
#define TPD_RES_Y                800

#define TPD_HAVE_BUTTON



#define TPD_KEY_COUNT           3

#ifdef TPD_X_INVERT
    #define TPD_KEYS            {KEY_MENU, KEY_HOMEPAGE,KEY_BACK}
#else
    #define TPD_KEYS            {KEY_BACK, KEY_HOMEPAGE, KEY_MENU}
#endif

//#define TPD_KEYS_DIM            {{40,505,80,50},{120,505,80,50},{200,505,80,50}}
//#define TPD_KEYS_DIM            {{75,825,150,50},{240,825,180,50},{405,825,150,50}}

#define TPD_BUTTON_WIDTH        (140)
#define TPD_BUTTON_HEIGHT       (80)

#define KEY_1                   80,850              
#define KEY_2                   240,850
#define KEY_3                   400,850

#define TPD_KEYS_DIM            {{KEY_1,TPD_BUTTON_WIDTH,TPD_BUTTON_HEIGHT},{KEY_2,TPD_BUTTON_WIDTH,TPD_BUTTON_HEIGHT},{KEY_3,TPD_BUTTON_WIDTH,TPD_BUTTON_HEIGHT}} 


#define TPD_HEIGHT	      800
#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#define TPD_POWER_SOURCE         MT65XX_POWER_LDO_VGP2

//#define TPD_CLOSE_POWER_IN_SLEEP
#define TP_DEBUG
#define TP_FIRMWARE_UPDATE
//#define POWERUP_AUTO_CHECK        //开机比较版本号升级固件

#define TPD_OK 0

// debug macros
#if defined(TP_DEBUG)
#define pr_tp(format, args...) printk("<0>" format, ##args)
#define pr_ch(format, arg...)                      \
printk("<0>" "%s <%d>,%s(),cheehwa_print:\n\t"  \
           format,__FILE__,__LINE__,__func__, ##arg)
#else
#define pr_tp(format, args...)  do {} while (0)
#define pr_ch(format, arg...)  do {} while (0)
#undef pr_k(format, arg...)
#define pr_k(format, arg...)  do {} while (0)
#endif



#ifdef TP_FIRMWARE_UPDATE
#define U8 unsigned char
#define S8 signed char
#define U16 unsigned short
#define S16 signed short
#define U32 unsigned int
#define S32 signed int
#if SMC_REPORT
#define TOUCH_ADDR_MSG20XX   0xC0
#else
#define TOUCH_ADDR_MSG20XX   0x4C
#endif
#define FW_ADDR_MSG20XX      0xC4
#define FW_UPDATE_ADDR_MSG20XX   0x92
#define DWIIC_MODE_ISP    0
#define DWIIC_MODE_DBBUS  1
#define   DOWNLOAD_FIRMWARE_BUF_SIZE   94*1024     //59
static U8 *download_firmware_buf = NULL;
//static int FwDataCnt = 0;
//static int FwVersion;
//static struct class *firmware_class;
//static struct device *firmware_cmd_dev;

static u8 g_dwiic_info_data[1024];   // Buffer for info data
#define N_BYTE_PER_TIME (8)//adair:1024的约数,根据平台修改
#define UPDATE_TIMES (1024/N_BYTE_PER_TIME)
u32 crc_tab[256];

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static int update_switch = 0;
#define ENABLE_DMA      0
#if ENABLE_DMA
static u8 *gpDMABuf_va = NULL;
static u32 gpDMABuf_pa = NULL;
#endif

#endif

#ifdef CUSTOM_CTP_VIRTUAL_PS
    #define TPD_PROXIMITY 
#endif

#ifdef TPD_PROXIMITY
//#define CTP_PROXIMITY_ESD_MEASURE 
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)   
static DEFINE_MUTEX(msg2133_sensor_mutex);
static int g_bPsSensorOpen = 0;
static int g_nPsSensorDate = 1;
static int g_bSuspend = 0;
static struct wake_lock ps_lock;
static int msg2133_enable_ps(int enable);
static void tpd_initialize_ps_sensor_function();
#endif

///int SMC_SWITCH=0;
#define SMC_REPORT      0//SMC_SWITCH

struct touch_info
{
    unsigned short y[3];
    unsigned short x[3];
    unsigned short p[3];
    unsigned short count;
};

typedef struct
{
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short pos_x2;
    unsigned short pos_y2;
    unsigned short temp2;
    unsigned short temp;
    short dst_x;
    short dst_y;
    unsigned char checksum;
} SHORT_TOUCH_STATE;

static int boot_mode = 0;
static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE, 0}, {}};
//unsigned short force[] = {0, TOUCH_ADDR_MSG20XX, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata msg2133_i2c_tpd = { I2C_BOARD_INFO(TPD_DEVICE, (TOUCH_ADDR_MSG20XX >> 1))};
extern int tpd_firmware_version[2];

#ifdef TP_FIRMWARE_UPDATE

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)

//#define TP_DEBUG    printk
#define TP_DEBUG(fmt, arg...)    {printk(fmt, ##arg);  printk("\n");}


#ifdef POWERUP_AUTO_CHECK
static const unsigned char caucMsgFirmware[] =
{
    //#include "msctpc3_0035_V502_131127.h"
    #include "msctpc3_0035_V503_131209.h"
};
#endif

static  char *fw_version;
static u8 temp[94][1024];
static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

/*disable irq*/
static void HalDisableIrq(void)
{
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, NULL, 1);
}
/*enable irq*/
static void HalEnableIrq(void)
{
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}


u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}


static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}


static void HalTscrCReadI2CSeq( u8 addr, u8* read_data, u8 size )
{
    //according to your platform.
    int rc;

    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = I2C_M_RD,
            .len = size,
            .buf = read_data,
        },
    };

    rc = i2c_transfer( i2c_client->adapter, msgs, 1 );
    if( rc < 0 )
    {
        printk( "HalTscrCReadI2CSeq error %d\n", rc );
    }
}

static void HalTscrCDevWriteI2CSeq( u8 addr, u8* data, u16 size )
{
    //according to your platform.
    int rc;
    int m = 5;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = 0,
            .len = size,
            .buf = data,
        },
    };
    
    while(m--)
    {
        rc = i2c_transfer( i2c_client->adapter, msgs, 1 );
        if( rc < 0 )
        {
            printk( "HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc, addr );
        }
        else
         return;
    }
}



static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static void dbbusDWIICEnterSerialDebugMode( void )
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 5 );
}

static void dbbusDWIICStopMCU( void )
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICIICUseBus( void )
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICIICReshape( void )
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICIICNotUseBus( void )
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICNotStopMCU( void )
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICExitSerialDebugMode( void )
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, data, 1 );

    // Delay some interval to guard the next transaction
    //udelay ( 200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode( void )
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };

    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 );
    mdelay( 10 );         // delay about 1ms
}

static u8 drvISP_Read( u8 n, u8* pDataToRead )  //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1 );
    mdelay( 10 );         // delay about 1000us*****
    if( n == 1 )
    {
        HalTscrCReadI2CSeq( FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );

        // Ideally, the obtained dbbus_rx_data[0~1] stands for the following meaning:
        //  dbbus_rx_data[0]  |  dbbus_rx_data[1]  | status
        // -------------------+--------------------+--------
        //       0x00         |       0x00         |  0x00
        // -------------------+--------------------+--------
        //       0x??         |       0x00         |  0x??
        // -------------------+--------------------+--------
        //       0x00         |       0x??         |  0x??
        //                 
        // Therefore, we build this field patch to return the status to *pDataToRead.
        *pDataToRead = ( (dbbus_rx_data[0] >= dbbus_rx_data[1] )? \
                          dbbus_rx_data[0]  : dbbus_rx_data[1] );
        
    }
    else
    {
        HalTscrCReadI2CSeq( FW_UPDATE_ADDR_MSG21XX, pDataToRead, n );
    }

    return 0;
}

static void drvISP_WriteEnable( void )
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
}


static void drvISP_ExitIspMode( void )
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1 );
}

static u8 drvISP_ReadStatus( void )
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    mdelay( 1 );         // delay about 100us*****
    drvISP_Read( 1, &bReadData );
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    return bReadData;
}


static void drvISP_BlockErase( u32 addr )
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    mdelay( 1 );         // delay about 100us*****
    timeOutCount = 0;
    while( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if( timeOutCount >= 100000 ) break;  /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    //bWriteData[4] = (addr & 0xFF) ;
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    //HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 5);
    HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    mdelay( 1 );         // delay about 100us*****
    timeOutCount = 0;
    while( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if( timeOutCount >= 500000 ) break;  /* around 5 sec timeout */
    }
}

static void drvISP_Program( u16 k, u8* pDataToWrite )
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u32 timeOutCount = 0;
    for( j = 0; j < 8; j++ )  //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = ( addr + 128 * j ) >> 16;
        TX_data[3] = ( addr + 128 * j ) >> 8;
        TX_data[4] = ( addr + 128 * j );
        for( i = 0; i < 128; i++ )
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        mdelay( 1 );         // delay about 100us*****

        timeOutCount = 0;
        while( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if( timeOutCount >= 100000 ) break;  /* around 1 sec timeout */
        }



        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, TX_data, 133 ); //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    }
}


static void drvISP_Verify( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for( j = 0; j < 8; j++ )  //128*8 cycle
    {
        bWriteData[2] = ( u8 )( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 )( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 )( addr + j * 128 );
        mdelay( 1 );         // delay about 100us*****


        timeOutCount = 0;
        while( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if( timeOutCount >= 100000 ) break;  /* around 1 sec timeout */
        }



        HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 );  //write read flash addr
        mdelay( 1 );         // delay about 100us*****
        drvISP_Read( 128, RX_data );
        HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );  //cmd end
        for( i = 0; i < 128; i++ )  //log out if verify error
        {
            if( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                TP_DEBUG( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static ssize_t firmware_update_show( struct device *dev,
                                     struct device_attribute *attr, char *buf )
{
    return sprintf( buf, "%s\n", fw_version );
}

static void _HalTscrHWReset( void )
{
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	mdelay( 120 ); 
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	mdelay( 300 ); 
}


static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
    u8  dwiic_rx_data[4];
    u16 reg_data=0;
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );	
    mdelay ( 100 );
	TP_DEBUG("2222222222");
	
    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
	TP_DEBUG("33333333333333");
	
    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
	TP_DEBUG("4444444444444");
    mdelay ( 50 );

    // recive info data
    //HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 1024 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dwiic_rx_data[0], 8 );
    TP_DEBUG("55555555555555");
    return ( 1 );
}

static int drvTP_erase_emem_c33 ( int emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
  
    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;
    TP_DEBUG("111111111111");
    drvTP_read_info_dwiic_c33();
	
    if ( 0/*g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' */)
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );
        TP_DEBUG("666666666666");
        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );
        TP_DEBUG("777777777777");
        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );
        TP_DEBUG("88888888888");
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
        TP_DEBUG("9999999999999");
    }

    //erase main
    TP_DEBUG("aaaaaaaaaaa");
    drvTP_erase_emem_c33 ( 1 );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    //if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( 1 )
    {
        case 0:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case 1:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case 2:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }
	TP_DEBUG("bbbbbbbbbbbbbb");
    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );
	TP_DEBUG("ccccccccccccc");
    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        //if ( emem_type == EMEM_INFO )
		//	i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            //if ( emem_type == EMEM_MAIN ) break;
        }
        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        TP_DEBUG("dddddddddddddd");
        #if 1
        {
            u32 n = 0;
            for(n=0;n<UPDATE_TIMES;n++)
            {
                TP_DEBUG("i=%d,n=%d",i,n);
                HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i]+n*N_BYTE_PER_TIME, N_BYTE_PER_TIME );
            }
        }
        #else
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );
        #endif
        TP_DEBUG("eeeeeeeeeeee");
        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
        TP_DEBUG("ffffffffffffff");
        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }
        TP_DEBUG("ggggggggg");
    //if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
        TP_DEBUG("hhhhhhhhhhhhhh");
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    //if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        TP_DEBUG("iiiiiiiiii");
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
        TP_DEBUG("jjjjjjjjjjjjj");
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    //if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        TP_DEBUG("kkkkkkkkkkk");
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    TP_DEBUG("lllllllllllll");
    update_pass = 1;
	//if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }
    if ( !update_pass )
    {
        TP_DEBUG ( "update_C33 FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	HalEnableIrq();	
        return size;
    }
    TP_DEBUG ( "update_C33 OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
    HalEnableIrq();	
    return size;
}

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    update_switch = 1;

	TP_DEBUG("firmware_update_store ffffffffff\r\n");
	HalDisableIrq();

    _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "111dbbus_rx version[0]=0x%x\r\n ", dbbus_rx_data[0] );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x \r\n", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 )
        {
            ssize_t sizeRet = firmware_update_c33 ( dev, attr, buf, size );

            //Chenzz++ 20130606 防止升级过程中灭屏关掉TP电源
            update_switch = 0;

            return sizeRet;
		}
        else
        {
           // return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
        //return firmware_update_c2 ( dev, attr, buf, size );
    } 

//Chenzz++ 20130606 防止升级过程中灭屏关掉TP电源
    update_switch = 0;
}


static DEVICE_ATTR( update, 0777, firmware_update_show, firmware_update_store );

/*test=================*/
static ssize_t firmware_clear_show( struct device *dev,
                                    struct device_attribute *attr, char *buf )
{
    u16 k = 0, i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = 0;
    u32 timeOutCount = 0;
    for( k = 0; k < 94; i++ )  // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;
        for( j = 0; j < 8; j++ )  //128*8 cycle
        {
            bWriteData[2] = ( u8 )( ( addr + j * 128 ) >> 16 );
            bWriteData[3] = ( u8 )( ( addr + j * 128 ) >> 8 );
            bWriteData[4] = ( u8 )( addr + j * 128 );
            mdelay( 1 );         // delay about 100us*****

            timeOutCount = 0;
            while( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
            {
                timeOutCount++;
                if( timeOutCount >= 100000 ) break;  /* around 1 sec timeout */
            }


            HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 );  //write read flash addr
            mdelay( 1 );         // delay about 100us*****
            drvISP_Read( 128, RX_data );
            HalTscrCDevWriteI2CSeq( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );  //cmd end
            for( i = 0; i < 128; i++ )  //log out if verify error
            {
                if( RX_data[i] != 0xFF )
                {
                    TP_DEBUG( "k=%d,j=%d,i=%d===============erase not clean================", k, j, i );
                }
            }
        }
    }
    TP_DEBUG( "read finish\n" );
    return sprintf( buf, "%s\n", fw_version );
}

static ssize_t firmware_clear_store( struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size )
{

    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    //mdelay ( 100 );        // delay about 100ms*****

    // Enable slave's ISP ECO mode
    /*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();*/

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;

    // Disable the Watchdog
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );



    //Get_Chip_Version();
    //FwVersion  = 2;

    //if (FwVersion  == 2)
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x11;
        dbbus_tx_data[2] = 0xE2;
        dbbus_tx_data[3] = 0x00;

        HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    }

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // ISP Speed Change to 400K
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
    HalTscrCDevWriteI2CSeq( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();

    dbbusDWIICNotStopMCU();

    dbbusDWIICExitSerialDebugMode();


    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    TP_DEBUG( "chip erase+\n" );
    drvISP_BlockErase( 0x00000 );
    TP_DEBUG( "chip erase-\n" );
    drvISP_ExitIspMode();
    return size;
}

static DEVICE_ATTR( clear, 0777, firmware_clear_show, firmware_clear_store );

/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    TP_DEBUG( "*** firmware_version_show fw_version = %s***\n", fw_version );
    return sprintf( buf, "%s\n", fw_version );   
}
static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
     unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;

	fw_version = kzalloc(sizeof(char), GFP_KERNEL);
	
	_HalTscrHWReset();
    mdelay ( 100 );

	dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
	mdelay ( 50 );

	// Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, &dbbus_tx_data[0], 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
    	//msg21xxA
    	TP_DEBUG("***TP IC is msg21xxA ***\n");
		_HalTscrHWReset();
    	//mdelay ( 500 );

		dbbusDWIICEnterSerialDebugMode();
    	dbbusDWIICStopMCU();
    	dbbusDWIICIICUseBus();
    	dbbusDWIICIICReshape();
		//mdelay ( 50 );
	
	    dbbus_tx_data[0] = 0x53;
	    dbbus_tx_data[1] = 0x00;
	    dbbus_tx_data[2] = 0x2A;
	    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
		mdelay( 1 );
	    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);
	    major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
	    minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];	
	}
	else
	{
		//msg21xx
		TP_DEBUG("***TP IC is msg21xx ***\n");
		_HalTscrHWReset();
    	//mdelay ( 500 );

		dbbusDWIICEnterSerialDebugMode();
    	dbbusDWIICStopMCU();
    	dbbusDWIICIICUseBus();
    	dbbusDWIICIICReshape();
		//mdelay ( 50 );
		
		dbbus_tx_data[0] = 0x53;
	    dbbus_tx_data[1] = 0x00;
	    dbbus_tx_data[2] = 0x74;
	    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
		mdelay( 1 );
	    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);
	    major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
	    minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
	}	
    TP_DEBUG("***FW Version major = %d ***\n", major);
    TP_DEBUG("***FW Version minor = %d ***\n", minor);

	sprintf(fw_version,"%03d%03d", major, minor);	    
	printk("***fw_version = %s ***\n", fw_version);
		
    //_HalTscrHWReset();    
	
    return size;
}



static DEVICE_ATTR( version, 0777, firmware_version_show, firmware_version_store );

static ssize_t firmware_data_show( struct device *dev,
                                   struct device_attribute *attr, char *buf )
{
    return FwDataCnt;
}

static ssize_t firmware_data_store( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{

    int i;
    TP_DEBUG( "***FwDataCnt = %d ***\n", FwDataCnt );
    memcpy( temp[FwDataCnt], buf, 1024 );
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR( data, 0777, firmware_data_show, firmware_data_store );
#endif

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = TPD_DEVICE,
     //   .owner = THIS_MODULE,
    },

    .probe = tpd_probe,
    .remove = __devexit_p(tpd_remove),
    .id_table = tpd_id,
    .detect = tpd_detect,
    //.address_data = &addr_data,
};


static  void tpd_down(int x, int y)
{
	input_report_key(tpd->dev, BTN_TOUCH, 1);
//	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	TPD_DOWN_DEBUG_TRACK(x, y);
}

static void tpd_up(int x, int y)
{
    //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    	input_mt_sync(tpd->dev);
}

unsigned char tpd_check_sum(unsigned char *pval)
{
    int i, sum = 0;

    for(i = 0; i < 7; i++)
    {
        sum += pval[i];
    }

    return (unsigned char)((-sum) & 0xFF);
}

static bool msg2033_i2c_read(char *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->timing = 100;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	//printk("enter msg2033_i2c_read!\n ");        //pjdebug
	ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

	if(ret <= 0)
	{
		pr_tp("msg_i2c_read_interface error\n");
		return false;
	}

	return true;
}

static int tpd_touchinfo(struct touch_info *cinfo)
{
    SHORT_TOUCH_STATE ShortTouchState;
    BYTE reg_val[8] = {0};
    unsigned int  temp = 0;

#ifdef TPD_PROXIMITY		
    int err;		
    hwm_sensor_data sensor_data;
#endif

	p_point_num = point_num;
    if(update_switch)
    {
        return false;
    }

	if (false == msg2033_i2c_read(reg_val, 8))
	{
	    return false;
	}

    ShortTouchState.pos_x = ((reg_val[1] & 0xF0) << 4) | reg_val[2];
    ShortTouchState.pos_y = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
    ShortTouchState.dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
    ShortTouchState.dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

    if((ShortTouchState.dst_x) & 0x0800)
    {
        ShortTouchState.dst_x |= 0xF000;
    }

    if((ShortTouchState.dst_y) & 0x0800)
    {
        ShortTouchState.dst_y |= 0xF000;
    }

    ShortTouchState.pos_x2 = ShortTouchState.pos_x + ShortTouchState.dst_x;
    ShortTouchState.pos_y2 = ShortTouchState.pos_y + ShortTouchState.dst_y;
    //printk("MSG_ID=0x%x,X= %d ,Y=%d\n", reg_val[0], ShortTouchState.pos_x, ShortTouchState.pos_x);
    temp = tpd_check_sum(reg_val);

    if(temp == reg_val[7])
    {
        //printk("TP_PS \nreg_val[1]=0x%x\nreg_val[2]=0x%x\nreg_val[6]=0x%x\nreg_val[7]=0x%x by cheehwa\n", reg_val[1], reg_val[2], reg_val[6], reg_val[7]);

        if(reg_val[0] == 0x52) //CTP  ID
        {
			if(reg_val[1] == 0xFF&& reg_val[4] == 0xFF)
			{
	        #ifdef TPD_PROXIMITY
		    	if((reg_val[5] == 0x80)||(reg_val[5] == 0x40))
				{
		   			printk(TPD_DEVICE"TP_PROXIMITY g_bPsSensorOpen = %d,g_nPsSensorDate=%d\n",g_bPsSensorOpen,g_nPsSensorDate );		  
		   			if(reg_val[5] == 0x80) // close Panel
		   			{
						if((g_bPsSensorOpen == 1) /*&& (g_nPsSensorDate == 1)*/)
						{
							mutex_lock(&msg2133_sensor_mutex);
							g_nPsSensorDate = 0;
							mutex_unlock(&msg2133_sensor_mutex);
						}
		   			}
		    			else// Open Panel
		    			{
							if(/*(g_bPsSensorOpen == 1) && */(g_nPsSensorDate == 0))
							{
								mutex_lock(&msg2133_sensor_mutex);
								g_nPsSensorDate = 1;
								mutex_unlock(&msg2133_sensor_mutex);
							}
		    			}
					sensor_data.values[0] = g_nPsSensorDate;
					sensor_data.value_divide = 1;
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;				
					if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
						printk(" Proximity call hwmsen_get_interrupt_data failed= %d\n", err);	
					}	
					return 0;
								
				}
			#ifdef CTP_PROXIMITY_ESD_MEASURE
				else if(reg_val[5] == 0xC0) 
				{			
					if(g_bPsSensorOpen == 1)
					{
						msg2133_enable_ps(1);
					}
				}
			#endif
            #endif
            
		    #ifdef TPD_HAVE_BUTTON
		        printk("reg_val[5]  virtual value=%x \r\n",reg_val[5] );
				if(reg_val[5] == 0x01)
				{
				     cinfo->x[0] = tpd_keys_dim_local[0][0];
				     cinfo->y[0] = tpd_keys_dim_local[0][1];
				     point_num = 1;
				}
				#if (TPD_KEY_COUNT>=2)
				else if(reg_val[5] == 0x02)
				{
				     cinfo->x[0] = tpd_keys_dim_local[1][0];
				     cinfo->y[0] = tpd_keys_dim_local[1][1];
				     point_num = 1;
				}
				#endif
				#if (TPD_KEY_COUNT>=3)
				else if(reg_val[5] == 0x08)
				{
				     cinfo->x[0] = tpd_keys_dim_local[2][0];
				     cinfo->y[0] = tpd_keys_dim_local[2][1];
				     point_num = 1;
				}
				#endif
				#if (TPD_KEY_COUNT>=4)
				else if(reg_val[5] == 0x04)
				{
				     cinfo->x[0] = tpd_keys_dim_local[3][0];
				     cinfo->y[0] = tpd_keys_dim_local[3][1];
				     point_num = 1;
				}
				#endif
				else
#endif
				{
					point_num = 0;
				}
			}
			else if(ShortTouchState.pos_x > 2047 || ShortTouchState.pos_y > 2047)
			{
				return  false;
			}
			else if((ShortTouchState.dst_x == 0) && (ShortTouchState.dst_y == 0))
			{
				#if defined(TPD_XY_INVERT)
       					#if defined(TPD_X_INVERT)
                  				cinfo->x[0] = (2048-ShortTouchState.pos_y) * TPD_RES_X / 2048;
      					#else
        						cinfo->x[0] = (ShortTouchState.pos_y) * TPD_RES_X / 2048;
      					#endif
  
      					#if defined(TPD_Y_INVERT)
                  				cinfo->y[0] = (2048-ShortTouchState.pos_x) * TPD_RES_Y / 2048; 
      					#else
                  				cinfo->y[0] = ShortTouchState.pos_x * TPD_RES_Y / 2048; 
      					#endif
    
    				#else
       					#if defined(TPD_X_INVERT)
                  				cinfo->x[0] = (2048-ShortTouchState.pos_x) * TPD_RES_X / 2048;
      					#else
                  				cinfo->x[0] = ShortTouchState.pos_x * TPD_RES_X / 2048;
      					#endif
						
        					#if defined(TPD_Y_INVERT)
                  				cinfo->y[0] = (2048-ShortTouchState.pos_y) * TPD_RES_Y / 2048;
     	 				#else
                  				cinfo->y[0] = ShortTouchState.pos_y * TPD_RES_Y / 2048;
      					#endif
    
    				#endif
                			point_num = 1;
			}
			else
			{
				if(ShortTouchState.pos_x2 > 2047 || ShortTouchState.pos_y2 > 2047)
					return false;
					
				#if defined(TPD_XY_INVERT)
      					#if defined(TPD_X_INVERT)
         					cinfo->x[0] = (2048-ShortTouchState.pos_y) * TPD_RES_X / 2048;
      					#else
                  				cinfo->x[0] = ShortTouchState.pos_y * TPD_RES_X / 2048;
      					#endif
						
      					#if defined(TPD_Y_INVERT)
                  				cinfo->y[0] = (2048-ShortTouchState.pos_x) * TPD_RES_Y / 2048;
      					#else
                  				cinfo->y[0] = ShortTouchState.pos_x * TPD_RES_Y / 2048;
      					#endif
						
      					#if defined(TPD_X_INVERT)
                  				cinfo->x[1] = (2048-ShortTouchState.pos_y2) * TPD_RES_X / 2048;
      					#else
                  				cinfo->x[1] = ShortTouchState.pos_y2 * TPD_RES_X / 2048;
      					#endif
						
      					#if defined(TPD_Y_INVERT)
                  				cinfo->y[1] = (2048-ShortTouchState.pos_x2) * TPD_RES_Y / 2048;
      					#else
                  				cinfo->y[1] = ShortTouchState.pos_x2 * TPD_RES_Y / 2048;
      					#endif   
    				#else    
      					#if defined(TPD_X_INVERT)
        						cinfo->x[0] = (2048-ShortTouchState.pos_x) * TPD_RES_X / 2048;
      					#else
                  				cinfo->x[0] = ShortTouchState.pos_x * TPD_RES_X / 2048;
      					#endif 
						
      					#if defined(TPD_Y_INVERT)
                  				cinfo->y[0] = (2048-ShortTouchState.pos_y) * TPD_RES_Y / 2048;
      					#else
                  				cinfo->y[0] = ShortTouchState.pos_y * TPD_RES_Y / 2048;
      					#endif  
						
      					#if defined(TPD_X_INVERT)
                  				cinfo->x[1] = (2048-ShortTouchState.pos_x2) * TPD_RES_X / 2048;
     					#else
                  				cinfo->x[1] = ShortTouchState.pos_x2 * TPD_RES_X / 2048;
      					#endif  
						
      					#if defined(TPD_Y_INVERT)
                  				cinfo->y[1] = (2048-ShortTouchState.pos_y2) * TPD_RES_Y / 2048;
      					#else
                  				cinfo->y[1] = ShortTouchState.pos_y2 * TPD_RES_Y / 2048;
      					#endif
    				#endif
               			 point_num = 2;
			}
		}
		return true;
	}
	else
	{
		return  false;
	}
}


static int touch_event_handler(void *unused)
{
    struct touch_info cinfo;
    static BYTE touch_state = 0;
    static unsigned short Pos_x = 0,Pos_y = 0;
    unsigned long time_eclapse;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
    
    do
    {
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        
        //pr_tp("TPD interrupt has been triggered\n");
        
        set_current_state(TASK_RUNNING);
        
        if(tpd_touchinfo(&cinfo))
        {
            printk("MSG_X = %d,MSG_Y = %d\n", cinfo.x[0], cinfo.y[0]);
            //printk("MSG_X = %d,MSG_Y = %d\n", 320-cinfo.x[0], cinfo.y[0]);
            //cinfo.x[0]=320-cinfo.x[0];
            //cinfo.x[1]=320-cinfo.x[1];
            
            if(point_num > 0)
            {  
                if(point_num == 1)
                {
                    //if(boot_mode != NORMAL_BOOT)
                    if ((FACTORY_BOOT == boot_mode) || (RECOVERY_BOOT == boot_mode))
                    {
                        if(cinfo.y[0] > TPD_HEIGHT)
                            tpd_button(cinfo.x[0], cinfo.y[0], 1);
                        else
                            tpd_down(cinfo.x[0], cinfo.y[0]);
                    }
                    else
                    {
                        tpd_down(cinfo.x[0], cinfo.y[0]);
                    }
                    
                    input_sync(tpd->dev);
                }
                else if(point_num == 2)
                {
                    tpd_down(cinfo.x[0], cinfo.y[0]);
                    tpd_down(cinfo.x[1], cinfo.y[1]);
                    input_sync(tpd->dev);
                }
            }
            else 
            {
                if(p_point_num == 1)
                {
                    tpd_up(cinfo.x[0], cinfo.y[0]);
                }
                else if(p_point_num == 2)
                {
                    tpd_up(cinfo.x[0], cinfo.y[0]);
                    tpd_up(cinfo.x[1], cinfo.y[1]);
                }
                
                //if(boot_mode != NORMAL_BOOT)
                if ((FACTORY_BOOT == boot_mode) || (RECOVERY_BOOT == boot_mode))
                {
                    if(p_point_num == 1)
                    {
                        tpd_button(cinfo.x[0], cinfo.y[0], 0);
                    }
                }
                
                input_mt_sync(tpd->dev);	
                input_sync(tpd->dev);
            }
        }
    }while(!kthread_should_stop());
    
    return 0;
}

static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    //pr_tp("TPD interrupt has been triggered\n");
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}


static int msg2133_readid(void)
{
    //unsigned char aucBuffer[4] = {0x53, 0x00, 0x74};
    unsigned char aucBuffer[4] = {0x53, 0x00, 0x2A};
    int ret, result = -1;
    int iTimeout = 0;
    
    tpd_firmware_version[0] = 0x00;
    tpd_firmware_version[1] = 0x00;
    while (iTimeout++ < 5)
    {
        ret = i2c_master_send(i2c_client, aucBuffer, 3);
        if (ret <= 0)
        {
            printk("%s() send cmd fail!(%x, %x)\n",__func__, ret, i2c_client->addr);
            
            continue;    
        }
        
        msleep(50);
        
        if (false == msg2033_i2c_read(aucBuffer, 4))
        {
            printk("%s() read id error!!\n", __func__);
            result = -1;
        }
        else
        {        
            printk("%s() %x %x %x %x \n",__func__, aucBuffer[0], aucBuffer[1], aucBuffer[2], aucBuffer[3]);
        
            //tpd_firmware_version[0] = aucBuffer[1];
            tpd_firmware_version[0] = (aucBuffer[1] << 8) + aucBuffer[0];

            //tpd_firmware_version[1] = aucBuffer[3];
            tpd_firmware_version[1] = (aucBuffer[3] << 8) + aucBuffer[2];
            result = 0;

            break;
        }
    };

    return result;
}

#ifdef TPD_PROXIMITY
static int msg2133_enable_ps(int enable)
{
	u8 ps_store_data[4];

	mutex_lock(&msg2133_sensor_mutex);

	printk("msg2133 do enable: %d, current state: %d\n", enable, g_bPsSensorOpen);
	if(enable == 1)
	{
		if(g_bPsSensorOpen == 0)
		{
			ps_store_data[0] = 0x52;
			ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x62;
			ps_store_data[3] = 0xa0;		
			printk("msg2133 do enable: %d, current state: %d\n", enable, g_bPsSensorOpen);
			HalTscrCDevWriteI2CSeq(0x26, &ps_store_data[0], 4);
			g_bPsSensorOpen = 1;
		}
	}
	else
	{	
		if(g_bPsSensorOpen == 1)
		{
			ps_store_data[0] = 0x52;
			ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x62;
			ps_store_data[3] = 0xa1;
			HalTscrCDevWriteI2CSeq(0x26, &ps_store_data[0], 4);	
			g_bPsSensorOpen = 0;			
		}
		g_nPsSensorDate = 1;
	}
	mutex_unlock(&msg2133_sensor_mutex);
	return 0;
}

int msg2133_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;

	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					printk("msg2133_ps_operate++++++++1\n");
					wake_lock(&ps_lock);		//wujinyou
					if(err = msg2133_enable_ps(1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					g_bPsSensorOpen = 1;
				}
				else
				{
					printk("msg2133_ps_operate++++++++0\n");
					wake_unlock(&ps_lock);		//wujinyou
					if(err = msg2133_enable_ps(0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					g_bPsSensorOpen = 0;
				}
			}
			break;

		case SENSOR_GET_DATA:
			printk("msg2133_ps get date: %d\n", g_nPsSensorDate);
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				sensor_data->values[0] = g_nPsSensorDate;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


static void tpd_initialize_ps_sensor_function()
{
	struct hwmsen_object obj_ps = {0};
	int err = 0;
	
	g_nPsSensorDate = 1;

	obj_ps.self = NULL;	// no use
	obj_ps.polling = 1;
	obj_ps.sensor_operate = msg2133_ps_operate;

	wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock"); 
		
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		TPD_DEBUG("attach fail = %d\n", err);
		return;
	}
}
#endif

#ifdef TPD_PROXIMITY
static int msg21xx_alsps_probe(struct platform_device *pdev)
{
    pr_tp("%s enter!\n", __FUNCTION__);
    return 0;
}

static int msg21xx_alsps_remove(struct platform_device *pdev)
{
    pr_tp("%s enter!\n", __FUNCTION__);
    return 0;
}
//************* For GT9XXF End **********************//


static struct platform_driver msg21xx_alsps_driver = {   
    .probe      = msg21xx_alsps_probe,	
    .remove     = msg21xx_alsps_remove,
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
#endif

#ifdef POWERUP_AUTO_CHECK

static struct wake_lock tp_lock;

static int touch_firmware_upgrade_thread(void *unused)
{
    unsigned short msg2133_fm_major = 0, msg2133_fm_minor = 0;

	msg2133_fm_major = tpd_firmware_version[0];
    msg2133_fm_minor = tpd_firmware_version[1];

    printk("firmware major minor(%x, %x)\n",msg2133_fm_major, msg2133_fm_minor);

	//if(msg2133_read_fw_ver_custom() >= 0)
	{
	    unsigned short msg2133_bin_major, msg2133_bin_minor;
	    
    	//msg2133_bin_major=(caucMsgFirmware[0x3076]<<8) | caucMsgFirmware[0x3077];
    	//msg2133_bin_minor=(caucMsgFirmware[0x3074]<<8) | caucMsgFirmware[0x3075];

    	msg2133_bin_major=(caucMsgFirmware[0x7f4d]<<8) | caucMsgFirmware[0x7f4e];
    	msg2133_bin_minor=(caucMsgFirmware[0x7f4f]<<8) | caucMsgFirmware[0x7f50];

        printk("bin major minor(%x, %x)\n",msg2133_bin_major, msg2133_bin_minor);
        
		if(msg2133_fm_major==0xFF||msg2133_fm_major==0x00||((msg2133_fm_major==msg2133_bin_major)&&(msg2133_fm_minor<msg2133_bin_minor)))
		{
		    printk("%s() enter msg2133 update!! \n",__func__);
            wake_lock_init(&tp_lock, WAKE_LOCK_SUSPEND, "tp wakelock"); 
            wake_lock(&tp_lock);

		    memset(temp, sizeof(temp), 0);
            memcpy(temp, caucMsgFirmware, sizeof(caucMsgFirmware));
			firmware_update_store(NULL,NULL,NULL,0);
			
			msg2133_readid();
	
            wake_unlock(&tp_lock);
            wake_lock_destroy(&tp_lock);	
		}
        else
        {
    	    printk("%s() msg2133 no need update!! \n",__func__);
        }
	}
	
	kthread_should_stop(); 	
}
#endif

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = TPD_OK;
    //char data;
    //client->timing = 400;
     U8 ps_store_data[4];
     
    i2c_client = client;
	
	i2c_client->timing = 100;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	    
    printk("In tpd_probe_ ,the i2c addr=0x%x", client->addr);
    
#if 1//def TPD_CLOSE_POWER_IN_SLEEP
    //hwPowerDown(TPD_POWER_SOURCE, "TP");
    //hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");
    //msleep(100);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#else
    //  mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    //    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    //  mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
#endif

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    //mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    //mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    msleep(100);

//for( i=0;i<4;i++)
 //       __set_bit(tpd_keys[i], tpd->dev->keybit);
    //tpd->kpd->id.bustype = BUS_HOST;
    //tpd->kpd->id.vendor  = 0x0001;
    //tpd->kpd->id.product = 0x0001;
    //tpd->kpd->id.version = 0x0100;
   // if(input_register_device(tpd->dev))
    //    TPD_DMESG("input_register_device failed.(kpd)\n");
	
    /*
        {
            msg2033_i2c_read(&data,1);

            pr_tp("The CTP_ID=0x%x in %s(),L:%d\n",data,__func__,__LINE__);
        }

        if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data)) < 0)
        {
            pr_tp("I2C transfer error, func: %s\n", __func__);
            return -1;
        }
        pr_tp("The CTP_ID=0x%x in %s(),L:%d\n",data,__func__,__LINE__);
    */
    
    if (msg2133_readid() != 0) {
        goto failed;
    }
    tpd_load_status = 1;
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if(IS_ERR(thread))
    {
        retval = PTR_ERR(thread);
        pr_tp(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
        goto failed;
    }

    /*
        if (TRUE==msg2033_i2c_read(data,1))
        {
            pr_ch("Mstar 's TP\n");
            SMC_SWITCH=1;
        }
    */

#ifdef TP_FIRMWARE_UPDATE

    #ifdef POWERUP_AUTO_CHECK
        kthread_run(touch_firmware_upgrade_thread, 0, TPD_DEVICE);
    #endif
    
    firmware_class = class_create( THIS_MODULE, "ms-touchscreen-msg20xx" );
    if( IS_ERR( firmware_class ) )
        pr_err( "Failed to create class(firmware)!\n" );
    firmware_cmd_dev = device_create( firmware_class,
                                      NULL, 0, NULL, "device" );
    if( IS_ERR( firmware_cmd_dev ) )
        pr_err( "Failed to create device(firmware_cmd_dev)!\n" );

    // version
    if( device_create_file( firmware_cmd_dev, &dev_attr_version ) < 0 )
        pr_err( "Failed to create device file(%s)!\n", dev_attr_version.attr.name );
    // update
    if( device_create_file( firmware_cmd_dev, &dev_attr_update ) < 0 )
        pr_err( "Failed to create device file(%s)!\n", dev_attr_update.attr.name );
    // data
    if( device_create_file( firmware_cmd_dev, &dev_attr_data ) < 0 )
        pr_err( "Failed to create device file(%s)!\n", dev_attr_data.attr.name );
    // clear
    if( device_create_file( firmware_cmd_dev, &dev_attr_clear ) < 0 )
        pr_err( "Failed to create device file(%s)!\n", dev_attr_clear.attr.name );

    dev_set_drvdata( firmware_cmd_dev, NULL );
#endif

//    pr_k("create device file:%s\n", dev_attr_proximity_sensor.attr.name);
    printk("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
        
#ifdef TPD_PROXIMITY
    if(platform_driver_register(&msg21xx_alsps_driver)) {
		pr_tp("failed to register platform_driver_register(&gt9xx_alsps_driver)\n");
	}
	tpd_initialize_ps_sensor_function();
#endif

    return 0;

failed:
    return -1;

}

static int __devexit tpd_remove(struct i2c_client *client)

{
    printk("TPD removed\n");
#ifdef TP_FIRMWARE_UPDATE
#if ENABLE_DMA

    if(gpDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, gpDMABuf_va, gpDMABuf_pa);
        gpDMABuf_va = NULL;
        gpDMABuf_pa = NULL;
    }

#endif
#endif
    return 0;
}


static int tpd_local_init(void)
{
    printk(" MSG2133 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

	boot_mode = get_boot_mode();
	//if(boot_mode == 3)
	{
    //    boot_mode = NORMAL_BOOT;
    }

    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        printk("unable to add i2c driver.\n");
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
    pr_tp("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;
    return 0;
}

static int tpd_resume(struct i2c_client *client)
{
    int retval = TPD_OK;
    printk("TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");
#else
     //mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    //    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    //  mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    //    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    //msleep(200);
    /*
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
        msleep(1);
    */
    //mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
    msleep(20);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_PROXIMITY
	if(g_bPsSensorOpen == 1)
	{
		g_bPsSensorOpen = 0;
		TPD_DMESG("msg sensor resume in calling %d \n", g_bPsSensorOpen);
		msg2133_enable_ps(1);// CTP Proximity function open command  again ;
	}
#endif	
    return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
    int retval = TPD_OK;
	uint8_t ProximityEn;

#ifdef TP_FIRMWARE_UPDATE
//Chenzz++ 20130606 防止升级过程中灭屏关掉TP电源
    if (update_switch)
    {
        printk("WARN!! TPD on update~~ force return~~\n");
		return TPD_OK;
    }
#endif

#ifdef TPD_PROXIMITY
	if(g_bPsSensorOpen == 1)//if(g_nPsSensorDate == 0)
	{
		TPD_DMESG("msg suspend in calling tp no need to suspend\n");
		return TPD_OK;
	}
	g_bSuspend = 1;
#endif
    printk("TPD enter sleep\n");
	

    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    //mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerDown(TPD_POWER_SOURCE, "TP");

    //i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
    //mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    //mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#endif

    return retval;
}


static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "MSG2133",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};
/* called when loaded into kernel */
static int __init msg2133_tpd_driver_init(void)
{
    printk("MediaTek MSG2133 touch panel driver init\n");
    
    //i2c_register_board_info(0, &msg2133_i2c_tpd, 1);
    i2c_register_board_info(1, &msg2133_i2c_tpd, 1);
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
        printk("add MSG2133 driver failed\n");
    }

    return 0;
}

/* should never be called */
static void __exit msg2133_tpd_driver_exit(void)
{
    printk("MediaTek MSG2133 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(msg2133_tpd_driver_init);
module_exit(msg2133_tpd_driver_exit);


