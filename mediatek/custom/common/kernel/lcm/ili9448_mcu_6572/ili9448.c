
#ifdef BUILD_LK

#else
    #include <linux/string.h>
#endif

#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH     320
#define FRAME_HEIGHT    480
#define LCM_ID          (0x9488)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

static __inline void send_ctrl_cmd(unsigned int cmd)
{
   lcm_util.send_cmd(cmd);
}


static __inline void send_data_cmd(unsigned int data)
{
   lcm_util.send_data(data&0xff);
}


static __inline unsigned int read_data_cmd(void)
{
   return 0xFF&lcm_util.read_data();
}


static void sw_clear_panel(unsigned int color)
{
   unsigned short x0, y0, x1, y1, x, y;
   unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;
   
   x0 = (unsigned short)0;
   y0 = (unsigned short)0;
   x1 = (unsigned short)FRAME_WIDTH-1;
   y1 = (unsigned short)FRAME_HEIGHT-1;
   
   h_X_start=((x0&0xFF00)>>8);
   l_X_start=(x0&0x00FF);
   h_X_end=((x1&0xFF00)>>8);
   l_X_end=(x1&0x00FF);
   
   h_Y_start=((y0&0xFF00)>>8);
   l_Y_start=(y0&0x00FF);
   h_Y_end=((y1&0xFF00)>>8);
   l_Y_end=(y1&0x00FF);
   
   send_ctrl_cmd(0x2A);
   send_data_cmd(h_X_start); 
   send_data_cmd(l_X_start); 
   send_data_cmd(h_X_end); 
   send_data_cmd(l_X_end); 
   
   send_ctrl_cmd(0x2B);
   send_data_cmd(h_Y_start); 
   send_data_cmd(l_Y_start); 
   send_data_cmd(h_Y_end); 
   send_data_cmd(l_Y_end); 
   
   send_ctrl_cmd(0x29); 
   
   send_ctrl_cmd(0x2C);
   for (y = y0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
         lcm_util.send_data(color);
      }
   }
}


static void init_lcm_registers(void)
{
//************* Start Initial Sequence **********//
    send_ctrl_cmd(0XF7);
    send_data_cmd(0xA9);
    send_data_cmd(0x51);
    send_data_cmd(0x2C);
    send_data_cmd(0x82);
    
    send_ctrl_cmd(0xC0);
    send_data_cmd(0x11);
    send_data_cmd(0x09);
    
    send_ctrl_cmd(0xC1);
    send_data_cmd(0x41);
    
    send_ctrl_cmd(0XC5);
    send_data_cmd(0x00);
    send_data_cmd(0x2A);
    send_data_cmd(0x80);
    
    send_ctrl_cmd(0xB1);
    send_data_cmd(0xB0);
    send_data_cmd(0x11);
    
    send_ctrl_cmd(0xB4);
    send_data_cmd(0x02);
    
    send_ctrl_cmd(0xB6);
    send_data_cmd(0x02);
    send_data_cmd(0x22);
    send_data_cmd(0x3B);
    
    send_ctrl_cmd(0xB7);
    send_data_cmd(0xC6);
    
    send_ctrl_cmd(0xBE);
    send_data_cmd(0x00);
    send_data_cmd(0x04);
    
    send_ctrl_cmd(0xE9);
    send_data_cmd(0x00);
    
    send_ctrl_cmd(0x36);
    send_data_cmd(0x08);

//Chenzz++ i110s ili9448 使用TE ON 后会出现切换,
//TE ON V-sync
    //send_ctrl_cmd(0x35);    
    //send_data_cmd(0x00);
    
//TE OFF    
    send_ctrl_cmd(0x34);    
    
//end
    
    
    send_ctrl_cmd(0x3A);        //18bit
    //send_data_cmd(0x55);
    send_data_cmd(0x66);
    
    send_ctrl_cmd(0xE0);
    send_data_cmd(0x00);
    send_data_cmd(0x07);
    send_data_cmd(0x12);
    send_data_cmd(0x0B);
    send_data_cmd(0x18);
    send_data_cmd(0x0B);
    send_data_cmd(0x3F);
    send_data_cmd(0x9B);
    send_data_cmd(0x4B);
    send_data_cmd(0x0B);
    send_data_cmd(0x0F);
    send_data_cmd(0x0B);
    send_data_cmd(0x15);
    send_data_cmd(0x17);
    send_data_cmd(0x0F);
    
    send_ctrl_cmd(0XE1);
    send_data_cmd(0x00);
    send_data_cmd(0x16);
    send_data_cmd(0x1B);
    send_data_cmd(0x02);
    send_data_cmd(0x0F);
    send_data_cmd(0x06);
    send_data_cmd(0x34);
    send_data_cmd(0x46);
    send_data_cmd(0x48);
    send_data_cmd(0x04);
    send_data_cmd(0x0D);
    send_data_cmd(0x0D);
    send_data_cmd(0x35);
    send_data_cmd(0x36);
    send_data_cmd(0x0F);
    
    send_ctrl_cmd(0x11);
    MDELAY(120);
    send_ctrl_cmd(0x29);
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
   
   params->type   = LCM_TYPE_DBI;
   params->ctrl   = LCM_CTRL_PARALLEL_DBI;
   params->width  = FRAME_WIDTH;
   params->height = FRAME_HEIGHT;
   params->io_select_mode = 2;

   params->dbi.port                    = 0;
   params->dbi.clock_freq              = LCM_DBI_CLOCK_FREQ_104M;
   params->dbi.data_width              = LCM_DBI_DATA_WIDTH_18BITS;
   params->dbi.data_format.color_order = LCM_COLOR_ORDER_RGB;
   params->dbi.data_format.trans_seq   = LCM_DBI_TRANS_SEQ_MSB_FIRST;
   params->dbi.data_format.padding     = LCM_DBI_PADDING_ON_LSB;
   params->dbi.data_format.format      = LCM_DBI_FORMAT_RGB666;
   params->dbi.data_format.width       = LCM_DBI_DATA_WIDTH_18BITS;
   params->dbi.cpu_write_bits          = LCM_DBI_CPU_WRITE_32_BITS;
   params->dbi.io_driving_current      = LCM_DRIVING_CURRENT_8MA;
   
   params->dbi.parallel.write_setup    = 2;
   params->dbi.parallel.write_hold     = 2;
   params->dbi.parallel.write_wait     = 10;
   params->dbi.parallel.read_setup     = 2;
   params->dbi.parallel.read_hold      = 2;
   params->dbi.parallel.read_latency   = 10;
   params->dbi.parallel.wait_period    = 0;

//Chenzz++ i110s ili9448 使用TE ON 后会出现切换,
   //params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
}

static unsigned int lcm_compare_id(void)
{
    unsigned char ucID0, ucID1, ucID2;

    send_ctrl_cmd(0xD3);
    
    read_data_cmd();    //Null
    
    ucID0 = read_data_cmd();
    ucID1 = read_data_cmd();
    ucID2 = read_data_cmd();

#ifdef BUILD_LK
    printf("read id = (0x%x, 0x%x, 0x%x)\n",ucID0, ucID1, ucID2);
#endif

    if((ucID1 == (LCM_ID / 256)) && (ucID2 == (LCM_ID & 0xff)))
    {
#ifdef BUILD_LK
        printf("Match ili9488 lcm driver ok!\n");
#endif

        return 1;
    }
    else
    {
        return 0;
    }
}


static void lcm_init(void)
{
#ifdef BUILD_LK
    lcm_compare_id();
#endif

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(25);
    SET_RESET_PIN(1);
    MDELAY(50);

    init_lcm_registers();
}


static void lcm_suspend(void)
{
    //sw_clear_panel(0);
    send_ctrl_cmd(0x28);
    MDELAY(10); 
    send_ctrl_cmd(0x10);
    MDELAY(120); 
}


static void lcm_resume(void)
{
   send_ctrl_cmd(0x11);
   MDELAY(120);
   send_ctrl_cmd(0x29);
}


static void lcm_update(unsigned int x, unsigned int y,
                                       unsigned int width, unsigned int height)
{
    unsigned short x0, y0, x1, y1;
    unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;
    
    
    x0 = (unsigned short)x;
    y0 = (unsigned short)y;
    x1 = (unsigned short)x+width-1;
    y1 = (unsigned short)y+height-1;
    
    h_X_start=((x0&0xFF00)>>8);
    l_X_start=(x0&0x00FF);
    h_X_end=((x1&0xFF00)>>8);
    l_X_end=(x1&0x00FF);
    
    h_Y_start=((y0&0xFF00)>>8);
    l_Y_start=(y0&0x00FF);
    h_Y_end=((y1&0xFF00)>>8);
    l_Y_end=(y1&0x00FF);
    
    send_ctrl_cmd(0x2A);
    send_data_cmd(h_X_start); 
    send_data_cmd(l_X_start); 
    send_data_cmd(h_X_end); 
    send_data_cmd(l_X_end); 
    
    send_ctrl_cmd(0x2B);
    send_data_cmd(h_Y_start); 
    send_data_cmd(l_Y_start); 
    send_data_cmd(h_Y_end); 
    send_data_cmd(l_Y_end); 
    
    send_ctrl_cmd(0x29); 
    
    send_ctrl_cmd(0x2C);
}


static void lcm_setbacklight(unsigned int level)
{

}


LCM_DRIVER ili9448_mcu_6572_lcm_drv = 
{
   .name			= "ili9448",
   .set_util_funcs = lcm_set_util_funcs,
   .get_params     = lcm_get_params,
   .init           = lcm_init,
   .suspend        = lcm_suspend,
   .resume         = lcm_resume,
   .update         = lcm_update,
   //.set_backlight	= lcm_setbacklight,
   .compare_id     = lcm_compare_id,
};


