
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
#define LCM_ID          (0x9486)

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
    send_ctrl_cmd(0xC0); 
    send_data_cmd(0x10); 
    send_data_cmd(0x10); 
     
    
    send_ctrl_cmd(0xC1); 
    send_data_cmd(0x45); 
    send_data_cmd(0x07); 
     
    
    send_ctrl_cmd(0xC5); 
    send_data_cmd(0x00); 
    send_data_cmd(0x30); 
    send_data_cmd(0x80); 
    
    
    send_ctrl_cmd(0xC2); 
    send_data_cmd(0x33); 
    
    send_ctrl_cmd(0xb1); 
    send_data_cmd(0xd0); 
    send_data_cmd(0x11); 
    
    send_ctrl_cmd(0xb4); 
    send_data_cmd(0x02); 
    
    send_ctrl_cmd(0xb6); 
    send_data_cmd(0x00); 
    send_data_cmd(0x02); 
    send_data_cmd(0x3b); 
    
    send_ctrl_cmd(0xb7); 
    send_data_cmd(0x07);
    
    send_ctrl_cmd(0xF0); 
    send_data_cmd(0x36); 
    send_data_cmd(0xA5); 
    send_data_cmd(0xD3);
    
    send_ctrl_cmd(0xE5); 
    send_data_cmd(0x80);
    
    send_ctrl_cmd(0xF0); 
    send_data_cmd(0x36); 
    send_data_cmd(0xA5); 
    send_data_cmd(0x53);
    
    send_ctrl_cmd(0xF0); 
    send_data_cmd(0x02); 
    send_data_cmd(0x37); 
    send_data_cmd(0x20);
    send_data_cmd(0x00); 
    send_data_cmd(0x00); 
    send_data_cmd(0x00);
    send_data_cmd(0x02); 
    send_data_cmd(0x37); 
    send_data_cmd(0x20);
    send_data_cmd(0x00); 
    send_data_cmd(0x00); 
    send_data_cmd(0x00);

    send_ctrl_cmd(0x35);    //TE ON V-sync
    send_data_cmd(0x00);
    
    send_ctrl_cmd(0x36); 
    send_data_cmd(0x48);
    
    send_ctrl_cmd(0x3A); 
    send_data_cmd(0x66);
    
    
    send_ctrl_cmd(0x11); 
    MDELAY(150); 
    send_ctrl_cmd(0x20); 
    
    send_ctrl_cmd(0x29);
    MDELAY(150);
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
#if 0
    memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
    // enable tearing-free
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = CMD_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;
    params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
    
    // Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =8;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =17;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
#endif

#else
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

   params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
#endif
}

static unsigned int lcm_compare_id(void)
{
    unsigned char ucID0, ucID1, ucID2;

    send_ctrl_cmd(0xD3);
    
    read_data_cmd();    //Null
    
    ucID0 = read_data_cmd();
    ucID1 = read_data_cmd();    //94
    ucID2 = read_data_cmd();    //86

#ifdef BUILD_LK
    printf("RM68140 read id = (0x%x, 0x%x, 0x%x)\n",ucID0, ucID1, ucID2);
#endif

    if((ucID1 == (LCM_ID / 256)) && (ucID2 == (LCM_ID & 0xff)))
    {
#ifdef BUILD_LK
        printf("Match rm68140 lcm driver ok!\n");
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
    //lcm_compare_id();
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
   sw_clear_panel(0);
   send_ctrl_cmd(0x10);
   MDELAY(5);
}


static void lcm_resume(void)
{
   send_ctrl_cmd(0x11);
   MDELAY(120);
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
    if(level > 255) level = 255;
    
    send_ctrl_cmd(0x51);
    send_data_cmd(level);	
}


LCM_DRIVER rm68140_mcu_6572_lcm_drv = 
{
   .name			= "rm68140",
   .set_util_funcs = lcm_set_util_funcs,
   .get_params     = lcm_get_params,
   .init           = lcm_init,
   .suspend        = lcm_suspend,
   .resume         = lcm_resume,
   .update         = lcm_update,
   //.set_backlight	= lcm_setbacklight,
   .compare_id     = lcm_compare_id,
};


