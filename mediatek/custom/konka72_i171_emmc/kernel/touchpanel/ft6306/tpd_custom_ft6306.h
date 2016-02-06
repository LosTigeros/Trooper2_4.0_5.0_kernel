/*****************************************************************************
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *  tpd_custom_ft5306.h
 *
 * Project:
 * --------
 *  W900
 *
 * Author:
 * -------
 *  cheehwa,"yuanjianhua@konka.com"
 *
 * Description:
 * ------------
 *  The head fiel for CTP's driver, FocalTech's chip FT5306
 *
 *
 *============================================================================
 * History:
 * Sun Feb 12 2012, creat
 *
 *============================================================================
 ****************************************************************************/

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
//#define TPD_TYPE_RESISTIVE
#define TPD_I2C_NUMBER           1
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_DELAY                (2*HZ/100)
#define TPD_RES_X                480
#define TPD_RES_Y                854
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
#define TPD_HAVE_TREMBLE_ELIMINATION

//#define TPD_HAVE_POWER_ON_OFF
#define PRESSURE_FACTOR 10

#define TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGHT       854
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM            {{55,900,110,50},{240,900,110,50},{424,900,110,50}}


#endif /* TOUCHPANEL_H__ */
