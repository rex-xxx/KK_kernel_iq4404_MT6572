/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 09 10 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .10y dual sensor
 *
 * 09 02 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .roll back dual sensor
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Feb 24 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Aug 5 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Apr 7 2009 mtk02204
 * [DUMA00004012] [Camera] Restructure and rename camera related custom folders and folder name of came
 * 
 *
 * Mar 26 2009 mtk02204
 * [DUMA00003515] [PC_Lint] Remove PC_Lint check warnings of camera related drivers.
 * 
 *
 * Mar 2 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Feb 24 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Dec 27 2008 MTK01813
 * DUMA_MBJ CheckIn Files
 * created by clearfsimport
 *
 * Dec 10 2008 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Oct 27 2008 mtk01051
 * [DUMA00000851] Camera related drivers check in
 * Modify Copyright Header
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H


typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_CAPTURE
} OV5645MIPI_SENSOR_MODE;

typedef enum _OV5645MIPI_OP_TYPE_ {
        OV5645MIPI_MODE_NONE,
        OV5645MIPI_MODE_PREVIEW,
        OV5645MIPI_MODE_CAPTURE,
        OV5645MIPI_MODE_QCIF_VIDEO,
        OV5645MIPI_MODE_CIF_VIDEO,
        OV5645MIPI_MODE_QVGA_VIDEO
    } OV5645MIPI_OP_TYPE;

extern OV5645MIPI_OP_TYPE OV5645MIPI_g_iOV5645MIPI_Mode;

#define OV5645MIPI_ID_REG                          (0x300A)
#define OV5645MIPI_INFO_REG                        (0x300B)
 
/* sensor size */
#define OV5645MIPI_IMAGE_SENSOR_SVGA_WIDTH          (1280)
#define OV5645MIPI_IMAGE_SENSOR_SVGA_HEIGHT         (960)
#define OV5645MIPI_IMAGE_SENSOR_QSXGA_WITDH         (2560)//(2592) 
#define OV5645MIPI_IMAGE_SENSOR_QSXGA_HEIGHT        (1920)//(1944)
#define OV5645MIPI_IMAGE_SENSOR_VIDEO_WITDH         (1280) 
#define OV5645MIPI_IMAGE_SENSOR_VIDEO_HEIGHT        (960)

#define OV5645MIPI_IMAGE_SENSOR_720P_WIDTH          (1280)
#define OV5645MIPI_IMAGE_SENSOR_720P_HEIGHT         (720)


/* Sesnor Pixel/Line Numbers in One Period */	
#define OV5645MIPI_PV_PERIOD_PIXEL_NUMS    		(1896)  	/* Default preview line length HTS*/
#define OV5645MIPI_PV_PERIOD_LINE_NUMS     		(984)   	/* Default preview frame length  VTS*/
#define OV5645MIPI_FULL_PERIOD_PIXEL_NUMS  		(2844)  	/* Default full size line length */
#define OV5645MIPI_FULL_PERIOD_LINE_NUMS   		(1968)  	/* Default full size frame length */

/* Sensor Exposure Line Limitation */
#define OV5645MIPI_PV_EXPOSURE_LIMITATION      	(984-4)
#define OV5645MIPI_FULL_EXPOSURE_LIMITATION    	(1968-4)

/* Config the ISP grab start x & start y, Config the ISP grab width & height */
#define OV5645MIPI_PV_GRAB_START_X 				   (0)
#define OV5645MIPI_PV_GRAB_START_Y  			(1)
#define OV5645MIPI_FULL_GRAB_START_X   			(0)
#define OV5645MIPI_FULL_GRAB_START_Y	  		(1)

/*50Hz,60Hz*/
#define OV5645MIPI_NUM_50HZ                        (50 * 2)
#define OV5645MIPI_NUM_60HZ                        (60 * 2)

/* FRAME RATE UNIT */
#define OV5645MIPI_FRAME_RATE_UNIT                 (10)

/* MAX CAMERA FRAME RATE */
#define OV5645MIPI_MAX_CAMERA_FPS                  (OV5645MIPI_FRAME_RATE_UNIT * 30)

#define OV5645_PREVIEW_MODE             0
#define OV5645_VIDEO_MODE               1
#define OV5645_PREVIEW_FULLSIZE_MODE    2


/* SENSOR READ/WRITE ID */
#define OV5645MIPI_WRITE_ID						    0x78
#define OV5645MIPI_READ_ID							0x79

UINT32 OV5645MIPIopen(void);
UINT32 OV5645MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 OV5645MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV5645MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV5645MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 OV5645MIPIClose(void);
UINT32 OV5645MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT pfFunc);
#endif /* __SENSOR_H */
