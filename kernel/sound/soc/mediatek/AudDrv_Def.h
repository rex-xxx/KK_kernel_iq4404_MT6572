/******************************************************************************
*
 *
 * Filename:
 * ---------
 *   AudDrv_Common.h
 *
 * Project:
 * --------
 *   MT6583 FPGA LDVT Audio Driver
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 *   Chipeng Chang (MTK02308)
 *
 *---------------------------------------------------------------------------
---
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *

*******************************************************************************/

#ifndef AUDIO_DEF_H
#define AUDIO_DEF_H

#include "AudDrv_Type_Def.h"

//#define PM_MANAGER_API
#define AUDIO_MEMORY_SRAM
#define AUDIO_MEM_IOREMAP

// below for audio debugging
#define DEBUG_AUDDRV
#define DEBUG_AFE_REG
#define DEBUG_ANA_REG
#define DEBUG_AUD_CLK

#ifdef DEBUG_AUDDRV
#define PRINTK_AUDDRV(format, args...) printk(format, ##args )
#else
#define PRINTK_AUDDRV(format, args...)
#endif

#ifdef DEBUG_AFE_REG
#define PRINTK_AFE_REG(format, args...) printk(format, ##args )
#else
#define PRINTK_AFE_REG(format, args...)
#endif

#ifdef DEBUG_ANA_REG
#define PRINTK_ANA_REG(format, args...) printk(format, ##args )
#else
#define PRINTK_ANA_REG(format, args...)
#endif

#ifdef DEBUG_AUD_CLK
#define PRINTK_AUD_CLK(format, args...)  printk(format, ##args )
#else
#define PRINTK_AUD_CLK(format, args...)
#endif

#define PRINTK_AUD_ERROR(format, args...)  printk(format, ##args )

// if need assert , use AUDIO_ASSERT(true)
#define AUDIO_ASSERT(value) BUG_ON(false)


/**********************************
 *  Other Definitions             *
 **********************************/
#define BIT_00	0x00000001        /* ---- ---- ---- ---- ---- ---- ---- ---1 */
#define BIT_01	0x00000002        /* ---- ---- ---- ---- ---- ---- ---- --1- */
#define BIT_02	0x00000004        /* ---- ---- ---- ---- ---- ---- ---- -1-- */
#define BIT_03	0x00000008        /* ---- ---- ---- ---- ---- ---- ---- 1--- */
#define BIT_04	0x00000010        /* ---- ---- ---- ---- ---- ---- ---1 ---- */
#define BIT_05	0x00000020        /* ---- ---- ---- ---- ---- ---- --1- ---- */
#define BIT_06	0x00000040        /* ---- ---- ---- ---- ---- ---- -1-- ---- */
#define BIT_07	0x00000080        /* ---- ---- ---- ---- ---- ---- 1--- ---- */
#define BIT_08	0x00000100        /* ---- ---- ---- ---- ---- ---1 ---- ---- */
#define BIT_09	0x00000200        /* ---- ---- ---- ---- ---- --1- ---- ---- */
#define BIT_10	0x00000400        /* ---- ---- ---- ---- ---- -1-- ---- ---- */
#define BIT_11	0x00000800        /* ---- ---- ---- ---- ---- 1--- ---- ---- */
#define BIT_12	0x00001000        /* ---- ---- ---- ---- ---1 ---- ---- ---- */
#define BIT_13	0x00002000        /* ---- ---- ---- ---- --1- ---- ---- ---- */
#define BIT_14	0x00004000        /* ---- ---- ---- ---- -1-- ---- ---- ---- */
#define BIT_15	0x00008000        /* ---- ---- ---- ---- 1--- ---- ---- ---- */
#define BIT_16	0x00010000        /* ---- ---- ---- ---1 ---- ---- ---- ---- */
#define BIT_17	0x00020000        /* ---- ---- ---- --1- ---- ---- ---- ---- */
#define BIT_18	0x00040000        /* ---- ---- ---- -1-- ---- ---- ---- ---- */
#define BIT_19	0x00080000        /* ---- ---- ---- 1--- ---- ---- ---- ---- */
#define BIT_20	0x00100000        /* ---- ---- ---1 ---- ---- ---- ---- ---- */
#define BIT_21	0x00200000        /* ---- ---- --1- ---- ---- ---- ---- ---- */
#define BIT_22	0x00400000        /* ---- ---- -1-- ---- ---- ---- ---- ---- */
#define BIT_23	0x00800000        /* ---- ---- 1--- ---- ---- ---- ---- ---- */
#define BIT_24	0x01000000        /* ---- ---1 ---- ---- ---- ---- ---- ---- */
#define BIT_25	0x02000000        /* ---- --1- ---- ---- ---- ---- ---- ---- */
#define BIT_26	0x04000000        /* ---- -1-- ---- ---- ---- ---- ---- ---- */
#define BIT_27	0x08000000        /* ---- 1--- ---- ---- ---- ---- ---- ---- */
#define BIT_28	0x10000000        /* ---1 ---- ---- ---- ---- ---- ---- ---- */
#define BIT_29	0x20000000        /* --1- ---- ---- ---- ---- ---- ---- ---- */
#define BIT_30	0x40000000        /* -1-- ---- ---- ---- ---- ---- ---- ---- */
#define BIT_31	0x80000000        /* 1--- ---- ---- ---- ---- ---- ---- ---- */
#define MASK_ALL          (0xFFFFFFFF)

// cpu dai name
#define MT_SOC_DAI_NAME "mt-soc-dai-driver"
#define MT_SOC_DL1DAI_NAME "mt-soc-dl1dai-driver"
#define MT_SOC_DL1DATA2DAI_NAME "mt-soc-dl1data2dai-driver"
#define MT_SOC_UL1DAI_NAME "mt-soc-ul1dai-driver"
#define MT_SOC_UL1DATA2_NAME "mt-soc-ul1data2dai-driver"
#define MT_SOC_UL2DAI_NAME "mt-soc-ul2dai-driver"
#define MT_SOC_VOICE_NAME "mt-soc-voicedai-driver"


// platform name
#define MT_SOC_DL1_PCM   "mt-soc-dl1-pcm"
#define MT_SOC_DL1DATA2_PCM   "mt-soc-dl1_data2-pcm"
#define MT_SOC_DL2_PCM   "mt-soc-dl2-pcm"
#define MT_SOC_UL1_PCM   "mt-soc-ul1-pcm"
#define MT_SOC_UL2_PCM   "mt-soc-ul2-pcm"
#define MT_SOC_AWB_PCM   "mt-soc-awb-pcm"
#define MT_SOC_DAI_PCM   "mt-soc-DAI-pcm"
#define MT_SOC_MODDAI_PCM   "mt-soc-MODDAI-pcm"
#define MT_SOC_VOICE  "mt-soc-voice"
#define MT_SOC_IFMI2S2  "mt-soc-fm-i2s2"
#define MT_SOC_VOICE_EXT  "mt-soc-voice-ext"
#define MT_SOC_DUMMY_PCM  "mt-soc-dummy-pcm"
#define MT_SOC_ROUTING_PCM  "mt-soc-routing-pcm"

//codec dai name
#define MT_SOC_CODEC_TXDAI_NAME "mt-soc-codec-tx-dai"
#define MT_SOC_CODEC_RXDAI_NAME "mt-soc-codec-rx-dai"
#define MT_SOC_CODEC_PCMTXDAI_NAME "mt-soc-codec-pcmtx-dai"
#define MT_SOC_CODEC_PCMRXDAI_NAME "mt-soc-codec-pcmrx-dai"
#define MT_SOC_CODEC_FMI2S2TXDAI_NAME "mt-soc-codec-fmi2s2tx-dai"
#define MT_SOC_CODEC_FMI2S2RXDAI_NAME "mt-soc-codec-fmi2s2rx-dai"
#define MT_SOC_ROUTING_DAI_NAME "Routing-Control"

#define MT_SOC_CODEC_STUB_NAME "mt-soc-codec-stub"
#define MT_SOC_CODEC_NAME "mt-soc-codec"
#define MT_SOC_CODEC_DUMMY_NAME "mt-soc-dummy-codec"
#define MT_SOC_CODEC_DUMMY_DAI_NAME "mt-soc-dummy-dai-codec"

// stream name
#define MT_SOC_DL1_STREAM_NAME "MultiMedia1 PLayback"
#define MT_SOC_DL1DATA2_STREAM_NAME "MultiMedia1data2 PLayback"
#define MT_SOC_DL2_STREAM_NAME "MultiMedia2 PLayback"
#define MT_SOC_PCM1_STREAM_NAME "PCM1 PLayback"
#define MT_SOC_PCM2_STREAM_NAME "PCM2 PLayback"
#define MT_SOC_FM_I2S2_STREAM_NAME "FM_I2S2 PLayback"
#define MT_SOC_FM_I2S2_RECORD_STREAM_NAME "FM_I2S2 Record"
#define MT_SOC_UL1_STREAM_NAME "MultiMedia1 Capture"
#define MT_SOC_UL1DATA2_STREAM_NAME "MultiMedia1data2 Capture"
#define MT_SOC_AWB_STREAM_NAME "MultiMedia_awb_Capture"
#define MT_SOC_DAI_STREAM_NAME "MultiMedia_dai_Capture"
#define MT_SOC_MODDAI_STREAM_NAME "MultiMedia_Moddai_Capture"
#define MT_SOC_ROUTING_STREAM_NAME "MultiMedia Routing"

#endif


