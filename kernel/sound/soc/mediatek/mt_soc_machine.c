/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_machine.c
 *
 * Project:
 * --------
 *   Audio soc machine driver
 *
 * Description:
 * ------------
 *   Audio machine driver
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#define CONFIG_MTK_DEEP_IDLE
#ifdef CONFIG_MTK_DEEP_IDLE
#include <mach/mt_clkmgr.h>
#include <mach/mt_idle.h>
#endif

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/xlog.h>
#include <mach/irqs.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/mt_reg_base.h>
#include <asm/div64.h>
#include <linux/aee.h>
#include <mach/pmic_mt6323_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/platform_device.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <asm/mach-types.h>
#include <linux/debugfs.h>

static int mt_soc_spk_control;
static  struct dentry *mt_sco_audio_debugfs;
#define DEBUG_FS_NAME "mtksocaudio"


extern void Sound_Speaker_Turnon();
extern void Sound_Speaker_Turnoff();

static int mtmachine_startup(struct snd_pcm_substream *substream)
{
    printk("mtmachine_startup \n");
    return 0;
}

static int mtmachine_prepare(struct snd_pcm_substream *substream)
{
    printk("mtmachine_prepare \n");
    return 0;
}

static struct snd_soc_ops mt_machine_audio_ops =
{
    .startup    = mtmachine_startup,
    .prepare   = mtmachine_prepare,
};

static int mtmachine_startupmedia2(struct snd_pcm_substream *substream)
{
    printk("mtmachine_startupmedia2 \n");
    return 0;
}

static int mtmachine_preparemedia2(struct snd_pcm_substream *substream)
{
    printk("mtmachine_preparemedia2 \n");
    return 0;
}

static struct snd_soc_ops mtmachine_audio_ops2 =
{
    .startup    = mtmachine_startupmedia2,
    .prepare   = mtmachine_preparemedia2,
};

static int mt_soc_audio_init(struct snd_soc_pcm_runtime *rtd)
{
    printk("mt_soc_audio_init\n");
    return 0;
}

static int mt_soc_audio_init2(struct snd_soc_pcm_runtime *rtd)
{
    printk("mt_soc_audio_init2\n");
    return 0;
}

static int mt_soc_debug_open(struct inode *inode, struct file *file)
{
    printk("mt_soc_debug_open \n");
    return 0;
}

static ssize_t mt_soc_debug_read(struct file *file, char __user *buf,
                                 size_t count, loff_t *pos)
{
    const int size = 4096;
    printk("mt_soc_debug_read \n");
    char buffer[size];
    AudDrv_Clk_On();
    int n = 0;
    n =    scnprintf(buffer  , size-n, "mt_soc_debug_read\n");
    n += scnprintf(buffer+n,size-n,"AUDIOAFE_TOP_CON0  = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON0));
    n += scnprintf(buffer+n,size-n,"AUDIO_TOP_CON1  = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON1));
    n += scnprintf(buffer+n,size-n,"AUDIO_TOP_CON2  = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON2));
    n += scnprintf(buffer+n,size-n,"AUDIO_TOP_CON3  = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON3));
    n += scnprintf(buffer+n,size-n,"AFE_DAC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_DAC_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_DAC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_DAC_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_I2S_CON  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON));
    n += scnprintf(buffer+n,size-n,"AFE_DAIBT_CON0  = 0x%x\n",Afe_Get_Reg(AFE_DAIBT_CON0));

    n += scnprintf(buffer+n,size-n,"AFE_CONN0  = 0x%x\n", Afe_Get_Reg(AFE_CONN0));
    n += scnprintf(buffer+n,size-n,"AFE_CONN1  = 0x%x\n", Afe_Get_Reg(AFE_CONN1));
    n += scnprintf(buffer+n,size-n,"AFE_CONN2  = 0x%x\n", Afe_Get_Reg(AFE_CONN2));
    n += scnprintf(buffer+n,size-n,"AFE_CONN3  = 0x%x\n", Afe_Get_Reg(AFE_CONN3));
    n += scnprintf(buffer+n,size-n,"AFE_CONN4  = 0x%x\n", Afe_Get_Reg(AFE_CONN4));
    n += scnprintf(buffer+n,size-n,"AFE_I2S_CON1  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_I2S_CON2  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON2));
    n += scnprintf(buffer+n,size-n,"AFE_MRGIF_CON  = 0x%x\n",Afe_Get_Reg(AFE_MRGIF_CON));

    n += scnprintf(buffer+n,size-n,"AFE_DL1_BASE  = 0x%x\n", Afe_Get_Reg(AFE_DL1_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_DL1_CUR  = 0x%x\n", Afe_Get_Reg(AFE_DL1_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_DL1_END  = 0x%x\n", Afe_Get_Reg(AFE_DL1_END));
    n += scnprintf(buffer+n,size-n,"AFE_I2S_CON3  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON3));

    n += scnprintf(buffer+n,size-n,"AFE_DL2_BASE  = 0x%x\n", Afe_Get_Reg(AFE_DL2_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_DL2_CUR  = 0x%x\n", Afe_Get_Reg(AFE_DL2_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_DL2_END  = 0x%x\n", Afe_Get_Reg(AFE_DL2_END));
    n += scnprintf(buffer+n,size-n,"AFE_CONN5  = 0x%x\n", Afe_Get_Reg(AFE_CONN5));
    n += scnprintf(buffer+n,size-n,"AFE_CONN_24BIT  = 0x%x\n", Afe_Get_Reg(AFE_CONN_24BIT));
    n += scnprintf(buffer+n,size-n,"AFE_AWB_BASE  = 0x%x\n", Afe_Get_Reg(AFE_AWB_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_AWB_END  = 0x%x\n", Afe_Get_Reg(AFE_AWB_END));
    n += scnprintf(buffer+n,size-n,"AFE_AWB_CUR  = 0x%x\n", Afe_Get_Reg(AFE_AWB_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_VUL_BASE  = 0x%x\n", Afe_Get_Reg(AFE_VUL_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_VUL_END  = 0x%x\n", Afe_Get_Reg(AFE_VUL_END));
    n += scnprintf(buffer+n,size-n,"AFE_VUL_CUR  = 0x%x\n", Afe_Get_Reg(AFE_VUL_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_CONN6  = 0x%x\n", Afe_Get_Reg(AFE_CONN6));
    n += scnprintf(buffer+n,size-n,"AFE_DAI_BASE  = 0x%x\n",Afe_Get_Reg(AFE_DAI_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_DAI_END  = 0x%x\n",Afe_Get_Reg(AFE_DAI_END));
    n += scnprintf(buffer+n,size-n,"AFE_DAI_CUR  = 0x%x\n",Afe_Get_Reg(AFE_DAI_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_MSB  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MSB));

    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_MON0  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON0));
    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_MON1  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON1));
    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_MON2  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON2));
    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_MON4  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON4));

    n += scnprintf(buffer+n,size-n,"AFE_ADDA_DL_SRC2_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_DL_SRC2_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_UL_SRC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_SRC_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_UL_SRC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_SRC_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_TOP_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_TOP_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_UL_DL_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_DL_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_SRC_DEBUG  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_SRC_DEBUG_MON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_SRC_DEBUG_MON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON1));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_NEWIF_CFG0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_NEWIF_CFG0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_NEWIF_CFG1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_NEWIF_CFG1));

    n += scnprintf(buffer+n,size-n,"AFE_SIDETONE_DEBUG  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_DEBUG));
    n += scnprintf(buffer+n,size-n,"AFE_SIDETONE_MON  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_MON));
    n += scnprintf(buffer+n,size-n,"AFE_SIDETONE_CON0  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_SIDETONE_COEFF  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_COEFF));
    n += scnprintf(buffer+n,size-n,"AFE_SIDETONE_CON1  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_SIDETONE_GAIN  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_GAIN));
    n += scnprintf(buffer+n,size-n,"AFE_SGEN_CON0  = 0x%x\n", Afe_Get_Reg(AFE_SGEN_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_SGEN_CON0  = 0x%x\n", Afe_Get_Reg(AFE_SGEN_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_TOP_CON0  = 0x%x\n", Afe_Get_Reg(AFE_TOP_CON0));

    n += scnprintf(buffer+n,size-n,"AFE_ADDA_PREDIS_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_PREDIS_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_ADDA_PREDIS_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_PREDIS_CON1));

    n += scnprintf(buffer+n,size-n,"AFE_MRG_MON0  = 0x%x\n",Afe_Get_Reg(AFE_MRGIF_MON0));
    n += scnprintf(buffer+n,size-n,"AFE_MRG_MON1  = 0x%x\n",Afe_Get_Reg(AFE_MRGIF_MON1));
    n += scnprintf(buffer+n,size-n,"AFE_MRG_MON2  = 0x%x\n",Afe_Get_Reg(AFE_MRGIF_MON2));

    n += scnprintf(buffer+n,size-n,"AFE_MOD_DAI_BASE  = 0x%x\n", Afe_Get_Reg(AFE_MOD_DAI_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_MOD_DAI_END  = 0x%x\n", Afe_Get_Reg(AFE_MOD_DAI_END));
    n += scnprintf(buffer+n,size-n,"AFE_MOD_DAI_CUR  = 0x%x\n", Afe_Get_Reg(AFE_MOD_DAI_CUR));

    n += scnprintf(buffer+n,size-n,"AFE_HDMI_OUT_CON0  = 0x%x\n", Afe_Get_Reg(AFE_HDMI_OUT_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_HDMI_BASE  = 0x%x\n", Afe_Get_Reg(AFE_HDMI_BASE));
    n += scnprintf(buffer+n,size-n,"AFE_HDMI_CUR  = 0x%x\n", Afe_Get_Reg(AFE_HDMI_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_HDMI_END  = 0x%x\n", Afe_Get_Reg(AFE_HDMI_END));
    n += scnprintf(buffer+n,size-n,"AFE_HDMI_CONN0  = 0x%x\n", Afe_Get_Reg(AFE_HDMI_CONN0));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_CON  = 0x%x\n",Afe_Get_Reg(AFE_IRQ_MCU_CON));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_MCU_CON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CON));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_STATUS  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_STATUS));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_CLR  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CLR));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_MCU_CNT1  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CNT1));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_MCU_CNT2  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CNT2));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_MCU_EN  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_EN));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ_MON2  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_MON2));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ1_CNT_MON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ1_MCU_CNT_MON));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ2_CNT_MON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ2_MCU_CNT_MON));
    n += scnprintf(buffer+n,size-n,"AFE_IRQ1_EN_CNT_MON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ1_MCU_EN_CNT_MON));
    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_MAXLEN  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MAXLEN));
    n += scnprintf(buffer+n,size-n,"AFE_MEMIF_PBUF_SIZE  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_PBUF_SIZE));

    n += scnprintf(buffer+n,size-n,"AFE_GAIN1_CON0  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN1_CON1  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN1_CON2  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON2));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN1_CON3  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON3));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN1_CONN  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CONN));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN1_CUR  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CON0  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CON1  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CON2  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON2));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CON3  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON3));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CONN  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CUR  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CUR));
    n += scnprintf(buffer+n,size-n,"AFE_GAIN2_CONN2  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN2));

    n += scnprintf(buffer+n,size-n,"FPGA_CFG2  = 0x%x\n", Afe_Get_Reg(FPGA_CFG2));
    n += scnprintf(buffer+n,size-n,"FPGA_CFG3  = 0x%x\n", Afe_Get_Reg(FPGA_CFG3));
    n += scnprintf(buffer+n,size-n,"FPGA_CFG0  = 0x%x\n", Afe_Get_Reg(FPGA_CFG0));
    n += scnprintf(buffer+n,size-n,"FPGA_CFG1  = 0x%x\n", Afe_Get_Reg(FPGA_CFG1));
    n += scnprintf(buffer+n,size-n,"FPGA_STC  = 0x%x\n", Afe_Get_Reg(FPGA_STC));

    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON0));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON1));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON2  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON2));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON3  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON3));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON4  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON4));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON5  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON5));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON6  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON6));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON7  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON7));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON8  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON8));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON9  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON9));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON10  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON10));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON11  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON11));
    n += scnprintf(buffer+n,size-n,"PCM_INTF_CON1  = 0x%x\n", Afe_Get_Reg(PCM_INTF_CON));
    n += scnprintf(buffer+n,size-n,"PCM_INTF_CON2  = 0x%x\n", Afe_Get_Reg(PCM_INTF_CON2));
    n += scnprintf(buffer+n,size-n,"PCM2_INTF_CON  = 0x%x\n", Afe_Get_Reg(PCM2_INTF_CON));

    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON13  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON13));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON14  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON14));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON15  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON15));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON16  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON16));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON17  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON17));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON18  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON18));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON19  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON19));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON20  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON20));
    n += scnprintf(buffer+n,size-n,"AFE_ASRC_CON21  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON21));

    printk("mt_soc_debug_read len = %d\n", n);

    AudDrv_Clk_Off();

    return  simple_read_from_buffer(buf, count, pos, buffer, n);
}


static const struct file_operations mtaudio_debug_ops =
{
    .open = mt_soc_debug_open,
    .read = mt_soc_debug_read,
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt_soc_dai_common[] =
{
    /* FrontEnd DAI Links */
    {
        .name = "MultiMedia1",
        .stream_name = MT_SOC_DL1_STREAM_NAME,
        .cpu_dai_name   = MT_SOC_DL1DAI_NAME,
        .platform_name  = MT_SOC_DL1_PCM,
        .codec_dai_name = MT_SOC_CODEC_TXDAI_NAME,
        .codec_name = MT_SOC_CODEC_NAME,
        .init = mt_soc_audio_init,
        .ops = &mt_machine_audio_ops,
    },
    {
        .name = "MultiMedia2",
        .stream_name = MT_SOC_UL1_STREAM_NAME,
        .cpu_dai_name   = MT_SOC_UL1DAI_NAME,
        .platform_name  = MT_SOC_UL1_PCM,
        .codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
        .codec_name = MT_SOC_CODEC_DUMMY_NAME,
        .init = mt_soc_audio_init,
        .ops = &mt_machine_audio_ops,
    },
    {
        .name = "PCM2_VOICE",
        .stream_name = MT_SOC_PCM2_STREAM_NAME,
        .cpu_dai_name   = MT_SOC_VOICE_NAME,
        .platform_name  = MT_SOC_VOICE,
        .codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
        .codec_name = MT_SOC_CODEC_DUMMY_NAME,
        .init = mt_soc_audio_init,
        .ops = &mt_machine_audio_ops,
    },
    /*  todo: enable fm function when ready
    {
        .name = "FM_I2S2_OUT",
        .stream_name = MT_SOC_FM_I2S2_STREAM_NAME,
        .cpu_dai_name   = MT_SOC_DAI_NAME,
        .platform_name  = MT_SOC_IFMI2S2,
        .codec_dai_name = MT_SOC_CODEC_FMI2S2RXDAI_NAME,
        .codec_name = MT_SOC_CODEC_NAME,
        .init = mt_soc_audio_init,
        .ops = &mt_machine_audio_ops,
    },
    {
        .name = "FM_I2S2_IN",
        .stream_name = MT_SOC_FM_I2S2_RECORD_STREAM_NAME,
        .cpu_dai_name   = MT_SOC_DAI_NAME,
        .platform_name  = MT_SOC_AWB_PCM,
        .codec_dai_name = MT_SOC_CODEC_FMI2S2RXDAI_NAME,
        .codec_name = MT_SOC_CODEC_NAME,
        .init = mt_soc_audio_init,
        .ops = &mt_machine_audio_ops,
    },
    */
    /*
    {
        .name = "PLATOFRM_CONTROL",
        .stream_name = MT_SOC_ROUTING_STREAM_NAME,
        .cpu_dai_name   = MT_SOC_ROUTING_DAI_NAME,
        .platform_name  = MT_SOC_ROUTING_PCM,
        .codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
        .codec_name = MT_SOC_CODEC_DUMMY_NAME,
        .init = mt_soc_audio_init2,
        .ops = &mtmachine_audio_ops2,
    },
    */
};

static int mt6595_get_spk(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("%s: mt6595_get_spk = %d\n", __func__, mt_soc_spk_control);
    ucontrol->value.integer.value[0] = mt_soc_spk_control;
    return 0;
}

static int mt6595_set_spk(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    if (mt_soc_spk_control == ucontrol->value.integer.value[0])
    {
        Sound_Speaker_Turnon();
    }
    else
    {
        Sound_Speaker_Turnoff();
    }
    mt_soc_spk_control = ucontrol->value.integer.value[0];
    return 1;
}

static const char *spk_function[] = {"Off", "On"};

static const struct soc_enum mt_soc_machine_enum[] =
{
    SOC_ENUM_SINGLE_EXT(2, spk_function)
};

static const struct snd_kcontrol_new mt_soc_controls[] =
{
    //SOC_ENUM_EXT("Speaker Function", mt_soc_machine_enum[0], mt6595_get_spk, mt6595_set_spk),
};

static struct snd_soc_card snd_soc_card_mt =
{
    .name       = "mt-snd-card",
    .dai_link   = mt_soc_dai_common,
    .num_links  = ARRAY_SIZE(mt_soc_dai_common),
    .controls = mt_soc_controls,
    .num_controls = ARRAY_SIZE(mt_soc_controls),
};

static struct platform_device *mt_snd_device;

static int __init mt_soc_snd_init(struct platform_device *pdev)
{
    int ret;
    struct snd_soc_card *card = &snd_soc_card_mt;
    printk("mt_soc_snd_init card addr = %p \n", card);

    mt_snd_device = platform_device_alloc("soc-audio", -1);
    if (!mt_snd_device)
    {
        printk("mt6589_probe  platform_device_alloc fail\n");
        return -ENOMEM;
    }
    platform_set_drvdata(mt_snd_device, &snd_soc_card_mt);
    ret = platform_device_add(mt_snd_device);

    if (ret != 0)
    {
        printk("mt_soc_snd_init goto put_device fail\n");
        goto put_device;
    }

    printk("mt_soc_snd_init dai_link = %p \n", snd_soc_card_mt.dai_link);

    // create debug file
    mt_sco_audio_debugfs = debugfs_create_file(DEBUG_FS_NAME,
                                               S_IFREG | S_IRUGO, NULL, (void *) DEBUG_FS_NAME, &mtaudio_debug_ops);

    return 0;
put_device:
    platform_device_put(mt_snd_device);
    return ret;

}

static int __exit mt_soc_snd_exit(struct platform_device *pdev)
{
    platform_device_unregister(mt_snd_device);
    return 0;
}

module_init(mt_soc_snd_init);
module_exit(mt_soc_snd_exit);

/* Module information */
MODULE_AUTHOR("ChiPeng <chipeng.chang@mediatek.com>");
MODULE_DESCRIPTION("ALSA SoC driver ");

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mt-snd-card");


