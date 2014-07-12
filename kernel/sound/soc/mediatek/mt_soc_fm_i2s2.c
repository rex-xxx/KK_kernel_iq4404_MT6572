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
 *   mt_soc_fm_i2s2.c
 *
 * Project:
 * --------
 *   voice call platform driver
 *
 * Description:
 * ------------
 *
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

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_digital_type.h"

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
#include <mach/pmic_mt6320_sw.h>
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


/*
 *    function implementation
 */

static int mtk_fm_i2s2_probe(struct platform_device *pdev);
static int mtk_fm_i2s2_close(struct snd_pcm_substream *substream);
static int mtk_soc_fm_i2s2_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_fm_i2s2_platform_probe(struct snd_soc_platform *platform);

static const uint16_t kFmChipSamplingRateHz = 32000;
static const uint16_t kFmUplinkSamplingRateHz = 44100; // For MT6627, I2S FM only support 32000, but use ASRC to 44100, so that MEMIF will get I2Sdata with 44100Hz
static const uint16_t kFmDownlinkSamplingRateHz = 44100;

#define MAX_PCM_DEVICES     4
#define MAX_PCM_SUBSTREAMS  128
#define MAX_MIDI_DEVICES 0

/* defaults */
#define MAX_BUFFER_SIZE     (16*1024)
#define MIN_PERIOD_SIZE     64
#define MAX_PERIOD_SIZE     MAX_BUFFER_SIZE
#define USE_FORMATS         (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define FMI2S2_USE_RATE         SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000
#define FMI2S2_USE_RATE_MIN        8000
#define FMI2S2_USE_RATE_MAX        48000
#define USE_CHANNELS_MIN    1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     8192

static uint32 Fm_Hw_gain = 0x80000;

static uint32_t GetFmUplinkSamplingRate()
{
    return kFmUplinkSamplingRateHz;
}

static uint32_t GetFmDownlinkSamplingRate()
{
    return kFmDownlinkSamplingRateHz;
}

static int mtkalsa_fmi2s2_constraints(struct snd_pcm_runtime *runtime)
{
    int err;
    err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (err < 0)
    {
        printk("mtkalsa_fmi2s2_constraints SNDRV_PCM_HW_PARAM_PERIODS err = %d", err);
        return err;
    }
    err = snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 1, UINT_MAX);
    if (err < 0)
    {
        printk("snd_pcm_hw_constraint_minmax SNDRV_PCM_HW_PARAM_BUFFER_BYTES err = %d", err);
        return err;
    }
    return 0;
}

static struct snd_pcm_hardware mtk_voice_hardware =
{
    .info = (SNDRV_PCM_INFO_MMAP |
    SNDRV_PCM_INFO_INTERLEAVED |
    SNDRV_PCM_INFO_RESUME |
    SNDRV_PCM_INFO_MMAP_VALID),
    .formats =      USE_FORMATS,
    .rates =        FMI2S2_USE_RATE,
    .rate_min =     FMI2S2_USE_RATE_MIN,
    .rate_max =     FMI2S2_USE_RATE_MAX,
    .channels_min =     USE_CHANNELS_MIN,
    .channels_max =     USE_CHANNELS_MAX,
    .buffer_bytes_max = MAX_BUFFER_SIZE,
    .period_bytes_max = MAX_PERIOD_SIZE,
    .periods_min =      1,
    .periods_max =      4096,
    .fifo_size =        0,
};

/* Conventional and unconventional sample rate supported */
static unsigned int fmi2s2_supported_sample_rates[] =
{
    32000, 44100, 48000,
};

static struct snd_pcm_hw_constraint_list fmi2s2_constraints_sample_rates =
{
    .count = ARRAY_SIZE(fmi2s2_supported_sample_rates),
    .list = fmi2s2_supported_sample_rates,
    .mask = 0,
};

static struct snd_pcm_hardware mtk_fm_i2s2_hardware =
{
    .info = (SNDRV_PCM_INFO_MMAP |
    SNDRV_PCM_INFO_INTERLEAVED |
    SNDRV_PCM_INFO_RESUME |
    SNDRV_PCM_INFO_MMAP_VALID),
    .formats =      USE_FORMATS,
    .rates =        FMI2S2_USE_RATE,
    .rate_min =     FMI2S2_USE_RATE_MIN,
    .rate_max =     FMI2S2_USE_RATE_MAX,
    .channels_min =     USE_CHANNELS_MIN,
    .channels_max =     USE_CHANNELS_MAX,
    .buffer_bytes_max = MAX_BUFFER_SIZE,
    .period_bytes_max = MAX_PERIOD_SIZE,
    .periods_min =      1,
    .periods_max =      4096,
    .fifo_size =        0,
};

static int mtk_fmi2s2_open(struct snd_pcm_substream *substream)
{
    printk("mtk_fmi2s2_open\n");

    struct snd_pcm_runtime *runtime = substream->runtime;
    int err = 0;
    int ret = 0;
    runtime->hw = mtk_fm_i2s2_hardware;
    memcpy((void *)(&(runtime->hw)), (void *)&mtk_fm_i2s2_hardware , sizeof(struct snd_pcm_hardware));

    printk("runtime->hw->rates= 0x%x mtk_fm_i2s2_hardware = = 0x%x\n ", runtime->hw.rates, &mtk_fm_i2s2_hardware);

    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &fmi2s2_constraints_sample_rates);
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

    if (ret < 0)
    {
        printk("snd_pcm_hw_constraint_integer failed\n");
    }

    // here open audio clocks
    AudDrv_Clk_On();
    AudDrv_ANA_Clk_On();

    //print for hw pcm information
    printk("mtk_fmi2s2_open runtime rate = %d channels = %d substream->pcm->device = %d\n",
           runtime->rate, runtime->channels, substream->pcm->device);
    runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        printk("SNDRV_PCM_STREAM_PLAYBACK mtkalsa_fmi2s2_constraints\n");
    }
    else
    {
        return -1;
    }

    if (err < 0)
    {
        printk("mtk_fm_i2s2_close\n");
        mtk_fm_i2s2_close(substream);
        return err;
    }
    printk("mtk_fmi2s2_open return\n");
    return 0;
}

static int mtk_fm_i2s2_close(struct snd_pcm_substream *substream)
{
    printk("mtk_fm_i2s2_close\n");
    // disable  InterConnection
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O15);
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O16);
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I12, Soc_Aud_InterConnectionOutput_O03);
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I13, Soc_Aud_InterConnectionOutput_O04);

    // Disable HW_GAIN2
    SetHwDigitalGainEnable(Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2, false);

    // Set DAC I2S Out
    if (GetI2SDacEnable() == false)
    {
        SetI2SDacEnable(false);
    }

    // Disable 2nd I2S IN (Disable ASRC first and than 2nd I2S IN later)
    SetI2SASRCEnable(false);
    SetI2SASRCConfig(false, GetFmUplinkSamplingRate()); // Setting to bypass ASRC
    Set2ndI2SInEnable(false);

    EnableAfe(false);

    return 0;
}

static int mtk_fmi2s2_start(struct snd_pcm_substream *substream)
{
    printk("mtk_fmi2s2_start \n");
    return 0;
}

static int mtk_fm_i2s2_trigger(struct snd_pcm_substream *substream, int cmd)
{
    printk("mtk_fm_i2s2_trigger cmd = %d\n", cmd);
    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            break;
    }
    return 0;
}

static int mtk_pcm_copy(struct snd_pcm_substream *substream,
                        int channel, snd_pcm_uframes_t pos,
                        void __user *dst, snd_pcm_uframes_t count)
{

    printk("mtk_pcm_copy pos = %d count = %d\n ", pos, count);
    count = count << 2;
    return 0;
}

static int mtk_pcm_silence(struct snd_pcm_substream *substream,
                           int channel, snd_pcm_uframes_t pos,
                           snd_pcm_uframes_t count)
{
    printk("mtk_pcm_silence \n");
    return 0; /* do nothing */
}

static void *dummy_page[2];
static struct page *mtk_pcm_page(struct snd_pcm_substream *substream,
                                 unsigned long offset)
{
    printk("dummy_pcm_page \n");
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}

static int mtk_fmi2s2_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtimeStream = substream->runtime;
    printk("mtk_fmi2s2_prepare rate = %d  channels = %d period_size = %d\n",
           runtimeStream->rate, runtimeStream->channels, runtimeStream->period_size);
    // Enable 2nd I2S IN
    const bool bIsSlaveMode = true;
    Set2ndI2SInConfig(runtimeStream->rate, bIsSlaveMode); // FM Rx I2S in from connsys is 32K / Master
    SetI2SASRCConfig(true, GetFmUplinkSamplingRate());  // Covert from 32000 Hz to 44100 Hz
    SetI2SASRCEnable(true);
    Set2ndI2SInEnable(true);

    // Set InterConnection
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O15);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O16);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I12, Soc_Aud_InterConnectionOutput_O03);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I13, Soc_Aud_InterConnectionOutput_O04);

    //EnableSideGenHw(Soc_Aud_InterConnectionInput_I00,Soc_Aud_MemIF_Direction_DIRECTION_INPUT,true);

    SetHwDigitalGainMode(Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2, Soc_Aud_I2S_SAMPLERATE_I2S_44K, 0xC8);
    SetHwDigitalGainEnable(Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2, true);

    // Set DAC I2S Out
    if (GetI2SDacEnable() == false)
    {
        SetI2SDacOut(GetFmDownlinkSamplingRate());
        SetI2SDacEnable(true);
    }
    SetHwDigitalGain(Fm_Hw_gain, Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2);
    EnableAfe(true);

    return 0;
}

static int mtk_fmi2s2_stop(struct snd_pcm_substream *substream)
{
    printk("mtk_fmi2s2_stop \n");
    return 0;
}

static snd_pcm_uframes_t mtk_pcm_pointer(struct snd_pcm_substream *substream)
{
    printk("mtk_pcm_pointer \n");
    return 0;
}

static int mtk_fm2is2_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *hw_params)
{
    printk("mtk_fm2is2_hw_params \n");
    int ret = 0;
    return ret;
}

static int mtk_fmi2s2_hw_free(struct snd_pcm_substream *substream)
{
    printk("mtk_fmi2s2_hw_free \n");
    return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_ops mtk_fmi2s2_ops =
{
    .open =     mtk_fmi2s2_open,
    .close =    mtk_fm_i2s2_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_fm2is2_hw_params,
    .hw_free =  mtk_fmi2s2_hw_free,
    .prepare =  mtk_fmi2s2_prepare,
    .trigger =  mtk_fm_i2s2_trigger,
    .copy =     mtk_pcm_copy,
    .silence =  mtk_pcm_silence,
    .page =     mtk_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_fmi2s2_platform =
{
    .ops        = &mtk_fmi2s2_ops,
    .pcm_new    = mtk_soc_fm_i2s2_new,
    .probe      = mtk_fm_i2s2_platform_probe,
};

static int mtk_fmi2s2_hwgain_get(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = Fm_Hw_gain;
    printk("%s: hw_gain=%d\n", __func__, Fm_Hw_gain);
    return 0;
}

static int mtk_fmi2s2_hwgain_set(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
    Fm_Hw_gain = ucontrol->value.integer.value[0];
    SetHwDigitalGain(Fm_Hw_gain, Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2);
    printk("%s: hw_gain=%d\n", __func__, Fm_Hw_gain);
    return 0;
}


static struct snd_kcontrol_new mtk_fmi2s2_controls[] =
{
    SOC_SINGLE_EXT("FM_I2S2_GAIN", SND_SOC_NOPM, 0, 0x80000, 0,
    mtk_fmi2s2_hwgain_get,
    mtk_fmi2s2_hwgain_set),
};


static int mtk_fm_i2s2_probe(struct platform_device *pdev)
{
    printk("mtk_fm_i2s2_probe\n");
    int ret = 0;
    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_IFMI2S2);
    }

    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_soc_fmi2s2_platform);
}

static int mtk_soc_fm_i2s2_new(struct snd_soc_pcm_runtime *rtd)
{
    int ret = 0;
    printk("%s\n", __func__);
    return ret;
}

static int mtk_fm_i2s2_platform_probe(struct snd_soc_platform *platform)
{
    printk("mtk_fm_i2s2_platform_probe\n");

    snd_soc_add_platform_controls(platform, mtk_fmi2s2_controls,
				      ARRAY_SIZE(mtk_fmi2s2_controls));
    return 0;
}

static int mtk_fmi2s2_remove(struct platform_device *pdev)
{
    pr_debug("%s\n", __func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static struct platform_driver mtk_fmi2s2_driver =
{
    .driver = {
        .name = MT_SOC_IFMI2S2,
        .owner = THIS_MODULE,
    },
    .probe = mtk_fm_i2s2_probe,
    .remove = mtk_fmi2s2_remove,
};

static struct platform_device *soc_mtk_fmi2s2_dev;

static int __init mtk_soc_fmi2s2_platform_init(void)
{
    printk("%s\n", __func__);
    int ret = 0;

    soc_mtk_fmi2s2_dev = platform_device_alloc(MT_SOC_IFMI2S2 , -1);
    if (!soc_mtk_fmi2s2_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtk_fmi2s2_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtk_fmi2s2_dev);
        return ret;
    }

    ret = platform_driver_register(&mtk_fmi2s2_driver);

    return ret;

}
module_init(mtk_soc_fmi2s2_platform_init);

static void __exit mtk_soc_fmi2s2_platform_exit(void)
{

    printk("%s\n", __func__);
    platform_driver_unregister(&mtk_fmi2s2_driver);
}
module_exit(mtk_soc_fmi2s2_platform_exit);

MODULE_DESCRIPTION("AFE FMI2S2  module platform driver");
MODULE_LICENSE("GPL");


