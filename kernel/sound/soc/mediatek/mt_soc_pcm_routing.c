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
 *   mt6583.c
 *
 * Project:
 * --------
 *   MT6583  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio register
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

static int mtk_afe_routing_probe(struct platform_device *pdev);
static int mtk_routing_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_routing_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_routing_platform_probe(struct snd_soc_platform *platform);

#define MAX_PCM_DEVICES     4
#define MAX_PCM_SUBSTREAMS  128
#define MAX_MIDI_DEVICES

/* defaults */
#define MAX_BUFFER_SIZE     (16*1024)
#define MIN_PERIOD_SIZE     64
#define MAX_PERIOD_SIZE     MAX_BUFFER_SIZE
#define USE_FORMATS         (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE        SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        48000
#define USE_CHANNELS_MIN    1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     2048

static int mtkalsa_playback_constraints(struct snd_pcm_runtime *runtime)
{
    int err;
    err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (err < 0)
    {
        printk("mtkalsa_playback_constraints SNDRV_PCM_HW_PARAM_PERIODS err = %d", err);
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


static struct snd_pcm_hardware mtk_pcm_hardware =
{
    .info = (SNDRV_PCM_INFO_MMAP |
    SNDRV_PCM_INFO_INTERLEAVED |
    SNDRV_PCM_INFO_RESUME |
    SNDRV_PCM_INFO_MMAP_VALID),
    .formats =      USE_FORMATS,
    .rates =        USE_RATE,
    .rate_min =     USE_RATE_MIN,
    .rate_max =     USE_RATE_MAX,
    .channels_min =     USE_CHANNELS_MIN,
    .channels_max =     USE_CHANNELS_MAX,
    .buffer_bytes_max = MAX_BUFFER_SIZE,
    .period_bytes_max = MAX_PERIOD_SIZE,
    .periods_min =      1,
    .periods_max =      4096,
    .fifo_size =        0,
};

static struct snd_soc_pcm_runtime *pruntimepcm = NULL;

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] =
{
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 96000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(supported_sample_rates),
    .list = supported_sample_rates,
    .mask = 0,
};

static int mtk_routing_pcm_open(struct snd_pcm_substream *substream)
{
    printk("mtk_routing_pcm_open\n");

    struct snd_pcm_runtime *runtime = substream->runtime;

    int err = 0;
    int ret = 0;

    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &constraints_sample_rates);
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);


    if (ret < 0)
    {
        printk("snd_pcm_hw_constraint_integer failed\n");
    }

    //print for hw pcm information
    printk("mtk_routing_pcm_open runtime rate = %d channels = %d \n", runtime->rate, runtime->channels);
    if (substream->pcm->device & 1)
    {
        runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
        runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    }
    if (substream->pcm->device & 2)
        runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
                              SNDRV_PCM_INFO_MMAP_VALID);

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        printk("SNDRV_PCM_STREAM_PLAYBACK mtkalsa_playback_constraints\n");
    }
    else
    {

    }

    if (err < 0)
    {
        printk("mtk_routing_pcm_close\n");
        mtk_routing_pcm_close(substream);
        return err;
    }
    printk("mtk_routing_pcm_open return\n");
    return 0;
}

static int mtk_routing_pcm_close(struct snd_pcm_substream *substream)
{
    struct snd_dummy *dummy = snd_pcm_substream_chip(substream);
    return 0;
}

static int mtk_routing_pcm_start(struct snd_pcm_substream *substream)
{
    printk("mtk_routing_pcm_start \n");
    return 0;
}

static int mtk_routing_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    printk("%s cmd = %d\n", __func__,cmd);
    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            break;
    }
    return -EINVAL;
}

static int mtk_routing_pcm_copy(struct snd_pcm_substream *substream,
                        int channel, snd_pcm_uframes_t pos,
                        void __user *dst, snd_pcm_uframes_t count)
{

    printk("mtk_routing_pcm_copy pos = %d count = %d\n ", pos, count);
    count = count << 2;
    return 0;
}

static int mtk_routing_pcm_silence(struct snd_pcm_substream *substream,
                           int channel, snd_pcm_uframes_t pos,
                           snd_pcm_uframes_t count)
{
    printk("mtk_routing_pcm_silence \n");
    return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_routing_pcm_page(struct snd_pcm_substream *substream,
                                 unsigned long offset)
{
    printk("dummy_pcm_page \n");
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}

static int mtk_routing_pcm_prepare(struct snd_pcm_substream *substream)
{
    printk("mtk_alsa_prepare \n");
    return 0;
}

static int mtk_routing_pcm_stop(struct snd_pcm_substream *substream)
{
    printk("mtk_routing_pcm_stop \n");
    return 0;
}

static kal_int32 Previous_Hw_cur = 0;
static snd_pcm_uframes_t mtk_routing_pcm_pointer(struct snd_pcm_substream *substream)
{
    printk("mtk_routing_pcm_pointer \n");
    return (Previous_Hw_cur >> 2);
}

static int mtk_routing_pcm_hw_params(struct snd_pcm_substream *substream,
                             struct snd_pcm_hw_params *hw_params)
{
    PRINTK_AUDDRV("mtk_routing_pcm_hw_params \n");
    int ret = 0;
    return ret;
}

static int mtk_routing_pcm_hw_free(struct snd_pcm_substream *substream)
{
    PRINTK_AUDDRV("mtk_routing_pcm_hw_free \n");
    return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_ops mtk_afe_ops =
{
    .open =     mtk_routing_pcm_open,
    .close =    mtk_routing_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_routing_pcm_hw_params,
    .hw_free =  mtk_routing_pcm_hw_free,
    .prepare =  mtk_routing_pcm_prepare,
    .trigger =  mtk_routing_pcm_trigger,
    .copy =     mtk_routing_pcm_copy,
    .silence =  mtk_routing_pcm_silence,
    .page =     mtk_routing_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_routing_platform =
{
    .ops        = &mtk_afe_ops,
    .pcm_new    = mtk_asoc_routing_pcm_new,
    .probe      = mtk_afe_routing_platform_probe,
};

static int mtk_afe_routing_probe(struct platform_device *pdev)
{
    printk("mtk_afe_routing_probe\n");
    int ret = 0;
    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_ROUTING_PCM);
    }

    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_soc_routing_platform);
}

static int mtk_asoc_routing_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    pruntimepcm  = rtd;
    int ret = 0;

    printk("%s\n", __func__);
    return ret;
}



static int msm_routing_lsm_func_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	int i;
	u16 port_id;
	return 0;
}

static int msm_routing_lsm_func_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	int i;
	u16 port_id;
	return 0;
}

static const char * const lsm_func_text[] = {
	"None", "AUDIO", "BEACON", "ULTRASOUND"
};
static const struct soc_enum lsm_func_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lsm_func_text), lsm_func_text);
static const struct snd_kcontrol_new lsm_function[] = {
	SOC_ENUM_EXT("test1", lsm_func_enum,
		     msm_routing_lsm_func_get, msm_routing_lsm_func_put),
};

static int mtk_afe_routing_platform_probe(struct snd_soc_platform *platform)
{
    printk("mtk_afe_routing_platform_probe\n");
    /*
    snd_soc_add_platform_controls(platform, lsm_function,
				      ARRAY_SIZE(lsm_function));
				      */
    return 0;
}


static int mtk_afe_routing_remove(struct platform_device *pdev)
{
    pr_debug("%s\n", __func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static struct platform_driver mtk_afe_routing_driver =
{
    .driver = {
        .name = MT_SOC_ROUTING_PCM,
        .owner = THIS_MODULE,
    },
    .probe = mtk_afe_routing_probe,
    .remove = mtk_afe_routing_remove,
};

static struct platform_device *soc_mtkafe_routing_dev;

static int __init mtk_soc_routing_platform_init(void)
{
    printk("%s\n", __func__);
    int ret =0;

    soc_mtkafe_routing_dev = platform_device_alloc(MT_SOC_ROUTING_PCM , -1);
    if (!soc_mtkafe_routing_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtkafe_routing_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtkafe_routing_dev);
        return ret;
    }

    ret = platform_driver_register(&mtk_afe_routing_driver);

    return ret;

}
module_init(mtk_soc_routing_platform_init);

static void __exit mtk_soc_routing_platform_exit(void)
{

    printk("%s\n", __func__);
    platform_driver_unregister(&mtk_afe_routing_driver);
}
module_exit(mtk_soc_routing_platform_exit);

MODULE_DESCRIPTION("afe eouting driver");
MODULE_LICENSE("GPL");


