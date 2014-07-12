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
 *   mtk_dai_stub
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
 *   Audio dai stub file
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"


/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] =
{
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
    96000,192000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(supported_sample_rates),
    .list = supported_sample_rates,
    .mask = 0,
};


static int multimedia_startup(struct snd_pcm_substream *substream,
                              struct snd_soc_dai *dai)
{
    printk("dai multimedia_startup \n");
    snd_pcm_hw_constraint_list(substream->runtime, 0,
                               SNDRV_PCM_HW_PARAM_RATE,
                               &constraints_sample_rates);
    return 0;
}

static struct snd_soc_dai_ops mtk_dai_stub_ops =
{
    .startup    = multimedia_startup,
};

static struct snd_soc_dai_driver mtk_dai_stub_dai[] =
{
    {
        .playback = {
            .stream_name = MT_SOC_DL1_STREAM_NAME,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 48000,
        },
        .name = MT_SOC_DL1DAI_NAME,
        .ops = &mtk_dai_stub_ops,
    },
    {
        .capture = {
            .stream_name = MT_SOC_UL1_STREAM_NAME,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 48000,
        },
        .name = MT_SOC_UL1DAI_NAME,
        .ops = &mtk_dai_stub_ops,
    },
    {
        .playback = {
            .stream_name = MT_SOC_PCM2_STREAM_NAME,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 16000,
        },
        .name = MT_SOC_VOICE_NAME,
        .ops = &mtk_dai_stub_ops,
    },
    {
        .playback = {
            .stream_name = MT_SOC_FM_I2S2_STREAM_NAME,
            .rates = SNDRV_PCM_RATE_8000_48000 ,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 48000,
        },
        .name = "FM_I2S2_OUT",
        .ops = &mtk_dai_stub_ops,
    },
    {
        .capture = {
            .stream_name = MT_SOC_FM_I2S2_RECORD_STREAM_NAME,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 48000,
        },
        .name = "FM_I2S2_IN",
        .ops = &mtk_dai_stub_ops,
    },
    /*
    {
        .playback = {
            .stream_name = MT_SOC_ROUTING_STREAM_NAME,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 48000,
        },

        .capture = {
            .stream_name = MT_SOC_ROUTING_STREAM_NAME,
            .rates =  SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
            .channels_min = 1,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 48000,
        },
        .name = "PLATFORM_ROUTING",
        .ops = &mtk_dai_stub_ops,
    },
    */
};

static const struct snd_soc_component_driver mt_dai_component = {
	.name		= MT_SOC_DAI_NAME,
};

static int mtk_dai_stub_dev_probe(struct platform_device *pdev)
{
    int rc = 0;

    printk("mtk_dai_stub_dev_probe  name %s\n", dev_name(&pdev->dev));

    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_DAI_NAME);
    }
    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));

    rc = snd_soc_register_component(&pdev->dev, &mt_dai_component,
					  mtk_dai_stub_dai, ARRAY_SIZE(mtk_dai_stub_dai));

    printk("%s: rc  = %d\n", __func__,rc );
    return rc;
}

static int mtk_dai_stub_dev_remove(struct platform_device *pdev)
{
    printk("%s:\n", __func__);

    snd_soc_unregister_component(&pdev->dev);

    return 0;
}

static struct platform_driver mtk_dai_stub_driver =
{
    .probe  = mtk_dai_stub_dev_probe,
    .remove = mtk_dai_stub_dev_remove,
    .driver = {
        .name = MT_SOC_DAI_NAME,
        .owner = THIS_MODULE,
    },
};

static struct platform_device *soc_mtk_dai_dev;


static int __init mtk_dai_stub_init(void)
{
    printk("%s:\n", __func__);
    int ret;
    soc_mtk_dai_dev = platform_device_alloc(MT_SOC_DAI_NAME , -1);
    if (!soc_mtk_dai_dev)
    {
        return -ENOMEM;
    }
    ret = platform_device_add(soc_mtk_dai_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtk_dai_dev);
        return ret;
    }
    return platform_driver_register(&mtk_dai_stub_driver);
}

static void __exit mtk_dai_stub_exit(void)
{
    printk("%s:\n", __func__);

    platform_driver_unregister(&mtk_dai_stub_driver);
}

module_init(mtk_dai_stub_init);
module_exit(mtk_dai_stub_exit);

/* Module information */
MODULE_DESCRIPTION("MTK SOC DAI driver");
MODULE_LICENSE("GPL v2");


