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
 *   mtk_codec_stub
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
 *   Audio codec stub file
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

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "mt_soc_analog_type.h"

static mt6331_Codec_Data_Priv *mCodec_data = NULL;
static uint32 mBlockSampleRate[AUDIO_ANALOG_DEVICE_INOUT_MAX] = {48000, 48000};
static DEFINE_MUTEX(Ana_Ctrl_Mutex);
static DEFINE_SPINLOCK(AudAna_lock);

static int ClsqCount = 0;
static void ClsqEnable(bool enable)
{
    spin_lock(& AudAna_lock);

    if (enable)
    {
        if (ClsqCount == 0)
        {
            Ana_Set_Reg(TOP_CLKSQ_SET, 0x0003, 0xffff); //CKSQ Enable
        }
        ClsqCount++;
    }
    else
    {
        ClsqCount--;
        if (ClsqCount < 0)
        {
            printk("ClsqEnable count <0 \n");
        }
        if (ClsqCount == 0)
        {
            Ana_Set_Reg(TOP_CLKSQ_SET, 0x0000, 0xffff);
        }
    }
    spin_unlock(& AudAna_lock);
}

static int TopCkCount = 0;
static void Topck_Enable(bool enable)
{
    if (enable == true)
    {
        if (TopCkCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x3000, 0xffff);  //AUD clock power down released
        }
        TopCkCount++;
    }
    else
    {
        TopCkCount--;
        if (TopCkCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x0000, 0xffff);
        }
    }
}

static int NvRegCount = 0;
static void NvregEnable(bool enable)
{
    if (enable == true)
    {
        if (NvRegCount == 0)
        {
            Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0000, 0xffff);  //AUD clock power down released
        }
        NvRegCount++;
    }
    else
    {
        NvRegCount--;
        if (NvRegCount == 0)
        {
            Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0001, 0xffff);
        }
    }
}


static int AdcClockCount = 0;
static void AdcClockEnable(bool enable)
{
    if (enable == true)
    {
        if (AdcClockCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x3000, 0xffff);  //AUD clock power down released
        }
        AdcClockCount++;
    }
    else
    {
        AdcClockCount--;
        if (AdcClockCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x0000, 0xffff);
        }
    }
}


static void SetDcCompenSation()
{

}

static void OpenClassH()
{
    Ana_Set_Reg(AFE_CLASSH_CFG7, 0x8909, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG8, 0x0909, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG9, 0x1309, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG10, 0x0909, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG11, 0x0915, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG12, 0x1414, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG13, 0x1414, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG14, 0x009c, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG26, 0x9313, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG27, 0x1313, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG28, 0x1313, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG29, 0x1515, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG30, 0x1515, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG1, 0x0024, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG2, 0x2f90, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG3, 0x1104, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG4, 0x2412, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG5, 0x0201, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG6, 0x2800, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG21, 0xa108, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG22, 0x06db, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG23, 0x0bd6, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG24, 0x1492, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG25, 0x1740, 0xffff); // Classh CK fix 591KHz
    Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd518, 0xffff); // Classh CK fix 591KHz
    msleep(1);
    Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd419, 0xffff); // Classh CK fix 591KHz
    msleep(1);
    Ana_Set_Reg(AFE_CLASSH_CFG1,   0x0021, 0xffff); // Classh CK fix 591KHz
    msleep(1);
}


static uint32 GetULNewIFFrequency(uint32 frequency)
{
    uint32 Reg_value = 0;
    switch (frequency)
    {
        case 8000:
        case 16000:
        case 32000:
            Reg_value = 1;
            break;
        case 48000:
            Reg_value = 3;
        default:
            break;
    }
    return Reg_value;
}

uint32 GetULFrequency(uint32 frequency)
{
    uint32 Reg_value = 0;
    switch (frequency)
    {
        case 8000:
        case 16000:
        case 32000:
            Reg_value = 0x0;
            break;
        case 48000:
            Reg_value = 0x1;
        default:
            break;

    }
    return Reg_value;
}

static int mt6323_codec_startup(struct snd_pcm_substream *substream , struct snd_soc_dai *Daiport)
{
    printk("mt6323_codec_startup \n");
    return 0;
}

static int mt6323_codec_prepare(struct snd_pcm_substream *substream , struct snd_soc_dai *Daiport)
{
    printk("mt6323_codec_prepare \n ");
    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        printk("mt6323_codec_prepare set up SNDRV_PCM_STREAM_CAPTURE rate = %d\n", substream->runtime->rate);
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC] = substream->runtime->rate;

    }
    else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        printk("mt6323_codec_prepare set up SNDRV_PCM_STREAM_PLAYBACK rate = %d\n", substream->runtime->rate);
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = substream->runtime->rate;
    }
    return 0;
}

static int mt6323_codec_trigger(struct snd_pcm_substream *substream , int command , struct snd_soc_dai *Daiport)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    switch (command)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            break;
    }

    printk("mt6323_codec_trigger command = %d\n ", command);
    return 0;
}

static const struct snd_soc_dai_ops mt6323_aif1_dai_ops =
{
    .startup    = mt6323_codec_startup,
    .prepare   = mt6323_codec_prepare,
    .trigger     = mt6323_codec_trigger,
};

static struct snd_soc_dai_driver mtk_6323_dai_codecs[] =
{
    {
        .name = MT_SOC_CODEC_TXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_DL1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
    {
        .name = MT_SOC_CODEC_RXDAI_NAME,
        .capture = {
            .stream_name = MT_SOC_UL1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
    {
        .name = MT_SOC_CODEC_PCMTXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_PCM2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
    {
        .name = MT_SOC_CODEC_PCMRXDAI_NAME,
        .capture = {
            .stream_name = MT_SOC_PCM2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
    {
        .name = MT_SOC_CODEC_FMI2S2RXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_FM_I2S2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
        .capture = {
            .stream_name = MT_SOC_FM_I2S2_RECORD_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
    {
        .name = MT_SOC_CODEC_STUB_NAME,
        .playback = {
            .stream_name = MT_SOC_ROUTING_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
};


uint32 GetDLNewIFFrequency(unsigned int frequency)
{
    printk("AudioPlatformDevice ApplyDLNewIFFrequency ApplyDLNewIFFrequency = %d", frequency);
    uint32 Reg_value = 0;
    switch (frequency)
    {
        case 8000:
            Reg_value = 0;
            break;
        case 11025:
            Reg_value = 1;
            break;
        case 12000:
            Reg_value = 2;
            break;
        case 16000:
            Reg_value = 3;
            break;
        case 22050:
            Reg_value = 4;
            break;
        case 24000:
            Reg_value = 5;
            break;
        case 32000:
            Reg_value = 6;
            break;
        case 44100:
            Reg_value = 7;
            break;
        case 48000:
            Reg_value = 8;
        default:
            printk("ApplyDLNewIFFrequency with frequency = %d", frequency);
    }
    return Reg_value;
}

static bool GetDLStatus()
{
    int i = 0;
    for (i = 0; i < AUDIO_ANALOG_DEVICE_2IN1_SPK ; i++)
    {
        if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
        {
            return true;
        }
    }
    return false;
}

static void TurnOnDacPower()
{
    printk("TurnOnDacPower\n");
    ClsqEnable(true);
    Topck_Enable(true);
    Ana_Set_Reg(AUDIO_TOP_CON0 , 0x0000, 0xffffffff);
    Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffffffff);
    Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffffffff); //sdm audio fifo clock power on
    Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffffffff); //sdm power on
    Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffffffff); //sdm fifo enable
    Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffffffff); //set attenuation gain
    Ana_Set_Reg(AFE_UL_DL_CON0 , 0x0001, 0xffffffff); //[0] afe enable

    Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0 , GetDLNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC]) << 12 | 0x330 , 0xffffffff);
    Ana_Set_Reg(AFE_DL_SRC2_CON0_H , mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] , 0xffff000f);

    Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1801 , 0xffffffff); //turn off mute function and turn on dl
    Ana_Set_Reg(PMIC_AFE_TOP_CON0 , 0x0000 , 0xffffffff); //set DL in normal path, not from sine gen table
}

static void TurnOffDacPower()
{
    printk("TurnOffDacPower\n");
    Ana_Set_Reg(AFE_CLASSH_CFG0, 0xd518, 0xffff); // ClassH off
    Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0518, 0xffff); // NCP off
}

static void Audio_Amp_Change(int channels , bool enable)
{
    if (enable)
    {
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
        // here pmic analog control
        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] == false &&
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] == false)
        {
            printk("Audio_Amp_Change on amp\n");
            Ana_Set_Reg(0x0680, 0x0000, 0xffff); // Enable AUDGLB
            OpenClassH();
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); // Enable cap-less LDOs (1.6V)
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); // Enable NV regulator (-1.6V)
            Ana_Set_Reg(ZCD_CON0,   0x0000, 0xffff); // Disable AUD_ZCD
            Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); // Disable headphone, voice and short-ckt protection.
            Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); //Enable IBIST
            Ana_Set_Reg(ZCD_CON2,  0x0F9F , 0xffff); //Set HPR/HPL gain as minimum (~ -40dB)
            Ana_Set_Reg(ZCD_CON3,  0x001F , 0xffff); //Set voice gain as minimum (~ -40dB)
            Ana_Set_Reg(AUDBUF_CFG1,  0x0900 , 0xffff); //De_OSC of HP and enable output STBENH
            Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); //De_OSC of voice, enable output STBENH
            Ana_Set_Reg(AUDBUF_CFG0,  0xE009 , 0xffff); //Enable voice driver
            Ana_Set_Reg(AUDBUF_CFG1,  0x0940 , 0xffff); //Enable pre-charge buffer
            msleep(1);
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); //Enable AUD_CLK
            if (channels == AUDIO_ANALOG_CHANNELS_LEFT1)
            {
                Ana_Set_Reg(AUDDAC_CFG0, 0x000d , 0xffff); //Enable Audio DAC
            }
            else
            {
                Ana_Set_Reg(AUDDAC_CFG0, 0x000e , 0xffff); //Enable Audio DAC
            }
            Ana_Set_Reg(AUDDAC_CFG0, 0x000f , 0xffff); //Enable Audio DAC
            SetDcCompenSation();

            Ana_Set_Reg(AUDBUF_CFG0, 0xE149 , 0xffff); // Switch HP MUX to audio DAC
            Ana_Set_Reg(AUDBUF_CFG0, 0xE14F , 0xffff); // Enable HPR/HPL
            Ana_Set_Reg(AUDBUF_CFG1, 0x0900 , 0xffff); // Disable pre-charge buffer
            Ana_Set_Reg(AUDBUF_CFG2, 0x0020 , 0xffff); // Disable De_OSC of voice
            Ana_Set_Reg(AUDBUF_CFG0, 0xE14E , 0xffff); // Disable voice buffer
            Ana_Set_Reg(ZCD_CON2,       0x0489 , 0xffff); // Set HPR/HPL gain as 0dB, step by step

        }
        else if (channels == AUDIO_ANALOG_CHANNELS_LEFT1)
        {
            Ana_Set_Reg(AUDDAC_CFG0, 0x000f, 0x0020); // enable audio bias. enable audio DAC, HP buffers

        }
        else if (channels == AUDIO_ANALOG_CHANNELS_RIGHT1)
        {
            Ana_Set_Reg(AUDDAC_CFG0, 0x000f, 0x0040); // enable audio bias. enable audio DAC, HP buffers
        }
    }
    else
    {

        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] == false &&
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] == false)
        {
            printk("Audio_Amp_Change off amp\n");
            Ana_Set_Reg(AUDBUF_CFG0, 0xE149, 0xffff); // Disable HPR/HPL
            Ana_Set_Reg(AUDDAC_CFG0, 0x0000, 0xffff); // Disable Audio DAC
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5500, 0xffff); // Disable AUD_CLK
            Ana_Set_Reg(IBIASDIST_CFG0, 0x0192, 0xffff); // Disable IBIST
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0028, 0xffff); // Disable NV regulator (-1.6V)
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0000, 0xffff); // Disable cap-less LDOs (1.6V)
        }
        else if (channels == AUDIO_ANALOG_CHANNELS_LEFT1)
        {
            Ana_Set_Reg(AUDDAC_CFG0, 0x000e, 0xffff); // enable audio bias. enable audio DAC, HP buffers
        }
        else if (channels == AUDIO_ANALOG_CHANNELS_RIGHT1)
        {
            Ana_Set_Reg(AUDDAC_CFG0, 0x000d, 0xffff); // enable audio bias. enable audio DAC, HP buffers
        }
        if (GetDLStatus() == false)
        {
            TurnOffDacPower();
        }
    }
}


static int Audio_AmpL_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpL_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL];
    return 0;
}

static int Audio_AmpL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    mutex_lock(&Ana_Ctrl_Mutex);

    printk("%s() gain = %d \n ", __func__, ucontrol->value.integer.value[0]);
    if (ucontrol->value.integer.value[0])
    {
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = ucontrol->value.integer.value[0];
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , false);
    }
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}


static int Audio_AmpR_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR];
    return 0;
}

static int Audio_AmpR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    mutex_lock(&Ana_Ctrl_Mutex);

    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = ucontrol->value.integer.value[0];
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , false);
    }
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}

static bool  GetULinkStatus()
{
    int i = 0;
    for (i = AUDIO_ANALOG_DEVICE_IN_ADC1 ; i < AUDIO_ANALOG_DEVICE_MAX ; i ++)
    {
        if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
        {
            return true;
        }
    }
    return false;
}

// todo:
static void Voice_Amp_Change(bool enable)
{
    int i = 0;
    if (enable)
    {
        printk("turn on ampL\n");
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
    }
    else
    {
        printk("turn off ampL\n");
        if (GetDLStatus() == false)
        {
            TurnOffDacPower();
        }
    }
}

static int Voice_Amp_Get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL];
    return 0;
}

static int Voice_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    mutex_lock(&Ana_Ctrl_Mutex);

    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        Voice_Amp_Change(true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] = ucontrol->value.integer.value[0];
        Voice_Amp_Change(false);
    }
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}


static void Speaker_Amp_Change(bool enable)
{
    if (enable)
    {
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
        printk("turn on Speaker_Amp_Change \n");
        // here pmic analog control
        Ana_Set_Reg(AUDNVREGGLB_CFG0  , 0x0000 , 0xffffffff);
        OpenClassH();
        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0028 , 0xffffffff); //Enable cap-less LDOs (1.6V)
        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0068 , 0xffffffff); //Enable NV regulator (-1.6V)
        Ana_Set_Reg(ZCD_CON0  , 0x0000 , 0xffffffff); //Disable AUD_ZCD
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00C0 , 0xffffffff); // Disable line-out and short-ckt protection. LO MUX is opened
        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0092 , 0xffffffff); // Enable IBIST
        Ana_Set_Reg(ZCD_CON1  , 0x0F9F , 0xffffffff); // Set LOR/LOL gain as minimum (~ -40dB)
        Ana_Set_Reg(AUDBUF_CFG7  , 0x0102 , 0xffffffff); // De_OSC of LO and enable output STBENH
        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5501 , 0xffffffff); // Enable AUD_CLK
        Ana_Set_Reg(AUDDAC_CFG0  , 0x000F , 0xffffffff); //Enable Audio DAC
        SetDcCompenSation();
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); //Switch LO MUX to audio DAC
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00EB , 0xffffffff); //Enable LOR/LOL
        Ana_Set_Reg(ZCD_CON1  , 0x0489 , 0xffffffff); // Set LOR/LOL gain as 0dB
    }
    else
    {
        printk("turn off Speaker_Amp_Change \n");
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); // Disable LOR/LOL
        Ana_Set_Reg(AUDDAC_CFG0  , 0x0000 , 0xffffffff); // Disable Audio DAC
        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5500 , 0xffffffff); // Disable AUD_CLK
        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0192 , 0xffffffff); // Disable IBIST
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0028 , 0xffffffff); // Disable NV regulator (-1.6V)
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0000 , 0xffffffff); // Disable cap-less LDOs (1.6V)
        if (GetDLStatus() == false)
        {
            TurnOffDacPower();
        }
    }
}

static int Speaker_Amp_Get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] ;
    return 0;
}

static int Speaker_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s() gain = %d \n ", __func__, ucontrol->value.integer.value[0]);
    if (ucontrol->value.integer.value[0])
    {
        Speaker_Amp_Change(true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
        Speaker_Amp_Change(false);
    }
    return 0;
}

static void Headset_Speaker_Amp_Change(bool enable)
{
    if (enable)
    {
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
        printk("turn on Speaker_Amp_Change \n");
        // here pmic analog control
        Ana_Set_Reg(AUDNVREGGLB_CFG0  , 0x0000 , 0xffffffff);
        OpenClassH();

        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0028 , 0xffffffff); //Enable cap-less LDOs (1.6V)
        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0068 , 0xffffffff); //Enable NV regulator (-1.6V)
        Ana_Set_Reg(ZCD_CON0  , 0x0000 , 0xffffffff); //Disable AUD_ZCD

        Ana_Set_Reg(AUDBUF_CFG0  , 0xE008 , 0xffffffff); //Disable headphone, voice and short-ckt protection.
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00C0 , 0xffffffff); // Disable line-out and short-ckt protection. LO MUX is opened

        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0092 , 0xffffffff); // Enable IBIST
        Ana_Set_Reg(ZCD_CON2  , 0x0F9F , 0xffffffff); // Set LOR/LOL gain as minimum (~ -40dB)
        Ana_Set_Reg(ZCD_CON1  , 0x0F9F , 0xffffffff); // Set LOR/LOL gain as minimum (~ -40dB)
        Ana_Set_Reg(ZCD_CON3 , 0x001F , 0xffffffff); // Set voice gain as minimum (~ -40dB)

        Ana_Set_Reg(AUDBUF_CFG1  , 0x0900 , 0xffffffff); // De_OSC of HP and enable output STBENH
        Ana_Set_Reg(AUDBUF_CFG7  , 0x0102 , 0xffffffff); // De_OSC of LO and enable output STBENH
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0022 , 0xffffffff); // De_OSC of voice, enable output STBENH
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE009 , 0xffffffff); // Enable voice driver
        Ana_Set_Reg(AUDBUF_CFG1  , 0x0940 , 0xffffffff); // Enable pre-charge buffer_map_state
        msleep(1);

        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5501 , 0xffffffff); // Enable AUD_CLK
        Ana_Set_Reg(AUDDAC_CFG0  , 0x000F , 0xffffffff); //Enable Audio DAC
        SetDcCompenSation();
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); //Switch LO MUX to audio DAC
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00EB , 0xffffffff); //Enable LOR/LOL
        Ana_Set_Reg(ZCD_CON1  , 0x0489 , 0xffffffff); // Set LOR/LOL gain as 0dB

        Ana_Set_Reg(AUDBUF_CFG0  , 0xE0A9 , 0xffffffff); // Switch HP MUX to audio DAC
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE0AF , 0xffffffff); // Enable HPR/HPL
        Ana_Set_Reg(AUDBUF_CFG1  , 0x0900 , 0xffffffff); // Disable pre-charge buffer
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0020 , 0xffffffff); // Disable De_OSC of voice
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE0AE , 0xffffffff); // Disable voice buffer
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0489 , 0xffffffff); // Set HPR/HPL gain as 0dB

    }
    else
    {
        printk("turn off Speaker_Amp_Change \n");
        Ana_Set_Reg(AUDDAC_CFG0  , 0xE149 , 0xffffffff); // Disable HPR/HPL
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); // Disable LOR/LOL
        Ana_Set_Reg(AUDDAC_CFG0  , 0x0000 , 0xffffffff); // Disable Audio DAC
        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5500 , 0xffffffff); // Disable AUD_CLK
        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0192 , 0xffffffff); // Disable IBIST
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0028 , 0xffffffff); // Disable NV regulator (-1.6V)
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0000 , 0xffffffff); // Disable cap-less LDOs (1.6V)
        if (GetDLStatus() == false)
        {
            TurnOffDacPower();
        }
    }

}


static int Headset_Speaker_Amp_Get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R] ;
    return 0;
}

static int Headset_Speaker_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s() gain = %d \n ", __func__, ucontrol->value.integer.value[0]);
    if (ucontrol->value.integer.value[0])
    {
        Headset_Speaker_Amp_Change(true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R] = ucontrol->value.integer.value[0];
        Headset_Speaker_Amp_Change(false);
    }
    return 0;
}


static const char *amp_function[] = {"Off", "On"};
static const char *DAC_SampleRate_function[] = {"8000", "11025", "16000", "24000", "32000", "44100", "48000"};
static const char *DAC_DL_PGA_Headset_GAIN[] = {"-5Db", "-3Db", "-1Db", "1Db", "3Db", "5Db", "7Db", "9Db"};
static const char *DAC_DL_PGA_Handset_GAIN[] = {"-21Db", "-19Db", "-17Db", "-15Db", "-13Db", "-11Db", "-9Db", "-7Db", "-5Db",
                                                "-3Db", "-1Db", "1Db", "3Db", "5Db", "7Db", "9Db"
                                               };
static const char *DAC_DL_PGA_Speaker_GAIN[] = {"-60Db", "0Db", "1Db", "2Db", "3Db", "4Db", "5Db", "6Db", "7Db",
                                                "8Db", "9Db", "10Db", "11Db", "12Db", "13Db", "14Db", "15Db", "16Db", "17Db"
                                               };

static const char *Voice_Mux_function[] = {"Voice", "Speaker"};

static int Speaker_PGA_Get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL];
    return 0;
}

static int Speaker_PGA_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    int index = 0;

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(SPK_CON9, index << 8, 0x00000f00);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
    return 0;
}

static int Handset_PGA_Get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL];
    return 0;
}

static int Handset_PGA_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    int index = 0;

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDTOP_CON7, index << 4, 0x000000f0);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL] = ucontrol->value.integer.value[0];
    return 0;
}


static int Headset_PGAL_Get(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL];
    return 0;
}

static int Headset_PGAL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    int index = 0;

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDTOP_CON5, index << 12, 0x00007000);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL] = ucontrol->value.integer.value[0];
    return 0;
}

static int Headset_PGAR_Get(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
    return 0;
}

static int Headset_PGAR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    int index = 0;

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDTOP_CON5, index << 8, 0x000000700);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] = ucontrol->value.integer.value[0];
    return 0;
}


static int Voice_Mux_Get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_VOICE]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_VOICE];
    return 0;
}

static int Voice_Mux_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        printk("%s()\n", __func__);
        snd_soc_dapm_disable_pin(&codec->dapm, "SPEAKER");
        snd_soc_dapm_disable_pin(&codec->dapm, "RX_BIAS");
        snd_soc_dapm_sync(&codec->dapm);
    }
    else
    {
        printk("%s()\n", __func__);
        snd_soc_dapm_enable_pin(&codec->dapm, "SPEAKER");
        snd_soc_dapm_enable_pin(&codec->dapm, "RX_BIAS");
        snd_soc_dapm_sync(&codec->dapm);
    }

    Ana_Set_Reg(AUDTOP_CON7, ucontrol->value.integer.value[0]  << 12, 0x1000);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_VOICE] = ucontrol->value.integer.value[0];
    return 0;
}


static const struct soc_enum Audio_Amp_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    // here comes pga gain setting
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN), DAC_DL_PGA_Headset_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN), DAC_DL_PGA_Headset_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN), DAC_DL_PGA_Handset_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN), DAC_DL_PGA_Speaker_GAIN),
    // Mux Function
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Voice_Mux_function), Voice_Mux_function),
};

static const struct snd_kcontrol_new mt6331_snd_controls[] =
{
    SOC_ENUM_EXT("Audio_Amp_R_Switch", Audio_Amp_Enum[0], Audio_AmpR_Get, Audio_AmpR_Set),
    SOC_ENUM_EXT("Audio_Amp_L_Switch", Audio_Amp_Enum[1], Audio_AmpL_Get, Audio_AmpL_Set),
    SOC_ENUM_EXT("Voice_Amp_Switch", Audio_Amp_Enum[2], Voice_Amp_Get, Voice_Amp_Set),
    SOC_ENUM_EXT("Speaker_Amp_Switch", Audio_Amp_Enum[3], Speaker_Amp_Get, Speaker_Amp_Set),
    SOC_ENUM_EXT("Headset_Speaker_Amp_Switch", Audio_Amp_Enum[4], Headset_Speaker_Amp_Get, Headset_Speaker_Amp_Set),
    SOC_ENUM_EXT("Headset_PGAL_GAIN", Audio_Amp_Enum[5], Headset_PGAL_Get, Headset_PGAL_Set),
    SOC_ENUM_EXT("Headset_PGAR_GAIN", Audio_Amp_Enum[6], Headset_PGAR_Get, Headset_PGAR_Set),
    SOC_ENUM_EXT("Hanset_PGA_GAIN", Audio_Amp_Enum[7], Handset_PGA_Get, Handset_PGA_Set),
    SOC_ENUM_EXT("Speaker_PGA_GAIN", Audio_Amp_Enum[8], Speaker_PGA_Get, Speaker_PGA_Set),
};

static const struct snd_kcontrol_new mt6331_Voice_Switch[] =
{
    SOC_DAPM_ENUM_EXT("Voice Mux", Audio_Amp_Enum[9], Voice_Mux_Get, Voice_Mux_Set),
};

static bool GetAdcStatus()
{
    int i = 0;
    for (i = AUDIO_ANALOG_DEVICE_IN_ADC1 ; i < AUDIO_ANALOG_DEVICE_MAX ; i++)
    {
        if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
        {
            return true;
        }
    }
    return false;
}


static bool TurnOnADcPower(int ADCType, bool enable)
{
    if (enable)
    {
        if (GetAdcStatus() == false)
        {
            uint32 ULIndex = GetULFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]);
            NvregEnable(true);
            ClsqEnable(true);

            Topck_Enable(true);

            Ana_Set_Reg(AUDADC_CFG0, 0x0400, 0xffff);      // Enable ADC CLK
            Ana_Set_Reg(AUDMICBIAS_CFG0, 0x710F, 0xffff);   //Enable MICBIAS0,1 (2.7V)
            Ana_Set_Reg(AUDMICBIAS_CFG1, 0x710F, 0xffff);   //Enable MICBIAS2,3 (2.7V)
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0007, 0xffff);   //Enable LCLDO18_ENC (1.8V), Remote-Sense
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2277, 0xffff);   //Enable LCLDO19_ADCCH0_1, Remote-Sense
            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0033, 0xffff);   //Set PGA CH0_1 gain = 18dB
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0051, 0xffff);   //Enable PGA CH0_1 (CH0 in)
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0xffff);   //Enable ADC CH0_1 (PGA in)

            ClsqEnable(true);
            AdcClockEnable(true);

            Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);   //configure ADC setting
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   //power on ADC clk
            Ana_Set_Reg(AFE_MIC_ARRAY_CFG, 0x44e4, 0xffff);   //AFE_MIC_ARRAY_CFG
            Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   //configure ADC setting
            Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);   //turn on afe


            Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x302F | (GetULNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]) << 10), 0xffff); // config UL up8x_rxif adc voice mode
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (ULIndex << 3 | ULIndex << 1) , 0x0010); // ULsampling rate

            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0041, 0xffff);   //power on uplink

        }
        // todo , open ADC indivisaully
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   //Enable LCLDO19_ADCCH0_1, Remote-Sense
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0051, 0x0077);   //Enable PGA CH0_1
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x00ff);   //Enable ADC CH0_1 (PGA in)
        }
        else   if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x00f0);   //Enable LCLDO19_ADCCH2, Remote-Sensen)
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x01c1, 0x01c1);   //Enable PGA CH2
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x00ff);   //Enable ADC CH0_1 (PGA in)
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0700, 0x0f00);   //Enable LCLDO19_ADCCH3, Remote-Sense
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0500, 0x0500);   //Enable ADC CH3 (PGA in)
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0005, 0x000f);   //Enable preamp CH3
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x7000, 0xf000);   //Enable LCLDO19_ADCCH4, Remote-Sense
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x5000, 0x5000);   //Enable ADC CH4 (PGA in)
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0050, 0x00f0);   //Enable preamp CH4
        }
        else
        {
            printk("\n");
        }
    }
    else
    {
        if (GetAdcStatus() == false)
        {

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0000, 0xffff);   //Disable ADC CH0_1 (PGA in) Disable ADC CH_2 (PGA in)
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0000, 0xffff);   //Disable PGA CH0_1 (CH0 in) Disable PGA CH_2
            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0000, 0xffff);   //Set PGA CH0_1 gain = 0dB  Set PGA CH_2 gain = 0dB

            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2222, 0xffff);   //disable LCLDO19_ADCCH0_1, Remote-Sense
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0002, 0xffff);   //disable LCLDO18_ENC (1.8V), Remote-Sense

            Ana_Set_Reg(AUDMICBIAS_CFG1, 0x0000, 0xffff);   //power on clock
            Ana_Set_Reg(AUDMICBIAS_CFG0, 0x0000, 0xffff);   //power on ADC clk
            Ana_Set_Reg(AUDADC_CFG0, 0x0000, 0xffff);   //configure ADC setting
            ClsqEnable(false);
            NvregEnable(false);
            if (GetDLStatus() == false)
            {
                // check for if DL/UL will share same register

            }
            else
            {

            }

        }
        //todo , close analog
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {

        }
    }
}

// here start uplink power function
static const char *ADC_function[] = {"Off", "On"};
static const char *PreAmp_Mux_function[] = {"OPEN", "IN_ADC1", "IN_ADC2"};
static const char *ADC_SampleRate_function[] = {"8000", "16000", "32000", "48000"};
static const char *ADC_UL_PGA_GAIN[] = { "0Db", "6Db", "12Db", "18Db", "24Db","30Db"};
static const char *Pmic_Digital_Mux[] = { "ADC1", "ADC2", "ADC3", "ADC4"};


static const struct soc_enum Audio_UL_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(PreAmp_Mux_function), PreAmp_Mux_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
};

static int Audio_ADC1_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1];
    return 0;
}

static int Audio_ADC1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1] = ucontrol->value.integer.value[0];
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC1 , false);
    }
    return 0;
}

static int Audio_ADC2_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2];
    return 0;
}

static int Audio_ADC2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC2 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2] = ucontrol->value.integer.value[0];
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC2 , false);
    }
    return 0;
}

static int Audio_ADC3_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3];
    return 0;
}

static int Audio_ADC3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC3 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3] = ucontrol->value.integer.value[0];
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC3 , false);
    }
    return 0;
}

static int Audio_ADC4_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4];
    return 0;
}

static int Audio_ADC4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC4 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4] = ucontrol->value.integer.value[0];
        TurnOnADcPower(AUDIO_ANALOG_DEVICE_IN_ADC4 , false);
    }
    return 0;
}



static int Audio_PreAmp1_Get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]; = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1];
    return 0;
}

static int Audio_PreAmp1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(PreAmp_Mux_function))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    // todo :: set pinmux by k control
    // pinmux open
    if (ucontrol->value.integer.value[0] == 0)
    {
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0041, 0x003f);	// pinumx open
    }
    else if (ucontrol->value.integer.value[0] == 1)
    {
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0051, 0x003f);       // ADC 0
    }
    // ADC2
    else if (ucontrol->value.integer.value[0] == 2)
    {
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0071, 0x003f);    // ADC 1
    }
    else if (ucontrol->value.integer.value[0] == 3)
    {

    }
    else if (ucontrol->value.integer.value[0] == 4)
    {

    }
    else
    {

        printk("AnalogSetMux warning");
    }
    printk("%s() done \n", __func__);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_PreAmp2_Get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    printk("%s() mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2]; = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2];
    return 0;
}

static int Audio_PreAmp2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    printk("%s()\n", __func__);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(PreAmp_Mux_function))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    if (ucontrol->value.integer.value[0] == 0)
    {
    }
    else if (ucontrol->value.integer.value[0] == 1)
    {
    }
    else if (ucontrol->value.integer.value[0] == 2)
    {
    }
    else if (ucontrol->value.integer.value[0] == 3)
    {
    }
    else
    {
    }
    printk("%s() done \n", __func__);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_2] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_PGA1_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL];
    return 0;
}

static int Audio_PGA1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index , 0x0007);
    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_MICAMPL] = ucontrol->value.integer.value[0];
    return 0;
}


static int Audio_PGA2_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_PGA2_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL];
    return 0;
}

static int Audio_PGA2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index <<4, 0x0070);
    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_MICAMPL] = ucontrol->value.integer.value[0];
    return 0;
}


static int Audio_PGA3_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL];
    return 0;
}

static int Audio_PGA3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index <<8, 0x0700);
    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_MICAMPL] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_PGA4_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMPL];
    return 0;
}

static int Audio_PGA4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index <<12, 0x7000);
    mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_MICAMPL] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_MicSource1_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_MicSource1_Get = %d\n", mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1];
    return 0;
}

static int Audio_MicSource1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index | index <<8, 0x0303);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_MicSource2_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__,mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_2];
    return 0;
}

static int Audio_MicSource2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index <<2| index <<10, 0x0c0c);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_2] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_MicSource3_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__,mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_3]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_3];
    return 0;
}

static int Audio_MicSource3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index <<4| index <<12, 0x3030);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_3] = ucontrol->value.integer.value[0];
    return 0;
}


static int Audio_MicSource4_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__,mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_4]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_4];
    return 0;
}

static int Audio_MicSource4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index <<6| index <<14, 0xc0c0);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_4] = ucontrol->value.integer.value[0];
    return 0;
}



static const struct snd_kcontrol_new mt6331_UL_Codec_controls[] =
{
    SOC_ENUM_EXT("Audio_ADC_1_Switch", Audio_UL_Enum[0], Audio_ADC1_Get, Audio_ADC1_Set),
    SOC_ENUM_EXT("Audio_ADC_2_Switch", Audio_UL_Enum[1], Audio_ADC2_Get, Audio_ADC2_Set),
    SOC_ENUM_EXT("Audio_ADC_3_Switch", Audio_UL_Enum[2], Audio_ADC2_Get, Audio_ADC2_Set),
    SOC_ENUM_EXT("Audio_ADC_4_Switch", Audio_UL_Enum[3], Audio_ADC2_Get, Audio_ADC2_Set),
    SOC_ENUM_EXT("Audio_Preamp1_Switch", Audio_UL_Enum[4], Audio_PreAmp1_Get, Audio_PreAmp1_Set),
    SOC_ENUM_EXT("Audio_PGA1_Setting", Audio_UL_Enum[5], Audio_PGA1_Get, Audio_PGA1_Set),
    SOC_ENUM_EXT("Audio_PGA2_Setting", Audio_UL_Enum[6], Audio_PGA2_Get, Audio_PGA2_Set),
    SOC_ENUM_EXT("Audio_PGA3_Setting", Audio_UL_Enum[7], Audio_PGA3_Get, Audio_PGA3_Set),
    SOC_ENUM_EXT("Audio_PGA4_Setting", Audio_UL_Enum[8], Audio_PGA4_Get, Audio_PGA4_Set),
    SOC_ENUM_EXT("Audio_MicSource1_Setting", Audio_UL_Enum[9], Audio_MicSource1_Get, Audio_MicSource1_Set),
    SOC_ENUM_EXT("Audio_MicSource2_Setting", Audio_UL_Enum[10], Audio_MicSource2_Get, Audio_MicSource2_Set),
    SOC_ENUM_EXT("Audio_MicSource3_Setting", Audio_UL_Enum[11], Audio_MicSource3_Get, Audio_MicSource3_Set),
    SOC_ENUM_EXT("Audio_MicSource4_Setting", Audio_UL_Enum[12], Audio_MicSource4_Get, Audio_MicSource4_Set),
};

static int hpl_dac_event(struct snd_soc_dapm_widget *w,
                         struct snd_kcontrol *kcontrol, int event)
{
    struct snd_soc_codec *codec = w->codec;
    printk("hpl_dac_event event = %d\n", event);

    printk(codec->dev, "%s %s %d\n", __func__, w->name, event);

    switch (event)
    {
        case SND_SOC_DAPM_PRE_PMU:
            printk("%s SND_SOC_DAPM_PRE_PMU", __func__);
            break;
        case SND_SOC_DAPM_POST_PMU:
            printk("%s SND_SOC_DAPM_POST_PMU", __func__);
            break;
        case SND_SOC_DAPM_PRE_PMD:
            printk("%s SND_SOC_DAPM_PRE_PMD", __func__);
            break;
        case SND_SOC_DAPM_POST_PMD:
            printk("%s SND_SOC_DAPM_POST_PMD", __func__);
            break;
        case SND_SOC_DAPM_PRE_REG:
            printk("%s SND_SOC_DAPM_PRE_REG", __func__);
            break;
        case SND_SOC_DAPM_POST_REG:
            printk("%s SND_SOC_DAPM_POST_REG", __func__);
            break;
    }
    return 0;
}


static int hpr_dac_event(struct snd_soc_dapm_widget *w,
                         struct snd_kcontrol *kcontrol, int event)
{
    struct snd_soc_codec *codec = w->codec;
    printk("hpr_dac_event event = %d\n", event);

    printk(codec->dev, "%s %s %d\n", __func__, w->name, event);

    switch (event)
    {
        case SND_SOC_DAPM_PRE_PMU:
            printk("%s SND_SOC_DAPM_PRE_PMU", __func__);
            break;
        case SND_SOC_DAPM_POST_PMU:
            printk("%s SND_SOC_DAPM_POST_PMU", __func__);
            break;
        case SND_SOC_DAPM_PRE_PMD:
            printk("%s SND_SOC_DAPM_PRE_PMD", __func__);
            break;
        case SND_SOC_DAPM_POST_PMD:
            printk("%s SND_SOC_DAPM_POST_PMD", __func__);
            break;
        case SND_SOC_DAPM_PRE_REG:
            printk("%s SND_SOC_DAPM_PRE_REG", __func__);
            break;
        case SND_SOC_DAPM_POST_REG:
            printk("%s SND_SOC_DAPM_POST_REG", __func__);
            break;
    }
    return 0;
}

static int spk_amp_event(struct snd_soc_dapm_widget *w,
                         struct snd_kcontrol *kcontrol, int event)
{
    struct snd_soc_codec *codec = w->codec;
    printk("spk_amp_event event = %d\n", event);
    switch (event)
    {
        case SND_SOC_DAPM_PRE_PMU:
            printk("%s SND_SOC_DAPM_PRE_PMU", __func__);
            break;
        case SND_SOC_DAPM_POST_PMU:
            printk("%s SND_SOC_DAPM_POST_PMU", __func__);
            break;
        case SND_SOC_DAPM_PRE_PMD:
            printk("%s SND_SOC_DAPM_PRE_PMD", __func__);
            break;
        case SND_SOC_DAPM_POST_PMD:
            printk("%s SND_SOC_DAPM_POST_PMD", __func__);
            break;
    }
    return 0;
}

static void speaker_event(struct snd_soc_dapm_widget *w,
                          struct snd_kcontrol *kcontrol, int event)
{
    struct snd_soc_codec *codec = w->codec;
    printk("speaker_event = %d\n", event);
    switch (event)
    {
        case SND_SOC_DAPM_PRE_PMU:
            printk("%s SND_SOC_DAPM_PRE_PMU", __func__);
            break;
        case SND_SOC_DAPM_POST_PMU:
            printk("%s SND_SOC_DAPM_POST_PMU", __func__);
            break;
        case SND_SOC_DAPM_PRE_PMD:
            printk("%s SND_SOC_DAPM_PRE_PMD", __func__);
            break;
        case SND_SOC_DAPM_POST_PMD:
            printk("%s SND_SOC_DAPM_POST_PMD", __func__);
        case SND_SOC_DAPM_PRE_REG:
            printk("%s SND_SOC_DAPM_PRE_REG", __func__);
        case SND_SOC_DAPM_POST_REG:
            printk("%s SND_SOC_DAPM_POST_REG", __func__);
            break;
    }
    return 0;
}


/* The register address is the same as other codec so it can use resmgr */
static int codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
                                struct snd_kcontrol *kcontrol, int event)
{
    struct snd_soc_codec *codec = w->codec;
    printk("codec_enable_rx_bias = %d\n", event);
    switch (event)
    {
        case SND_SOC_DAPM_PRE_PMU:
            printk("%s SND_SOC_DAPM_PRE_PMU", __func__);
            break;
        case SND_SOC_DAPM_POST_PMU:
            printk("%s SND_SOC_DAPM_POST_PMU", __func__);
            break;
        case SND_SOC_DAPM_PRE_PMD:
            printk("%s SND_SOC_DAPM_PRE_PMD", __func__);
            break;
        case SND_SOC_DAPM_POST_PMD:
            printk("%s SND_SOC_DAPM_POST_PMD", __func__);
        case SND_SOC_DAPM_PRE_REG:
            printk("%s SND_SOC_DAPM_PRE_REG", __func__);
        case SND_SOC_DAPM_POST_REG:
            printk("%s SND_SOC_DAPM_POST_REG", __func__);
            break;
    }
    return 0;
}

static const struct snd_soc_dapm_widget mt6331_dapm_widgets[] =
{
    /* Outputs */
    SND_SOC_DAPM_OUTPUT("EARPIECE"),
    SND_SOC_DAPM_OUTPUT("HEADSET"),
    SND_SOC_DAPM_OUTPUT("SPEAKER"),

    SND_SOC_DAPM_PGA_E("SPEAKER PGA", SND_SOC_NOPM,
    0, 0, NULL, 0, speaker_event,
    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),

    SND_SOC_DAPM_DAC_E("RX_BIAS", NULL, SND_SOC_NOPM, 0, 0,
    codec_enable_rx_bias,     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),

    SND_SOC_DAPM_MUX_E("VOICE_Mux_E", SND_SOC_NOPM, 0, 0,
    &mt6331_Voice_Switch, codec_enable_rx_bias,
    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),


    /*
    SND_SOC_DAPM_MIXER_E("HPL DAC", 0, 0, 0, mt6331_snd_controls,  ARRAY_SIZE(mt6331_snd_controls),
    hpl_dac_event,
    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),
    SND_SOC_DAPM_MIXER_E("HPR DAC", SND_SOC_NOPM, 0, 0, NULL, 0,
    hpr_dac_event,
    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),
    SND_SOC_DAPM_MIXER("HPL_DAC_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),
    SND_SOC_DAPM_MIXER("HPR_DAC_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),
    SND_SOC_DAPM_MIXER("HPR_VOICE_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),
    SND_SOC_DAPM_MIXER("HPR_SPEAKER_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),

    SND_SOC_DAPM_MIXER_E("HPL_PA_MIXER", 4, 0, 0, NULL, 0, hpl_dac_event,
    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),
    SND_SOC_DAPM_MIXER("HPR_PA_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),

    SND_SOC_DAPM_DAC_E("DAC1R", MT_SOC_DL1_STREAM_NAME , 0, 0, 0, hpl_dac_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG),
    */

};

static const struct snd_soc_dapm_route mtk_audio_map[] =
{
    //{"HPL DAC", "Audio_Amp_L_Switch", "HPL_PA_MIXER"},
    //{"HPR DAC", "Audio_Amp_R_Switch", "HPR_PA_MIXER"},
    //{"HPL DAC", "Audio_Amp_L_Switch", "HPL_PA_MIXER"},
    //{"SPEAKER PGA", NULL ,"SPEAKER"},
    //{"SPEAKER", NULL , "SPEAKER PGA"},
    {"VOICE_Mux_E", "Voice Mux", "SPEAKER PGA"},
};

static void mt6331_codec_init_reg(struct snd_soc_codec *codec)
{
    printk("mt6331_codec_init_reg \n");
}

static int mt6331_codec_probe(struct snd_soc_codec *codec)
{
    printk(codec->dev, "%s()\n", __func__);
    struct snd_soc_dapm_context *dapm = &codec->dapm;
    mt6331_codec_init_reg(codec);
    snd_soc_dapm_new_controls(dapm, mt6331_dapm_widgets,
                              ARRAY_SIZE(mt6331_dapm_widgets));
    snd_soc_dapm_add_routes(dapm, mtk_audio_map,
                            ARRAY_SIZE(mtk_audio_map));

    //add codec controls
    snd_soc_add_codec_controls(codec, mt6331_snd_controls,
                               ARRAY_SIZE(mt6331_snd_controls));
    snd_soc_add_codec_controls(codec, mt6331_UL_Codec_controls,
                               ARRAY_SIZE(mt6331_UL_Codec_controls));
    snd_soc_add_codec_controls(codec, mt6331_Voice_Switch,
                               ARRAY_SIZE(mt6331_Voice_Switch));

    // here to set  private data
    mCodec_data = kzalloc(sizeof(mt6331_Codec_Data_Priv), GFP_KERNEL);
    if (!mCodec_data)
    {
        printk(codec->dev, "Failed to allocate private data\n");
        return -ENOMEM;
    }

    snd_soc_codec_set_drvdata(codec, mCodec_data);
    memset((void *)mCodec_data , 0 , sizeof(mt6331_Codec_Data_Priv));


    return 0;
}

static int mt6331_codec_remove(struct snd_soc_codec *codec)
{
    printk(codec->dev, "%s()\n", __func__);
    return 0;
}

static int mt6331_read(struct snd_soc_codec *codec,
                       unsigned int reg)
{
    printk("mt6331_read reg = 0x%x", reg);
    Ana_Get_Reg(reg);
}

static int mt6331_write(struct snd_soc_codec *codec, unsigned int reg,
                        unsigned int value)
{
    printk("mt6331_write reg = 0x%x  value= 0x%x\n", reg, value);
    Ana_Set_Reg(reg , value , 0xffffffff);
}

static int mt6331_Readable_registers()
{
    return 1;
}

static int mt6331_volatile_registers()
{
    return 1;
}

static int mt6331_registers_defaults()
{
    return 0;
}

static struct snd_soc_codec_driver soc_mtk_codec =
{
    .probe    = mt6331_codec_probe,
    .remove = mt6331_codec_remove,

    .read = mt6331_read,
    .write = mt6331_write,

    .readable_register = mt6331_Readable_registers,
    .volatile_register = mt6331_volatile_registers,

    // use add control to replace
    //.controls = mt6331_snd_controls,
    //.num_controls = ARRAY_SIZE(mt6331_snd_controls),

    .dapm_widgets = mt6331_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(mt6331_dapm_widgets),
    .dapm_routes = mtk_audio_map,
    .num_dapm_routes = ARRAY_SIZE(mtk_audio_map),

};

static int mtk_mt6331_codec_dev_probe(struct platform_device *pdev)
{
    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s.%d", "msm-stub-codec", 1);
    }

    dev_err(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));

    return snd_soc_register_codec(&pdev->dev,
                                  &soc_mtk_codec, mtk_6323_dai_codecs, ARRAY_SIZE(mtk_6323_dai_codecs));
}

static int mtk_mt6331_codec_dev_remove(struct platform_device *pdev)
{
    printk("%s:\n", __func__);

    snd_soc_unregister_codec(&pdev->dev);
    return 0;

}

static struct platform_driver mtk_codec_6331_driver =
{
    .driver = {
        .name = MT_SOC_CODEC_NAME,
        .owner = THIS_MODULE,
    },
    .probe  = mtk_mt6331_codec_dev_probe,
    .remove = mtk_mt6331_codec_dev_remove,
};

static struct platform_device *soc_mtk_codec6331_dev;

static int __init mtk_mt6331_codec_init(void)
{
    printk("%s:\n", __func__);
    int ret;
    soc_mtk_codec6331_dev = platform_device_alloc(MT_SOC_CODEC_NAME, -1);
    if (!soc_mtk_codec6331_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtk_codec6331_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtk_codec6331_dev);
        return ret;
    }

    return platform_driver_register(&mtk_codec_6331_driver);
}
module_init(mtk_mt6331_codec_init);

static void __exit mtk_mt6331_codec_exit(void)
{
    printk("%s:\n", __func__);

    platform_driver_unregister(&mtk_codec_6331_driver);
}

module_exit(mtk_mt6331_codec_exit);

/* Module information */
MODULE_DESCRIPTION("MTK  codec driver");
MODULE_LICENSE("GPL v2");

