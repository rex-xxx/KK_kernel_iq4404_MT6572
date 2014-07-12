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
 *   mt_soc_pcm_afe.c
 *
 * Project:
 * --------
 *   ROME  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio dl1 data1 playback
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
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
#include <mach/mtk_wcn_cmb_stub.h>
#endif
*/

// MTK add for dummy drivers
#define AUDIO_MEMORY_SRAM
#define AUDIO_MEM_IOREMAP

// here is temp address for ioremap audio hardware regisster
extern void *AFE_BASE_ADDRESS ;
extern void *AFE_SRAM_ADDRESS;

static struct pcm_afe_info *prtdtemp = NULL;

//information about
AFE_MEM_CONTROL_T    AFE_dL1_Control_context;
static bool InterruptTrigger = 1;
static bool fake_buffer = 1;

static DEFINE_SPINLOCK(auddrv_lock);
static DEFINE_SPINLOCK(auddrv_SphCtlState_lock);
static DEFINE_SPINLOCK(auddrv_DLCtl_lock);

/*
 *    function implementation
 */

int AudDrv_Allocate_DL1_Buffer(kal_uint32 Afe_Buf_Length);
void StartAudioPcmHardware();
void StopAudioPcmHardware();
void Auddrv_DL_Interrupt_Handler(void);
static irqreturn_t ALSA_Drv_IRQ_handler(int irq, void *dev_id);
static int mtk_afe_probe(struct platform_device *pdev);
static int mtk_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_afe_probe(struct snd_soc_platform *platform);

extern void Sound_Speaker_Turnon();
extern void Sound_Speaker_Turnoff();

#define MAX_PCM_DEVICES     4
#define MAX_PCM_SUBSTREAMS  128
#define MAX_MIDI_DEVICES

/* defaults */
#define MAX_BUFFER_SIZE     (24*1024)
#define MIN_PERIOD_SIZE       64
#define MAX_PERIOD_SIZE     MAX_BUFFER_SIZE
#define USE_FORMATS         (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE        SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        192000
#define USE_CHANNELS_MIN     1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     8192

static int mtkalsa_playback_constraints(struct snd_pcm_runtime *runtime)
{
    int err;
    err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (err < 0)
    {
        PRINTK_AUDDRV("mtkalsa_playback_constraints SNDRV_PCM_HW_PARAM_PERIODS err = %d", err);
        return err;
    }
    err = snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 1, UINT_MAX);
    if (err < 0)
    {
        PRINTK_AUDDRV("snd_pcm_hw_constraint_minmax SNDRV_PCM_HW_PARAM_BUFFER_BYTES err = %d", err);
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


/*****************************************************************************
 * FUNCTION
 *  AudDrv_Allocate_DL1_Buffer / AudDrv_Free_DL1_Buffer
 *
 * DESCRIPTION
 *  allocate DL1 Buffer
 *

******************************************************************************/
int AudDrv_Allocate_DL1_Buffer(kal_uint32 Afe_Buf_Length)
{
#ifdef AUDIO_MEMORY_SRAM
    kal_uint32 u4PhyAddr = 0;
#endif
    AFE_BLOCK_T *pblock = &AFE_dL1_Control_context.rBlock;
    pblock->u4BufferSize = Afe_Buf_Length;

    PRINTK_AUDDRV("AudDrv_Allocate_DL1_Buffer length = %d\n", Afe_Buf_Length);
    if (Afe_Buf_Length > AFE_INTERNAL_SRAM_SIZE)
    {
        PRINTK_AUDDRV("Afe_Buf_Length > AUDDRV_DL1_MAX_BUFFER_LENGTH \n");
        return -1;
    }

    // allocate memory
    if (pblock->pucPhysBufAddr == 0)
    {
#ifdef AUDIO_MEMORY_SRAM
        // todo , there should be a sram manager to allocate memory for low  power.powervr_device
        u4PhyAddr = AFE_INTERNAL_SRAM_PHY_BASE;
        pblock->pucPhysBufAddr = u4PhyAddr;

#ifdef AUDIO_MEM_IOREMAP
        PRINTK_AUDDRV("AudDrv_Allocate_DL1_Buffer length AUDIO_MEM_IOREMAP  = %d\n",
                      Afe_Buf_Length);
        pblock->pucVirtBufAddr = (kal_uint8 *)AFE_SRAM_ADDRESS;
#else
        pblock->pucVirtBufAddr = AFE_INTERNAL_SRAM_VIR_BASE;
#endif

#else
        PRINTK_AUDDRV("AudDrv_Allocate_DL1_Buffer use dram");
        pblock->pucVirtBufAddr = dma_alloc_coherent(0, pblock->u4BufferSize, &pblock->pucPhysBufAddr, GFP_KERNEL);
#endif
    }
    PRINTK_AUDDRV("AudDrv_Allocate_DL1_Buffer pucVirtBufAddr = %p\n", pblock->pucVirtBufAddr);

    // check 32 bytes align
    if ((pblock->pucPhysBufAddr & 0x1f) != 0)
    {
        PRINTK_AUDDRV("[Auddrv] AudDrv_Allocate_DL1_Buffer is not aligned (0x%x) \n", pblock->pucPhysBufAddr);
    }

    pblock->u4SampleNumMask = 0x001f;  // 32 byte align
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;

    // set sram address top hardware
    Afe_Set_Reg(AFE_DL1_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_DL1_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1), 0xffffffff);

    return 0;
}

void Auddrv_DL_Interrupt_Handler(void)  // irq1 ISR handler
{
    unsigned long flags;
    kal_int32 Afe_consumed_bytes = 0;
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    AFE_BLOCK_T *Afe_Block = &(AFE_dL1_Control_context.rBlock);

    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
    if (HW_Cur_ReadIdx == 0)
    {
        PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx ==0 \n");
        HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
    }
    HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);

    /*
    PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx=0x%x HW_memory_index = 0x%x Afe_Block->pucPhysBufAddr = 0x%x\n",
                  HW_Cur_ReadIdx, HW_memory_index, Afe_Block->pucPhysBufAddr);*/


    // get hw consume bytes
    if (HW_memory_index > Afe_Block->u4DMAReadIdx)
    {
        Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
    }
    else
    {
        Afe_consumed_bytes = Afe_Block->u4BufferSize + HW_memory_index - Afe_Block->u4DMAReadIdx ;
    }

    if ((Afe_consumed_bytes & 0x1f) != 0)
    {
        //PRINTK_AUDDRV("[Auddrv] DMA address is not aligned 32 bytes \n");
    }
    /*
    PRINTK_AUDDRV("+Auddrv_DL_Interrupt_Handler ReadIdx:%x WriteIdx:%x, DataRemained:%x, Afe_consumed_bytes:%x HW_memory_index = %x \n",
           Afe_Block->u4DMAReadIdx, Afe_Block->u4WriteIdx, Afe_Block->u4DataRemained, Afe_consumed_bytes, HW_memory_index);*/


    if (Afe_Block->u4DataRemained < Afe_consumed_bytes || Afe_Block->u4DataRemained <= 0 || Afe_Block->u4DataRemained  > Afe_Block->u4BufferSize)
    {
        // buffer underflow --> clear  whole buffer
        //memset(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);

        PRINTK_AUDDRV("+DL_Handling underflow ReadIdx:%x WriteIdx:%x, DataRemained:%x, Afe_consumed_bytes:%x HW_memory_index = 0x%x\n",
                      Afe_Block->u4DMAReadIdx, Afe_Block->u4WriteIdx, Afe_Block->u4DataRemained, Afe_consumed_bytes, HW_memory_index);
        Afe_Block->u4DMAReadIdx  = HW_memory_index;
        Afe_Block->u4WriteIdx  = Afe_Block->u4DMAReadIdx;
        Afe_Block->u4DataRemained = Afe_Block->u4BufferSize;
        /*
        PRINTK_AUDDRV("-DL_Handling underflow ReadIdx:%x WriteIdx:%x, DataRemained:%x, Afe_consumed_bytes %x \n",
                      Afe_Block->u4DMAReadIdx, Afe_Block->u4WriteIdx, Afe_Block->u4DataRemained, Afe_consumed_bytes);*/
    }
    else
    {

        PRINTK_AUDDRV("+DL_Handling normal ReadIdx:%x ,DataRemained:%x, WriteIdx:%x \n",
               Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained, Afe_Block->u4WriteIdx);
        Afe_Block->u4DataRemained -= Afe_consumed_bytes;
        Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
        Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
        /*
        PRINTK_AUDDRV("-DL_Handling normal ReadIdx:%x ,DataRemained:%x, WriteIdx:%x \n",
               Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained, Afe_Block->u4WriteIdx);*/
    }
    InterruptTrigger = 1 ;
    snd_pcm_period_elapsed(prtdtemp->substream);
}

static int mtk_pcm_dl1_stop(struct snd_pcm_substream *substream)
{
    PRINTK_AUDDRV("mtk_pcm_dl1_stop \n");

    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, false);

    // here to turn off digital part
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I05, Soc_Aud_InterConnectionOutput_O03);
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I06, Soc_Aud_InterConnectionOutput_O04);
    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, false);

    if (GetI2SDacEnable() == false)
    {
        SetI2SDacEnable(false);
    }

    EnableAfe(false);

    // clean audio hardware buffer
    AFE_BLOCK_T *Afe_Block = &(AFE_dL1_Control_context.rBlock);
    //memset(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);

    Afe_Block->u4DMAReadIdx  = 0;
    Afe_Block->u4WriteIdx  = 0;
    Afe_Block->u4DataRemained = 0;

    return 0;
}

static kal_int32 Previous_Hw_cur = 0;
static snd_pcm_uframes_t mtk_pcm_pointer(struct snd_pcm_substream *substream)
{
    //PRINTK_AUDDRV("mtk_pcm_pointer \n");
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct dummy_hrtimer_pcm *dpcm = runtime->private_data;
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    AFE_BLOCK_T *Afe_Block = &(AFE_dL1_Control_context.rBlock);
    if (InterruptTrigger == 1)
    {
        HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
        if (HW_Cur_ReadIdx == 0)
        {
            PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx ==0 \n");
            HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
        }
        HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
        Previous_Hw_cur = HW_memory_index;
        //PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx =0x%x HW_memory_index = 0x%x pointer return = 0x%x \n", HW_Cur_ReadIdx, HW_memory_index,(HW_memory_index >> 2));
        InterruptTrigger = 0;
        return (HW_memory_index >> 2);
    }
    return (Previous_Hw_cur >> 2);
}


static int mtk_pcm_hw_params(struct snd_pcm_substream *substream,
                             struct snd_pcm_hw_params *hw_params)
{
    PRINTK_AUDDRV("mtk_pcm_hw_params \n");
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
    int ret = 0;
    if (fake_buffer)
    {
        /* runtime->dma_bytes has to be set manually to allow mmap */
        substream->runtime->dma_bytes = params_buffer_bytes(hw_params);

        // here to allcoate sram to hardware ---------------------------
        AudDrv_Allocate_DL1_Buffer(AFE_INTERNAL_SRAM_SIZE);
        substream->runtime->dma_bytes = AFE_INTERNAL_SRAM_SIZE;
        substream->runtime->dma_area = (unsigned char *)AFE_SRAM_ADDRESS;
        substream->runtime->dma_addr = AFE_INTERNAL_SRAM_PHY_BASE;

        // -------------------------------------------------------
        PRINTK_AUDDRV("1 dma_bytes = %d dma_area = %p dma_addr = 0x%x\n",
               substream->runtime->dma_bytes, substream->runtime->dma_area, substream->runtime->dma_addr);
        return 0;
    }
    else
    {
        dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
        dma_buf->dev.dev = substream->pcm->card->dev;
        dma_buf->private_data = NULL;
        ret =  snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
    }
    PRINTK_AUDDRV("2 dma_bytes = %d dma_area = %p dma_addr = 0x%x\n",
           substream->runtime->dma_bytes, substream->runtime->dma_area, substream->runtime->dma_addr);
    return ret;
}

static int mtk_pcm_hw_free(struct snd_pcm_substream *substream)
{
    PRINTK_AUDDRV("mtk_pcm_hw_free \n");
    if (fake_buffer)
    {
        return 0;
    }
    return snd_pcm_lib_free_pages(substream);
}

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] =
{
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,88200,96000,192000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(supported_sample_rates),
    .list = supported_sample_rates,
    .mask = 0,
};

static int mtk_pcm_open(struct snd_pcm_substream *substream)
{
    PRINTK_AUDDRV("mtk_pcm_open\n");

    struct snd_pcm_runtime *runtime = substream->runtime;
    struct pcm_afe_info *prtd = NULL;

    prtd = kzalloc(sizeof(struct pcm_afe_info), GFP_KERNEL);
    if (prtd == NULL)
    {
        PRINTK_AUDDRV("Failed to allocate memory for msm_audio\n");
        return -ENOMEM;
    }
    else
    {
        PRINTK_AUDDRV("prtd %x\n", (unsigned int)prtd);
    }
    prtd->substream = substream;
    prtdtemp = prtd;
    runtime->private_data = prtd;

    prtd = kzalloc(sizeof(struct pcm_afe_info), GFP_KERNEL);

    if (prtd == NULL)
    {
        PRINTK_AUDDRV("Failed to allocate memory for msm_audio\n");
        return -ENOMEM;
    }
    else
    {
        PRINTK_AUDDRV("prtd %x\n", (unsigned int)prtd);
    }

    int err = 0;
    int ret = 0;
    prtd->substream = substream;
    runtime->hw = mtk_pcm_hardware;
    memcpy((void *)(&(runtime->hw)), (void *)&mtk_pcm_hardware , sizeof(struct snd_pcm_hardware));
    runtime->private_data = prtd;

    PRINTK_AUDDRV("runtime->hw->rates= 0x%x mtk_pcm_hardware = = 0x%x\n ", runtime->hw.rates, &mtk_pcm_hardware);
    PRINTK_AUDDRV("runtime->hw = 0x%x\n ", &runtime->hw);

    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &constraints_sample_rates);
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

    if (ret < 0)
    {
        PRINTK_AUDDRV("snd_pcm_hw_constraint_integer failed\n");
    }

    // temp solution for ignore clock setting
    AudDrv_Clk_AllOn();
    if (err < 0)
    {
        return err;
    }

    //print for hw pcm information
    PRINTK_AUDDRV("mtk_pcm_open runtime rate = %d channels = %d substream->pcm->device = %d\n",
        runtime->rate, runtime->channels,substream->pcm->device);
    runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        PRINTK_AUDDRV("SNDRV_PCM_STREAM_PLAYBACK mtkalsa_playback_constraints\n");
    }
    else
    {

    }

    if (err < 0)
    {
        PRINTK_AUDDRV("mtk_pcm_close\n");
        mtk_pcm_close(substream);
        return err;
    }
    PRINTK_AUDDRV("mtk_pcm_open return\n");
    return 0;
}

static int mtk_pcm_close(struct snd_pcm_substream *substream)
{
    PRINTK_AUDDRV("%s \n",__func__);
    return 0;
}

static int mtk_pcm_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtimeStream = substream->runtime;
    AFE_BLOCK_T *Afe_Block = &(AFE_dL1_Control_context.rBlock);
    PRINTK_AUDDRV("mtk_alsa_prepare rate = %d  channels = %d period_size = %d\n",
           runtimeStream->rate, runtimeStream->channels, runtimeStream->period_size);
    return 0;
}

static int mtk_pcm_dl1_start(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    PRINTK_AUDDRV("mtk_pcm_dl1_start period = %d \n",runtime->period_size);

    if (runtime->format == SNDRV_PCM_FMTBIT_S32||runtime->format == SNDRV_PCM_FMTBIT_U32)
    {
        SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1, AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
        SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2, AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
        SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O03);
        SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O04);
    }
    else
    {
        SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1, AFE_WLEN_16_BIT);
        SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1, AFE_WLEN_16_BIT);
        SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O03);
        SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O04);
    }

    // here start digital part
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I05, Soc_Aud_InterConnectionOutput_O03);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I06, Soc_Aud_InterConnectionOutput_O04);

    // set dl1 sample ratelimit_state
    SetSampleRate (Soc_Aud_Digital_Block_MEM_DL1, runtime->rate);
    SetChannels(Soc_Aud_Digital_Block_MEM_DL1, runtime->channels);

    // start I2S DAC out
    SetI2SDacOut(substream->runtime->rate);
    SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
    SetI2SDacEnable(true);
    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, true);

    //EnableSideGenHw(Soc_Aud_InterConnectionInput_I05,Soc_Aud_MemIF_Direction_DIRECTION_INPUT,true);


    // here to set interrupt
    SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, runtime->period_size);
    SetIrqMcuSampleRate( Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, runtime->rate);
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, true);

    EnableAfe(true);
    return 0;
}

static int mtk_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    PRINTK_AUDDRV("mtk_pcm_trigger cmd = %d\n", cmd);

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
            return mtk_pcm_dl1_start(substream);
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            return mtk_pcm_dl1_stop(substream);
    }
    return -EINVAL;
}

static int mtk_pcm_copy(struct snd_pcm_substream *substream,
                        int channel, snd_pcm_uframes_t pos,
                        void __user *dst, snd_pcm_uframes_t count)
{

    //PRINTK_AUDDRV("mtk_pcm_copy pos = %d count = %d\n ", pos, count);
    count = count << 2;

    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Afe_Block = NULL;
    int written_size = count , ret = 0, copy_size = 0, Afe_WriteIdx_tmp;
    unsigned long flags;
    char *data_w_ptr = (char *)dst;

    // check which memif nned to be write
    pAfe_MEM_ConTrol = &AFE_dL1_Control_context;
    Afe_Block = &(pAfe_MEM_ConTrol->rBlock);

    // handle for buffer management
    /*
    PRINTK_AUDDRV("AudDrv_write WriteIdx=%x, ReadIdx=%x, DataRemained=%x \n",
           Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);*/

    if (Afe_Block->u4BufferSize == 0)
    {
        PRINTK_AUDDRV("AudDrv_write: u4BufferSize=0 Error");
        return 0;
    }

    spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
    copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;  //  free space of the buffer
    spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
    if (count <=  copy_size)
    {
        if (copy_size < 0)
        {
            copy_size = 0;
        }
        else
        {
            copy_size = count;
        }
    }

    if (copy_size != 0)
    {
        spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
        Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
        spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

        if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) // copy once
        {
            if (!access_ok(VERIFY_READ, data_w_ptr, copy_size))
            {
                PRINTK_AUDDRV("AudDrv_write 0ptr invalid data_w_ptr=%x, size=%d", (kal_uint32)data_w_ptr, copy_size);
                PRINTK_AUDDRV("AudDrv_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
            }
            else
            {
                /*
                PRINTK_AUDDRV("memcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= 0x%x data_w_ptr = %p copy_size = 0x%x\n",
                       Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr, copy_size);*/
                if (copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr, copy_size))
                {
                    PRINTK_AUDDRV("AudDrv_write Fail copy from user \n");
                    return -1;
                }
            }

            spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
            Afe_Block->u4DataRemained += copy_size;
            Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
            Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
            spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
            data_w_ptr += copy_size;
            count -= copy_size;
            /*
             PRINTK_AUDDRV("AudDrv_write finish1, copy_size:%x, WriteIdx:%x, ReadIdx=%x, DataRemained:%x, count=%x \r\n",
                               copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained, count);
                               */
        }
        else  // copy twice
        {
            kal_uint32 size_1 = 0, size_2 = 0;
            size_1 = Afe_Block->u4BufferSize - Afe_WriteIdx_tmp;
            size_2 = copy_size - size_1;
            if (!access_ok(VERIFY_READ, data_w_ptr, size_1))
            {
                PRINTK_AUDDRV("AudDrv_write 1ptr invalid data_w_ptr=%x, size_1=%d", (kal_uint32)data_w_ptr, size_1);
                PRINTK_AUDDRV("AudDrv_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
            }
            else
            {
                /*
                                PRINTK_AUDDRV("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr = %p size_1 = %x\n",
                                    Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr,size_1);*/
                if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr , size_1)))
                {
                    PRINTK_AUDDRV("AudDrv_write Fail 1 copy from user");
                    return -1;
                }
            }
            spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
            Afe_Block->u4DataRemained += size_1;
            Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
            Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
            Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
            spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

            if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2))
            {
                PRINTK_AUDDRV("AudDrv_write 2ptr invalid data_w_ptr=%x, size_1=%d, size_2=%d", (kal_uint32)data_w_ptr, size_1, size_2);
                PRINTK_AUDDRV("AudDrv_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
            }
            else
            {
                /*
                                PRINTK_AUDDRV("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr+size_1 = %p size_2 = %x\n",
                                    Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr+size_1,size_2);*/
                if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), (data_w_ptr + size_1), size_2)))
                {
                    PRINTK_AUDDRV("AudDrv_write Fail 2  copy from user");
                    return -1;
                }
            }
            spin_lock_irqsave(&auddrv_DLCtl_lock, flags);

            Afe_Block->u4DataRemained += size_2;
            Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
            Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
            spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
            count -= copy_size;
            data_w_ptr += copy_size;
            /*
            PRINTK_AUDDRV("AudDrv_write finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
                   copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);*/
        }
    }
    //PRINTK_AUDDRV("dummy_pcm_copy return \n");
}

static int mtk_pcm_silence(struct snd_pcm_substream *substream,
                           int channel, snd_pcm_uframes_t pos,
                           snd_pcm_uframes_t count)
{
    PRINTK_AUDDRV("%s \n",__func__);
    return 0; /* do nothing */
}

static void *dummy_page[2];

static struct page *mtk_pcm_page(struct snd_pcm_substream *substream,
                                 unsigned long offset)
{
    PRINTK_AUDDRV("%s \n",__func__);
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}

static struct snd_pcm_ops mtk_afe_ops =
{
    .open =     mtk_pcm_open,
    .close =    mtk_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_pcm_hw_params,
    .hw_free =  mtk_pcm_hw_free,
    .prepare =  mtk_pcm_prepare,
    .trigger =  mtk_pcm_trigger,
    .pointer =  mtk_pcm_pointer,
    .copy =     mtk_pcm_copy,
    .silence =  mtk_pcm_silence,
    .page =     mtk_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform =
{
    .ops        = &mtk_afe_ops,
    .pcm_new    = mtk_asoc_pcm_new,
    .probe      = mtk_afe_afe_probe,
};

static int mtk_afe_probe(struct platform_device *pdev)
{
    PRINTK_AUDDRV("%s \n",__func__);

    int ret = 0;
    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_DL1_PCM);
    }

    PRINTK_AUDDRV("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    InitAfeControl();
    AudDrv_Clk_AllOn();
    ret = Register_Aud_Irq(&pdev->dev);
    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_soc_platform);
}

static int mtk_asoc_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    pruntimepcm  = rtd;
    int ret = 0;

    PRINTK_AUDDRV("%s\n", __func__);
    return ret;
}


static int mtk_afe_afe_probe(struct snd_soc_platform *platform)
{
    PRINTK_AUDDRV("mtk_afe_afe_probe\n");
    return 0;
}

static int mtk_afe_remove(struct platform_device *pdev)
{
    PRINTK_AUDDRV("%s \n",__func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static struct platform_driver mtk_afe_driver =
{
    .driver = {
        .name = MT_SOC_DL1_PCM,
        .owner = THIS_MODULE,
    },
    .probe = mtk_afe_probe,
    .remove = mtk_afe_remove,
};

static struct platform_device *soc_mtkafe_dev;

static int __init mtk_soc_platform_init(void)
{
    PRINTK_AUDDRV("%s \n",__func__);
    int ret;
    soc_mtkafe_dev = platform_device_alloc(MT_SOC_DL1_PCM, -1);
    if (!soc_mtkafe_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtkafe_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtkafe_dev);
        return ret;
    }

    ret = platform_driver_register(&mtk_afe_driver);
    return ret;

}
module_init(mtk_soc_platform_init);

static void __exit mtk_soc_platform_exit(void)
{
    PRINTK_AUDDRV("%s \n",__func__);

    platform_driver_unregister(&mtk_afe_driver);
}
module_exit(mtk_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");


