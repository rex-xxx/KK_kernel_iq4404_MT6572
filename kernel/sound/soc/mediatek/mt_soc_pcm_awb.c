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
 *   mtk_pcm_awb.c
 *
 * Project:
 * --------
 *   MT6582  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audioawb capture
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <asm/mach-types.h>

/*
#define DEBUG_AUDDRV
#ifdef DEBUG_AUDDRV
#define printk(format, args...) printk(format, ##args )
#else
#define printk(format, args...)
#endif
*/

struct snd_dma_buffer *Awb_Capture_dma_buf  = NULL;
static struct pcm_afe_info *prtdtemp = NULL;

//information about
AFE_MEM_CONTROL_T  AWB_Control_context;
static bool InterruptTrigger = 1;
static DEFINE_SPINLOCK(auddrv_AWBInCtl_lock);

/*
 *    function implementation
 */
static void StartAudioAWBHardware(struct snd_pcm_substream *substream);
static void StopAudioAWBHardware(struct snd_pcm_substream *substream);
void Auddrv_AWB_Interrupt_Handler(void);
static int mtk_awb_probe(struct platform_device *pdev);
static int mtk_awb_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_awb_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_awb_probe(struct snd_soc_platform *platform);

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
#define USE_PERIODS_MIN     1024
#define USE_PERIODS_MAX     16*1024

static int mtkalsa_awb_constraints(struct snd_pcm_runtime *runtime)
{
    int err;
    err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (err < 0)
    {
        printk("mtkalsa_awb_constraints SNDRV_PCM_HW_PARAM_PERIODS err = %d", err);
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


static struct snd_pcm_hardware mtk_awb_hardware =
{
    .info = (SNDRV_PCM_INFO_INTERLEAVED),
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


static void StopAudioAWBHardware(struct snd_pcm_substream *substream)
{
    printk("StopAudioAWBHardware \n");

    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB,false);

    // here to set interrupt
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);

    // here to turn off digital part
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O05);
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O06);

    EnableAfe(false);
}

static void StartAudioAWBHardware(struct snd_pcm_substream *substream)
{
    printk("StartAudioAWBHardware \n");

    //EnableSideGenHw(Soc_Aud_InterConnectionOutput_O05,Soc_Aud_MemIF_Direction_DIRECTION_OUTPUT,true);

    // here to set interrupt
    SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->period_size);
    SetIrqMcuSampleRate( Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->rate);
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, true);

    SetSampleRate(Soc_Aud_Digital_Block_MEM_AWB, substream->runtime->rate);
    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB,true);

    // here to turn off digital part
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O05);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O06);

    EnableAfe(true);
}


static bool CheckSize(uint32 size)
{
    if ((size) == 0)
    {
        printk("CheckSize size = 0\n");
        return true;
    }
    return false;
}

void Auddrv_AWB_Interrupt_Handler(void)  // irq1 ISR handler
{
    unsigned long flags;
    kal_uint32 Afe_Dac_Con0 = Afe_Get_Reg(AFE_DAC_CON0);
    AFE_MEM_CONTROL_T *Mem_Block = &AWB_Control_context;
    AFE_BLOCK_T *Awb_Block = &(AWB_Control_context.rBlock);

    printk("Auddrv_AWB_Interrupt_Handler \n ");
    kal_uint32 HW_Cur_ReadIdx = 0;
    kal_int32 Hw_Get_bytes = 0;
    AFE_BLOCK_T  *mBlock = NULL;

    if (Mem_Block == NULL)
    {
        return;
    }
    mBlock = &Mem_Block->rBlock;
    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_AWB_CUR);
    printk("Auddrv_AWB_Interrupt_Handler HW_Cur_ReadIdx = 0x%x\n ", HW_Cur_ReadIdx);

    if (CheckSize(HW_Cur_ReadIdx))
    {
        return;
    }
    if (mBlock->pucVirtBufAddr  == NULL)
    {
        return;
    }

    // HW already fill in
    Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;
    if (Hw_Get_bytes < 0)
    {
        Hw_Get_bytes += mBlock->u4BufferSize;
    }

    printk("Auddrv_Handle_Mem_context Hw_Get_bytes:%x, HW_Cur_ReadIdx:%x, u4DMAReadIdx:%x, u4WriteIdx:0x%x, pucPhysBufAddr:%x Mem_Block->MemIfNum = %d \n",
           Hw_Get_bytes, HW_Cur_ReadIdx, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->pucPhysBufAddr, Mem_Block->MemIfNum);

    mBlock->u4WriteIdx  += Hw_Get_bytes;
    mBlock->u4WriteIdx  %= mBlock->u4BufferSize;
    mBlock->u4DataRemained += Hw_Get_bytes;

    // buffer overflow
    if (mBlock->u4DataRemained > mBlock->u4BufferSize)
    {
        printk("Auddrv_Handle_Mem_context buffer overflow u4DMAReadIdx:%x, u4WriteIdx:%x, u4DataRemained:%x, u4BufferSize:%x \n",
               mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4DataRemained, mBlock->u4BufferSize);
        mBlock->u4DataRemained = mBlock->u4BufferSize / 2;
        mBlock->u4DMAReadIdx = mBlock->u4WriteIdx - mBlock->u4BufferSize / 2;
        if (mBlock->u4DMAReadIdx < 0)
        {
            mBlock->u4DMAReadIdx += mBlock->u4BufferSize;
        }
    }
    InterruptTrigger = 1;
    snd_pcm_period_elapsed(prtdtemp->substream);

}


static int mtk_awb_pcm_prepare(struct snd_pcm_substream *substream)
{
    printk("mtk_awb_pcm_prepare substream->rate = %d  substream->channels = %d \n", substream->runtime->rate, substream->runtime->channels);
    return 0;
}

static int mtk_awb_alsa_stop(struct snd_pcm_substream *substream)
{
    printk("mtk_awb_alsa_stop \n");
    StopAudioAWBHardware(substream);
    AFE_BLOCK_T *Awb_Block = &(AWB_Control_context.rBlock);
    Awb_Block->u4DMAReadIdx  = 0;
    Awb_Block->u4WriteIdx  = 0;
    Awb_Block->u4DataRemained = 0;
    return 0;
}

static kal_int32 Previous_Hw_cur = 0;
static snd_pcm_uframes_t mtk_awb_pcm_pointer(struct snd_pcm_substream *substream)
{
    printk("mtk_awb_pcm_pointer \n");
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct dummy_hrtimer_pcm *dpcm = runtime->private_data;
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    AFE_BLOCK_T *Awb_Block = &(AWB_Control_context.rBlock);
    if (InterruptTrigger == 1)
    {
        HW_Cur_ReadIdx = Afe_Get_Reg(AFE_AWB_CUR);
        if (HW_Cur_ReadIdx == 0)
        {
            printk("[Auddrv] mtk_awb_pcm_pointer  HW_Cur_ReadIdx ==0 \n");
            HW_Cur_ReadIdx = Awb_Block->pucPhysBufAddr;
        }
        HW_memory_index = (HW_Cur_ReadIdx - Awb_Block->pucPhysBufAddr);
        Previous_Hw_cur = HW_memory_index;
        printk("[Auddrv] mtk_awb_pcm_pointer =0x%x HW_memory_index = 0x%x\n", HW_Cur_ReadIdx, HW_memory_index);
        InterruptTrigger = 0;
        return (HW_memory_index >> 2);
    }
    return (Previous_Hw_cur >> 2);
}


static void SetAWBBuffer(struct snd_pcm_substream *substream,
                         struct snd_pcm_hw_params *hw_params)
{
    printk("SetAWBBuffer\n");
    struct snd_pcm_runtime *runtime = substream->runtime;
    AFE_BLOCK_T *pblock = &AWB_Control_context.rBlock;
    pblock->pucPhysBufAddr =  runtime->dma_addr;
    pblock->pucVirtBufAddr =  runtime->dma_area;
    pblock->u4BufferSize = runtime->dma_bytes;
    pblock->u4SampleNumMask = 0x001f;  // 32 byte align
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;
    printk("dma_bytes = %d dma_area = %p dma_addr = 0x%x\n",
           pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
    // set sram address top hardware
    Afe_Set_Reg(AFE_AWB_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_AWB_END  , pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);

}

static int mtk_awb_pcm_hw_params(struct snd_pcm_substream *substream,
                                     struct snd_pcm_hw_params *hw_params)
{
    printk("mtk_awb_pcm_hw_params \n");
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
    struct snd_dma_buffer *runtimedma_buf = &substream->runtime->dma_buffer_p;
    int ret = 0;

    dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
    dma_buf->dev.dev = substream->pcm->card->dev;
    dma_buf->private_data = NULL;

    /*
    dma_buf->area = dma_alloc_coherent(substream->pcm->card->dev,
                                       runtime->hw.buffer_bytes_max,
                                       &dma_buf->addr, GFP_KERNEL);
    if (!dma_buf->area)
    {
        printk("%s:mtk_awb_pcm_hw_params failed\n", __func__);
        return -ENOMEM;
    }*/

    if (Awb_Capture_dma_buf->area)
    {
        printk("mtk_awb_pcm_hw_params Awb_Capture_dma_buf->area\n");
        runtime->dma_bytes = Awb_Capture_dma_buf->bytes;
        runtime->dma_area = Awb_Capture_dma_buf->area;
        runtime->dma_addr = Awb_Capture_dma_buf->addr;
    }
    else
    {
        printk("mtk_awb_pcm_hw_params snd_pcm_lib_malloc_pages\n");
        ret =  snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
    }
    printk("mtk_awb_pcm_hw_params dma_bytes = %d dma_area = %p dma_addr = 0x%x\n",
           runtime->dma_bytes, runtime->dma_area, runtime->dma_addr);

    printk("runtime->hw.buffer_bytes_max = 0x%x \n", runtime->hw.buffer_bytes_max);
    SetAWBBuffer(substream, hw_params);

    printk("dma_bytes = %d dma_area = %p dma_addr = 0x%x\n",
           substream->runtime->dma_bytes, substream->runtime->dma_area, substream->runtime->dma_addr);
    return ret;
}

static int mtk_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
    printk("mtk_capture_pcm_hw_free \n");
    if (Awb_Capture_dma_buf->area)
    {
        return 0;
    }
    else
    {
        return snd_pcm_lib_free_pages(substream);
    }
}

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] =
{
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(supported_sample_rates),
    .list = supported_sample_rates,
};

static int mtk_awb_pcm_open(struct snd_pcm_substream *substream)
{
    printk("mtk_awb_pcm_open\n");

    struct snd_pcm_runtime *runtime = substream->runtime;
    struct pcm_afe_info *prtd = NULL;

    prtd = kzalloc(sizeof(struct pcm_afe_info), GFP_KERNEL);
    if (prtd == NULL)
    {
        printk("Failed to allocate memory for msm_audio\n");
        return -ENOMEM;
    }
    else
    {
        printk("prtd %x\n", (unsigned int)prtd);
    }
    prtd->substream = substream;
    runtime->private_data = prtd;

    prtd = kzalloc(sizeof(struct pcm_afe_info), GFP_KERNEL);

    if (prtd == NULL)
    {
        printk("Failed to allocate memory for msm_audio\n");
        return -ENOMEM;
    }
    else
    {
        printk("prtd %x\n", (unsigned int)prtd);
    }

    int err = 0;
    int ret = 0;
    prtd->substream = substream;
    prtdtemp = prtd;
    runtime->hw = mtk_awb_hardware;
    memcpy((void *)(&(runtime->hw)), (void *)&mtk_awb_hardware , sizeof(struct snd_pcm_hardware));
    runtime->private_data = prtd;

    printk("runtime->hw->rates = 0x%x mtk_awb_hardware = = 0x%x\n ", runtime->hw.rates, &mtk_awb_hardware);

    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &constraints_sample_rates);
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);


    if (ret < 0)
    {
        printk("snd_pcm_hw_constraint_integer failed\n");
    }

    // here open audio clocks
    AudDrv_Clk_On();
    AudDrv_ANA_Clk_On();
    if (err < 0)
    {
        return err;
    }

    //print for hw pcm information
    printk("mtk_awb_pcm_open runtime rate = %d channels = %d \n", runtime->rate, runtime->channels);
    printk("mtk_awb_pcm_open substream->pcm->device  %d \n", substream->pcm->device);

    runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        printk("SNDRV_PCM_STREAM_CAPTURE \n");
    }
    else
    {
        return -1;
    }

    if (err < 0)
    {
        printk("mtk_awb_pcm_close\n");
        mtk_awb_pcm_close(substream);
        return err;
    }
    printk("mtk_awb_pcm_open return\n");
    return 0;
}

static int mtk_awb_pcm_close(struct snd_pcm_substream *substream)
{
    mtk_awb_alsa_stop(substream);
    return 0;
}

static int mtk_awb_alsa_start(struct snd_pcm_substream *substream)
{
    printk("mtk_awb_alsa_start \n");
    StartAudioAWBHardware(substream);
    return 0;
}

static int mtk_capture_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    printk("mtk_capture_pcm_trigger cmd = %d\n", cmd);

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
            return mtk_awb_alsa_start(substream);
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            return mtk_awb_alsa_stop(substream);
    }
    return -EINVAL;
}

static bool CheckNullPointer(void *pointer)
{
    if (pointer == NULL)
    {
        printk("CheckNullPointer pointer = NULL");
        return true;
    }
    return false;
}

static int mtk_awb_pcm_copy(struct snd_pcm_substream *substream,
                                int channel, snd_pcm_uframes_t pos,
                                void __user *dst, snd_pcm_uframes_t count)
{
    printk("mtk_awb_pcm_copy pos = %d count = %d\n ", pos, count);
    count = count << 2;

    AFE_MEM_CONTROL_T *pAWB_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Awb_Block = NULL;
    char *Read_Data_Ptr = (char *)dst;
    ssize_t ret , DMA_Read_Ptr = 0 , read_size = 0, read_count = 0;
    unsigned long flags;

    // check which memif nned to be write
    pAWB_MEM_ConTrol = &AWB_Control_context;
    Awb_Block = &(pAWB_MEM_ConTrol->rBlock);

    if (pAWB_MEM_ConTrol == NULL)
    {
        printk("cannot find MEM control !!!!!!!\n");
        msleep(50);
        return 0;
    }

    if (Awb_Block->u4BufferSize <= 0)
    {
        msleep(50);
        return 0;
    }

    if (CheckNullPointer((void *)Awb_Block->pucVirtBufAddr))
    {
        printk("CheckNullPointer  pucVirtBufAddr = %p\n", Awb_Block->pucVirtBufAddr);
        return 0;
    }

    spin_lock_irqsave(&auddrv_AWBInCtl_lock, flags);
    if (Awb_Block->u4DataRemained >  Awb_Block->u4BufferSize)
    {
        printk("AudDrv_MEMIF_Read u4DataRemained=%x > u4BufferSize=%x" , Awb_Block->u4DataRemained, Awb_Block->u4BufferSize);
        Awb_Block->u4DataRemained = 0;
        Awb_Block->u4DMAReadIdx   = Awb_Block->u4WriteIdx;
    }
    if (count >  Awb_Block->u4DataRemained)
    {
        read_size = Awb_Block->u4DataRemained;
    }
    else
    {
        read_size = count;
    }

    DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
    spin_unlock_irqrestore(&auddrv_AWBInCtl_lock, flags);
    /*
    printk("AudDrv_MEMIF_Read finish0, read_count:%x, read_size:%x, u4DataRemained:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x \r\n",
           read_count, read_size, Awb_Block->u4DataRemained, Awb_Block->u4DMAReadIdx, Awb_Block->u4WriteIdx);*/

    if (DMA_Read_Ptr + read_size < Awb_Block->u4BufferSize)
    {
        if (DMA_Read_Ptr != Awb_Block->u4DMAReadIdx)
        {
            printk("AudDrv_MEMIF_Read 1, read_size:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x \r\n",
                   read_size, Awb_Block->u4DataRemained, DMA_Read_Ptr, Awb_Block->u4DMAReadIdx);
        }

        if (copy_to_user((void __user *)Read_Data_Ptr, (Awb_Block->pucVirtBufAddr + DMA_Read_Ptr), read_size))
        {

            printk("AudDrv_MEMIF_Read Fail 1 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:0x%x,read_size:%x", Read_Data_Ptr, Awb_Block->pucVirtBufAddr, Awb_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return 0;
        }

        read_count += read_size;
        spin_lock(&auddrv_AWBInCtl_lock);
        Awb_Block->u4DataRemained -= read_size;
        Awb_Block->u4DMAReadIdx += read_size;
        Awb_Block->u4DMAReadIdx %= Awb_Block->u4BufferSize;
        DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_AWBInCtl_lock);

        Read_Data_Ptr += read_size;
        count -= read_size;
        /*
        printk("AudDrv_MEMIF_Read finish1, copy size:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x, u4DataRemained:%x \r\n",
               read_size, Awb_Block->u4DMAReadIdx, Awb_Block->u4WriteIdx, Awb_Block->u4DataRemained);*/
    }

    else
    {
        uint32 size_1 = Awb_Block->u4BufferSize - DMA_Read_Ptr;
        uint32 size_2 = read_size - size_1;

        if (DMA_Read_Ptr != Awb_Block->u4DMAReadIdx)
        {

            printk("AudDrv_MEMIF_Read 2, read_size1:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x \r\n",
                   size_1, Awb_Block->u4DataRemained, DMA_Read_Ptr, Awb_Block->u4DMAReadIdx);
        }
        if (copy_to_user((void __user *)Read_Data_Ptr, (Awb_Block->pucVirtBufAddr + DMA_Read_Ptr), size_1))
        {

            printk("AudDrv_MEMIF_Read Fail 2 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:0x%x,read_size:%x",
                   Read_Data_Ptr, Awb_Block->pucVirtBufAddr, Awb_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return 0;
        }

        read_count += size_1;
        spin_lock(&auddrv_AWBInCtl_lock);
        Awb_Block->u4DataRemained -= size_1;
        Awb_Block->u4DMAReadIdx += size_1;
        Awb_Block->u4DMAReadIdx %= Awb_Block->u4BufferSize;
        DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_AWBInCtl_lock);
        /*
        printk("AudDrv_MEMIF_Read finish2, copy size_1:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x, u4DataRemained:%x \r\n",
               size_1, Awb_Block->u4DMAReadIdx, Awb_Block->u4WriteIdx, Awb_Block->u4DataRemained);*/

        if (DMA_Read_Ptr != Awb_Block->u4DMAReadIdx)
        {

            printk("AudDrv_AWB_Read 3, read_size2:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x \r\n",
                   size_2, Awb_Block->u4DataRemained, DMA_Read_Ptr, Awb_Block->u4DMAReadIdx);
        }
        if (copy_to_user((void __user *)(Read_Data_Ptr + size_1), (Awb_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2))
        {

            printk("AudDrv_MEMIF_Read Fail 3 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x , DMA_Read_Ptr:0x%x, read_size:%x", Read_Data_Ptr, Awb_Block->pucVirtBufAddr, Awb_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return read_count << 2;
        }

        read_count += size_2;
        spin_lock(&auddrv_AWBInCtl_lock);
        Awb_Block->u4DataRemained -= size_2;
        Awb_Block->u4DMAReadIdx += size_2;
        DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_AWBInCtl_lock);

        count -= read_size;
        Read_Data_Ptr += read_size;
        /*
        printk("AudDrv_MEMIF_Read finish3, copy size_2:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x u4DataRemained:%x \r\n",
               size_2, Awb_Block->u4DMAReadIdx, Awb_Block->u4WriteIdx, Awb_Block->u4DataRemained);*/
    }

    return read_count >> 2;
}

static int mtk_capture_pcm_silence(struct snd_pcm_substream *substream,
                                   int channel, snd_pcm_uframes_t pos,
                                   snd_pcm_uframes_t count)
{
    printk("dummy_pcm_silence \n");
    return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_capture_pcm_page(struct snd_pcm_substream *substream,
                                         unsigned long offset)
{
    printk("dummy_pcm_page \n");
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}


static struct snd_pcm_ops mtk_awb_ops =
{
    .open =     mtk_awb_pcm_open,
    .close =    mtk_awb_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_awb_pcm_hw_params,
    .hw_free =  mtk_capture_pcm_hw_free,
    .prepare =  mtk_awb_pcm_prepare,
    .trigger =  mtk_capture_pcm_trigger,
    .pointer =  mtk_awb_pcm_pointer,
    .copy =     mtk_awb_pcm_copy,
    .silence =  mtk_capture_pcm_silence,
    .page =     mtk_capture_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform =
{
    .ops        = &mtk_awb_ops,
    .pcm_new    = mtk_asoc_awb_pcm_new,
    .probe      = mtk_afe_awb_probe,
};

static int mtk_awb_probe(struct platform_device *pdev)
{
    printk("mtk_awb_probe\n");
    int ret = 0;
    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_AWB_PCM);
    }

    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_soc_platform);
}
static u64 awb_pcm_dmamask = DMA_BIT_MASK(32);

static int mtk_asoc_awb_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_card *card = rtd->card->snd_card;
    struct snd_pcm *pcm = rtd->pcm;
    int ret = 0;
    if (!card->dev->dma_mask)
    {
        card->dev->dma_mask = &awb_pcm_dmamask;
    }
    if (!card->dev->coherent_dma_mask)
    {
        card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
    }

    printk("mtk_asoc_awb_pcm_new \n");
    return ret;
}


static int mtk_afe_awb_probe(struct snd_soc_platform *platform)
{
    printk("mtk_afe_awb_probe\n");
    Awb_Capture_dma_buf = kmalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
    memset((void *)Awb_Capture_dma_buf, 0, sizeof(struct snd_dma_buffer));
    printk("mtk_afe_awb_probe dma_alloc_coherent\n");

    Awb_Capture_dma_buf->area = dma_alloc_coherent(0,
                                               MAX_BUFFER_SIZE,
                                               &Awb_Capture_dma_buf->addr, GFP_KERNEL);

    if (Awb_Capture_dma_buf->area)
    {
        Awb_Capture_dma_buf->bytes = MAX_BUFFER_SIZE;
    }
    return 0;
}


static int mtk_awb_remove(struct platform_device *pdev)
{
    pr_debug("%s\n", __func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static struct platform_driver mtk_awb_capture_driver =
{
    .driver = {
        .name = MT_SOC_AWB_PCM,
        .owner = THIS_MODULE,
    },
    .probe = mtk_awb_probe,
    .remove = mtk_awb_remove,
};

static struct platform_device *soc_mtkafe_capture_dev;

static int __init mtk_soc_awb_platform_init(void)
{
    printk("%s\n", __func__);
    int ret = 0;
    soc_mtkafe_capture_dev = platform_device_alloc(MT_SOC_AWB_PCM, -1);
    if (!soc_mtkafe_capture_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtkafe_capture_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtkafe_capture_dev);
        return ret;
    }

    ret = platform_driver_register(&mtk_awb_capture_driver);
    return ret;
}
module_init(mtk_soc_awb_platform_init);

static void __exit mtk_soc_awb_platform_exit(void)
{

    printk("%s\n", __func__);
    platform_driver_unregister(&mtk_awb_capture_driver);
}

module_exit(mtk_soc_awb_platform_exit);

EXPORT_SYMBOL(Auddrv_AWB_Interrupt_Handler);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");


