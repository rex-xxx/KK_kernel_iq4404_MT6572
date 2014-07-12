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
 *  mt_sco_afe_control.c
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
#include "mt_soc_digital_type.h"
#include "AudDrv_Def.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_afe_connection.h"

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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <asm/mach-types.h>
static DEFINE_SPINLOCK(afe_control_lock);

/*
 *    global variable control
 */
// address for ioremap audio hardware register
void *AFE_BASE_ADDRESS = 0;
void *AFE_SRAM_ADDRESS = 0;
void *AFE_TOP_ADDRESS = 0;


// static  variable
static AudioMrgIf *mAudioMrg = NULL;
bool AudioMrgStatus = false;
static AudioDigitalDAIBT *AudioDaiBt = NULL;
static bool  AudioDaiBtStatus = false;
static AudioDigtalI2S *AudioAdcI2S = NULL;
static bool AudioAdcI2SStatus = false;
static AudioDigtalI2S *m2ndI2S = NULL;

static AudioIrqMcuMode *mAudioMcuMode[Soc_Aud_IRQ_MCU_MODE_NUM_OF_IRQ_MODE] = {NULL};
static AudioMemIFAttribute *mAudioMEMIF[Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK] = {NULL};
static bool  mAudioInit = false;

extern Auddrv_DL_Interrupt_Handler();
extern Auddrv_UL_Interrupt_Handler();
extern Auddrv_AWB_Interrupt_Handler();


// mutex lock
static DEFINE_MUTEX(afe_control_mutex);
static DEFINE_SPINLOCK(auddrv_irqstatus_lock);

static const uint16_t kSideToneCoefficientTable16k[] =
{
    0x049C, 0x09E8, 0x09E0, 0x089C,
    0xFF54, 0xF488, 0xEAFC, 0xEBAC,
    0xfA40, 0x17AC, 0x3D1C, 0x6028,
    0x7538
};

static const uint16_t kSideToneCoefficientTable32k[] =
{
    0xff58, 0x0063, 0x0086, 0x00bf,
    0x0100, 0x013d, 0x0169, 0x0178,
    0x0160, 0x011c, 0x00aa, 0x0011,
    0xff5d, 0xfea1, 0xfdf6, 0xfd75,
    0xfd39, 0xfd5a, 0xfde8, 0xfeea,
    0x005f, 0x0237, 0x0458, 0x069f,
    0x08e2, 0x0af7, 0x0cb2, 0x0df0,
    0x0e96
};

/*
 *    function implementation
 */
static irqreturn_t AudDrv_IRQ_handler(int irq, void *dev_id);


/*****************************************************************************
 * FUNCTION
 *  InitAfeControl ,ResetAfeControl
 *
 * DESCRIPTION
 *  afe init function
 *
 *****************************************************************************
 */

bool InitAfeControl(void)
{
    printk("InitAfeControl \n");
    int i = 0;
    // first time to init , reg init.
    Auddrv_Reg_map();
    mutex_lock(&afe_control_mutex);
    if (mAudioInit == false)
    {
        mAudioInit = true;
        mAudioMrg = kzalloc(sizeof(AudioMrgIf), GFP_KERNEL);
        AudioDaiBt = kzalloc(sizeof(AudioDigitalDAIBT), GFP_KERNEL);
        AudioAdcI2S =  kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
        m2ndI2S =  kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
        for (i = 0; i < Soc_Aud_IRQ_MCU_MODE_NUM_OF_IRQ_MODE ; i ++)
        {
            mAudioMcuMode[i] = kzalloc(sizeof(AudioIrqMcuMode)  , GFP_KERNEL);
        }
        for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK ; i ++)
        {
            mAudioMEMIF[i] = kzalloc(sizeof(AudioMemIFAttribute), GFP_KERNEL);
        }
    }
    mutex_unlock(&afe_control_mutex);
}

bool ResetAfeControl(void)
{
    int i = 0;
    printk("ResetAfeControl \n");
    mutex_lock(&afe_control_mutex);
    mAudioInit = false;
    memset((void *)(mAudioMrg), 0, sizeof(AudioMrgIf));
    memset((void *)(AudioDaiBt), 0, sizeof(AudioDigitalDAIBT));
    for (i = 0; i < Soc_Aud_IRQ_MCU_MODE_NUM_OF_IRQ_MODE ; i ++)
    {
        memset((void *)(mAudioMcuMode[i]), 0, sizeof(AudioIrqMcuMode));
    }
    for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK ; i ++)
    {
        memset((void *)(mAudioMEMIF[i]), 0, sizeof(AudioMemIFAttribute));
    }
    mutex_unlock(&afe_control_mutex);
}


/*****************************************************************************
 * FUNCTION
 *  Register_aud_irq
 *
 * DESCRIPTION
 *  IRQ handler
 *
 *****************************************************************************
 */

bool Register_Aud_Irq(void *dev)
{
    int ret = request_irq(MT6595_AFE_MCU_IRQ_LINE, AudDrv_IRQ_handler, IRQF_TRIGGER_LOW/*IRQF_TRIGGER_FALLING*/, "Afe_ISR_Handle", dev);
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_IRQ_handler / AudDrv_magic_tasklet
 *
 * DESCRIPTION
 *  IRQ handler
 *
 *****************************************************************************
 */
irqreturn_t AudDrv_IRQ_handler(int irq, void *dev_id)
{
    unsigned long flags;
    kal_uint32 volatile u4RegValue;

    //spin_lock_irqsave(&auddrv_irqstatus_lock, flags);
    u4RegValue = Afe_Get_Reg(AFE_IRQ_MCU_STATUS);
    u4RegValue &= 0xf;

    //printk("AudDrv_IRQ_handler AFE_IRQ_MCU_STATUS = %x \n", u4RegValue);

    // here is error handle , for interrupt is trigger but not status , clear all interrupt with bit 6
    if (u4RegValue == 0)
    {
        PRINTK_AUDDRV("u4RegValue == 0 \n");
        //AudDrv_Clk_On();
        Afe_Set_Reg(AFE_IRQ_MCU_CLR, 1 << 6 , 0xff);
        Afe_Set_Reg(AFE_IRQ_MCU_CLR, 1 , 0xff);
        Afe_Set_Reg(AFE_IRQ_MCU_CLR, 1 << 1 , 0xff);
        Afe_Set_Reg(AFE_IRQ_MCU_CLR, 1 << 2 , 0xff);
        Afe_Set_Reg(AFE_IRQ_MCU_CLR, 1 << 3 , 0xff);
        Afe_Set_Reg(AFE_IRQ_MCU_CLR, 1 << 4 , 0xff);
        //AudDrv_Clk_Off();
        goto AudDrv_IRQ_handler_exit;
    }

    if (u4RegValue & INTERRUPT_IRQ1_MCU)
    {
        Auddrv_DL_Interrupt_Handler();
    }
    if (u4RegValue & INTERRUPT_IRQ2_MCU)
    {
        if (mAudioMEMIF[Soc_Aud_Digital_Block_MEM_VUL]->mState == true)
        {
            Auddrv_UL_Interrupt_Handler();
        }
        if (mAudioMEMIF[Soc_Aud_Digital_Block_MEM_AWB]->mState == true)
        {
            Auddrv_AWB_Interrupt_Handler();
        }
    }
    if (u4RegValue & INTERRUPT_IRQ_MCU_DAI_SET)
    {

    }
    if (u4RegValue & INTERRUPT_IRQ_MCU_DAI_RST)
    {

    }
    // clear irq
    Afe_Set_Reg(AFE_IRQ_MCU_CLR, u4RegValue , 0xff);
    //spin_unlock_irqrestore(&auddrv_irqstatus_lock, flags);
AudDrv_IRQ_handler_exit:
    return IRQ_HANDLED;
}

/*****************************************************************************
 * FUNCTION
 *  Auddrv_Reg_map
 *
 * DESCRIPTION
 * Auddrv_Reg_map
 *
 *****************************************************************************
 */

void Auddrv_Reg_map()
{
    AFE_SRAM_ADDRESS = ioremap_nocache(AFE_INTERNAL_SRAM_PHY_BASE, AFE_INTERNAL_SRAM_SIZE);
    AFE_BASE_ADDRESS = ioremap_nocache(AUDIO_HW_PHYSICAL_BASE, 0x6000);

    // temp for hardawre code  set 0x1000629c = 0xd
    AFE_TOP_ADDRESS = ioremap_nocache(0x1000629c , 0x1000);

    printk("Auddrv_Reg_map AFE_TOP_ADDRESS = %p\n",AFE_TOP_ADDRESS);

    volatile uint32 *AFE_Register = (volatile uint32*)AFE_TOP_ADDRESS;
    volatile uint32 val_tmp;
    val_tmp = 0xd;
    mt65xx_reg_sync_writel(val_tmp,AFE_Register);
    AudDrv_Clk_AllOn();

    printk("AFE_BASE_ADDRESS = %p AFE_SRAM_ADDRESS = %p\n", AFE_BASE_ADDRESS, AFE_SRAM_ADDRESS);
}

static bool CheckMemIfEnable(void)
{
    int i = 0;
    for (i = 0 ;  i < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK ;  i++)
    {
        if ((mAudioMEMIF[i]->mState) ==  true)
        {
            return true;
        }
    }
    return false;
}


/*****************************************************************************
 * FUNCTION
 *  Auddrv_Reg_map
 *
 * DESCRIPTION
 * Auddrv_Reg_map
 *
 *****************************************************************************
 */
void EnableAfe(bool bEnable)
{
    unsigned int flags;
    spin_lock_irqsave(&afe_control_lock, flags);
    bool MemEnable = CheckMemIfEnable();
    if (0 == bEnable && false == MemEnable)
    {
        Afe_Set_Reg(AFE_DAC_CON0, 0x0, 0x0);
    }
    else if (1 == bEnable && true == MemEnable)
    {
        Afe_Set_Reg(AFE_DAC_CON0, 0x1, 0x1);
    }
    spin_unlock_irqrestore(&afe_control_lock, flags);
}

static uint32 SampleRateTransform(uint32 SampleRate)
{
    switch (SampleRate)
    {
        case 8000:
            return Soc_Aud_I2S_SAMPLERATE_I2S_8K;
        case 11025:
            return Soc_Aud_I2S_SAMPLERATE_I2S_11K;
        case 12000:
            return Soc_Aud_I2S_SAMPLERATE_I2S_12K;
        case 16000:
            return Soc_Aud_I2S_SAMPLERATE_I2S_16K;
        case 22050:
            return Soc_Aud_I2S_SAMPLERATE_I2S_22K;
        case 24000:
            return Soc_Aud_I2S_SAMPLERATE_I2S_24K;
        case 32000:
            return Soc_Aud_I2S_SAMPLERATE_I2S_32K;
        case 44100:
            return Soc_Aud_I2S_SAMPLERATE_I2S_44K;
        case 48000:
            return Soc_Aud_I2S_SAMPLERATE_I2S_48K;
        default:
            break;
    }
    return Soc_Aud_I2S_SAMPLERATE_I2S_44K;
}

bool SetSampleRate(uint32 Aud_block, uint32 SampleRate)
{
    SampleRate = SampleRateTransform(SampleRate);
    switch (Aud_block)
    {
        case Soc_Aud_Digital_Block_MEM_DL1:
        {
            Afe_Set_Reg(AFE_DAC_CON1, SampleRate , 0x0000000f);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_DL2:
        {
            Afe_Set_Reg(AFE_DAC_CON1, SampleRate << 4 , 0x000000f0);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_I2S:
        {
            Afe_Set_Reg(AFE_DAC_CON1, SampleRate << 8 , 0x00000f00);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_AWB:
        {
            Afe_Set_Reg(AFE_DAC_CON1, SampleRate << 12, 0x0000f000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_VUL:
        {
            Afe_Set_Reg(AFE_DAC_CON1, SampleRate << 16, 0x000f0000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_DAI:
        {
            if (SampleRate == Soc_Aud_I2S_SAMPLERATE_I2S_8K)
            {
                Afe_Set_Reg(AFE_DAC_CON1, 0 << 20 , 1 << 20);
            }
            else if (SampleRate == Soc_Aud_I2S_SAMPLERATE_I2S_16K)
            {
                Afe_Set_Reg(AFE_DAC_CON1, 1 << 20 , 1 << 20);
            }
            else
            {
                return false;
            }
            break;
        }
        case Soc_Aud_Digital_Block_MEM_MOD_DAI:
        {
            if (SampleRate == Soc_Aud_I2S_SAMPLERATE_I2S_8K)
            {
                Afe_Set_Reg(AFE_DAC_CON1, 0 << 30, 1 << 30);
            }
            else if (SampleRate == Soc_Aud_I2S_SAMPLERATE_I2S_16K)
            {
                Afe_Set_Reg(AFE_DAC_CON1, 1 << 30, 1 << 30);
            }
            else
            {
                return false;
            }
            break;
        }
        return true;
    }
}

bool SetChannels(uint32 Memory_Interface, uint32 channel)
{
    switch (Memory_Interface)
    {
        case Soc_Aud_Digital_Block_MEM_AWB:
        {
            Afe_Set_Reg(AFE_DAC_CON1, channel << 24, 1 << 24);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_VUL:
        {
            Afe_Set_Reg(AFE_DAC_CON1, channel << 27, 1 << 27);
            break;
        }
        default:
            printk("SetChannels  Memory_Interface = %d, channel = %d\n", Memory_Interface, channel);
            return false;
    }
    return true;
}

bool Set2ndI2SOut(uint32 samplerate)
{
    printk("Set2ndI2SOut\n");
    uint32 u32AudioI2S = 0;

    // set 2nd samplerate to AFE_ADC_CON1
    SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, samplerate);
    u32AudioI2S |= (Soc_Aud_INV_LRCK_NO_INVERSE << 5);
    u32AudioI2S |= (Soc_Aud_I2S_FORMAT_I2S << 3);
    u32AudioI2S |= (Soc_Aud_I2S_SRC_MASTER_MODE << 2); // default set to master mode
    u32AudioI2S |= (Soc_Aud_I2S_WLEN_WLEN_16BITS << 1);
    Afe_Set_Reg(AFE_I2S_CON, u32AudioI2S, MASK_ALL);
    return true;
}

bool  SetMrgI2SEnable(bool bEnable, unsigned int sampleRate)
{
#if 0
    if (bEnable == true)
    {
        // To enable MrgI2S
        if (mAudioMrg.MrgIf_En == true)
        {
            // Merge Interface already turn on.
            //if sample Rate change, then it need to restart with new setting; else do nothing.
            if (mAudioMrg.Mrg_I2S_SampleRate != SampleRateTransform(sampleRate))
            {
                //Turn off Merge Interface first to switch I2S sampling rate
                Afe_Set_Reg(AFE_MRGIF_CON, 0, 1 << 16); // Turn off I2S
                if (AudioDaiBt.mDAIBT_ON == true)
                {
                    Afe_Set_Reg(AFE_DAIBT_CON0, 0, 0x1);    //Turn off DAIBT first
                }

                Afe_Set_Reg(AFE_MRGIF_CON, 0, 0x1);    // Turn off Merge Interface
                // need delay? then turn on merge interface, I2S, PCM
                Afe_Set_Reg(AFE_MRGIF_CON, 1, 0x1);    // Turn on Merge Interface
                if (AudioDaiBt.mDAIBT_ON == true)
                {
                    Afe_Set_Reg(AFE_DAIBT_CON0, 1, 0x1);    //Turn on DAIBT
                }
                mAudioMrg.Mrg_I2S_SampleRate = I2SSampleRateTransform(sampleRate);
                Afe_Set_Reg(AFE_MRGIF_CON, mAudioMrg.Mrg_I2S_SampleRate << 20, 0xF00000); // set Mrg_I2S Samping Rate
                Afe_Set_Reg(AFE_MRGIF_CON, 1 << 16, 1 << 16); // set Mrg_I2S enable
            }
        }
        else
        {
            // turn on merge Interface from off state
            mAudioMrg.Mrg_I2S_SampleRate = I2SSampleRateTransform(sampleRate);
            Afe_Set_Reg(AFE_MRGIF_CON, mAudioMrg.Mrg_I2S_SampleRate << 20, 0xF00000); // set Mrg_I2S Samping Rate
            Afe_Set_Reg(AFE_MRGIF_CON, 1 << 16, 1 << 16);                        // set Mrg_I2S enable
            Afe_Set_Reg(AFE_MRGIF_CON, 1, 0x1);                              // Turn on Merge Interface
        }
        mAudioMrg.MrgIf_En = true;
        mAudioMrg.Mergeif_I2S_Enable = true;
    }
    else
    {
        if (mAudioMrg.MrgIf_En == true)
        {
            Afe_Set_Reg(AFE_MRGIF_CON, 0, 1 << 16); // Turn off I2S
            if (AudioDaiBt.mDAIBT_ON == false)
            {
                // DAIBT also not using, then it's OK to disable Merge Interface
                usleep(30);//delay 30us
                Afe_Set_Reg(AFE_MRGIF_CON, 0, 0x1);    // Turn off Merge Interface
                mAudioMrg.MrgIf_En = false;
            }
        }
        mAudioMrg.Mergeif_I2S_Enable = false;
    }
#endif
    return true;
}

bool SetI2SAdcIn(AudioDigtalI2S *DigtalI2S)
{
    memcpy((void *)AudioAdcI2S, (void *)DigtalI2S, sizeof(AudioDigtalI2S));

    if (false == AudioAdcI2SStatus)
    {
        uint32 eSamplingRate = SampleRateTransform(AudioAdcI2S->mI2S_SAMPLERATE);
        uint32 dVoiceModeSelect = 0;
        Afe_Set_Reg(AFE_ADDA_TOP_CON0, 0, 0x1); //Using Internal ADC
        if (eSamplingRate == Soc_Aud_I2S_SAMPLERATE_I2S_8K)
        {
            dVoiceModeSelect = 0;
        }
        else if (eSamplingRate == Soc_Aud_I2S_SAMPLERATE_I2S_16K)
        {
            dVoiceModeSelect = 1;
        }
        else if (eSamplingRate == Soc_Aud_I2S_SAMPLERATE_I2S_32K)
        {
            dVoiceModeSelect = 2;
        }
        else if (eSamplingRate == Soc_Aud_I2S_SAMPLERATE_I2S_48K)
        {
            dVoiceModeSelect = 3;
        }
        else
        {
        }
        Afe_Set_Reg(AFE_ADDA_UL_SRC_CON0, ((dVoiceModeSelect << 2) | dVoiceModeSelect) << 17, 0x001E0000);
        Afe_Set_Reg(AFE_ADDA_NEWIF_CFG0, 0x03F87201, 0xFFFFFFFF); // up8x txif sat on
        Afe_Set_Reg(AFE_ADDA_NEWIF_CFG1, ((dVoiceModeSelect < 3) ? 1 : 3) << 10, 0x00000C00);

    }
    else
    {
        Afe_Set_Reg(AFE_ADDA_TOP_CON0, 1, 0x1); //Using External ADC
        uint32 Audio_I2S_Adc = 0;
        Audio_I2S_Adc |= (AudioAdcI2S->mLR_SWAP << 31);
        Audio_I2S_Adc |= (AudioAdcI2S->mBuffer_Update_word << 24);
        Audio_I2S_Adc |= (AudioAdcI2S->mINV_LRCK << 23);
        Audio_I2S_Adc |= (AudioAdcI2S->mFpga_bit_test << 22);
        Audio_I2S_Adc |= (AudioAdcI2S->mFpga_bit << 21);
        Audio_I2S_Adc |= (AudioAdcI2S->mloopback << 20);
        Audio_I2S_Adc |= (SampleRateTransform(AudioAdcI2S->mI2S_SAMPLERATE) << 8);
        Audio_I2S_Adc |= (AudioAdcI2S->mI2S_FMT << 3);
        Audio_I2S_Adc |= (AudioAdcI2S->mI2S_WLEN << 1);
        printk("%s Audio_I2S_Adc = 0x%x", __FUNCTION__, Audio_I2S_Adc);
        Afe_Set_Reg(AFE_I2S_CON2, Audio_I2S_Adc, MASK_ALL);
    }
    return true;
}

bool EnableSideGenHw(uint32 connection , bool direction  , bool  Enable)
{
    printk("+%s(), connection = %d, direction = %d, Enable= %d\n", __FUNCTION__, connection, direction, Enable);
    if (Enable && direction)
    {
        switch (connection)
        {
            case Soc_Aud_InterConnectionInput_I00:
            case Soc_Aud_InterConnectionInput_I01:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x04662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I02:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x14662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I03:
            case Soc_Aud_InterConnectionInput_I04:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x24662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I05:
            case Soc_Aud_InterConnectionInput_I06:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x34662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I07:
            case Soc_Aud_InterConnectionInput_I08:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x44662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I09:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x54662662, 0xffffffff);
            case Soc_Aud_InterConnectionInput_I10:
            case Soc_Aud_InterConnectionInput_I11:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x64662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I12:
            case Soc_Aud_InterConnectionInput_I13:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x74662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I14:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x84662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I15:
            case Soc_Aud_InterConnectionInput_I16:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x94662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I17:
            case Soc_Aud_InterConnectionInput_I18:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x4662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I19:
            case Soc_Aud_InterConnectionInput_I20:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x4662662, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionInput_I21:
            case Soc_Aud_InterConnectionInput_I22:
                break;
                Afe_Set_Reg(AFE_SGEN_CON0, 0x4662662, 0xffffffff);
            default:
                break;
        }
    }
    else if (Enable)
    {
        switch (connection)
        {
            case Soc_Aud_InterConnectionOutput_O00:
            case Soc_Aud_InterConnectionOutput_O01:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x0c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O02:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x1c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O03:
            case Soc_Aud_InterConnectionOutput_O04:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x2c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O05:
            case Soc_Aud_InterConnectionOutput_O06:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x3c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O07:
            case Soc_Aud_InterConnectionOutput_O08:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x4c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O09:
            case Soc_Aud_InterConnectionOutput_O10:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x5c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O11:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x6c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O12:
                if (Soc_Aud_I2S_SAMPLERATE_I2S_8K == mAudioMEMIF[Soc_Aud_Digital_Block_MEM_MOD_DAI]->mSampleRate) //MD connect BT Verify (8K SamplingRate)
                {
                    Afe_Set_Reg(AFE_SGEN_CON0, 0x7c0e80e8, 0xffffffff);
                }
                else if (Soc_Aud_I2S_SAMPLERATE_I2S_16K == mAudioMEMIF[Soc_Aud_Digital_Block_MEM_MOD_DAI]->mSampleRate)
                {
                    Afe_Set_Reg(AFE_SGEN_CON0, 0x7c0f00f0, 0xffffffff);
                }
                else
                {
                    Afe_Set_Reg(AFE_SGEN_CON0, 0x7c6c26c2, 0xffffffff);    //Default
                }
                break;
            case Soc_Aud_InterConnectionOutput_O13:
            case Soc_Aud_InterConnectionOutput_O14:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x8c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O15:
            case Soc_Aud_InterConnectionOutput_O16:
                Afe_Set_Reg(AFE_SGEN_CON0, 0x9c6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O17:
            case Soc_Aud_InterConnectionOutput_O18:
                Afe_Set_Reg(AFE_SGEN_CON0, 0xac6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O19:
            case Soc_Aud_InterConnectionOutput_O20:
                Afe_Set_Reg(AFE_SGEN_CON0, 0xbc6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O21:
            case Soc_Aud_InterConnectionOutput_O22:
                Afe_Set_Reg(AFE_SGEN_CON0, 0xcc6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O23:
            case Soc_Aud_InterConnectionOutput_O24:
                Afe_Set_Reg(AFE_SGEN_CON0, 0xdc6c26c2, 0xffffffff);
                break;
            case Soc_Aud_InterConnectionOutput_O25:
                Afe_Set_Reg(AFE_SGEN_CON0, 0xec6c26c2, 0xffffffff);
            default:
                break;
        }
    }
    else
    {
        //don't set [31:28] as 0 when disable sinetone HW, because it will repalce i00/i01 input with sine gen output.
        //Set 0xf is correct way to disconnect sinetone HW to any I/O.
        Afe_Set_Reg(AFE_SGEN_CON0, 0xf0000000 , 0xffffffff);
    }
    return true;
}



bool SetI2SAdcEnable(bool bEnable)
{
    Afe_Set_Reg(AFE_ADDA_UL_SRC_CON0, bEnable ? 1 : 0, 0x01);
    mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState = bEnable;
    if (bEnable == true)
    {
        Afe_Set_Reg(AFE_ADDA_UL_DL_CON0, 0x0001, 0x0001);
    }
    else if (mAudioMEMIF[Soc_Aud_Digital_Block_I2S_OUT_DAC]->mState == false &&
             mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState == false)
    {
        Afe_Set_Reg(AFE_ADDA_UL_DL_CON0, 0x0000, 0x0001);
    }
    return true;
}

bool Set2ndI2SEnable(bool bEnable)
{
    Afe_Set_Reg(AFE_I2S_CON, bEnable, 0x1);
    return true;
}

bool CleanPreDistortion()
{
    //printk("%s \n", __FUNCTION__);
    Afe_Set_Reg(AFE_ADDA_PREDIS_CON0, 0, MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_PREDIS_CON1, 0, MASK_ALL);
}

bool SetDLSrc2(uint32 SampleRate)
{
    uint32 AfeAddaDLSrc2Con0, AfeAddaDLSrc2Con1;
    if (SampleRate == 8000)
    {
        AfeAddaDLSrc2Con0 = 0;
    }
    else if (SampleRate == 11025)
    {
        AfeAddaDLSrc2Con0 = 1;
    }
    else if (SampleRate == 12000)
    {
        AfeAddaDLSrc2Con0 = 2;
    }
    else if (SampleRate == 16000)
    {
        AfeAddaDLSrc2Con0 = 3;
    }
    else if (SampleRate == 22050)
    {
        AfeAddaDLSrc2Con0 = 4;
    }
    else if (SampleRate == 24000)
    {
        AfeAddaDLSrc2Con0 = 5;
    }
    else if (SampleRate == 32000)
    {
        AfeAddaDLSrc2Con0 = 6;
    }
    else if (SampleRate == 44100)
    {
        AfeAddaDLSrc2Con0 = 7;
    }
    else if (SampleRate == 48000)
    {
        AfeAddaDLSrc2Con0 = 8;
    }
    else
    {
        AfeAddaDLSrc2Con0 = 7;    //Default 44100
    }
    //ASSERT(0);
    if (AfeAddaDLSrc2Con0 == 0 || AfeAddaDLSrc2Con0 == 3) //8k or 16k voice mode
    {
        AfeAddaDLSrc2Con0 = (AfeAddaDLSrc2Con0 << 28) | (0x03 << 24) | (0x03 << 11) | (0x01 << 5);
    }
    else
    {
        AfeAddaDLSrc2Con0 = (AfeAddaDLSrc2Con0 << 28) | (0x03 << 24) | (0x03 << 11);
    }
    //SA suggest apply -0.3db to audio/speech path
    AfeAddaDLSrc2Con0 = AfeAddaDLSrc2Con0 | (0x01 << 1); //2013.02.22 for voice mode degrade 0.3 db
    AfeAddaDLSrc2Con1 = 0xf74f0000;

    Afe_Set_Reg(AFE_ADDA_DL_SRC2_CON0, AfeAddaDLSrc2Con0, MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_DL_SRC2_CON1, AfeAddaDLSrc2Con1, MASK_ALL);

}

bool SetI2SDacOut(uint32 SampleRate)
{
    //printk("SetI2SDacOut \n");
    CleanPreDistortion();
    SetDLSrc2(SampleRate);
    uint32 Audio_I2S_Dac = 0;
    Audio_I2S_Dac |= (Soc_Aud_LR_SWAP_NO_SWAP << 31);
    Audio_I2S_Dac |= (SampleRateTransform(SampleRate) << 8);
    Audio_I2S_Dac |= (Soc_Aud_INV_LRCK_NO_INVERSE << 5);
    Audio_I2S_Dac |= (Soc_Aud_I2S_FORMAT_I2S << 3);
    Audio_I2S_Dac |= (Soc_Aud_I2S_WLEN_WLEN_16BITS << 1);
    Afe_Set_Reg(AFE_I2S_CON1, Audio_I2S_Dac, MASK_ALL);
    return true;
}

bool SetHwDigitalGainMode(uint32 GainType, uint32 SampleRate, uint32 SamplePerStep)
{
    //printk("SetHwDigitalGainMode GainType = %d, SampleRate = %d, SamplePerStep= %d\n", GainType, SampleRate, SamplePerStep);
    uint32 value = 0;
    value = SamplePerStep << 8 | SampleRate << 4;
    switch (GainType)
    {
        case Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN1:
            Afe_Set_Reg(AFE_GAIN1_CON0, value, 0xfff0);
            break;
        case Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2:
            Afe_Set_Reg(AFE_GAIN2_CON0, value, 0xfff0);
            break;
        default:
            return false;
    }
    return true;
}

bool SetHwDigitalGainEnable(int GainType, bool Enable)
{
    printk("+%s(), GainType = %d, Enable = %d\n", __FUNCTION__, GainType, Enable);
    switch (GainType)
    {
        case Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN1:
            if (Enable)
            {
                Afe_Set_Reg(AFE_GAIN1_CUR, 0, 0xFFFFFFFF);    //Let current gain be 0 to ramp up
            }
            Afe_Set_Reg(AFE_GAIN1_CON0, Enable, 0x1);
            break;
        case Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2:
            if (Enable)
            {
                Afe_Set_Reg(AFE_GAIN2_CUR, 0, 0xFFFFFFFF);    //Let current gain be 0 to ramp up
            }
            Afe_Set_Reg(AFE_GAIN2_CON0, Enable, 0x1);
            break;
        default:
            printk("%s with no match type\n", __func__);
            return false;
    }
    return true;
}

bool  SetHwDigitalGain(uint32 Gain , int GainType)
{
    printk("+%s(), Gain = 0x%x, gain type = %d\n", __func__, Gain, GainType);
    switch (GainType)
    {
        case Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN1:
            Afe_Set_Reg(AFE_GAIN1_CON1, Gain, 0xffffffff);
            break;
        case Soc_Aud_Hw_Digital_Gain_HW_DIGITAL_GAIN2:
            Afe_Set_Reg(AFE_GAIN2_CON1, Gain, 0xffffffff);
            break;
        default:
            printk("%s with no match type\n", __func__);
            return false;
    }
    return true;
}

bool SetModemPcmConfig(int modem_index , AudioDigitalPCM p_modem_pcm_attribute)
{
    printk("+%s()\n", __func__);
    uint32 reg_pcm2_intf_con = 0;
    if (modem_index == MODEM_1)
    {
        reg_pcm2_intf_con |= (p_modem_pcm_attribute.mTxLchRepeatSel     & 0x1) << 13;
        reg_pcm2_intf_con |= (p_modem_pcm_attribute.mVbt16kModeSel      & 0x1) << 12;
        reg_pcm2_intf_con |= (p_modem_pcm_attribute.mSingelMicSel       & 0x1) << 7;
        reg_pcm2_intf_con |= (p_modem_pcm_attribute.mPcmWordLength      & 0x1) << 4;
        reg_pcm2_intf_con |= (p_modem_pcm_attribute.mPcmModeWidebandSel & 0x1) << 3;
        reg_pcm2_intf_con |= (p_modem_pcm_attribute.mPcmFormat          & 0x3) << 1;
        printk("%s(), PCM2_INTF_CON(0x%x) = 0x%x\n", __FUNCTION__, PCM2_INTF_CON, reg_pcm2_intf_con);
        Afe_Set_Reg(PCM2_INTF_CON, reg_pcm2_intf_con, MASK_ALL);
    }
    else if (modem_index == MODEM_2 || modem_index == MODEM_EXTERNAL)    // MODEM_2 use PCM_INTF_CON1 (0x530) !!!
    {
        // config ASRC for modem 2
        if (p_modem_pcm_attribute.mPcmModeWidebandSel == Soc_Aud_PCM_MODE_PCM_MODE_8K)
        {
            Afe_Set_Reg(AFE_ASRC_CON1, 0x00001964, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON2, 0x00400000, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON3, 0x00400000, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON4, 0x00001964, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON7, 0x00000CB2, MASK_ALL);
        }
        else if (p_modem_pcm_attribute.mPcmModeWidebandSel == Soc_Aud_PCM_MODE_PCM_MODE_16K)
        {
            Afe_Set_Reg(AFE_ASRC_CON1, 0x00000cb2, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON2, 0x00400000, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON3, 0x00400000, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON4, 0x00000cb2, MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON7, 0x00000659, MASK_ALL);
        }


        uint32 reg_pcm_intf_con1 = 0;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mTxLchRepeatSel        & 0x01) << 19;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mVbt16kModeSel         & 0x01) << 18;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mExtModemSel       & 0x01) << 17;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mExtendBckSyncLength  & 0x1F) << 9;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mExtendBckSyncTypeSel & 0x01) << 8;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mSingelMicSel      & 0x01) << 7;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mAsyncFifoSel      & 0x01) << 6;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mSlaveModeSel      & 0x01) << 5;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mPcmWordLength         & 0x01) << 4;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mPcmModeWidebandSel    & 0x01) << 3;
        reg_pcm_intf_con1 |= (p_modem_pcm_attribute.mPcmFormat             & 0x03) << 1;

        printk("%s(), PCM_INTF_CON1(0x%x) = 0x%x", __FUNCTION__, PCM_INTF_CON, reg_pcm_intf_con1);
        Afe_Set_Reg(PCM_INTF_CON, reg_pcm_intf_con1, MASK_ALL);

    }
    return true;
}

bool SetModemPcmEnable(int modem_index, bool modem_pcm_on)
{
    printk("+%s(), modem_index = %d, modem_pcm_on = %d\n", __FUNCTION__, modem_index, modem_pcm_on);

    if (modem_index == MODEM_1) // MODEM_1 use PCM2_INTF_CON (0x53C) !!!
    {
        Afe_Set_Reg(PCM2_INTF_CON, modem_pcm_on, 0x1);
        mAudioMEMIF[Soc_Aud_Digital_Block_MODEM_PCM_1_O]->mState = modem_pcm_on;
    }
    else if (modem_index == MODEM_2 || modem_index == MODEM_EXTERNAL) // MODEM_2 use PCM_INTF_CON1 (0x530) !!!
    {
        if (modem_pcm_on == true) // turn on ASRC before Modem PCM on
        {
            Afe_Set_Reg(AFE_ASRC_CON6, 0x0001183F, MASK_ALL); // pre ver. 0x0001188F
            Afe_Set_Reg(AFE_ASRC_CON0, 0x06003031, 0xFFFFFFBF);
            Afe_Set_Reg(PCM_INTF_CON, 0x1, 0x1);
        }
        else if (modem_pcm_on == false) // turn off ASRC after Modem PCM off
        {
            Afe_Set_Reg(PCM_INTF_CON, 0x0, 0x1);
            Afe_Set_Reg(AFE_ASRC_CON6, 0x00000000, MASK_ALL);
            uint32 dNeedDisableASM  = (Afe_Get_Reg(AFE_ASRC_CON0) & 0x0040) ? 1 : 0;
            Afe_Set_Reg(AFE_ASRC_CON0, 0, (1 << 4 | 1 << 5 | dNeedDisableASM));
            Afe_Set_Reg(AFE_ASRC_CON0, 0x0, 0x1);
        }
        mAudioMEMIF[Soc_Aud_Digital_Block_MODEM_PCM_2_O]->mState = modem_pcm_on;
    }
    else
    {
        printk("%s(), no such modem_index: %d!!", __FUNCTION__, modem_index);
        return false;
    }
    return true;
}


bool EnableSideToneFilter(bool stf_on)
{
    printk("+%s(), stf_on = %d", __func__, stf_on);
    // MD max support 16K sampling rate
    const uint8_t kSideToneHalfTapNum = sizeof(kSideToneCoefficientTable16k) / sizeof(uint16_t);

    if (stf_on == false)
    {
        // bypass STF result & disable
        const bool bypass_stf_on = true;
        uint32_t reg_value = (bypass_stf_on << 31) | (stf_on << 8);
        Afe_Set_Reg(AFE_SIDETONE_CON1, reg_value, MASK_ALL);
        printk("%s(), AFE_SIDETONE_CON1[0x%x] = 0x%x", __FUNCTION__, AFE_SIDETONE_CON1, reg_value);

        // set side tone gain = 0
        Afe_Set_Reg(AFE_SIDETONE_GAIN, 0, MASK_ALL);
        printk("%s(), AFE_SIDETONE_GAIN[0x%x] = 0x%x", __FUNCTION__, AFE_SIDETONE_GAIN, 0);
    }
    else
    {
        // set side tone gain
        Afe_Set_Reg(AFE_SIDETONE_GAIN, 0, MASK_ALL);
        printk("%s(), AFE_SIDETONE_GAIN[0x%x] = 0x%x", __FUNCTION__, AFE_SIDETONE_GAIN, 0);

        // using STF result & enable & set half tap num
        const bool bypass_stf_on = false;
        uint32_t write_reg_value = (bypass_stf_on << 31) | (stf_on << 8) | kSideToneHalfTapNum;
        Afe_Set_Reg(AFE_SIDETONE_CON1, write_reg_value, MASK_ALL);
        printk("%s(), AFE_SIDETONE_CON1[0x%x] = 0x%x", __FUNCTION__, AFE_SIDETONE_CON1, write_reg_value);

        // set side tone coefficient
        const bool enable_read_write = true; // enable read/write side tone coefficient
        const bool read_write_sel = true;    // for write case
        const bool sel_ch2 = false;          // using uplink ch1 as STF input

        uint32_t   read_reg_value = Afe_Get_Reg(AFE_SIDETONE_CON0);
        size_t coef_addr = 0;
        for (coef_addr = 0; coef_addr < kSideToneHalfTapNum; coef_addr++)
        {
            bool old_write_ready = (read_reg_value >> 29) & 0x1;
            write_reg_value = enable_read_write << 25 |
                              read_write_sel    << 24 |
                              sel_ch2           << 23 |
                              coef_addr         << 16 |
                              kSideToneCoefficientTable16k[coef_addr];
            Afe_Set_Reg(AFE_SIDETONE_CON0, write_reg_value, 0x39FFFFF);
            printk("%s(), AFE_SIDETONE_CON0[0x%x] = 0x%x", __FUNCTION__, AFE_SIDETONE_CON0, write_reg_value);

            // wait until flag write_ready changed (means write done)
            int try_cnt = 0;
            for (try_cnt = 0; try_cnt < 10; try_cnt++)  // max try 10 times
            {
                msleep(3);
                read_reg_value = Afe_Get_Reg(AFE_SIDETONE_CON0);
                bool new_write_ready = (read_reg_value >> 29) & 0x1;
                if (new_write_ready != old_write_ready) // flip => ok
                {
                    break;
                }
                else
                {

                    BUG_ON(new_write_ready != old_write_ready);
                    return false;
                }
            }
        }
    }

    printk("-%s(), stf_on = %d", __FUNCTION__, stf_on);
    return true;
}


bool SetMemoryPathEnable(uint32 Aud_block, bool bEnable)
{
    if (Aud_block  >= Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE)
    {
        return true;
    }
    if (bEnable)
    {
        mAudioMEMIF[Aud_block]->mState =  true;
        Afe_Set_Reg(AFE_DAC_CON0, bEnable << (Aud_block + 1) , 1 << (Aud_block + 1));
    }
    else
    {
#ifdef SIDEGEN_ENABLE
        Afe_Set_Reg(AFE_SGEN_CON0, 0x0, 0xffffffff);
#endif
        Afe_Set_Reg(AFE_DAC_CON0, bEnable << (Aud_block + 1), 1 << (Aud_block + 1));
        mAudioMEMIF[Aud_block]->mState =  false;
    }
}


bool SetI2SDacEnable(bool bEnable)
{
    mAudioMEMIF[Soc_Aud_Digital_Block_I2S_OUT_DAC]->mState = bEnable;
    if (bEnable)
    {
        Afe_Set_Reg(AFE_ADDA_DL_SRC2_CON0, bEnable, 0x01);
        Afe_Set_Reg(AFE_I2S_CON1, bEnable, 0x1);
        Afe_Set_Reg(AFE_ADDA_UL_DL_CON0, bEnable, 0x0001);
        Afe_Set_Reg(FPGA_CFG1, 0, 0x10);    //For FPGA Pin the same with DAC
    }
    else
    {
        Afe_Set_Reg(AFE_ADDA_DL_SRC2_CON0, bEnable, 0x01);
        Afe_Set_Reg(AFE_I2S_CON1, bEnable, 0x1);

        if (mAudioMEMIF[Soc_Aud_Digital_Block_I2S_OUT_DAC]->mState == false &&
            mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState == false)
        {
            Afe_Set_Reg(AFE_ADDA_UL_DL_CON0, bEnable, 0x0001);
        }
        Afe_Set_Reg(FPGA_CFG1, 1 << 4, 0x10);    //For FPGA Pin the same with DAC
    }
    return true;
}

bool GetI2SDacEnable()
{
    return mAudioMEMIF[Soc_Aud_Digital_Block_I2S_OUT_DAC]->mState ;
}

bool checkUplinkMEMIfStatus()
{
    int i = 0;
    for (i = Soc_Aud_Digital_Block_MEM_VUL ; i < Soc_Aud_Digital_Block_MEM_MOD_DAI ; i ++)
    {
        if (mAudioMEMIF[i]->mState  == true)
        {
            return true;
        }
    }
    return false;
}


bool SetConnection(uint32 ConnectionState, uint32 Input , uint32 Output)
{
    return SetConnectionState(ConnectionState, Input, Output);
}

bool SetIrqEnable(uint32 Irqmode, bool bEnable)
{
    //printk("+%s(), Irqmode = %d, bEnable = %d\n", __FUNCTION__, Irqmode, bEnable);
    switch (Irqmode)
    {
        case Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE:
        {
            Afe_Set_Reg(AFE_IRQ_MCU_CON, (bEnable << Irqmode), (1 << Irqmode));
            break;
        }
        case Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE:
        case Soc_Aud_IRQ_MCU_MODE_IRQ3_MCU_MODE:
        {
            if (checkUplinkMEMIfStatus() == false)
            {
                Afe_Set_Reg(AFE_IRQ_MCU_CON, (bEnable << Irqmode), (1 << Irqmode));
            }
            break;
        }
        default:
            break;
    }
    printk("-%s(), Irqmode = %d, bEnable = %d\n", __FUNCTION__, Irqmode, bEnable);
    return true;
}

bool SetIrqMcuSampleRate(uint32  Irqmode, uint32 SampleRate)
{
    switch (Irqmode)
    {
        case Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE:
        {
            Afe_Set_Reg(AFE_IRQ_MCU_CON, (SampleRateTransform(SampleRate) << 4), 0x000000f0);
            break;
        }
        case Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE:
        {
            Afe_Set_Reg(AFE_IRQ_MCU_CON, (SampleRateTransform(SampleRate) << 8), 0x00000f00);
            break;
        }
        case Soc_Aud_IRQ_MCU_MODE_IRQ3_MCU_MODE:
        default:
            return false;
    }
    return true;
}

bool SetIrqMcuCounter(uint32 Irqmode, uint32 Counter)
{
    switch (Irqmode)
    {
        case Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE:
        {
            Afe_Set_Reg(AFE_IRQ_MCU_CNT1, Counter, 0xffffffff);
            break;
        }
        case Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE:
        {
            Afe_Set_Reg(AFE_IRQ_MCU_CNT2, Counter, 0xffffffff);
            break;
        }
        case Soc_Aud_IRQ_MCU_MODE_IRQ3_MCU_MODE:
        default:
            return false;
    }
    return true;
}

bool SetMemDuplicateWrite(uint32 InterfaceType, int dupwrite)
{
    switch (InterfaceType)
    {
        case Soc_Aud_Digital_Block_MEM_DAI:
        {
            Afe_Set_Reg(AFE_DAC_CON1, dupwrite << 29, 1 << 29);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_MOD_DAI:
        {
            Afe_Set_Reg(AFE_DAC_CON1, dupwrite << 31, 1 << 31);
            break;
        }
        default:
            return false;
    }
    return true;
}


bool Set2ndI2SInConfig(unsigned int sampleRate, bool bIsSlaveMode)
{
    AudioDigtalI2S I2S2ndIn_attribute;
    memset((void *)&I2S2ndIn_attribute, 0, sizeof(I2S2ndIn_attribute));
    I2S2ndIn_attribute.mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
    I2S2ndIn_attribute.mI2S_SLAVE = bIsSlaveMode;
    I2S2ndIn_attribute.mI2S_SAMPLERATE = sampleRate;
    I2S2ndIn_attribute.mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
    I2S2ndIn_attribute.mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
    I2S2ndIn_attribute.mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
    Set2ndI2SIn(&I2S2ndIn_attribute);
    return true;
}

bool Set2ndI2SIn(AudioDigtalI2S *mDigitalI2S)
{
    memcpy((void *)m2ndI2S, (void *)mDigitalI2S, sizeof(AudioDigtalI2S));
    uint32 Audio_I2S_Adc = 0;
    if (!m2ndI2S->mI2S_SLAVE) //Master setting SampleRate only
    {
        SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, m2ndI2S->mI2S_SAMPLERATE);
    }
    Audio_I2S_Adc |= (m2ndI2S->mINV_LRCK << 5);
    Audio_I2S_Adc |= (m2ndI2S->mI2S_FMT << 3);
    Audio_I2S_Adc |= (m2ndI2S->mI2S_SLAVE << 2);
    Audio_I2S_Adc |= (m2ndI2S->mI2S_WLEN << 1);
    Audio_I2S_Adc |= (m2ndI2S->mI2S_IN_PAD_SEL << 28);
    Audio_I2S_Adc |= 1 << 31;//Default enable phase_shift_fix for better quality
    printk("Set2ndI2SIn Audio_I2S_Adc= 0x%x", Audio_I2S_Adc);
    Afe_Set_Reg(AFE_I2S_CON, Audio_I2S_Adc, 0xfffffffe);
    if (!m2ndI2S->mI2S_SLAVE)
    {
        Afe_Set_Reg(FPGA_CFG1, 1 << 8, 0x0100);
    }
    else
    {
        Afe_Set_Reg(FPGA_CFG1, 0, 0x0100);
    }
    return true;
}

bool Set2ndI2SInEnable(bool bEnable)
{
    printk("Set2ndI2SInEnable bEnable = %d", bEnable);
    m2ndI2S->mI2S_EN = bEnable;
    Afe_Set_Reg(AFE_I2S_CON, bEnable, 0x1);
    mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_2]->mState = bEnable;
    return true;
}

bool SetI2SASRCConfig(bool bIsUseASRC, unsigned int dToSampleRate)
{
    printk("+%s() bIsUseASRC [%d] dToSampleRate [%d]\n", __FUNCTION__, bIsUseASRC, dToSampleRate);
    if (true == bIsUseASRC)
    {
        BUG_ON(!(dToSampleRate == 44100 || dToSampleRate == 48000));
        Afe_Set_Reg(AFE_CONN4, 0, 1 << 30);
        SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, dToSampleRate);//To target sample rate
        Afe_Set_Reg(AFE_ASRC_CON13, 0, 1 << 16); //0:Stereo 1:Mono
        if (dToSampleRate == 44100)
        {
            Afe_Set_Reg(AFE_ASRC_CON14, 0xDC8000, AFE_MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON15, 0xA00000, AFE_MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON17, 0x1FBD, AFE_MASK_ALL);
        }
        else
        {
            Afe_Set_Reg(AFE_ASRC_CON14, 0x600000, AFE_MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON15, 0x400000, AFE_MASK_ALL);
            Afe_Set_Reg(AFE_ASRC_CON17, 0xCB2, AFE_MASK_ALL);
        }

        Afe_Set_Reg(AFE_ASRC_CON16, 0x00075987, AFE_MASK_ALL);//Calibration setting
        Afe_Set_Reg(AFE_ASRC_CON20, 0x00001b00, AFE_MASK_ALL);//Calibration setting
    }
    else
    {
        Afe_Set_Reg(AFE_CONN4, 1 << 30, 1 << 30);
    }
    return true;
}

bool SetI2SASRCEnable(bool bEnable)
{
    if (true == bEnable)
    {
        Afe_Set_Reg(AFE_ASRC_CON0, ((1 << 6) | (1 << 0)), ((1 << 6) | (1 << 0)));
    }
    else
    {
        uint32 dNeedDisableASM  = (Afe_Get_Reg(AFE_ASRC_CON0) & 0x0030) ? 1 : 0;
        Afe_Set_Reg(AFE_ASRC_CON0, 0, (1 << 6 | dNeedDisableASM));
    }
    return true;
}

bool  SetMemIfFetchFormatPerSample(uint32 InterfaceType, uint32 eFetchFormat)
{
    mAudioMEMIF[InterfaceType]->mFetchFormatPerSample = eFetchFormat;
    /*printk("+%s(), InterfaceType = %d, eFetchFormat = %d, mAudioMEMIF[InterfaceType].mFetchFormatPerSample = %d\n",__FUNCTION__
           , InterfaceType, eFetchFormat, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample);*/
    switch (InterfaceType)
    {
        case Soc_Aud_Digital_Block_MEM_DL1:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 16 , 0x00030000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 12 , 0x00003000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_DL2:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 18 , 0x0000c000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_I2S:
        {
            //Afe_Set_Reg(AFE_DAC_CON1, mAudioMEMIF[InterfaceType].mSampleRate << 8 , 0x00000f00);
            printk("Unsupport MEM_I2S");
            break;
        }
        case Soc_Aud_Digital_Block_MEM_AWB:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 20, 0x00300000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_VUL:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 22, 0x00C00000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 14, 0x000C0000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_DAI:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 24, 0x03000000);
            break;
        }
        case Soc_Aud_Digital_Block_MEM_MOD_DAI:
        {
            Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, mAudioMEMIF[InterfaceType]->mFetchFormatPerSample << 26, 0x0C000000);
            break;
        }
        default:
            return false;
    }
    return true;
}

bool SetoutputConnectionFormat(uint32 ConnectionFormat,uint32  Output)
{
    //printk("+%s(), Data Format = %d, Output = %d\n", __FUNCTION__, ConnectionFormat, Output);
    Afe_Set_Reg(AFE_CONN_24BIT, (ConnectionFormat << Output), (1 << Output));
    return true;
}




