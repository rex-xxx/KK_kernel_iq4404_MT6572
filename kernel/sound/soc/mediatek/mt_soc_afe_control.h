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
 *  mt_sco_afe_control.h
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

#ifndef _AUDIO_AFE_CONTROL_H
#define _AUDIO_AFE_CONTROL_H

#include "mt_soc_digital_type.h"
#include "AudDrv_Def.h"

#define GIC_PRIVATE_SIGNALS          (32)
#define AUDDRV_DL1_MAX_BUFFER_LENGTH (0x6000)
#define MT6595_AFE_MCU_IRQ_LINE (GIC_PRIVATE_SIGNALS + 0x86)
#define MASK_ALL          (0xFFFFFFFF)
#define AFE_MASK_ALL  (0xffffffff)

bool InitAfeControl(void);
bool ResetAfeControl(void);
bool Register_Aud_Irq(void *dev);
void Auddrv_Reg_map(void);

bool SetSampleRate(uint32 Aud_block, uint32 SampleRate);
bool SetChannels(uint32 Memory_Interface, uint32 channel);

bool SetIrqMcuCounter(uint32 Irqmode, uint32 Counter);
bool SetIrqEnable(uint32 Irqmode, bool bEnable);
bool SetIrqMcuSampleRate(uint32  Irqmode, uint32 SampleRate);

bool SetConnection(uint32 ConnectionState, uint32 Input , uint32 Output);
bool SetMemoryPathEnable(uint32 Aud_block, bool bEnable);
bool SetI2SDacEnable(bool bEnable);
bool GetI2SDacEnable();
void EnableAfe(bool bEnable);
bool Set2ndI2SOut(uint32 samplerate);
bool SetI2SAdcIn(AudioDigtalI2S *DigtalI2S);
bool SetMrgI2SEnable(bool bEnable, unsigned int sampleRate);
bool SetI2SAdcEnable(bool bEnable);
bool Set2ndI2SEnable(bool bEnable);
bool SetI2SDacOut(uint32 SampleRate);
bool SetHwDigitalGainMode(uint32 GainType, uint32 SampleRate, uint32 SamplePerStep);
bool SetHwDigitalGainEnable(int GainType, bool Enable);
bool SetHwDigitalGain(uint32 Gain , int GainType);

bool SetMemDuplicateWrite(uint32 InterfaceType, int dupwrite);
bool EnableSideGenHw(uint32 connection , bool direction  , bool  Enable);
bool CleanPreDistortion(void);
bool EnableSideToneFilter(bool stf_on);
bool SetModemPcmEnable(int modem_index, bool modem_pcm_on);
bool SetModemPcmConfig(int modem_index, AudioDigitalPCM p_modem_pcm_attribute);

bool Set2ndI2SIn(AudioDigtalI2S *mDigitalI2S);
bool Set2ndI2SInConfig(unsigned int sampleRate, bool bIsSlaveMode);
bool Set2ndI2SInEnable(bool bEnable);

bool SetI2SASRCConfig(bool bIsUseASRC, unsigned int dToSampleRate);
bool SetI2SASRCEnable(bool bEnable);

bool checkUplinkMEMIfStatus(void);
bool  SetMemIfFetchFormatPerSample(uint32 InterfaceType, uint32 eFetchFormat);
bool SetoutputConnectionFormat(uint32 ConnectionFormat,uint32  Output);


#endif

