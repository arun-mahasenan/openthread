/*
 *  Copyright (c) 2021, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the transmit power control functionality.
 */

#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE && (OPENTHREAD_MTD || OPENTHREAD_FTD)
#include "power_control.hpp"
#include "power_control_stats.hpp"
#endif
#include <math.h>
#include <openthread/link_metrics.h>
#include <openthread/platform/diag.h>
#include "common/code_utils.hpp"
#include "instance/instance.hpp"
#include "common/locator_getters.hpp"
#include "common/logging.hpp"
#include "common/random.hpp"
#include "thread/router_table.hpp"

#if OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE

namespace ot {
namespace Utils {

#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
#define logTpc(...)                        \
    if (GetMode() != kPowerControlDisable) \
    {                                      \
        otLogDebgUtil(__VA_ARGS__);        \
    }
#else
#define logTpc(...)
#endif

static void logMode(const char *aPrefix, uint8_t aMode)
{
    OT_UNUSED_VARIABLE(aPrefix);
    OT_UNUSED_VARIABLE(aMode);

#if OPENTHREAD_CONFIG_LOG_LEVEL_UTIL >= OT_LOG_LEVEL_INFO
    const bool lm = aMode & kPowerControlSingleProbeLinkMetrics;
    const bool rx = aMode & kPowerControlRxEncodedAck;
    const bool tx = aMode & kPowerControlTxEncodedAck;
#endif

    otLogInfoUtil("TPC: %s, link metrics: %u, handle ACKs: %u, encode ACKs: %u", aPrefix, lm, rx, tx);
}

PowerControl::PowerControl(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mRouterLinkMarginSetPoint(kRouterLinkMarginSetPoint)
    , mEndDeviceLinkMarginSetPoint(kEndDeviceLinkMarginSetPoint)
    , mKp(kKp)
    , mKi(kKi)
    , mAlpha(kAlpha)
    , mMode(PowerControlMode::kPowerControlDisable)
    , mStepUpAckLost(3)
#if OPENTHREAD_RADIO
    , mIsAttached(false)
    , mTable(&aInstance, &mMode, &mKi, &mMaxTxPower)
#endif
{
    int8_t mTxPower;
    mErrorGain[0] = kErrorGain0;
    mErrorGain[1] = kErrorGain1;
    mErrorGain[2] = kErrorGain2;
    mErrorGain[3] = kErrorGain3;
    otPlatRadioGetTransmitPower(&GetInstance(), &mTxPower); // This init doesnt seem to work
    SetMaxTxPower(mTxPower);
    SetMaxTxPower(16);
}

Error PowerControl::Start(void)
{
    Error error = kErrorNone;

    logMode("start", mMode);

    if (mMode & kPowerControlTxEncodedAck)
    {
        // Set parameters for the Radio driver to set RSSI feedback in ACK header
        otPlatRadioSetLmeAckSetPoint(&GetInstance(), mRouterLinkMarginSetPoint);
        otPlatRadioSetLmeAckEnabled(&GetInstance(), true);
    }
    else
    {
        otPlatRadioSetLmeAckEnabled(&GetInstance(), false);
    }
    return error;
}

Error PowerControl::Stop(void)
{
    Error error = kErrorNone;
    ResetNeighbors();

    otLogDebgUtil("TPC: stopped");
    return error;
}

Error PowerControl::SetMode(PowerControlMode aMode)
{
    Error error = kErrorNone;

#if !OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    if (aMode == PowerControlMode::kPowerControlSingleProbeLinkMetrics)
    {
        error = kErrorInvalidArgs;
    }
    else
#endif
    {
        // Stop and Reset previous running Power Control mode and statistics
        Stop();
        mMode = aMode;
        Start();
    }

    return error;
}

PowerControlMode PowerControl::GetMode()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetMode %d", mMode);
#endif
    return mMode;
}

void PowerControl::SetErrorGain(int8_t aGain[])
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetLinkMetricsProbeInterval %d", aGain[0], aGain[1], aGain[2], aGain[3]);
#endif
    memcpy(mErrorGain, aGain, 4);
}

void PowerControl::SetErrorGainAtIndex(int8_t aGain, uint8_t index)
{
    ////Spinel interface can take only +ve numbers. Flipping the sign is a work around for this limitation.
    if (index < 3 && aGain > 0)
    {
        aGain = -1 * aGain;
    }
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetLinkMetricsProbeInterval index(%d) gain(%d)", index, aGain);
#endif
    mErrorGain[index] = aGain;
}

int8_t *PowerControl::GetErrorGain()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetGain %d", mErrorGain);
#endif
    return mErrorGain;
}

int8_t PowerControl::GetErrorGainAtIndex(uint8_t index)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetGain index(%d) value(%d)", index, mErrorGain[index]);
#endif
    return mErrorGain[index];
}

void PowerControl::SetRouterLinkMarginSetPoint(uint8_t aSetPoint)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetRouterLinkMarginSetPoint %d", aSetPoint);
#endif
    mRouterLinkMarginSetPoint = aSetPoint;
}

uint8_t PowerControl::GetRouterLinkMarginSetPoint()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetRouterLinkMarginSetPoint %d", mRouterLinkMarginSetPoint);
#endif
    return mRouterLinkMarginSetPoint;
}

void PowerControl::SetEndDeviceLinkMarginSetPoint(uint8_t aSetPoint)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetEndDeviceLinkMarginSetPoint %d", aSetPoint);
#endif
    mEndDeviceLinkMarginSetPoint = aSetPoint;
}

uint8_t PowerControl::GetEndDeviceLinkMarginSetPoint()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetEndDeviceLinkMarginSetPoint %d", mEndDeviceLinkMarginSetPoint);
#endif
    return mEndDeviceLinkMarginSetPoint;
}

void PowerControl::SetProportionalGain(int32_t aGain)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetProportionalGain %d", aGain);
#endif
    // Spinel interface can take only +ve numbers.
    // Flipping the sign is a work around for this limitation.
    mKp = aGain * -1;
}

int32_t PowerControl::GetProportionalGain()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetProportionalGain %d", mKp);
#endif
    return mKp;
}

void PowerControl::SetIntegralGain(int32_t aGain)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetIntegralGain %d", aGain);
#endif
    // Spinel interface can take only +ve numbers.
    // Flipping the sign is a work around for this limitation.
    mKi = aGain * -1;
}

int32_t PowerControl::GetIntegralGain()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetIntegralGain");
#endif
    return mKi;
}

void PowerControl::SetWeightAverage(uint8_t aWeight)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::SetWeightAverage %u%%", aWeight);
#endif
    mAlpha = aWeight;
}

uint8_t PowerControl::GetWeightAverage()
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogInfoUtil("TPC:PowerControl::GetWeightAverage %u%%", mAlpha);
#endif
    return mAlpha;
}

bool PowerControl::IsAttached()
{
#if OPENTHREAD_RADIO
    return mIsAttached;
#else
    return GetInstance().Get<Mle::MleRouter>().IsAttached();
#endif
}

void PowerControl::HandleAck(ot::Mac::Address &aAddress, uint16_t aFCF, Error aAckStatus)
{
#if OPENTHREAD_RADIO
    PowerControlTable::Entry *neighbor;
#else
    NeighborType *neighbor;
#endif

    // Return if transmit power control is not enabled in this device
    if (mMode == kPowerControlDisable || !IsAttached())
    {
        return;
    }

    switch (aAddress.GetType())
    {
    case ot::Mac::Address::kTypeShort:
#if OPENTHREAD_RADIO
        neighbor = mTable.FindEntry(aAddress.GetShort());
#else
        neighbor = GetInstance().Get<NeighborTable>().FindNeighbor(aAddress.GetShort(), Neighbor::kInStateValid);
#endif

        logTpc("TPC:neighbor ShortAdd=%d", aAddress.GetShort());

        break;

    case ot::Mac::Address::kTypeExtended:
#if OPENTHREAD_RADIO
        neighbor = mTable.FindEntry(aAddress.GetExtended());
#else
        neighbor = GetInstance().Get<NeighborTable>().FindNeighbor(aAddress.GetExtended(), Neighbor::kInStateValid);
#endif

        logTpc("TPC:neighbor Ext Add");

        break;

    default:
        neighbor = nullptr;
        break;
    }

    if (neighbor == nullptr)
    {
        return;
    }
#if !OPENTHREAD_RADIO
    else
    {
        logTpc("TPC:neighbor state=%d", neighbor->GetState());
    }
#endif

    if (neighbor != nullptr && neighbor->IsStateValid())
    {
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
        logMode("handle ack, neighbor's mode", neighbor->GetPowerControlMode());
#endif
        // Check if power control is enabled for the neighbor
        if ((neighbor->GetPowerControlMode() == kPowerControlDisable))
        {
            return;
        }

        if (aAckStatus == kErrorNone)
        {
            neighbor->UpdateAckStatus(true);
            // If Encoded ACK is enabled and the neighbor requested for Encoded ACK
            if ((mMode & kPowerControlRxEncodedAck) && (neighbor->GetPowerControlMode() & kPowerControlTxEncodedAck))
            {
                HandleEncodedAck(neighbor, aFCF);
            }
        }
        else if (aAckStatus == kErrorNoAck)
        {
            // If we lost an ACK, we need to step-up the power
            neighbor->UpdateAckStatus(false);
            HandleLostAck(neighbor);
        }
    }
    else if (!neighbor->IsStateValid())
    {
        neighbor->SetTxPower(mMaxTxPower);
    }
}

void PowerControl::HandleLostAck(NeighborType *neighbor)
{
    if (neighbor != nullptr)
    {
        uint8_t acksLostInWindow = neighbor->GetLostAckCountInWindow();

        if (acksLostInWindow > 0 && acksLostInWindow < 5) // we maintain window of size 4
        {
            uint8_t updatedTxPower = neighbor->GetTxPower() + mStepUpAckLost;
            if (updatedTxPower > mMaxTxPower)
            {
                updatedTxPower = mMaxTxPower;
            }

            neighbor->SetTxPower(updatedTxPower);
            neighbor->SetIntegral(100 * updatedTxPower / mKi);

            logTpc("TPC:HandleLostAck TxPower= %d  ", updatedTxPower);
        }
    }
}

void PowerControl::HandleEncodedAck(NeighborType *neighbor, uint16_t codedByte)
{
    // Bits 5 & 7 are the coded bits in ACK Frame Control Field
    uint8_t codeWord = (((codedByte >> 5) & 0x1) | ((codedByte >> 6) & 0x2));
    if (codeWord == 3) // If we get consequtive feedback to reduce power, we can trigger a LinkMetrics probe
    {
        // round(mAlpha*codeWord+(1-mAlpha)*neighbor->mCodeWord);
        neighbor->SetCodeWordCount(neighbor->GetCodeWordCount() + 1);
    }
    else
    {
        neighbor->SetCodeWordCount(0);
    }

    // TODO: Trigger Link Metrics Probe if we get continuous feedback to reduce power
    uint8_t updatedTxPower = PILoop(neighbor, codeWord);
    neighbor->SetTxPower(updatedTxPower);

    logTpc("TPC:HandleEncodedAck TxPower=%d codedByte=%d codeWord=%d ", updatedTxPower, codedByte, codeWord);
}

void PowerControl::HandleLinkMetricsReport(uint16_t aRloc16, uint8_t aLinkMargin)
{
    NeighborType *neighbor;
    int           excessPower;
    int           newPower;
    int           finalPower;

#if OPENTHREAD_RADIO
    neighbor = mTable.FindEntry(aRloc16);
#else
    neighbor = GetInstance().Get<NeighborTable>().FindNeighbor(aRloc16, Neighbor::kInStateValid);
#endif

    if (neighbor)
    {
        logTpc("TPC:HandleLinkMetrics OldPower=%d", neighbor->GetTxPower());

        excessPower = aLinkMargin - (aRloc16 & 0x01FF ? mEndDeviceLinkMarginSetPoint : mRouterLinkMarginSetPoint);

        // Get Link Budget above the current Tx Power
        // weighted average of current power & excess power
        newPower   = neighbor->GetTxPower() - excessPower;
        finalPower = (mAlpha * newPower + (100 - mAlpha) * neighbor->GetTxPower()) / 100;

        logTpc("TPC:excessPower=%d oldPower=%d newPower=%d mTxPower=%d ", excessPower, neighbor->GetTxPower(), newPower,
               finalPower);

        if (finalPower < mMinTxPower)
        {
            finalPower = mMinTxPower;
        }
        else if (finalPower > mMaxTxPower)
        {
            finalPower = mMaxTxPower;
        }

        neighbor->SetTxPower(finalPower);
        neighbor->SetIntegral(100 * finalPower / mKi);

        logTpc("TPC:HandleLinkMetrics NewPower=%d RSSI=%d", neighbor->GetTxPower());
    }
}

#if !OPENTHREAD_RADIO
void PowerControl::NeighborInit(NeighborType *neighbor)
{
    if (NULL != neighbor)
    {
        ResetNeighbor(neighbor);
        logTpc("TPC:NeighborInit MaxTxPower=%d RLOC=%d ", mMaxTxPower, neighbor->GetRloc16());
    }
}
#endif

uint8_t PowerControl::PILoop(NeighborType *neighbor, uint8_t codeWord)
{
    // Here we apply non-linear correction based on feedback.
    // State is maintained in the Loop Integral,
    // which needs to be updated if Tx power is changed by Lost ACK or Link Probe

    int power = (mKp * mErrorGain[codeWord] + mKi * neighbor->GetIntegral()) / 100;

    logTpc("TPC:mKp*mErrorGain %d  ", (mKp * mErrorGain[codeWord]));
    logTpc("TPC:mKi*GetIntegral %d  ", (mKi * neighbor->GetIntegral()));

    int integral = neighbor->GetIntegral() + mErrorGain[codeWord];

    logTpc("TPC:Integral %d  ", (integral));

    if (power > mMaxTxPower)
    {
        power = mMaxTxPower;
        neighbor->SetIntegral(100 * mMaxTxPower / mKi);
        logTpc("TPC:Integral Updated to Max %d  ", (neighbor->GetIntegral()));
    }
    else if (power < mMinTxPower)
    {
        power = mMinTxPower;
        neighbor->SetIntegral(100 * mMinTxPower / mKi);
        logTpc("TPC:Integral Updated to Min %d  ", neighbor->GetIntegral());
    }
    else
    {
        neighbor->SetIntegral(integral);
        logTpc("TPC:Integral Updated to %d  ", neighbor->GetIntegral());
    }

    logTpc("TPC:PI Loop Updated Tx power to %d  ", (power));

    return power;
}

void PowerControl::SetDefaultParams()
{
    mRouterLinkMarginSetPoint    = kRouterLinkMarginSetPoint;
    mEndDeviceLinkMarginSetPoint = kEndDeviceLinkMarginSetPoint;
    mKp                          = kKp;
    mKi                          = kKi;
    mAlpha                       = kAlpha;
    mMode                        = kPowerControlDisable;

    mErrorGain[0] = kErrorGain0;
    mErrorGain[1] = kErrorGain1;
    mErrorGain[2] = kErrorGain2;
    mErrorGain[3] = kErrorGain3;
}

#if OPENTHREAD_RADIO
void PowerControl::ResetNeighbors(void)
{
    mTable.ForEach([this](PowerControlTable::Entry &aEntry) {
        aEntry.mTxPower  = GetMaxTxPower();
        aEntry.mIntegral = 100 * GetMaxTxPower() / mKi;
        aEntry.ResetPowerControlData();
    });
}
#else
void PowerControl::ResetNeighbors(void)
{
    otNeighborInfo neighborInfo;
    otNeighborInfoIterator iterator = OT_NEIGHBOR_INFO_ITERATOR_INIT;

    otLogDebgUtil("TPC:ResetNeighbors");
    while (otThreadGetNextNeighborInfo(&GetInstance(), &iterator, &neighborInfo) == kErrorNone)
    {
        NeighborType *neighbor =
            GetInstance().Get<NeighborTable>().FindNeighbor(neighborInfo.mRloc16, Neighbor::kInStateValid);
        if (neighbor != NULL)
        {
            neighbor->SetTxPower(GetMaxTxPower());
            neighbor->SetIntegral(100 * GetMaxTxPower() / mKi);
            neighbor->ResetPowerControlData();
        }
    }
}
#endif

void PowerControl::ResetNeighbor(NeighborType *neighbor)
{
    otLogDebgUtil("TPC:Reset Neighbor %d", neighbor->GetRloc16());
    neighbor->SetTxPower(GetMaxTxPower());
    neighbor->SetIntegral(100 * GetMaxTxPower() / mKi);
    neighbor->ResetPowerControlData();
}

void PowerControl::UpdateNettEnergySavingStatistics(uint8_t aFrameLength, int8_t aTxPower)
{
    mEnergyStats += (aFrameLength + TPC_PHY_OVERHEAD) * GetCurrent(aTxPower);
    mTxCountStats++;
    mTotalPowerStats += aTxPower;
    mByteCountStats += aFrameLength + TPC_PHY_OVERHEAD;

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    ot::Utils::PowerControlStats &powerControlStats =
        static_cast<Instance *>(&GetInstance())->Get<ot::Utils::PowerControlStats>();

    if (GetMode() != kPowerControlDisable)
    {
        powerControlStats.UpdateFrameTxPowerHistogram(aTxPower);
    }
#endif

    // Update total TPC statistics.
    if (mTxCountStats == OPENTHREAD_CONFIG_MAC_POWER_CONTROL_DEBUG_LOG_FRAME_NUMBER)
    {
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE || OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
        if (GetMode() != kPowerControlDisable)
        {
            int8_t  avgPower;
            uint8_t maxCurrent;
            uint8_t efficiency;
            uint8_t batteryLife;

            avgPower   = mTotalPowerStats / mTxCountStats;
            maxCurrent = GetMaxCurrent();
            efficiency = ((mByteCountStats * maxCurrent) - mEnergyStats) /
                         (mByteCountStats * maxCurrent); // (Imax*∑B)-∑Bytes*I(p)/(Imax*∑B)

            batteryLife = (mByteCountStats * maxCurrent) / mEnergyStats;

            OT_UNUSED_VARIABLE(avgPower);
            OT_UNUSED_VARIABLE(efficiency);
            OT_UNUSED_VARIABLE(batteryLife);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
            powerControlStats.UpdateNeighborTxPowerHistogram(avgPower);
            powerControlStats.UpdateNeighborEnergySavingsFactorHistogram((mByteCountStats * maxCurrent * 2) /
                                                                         mEnergyStats);
#endif

            logTpc("TPC:TotalPower(%d) TxCount(%d) Average(%d)", mTotalPowerStats, mTxCountStats, avgPower);
            logTpc("TPC:Average Energy(%d) ByteCount(%d), Efficiency(%d) BatteryLife(%d)", mEnergyStats,
                   mByteCountStats, efficiency, batteryLife);
        }
#endif // OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE || OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
        mEnergyStats     = 0; // Bytes*I(mA) for energy calculation, 16bytes PHY overhead
        mTxCountStats    = 0;
        mTotalPowerStats = 0;
        mByteCountStats  = 0;
    }
}

} // namespace Utils
} // namespace ot

#endif // OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
