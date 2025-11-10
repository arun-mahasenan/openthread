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
 *   This file implements the OpenThread Transmit power control APIs.
 */

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
#include <utils/power_control.hpp>
#include <utils/power_control_probe_sender.hpp>
#include <utils/power_control_stats.hpp>
#include <openthread/power_control.h>

#include "instance/instance.hpp"
#include "common/locator_getters.hpp"

using namespace ot;

#if !OPENTHREAD_RADIO && \
    (OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE)
bool otPowerControlIsProbingEnabled(otInstance *aInstance)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    return instance.Get<ot::Utils::PowerControlProbeSender>().IsRunning();
}

void otPowerControlProbingStart(otInstance *aInstance)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    instance.Get<ot::Utils::PowerControlProbeSender>().Start();
}

void otPowerControlProbingStop(otInstance *aInstance)
{
    Instance &instance = *static_cast<Instance *>(aInstance);

    instance.Get<ot::Utils::PowerControlProbeSender>().Stop();
}

otError otPowerControlSetLinkMetricsProbeInterval(otInstance *aInstance, uint32_t value)
{
    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlProbeSender>().SetLinkMetricsProbeInterval(
        value);
}

uint32_t otPowerControlGetLinkMetricsProbeInterval(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlProbeSender>().GetLinkMetricsProbeInterval();
}
#endif

#if OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE

otError otPowerControlEnable(otInstance *aInstance, bool aEnable)
{
    ot::Utils::PowerControl &powerControl = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>();

    return aEnable ? powerControl.Start() : powerControl.Stop();
}

otError otPowerControlSetMode(otInstance *aInstance, PowerControlMode aMode)
{
    ot::Utils::PowerControl &powerControl = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>();

#if !OPENTHREAD_RADIO && \
    (OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE)
    ot::Utils::PowerControlProbeSender &probeSender =
        static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlProbeSender>();

    if (aMode & kPowerControlSingleProbeLinkMetrics)
    {
        probeSender.Start();
    }
    else
    {
        probeSender.Stop();
    }
#endif

    return powerControl.SetMode(aMode);
}

PowerControlMode otPowerControlGetMode(otInstance *aInstance)
{
    ot::Utils::PowerControl &powerControl = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>();

    return powerControl.GetMode();
}

void otPowerControlSetErrorGain(otInstance *aInstance, int8_t aGain[])
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetErrorGain(aGain);
}

void otPowerControlSetErrorGainAtIndex(otInstance *aInstance, int8_t aGain, uint8_t index)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetErrorGainAtIndex(aGain, index);
}

int8_t *otPowerControlGetErrorGain(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetErrorGain();
}

int8_t otPowerControlGetErrorGainAtIndex(otInstance *aInstance, uint8_t index)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetErrorGainAtIndex(index);
}

void otPowerControlSetRouterLinkMarginSetPoint(otInstance *aInstance, uint8_t aValue)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetRouterLinkMarginSetPoint(aValue);
}

void otPowerControlSetEndDeviceLinkMarginSetPoint(otInstance *aInstance, uint8_t aValue)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetEndDeviceLinkMarginSetPoint(aValue);
}

uint8_t otPowerControlGetRouterLinkMarginSetPoint(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetRouterLinkMarginSetPoint();
}

uint8_t otPowerControlGetEndDeviceLinkMarginSetPoint(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetEndDeviceLinkMarginSetPoint();
}

void otPowerControlSetProportionalGain(otInstance *aInstance, int32_t aValue)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetProportionalGain(aValue);
}

int32_t otPowerControlGetProportionalGain(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetProportionalGain();
}

void otPowerControlSetIntegralGain(otInstance *aInstance, int32_t aValue)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetIntegralGain(aValue);
}

int32_t otPowerControlGetIntegralGain(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetIntegralGain();
}

void otPowerControlSetWeightAverage(otInstance *aInstance, uint8_t aValue)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetWeightAverage(aValue);
}

uint8_t otPowerControlGetWeightAverage(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetWeightAverage();
}

void otPowerControlResetParams(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetDefaultParams();
}

uint8_t otPowerControlGetStepUpAckLost(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetStepUpAckLost();
}

void otPowerControlSetStepUpAckLost(otInstance *aInstance, uint8_t aValue)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetStepUpAckLost(aValue);
}
otError otPowerControlHandleLinkMetricsProbe(otInstance *aInstance, uint16_t aRloc16, uint8_t aLinkMargin)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().HandleLinkMetricsReport(aRloc16, aLinkMargin);

    return OT_ERROR_NONE;
}

#if OPENTHREAD_RADIO
otError otPowerControlAddTableEntry(otInstance *        aInstance,
                                    otShortAddress      aShortAddr,
                                    const otExtAddress *aExtAddr,
                                    uint8_t             aMode,
                                    bool                aIsRxOnWhenIdle)
{
    OT_UNUSED_VARIABLE(aInstance);

    const Mac::ExtAddress &extAddr = *static_cast<const Mac::ExtAddress *>(aExtAddr);

    return static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetTable().AddEntry(
        aShortAddr, extAddr, aMode, aIsRxOnWhenIdle);
}

otError otPowerControlRemoveTableEntry(otInstance *aInstance, otShortAddress aShortAddr)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    Utils::PowerControlTable &table = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetTable();
    Utils::PowerControlTable::Entry *entry = table.FindEntry(aShortAddr);

    VerifyOrExit(entry != nullptr, error = OT_ERROR_NOT_FOUND);

    table.RemoveEntry(*entry);

exit:
    return error;
}

void otPowerControlSetAttachmentStatus(otInstance *aInstance, bool aIsAttached)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().SetAttachmentStatus(aIsAttached);
}

otError otPowerControlSetRxOnWhenIdle(otInstance *aInstance, uint16_t aRloc16, bool aIsRxOnWhenIdle)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    Utils::PowerControlTable &table = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>().GetTable();
    Utils::PowerControlTable::Entry *entry = table.FindEntry(aRloc16);

    VerifyOrExit(entry != nullptr, error = OT_ERROR_NOT_FOUND);

    entry->SetRxOnWhenIdle(aIsRxOnWhenIdle);

exit:
    return error;
}
#else
void otPowerControlSetAttachmentStatus(otInstance *aInstance, bool aIsAttached)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aIsAttached);

    // Intentionally empty
}

otError otPowerControlSetRxOnWhenIdle(otInstance *aInstance, uint16_t aRloc16, bool aIsRxOnWhenIdle)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aRloc16);
    OT_UNUSED_VARIABLE(aIsRxOnWhenIdle);

    return OT_ERROR_NONE;
}
#endif // OPENTHREAD_RADIO

otError otPowerControlNeighborAdded(otInstance *        aInstance,
                                    uint16_t            aRloc16,
                                    const otExtAddress *aExtAddress,
                                    uint8_t             aMode,
                                    bool                aIsRxOnWhenIdle)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aMode);
    OT_UNUSED_VARIABLE(aIsRxOnWhenIdle);

    otError                  error        = OT_ERROR_NONE;
    ot::Utils::PowerControl &powerControl = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>();

    if ((aMode & kPowerControlRxEncodedAck) && (powerControl.GetMode() & kPowerControlTxEncodedAck))
    {
        SuccessOrExit(error = otPlatRadioAddLmeAckShortEntry(aInstance, aRloc16));
        SuccessOrExit(error = otPlatRadioAddLmeAckExtEntry(aInstance, aExtAddress));
    }

exit:
    return error;
}

otError otPowerControlNeighborRemoved(otInstance *        aInstance,
                                      uint16_t            aRloc16,
                                      const otExtAddress *aExtAddress,
                                      uint8_t             aMode)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError                  error        = OT_ERROR_NONE;
    ot::Utils::PowerControl &powerControl = static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControl>();

    if ((aMode & kPowerControlRxEncodedAck) && (powerControl.GetMode() & kPowerControlTxEncodedAck))
    {
        SuccessOrExit(error = otPlatRadioClearLmeAckExtEntry(aInstance, aExtAddress));
        SuccessOrExit(error = otPlatRadioClearLmeAckShortEntry(aInstance, aRloc16));
    }

exit:
    return error;
}

void otPowerControlGetFrameTxPowerHistogram(otInstance *aInstance, uint32_t *aArray, uint8_t *aCount)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlStats>().GetFrameTxPowerHistogram(aArray, aCount);
}

void otPowerControlResetFrameTxPowerHistogram(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlStats>().ResetFrameTxPowerHistogram();
}

void otPowerControlGetNeighborTxPowerHistogram(otInstance *aInstance, uint32_t *aArray, uint8_t *aCount)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlStats>().GetNeighborTxPowerHistogram(aArray, aCount);
}

void otPowerControlResetNeighborTxPowerHistogram(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlStats>().ResetNeighborTxPowerHistogram();
}

void otPowerControlGetNeighborEnergySavingsFactorHistogram(otInstance *aInstance, uint32_t *aArray, uint8_t *aCount)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlStats>().GetNeighborEnergySavingsFactorHistogram(
        aArray, aCount);
}

void otPowerControlResetNeighborEnergySavingsFactorHistogram(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    static_cast<Instance *>(aInstance)->Get<ot::Utils::PowerControlStats>().ResetNeighborEnergySavingsFactorHistogram();
}

#endif // OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE
#endif // OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
