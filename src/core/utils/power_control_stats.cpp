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
 *   This file implements the transmit power control statistics functionality.
 */

#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
#include "power_control.hpp"
#endif

#include <math.h>
#include "common/code_utils.hpp"
#include "instance/instance.hpp"
//#include "common/locator_getters.hpp"
#include "common/logging.hpp"

/** Counts number of elements inside the array
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#if OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE

namespace ot {
namespace Utils {

PowerControlStats::PowerControlStats(Instance &aInstance)
    : InstanceLocator(aInstance)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    memset(&mFrameTxPowerHistogramData, 0, sizeof(mFrameTxPowerHistogramData));
    memset(&mNeighborTxPowerHistogramData, 0, sizeof(mNeighborTxPowerHistogramData));
    memset(&mNeighborEnergySavingsFactorHistogramData, 0, sizeof(mNeighborEnergySavingsFactorHistogramData));
#endif
}

void PowerControlStats::GetFrameTxPowerHistogram(uint32_t *aArray, uint8_t *aCount)
{
    OT_UNUSED_VARIABLE(aArray);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    for (uint8_t i = 0; i < ARRAY_SIZE(mFrameTxPowerHistogramData); i++)
    {
        aArray[i] = mFrameTxPowerHistogramData[i];
    }
    *aCount = ARRAY_SIZE(mFrameTxPowerHistogramData);
#else
    *aCount = 0;
#endif
}

void PowerControlStats::ResetFrameTxPowerHistogram(void)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    memset(&mFrameTxPowerHistogramData, 0, sizeof(mFrameTxPowerHistogramData));
#endif
}

void PowerControlStats::UpdateFrameTxPowerHistogram(int8_t aTxPower)
{
    OT_UNUSED_VARIABLE(aTxPower);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    if ((aTxPower >= kFrameTxPowerHistogramMin) && (aTxPower <= kFrameTxPowerHistogramMax))
    {
        mFrameTxPowerHistogramData[aTxPower]++;
    }
#endif
}

void PowerControlStats::GetNeighborTxPowerHistogram(uint32_t *aArray, uint8_t *aCount)
{
    OT_UNUSED_VARIABLE(aArray);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    for (uint8_t i = 0; i < ARRAY_SIZE(mNeighborTxPowerHistogramData); i++)
    {
        aArray[i] = mNeighborTxPowerHistogramData[i];
    }
    *aCount = ARRAY_SIZE(mNeighborTxPowerHistogramData);
#else
    *aCount = 0;
#endif
}

void PowerControlStats::ResetNeighborTxPowerHistogram(void)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    memset(&mNeighborTxPowerHistogramData, 0, sizeof(mNeighborTxPowerHistogramData));
#endif
}

void PowerControlStats::UpdateNeighborTxPowerHistogram(int8_t aTxPower)
{
    OT_UNUSED_VARIABLE(aTxPower);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    if ((aTxPower >= kNeighborTxPowerHistogramMin) && (aTxPower <= kNeighborTxPowerHistogramMax))
    {
        mNeighborTxPowerHistogramData[aTxPower]++;
    }
#endif
}

void PowerControlStats::GetNeighborEnergySavingsFactorHistogram(uint32_t *aArray, uint8_t *aCount)
{
    OT_UNUSED_VARIABLE(aArray);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    for (uint8_t i = 0; i < ARRAY_SIZE(mNeighborEnergySavingsFactorHistogramData); i++)
    {
        aArray[i] = mNeighborEnergySavingsFactorHistogramData[i];
    }
    *aCount = ARRAY_SIZE(mNeighborEnergySavingsFactorHistogramData);
#else
    *aCount = 0;
#endif
}

void PowerControlStats::ResetNeighborEnergySavingsFactorHistogram(void)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    memset(&mNeighborEnergySavingsFactorHistogramData, 0, sizeof(mNeighborEnergySavingsFactorHistogramData));
#endif
}

void PowerControlStats::UpdateNeighborEnergySavingsFactorHistogram(uint8_t aEnergyFactor)
{
    OT_UNUSED_VARIABLE(aEnergyFactor);

#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    if ((aEnergyFactor >= kNeighborEnergySavingsFactorMin) && (aEnergyFactor <= kNeighborEnergySavingsFactorMax))
    {
        mNeighborEnergySavingsFactorHistogramData[aEnergyFactor - kNeighborEnergySavingsFactorMin]++;
    }
#endif
}

} // namespace Utils
} // namespace ot

#endif // OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE
