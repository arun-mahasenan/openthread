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
 *   This file includes definitions for power control statistics module.
 */

#ifndef POWER_CONTROL_STATS_HPP_
#define POWER_CONTROL_STATS_HPP_

#if OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE

namespace ot {
namespace Utils {

class PowerControlStats : public InstanceLocator, private NonCopyable
{
    enum
    {
        /**
         * The minimum and maximum values of Frame Transmit Power Histogram.
         *
         */
        kFrameTxPowerHistogramMin = 0,
        kFrameTxPowerHistogramMax = 16,

        /**
         * The minimum and maximum values of Neighbor Transmit Power Histogram.
         *
         */
        kNeighborTxPowerHistogramMin = 0,
        kNeighborTxPowerHistogramMax = 16,

        /**
         * The minimum and maximum values of Neighbor Energy Savings Factor Histogram.
         *
         */
        kNeighborEnergySavingsFactorMin = 2,
        kNeighborEnergySavingsFactorMax = 20,
    };

public:
    /**
     * This constructor initializes the object.
     *
     * @param[in]  aInstance     A reference to the OpenThread instance.
     *
     */
    explicit PowerControlStats(Instance &aInstance);

    /**
     * This method gets Frame Transmit Power Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     * @param[out]  aArray    A pointer to the histogram array.
     * @param[out]  aCount    A pointer to where the histogram array length is placed.
     *
     */
    void GetFrameTxPowerHistogram(uint32_t *aArray, uint8_t *aCount);

    /**
     * This method reset Frame Transmit Power Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     */
    void ResetFrameTxPowerHistogram(void);

    /**
     * This method update Frame Transmit Power Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     * @param[in]   aTxPower  A new value added to histogram.
     *                        Only values from 0dBm to 16dBm are allowed
     *
     */
    void UpdateFrameTxPowerHistogram(int8_t aTxPower);

    /**
     * This method gets Neighbor Transmit Power Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     * @param[out]  aArray    A pointer to the histogram array.
     * @param[out]  aCount    A pointer to where the histogram array length is placed.
     *
     */
    void GetNeighborTxPowerHistogram(uint32_t *aArray, uint8_t *aCount);

    /**
     * This method reset Neighbor Transmit Power Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     */
    void ResetNeighborTxPowerHistogram(void);

    /**
     * This method update Neighbor Transmit Power Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     * @param[in]   aTxPower  A new value added to histogram.
     *                        Only values from 0dBm to 16dBm are allowed
     *
     */
    void UpdateNeighborTxPowerHistogram(int8_t aTxPower);

    /**
     * This method gets Neighbor Energy Savings Factor Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     * @param[out]  aArray    A pointer to the histogram array.
     * @param[out]  aCount    A pointer to where the histogram array length is placed.
     *
     */
    void GetNeighborEnergySavingsFactorHistogram(uint32_t *aArray, uint8_t *aCount);

    /**
     * This method reset Neighbor Energy Savings Factor Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     */
    void ResetNeighborEnergySavingsFactorHistogram(void);

    /**
     * This method update Neighbor Energy Savings Factor Histogram of the Power Control Algorithm.
     * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
     *
     * @param[in] aEnergyFactor  A new value added to histogram.
     *                           The NeighborEnergySavingsFactor is multiplied by 2 and
     *                           its range is from 2 to 20.
     */
    void UpdateNeighborEnergySavingsFactorHistogram(uint8_t aEnergyFactor);

private:
#if OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE
    uint32_t mFrameTxPowerHistogramData[OPENTHREAD_CONFIG_POWER_CONTROL_FRAME_TXPOWER_HISTOGRAM_SIZE];
    uint32_t mNeighborTxPowerHistogramData[OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_TXPOWER_HISTOGRAM_SIZE];
    uint32_t mNeighborEnergySavingsFactorHistogramData
        [OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_ENERGY_SAVINGS_FACTOR_HISTOGRAM_SIZE];
#endif
};

/**
 * @}
 *
 */

} // namespace Utils
} // namespace ot

#endif // OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE

#endif // POWER_CONTROL_STATS_HPP_
