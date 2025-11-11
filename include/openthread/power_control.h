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
 * @brief
 *   This file includes the OpenThread API for power control feature
 */

#ifndef OPENTHREAD_POWER_CONTROL_H_
#define OPENTHREAD_POWER_CONTROL_H_

#include <openthread/instance.h>

#include <openthread/platform/radio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup api-power-control
 *
 * @brief
 *   This module includes functions for transmit power control feature.
 *
 *   The functions in this module are available when power control feature
 *   (`OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE`) is enabled.
 *
 *  Power Control module is used to adapt the transmit power to meet a set link margin
 * @{
 *
 */
/**
 * Power Control Modes Supported.
 * Bit positions used to represent if a mode is enabled or not. 1 means enabled
 * Bit0= Link Metrics Based, Bit1= Encoded ACK based
 */
enum PowerControlMode
{
    ///< Disable Power Control. Always used Max Tx Power
    kPowerControlDisable = 0x00,

    ///< Power Controller updates Tx power based on Single Probe Link Metrics.
    kPowerControlSingleProbeLinkMetrics = 1 << 0,

    ///< Power Controller updates Tx power based on feedback in Encoded ACK
    kPowerControlRxEncodedAck = 1 << 1,

    ///< Power Controller can share LinkMargin feedback in Encoded ACK
    kPowerControlTxEncodedAck = 1 << 2,

    //<Hybrid Power Control mode (LM Probe+ Transmit Encoded ACK + Receive Encoded ACK)
    kPowerControlHybrid = kPowerControlRxEncodedAck | kPowerControlTxEncodedAck
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
                          | kPowerControlSingleProbeLinkMetrics
#endif
};

//  TPC_PHY_OVERHEAD value 8 is sum of 5 Byte Synchronization header + 1 Byte PHY header + 2 Byte PHY footer
#define TPC_PHY_OVERHEAD 8

/**
 * This function enables/disables the Power Control operation.
 *
 * @param[in]  aInstance      A pointer to an OpenThread instance.
 * @param[in]  aEnabled       TRUE to enable/start Power control operation, FALSE to disable/stop it.
 *
 * @retval OT_ERROR_NONE      Power Control state changed successfully
 * @retval OT_ERROR_ALREADY   Power Control is already in the same state.
 *
 */
otError otPowerControlEnable(otInstance *aInstance, bool aEnable);

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
/**
 * This function indicates whether the Power Control operation is enabled and running.
 *
 * @param[in]  aInstance      A pointer to an OpenThread instance.
 *
 * @returns TRUE if the Power Control operation is enabled, FALSE otherwise.
 *
 */
bool otPowerControlIsProbingEnabled(otInstance *aInstance);

void otPowerControlProbingStart(otInstance *aInstance);
void otPowerControlProbingStop(otInstance *aInstance);

/**
 * This method sets the Link Metrics Probing interval.
 * @param[in] aSeconds Probe interval in seconds
 */
otError otPowerControlSetLinkMetricsProbeInterval(otInstance *aInstance, uint32_t aSeconds);

/**
 * This method returns the current Link Metrics Probing interval.
 * @returns aSeconds Probe interval in seconds
 */
uint32_t otPowerControlGetLinkMetricsProbeInterval(otInstance *aInstance);
#endif

/**
 * This method sets the Transmit power control mode.
 *
 * @param[in] aMode
 *            kPowerControlDisable                -> Disable Power Control.
 *            kPowerControlSingleProbeLinkMetrics -> Power Control based on Single Probe Link Metrics.
 *            kPowerControlTxEncodedAck           -> Power Control based on Encoded ACK Tx
 *            kPowerControlRxEncodedAck           -> Power Control based on Encoded ACK Rx
 *            kPowerControlHybrid                 -> Power Control based on EncodedAck & SingleProbeLinkMetrics.
 *
 * @retval OT_ERROR_NONE           Successfully added @p aExtAddress to the filter.
 * @retval OT_ERROR_INVALID_ARGS   Invalid argument provided.
 */
otError otPowerControlSetMode(otInstance *aInstance, PowerControlMode aMode);

/**
 * This method returns the Transmit power control mode.
 *
 * @return   aMode
 *            kPowerControlDisable                -> Disable Power Control.
 *            kPowerControlSingleProbeLinkMetrics -> Power Control based on Single Probe Link Metrics.
 *            kPowerControlTxEncodedAck           -> Power Control based on Encoded ACK Tx
 *            kPowerControlRxEncodedAck           -> Power Control based on Encoded ACK Rx
 *            kPowerControlHybrid                 -> Power Control based on EncodedAck & SingleProbeLinkMetrics.
 */
PowerControlMode otPowerControlGetMode(otInstance *aInstance);

/**
 * This method sets the non-linear Error Gain used for the PI Loop.
 * @param[in] aGain[4]
 */
void otPowerControlSetErrorGain(otInstance *aInstance, int8_t aGain[]);

/**
 * This method sets the non-linear Error Gain used for the PI Loop
 * for a specific index
 * @param[in] aGain[4]
 */
void otPowerControlSetErrorGainAtIndex(otInstance *aInstance, int8_t aGain, uint8_t index);

/**
 * This method gets the non-linear Error Gain used for the PI Loop.
 * @param[in] aGain[4]
 */
int8_t *otPowerControlGetErrorGain(otInstance *aInstance);

/**
 * This method gets the non-linear Error Gain used for the PI Loop for a specific index
 * @param[in] aGain[4]
 */
int8_t otPowerControlGetErrorGainAtIndex(otInstance *aInstance, uint8_t index);

/**
 * This method sets the desired Link Margin to be maintanied at the receiver (which are routers) while doing power
 * control. The PI Controller works to maintain this set-point
 * @param[in] aSetPoint The desired receiver link margin in dB
 */
void otPowerControlSetRouterLinkMarginSetPoint(otInstance *aInstance, uint8_t aValue);

/**
 * This method sets the desired Link Margin to be maintanied at the receiver (which are end-devices) while doing power
 * control. The PI Controller works to maintain this set-point
 * @param[in] aSetPoint The desired receiver link margin in dB
 */
void otPowerControlSetEndDeviceLinkMarginSetPoint(otInstance *aInstance, uint8_t aValue);
/**
 * This method returns the desired Link Margin to be maintanied at the receiver (which are routers) while doing power
 * control.
 * @param[in] aSetPoint The desired receiver link margin in dB
 */
uint8_t otPowerControlGetRouterLinkMarginSetPoint(otInstance *aInstance);

/**
 * This method returns the desired Link Margin to be maintanied at the receiver (which are end-devices) while doing
 * power control.
 * @param[in] aSetPoint The desired receiver link margin in dB
 */
uint8_t otPowerControlGetEndDeviceLinkMarginSetPoint(otInstance *aInstance);
/**
 * This method sets the proportional gain of the PI Controller (Kp)
 * @param[in] aGain
 */
void otPowerControlSetProportionalGain(otInstance *aInstance, int32_t aValue);

/**
 * This method returns the proportional gain of the PI Controller (Kp)
 * @returns aGain
 */
int32_t otPowerControlGetProportionalGain(otInstance *aInstance);

/**
 * This method sets the integral gain of the PI Controller (Ki)
 * @param[in] aGain
 */
void otPowerControlSetIntegralGain(otInstance *aInstance, int32_t aValue);

/**
 * This method returns the integral gain of the PI Controller (Ki)
 * @param[in] aGain
 */
int32_t otPowerControlGetIntegralGain(otInstance *aInstance);

/**
 * This method sets the weight in percent used for the current sample,
 * while averaing transmit power during a Link Metrics Update.
 * @param[in]  0 < aWeight < 100
 */
void otPowerControlSetWeightAverage(otInstance *aInstance, uint8_t aValue);

/**
 * This method returns the weight in percent used for the current sample,
 * while averaing transmit power during a Link Metrics Update.
 * @returns aWeight
 */
uint8_t otPowerControlGetWeightAverage(otInstance *aInstance);

/**
 * Reset PI Controller State parameters.
 *
 */
void otPowerControlResetParams(otInstance *aInstance);

/**
 * Gets the step-size increase in Tx Power when an ACK is lost.
 *
 */
uint8_t otPowerControlGetStepUpAckLost(otInstance *aInstance);

/**
 * Sets the step-size increase in Tx Power when an ACK is lost.
 *
 */
void otPowerControlSetStepUpAckLost(otInstance *aInstance, uint8_t aValue);

/**
 * Handles Link Metrics report.
 *
 */
otError otPowerControlHandleLinkMetricsProbe(otInstance *aInstance, uint16_t aRloc16, uint8_t aLinkMargin);

/**
 * Handles addition of a new neighbor.
 *
 */
otError otPowerControlNeighborAdded(otInstance *        aInstance,
                                    uint16_t            aRloc16,
                                    const otExtAddress *aExtAddress,
                                    uint8_t             aMode,
                                    bool                aIsRxOnWhenIdle);

/**
 * Handles removal of a new neighbor.
 *
 */
otError otPowerControlNeighborRemoved(otInstance *        aInstance,
                                      uint16_t            aRloc16,
                                      const otExtAddress *aExtAddress,
                                      uint8_t             aMode);

/**
 * Set flag when all neighbor TPC table entries are removed on leave
 *
 * @param[in]   aRemoveAllTpcEntries     Flag to set when all TPC neighbor entries are removed on thread leave
 */
void otPowerControlSetFlagTpcEntriesRemove(bool aRemoveAllTpcEntries);

/**
 * Return flag value that indicates if all neighbor TPC table entries were removed on thread leave
 *
 */
bool otPowerControlGetFlagTpcEntriesRemoved(void);

/**
 * Report attachment status to Power Control module.
 *
 */
void otPowerControlSetAttachmentStatus(otInstance *aInstance, bool aIsAttached);

/**
 * Set Is-rx-on-when-idle status of a neighbor in Power Control.
 *
 */
otError otPowerControlSetRxOnWhenIdle(otInstance *aInstance, uint16_t aRloc16, bool aIsRxOnWhenIdle);

#if OPENTHREAD_RADIO
/**
 * Add new entry to Power Control table.
 *
 */
otError otPowerControlAddTableEntry(otInstance *        aInstance,
                                    otShortAddress      aShortAddr,
                                    const otExtAddress *aExtAddr,
                                    uint8_t             aMode,
                                    bool                aIsRxOnWhenIdle);

/**
 * Remove entry from Power Control table.
 *
 */
otError otPowerControlRemoveTableEntry(otInstance *aInstance, otShortAddress aShortAddr);
#endif

/**
 * This method gets Frame Transmit Power Histogram of the Power Control Algorithm.
 *
 * This function is valid when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE configuration is enabled.
 *
 * @param[in]   aInstance     A pointer to an OpenThread instance.
 * @param[in]   aArray        A pointer to the histogram of Frame Transmit Power (in a form of an array).
 * @param[out]  aCount        A pointer to where the size of returned histogram array is placed.
 */
void otPowerControlGetFrameTxPowerHistogram(otInstance *aInstance, uint32_t *aArray, uint8_t *aCount);

/**
 * This method reset Frame Transmit Power Histogram of the Power Control Algorithm.
 * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
 *
 * @param[in]   aInstance     A pointer to an OpenThread instance.
 */
void otPowerControlResetFrameTxPowerHistogram(otInstance *aInstance);

/**
 * This method gets Neighbor Transmit Power Histogram of the Power Control Algorithm.
 *
 * This function is valid when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE configuration is enabled.
 *
 * @param[in]   aInstance     A pointer to an OpenThread instance.
 * @param[out]  aArray        A pointer to the histogram of Neighbor Transmit Power (in a form of an array).
 * @param[out]  aCount        A pointer to where the size of returned histogram array is placed.
 *
 */
void otPowerControlGetNeighborTxPowerHistogram(otInstance *aInstance, uint32_t *aArray, uint8_t *aCount);

/**
 * This method reset Neighbor Transmit Power Histogram of the Power Control Algorithm.
 * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
 *
 * @param[in]   aInstance     A pointer to an OpenThread instance.
 */
void otPowerControlResetNeighborTxPowerHistogram(otInstance *aInstance);

/**
 * This method gets Neighbor Energy Savings Factor Histogram of the Power Control Algorithm.
 *
 * This function is valid when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE configuration is enabled.
 *
 * @param[in]   aInstance     A pointer to an OpenThread instance.
 * @param[out]  aArray        A pointer to the histogram of Neighbor Energy Savings Factor (in a form of an array).
 * @param[out]  aCount        A pointer to where the size of returned histogram array is placed.
 *
 */
void otPowerControlGetNeighborEnergySavingsFactorHistogram(otInstance *aInstance, uint32_t *aArray, uint8_t *aCount);

/**
 * This method reset Neighbor Energy Savings Factor Histogram of the Power Control Algorithm.
 * Available when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
 *
 * @param[in]   aInstance     A pointer to an OpenThread instance.
 */
void otPowerControlResetNeighborEnergySavingsFactorHistogram(otInstance *aInstance);

/**
 * @}
 *
 */

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_POWER_CONTROL_H_
