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
 *   This file includes definitions for power control module.
 */

#ifndef POWER_CONTROL_HPP_
#define POWER_CONTROL_HPP_

#include "openthread-core-config.h"

#include <openthread/link_metrics.h>
#include <openthread/platform/radio.h>
#include "common/locator.hpp"
#include "common/non_copyable.hpp"
#include "common/timer.hpp"
#include "mac/mac.hpp"
#include "mac/mac_frame.hpp"
#if OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE
#include "power_control_table.hpp"
#endif

#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
#include <openthread/power_control.h>
#endif

#include "radio/radio.hpp"

namespace ot {
namespace Utils {

#if OPENTHREAD_RADIO
using NeighborType = PowerControlTable::Entry;
#else
using NeighborType = Neighbor;
#endif

/**
 * @addtogroup utils-power-control
 *
 * @brief
 *   This module enables transmit power control to maintain a set link margin at the receiver.
 *
 * @{
 */

#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE && (OPENTHREAD_MTD || OPENTHREAD_FTD)

/**
 * This class implements the transmit power control logic.
 *
 * Power Contoller will periodically monitor the opportunity to adapt
 * transmit power to meet a given link margin at the receiver.
 *
 */
class PowerControl : public InstanceLocator, private NonCopyable
{
private:
    uint16_t mTxCountStats;
    uint16_t mTotalPowerStats;
    uint16_t mByteCountStats;
    uint32_t mEnergyStats;

public:
    /**
     * Power Control Modes Supported.
     * Bit positions used to represent if a mode is enabled or not. 1 means enabled
     * Bit0= Link Metrics Based, Bit1= Encoded ACK based
     */

    enum
    {
        /**
         * The default Link Metric Set Point to be mainted at the receiver in dB for routers.
         *
         */
        kRouterLinkMarginSetPoint = 30,

        /**
         * The default Link Metric Set Point to be mainted at the receiver in dB for end devices.
         *
         */
        kEndDeviceLinkMarginSetPoint = 30,

        /**
         * The default Proportional Gain of the PI Loop.
         * Multiplied by 100 to avoid floating point operations
         */
        kKp = -30, // 100*0.3

        /**
         * The default Integral Gain of the PI Loop.
         * Multiplied by 100 to avoid floating point operations
         */
        kKi = -1, // 100*-0.01;

        /**
         * The default Error Gain of the PI Loop.
         *
         */
        kErrorGain0 = -52,
        kErrorGain1 = -16,
        kErrorGain2 = -2,
        kErrorGain3 = 4,

        /**
         * The default power control mode.
         *
         */
        kMode = PowerControlMode::kPowerControlDisable
    };

    /**
     * The default weight for averaging function in percent.
     *
     */
    uint8_t kAlpha = 50;

    /**
     * This constructor initializes the object.
     *
     * @param[in]  aInstance     A reference to the OpenThread instance.
     *
     */
    explicit PowerControl(Instance &aInstance);

    /**
     * This method starts the Power Control operation.
     *
     * @retval kErrorNone        Power Control started successfully.
     * @retval kErrorAlready     Power Control has already been started.
     *
     */
    Error Start(void);

    /**
     * This method stops the Power Control operation.
     *
     * @retval kErrorNone       Power Control stopped successfully.
     * @retval kErrorAlready    Power Control has already been stopped.
     *
     */
    Error Stop(void);

    /**
     * This is the call back function to adjust the tranmsit power when an Acknowledgemnet is lost from a neighbor.
     *
     * @param[in]  neighbor           Address of neighbor.
     * @returns void.
     */
    void HandleLostAck(NeighborType *neighbor);

    /**
     * This is the call back function to adjust the tranmsit power when an Encoded Acknowledgemnet is received from a
     * neighbor.
     *
     * @param[in]  neighbor           Address of neighbor.
     * @param[in]  aFCF               Frame Control Field of ACK(Bit 5 & Bit 7).
     * @param[in]  aAckStatus         Ack status- success or ACK timeout.
     * @returns void.
     */
    void HandleAck(ot::Mac::Address &aAddress, uint16_t aFCF, Error aAckStatus);

    /**
     * This method converts the number of Acknowledgemnets lost in a window to an Error Gain to control the PI Loop.
     *
     * @param[in]  ackLostInWindow         Number of Acks lost in the sliding window.
     * @returns    Error Gain to apply for correcting the Transmit power.
     */
    int8_t AckLostToCodeWord(uint8_t ackLostInWindow);

    /**
     * This method decodes the feedback in EncodedAck, maps to Error Gain and calls the PI Loop for power control.
     *
     * @param[in]  neighbor         Neighbor to apply the power control adjustment.
     * @param[in]  aFCF             Frame Control field of ACK, in which feedback is encoded.
     * @returns    void
     */
    void HandleEncodedAck(NeighborType *neighbor, uint16_t aFCF);

    /**
     * This method implements the Proportional-Integral Closed Loop based power control.
     *
     * @param[in]  neighbor         Neighbor to apply the power control adjustment.
     * @param[in]  codeWord         Decoded code word about the Link Marging from Neighbor.
     * @returns    adjusted transmit power of the neighbor
     */
    uint8_t PILoop(NeighborType *neighbor, uint8_t codeWord);

    /**
     * This method initializes the neighbor table entry with transmit power control state parameters.
     *
     * @param[in]  neighbor         Neighbor to apply the power control adjustment.
     * @returns    void
     */
    void NeighborInit(NeighborType *neighbor);

    /**
     * This method returns the maximum Transmit power configured for the power control algorithm.
     *
     * @returns    Transmit power in dBm
     */
    uint8_t GetMaxTxPower() { return mMaxTxPower; }

    /**
     * This method sets the maximum Transmit power configured for the power control algorithm.
     *
     * @param[in]    Transmit power in dBm
     */
    void SetMaxTxPower(uint8_t aPower) { mMaxTxPower = aPower; }

    /**
     * This method gets the minimum Transmit power configured for the power control algorithm.
     *
     * @returns Transmit power in dBm
     */
    uint8_t GetMinTxPower() { return mMinTxPower; }

    /**
     * This method sets the minimum Transmit power configured for the power control algorithm.
     *
     * @param[in]    Transmit power in dBm
     */
    void SetMinTxPower(uint8_t aPower) { mMinTxPower = aPower; }

    /**
     * This method sets the Transmit power control mode.
     *
     * @param[in] aMode
     *            kPowerControlDisable                ->  Disable Power Control.
     *            kPowerControlSingleProbeLinkMetrics -> Power Control based on Single Probe Link Metrics.
     *            kPowerControlTxEncodedAck           -> Power Control based on Encoded ACK Tx
     *            kPowerControlRxEncodedAck           -> Power Control based on Encoded ACK Rx
     *            kPowerControlHybrid                 -> Power Control based on EncodedAck & SingleProbeLinkMetrics.
     *
     * @retval kErrorNone              Successfully added @p aExtAddress to the filter.
     * @retval kErrorInvalidArgs       Invalid argument provided.
     */
    Error SetMode(PowerControlMode aMode);

    /**
     * This method returns the Transmit power control mode.
     *
     * @return   aMode
     *            kPowerControlDisable                ->  Disable Power Control.
     *            kPowerControlSingleProbeLinkMetrics -> Power Control based on Single Probe Link Metrics.
     *            kPowerControlTxEncodedAck           -> Power Control based on Encoded ACK Tx
     *            kPowerControlRxEncodedAck           -> Power Control based on Encoded ACK Rx
     *            kPowerControlHybrid                 -> Power Control based on EncodedAck & SingleProbeLinkMetrics.
     */
    PowerControlMode GetMode();

    /**
     * This method sets the non-linear Error Gain used for the PI Loop.
     * @param[in] aGain[4]
     */
    void SetErrorGain(int8_t aGain[]);

    /**
     * This method sets the non-linear Error Gain used for the PI Loop for specific index.
     * @param[in] aGain[4]
     */
    void SetErrorGainAtIndex(int8_t aGain, uint8_t index);

    /**
     * This method gets the non-linear Error Gain used for the PI Loop.
     * @param[in] aGain[4]
     */
    int8_t *GetErrorGain();

    /**
     * This method gets the non-linear Error Gain used for the PI Loop for specific index.
     * @param[in] aGain[4]
     */
    int8_t GetErrorGainAtIndex(uint8_t index);

    /**
     * This method sets the desired Link Margin to be maintanied at the receiver (which are router) while doing power
     * control. The PI Controller works to maintain this set-point
     * @param[in] aSetPoint The desired receiver link margin in dB
     */
    void SetRouterLinkMarginSetPoint(uint8_t aSetPoint);

    /**
     * This method returns the desired Link Margin to be maintanied at the receiver (which are router) while doing power
     * control.
     * @param[in] aSetPoint The desired receiver link margin in dB
     */
    uint8_t GetRouterLinkMarginSetPoint();

    /**
     * This method sets the desired Link Margin to be maintanied at the receiver (which is end-device) while doing power
     * control. The PI Controller works to maintain this set-point
     * @param[in] aSetPoint The desired receiver link margin in dB
     */
    void SetEndDeviceLinkMarginSetPoint(uint8_t aSetPoint);

    /**
     * This method returns the desired Link Margin to be maintanied at the receiver (which is end-device) while doing
     * power control.
     * @param[in] aSetPoint The desired receiver link margin in dB
     */
    uint8_t GetEndDeviceLinkMarginSetPoint();

    /**
     * This method sets the proportional gain of the PI Controller (Kp)
     * @param[in] aGain
     */
    void SetProportionalGain(int32_t aGain);

    /**
     * This method returns the proportional gain of the PI Controller (Kp)
     * @returns aGain
     */
    int32_t GetProportionalGain();

    /**
     * This method sets the integral gain of the PI Controller (Ki)
     * @param[in] aGain
     */
    void SetIntegralGain(int32_t aGain);

    /**
     * This method returns the integral gain of the PI Controller (Ki)
     * @param[in] aGain
     */
    int32_t GetIntegralGain();

    /**
     * This method sets the weight used for the current sample,
     * while averaing transmit power during a Link Metrics Update.
     * @param[in]  0 < aWeight < 100
     */
    void SetWeightAverage(uint8_t aWeight);

    /**
     * This method returns the weight used for the current sample,
     * while averaing transmit power during a Link Metrics Update.
     * @returns aWeight
     */
    uint8_t GetWeightAverage();

    /**
     * Sets Default Parameters for power controller.
     *
     */
    void SetDefaultParams();

    /**
     * Reset all power control state information in each neighbor.
     *
     */
    void ResetNeighbors(void);

    /**
     * Reset all power control state information in a given neighbor.
     *
     */
    void ResetNeighbor(NeighborType *neighbor);

    /**
     * Gets the current consumed Power for the transmit power
     *
     */
    uint8_t GetCurrent(uint8_t aPower) { return mPowerConsumed[aPower]; }

    /**
     * Gets the current consumed power for the Maximum transmit power.
     *
     */
    uint8_t GetMaxCurrent() { return mPowerConsumed[mMaxTxPower]; }

    /**
     * Gets the step-size increase in Tx Power when an ACK is lost
     *
     */
    uint8_t GetStepUpAckLost() { return mStepUpAckLost; }

    /**
     * Sets the step-size increase in Tx Power when an ACK is lost
     *
     */
    void SetStepUpAckLost(uint8_t aValue) { mStepUpAckLost = aValue; }

    /**
     * This method update Nett Energy saving statistics for the power control.
     *
     * @param[in]  aFrameLength     The transmitted MAC Frame Length.
     * @param[in]  aTxPower         The Tx power for the transmitted frame.
     *
     */
    void UpdateNettEnergySavingStatistics(uint8_t aFrameLength, int8_t aTxPower);

    bool IsAttached(void);

    void HandleLinkMetricsReport(uint16_t aRloc16, uint8_t aLinkMargin);

#if OPENTHREAD_RADIO

    void SetAttachmentStatus(bool aIsAttached) { mIsAttached = aIsAttached; }

    PowerControlTable &GetTable(void) { return mTable; }
#endif

private:
    /**
     * This parameter is used to enable or disable Power Control.
     *
     */
    bool mIsEnabled;

    /**
     * The Max Tx power used by power control algorithm.
     *
     */
    uint8_t mMaxTxPower = 16;

    /**
     * The Min Tx power used by power control algorithm.
     *
     */
    uint8_t mMinTxPower = 0;

    /**
     * The Link Margin to be mainted at the receiver (which are routers) while adjusting transmit power.
     * PI Controller works to maintain this set point.
     */
    uint8_t mRouterLinkMarginSetPoint = 30;

    /**
     * The Link Margin to be mainted at the receiver (which are end-devices) while adjusting transmit power.
     * PI Controller works to maintain this set point.
     */
    uint8_t mEndDeviceLinkMarginSetPoint = 30;

    /**
     * The Proportional Gain of the Controller. Decides step change in power due to current feedback.
     *
     */
    int32_t mKp = -30; // 100*-0.3 to avoid floating point operations;

    /**
     * The Integral Gain of the Controller. Decides change in power due to
     * PI Controller works to maintain this set point.
     */
    int32_t mKi = -1; // 100*-0.01;

    /**
     * The Link Margin to be mainted at the receiver while adjusting transmit power.
     * PI Controller works to maintain this set point.
     */
    int8_t mErrorGain[4] = {-52, -16, -2, 2};

    /**
     * The weight in percent for calculating weighted average when a Link Metrics report is recevied.
     */
    uint8_t mAlpha = 30;

    /**
     * The mode of power control which is used in the device.
     * Bit positions used to represent if a mode is enabled or not. 1 means enabled
     * Bit0= Link Metrics Based, Bit1= Encoded ACK based, Bit0 & Bit1 - Hybrid control based on Link Metrics & Encoded
     * ACK feedback. All bits zero indicates power control is disabled and max Tx power is used.
     */
    PowerControlMode mMode;

    /**
     * Step up of Power in dBm when an ACK loss is detected.
     */
    uint8_t mStepUpAckLost;

    /**
     * Transmit power vs Current consumption table for power efficiency calculation.
     */
    uint8_t mPowerConsumed[17] = {11, 12, 17, 19, 21, 22, 25, 27, 31, 32, 37, 39, 42, 46, 55, 59, 66};

#if OPENTHREAD_RADIO
    bool              mIsAttached;
    PowerControlTable mTable;
#endif
};

#endif // OPENTHREAD_CONFIG_POWER_CONTROL_CORE_ENABLE

/**
 * @}
 *
 */

} // namespace Utils
} // namespace ot

#endif // POWER_CONTROL_HPP_
