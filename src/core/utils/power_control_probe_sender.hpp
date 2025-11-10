/*
 *  Copyright (c) 2016, The OpenThread Authors.
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

#ifndef POWER_CONTROL_PROBE_SENDER_HPP_
#define POWER_CONTROL_PROBE_SENDER_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE

#include <common/timer.hpp>
#include <openthread/ip6.h>
#include <openthread/link_metrics.h>

namespace ot {
namespace Utils {

class PowerControlProbeSender : public InstanceLocator, private NonCopyable
{
public:
    enum
    {
        /**
         * The default Link Metric sample interval in milliseconds.
         *
         */
#ifndef _APPLE_FILLMORE_INTERNAL_CHANGES_
        kTimerInterval      = 60000,
#else
        // Set to 30 minutes
        kTimerInterval      = 30 * 60 * 1000,
#endif
        kMaxProbingInterval = UINT32_MAX / 1000,
    };

    /**
     * This constructor initializes the object.
     *
     * @param[in]  aInstance     A reference to the OpenThread instance.
     *
     */
    explicit PowerControlProbeSender(Instance &aInstance);

    void Start(void);

    void Stop(void);

    /**
     * This method indicates whether the Power Control operation is started and running.
     *
     * @retval TRUE if the Power Control operation is running, FALSE otherwise.
     *
     */

    bool IsRunning(void) const { return mTimer.IsRunning(); }

    /**
     * This method sets the Link Metrics Probing interval.
     * @param[in] aSeconds Probe interval in seconds
     */
    Error SetLinkMetricsProbeInterval(uint32_t aSeconds);

    /**
     * This method returns the current Link Metrics Probing interval.
     * @returns aSeconds Probe interval in seconds
     */
    uint32_t GetLinkMetricsProbeInterval();

    /**
     * This method returns true if the Link Metrics probe event for Parent is true.
     * Decreases the timeout for LMProbe by aLMTimerCheckInterval.
     *
     */
    bool IsParentLMProbeInterval(uint32_t aLMTimerCheckInterval);

    /**
     * This is the call back function when a LinkMetrics report is received from a subject in response to a probe
     * request.
     *
     * @param[in]  aAddress           Address of subject.
     * @param[in]  aMetricsValues     Link Metrics Value requested.
     * @param[in]  aStatus            Status.
     * @param[in]  aContext           Context.
     * @returns  void
     */
    static void HandleLinkMetricsReport(const otIp6Address *       aAddress,
                                        const otLinkMetricsValues *aMetricsValues,
                                        otLinkMetricsStatus        aStatus,
                                        void *                     aContext);
    void        HandleLinkMetricsReport(const otIp6Address *       aAddress,
                                        const otLinkMetricsValues *aMetricsValues,
                                        otLinkMetricsStatus        aStatus);

private:
    /**
     * Timers used for Link Metrics probing.
     *
     */
    static void HandleTimer(Timer &aTimer);
    void        HandleTimer(void);

    void *     mContext;
    TimerMilli mTimer;
    uint32_t   mLinkMetricsProbeInterval;
    uint32_t   mParentLMTimeout;
};

} // namespace Utils
} // namespace ot
#endif

#endif
