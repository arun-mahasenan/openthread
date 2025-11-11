#include "power_control_probe_sender.hpp"
#include <common/logging.hpp>
#include <mac/mac.hpp>
#include "openthread/instance.h"
#include "instance/instance.hpp"


#if (OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE) && \
    OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE

#define TPC_LINKMETRIC_TIMER_CHECK_INTERVAL 1000 // Interval to check for LMProbing timer expiry

constexpr uint16_t kMsPerS = 1000;

namespace ot {
namespace Utils {

PowerControlProbeSender::PowerControlProbeSender(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mTimer(aInstance, PowerControlProbeSender::HandleTimer)
    , mLinkMetricsProbeInterval(kTimerInterval)
{
}

void PowerControlProbeSender::Start(void)
{
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogDebgPlat("TPC: probe sender start with interval: %d", TPC_LINKMETRIC_TIMER_CHECK_INTERVAL);
#endif
    mTimer.Start(TPC_LINKMETRIC_TIMER_CHECK_INTERVAL);
}

void PowerControlProbeSender::Stop(void)
{
    if (mTimer.IsRunning())
    {
        mTimer.Stop();
    }

//exit:
    return;
}

bool PowerControlProbeSender::IsParentLMProbeInterval(uint32_t aLMTimerCheckInterval)
{
    if (mParentLMTimeout <= aLMTimerCheckInterval)
    {
        mParentLMTimeout = GetLinkMetricsProbeInterval() * 1000;
        return true;
    }
    else
    {
        mParentLMTimeout -= aLMTimerCheckInterval; // count down timer every LMProbe check interval
    }
    return false;
}

Error PowerControlProbeSender::SetLinkMetricsProbeInterval(uint32_t aSeconds)
{
    Error error = OT_ERROR_NONE;

    VerifyOrExit(aSeconds < kMaxProbingInterval, error = OT_ERROR_INVALID_ARGS);
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
    otLogDebgPlat("TPC: probe sender set interval to: %ld seconds", aSeconds);
#endif
    mLinkMetricsProbeInterval = aSeconds * kMsPerS;

exit:
    return error;
}

uint32_t PowerControlProbeSender::GetLinkMetricsProbeInterval()
{
    return mLinkMetricsProbeInterval / kMsPerS;
}

void PowerControlProbeSender::HandleLinkMetricsReport(const otIp6Address *       aAddress,
                                                      const otLinkMetricsValues *aMetricsValues,
                                                      otLinkMetricsStatus        aStatus,
                                                      void *                     aContext)
{
    static_cast<PowerControlProbeSender *>(aContext)->HandleLinkMetricsReport(aAddress, aMetricsValues, aStatus);
}

void PowerControlProbeSender::HandleLinkMetricsReport(const otIp6Address *       aAddress,
                                                      const otLinkMetricsValues *aMetricsValues,
                                                      otLinkMetricsStatus        aStatus)
{
    OT_UNUSED_VARIABLE(aStatus);
    if (NULL ==aMetricsValues)
    {
        otLogWarnPlat("TPC: HandleLinkMetricsReport metricsValues are null");
        goto exit;
    }
    ot::Mac::ExtAddress mac;
    memcpy(mac.m8, &aAddress->mFields.m8[8], 8); // neighborInfo.mExtAddress;
    mac.ToggleLocal();

//     Neighbor *neighbor;
//     neighbor = GetInstance().Get<NeighborTable>().FindNeighbor(mac, Neighbor::kInStateValid);

// #if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
//     otLogInfoPlat("TPC: probe sender received report from: %02x with link margin: %u", neighbor->GetRloc16(),
//                   aMetricsValues->mLinkMarginValue);
// #endif

//     if (neighbor)
//     {
//         if (aMetricsValues != nullptr)
//         {
//             otPowerControlHandleLinkMetricsProbe(&GetInstance(), neighbor->GetRloc16(),
//                                                  aMetricsValues->mLinkMarginValue);
//         }
//     }
exit:
    return;
}

void PowerControlProbeSender::HandleTimer(Timer &aTimer)
{
    aTimer.Get<PowerControlProbeSender>().HandleTimer();
}

void PowerControlProbeSender::HandleTimer(void)
{
    Error error;
    OT_UNUSED_VARIABLE(error);

    otNeighborInfo         neighborInfo;
    otNeighborInfoIterator iterator = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    
    otLinkMetrics linkMetrics;
    memset(&linkMetrics, 0, sizeof(linkMetrics));
    linkMetrics.mLinkMargin = true;
    
     if (Get<Mle::Mle>().IsAttached())
    {
        while (otThreadGetNextNeighborInfo(&GetInstance(), &iterator, &neighborInfo) == kErrorNone)
        {
            Neighbor *neighbor =
                GetInstance().Get<NeighborTable>().FindNeighbor(neighborInfo.mRloc16, Neighbor::kInStateValid);
            if (neighbor != nullptr)
            {
               // if (neighbor->IsRxOnWhenIdle() && neighbor->IsLMProbeInterval(TPC_LINKMETRIC_TIMER_CHECK_INTERVAL))
                {
#if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
                    otLogDebgPlat("TPC: probe sender sending query to: %02x ", neighborInfo.mRloc16);
#endif
                    ot::Mac::ExtAddress mac;
                    memcpy(mac.m8, neighborInfo.mExtAddress.m8, OT_EXT_ADDRESS_SIZE);
                    Ip6::Address ip6Address;
                    ip6Address.SetToLinkLocalAddress(mac);

                    error = otLinkMetricsQuery(&GetInstance(), &ip6Address, 0, &linkMetrics,
                                               &PowerControlProbeSender::HandleLinkMetricsReport, this);
                }
            }
        }
    }
    
    VerifyOrExit(otThreadGetDeviceRole(&GetInstance()) == OT_DEVICE_ROLE_CHILD);

    otRouterInfo parentInfo;
    VerifyOrExit(otThreadGetParentInfo(&GetInstance(), &parentInfo) == kErrorNone);
    VerifyOrExit(IsParentLMProbeInterval(TPC_LINKMETRIC_TIMER_CHECK_INTERVAL));

    #if OPENTHREAD_CONFIG_POWER_CONTROL_DEBUG_ENABLE
         otLogDebgPlat("TPC: probe sender sending query to the parent");
    #endif
    ot::Mac::ExtAddress mac;
    memcpy(mac.m8, parentInfo.mExtAddress.m8, OT_EXT_ADDRESS_SIZE);
    Ip6::Address ip6Address;
    ip6Address.SetToLinkLocalAddress(mac);
    error = otLinkMetricsQuery(&GetInstance(), &ip6Address, 0, &linkMetrics,
                                   &PowerControlProbeSender::HandleLinkMetricsReport, this);
   exit:

    mTimer.StartAt(TimerMilli::GetNow(), TPC_LINKMETRIC_TIMER_CHECK_INTERVAL);
}

} // namespace Utils
} // namespace ot

#endif // (OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE || OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE) &&
       // OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
