/*
 *  Copyright (c) 2022, The OpenThread Authors.
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
 *   This file contains definitions for Power Control CLI commands.
 */

#ifndef CLI_POWER_CONTROL_HPP_
#define CLI_POWER_CONTROL_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
#include <openthread/power_control.h>
#endif

//#include "cli/cli_output.hpp"
#include "cli/cli_config.h"
#include "cli/cli_utils.hpp"
#include "utils/lookup_table.hpp"
#include "utils/parse_cmdline.hpp"

#if _APPLE_FILLMORE_INTERNAL_CHANGES_
/**
 * @def OPENTHREAD_CONFIG_POWER_CONTROL_FRAME_TXPOWER_HISTOGRAM_SIZE
 *
 * Define the number of buckets in Frame Transmit Power Histogram of the Power Control Algorithm.
 * Used when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
 *
 */
#ifndef OPENTHREAD_CONFIG_POWER_CONTROL_FRAME_TXPOWER_HISTOGRAM_SIZE
#define OPENTHREAD_CONFIG_POWER_CONTROL_FRAME_TXPOWER_HISTOGRAM_SIZE 17
#endif

/**
 * @def OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_TXPOWER_HISTOGRAM_SIZE
 *
 * Define the number of buckets in Neighbor Transmit Power Histogram of the Power Control Algorithm.
 * Used when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
 *
 */
#ifndef OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_TXPOWER_HISTOGRAM_SIZE
#define OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_TXPOWER_HISTOGRAM_SIZE 17
#endif

/**
 * @def OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_ENERGY_SAVINGS_FACTOR_HISTOGRAM_SIZE
 *
 * Define the number of buckets in Neighbor Energy Savings Factor Histogram of the Power Control Algorithm.
 * Used when OPENTHREAD_CONFIG_POWER_CONTROL_HISTOGRAM_ENABLE is enabled.
 *
 */
#ifndef OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_ENERGY_SAVINGS_FACTOR_HISTOGRAM_SIZE
#define OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_ENERGY_SAVINGS_FACTOR_HISTOGRAM_SIZE 19
#endif
#endif //_APPLE_FILLMORE_INTERNAL_CHANGES_

namespace ot {
namespace Cli {

/**
 * This class implements the Network Data CLI.
 *
 */
class PowerControl : private Utils
{
public:
    typedef ot::Utils::CmdLineParser::Arg Arg;

    /**
     * Constructor
     *
     * @param[in]  aOutput The CLI console output context
     *
     */
//    explicit PowerControl(Output &aOutput)
//        : Output(aOutput)
    PowerControl(otInstance *aInstance, OutputImplementer &aOutputImplementer)
        : Utils(aInstance, aOutputImplementer)
    {
    }

    /**
     * This method interprets a list of CLI arguments.
     *
     * @param[in]  aArgs        An array of command line arguments.
     *
     */
    otError Process(Arg aArgs[]);

private:
    struct Command
    {
        const char *mName;
        otError (PowerControl::*mHandler)(Arg aArgs[]);
    };

    otError ProcessHelp(Arg aArgs[]);
    otError ProcessMode(Arg aArgs[]);
    otError ProcessHistograms(Arg aArgs[]);
    otError ProcessProbingInterval(Arg aArgs[]);
    otError ProcessWeightFactor(Arg aArgs[]);
    otError ProcessSetpoint(Arg aArgs[]);
    otError ProcessGain(Arg aArgs[]);
    otError ProcessErrorGain(Arg aArgs[]);
    otError ProcessAckStepUp(Arg aArgs[]);
    void    PrintHistogram(const char *aName, const uint32_t *array, uint8_t aCount);

    static constexpr Command sCommands[] = {
        {"ackstepup", &PowerControl::ProcessAckStepUp},
        {"errorgain", &PowerControl::ProcessErrorGain},
        {"gain", &PowerControl::ProcessGain},
        {"help", &PowerControl::ProcessHelp},
        {"histograms", &PowerControl::ProcessHistograms},
        {"mode", &PowerControl::ProcessMode},
        {"probinginterval", &PowerControl::ProcessProbingInterval},
        {"setpoint", &PowerControl::ProcessSetpoint},
        {"weightfactor", &PowerControl::ProcessWeightFactor},
    };

    static_assert(ot::Utils::LookupTable::IsSorted(sCommands), "Command Table is not sorted");
};

} // namespace Cli
} // namespace ot

#endif // CLI_POWER_CONTROL_HPP_
