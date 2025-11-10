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
 *   This file implements CLI commands for Power Control.
 */

#include "cli_power_control.hpp"

#include "cli/cli.hpp"
#include "common/encoding.hpp"

#ifdef _APPLE_FILLMORE_INTERNAL_CHANGES_
extern void persist_tpc_mode(uint8_t mode);
#endif

namespace ot {
namespace Cli {
#if OPENTHREAD_CONFIG_POWER_CONTROL_ENABLE
constexpr PowerControl::Command PowerControl::sCommands[];

#define MAX_HISTOGRAM_STRING_LENGTH 400

otError PowerControl::ProcessHelp(Arg aArgs[])
{
    OT_UNUSED_VARIABLE(aArgs);

    for (const Command &command : sCommands)
    {
        OutputLine(command.mName);
    }

    return OT_ERROR_NONE;
}

otError PowerControl::ProcessMode(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
        OutputLine("Mode: %u", otPowerControlGetMode(GetInstancePtr()) & 0x7);
    }
    else
    {
        uint8_t mode;

        SuccessOrExit(error = aArgs[0].ParseAsUint8(mode));
        SuccessOrExit(error = otPowerControlSetMode(GetInstancePtr(), static_cast<PowerControlMode>(mode)));
        
#ifdef _APPLE_FILLMORE_INTERNAL_CHANGES_
        persist_tpc_mode(mode);
#endif
    }
exit:
    return error;
}

void PowerControl::PrintHistogram(const char *aName, const uint32_t *array, uint8_t aCount)
{
    char     str[MAX_HISTOGRAM_STRING_LENGTH] = {0};
    uint16_t str_cnt                          = 0;

    str_cnt += snprintf(str + str_cnt, sizeof(str) - str_cnt, "%s = [", aName);

    for (uint8_t i = 0; i < aCount; i++)
    {
        str_cnt += snprintf(str + str_cnt, sizeof(str) - str_cnt, "%d", array[i]);

        if (i != aCount - 1)
        {
            str_cnt += snprintf(str + str_cnt, sizeof(str) - str_cnt, ", ");
        }
        else
        {
            str_cnt += snprintf(str + str_cnt, sizeof(str) - str_cnt, "]");
        }
     }

     OutputLine(str);
 }

otError PowerControl::ProcessHistograms(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
        uint32_t arr_frame[OPENTHREAD_CONFIG_POWER_CONTROL_FRAME_TXPOWER_HISTOGRAM_SIZE]                          = {0};
        uint32_t arr_neighbor[OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_TXPOWER_HISTOGRAM_SIZE]                    = {0};
        uint32_t arr_energy_saving[OPENTHREAD_CONFIG_POWER_CONTROL_NEIGHBOR_ENERGY_SAVINGS_FACTOR_HISTOGRAM_SIZE] = {0};
        uint8_t  array_size;

        otPowerControlGetFrameTxPowerHistogram(GetInstancePtr(), arr_frame, &array_size);
        PrintHistogram("FrameTxPowerHistogram", arr_frame, array_size);

        otPowerControlGetNeighborTxPowerHistogram(GetInstancePtr(), arr_neighbor, &array_size);
        PrintHistogram("NeighborTxPowerHistogram", arr_neighbor, array_size);

        otPowerControlGetNeighborEnergySavingsFactorHistogram(GetInstancePtr(), arr_energy_saving, &array_size);
        PrintHistogram("NeighborEnergySavingsFactorHistogram", arr_energy_saving, array_size);
    }
    else
    {
        otPowerControlResetFrameTxPowerHistogram(GetInstancePtr());
        otPowerControlResetNeighborTxPowerHistogram(GetInstancePtr());
        otPowerControlResetNeighborEnergySavingsFactorHistogram(GetInstancePtr());
    }

    return error;
}

otError PowerControl::ProcessProbingInterval(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
        OutputLine("Probing interval: %u seconds", otPowerControlGetLinkMetricsProbeInterval(GetInstancePtr()));
    }
    else
    {
        uint32_t interval;

        SuccessOrExit(error = aArgs[0].ParseAsUint32(interval));
        SuccessOrExit(error = otPowerControlSetLinkMetricsProbeInterval(GetInstancePtr(), interval));
    }
exit:
    return error;
}

otError PowerControl::ProcessWeightFactor(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
        OutputLine("Weight factor: %u", otPowerControlGetWeightAverage(GetInstancePtr()));
    }
    else
    {
        uint8_t weight;

        SuccessOrExit(error = aArgs[0].ParseAsUint8(weight));
        otPowerControlSetWeightAverage(GetInstancePtr(), weight);
    }

exit:
    return error;
}
otError PowerControl::ProcessSetpoint(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0] == "routers")
    {
        if (aArgs[1].IsEmpty())
        {
            OutputLine("Link margin: %u", otPowerControlGetRouterLinkMarginSetPoint(GetInstancePtr()));
        }
        else
        {
            uint8_t margin;

            SuccessOrExit(error = aArgs[1].ParseAsUint8(margin));
            otPowerControlSetRouterLinkMarginSetPoint(GetInstancePtr(), margin);
        }
    }
    else if (aArgs[0] == "children")
    {
        if (aArgs[1].IsEmpty())
        {
            OutputLine("Link margin: %u", otPowerControlGetEndDeviceLinkMarginSetPoint(GetInstancePtr()));
        }
        else
        {
            uint8_t margin;

            SuccessOrExit(error = aArgs[1].ParseAsUint8(margin));
            otPowerControlSetEndDeviceLinkMarginSetPoint(GetInstancePtr(), margin);
        }
    }
    else
    {
        ExitNow(error = OT_ERROR_INVALID_COMMAND);
    }

exit:
    return error;
}

otError PowerControl::ProcessGain(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0] == "proportional")
    {
        if (aArgs[1].IsEmpty())
        {
            OutputLine("Proportional gain: %d", otPowerControlGetProportionalGain(GetInstancePtr()));
        }
        else
        {
            int32_t gain;

            SuccessOrExit(error = aArgs[1].ParseAsInt32(gain));
            otPowerControlSetProportionalGain(GetInstancePtr(), gain * -1);
        }
    }
    else if (aArgs[0] == "integral")
    {
        if (aArgs[1].IsEmpty())
        {
            OutputLine("Integral gain: %d", otPowerControlGetIntegralGain(GetInstancePtr()));
        }
        else
        {
            int32_t gain;

            SuccessOrExit(error = aArgs[1].ParseAsInt32(gain));
            otPowerControlSetIntegralGain(GetInstancePtr(), gain * -1);
        }
    }
    else
    {
        ExitNow(error = OT_ERROR_INVALID_COMMAND);
    }

exit:
    return error;
}

otError PowerControl::ProcessErrorGain(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
        int8_t *gains = otPowerControlGetErrorGain(GetInstancePtr());
        OutputLine("Error gains: [%d, %d, %d, %d]", gains[0], gains[1], gains[2], gains[3]);
    }
    else
    {
        uint8_t index;
        int8_t gain;

        SuccessOrExit(error = aArgs[0].ParseAsUint8(index));
        VerifyOrExit(index < 4, error = OT_ERROR_INVALID_ARGS);

        if (aArgs[1].IsEmpty())
        {
            gain = otPowerControlGetErrorGainAtIndex(GetInstancePtr(), index);
            OutputLine("Error gain: %d", gain);
        }
        else
        {
            SuccessOrExit(error = aArgs[1].ParseAsInt8(gain));
            otPowerControlSetErrorGainAtIndex(GetInstancePtr(), gain, index);
        }
    }

exit:
    return error;
}


otError PowerControl::ProcessAckStepUp(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
        OutputLine("Lost ACK step up: %u", otPowerControlGetStepUpAckLost(GetInstancePtr()));
    }
    else
    {
        uint8_t step;

        SuccessOrExit(error = aArgs[0].ParseAsUint8(step));
        otPowerControlSetStepUpAckLost(GetInstancePtr(), step);
    }

exit:
    return error;
}

otError PowerControl::Process(Arg aArgs[])
{
    otError        error = OT_ERROR_INVALID_COMMAND;
    const Command *command;

    if (aArgs[0].IsEmpty())
    {
        error = ProcessHelp(aArgs);
        ExitNow();
    }

    command = ot::Utils::LookupTable::Find(aArgs[0].GetCString(), sCommands);
    VerifyOrExit(command != nullptr);

    error = (this->*command->mHandler)(aArgs + 1);

exit:
    return error;
}
#endif
} // namespace Cli
} // namespace ot
