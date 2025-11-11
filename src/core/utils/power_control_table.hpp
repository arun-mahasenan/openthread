#ifndef POWER_CONTROL_TABLE_HPP_
#define POWER_CONTROL_TABLE_HPP_

#include <openthread/error.h>
#include <mac/mac_types.hpp>
#include "openthread/power_control.h"

namespace ot {
namespace Utils {

class PowerControlTable
{
public:
    struct Entry
    {
        uint8_t GetTxPower(void) { return mTxPower; }

        void SetTxPower(uint8_t txPower) { mTxPower = txPower; }

        uint8_t GetCodeWordCount(void) { return mCodeWordCount; }

        void SetCodeWordCount(uint8_t codeWordCount) { mCodeWordCount = codeWordCount; }

        void SetIntegral(int integral) { mIntegral = integral; }

        int GetIntegral() { return mIntegral; }

        PowerControlMode GetPowerControlMode() const { return mPowerControlMode; }

        void SetPowerControlMode(PowerControlMode mode)
        {
            mPowerControlMode = mode;

            if (mode == PowerControlMode::kPowerControlDisable)
            {
                SetTxPower(0xFF); // Set to Max Tx Power
            }
        }

        void UpdateAckStatus(bool status)
        {
            if (status == false)
            {
                // Lost ACK, set bit to 1
                mLostAckStatus = (mLostAckStatus << 1) | 1;
            }
            else
            {
                // Received ACK, set bit to 0
                mLostAckStatus = (mLostAckStatus << 1) & 0xFE;
            }
        }

        uint8_t GetLostAckCountInWindow(void)
        {
            int count = 0;
            for (int i = 0; i < 4; i++)
            {
                if ((mLostAckStatus >> i) & 1)
                {
                    count++;
                }
            }
            return count;
        }

        bool IsStateValid(void) { return true; }

        bool IsRxOnWhenIdle(void) { return mIsRxOnWhenIdle; }

        void SetRxOnWhenIdle(bool aIsRxOnWhenIdle) { mIsRxOnWhenIdle = aIsRxOnWhenIdle; }

        uint16_t GetRloc16(void) { return mShortAddr; }

        void ResetPowerControlData(void)
        {
            mEnergy        = 0;
            mTxCount       = 0;
            mTotalPower    = 0;
            mByteCount     = 0;
            mCodeWordCount = 0;
        }

        void UpdateMeasurementWindowStatistics(uint8_t aFrameLength,
                                               int8_t  aTxPower,
                                               uint8_t aCurrentPower,
                                               uint8_t aMaxCurrentPower);

        uint8_t           mIdx;
        Mac::ExtAddress   mExtAddr;
        Mac::ShortAddress mShortAddr;
        bool              mIsRxOnWhenIdle;
        uint8_t           mTxPower;       /// Optimal Transmit Power for the neighbor
        int               mIntegral;      /// Integral sum of error for PI loop
        uint8_t           mLostAckStatus; /// Moving window of ACK received status.1 represents a lost ACK
        PowerControlMode  mPowerControlMode =
            PowerControlMode::kPowerControlHybrid; ///{0-disable TPC,1-LM Only TPC,2-Encoded ACK based TCP,
                                                   /// 3- LM and Encode ACK based TPC}
        uint32_t mEnergy;                          // Bytes*Current which is a measure of Total energy consumed
        uint16_t mTxCount;                         // Total bytes sent during this measurement window.
        uint16_t mTotalPower;    // Linear sum of TX power. Simpler approximation of logaritmic sum in dB.
        uint16_t mByteCount;     // Total bytes sent during this measurement window
        uint8_t  mCodeWordCount; // Count of consequtive Encode ACK codeword with Code 3 feedback from the neighbor;
    };

    PowerControlTable(otInstance *            aInstance,
                      const PowerControlMode *aModePtr,
                      const int32_t *         aKiPtr,
                      const uint8_t *         aMaxPowerPtr);

    Error AddEntry(Mac::ShortAddress aShortAddr, const Mac::ExtAddress &aExtAddr, uint8_t aMode, bool mIsRxWhenIdle);
    void  RemoveEntry(Entry &aEntry);

    uint8_t GetEntryCount(void) { return mCount; }
    Entry * GetEntry(uint8_t aIndex) { return &mEntries[aIndex]; }

    template <typename Fn> void ForEach(Fn aFn)
    {
        for (auto i = 0; i < mCount; ++i)
        {
            aFn(mEntries[i]);
        }
    }

    Entry *FindEntry(Mac::ShortAddress aAddress);
    Entry *FindEntry(Mac::ExtAddress &aAddress);

private:
    enum
    {
        kPowerControlTableSize = OPENTHREAD_CONFIG_MLE_MAX_CHILDREN + OPENTHREAD_CONFIG_MLE_MAX_ROUTERS,
    };

    Entry                   mEntries[kPowerControlTableSize];
    uint8_t                 mCount;
    otInstance *            mInstance;
    const PowerControlMode *mModePtr;
    const int32_t *         mKiPtr;
    const uint8_t *         mMaxPowerPtr;
};

} // namespace Utils
} // namespace ot

#endif
