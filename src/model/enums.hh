#ifndef __MODEL_ENUMS_HH__
#define __MODEL_ENUMS_HH__

namespace gem5
{


enum SleepyState
{
    AWAKE,
    ASLEEP,
    NUM_SLEEPY_STATE
};
extern const char* sleepyStateStrings[NUM_SLEEPY_STATE];

} // namespace gem5

#endif // __MODEL_ENUMS_HH__
