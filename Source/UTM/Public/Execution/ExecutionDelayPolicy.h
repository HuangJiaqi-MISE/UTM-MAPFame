#pragma once

#include "Execution/ExecutionDelayPolicyTypes.h"

class FExecutionDelayPolicy
{
public:
    static bool ShouldDelay(const FExecutionDelayPolicyInput& Input);
};
