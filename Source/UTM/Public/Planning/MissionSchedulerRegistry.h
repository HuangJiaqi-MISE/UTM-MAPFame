#pragma once

#include "CoreMinimal.h"
#include "Planning/MissionSchedulerTypes.h"

class IMissionScheduler
{
public:
    virtual ~IMissionScheduler() = default;

    virtual FString GetName() const = 0;

    virtual bool BuildSchedule(
        const FMissionSchedulerContext& Context,
        const TArray<FDroneMissionConfig>& RawMissions,
        TArray<FDroneMissionConfig>& OutScheduledMissions) const = 0;
};

class FMissionSchedulerRegistry
{
public:
    static FString GetSchedulerTypeName(EMissionSchedulerType SchedulerType);

    static TUniquePtr<IMissionScheduler> CreateScheduler(EMissionSchedulerType SchedulerType);

    static bool BuildSchedule(
        EMissionSchedulerType SchedulerType,
        const FMissionSchedulerContext& Context,
        const TArray<FDroneMissionConfig>& RawMissions,
        TArray<FDroneMissionConfig>& OutScheduledMissions);
};

