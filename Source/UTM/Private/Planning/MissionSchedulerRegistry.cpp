#include "Planning/MissionSchedulerRegistry.h"

namespace
{
    class FStaticMissionScheduler final : public IMissionScheduler
    {
    public:
        virtual FString GetName() const override
        {
            return TEXT("Static Scheduler");
        }

        virtual bool BuildSchedule(
            const FMissionSchedulerContext& Context,
            const TArray<FDroneMissionConfig>& RawMissions,
            TArray<FDroneMissionConfig>& OutScheduledMissions) const override
        {
            (void)Context;
            OutScheduledMissions = RawMissions;
            return true;
        }
    };

    class FNearestFirstMissionScheduler final : public IMissionScheduler
    {
    public:
        virtual FString GetName() const override
        {
            return TEXT("Nearest-First Scheduler");
        }

        virtual bool BuildSchedule(
            const FMissionSchedulerContext& Context,
            const TArray<FDroneMissionConfig>& RawMissions,
            TArray<FDroneMissionConfig>& OutScheduledMissions) const override
        {
            (void)Context;
            OutScheduledMissions.Reset();

            TArray<FDroneMissionConfig> Agents = RawMissions;
            Agents.Sort(
                [](const FDroneMissionConfig& Left, const FDroneMissionConfig& Right)
                {
                    return Left.MissionId < Right.MissionId;
                });

            TArray<int32> RemainingTaskIndices;
            RemainingTaskIndices.Reserve(RawMissions.Num());
            for (int32 MissionIndex = 0; MissionIndex < RawMissions.Num(); ++MissionIndex)
            {
                RemainingTaskIndices.Add(MissionIndex);
            }

            for (const FDroneMissionConfig& AgentMission : Agents)
            {
                int32 BestRemainingIndex = INDEX_NONE;
                int32 BestTaskMissionId = MAX_int32;
                double BestDistanceSq = TNumericLimits<double>::Max();

                for (int32 RemainingIndex = 0; RemainingIndex < RemainingTaskIndices.Num(); ++RemainingIndex)
                {
                    const FDroneMissionConfig& CandidateTask = RawMissions[RemainingTaskIndices[RemainingIndex]];
                    const double CandidateDistanceSq = FVector::DistSquared(AgentMission.StartWorld, CandidateTask.GoalWorld);

                    const bool bCloser = CandidateDistanceSq < BestDistanceSq;
                    const bool bTieBreak =
                        FMath::IsNearlyEqual(CandidateDistanceSq, BestDistanceSq) &&
                        CandidateTask.MissionId < BestTaskMissionId;

                    if (bCloser || bTieBreak)
                    {
                        BestRemainingIndex = RemainingIndex;
                        BestTaskMissionId = CandidateTask.MissionId;
                        BestDistanceSq = CandidateDistanceSq;
                    }
                }

                if (BestRemainingIndex == INDEX_NONE)
                {
                    UE_LOG(LogTemp, Error, TEXT("Nearest-First Scheduler failed to assign MissionId=%d"), AgentMission.MissionId);
                    return false;
                }

                const FDroneMissionConfig& AssignedTask = RawMissions[RemainingTaskIndices[BestRemainingIndex]];

                FDroneMissionConfig ScheduledMission = AgentMission;
                ScheduledMission.GoalWorld = AssignedTask.GoalWorld;
                OutScheduledMissions.Add(ScheduledMission);

                UE_LOG(LogTemp, Warning, TEXT("Nearest-First Scheduler assigned MissionId=%d to goal from MissionId=%d, distance=%.1f"),
                    AgentMission.MissionId,
                    AssignedTask.MissionId,
                    FMath::Sqrt(BestDistanceSq));

                RemainingTaskIndices.RemoveAt(BestRemainingIndex);
            }

            return true;
        }
    };
}

FString FMissionSchedulerRegistry::GetSchedulerTypeName(EMissionSchedulerType SchedulerType)
{
    switch (SchedulerType)
    {
    case EMissionSchedulerType::Static:
        return TEXT("Static Scheduler");
    case EMissionSchedulerType::NearestFirst:
        return TEXT("Nearest-First Scheduler");
    default:
        return TEXT("Unknown");
    }
}

TUniquePtr<IMissionScheduler> FMissionSchedulerRegistry::CreateScheduler(EMissionSchedulerType SchedulerType)
{
    switch (SchedulerType)
    {
    case EMissionSchedulerType::Static:
        return MakeUnique<FStaticMissionScheduler>();
    case EMissionSchedulerType::NearestFirst:
        return MakeUnique<FNearestFirstMissionScheduler>();
    default:
        return nullptr;
    }
}

bool FMissionSchedulerRegistry::BuildSchedule(
    EMissionSchedulerType SchedulerType,
    const FMissionSchedulerContext& Context,
    const TArray<FDroneMissionConfig>& RawMissions,
    TArray<FDroneMissionConfig>& OutScheduledMissions)
{
    OutScheduledMissions.Reset();

    TUniquePtr<IMissionScheduler> Scheduler = CreateScheduler(SchedulerType);
    if (!Scheduler)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create mission scheduler: %s"), *GetSchedulerTypeName(SchedulerType));
        return false;
    }

    return Scheduler->BuildSchedule(Context, RawMissions, OutScheduledMissions);
}
