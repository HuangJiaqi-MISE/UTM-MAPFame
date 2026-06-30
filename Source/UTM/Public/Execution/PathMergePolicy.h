#pragma once

#include "CoreMinimal.h"

struct FPathMergeRequest
{
    TMap<int32, TArray<FVector>> ExistingPaths;
    TMap<int32, TArray<FVector>> ReplannedPaths;
    TSet<int32> MissionIdsToReplace;
};

struct FPathMergeResult
{
    TMap<int32, TArray<FVector>> MergedPaths;
    int32 ReplacedPathCount = 0;
};

class FPathMergePolicy
{
public:
    static FPathMergeResult MergeByMissionId(const FPathMergeRequest& Request);
};

