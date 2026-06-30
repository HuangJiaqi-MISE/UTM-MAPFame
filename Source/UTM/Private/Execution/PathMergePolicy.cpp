#include "Execution/PathMergePolicy.h"

FPathMergeResult FPathMergePolicy::MergeByMissionId(const FPathMergeRequest& Request)
{
    FPathMergeResult Result;
    Result.MergedPaths = Request.ExistingPaths;

    for (const TPair<int32, TArray<FVector>>& ReplannedPath : Request.ReplannedPaths)
    {
        const int32 MissionId = ReplannedPath.Key;
        if (Request.MissionIdsToReplace.Num() > 0 && !Request.MissionIdsToReplace.Contains(MissionId))
        {
            continue;
        }

        Result.MergedPaths.Add(MissionId, ReplannedPath.Value);
        Result.ReplacedPathCount++;
    }

    return Result;
}

