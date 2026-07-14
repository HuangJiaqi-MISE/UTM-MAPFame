#pragma once

#include "CoreMinimal.h"
#include "Planning/DroneMissionTypes.h"

enum class EStaticUTMConflictType : uint8
{
    None,
    ProtectionFootprint,
    Downwash
};

class FUTMSafetyModel
{
public:
    static EStaticUTMConflictType GetStaticUTMConfigConflictType(
        const FIntVector& CellA,
        const FDroneMissionConfig& MissionA,
        const FIntVector& CellB,
        const FDroneMissionConfig& MissionB);

    static bool HasStaticUTMConfigConflict(
        const FIntVector& CellA,
        const FDroneMissionConfig& MissionA,
        const FIntVector& CellB,
        const FDroneMissionConfig& MissionB);

    static int32 GetMissionInfluenceRadiusCells(const FDroneMissionConfig& Mission);
};
