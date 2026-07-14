#include "Planning/UTMSafetyModel.h"

namespace
{
    struct FMissionFootprintBox
    {
        bool bValid = false;
        FIntVector Min = FIntVector::ZeroValue;
        FIntVector Max = FIntVector::ZeroValue;
    };

    FMissionFootprintBox MakeMissionProtectionBox(const FIntVector& CenterCell, const FDroneMissionConfig& Mission)
    {
        const int32 ProtectionXYRadiusCells = FMath::Max(0, Mission.ProtectionXYRadiusCells);
        const int32 ProtectionZUpCells = FMath::Max(0, Mission.ProtectionZUpCells);
        const int32 ProtectionZDownCells = FMath::Max(0, Mission.ProtectionZDownCells);

        FMissionFootprintBox Result;
        Result.bValid = true;
        Result.Min = FIntVector(
            CenterCell.X - ProtectionXYRadiusCells,
            CenterCell.Y - ProtectionXYRadiusCells,
            CenterCell.Z - ProtectionZDownCells);
        Result.Max = FIntVector(
            CenterCell.X + ProtectionXYRadiusCells,
            CenterCell.Y + ProtectionXYRadiusCells,
            CenterCell.Z + ProtectionZUpCells);
        return Result;
    }

    FMissionFootprintBox MakeMissionDownwashBox(const FIntVector& CenterCell, const FDroneMissionConfig& Mission)
    {
        const int32 DownwashXYRadiusCells = FMath::Max(0, Mission.DownwashXYRadiusCells);
        const int32 DownwashZBelowCells = FMath::Max(0, Mission.DownwashZBelowCells);

        if (DownwashZBelowCells <= 0)
        {
            return FMissionFootprintBox();
        }

        FMissionFootprintBox Result;
        Result.bValid = true;
        Result.Min = FIntVector(
            CenterCell.X - DownwashXYRadiusCells,
            CenterCell.Y - DownwashXYRadiusCells,
            CenterCell.Z - DownwashZBelowCells);
        Result.Max = FIntVector(
            CenterCell.X + DownwashXYRadiusCells,
            CenterCell.Y + DownwashXYRadiusCells,
            CenterCell.Z - 1);
        return Result;
    }

    bool MissionBoxesOverlap(const FMissionFootprintBox& Left, const FMissionFootprintBox& Right)
    {
        return Left.bValid
            && Right.bValid
            && Left.Min.X <= Right.Max.X && Right.Min.X <= Left.Max.X
            && Left.Min.Y <= Right.Max.Y && Right.Min.Y <= Left.Max.Y
            && Left.Min.Z <= Right.Max.Z && Right.Min.Z <= Left.Max.Z;
    }
}

EStaticUTMConflictType FUTMSafetyModel::GetStaticUTMConfigConflictType(
    const FIntVector& CellA,
    const FDroneMissionConfig& MissionA,
    const FIntVector& CellB,
    const FDroneMissionConfig& MissionB)
{
    const FMissionFootprintBox ProtectionA = MakeMissionProtectionBox(CellA, MissionA);
    const FMissionFootprintBox ProtectionB = MakeMissionProtectionBox(CellB, MissionB);
    if (MissionBoxesOverlap(ProtectionA, ProtectionB))
    {
        return EStaticUTMConflictType::ProtectionFootprint;
    }

    if (CellA.Z > CellB.Z
        && MissionBoxesOverlap(MakeMissionDownwashBox(CellA, MissionA), ProtectionB))
    {
        return EStaticUTMConflictType::Downwash;
    }

    if (CellB.Z > CellA.Z
        && MissionBoxesOverlap(MakeMissionDownwashBox(CellB, MissionB), ProtectionA))
    {
        return EStaticUTMConflictType::Downwash;
    }

    return EStaticUTMConflictType::None;
}

bool FUTMSafetyModel::HasStaticUTMConfigConflict(
    const FIntVector& CellA,
    const FDroneMissionConfig& MissionA,
    const FIntVector& CellB,
    const FDroneMissionConfig& MissionB)
{
    return GetStaticUTMConfigConflictType(CellA, MissionA, CellB, MissionB)
        != EStaticUTMConflictType::None;
}

int32 FUTMSafetyModel::GetMissionInfluenceRadiusCells(const FDroneMissionConfig& Mission)
{
    return FMath::Max3(
        FMath::Max(Mission.ProtectionXYRadiusCells, Mission.DownwashXYRadiusCells),
        FMath::Max(Mission.ProtectionZUpCells, Mission.ProtectionZDownCells),
        Mission.DownwashZBelowCells);
}
