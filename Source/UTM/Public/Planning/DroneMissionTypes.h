#pragma once

#include "CoreMinimal.h"
#include "DroneMissionTypes.generated.h"

USTRUCT(BlueprintType)
struct FDroneMissionConfig
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    int32 MissionId = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    FVector StartWorld = FVector(50.f, 50.f, 50.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
    FVector GoalWorld = FVector(850.f, 850.f, 50.f);

	// 表示无人机在水平面上的保护半径，1表示占 3x3，2表示占 5x5，以此类推
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UTM Safety", meta = (ClampMin = "0"))
    int32 ProtectionXYRadiusCells = 0;

    // 表示保护体积向上额外扩张多少层。如果是 1，就表示除了当前高度层，还把上面 1 层也算进保护体积。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UTM Safety", meta = (ClampMin = "0"))
    int32 ProtectionZUpCells = 0;

    // 表示保护体积向下额外扩张多少层。如果是 2，就表示当前层下面再多占 2 层安全空间。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UTM Safety", meta = (ClampMin = "0"))
    int32 ProtectionZDownCells = 0;

    // 表示下洗危险区在水平面的扩张半径。它不是普通保护盒，而是“只在无人机下方”的风险区。如果设为 1，表示下洗区在 XY 上覆盖 3x3；如果是 2，覆盖 5x5。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UTM Safety", meta = (ClampMin = "0"))
    int32 DownwashXYRadiusCells = 0;

    // 表示下洗危险区向下延伸多少层。如果是 2，表示从当前无人机下方连续 2 层都视为危险区。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UTM Safety", meta = (ClampMin = "0"))
    int32 DownwashZBelowCells = 0;

    /*
    
    例子： 位置：(x, y, z)
    ProtectionXYRadiusCells = 1
    ProtectionZUpCells = 1
    ProtectionZDownCells = 2
    DownwashXYRadiusCells = 1
    DownwashZBelowCells = 2
    
    侧视图：X-Z 或 Y-Z 截面，下洗危险区和保护体积是独立的。
    z+2
    z+1      [P]
    z        [D]
    z-1      [P][W]
    z-2      [P][W]
    z-3

    [D]：无人机当前所在 cell
    [P]：Protection box 覆盖到的层
    [W]：Downwash box 覆盖到的层

    */
};
