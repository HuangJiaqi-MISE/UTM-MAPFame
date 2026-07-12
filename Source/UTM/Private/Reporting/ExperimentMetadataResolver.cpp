#include "Reporting/ExperimentMetadataResolver.h"

namespace
{
    FString SanitizeExperimentToken(const FString& Input)
    {
        FString Result;
        Result.Reserve(Input.Len());

        for (const TCHAR Char : Input)
        {
            if (FChar::IsAlnum(Char))
            {
                Result.AppendChar(Char);
            }
        }

        return Result.IsEmpty() ? TEXT("Unknown") : Result;
    }

    FString GetDefaultExperimentGroupNameById(const FString& GroupId)
    {
        if (GroupId == TEXT("G1"))
        {
            return TEXT("ExecutionOnly");
        }
        if (GroupId == TEXT("G2"))
        {
            return TEXT("AlignmentV1");
        }
        if (GroupId == TEXT("G3"))
        {
            return TEXT("AlignmentV2Global");
        }
        if (GroupId == TEXT("R1"))
        {
            return TEXT("V2NoReplan");
        }
        if (GroupId == TEXT("R2"))
        {
            return TEXT("V2Local");
        }
        if (GroupId == TEXT("R3"))
        {
            return TEXT("V2Global");
        }

        return FString();
    }

    bool IsKnownExperimentGroupId(const FString& GroupId)
    {
        return
            GroupId == TEXT("G1") ||
            GroupId == TEXT("G2") ||
            GroupId == TEXT("G3") ||
            GroupId == TEXT("R1") ||
            GroupId == TEXT("R2") ||
            GroupId == TEXT("R3");
    }

    bool IsKnownExperimentPhase(const FString& Phase)
    {
        return Phase == TEXT("PhaseA") || Phase == TEXT("PhaseB");
    }

    FString GetDefaultExperimentPhaseByGroupId(const FString& GroupId)
    {
        if (GroupId.StartsWith(TEXT("G")))
        {
            return TEXT("PhaseA");
        }

        if (GroupId.StartsWith(TEXT("R")))
        {
            return TEXT("PhaseB");
        }

        return FString();
    }

    FString GetExperimentPhaseBySeed(const int32 Seed)
    {
        switch (Seed)
        {
        case 1:
            return TEXT("PhaseA");
        case 3:
            return TEXT("PhaseB");
        default:
            return FString();
        }
    }

    FString BuildFallbackExperimentRunId(
        const FExperimentMetadataResolverInput& Input,
        const FString& Phase,
        const FString& GroupId,
        const FString& ScenarioName)
    {
        const int32 DelayPercent = FMath::Clamp(FMath::RoundToInt(Input.StepDelayProbability * 100.0f), 0, 999);
        const FString SafePhase = Phase.IsEmpty() ? TEXT("PhaseA") : Phase;
        const FString SafeGroupId = GroupId.IsEmpty() ? TEXT("G1") : GroupId;
        const FString SafeScenarioName = ScenarioName.IsEmpty() ? Input.FallbackScenarioName : ScenarioName;

        return FString::Printf(
            TEXT("%s_%s_%s_%s_N%d_P%03d_S%02d"),
            *SanitizeExperimentToken(SafePhase),
            *SanitizeExperimentToken(SafeGroupId),
            *SanitizeExperimentToken(SafeScenarioName),
            *SanitizeExperimentToken(Input.PlannerName),
            Input.EffectiveAgentCount,
            DelayPercent,
            Input.ExecutionRandomSeed);
    }
}

FExperimentMetadata FExperimentMetadataResolver::Resolve(const FExperimentMetadataResolverInput& Input)
{
    FExperimentMetadata Result;
    Result.RunId = Input.RunId.TrimStartAndEnd();
    Result.Phase = Input.Phase.TrimStartAndEnd();
    Result.GroupId = Input.GroupId.TrimStartAndEnd();
    Result.GroupName = Input.GroupName.TrimStartAndEnd();

    const FString TrimmedScenarioName = Input.ScenarioName.TrimStartAndEnd();
    Result.ScenarioName = TrimmedScenarioName.IsEmpty() ? Input.FallbackScenarioName : TrimmedScenarioName;

    TArray<FString> Tokens;
    if (!Result.RunId.IsEmpty())
    {
        Result.RunId.ParseIntoArray(Tokens, TEXT("_"), true);
    }

    // 1. 优先从 run_id 解析显式信息，一般为空或不规范，但如果符合约定格式则优先使用
    if ((Result.Phase.IsEmpty() || !IsKnownExperimentPhase(Result.Phase)) &&
        Tokens.Num() > 0 &&
        IsKnownExperimentPhase(Tokens[0]))
    {
        Result.Phase = Tokens[0];
    }

    if ((Result.GroupId.IsEmpty() || !IsKnownExperimentGroupId(Result.GroupId)) &&
        Tokens.Num() > 1 &&
        IsKnownExperimentGroupId(Tokens[1]))
    {
        Result.GroupId = Tokens[1];
    }

    // 2. 如果 phase 还不明确，先用 seed 决定 phase
    // 当前实验协议约定：
    //   seed == 1 -> PhaseA
    //   seed == 3 -> PhaseB
    if (Result.Phase.IsEmpty() || !IsKnownExperimentPhase(Result.Phase))
    {
        const FString SeedPhase = GetExperimentPhaseBySeed(Input.ExecutionRandomSeed);
        if (!SeedPhase.IsEmpty())
        {
            Result.Phase = SeedPhase;
        }
    }

    // 3. phase 确定后，再根据配置推 group_id
    if (Result.GroupId.IsEmpty() || !IsKnownExperimentGroupId(Result.GroupId))
    {
        if (!Input.bEnableDiscreteAlignment && !Input.bEnableConflictAwareAlignment)
        {
            Result.GroupId = TEXT("G1");
        }
        else if (Input.bEnableDiscreteAlignment && !Input.bEnableConflictAwareAlignment)
        {
            Result.GroupId = TEXT("G2");
        }
        else if (Input.bEnableDiscreteAlignment && Input.bEnableConflictAwareAlignment)
        {
            switch (Input.ReplanMode)
            {
            case EExperimentMetadataReplanMode::Disabled:
                Result.GroupId = TEXT("R1");
                break;

            case EExperimentMetadataReplanMode::LocalConflictSet:
                Result.GroupId = TEXT("R2");
                break;

            case EExperimentMetadataReplanMode::GlobalUnfinished:
                Result.GroupId = (Result.Phase == TEXT("PhaseB")) ? TEXT("R3") : TEXT("G3");
                break;

            default:
                break;
            }
        }
    }

    // 4. 如果 phase 还没定下来，再根据 group_id 兜底
    if (Result.Phase.IsEmpty() || !IsKnownExperimentPhase(Result.Phase))
    {
        Result.Phase = GetDefaultExperimentPhaseByGroupId(Result.GroupId);
    }

    // 5. 归一化 G3 / R3 与 phase 的对应关系
    if (Result.GroupId == TEXT("G3") && Result.Phase == TEXT("PhaseB"))
    {
        Result.GroupId = TEXT("R3");
    }
    else if (Result.GroupId == TEXT("R3") && Result.Phase == TEXT("PhaseA"))
    {
        Result.GroupId = TEXT("G3");
    }

    // 6. group_name 按 group_id 统一生成
    if (Result.GroupName.IsEmpty() || GetDefaultExperimentGroupNameById(Result.GroupId) != Result.GroupName)
    {
        Result.GroupName = GetDefaultExperimentGroupNameById(Result.GroupId);
    }

    // 7. 如果 run_id 为空，自动生成
    if (Result.RunId.IsEmpty())
    {
        Result.RunId = BuildFallbackExperimentRunId(Input, Result.Phase, Result.GroupId, Result.ScenarioName);
    }

    // 8. 可选的一致性告警
    if (Result.Phase == TEXT("PhaseA") && Input.ExecutionRandomSeed != 1)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Experiment metadata mismatch: PhaseA usually expects execution_random_seed=1, got %d"),
            Input.ExecutionRandomSeed);
    }
    else if (Result.Phase == TEXT("PhaseB") && Input.ExecutionRandomSeed != 3)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Experiment metadata mismatch: PhaseB usually expects execution_random_seed=3, got %d"),
            Input.ExecutionRandomSeed);
    }

    return Result;
}
