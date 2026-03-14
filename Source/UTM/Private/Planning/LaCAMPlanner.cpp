#include "Planning/LaCAMPlanner.h"

#include "HAL/PlatformTime.h"
#include "Planning/GridMap3D.h"

#include <algorithm>
#include <deque>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace
{
    bool ValidateMissionIdsUniqueLaCAM(const TArray<FDroneMissionConfig>& Missions)
    {
        TSet<int32> SeenMissionIds;

        for (const FDroneMissionConfig& Mission : Missions)
        {
            if (Mission.MissionId <= 0)
            {
                UE_LOG(LogTemp, Error, TEXT("LaCAM: MissionId must be positive. Invalid MissionId=%d"), Mission.MissionId);
                return false;
            }

            if (SeenMissionIds.Contains(Mission.MissionId))
            {
                UE_LOG(LogTemp, Error, TEXT("LaCAM: duplicate MissionId detected: %d"), Mission.MissionId);
                return false;
            }

            SeenMissionIds.Add(Mission.MissionId);
        }

        return true;
    }
}

namespace LaCAMUE
{
    struct FDeadline
    {
        explicit FDeadline(double InTimeLimitMs)
            : StartSeconds(FPlatformTime::Seconds())
            , TimeLimitMs(InTimeLimitMs)
        {
        }

        double ElapsedMs() const
        {
            return (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
        }

        bool IsExpired() const
        {
            return TimeLimitMs > 0.0 && ElapsedMs() >= TimeLimitMs;
        }

        double StartSeconds = 0.0;
        double TimeLimitMs = 0.0;
    };

    struct FVertex
    {
        FVertex(int32 InId, int32 InGridIndex, const FIntVector& InCell)
            : Id(InId)
            , GridIndex(InGridIndex)
            , Cell(InCell)
        {
        }

        int32 Id = INDEX_NONE;
        int32 GridIndex = INDEX_NONE;
        FIntVector Cell = FIntVector::ZeroValue;
        std::vector<FVertex*> Neighbors;
        std::vector<FVertex*> Actions;
    };

    using Config = std::vector<FVertex*>;
    using Solution = std::vector<Config>;

    struct FGraph
    {
        explicit FGraph(const FGridMap3D& GridMap)
            : GridDim(GridMap.GridDim)
            , GridIndexToVertex(GridDim.X* GridDim.Y* GridDim.Z, nullptr)
        {
            for (int32 Z = 0; Z < GridDim.Z; ++Z)
            {
                for (int32 Y = 0; Y < GridDim.Y; ++Y)
                {
                    for (int32 X = 0; X < GridDim.X; ++X)
                    {
                        if (GridMap.IsBlocked(X, Y, Z))
                        {
                            continue;
                        }

                        const FIntVector Cell(X, Y, Z);
                        const int32 GridIndex = GridMap.ToIndex(X, Y, Z);
                        FVertex* Vertex = new FVertex(Vertices.size(), GridIndex, Cell);
                        Vertices.push_back(Vertex);
                        GridIndexToVertex[GridIndex] = Vertex;
                    }
                }
            }

            static const FIntVector Directions[6] =
            {
                FIntVector(1, 0, 0),
                FIntVector(-1, 0, 0),
                FIntVector(0, 1, 0),
                FIntVector(0, -1, 0),
                FIntVector(0, 0, 1),
                FIntVector(0, 0, -1)
            };

            for (FVertex* Vertex : Vertices)
            {
                Vertex->Neighbors.reserve(6);
                for (const FIntVector& Direction : Directions)
                {
                    const FIntVector NeighborCell = Vertex->Cell + Direction;
                    FVertex* Neighbor = GetVertex(NeighborCell);
                    if (Neighbor != nullptr)
                    {
                        Vertex->Neighbors.push_back(Neighbor);
                    }
                }

                Vertex->Actions = Vertex->Neighbors;
                Vertex->Actions.push_back(Vertex);
            }
        }

        ~FGraph()
        {
            for (FVertex* Vertex : Vertices)
            {
                delete Vertex;
            }
        }

        bool IsInside(const FIntVector& Cell) const
        {
            return Cell.X >= 0 && Cell.X < GridDim.X
                && Cell.Y >= 0 && Cell.Y < GridDim.Y
                && Cell.Z >= 0 && Cell.Z < GridDim.Z;
        }

        int32 ToGridIndex(const FIntVector& Cell) const
        {
            return Cell.X + GridDim.X * (Cell.Y + GridDim.Y * Cell.Z);
        }

        FVertex* GetVertex(const FIntVector& Cell) const
        {
            if (!IsInside(Cell))
            {
                return nullptr;
            }

            const int32 GridIndex = ToGridIndex(Cell);
            return GridIndexToVertex[GridIndex];
        }

        int32 Size() const
        {
            return Vertices.size();
        }

        FIntVector GridDim = FIntVector::ZeroValue;
        std::vector<FVertex*> Vertices;
        std::vector<FVertex*> GridIndexToVertex;
    };

    bool IsSameConfig(const Config& Left, const Config& Right)
    {
        if (Left.size() != Right.size())
        {
            return false;
        }

        for (size_t Index = 0; Index < Left.size(); ++Index)
        {
            if (Left[Index] != Right[Index])
            {
                return false;
            }
        }

        return true;
    }

    struct FConfigHasher
    {
        size_t operator()(const Config& InConfig) const
        {
            size_t Hash = InConfig.size();

            for (const FVertex* Vertex : InConfig)
            {
                const size_t Value = Vertex != nullptr ? static_cast<size_t>(Vertex->Id) : static_cast<size_t>(MAX_int32);
                Hash ^= Value + 0x9e3779b9 + (Hash << 6) + (Hash >> 2);
            }

            return Hash;
        }
    };

    struct FInstance
    {
        explicit FInstance(const FGridMap3D& GridMap)
            : Graph(GridMap)
        {
        }

        FGraph Graph;
        Config Starts;
        Config Goals;
    };

    class FDistTable
    {
    public:
        explicit FDistTable(const FInstance& InInstance)
            : VertexCount(InInstance.Graph.Size())
            , Table(InInstance.Starts.size(), std::vector<int32>(VertexCount, VertexCount))
        {
            for (int32 AgentIndex = 0; AgentIndex < static_cast<int32>(InInstance.Starts.size()); ++AgentIndex)
            {
                BuildDistancesForAgent(InInstance, AgentIndex);
            }
        }

        int32 Get(int32 AgentIndex, const FVertex* Vertex) const
        {
            return Table[AgentIndex][Vertex->Id];
        }

    private:
        void BuildDistancesForAgent(const FInstance& InInstance, int32 AgentIndex)
        {
            FVertex* Goal = InInstance.Goals[AgentIndex];
            std::queue<FVertex*> Open;
            Open.push(Goal);
            Table[AgentIndex][Goal->Id] = 0;

            while (!Open.empty())
            {
                FVertex* Current = Open.front();
                Open.pop();

                const int32 CurrentDistance = Table[AgentIndex][Current->Id];
                for (FVertex* Neighbor : Current->Neighbors)
                {
                    int32& NeighborDistance = Table[AgentIndex][Neighbor->Id];
                    if (CurrentDistance + 1 >= NeighborDistance)
                    {
                        continue;
                    }

                    NeighborDistance = CurrentDistance + 1;
                    Open.push(Neighbor);
                }
            }
        }

        int32 VertexCount = 0;
        std::vector<std::vector<int32>> Table;
    };

    using FPIBTHeuristic = std::tuple<int32, int32, float>;

    class FPIBT
    {
    public:
        FPIBT(const FInstance* InInstance, FDistTable* InDistTable, int32 InSeed)
            : Instance(InInstance)
            , MT(InSeed)
            , RandomReal(0.0f, 1.0f)
            , AgentCount(InInstance->Starts.size())
            , NoAgent(AgentCount)
            , DistTable(InDistTable)
            , OccupiedNow(InInstance->Graph.Size(), NoAgent)
            , OccupiedNext(InInstance->Graph.Size(), NoAgent)
        {
        }

        bool SetNewConfig(const Config& QFrom, Config& QTo, const std::vector<int32>& Order)
        {
            bool bSuccess = true;

            for (int32 AgentIndex = 0; AgentIndex < AgentCount; ++AgentIndex)
            {
                OccupiedNow[QFrom[AgentIndex]->Id] = AgentIndex;

                if (QTo[AgentIndex] != nullptr)
                {
                    if (OccupiedNext[QTo[AgentIndex]->Id] != NoAgent)
                    {
                        bSuccess = false;
                        break;
                    }

                    const int32 BlockingAgent = OccupiedNow[QTo[AgentIndex]->Id];
                    if (BlockingAgent != NoAgent
                        && BlockingAgent != AgentIndex
                        && QTo[BlockingAgent] == QFrom[AgentIndex])
                    {
                        bSuccess = false;
                        break;
                    }

                    OccupiedNext[QTo[AgentIndex]->Id] = AgentIndex;
                }
            }

            if (bSuccess)
            {
                for (int32 AgentIndex : Order)
                {
                    if (QTo[AgentIndex] == nullptr && !FuncPIBT(AgentIndex, QFrom, QTo))
                    {
                        bSuccess = false;
                        break;
                    }
                }
            }

            for (int32 AgentIndex = 0; AgentIndex < AgentCount; ++AgentIndex)
            {
                OccupiedNow[QFrom[AgentIndex]->Id] = NoAgent;
                if (QTo[AgentIndex] != nullptr)
                {
                    OccupiedNext[QTo[AgentIndex]->Id] = NoAgent;
                }
            }

            return bSuccess;
        }

    private:
        bool FuncPIBT(int32 AgentIndex, const Config& QFrom, Config& QTo)
        {
            std::vector<int32> NeighborAgents;
            NeighborAgents.reserve(QFrom[AgentIndex]->Neighbors.size());
            for (FVertex* Neighbor : QFrom[AgentIndex]->Neighbors)
            {
                const int32 NeighborAgent = OccupiedNow[Neighbor->Id];
                if (NeighborAgent != NoAgent)
                {
                    NeighborAgents.push_back(NeighborAgent);
                }
            }

            auto GetSuccessorCost =
                [&](FVertex* Vertex, bool bSwap = false) -> FPIBTHeuristic
                {
                    const float TieBreak = RandomReal(MT);
                    if (bSwap)
                    {
                        return std::make_tuple(-DistTable->Get(AgentIndex, Vertex), 0, TieBreak);
                    }

                    int32 Hindrance = 0;
                    for (int32 NeighborAgent : NeighborAgents)
                    {
                        if (QFrom[NeighborAgent] != Vertex
                            && DistTable->Get(NeighborAgent, Vertex) < DistTable->Get(NeighborAgent, QFrom[NeighborAgent]))
                        {
                            Hindrance += 1;
                        }
                    }

                    return std::make_tuple(DistTable->Get(AgentIndex, Vertex), Hindrance, TieBreak);
                };

            const std::vector<FVertex*>& Candidates = QFrom[AgentIndex]->Actions;
            std::vector<FPIBTHeuristic> Costs(Candidates.size());
            std::vector<int32> CandidateOrder(Candidates.size(), 0);
            std::iota(CandidateOrder.begin(), CandidateOrder.end(), 0);

            for (size_t CandidateIndex = 0; CandidateIndex < Candidates.size(); ++CandidateIndex)
            {
                Costs[CandidateIndex] = GetSuccessorCost(Candidates[CandidateIndex]);
            }

            std::sort(
                CandidateOrder.begin(),
                CandidateOrder.end(),
                [&](int32 LeftIndex, int32 RightIndex)
                {
                    return Costs[LeftIndex] < Costs[RightIndex];
                });

            const int32 BestCandidateIndex = CandidateOrder.empty() ? INDEX_NONE : CandidateOrder[0];
            const int32 SwapAgent =
                BestCandidateIndex == INDEX_NONE
                ? NoAgent
                : IsSwapRequiredAndPossible(AgentIndex, QFrom, QTo, Candidates[BestCandidateIndex]);

            if (SwapAgent != NoAgent)
            {
                for (size_t CandidateIndex = 0; CandidateIndex < Candidates.size(); ++CandidateIndex)
                {
                    Costs[CandidateIndex] = GetSuccessorCost(Candidates[CandidateIndex], true);
                    CandidateOrder[CandidateIndex] = CandidateIndex;
                }

                std::sort(
                    CandidateOrder.begin(),
                    CandidateOrder.end(),
                    [&](int32 LeftIndex, int32 RightIndex)
                    {
                        return Costs[LeftIndex] < Costs[RightIndex];
                    });
            }

            auto PerformSwap = [&]()
                {
                    if (SwapAgent != NoAgent
                        && QTo[SwapAgent] == nullptr
                        && OccupiedNext[QFrom[AgentIndex]->Id] == NoAgent)
                    {
                        OccupiedNext[QFrom[AgentIndex]->Id] = SwapAgent;
                        QTo[SwapAgent] = QFrom[AgentIndex];
                    }
                };

            for (size_t OrderIndex = 0; OrderIndex < CandidateOrder.size(); ++OrderIndex)
            {
                FVertex* NextVertex = Candidates[CandidateOrder[OrderIndex]];
                if (OccupiedNext[NextVertex->Id] != NoAgent)
                {
                    continue;
                }

                const int32 BlockingAgent = OccupiedNow[NextVertex->Id];
                if (BlockingAgent != NoAgent && QTo[BlockingAgent] == QFrom[AgentIndex])
                {
                    continue;
                }

                OccupiedNext[NextVertex->Id] = AgentIndex;
                QTo[AgentIndex] = NextVertex;

                if (BlockingAgent != NoAgent
                    && NextVertex != QFrom[AgentIndex]
                    && QTo[BlockingAgent] == nullptr
                    && !FuncPIBT(BlockingAgent, QFrom, QTo))
                {
                    continue;
                }

                if (OrderIndex == 0)
                {
                    PerformSwap();
                }

                return true;
            }

            OccupiedNext[QFrom[AgentIndex]->Id] = AgentIndex;
            QTo[AgentIndex] = QFrom[AgentIndex];
            return false;
        }

        int32 IsSwapRequiredAndPossible(
            int32 AgentIndex,
            const Config& QFrom,
            Config& QTo,
            FVertex* TargetVertex)
        {
            const int32 BlockingAgent = OccupiedNow[TargetVertex->Id];
            if (BlockingAgent != NoAgent
                && BlockingAgent != AgentIndex
                && QTo[BlockingAgent] == nullptr
                && IsSwapRequired(AgentIndex, BlockingAgent, QFrom[AgentIndex], QFrom[BlockingAgent])
                && IsSwapPossible(QFrom[BlockingAgent], QFrom[AgentIndex]))
            {
                return BlockingAgent;
            }

            if (TargetVertex != QFrom[AgentIndex])
            {
                for (FVertex* Neighbor : QFrom[AgentIndex]->Neighbors)
                {
                    const int32 CandidateAgent = OccupiedNow[Neighbor->Id];
                    if (CandidateAgent != NoAgent
                        && TargetVertex != QFrom[CandidateAgent]
                        && IsSwapRequired(CandidateAgent, AgentIndex, QFrom[AgentIndex], TargetVertex)
                        && IsSwapPossible(TargetVertex, QFrom[AgentIndex]))
                    {
                        return CandidateAgent;
                    }
                }
            }

            return NoAgent;
        }

        bool IsSwapRequired(
            int32 PusherAgent,
            int32 PullerAgent,
            FVertex* PusherOrigin,
            FVertex* PullerOrigin)
        {
            FVertex* PusherVertex = PusherOrigin;
            FVertex* PullerVertex = PullerOrigin;
            FVertex* TempVertex = nullptr;

            while (DistTable->Get(PusherAgent, PullerVertex) < DistTable->Get(PusherAgent, PusherVertex))
            {
                int32 NeighborCount = PullerVertex->Neighbors.size();
                for (FVertex* Neighbor : PullerVertex->Neighbors)
                {
                    const int32 OccupyingAgent = OccupiedNow[Neighbor->Id];
                    if (Neighbor == PusherVertex
                        || (Neighbor->Neighbors.size() == 1
                            && OccupyingAgent != NoAgent
                            && Instance->Goals[OccupyingAgent] == Neighbor))
                    {
                        NeighborCount -= 1;
                    }
                    else
                    {
                        TempVertex = Neighbor;
                    }
                }

                if (NeighborCount >= 2)
                {
                    return false;
                }

                if (NeighborCount <= 0)
                {
                    break;
                }

                PusherVertex = PullerVertex;
                PullerVertex = TempVertex;
            }

            return DistTable->Get(PullerAgent, PusherVertex) < DistTable->Get(PullerAgent, PullerVertex)
                && (DistTable->Get(PusherAgent, PusherVertex) == 0
                    || DistTable->Get(PusherAgent, PullerVertex) < DistTable->Get(PusherAgent, PusherVertex));
        }

        bool IsSwapPossible(FVertex* PusherOrigin, FVertex* PullerOrigin)
        {
            FVertex* PusherVertex = PusherOrigin;
            FVertex* PullerVertex = PullerOrigin;
            FVertex* TempVertex = nullptr;

            while (PullerVertex != PusherOrigin)
            {
                int32 NeighborCount = PullerVertex->Neighbors.size();
                for (FVertex* Neighbor : PullerVertex->Neighbors)
                {
                    const int32 OccupyingAgent = OccupiedNow[Neighbor->Id];
                    if (Neighbor == PusherVertex
                        || (Neighbor->Neighbors.size() == 1
                            && OccupyingAgent != NoAgent
                            && Instance->Goals[OccupyingAgent] == Neighbor))
                    {
                        NeighborCount -= 1;
                    }
                    else
                    {
                        TempVertex = Neighbor;
                    }
                }

                if (NeighborCount >= 2)
                {
                    return true;
                }

                if (NeighborCount <= 0)
                {
                    return false;
                }

                PusherVertex = PullerVertex;
                PullerVertex = TempVertex;
            }

            return false;
        }

        const FInstance* Instance = nullptr;
        std::mt19937 MT;
        std::uniform_real_distribution<float> RandomReal;
        int32 AgentCount = 0;
        int32 NoAgent = 0;
        FDistTable* DistTable = nullptr;
        std::vector<int32> OccupiedNow;
        std::vector<int32> OccupiedNext;
    };

    struct FLNode
    {
        FLNode() = default;

        FLNode(FLNode* Parent, int32 InAgentIndex, FVertex* InVertex)
            : AgentIndices(Parent->AgentIndices)
            , AssignedVertices(Parent->AssignedVertices)
            , Depth(Parent->Depth + 1)
        {
            AgentIndices.push_back(InAgentIndex);
            AssignedVertices.push_back(InVertex);
        }

        std::vector<int32> AgentIndices;
        std::vector<FVertex*> AssignedVertices;
        uint32 Depth = 0;
    };

    struct FHNode;

    struct FCompareHNodePointers
    {
        bool operator()(const FHNode* Left, const FHNode* Right) const;
    };

    struct FHNode
    {
        FHNode(Config InConfig, FDistTable* DistTable, FHNode* InParent = nullptr, int32 InG = 0, int32 InH = 0)
            : Q(std::move(InConfig))
            , Parent(InParent)
            , G(InG)
            , H(InH)
            , F(G + H)
            , Depth(InParent == nullptr ? 0 : InParent->Depth + 1)
            , Priorities(Q.size(), 0.0f)
            , Order(Q.size(), 0)
        {
            if (Parent != nullptr)
            {
                Parent->Neighbors.insert(this);
            }

            SearchTree.push(new FLNode());

            for (int32 AgentIndex = 0; AgentIndex < static_cast<int32>(Q.size()); ++AgentIndex)
            {
                if (Parent == nullptr)
                {
                    Priorities[AgentIndex] = static_cast<float>(DistTable->Get(AgentIndex, Q[AgentIndex])) / 10000.0f;
                }
                else if (DistTable->Get(AgentIndex, Q[AgentIndex]) != 0)
                {
                    Priorities[AgentIndex] = Parent->Priorities[AgentIndex] + 1.0f;
                }
                else
                {
                    Priorities[AgentIndex] =
                        Parent->Priorities[AgentIndex] - FMath::FloorToFloat(Parent->Priorities[AgentIndex]);
                }
            }

            std::iota(Order.begin(), Order.end(), 0);
            std::sort(
                Order.begin(),
                Order.end(),
                [&](int32 LeftIndex, int32 RightIndex)
                {
                    return Priorities[LeftIndex] > Priorities[RightIndex];
                });
        }

        ~FHNode()
        {
            while (!SearchTree.empty())
            {
                delete SearchTree.front();
                SearchTree.pop();
            }
        }

        const Config Q;
        FHNode* Parent = nullptr;
        std::set<FHNode*, FCompareHNodePointers> Neighbors;
        int32 G = 0;
        int32 H = 0;
        int32 F = 0;
        int32 Depth = 0;
        std::vector<float> Priorities;
        std::vector<int32> Order;
        std::queue<FLNode*> SearchTree;
    };

    bool FCompareHNodePointers::operator()(const FHNode* Left, const FHNode* Right) const
    {
        const size_t Count = Left->Q.size();
        for (size_t Index = 0; Index < Count; ++Index)
        {
            if (Left->Q[Index] != Right->Q[Index])
            {
                return Left->Q[Index]->Id < Right->Q[Index]->Id;
            }
        }

        return false;
    }

    class FSolver
    {
    public:
        FSolver(
            const FInstance* InInstance,
            FDistTable* InDistTable,
            const FDeadline* InDeadline,
            int32 InSeed,
            bool bInAnytime,
            int32 InVerboseLevel)
            : Instance(InInstance)
            , DistTable(InDistTable)
            , Deadline(InDeadline)
            , MT(InSeed)
            , RandomReal(0.0f, 1.0f)
            , VerboseLevel(InVerboseLevel)
            , PIBT(InInstance, InDistTable, InSeed)
            , bAnytime(bInAnytime)
        {
        }

        Solution Solve()
        {
            std::unordered_map<Config, FHNode*, FConfigHasher> Explored;
            std::vector<FHNode*> AllNodes;

            FHNode* InitialNode = new FHNode(Instance->Starts, DistTable);
            Open.push_front(InitialNode);
            Explored[InitialNode->Q] = InitialNode;
            AllNodes.push_back(InitialNode);

            while (!Open.empty() && !Deadline->IsExpired())
            {
                ++LoopCount;

                if (GoalNode != nullptr)
                {
                    const float Roll = RandomReal(MT);
                    if (Roll < RandomInsertProb2 / 2.0f)
                    {
                        Open.push_front(InitialNode);
                    }
                    else if (Roll < RandomInsertProb2)
                    {
                        const int32 RandomIndex =
                            std::uniform_int_distribution<int32>(0, static_cast<int32>(Open.size()) - 1)(MT);
                        Open.push_front(Open[RandomIndex]);
                    }
                }

                FHNode* CurrentNode = Open.front();

                if (GoalNode != nullptr && CurrentNode->F >= GoalNode->G)
                {
                    Open.pop_front();
                    Open.push_front(InitialNode);
                    continue;
                }

                if (GoalNode == nullptr && IsSameConfig(CurrentNode->Q, Instance->Goals))
                {
                    GoalNode = CurrentNode;

                    if (VerboseLevel > 0)
                    {
                        UE_LOG(
                            LogTemp,
                            Warning,
                            TEXT("LaCAM: solution found. Cost=%d Depth=%d Loop=%d Elapsed=%.2f ms"),
                            GoalNode->G,
                            GoalNode->Depth,
                            LoopCount,
                            Deadline->ElapsedMs());
                    }

                    if (!bAnytime)
                    {
                        break;
                    }

                    continue;
                }

                if (CurrentNode->SearchTree.empty())
                {
                    Open.pop_front();
                    continue;
                }

                FLNode* LowLevelNode = CurrentNode->SearchTree.front();
                CurrentNode->SearchTree.pop();

                if (LowLevelNode->Depth < CurrentNode->Q.size())
                {
                    const int32 AgentIndex = CurrentNode->Order[LowLevelNode->Depth];
                    std::vector<FVertex*> CandidateActions = CurrentNode->Q[AgentIndex]->Actions;
                    std::shuffle(CandidateActions.begin(), CandidateActions.end(), MT);
                    for (FVertex* Candidate : CandidateActions)
                    {
                        CurrentNode->SearchTree.push(new FLNode(LowLevelNode, AgentIndex, Candidate));
                    }
                }

                Config NextConfig(Instance->Starts.size(), nullptr);
                const bool bHasValidSuccessor = SetNewConfig(CurrentNode, LowLevelNode, NextConfig);
                delete LowLevelNode;

                if (!bHasValidSuccessor)
                {
                    continue;
                }

                auto ExistingNodeIt = Explored.find(NextConfig);
                if (ExistingNodeIt == Explored.end())
                {
                    const int32 GValue = GetGValue(CurrentNode, NextConfig);
                    const int32 HValue = GetHValue(NextConfig);
                    FHNode* NewNode = new FHNode(std::move(NextConfig), DistTable, CurrentNode, GValue, HValue);
                    Open.push_front(NewNode);
                    Explored[NewNode->Q] = NewNode;
                    AllNodes.push_back(NewNode);
                }
                else
                {
                    Rewrite(CurrentNode, ExistingNodeIt->second);

                    if (RandomReal(MT) >= RandomInsertProb1)
                    {
                        Open.push_front(ExistingNodeIt->second);
                    }
                    else
                    {
                        Open.push_front(InitialNode);
                    }
                }
            }

            Solution Result;
            for (FHNode* Node = GoalNode; Node != nullptr; Node = Node->Parent)
            {
                Result.push_back(Node->Q);
            }

            std::reverse(Result.begin(), Result.end());

            for (FHNode* Node : AllNodes)
            {
                delete Node;
            }

            return Result;
        }

    private:
        bool SetNewConfig(FHNode* HighLevelNode, FLNode* LowLevelNode, Config& OutConfig)
        {
            for (uint32 DepthIndex = 0; DepthIndex < LowLevelNode->Depth; ++DepthIndex)
            {
                OutConfig[LowLevelNode->AgentIndices[DepthIndex]] = LowLevelNode->AssignedVertices[DepthIndex];
            }

            return PIBT.SetNewConfig(HighLevelNode->Q, OutConfig, HighLevelNode->Order);
        }

        void Rewrite(FHNode* FromNode, FHNode* ToNode)
        {
            if (!bAnytime)
            {
                return;
            }

            FromNode->Neighbors.insert(ToNode);

            std::queue<FHNode*> Queue;
            Queue.push(FromNode);

            while (!Queue.empty())
            {
                FHNode* SourceNode = Queue.front();
                Queue.pop();

                for (FHNode* NeighborNode : SourceNode->Neighbors)
                {
                    const int32 GValue = SourceNode->G + GetEdgeCost(SourceNode->Q, NeighborNode->Q);
                    if (GValue < NeighborNode->G)
                    {
                        NeighborNode->G = GValue;
                        NeighborNode->F = NeighborNode->G + NeighborNode->H;
                        NeighborNode->Parent = SourceNode;
                        NeighborNode->Depth = SourceNode->Depth + 1;
                        Queue.push(NeighborNode);

                        if (GoalNode != nullptr && NeighborNode->F < GoalNode->F)
                        {
                            Open.push_front(NeighborNode);
                        }
                    }
                }
            }
        }

        int32 GetGValue(FHNode* ParentNode, const Config& NextConfig) const
        {
            return ParentNode->G + GetEdgeCost(ParentNode->Q, NextConfig);
        }

        int32 GetHValue(const Config& InConfig) const
        {
            int32 Heuristic = 0;
            for (int32 AgentIndex = 0; AgentIndex < static_cast<int32>(Instance->Starts.size()); ++AgentIndex)
            {
                Heuristic += DistTable->Get(AgentIndex, InConfig[AgentIndex]);
            }

            return Heuristic;
        }

        int32 GetEdgeCost(const Config& FromConfig, const Config& ToConfig) const
        {
            int32 Cost = 0;
            for (int32 AgentIndex = 0; AgentIndex < static_cast<int32>(Instance->Starts.size()); ++AgentIndex)
            {
                if (FromConfig[AgentIndex] != Instance->Goals[AgentIndex]
                    || ToConfig[AgentIndex] != Instance->Goals[AgentIndex])
                {
                    Cost += 1;
                }
            }

            return Cost;
        }

        const FInstance* Instance = nullptr;
        FDistTable* DistTable = nullptr;
        const FDeadline* Deadline = nullptr;
        std::mt19937 MT;
        std::uniform_real_distribution<float> RandomReal;
        int32 VerboseLevel = 0;
        FPIBT PIBT;
        FHNode* GoalNode = nullptr;
        std::deque<FHNode*> Open;
        int32 LoopCount = 0;
        bool bAnytime = false;
        static constexpr float RandomInsertProb1 = 0.001f;
        static constexpr float RandomInsertProb2 = 0.001f;
    };
}

FLaCAMPlanner::FLaCAMPlanner(
    double InTimeLimitMs,
    int32 InRandomSeed,
    bool bInAnytime,
    int32 InVerboseLevel)
    : TimeLimitMs(FMath::Max(0.0, InTimeLimitMs))
    , RandomSeed(InRandomSeed)
    , bAnytime(bInAnytime)
    , VerboseLevel(FMath::Max(0, InVerboseLevel))
{
}

bool FLaCAMPlanner::PlanMissions(
    const FGridMap3D& GridMap,
    const TArray<FDroneMissionConfig>& Missions,
    TMap<int32, TArray<FVector>>& OutPaths)
{
    OutPaths.Reset();

    if (Missions.Num() <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("LaCAM: no missions to plan"));
        return false;
    }

    if (!ValidateMissionIdsUniqueLaCAM(Missions))
    {
        return false;
    }

    TArray<FDroneMissionConfig> OrderedMissions = Missions;
    OrderedMissions.Sort(
        [](const FDroneMissionConfig& Left, const FDroneMissionConfig& Right)
        {
            return Left.MissionId < Right.MissionId;
        });

    LaCAMUE::FInstance Instance(GridMap);
    if (Instance.Graph.Size() <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("LaCAM: grid graph contains no traversable cells"));
        return false;
    }

    TSet<FIntVector> SeenStartCells;
    TSet<FIntVector> SeenGoalCells;
    TArray<int32> MissionIds;
    MissionIds.Reserve(OrderedMissions.Num());

    for (const FDroneMissionConfig& Mission : OrderedMissions)
    {
        const FIntVector StartCell = GridMap.WorldToCell(Mission.StartWorld);
        const FIntVector GoalCell = GridMap.WorldToCell(Mission.GoalWorld);

        if (!GridMap.IsInside(StartCell.X, StartCell.Y, StartCell.Z))
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("LaCAM: mission %d start cell is outside grid: (%d,%d,%d)"),
                Mission.MissionId,
                StartCell.X,
                StartCell.Y,
                StartCell.Z);
            return false;
        }

        if (!GridMap.IsInside(GoalCell.X, GoalCell.Y, GoalCell.Z))
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("LaCAM: mission %d goal cell is outside grid: (%d,%d,%d)"),
                Mission.MissionId,
                GoalCell.X,
                GoalCell.Y,
                GoalCell.Z);
            return false;
        }

        if (SeenStartCells.Contains(StartCell))
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("LaCAM: duplicate start cell detected for mission %d at (%d,%d,%d)"),
                Mission.MissionId,
                StartCell.X,
                StartCell.Y,
                StartCell.Z);
            return false;
        }

        if (SeenGoalCells.Contains(GoalCell))
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("LaCAM: duplicate goal cell detected for mission %d at (%d,%d,%d)"),
                Mission.MissionId,
                GoalCell.X,
                GoalCell.Y,
                GoalCell.Z);
            return false;
        }

        LaCAMUE::FVertex* StartVertex = Instance.Graph.GetVertex(StartCell);
        LaCAMUE::FVertex* GoalVertex = Instance.Graph.GetVertex(GoalCell);
        if (StartVertex == nullptr)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("LaCAM: mission %d start cell is blocked: (%d,%d,%d)"),
                Mission.MissionId,
                StartCell.X,
                StartCell.Y,
                StartCell.Z);
            return false;
        }

        if (GoalVertex == nullptr)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("LaCAM: mission %d goal cell is blocked: (%d,%d,%d)"),
                Mission.MissionId,
                GoalCell.X,
                GoalCell.Y,
                GoalCell.Z);
            return false;
        }

        SeenStartCells.Add(StartCell);
        SeenGoalCells.Add(GoalCell);
        Instance.Starts.push_back(StartVertex);
        Instance.Goals.push_back(GoalVertex);
        MissionIds.Add(Mission.MissionId);
    }

    LaCAMUE::FDistTable DistTable(Instance);
    LaCAMUE::FDeadline Deadline(TimeLimitMs);
    LaCAMUE::FSolver Solver(&Instance, &DistTable, &Deadline, RandomSeed, bAnytime, VerboseLevel);

    if (VerboseLevel > 0)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("LaCAM: planning %d missions, time limit %.2f ms, seed %d, anytime=%s"),
            OrderedMissions.Num(),
            TimeLimitMs,
            RandomSeed,
            bAnytime ? TEXT("true") : TEXT("false"));
    }

    const LaCAMUE::Solution Solution = Solver.Solve();
    if (Solution.empty())
    {
        UE_LOG(LogTemp, Error, TEXT("LaCAM: failed to find a solution within %.2f ms"), TimeLimitMs);
        return false;
    }

    for (int32 AgentIndex = 0; AgentIndex < MissionIds.Num(); ++AgentIndex)
    {
        TArray<FVector> PathPoints;
        PathPoints.Reserve(Solution.size());

        for (const LaCAMUE::Config& ConfigAtTime : Solution)
        {
            PathPoints.Add(GridMap.CellToWorld(ConfigAtTime[AgentIndex]->Cell));
        }

        OutPaths.Add(MissionIds[AgentIndex], MoveTemp(PathPoints));
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("LaCAM: solved %d missions. Makespan=%d Elapsed=%.2f ms"),
        OrderedMissions.Num(),
        Solution.size() > 0 ? Solution.size() - 1 : 0,
        Deadline.ElapsedMs());

    return true;
}
