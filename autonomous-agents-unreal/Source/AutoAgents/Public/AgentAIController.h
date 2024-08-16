#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "NavigationSystemMapVolume.h"
#include <variant>
#include <random>
#include "AgentAIController.generated.h"
class ANavigationSystemMapVolume;
class AAgentsGroupController;

UENUM(BlueprintType)
enum class EnumAgentState : uint8 {
	VE_Normal       UMETA(DisplayName = "Normal"),
	VE_Sit        UMETA(DisplayName = "Sit"),
	VE_CellPhone        UMETA(DisplayName = "CellPhone"),
	VE_QueueUp        UMETA(DisplayName = "QueueUp"),
	VE_Chat        UMETA(DisplayName = "Chat"),
};

UENUM(BlueprintType)
enum class EnumEnvironmentalState : uint8 {
	VE_Kneel       UMETA(DisplayName = "Kneel"),
	VE_PickUp       UMETA(DisplayName = "PickUp"),
	VE_WatchPerformance       UMETA(DisplayName = "WatchPerformance"),
	VE_Sit       UMETA(DisplayName = "Sit"),
	VE_Performance       UMETA(DisplayName = "Performance"),
	VE_Watch       UMETA(DisplayName = "Watch"),
	VE_Drink       UMETA(DisplayName = "Drink"),
	VE_None       UMETA(DisplayName = "None"),
	VE_Chat       UMETA(DisplayName = "Chat"),
	VE_Wait       UMETA(DisplayName = "Wait"),
	VE_Ponder       UMETA(DisplayName = "Ponder"),
	VE_Rest       UMETA(DisplayName = "Rest"),
	VE_CallPhone       UMETA(DisplayName = "CallPhone"),
	VE_Shopping       UMETA(DisplayName = "Shopping"),
	VE_OpenDoor       UMETA(DisplayName = "OpenDoor"),
	VE_Exhaustion       UMETA(DisplayName = "Exhaustion"),
};

// We classify pedestrians in the virtual train station environment as commuters, tourists, law enforcement officers, performers, etc.
UENUM(BlueprintType)
enum class PedestrianType {
	Commuter,
	Tourist,
	Policeman,
	Performer,
};

enum class AgentGoalType : uint8 {
	GoToRandomPosition = 0, // NOTE(changyu): Debug option.
	GoAndWatch = 1,
	TakeARest = 2,
	BuyADrink = 3,
	GoToThePlatform = 4,
	BuyATicket = 5,
	GoToTicketOfficeEntrance = 6,
};

UCLASS()
class AUTOAGENTS_API AAgentAIController : public AAIController
{
	GENERATED_BODY()

public:
	AAgentAIController();

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

private:
	friend AAgentsGroupController;
	// Global navigation map for path planning and obstacles avoidance
	ANavigationSystemMapVolume* NavigationMapPtr;
	// Global Agents group unified coordination control
	AAgentsGroupController* GroupControllerPtr;

	void InitMentalState();
	void UpdateMentalState(float DeltaTime);
	void UpdateCommuterMentalState(float DeltaTime);
	void UpdateTouristMentalState(float DeltaTime);
	void UpdatePolicemanMentalState(float DeltaTime);
	void UpdatePerformerMentalState(float DeltaTime);

	// Check whether the newly added target has a higher priority than the previous target
	bool IsHigherPriority(AgentGoalType NewGoal) const;
	// Execute the animation sequence required to accomplish a specific goal
	void SelectAction(EnumAgentState AgentState, 
		EnumEnvironmentalState EnvironmentalState = EnumEnvironmentalState::VE_None);
	// Remove the latest goal from the goal stack after completing it
	void PopbackCurrentState();

	FVector2D CurrentLocation() const;
	bool IsVisible(FVector2D Position, bool AllowAirWall = false) const;
	bool HasAgentInFrontSafeArea(float DeltaTime);
	bool HasAgentInDirection(float DeltaTime, FVector2D MovingDir) const;
	void UpdateFarthestVisiblePoint(bool bResetPath = false);
	void UpdateVelocityFromPathPlanning(float DeltaTime);
	void UpdateVelocityToAvoidOnComingAgents(float DeltaTime);
	bool IsGoalNeedsToWaitInLine() const;
	bool IsSameGoalWithOnComingAgent() const;
	bool IsWaitingInLine(float DeltaTime) const;
	bool SetTargetPositionAndFacing(FVector TargetPosition, FVector2D TargetFacing);
	bool ResetCurrentTargetPositionAndFacing();
	void FindNeighboringAgents();
	void MergeRoutineUpdate();
	void UpdateFinalMovement(float DeltaTime);
	bool IsArrivedTarget() const;
	
	// Routine E: Avoid dangerously close pedestrians
	// This is the fail-safe behavior routine,
	// reserved for emergencies due to the occasional failure of Routines C and D,
	// since in highly dynamic situations predictions have a nonzero probability of being incorrect.
	// Once a pedestrian perceives another pedestrian within its front safe area (Fig. 5(E)),
	// it will resort to a simple but effective behavior—brake as soon as possible to a full stop,
	// then try to turn to face away from the intruder, and proceed when the way ahead clears.
	void RoutineE(float DeltaTime);

	// Routine C: Maintain separation in a moving crowd
	// For a pedestrian H, other pedestrians are considered to be in H's temporary crowd 
	// if they are moving in a similar direction to H and are situated within a parabolic region 
	// in front of H defined by y = −(4 / R)x2 + R where R is the sensing range,
	// y is oriented in H's forward direction and x is oriented laterally (Fig. 5(C)). 
	// To maintain a comfortable distance from each individual Ci in this temporary crowd, a directed repulsive force.
	void RoutineC(float DeltaTime);

	// Routine D: Avoid oncoming pedestrians
	// To avoid pedestrians not in one's temporary crowd,
	// a pedestrian H estimates its own velocity v and the velocities vi of neighbors pedestrians Ci.
	// Two types of threats are considered here.
	// By intersecting its own linearly extrapolated trajectory T with the trajectories Ti of each of the Ci,
	// pedestrian H identifies potential collision threats of the first type:
	// cross-collision (Fig. 5(D1)).
	// In the case where the trajectories of H and Ci are almost parallel and will not intersect imminently,
	// a head-on collision (Fig. 5(D2)) may still occur if their lateral separation is too small;
	// hence, H measures its lateral separation from oncoming pedestrians.
	void RoutineD(float DeltaTime);

	// Routine A: Static obstacle avoidance
	// If there is a nearby obstacle in the direction of locomotion,
	// lateral directions to the left and right are tested
	// until a less cluttered direction is found (Fig. 4(b)).
	// If a large angle (currently set to 90-deg) must be swept before a good direction is found,
	// then the pedestrian will start to slow down,
	// which mimics the behavior of a real person upon encountering a tough array of obstacles;
	// i.e., slow down while turning the head to look around, then proceed.
	void RoutineA(float DeltaTime);

	// Routine F: Verify new directions relative to obstacles
	// Since the reactive behavior routines are executed sequentially (see Section 4.3.1),
	// motor control commands issued by Routines C, D or E
	// to avoid pedestrians may counteract those issued by Routines A or B to avoid obstacles,
	// thus steering the pedestrian towards obstacles again.
	// To avoid this, the pedestrian checks the new direction against surrounding obstacles once more.
	// If the way is clear, it proceeds. Otherwise, the original direction issued by
	// either the higher-level path planning modules or by Routine A,
	// whichever was executed most recently prior to the execution of Routine F,
	// will be used instead. However,
	// occasionally this could lead the pedestrian toward future collisions with other pedestrians (Fig. 5(F))
	// and, if so, it will simply slow down to a stop,
	// let those threatening pedestrians pass, and proceed.
	void RoutineF(float DeltaTime);

	// Routine B: Static obstacle avoidance in a complex turn
	// When a pedestrian needs to make a turn that cannot be finished in one step,
	// it will consider turns with increasing curvatures in both directions,
	// starting with the side that permits the smaller turning angle,
	// until a collision-free turn is found (Fig. 5(B)).
	// If the surrounding space is too cluttered,
	// the curve is likely to degenerate, causing the pedestrian to stop and turn on the spot.
	// The turn test is implemented by checking sample points along a curve
	// with interval equal to the distance of one step of the pedestrian
	// moving with the anticipated turn speed.
	void RoutineB(float DeltaTime);

	// Utils
	static void DecomposeVector(const FVector2D& Vector,
		const FVector2D& Facing,
		FVector2D& Nor,
		FVector2D& Tan);
	static FVector2D To2D(const FVector& Vector);
	static FVector To3D(const FVector2D& Vector2D, float Z = 0.f);

	// Debug Toolkits
	void StuckWarning();
	void DrawDebugState(FColor Color, float Height = 100.f) const;
	void DrawDebugFacing(FVector2D F, FColor Color) const;

	// Variables
	//

	int ID; // Assigned by high-level AgentsGroupController
	TArray<AAgentAIController const*> Neighbors;

	// Random generator per agent
	std::mt19937 Rnd;

	FVector2D Velocity;
	FVector2D Facing;

	// Each pedestrian maintains a set of internal mental state variables, which encodes the pedestrian's current physiological,
	// psychological or social needs. 
	// These variables include tiredness, thirst, curiosity, the propensity to be attracted by performances, the need to acquire a ticket, etc.
	// When the value of a mental state variable exceeds a specified threshold, 
	// an action selection mechanism chooses the appropriate behavior to fulfill the need.Once a need is fulfilled, the value of the
	// associated internal state variable begins to decrease asymptotically to zero.
	using AgentGoalDetail = std::variant<int, AActor*>;
	using AgentGoal = std::pair<AgentGoalType, AgentGoalDetail>;
	struct _MentalStVar {
		// Each pedestrian type has an associated action selection mechanism with appropriately set behavior triggering thresholds associated with mental state variables.
		PedestrianType AgentType;

		// The mental state variables dictate "why it should be done".
		float Tiredness;
		float Thirst;
		float Curiosity;
		bool bNeedATicket;

		float DurationTime;
		std::deque<float> DurationThreshold;

		// Every pedestrian maintains a stack of goals, the top one being the current goal.
		// Intuitively, the goal stack remembers "what needs doing".
		std::vector<AgentGoal> GoalStack;
		std::vector<AgentGoal> CompletedGoalStack;
	} MentalStVar;

	enum class EnumArriveState {
		None,
		ArrivedStartMid,
		ArrivedEndMid,
		ArrivedTarget,
	};

	struct _NaviVar {
		EnumArriveState ArriveState;
		// A valid mid-point location in Quad-tree map nearest start position.
		FVector StartMidPosition;
		// A valid mid-point location in Quad-tree map nearest end position.
		FVector EndMidPosition;
		std::deque<FVector> TargetPosition;
		// Some tasks, e.g., sit, requires specific facing.
		std::deque<FVector2D> TargetFacing;
		// The farthest point along path such that there is no obstacle on the line between p and the pedestrian H's current
		// position, is determined and set as an intermediate target.
		FVector2D VisiblePosition;
		// Path between pedestrian's current position and target position.
		ANavigationSystemMapVolume::Path NavigationPath;
		// Used to track visible position is continuous along navigation path.
		int NavigationStop;
	} NaviVar;

	struct _ReactRtVar {
		// Facing in previous time-step.
		FVector2D PreviousFacing;
		// Intermediate velocity and facing between reactive behavior routines.
		FVector2D ModifiedVelocity;
		FVector2D ModifiedFacing;
		// The original direction issued by either the higher-level path planning modules or by Routine A, 
		// whichever was executed most recently prior to the execution of Routine F.
		FVector2D BackupVelocity;
		FVector2D BackupFacing;

		bool OnComingBlock;
		AAgentAIController const * OnComingAgent;
		FVector2D AvoidFacing;
		float AvoidRemain = 0.f;
		bool DisableE;

		float BSlowDownFactor = 1.f;
		float ESlowDownFactor = 1.f;
		float FSlowDownFactor = 1.f;
		float DTurningFactor = 0.f;

		void Reset() {
			BSlowDownFactor = 1.f;
			ESlowDownFactor = 1.f;
			FSlowDownFactor = 1.f;
			DTurningFactor = 1.f;
		}
	} ReactRtVar;

	// Debug Stucked Agent
	struct _HistoryVar {
		const int HistoryListMaxLength = 25;
		const float AgentStuckThreshold = 1e-2;
		bool IsStucked;
		std::deque<FVector2D> HistoryPosList;
		std::deque<FVector2D> HistoryFacingList;
	} HistoryVar;

	// Constants
	//

	// This is motivated by the fact that at any given time,
	// people usually pay attention only to a limited number of other people
	static const int MaximumAttentionAgents;

	static const float AgentWalkVelocity;
	
	static const float TargetEps;
	// Pedestrian's visual sensing range (currently set to 5 meters)
	static const float AgentPerceptualRange;
	//120 - degree
	static const float CosSensingFanAngle;
	static const float VisibilitySegmentSamplingDensity;
	
	// predefined minimum distance allowed between H and other pedestrians
	// (usually 2.5 times H’s bounding box size).
	static const float PedestrianBoudingBoxSize;
	static const float PedestrianBoudingBoxScale;
	static const float MinimumDistanceAllowed;

	// If a large angle (currently set to 90-deg) must be swept before a good direction is found,
	// then the pedestrian will start to slow down,
	static const float RoutineASlowDownFactorRatio;
	static const float AngleDensity;

	// Act as an exponential damping factor for routine E
	static const float RoutineESlowDownFactorRatio;

	// Perceived "repulsiveness" to H (currently set to −0.025 for all pedestrians)
	static const float Repulsiveness;

	static const float CosSimilarity;
	static const float HeadonCosSimilarity;
	static const float Inertia;

	// The crowding factor wi determines H's willingness to "follow the crowd", 
	// with a smaller value of wi giving H a greater tendency to do so (currently 1.0 <= wi <= 5.0).
	static const float CrowdingFactor;
	static const float WaitingInLineCrowdingFactor;

	static const float IncreaseSpeedRatio;
	static const float DecreaseSpeedRatio;

	static const float TurningFactorRatio;
	static const float MaxHeadonCollisionAvoidanceDegree;
	static const float MaxCrossCollisionAvoidanceDegree;

	enum COLLSION_TYPE {
		INVALID = -1,
		HEADON_COLLISION = 0,
		CROSS_COLLISION = 1
	};

	static const float RoutineFSlowDownFactorRatio;
	static const float RoutineBSlowDownFactorRatio;
	static const float RoutineBUrgentSlowDownFactorRatio;
	static const float RoutineBTurningFactorThreshold;

	static const float AvoidOnComingAgentsDuration;
	static const float MaximumLineLength;
};
