#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>
#include <QElapsedTimer>
#include <QString>

#include <cstdint>
#include <vector>

#include "nav_core.h"
#include "nav_supervisor.h"
#include "sim_robot.h"
#include "sim_world.h"

class QLabel;
class QGraphicsEllipseItem;
class QGraphicsItem;
class QGraphicsLineItem;
class QGraphicsPolygonItem;
class QKeyEvent;
class QResizeEvent;
class QTimer;
class QTreeWidgetItem;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    enum class DeadEndRecoveryPhase {
        None,
        ApproachFront,
        Pivot180
    };

    enum class RouteExecuteStatus {
        Idle,
        NoRouteLoaded,
        NavBusy,
        StartNotOnRearLine,
        Started,
        Running,
        Completed,
        Cancelled
    };

    enum class CenterPivotSequencePhase {
        None,
        Centering,
        ApproachFront,
        Pivot180,
        Done,
        Failed
    };

    enum class PlanCompositePrepareMethod {
        None,
        FrontLine,
        FrontWall
    };

    enum class SmartRecognitionState {
        Idle,
        LocalUnvisited,
        PlanToFrontier,
        ExecutingFrontierRoute,
        FrontierAlreadyHere,
        NoFrontier,
        Error
    };

    enum class Mode1MissionState {
        Disabled,
        SearchSpecials,
        FoundRequiredSpecialsWaitActionDone,
        ReturnToStartPlan,
        ReturnToStartExecute,
        Done,
        Error
    };

    enum class Mode1MissionDoneReason {
        None,
        FoundRequiredSpecialsAndReturned,
        NoReturnRoute,
        ReturnRouteTooLong,
        ReturnQueueOverflow,
        StartCellInvalid,
        NoFrontierBeforeRequiredSpecials,
        Cancelled
    };

    enum class FloodFrontierDecision {
        None,
        TryFrontier,
        FallbackSafe
    };

    enum class FloodFrontierDecisionReason {
        None,
        NoFlood,
        CurrentCellUnreachable,
        NoFrontier,
        FrontierBetterThanSafeReturn,
        FrontierNotBetterThanSafeReturn
    };

    enum class FloodFrontierEntryRelative {
        None,
        Front,
        Right,
        Left,
        Back
    };

    enum class FloodFrontierEntryAction {
        None,
        AdvanceLine,
        SmoothRight,
        SmoothLeft,
        UnsupportedBackExit
    };

    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    void setupScene();
    void drawReferenceGrid();
    void drawBlackTape();
    void drawWorldWalls();
    int clearShadowMapOverlay();
    void updateShadowMapOverlay();
    void drawShadowMapOverlay();
    void markShadowMapOverlayDirty();
    void rebuildSceneItems();
    void initializeIrSensors();
    void initializeFloorSensors();
    void createTelemetryPanel();
    void updateTelemetryPanel();
    void syncPinnedTelemetryRows();
    void showControlsHelp();
    void showControlTuningDialog();
    void loadMazeFromDialog();
    bool loadMazeFile(const QString &path);
    void adjustSmoothTargetYawRate(int delta_deg_s);
    void promptSmoothTargetYawRate();
    void promptRoutePlanToCell();
    void planRouteToNearestFrontier();
    void runFloodFillToMode1Start();
    void clearFloodFrontierEvaluation();
    void evaluateFloodFrontierCandidates();
    void toggleTestSequence();
    void cancelTestSequence();
    void startCurrentTestSequenceStep();
    void advanceTestSequenceIfNeeded();
    void startCenterPivotSequence();
    void cancelCenterPivotSequence();
    void advanceCenterPivotSequenceIfNeeded();
    void loadAndStartTestPlan();
    void cancelPlanExecution();
    void executeLoadedRouteIfSafe();
    void cancelPlanCompositeAction();
    bool advancePlanCompositeActionIfNeeded();
    void startPlanCompositeCenterAndPivot180();
    bool startPlanAction(NavPlanAction action);
    void advancePlanExecutionIfNeeded();
    void toggleBasicNavAutonomy();
    void setBasicNavAutonomyEnabled(bool enabled);
    void toggleNavPolicy();
    void advanceBasicNavAutonomyIfNeeded();
    void startBasicNavRecommendedAction(NavRecommendedAction action,
                                        const RobotSensors &sensors);
    void cancelDeadEndRecovery();
    bool advanceDeadEndRecoveryIfNeeded();
    void setMode1MissionEnabled(bool enabled);
    void cancelMode1Mission(Mode1MissionDoneReason reason);
    bool mode1MissionAtStartCell(const NavMapDebugSnapshot &mapDebug) const;
    void syncNavSupervisorConfig();
    void updateNavSupervisor(bool smartNoFrontier = false);
    void syncMode1TelemetryFromSupervisor(const NavSupervisorDebugSnapshot &debug);
    void applyNavSupervisorOutput(const NavSupervisorOutput &output);
    bool advanceMode1MissionIfNeeded();
    void updateSmartRecognitionSupervisor(NavRecommendedAction recommendedAction,
                                          bool navReady,
                                          bool missionBlocked,
                                          NavRouteStatus frontierStatus,
                                          bool frontierPlanLoaded);
    void resetRobotPoseToWorldStart();
    void initializeNavMapFromWorldStart();
    void createRobotItem();
    void createIrSensorItems();
    void createFloorSensorItems();
    void updateRobotVisualOnly();
    void updateRobotGraphics();
    void updateIrSensors();
    void updateFloorSensors();
    void updateNavCorePipeline();
    void simulationStep();
    void togglePerformanceDebug();
    void resetPerformanceStats();
    void updatePerformanceSceneItemCounts();
    void recordPerformanceStep(double stepMs);
    void setSimulationRunning(bool running);
    void resetNavigationYawReference();
    void resetNavigationYawReferenceForSmoothStart();
    double navigationYawDeg() const;
    RobotSensors buildRobotSensorsSnapshot() const;
    void robotLocalToWorld(double local_x_mm,
                           double local_y_mm,
                           double *global_x_mm,
                           double *global_y_mm) const;
    void fitSceneToView();

    struct IrSensor {
        const char *name = "";
        double local_x_mm = 0.0;
        double local_y_mm = 0.0;
        double angle_offset_deg = 0.0;
        double max_distance_mm = 0.0;
        double last_distance_mm = 0.0;
        QGraphicsLineItem *ray_item = nullptr;
        QGraphicsEllipseItem *hit_item = nullptr;
        QLabel *value_label = nullptr;
    };

    struct FloorSensor {
        const char *name = "";
        double local_x_mm = 0.0;
        double local_y_mm = 0.0;
        bool is_black = false;
        TapeDebugKind debug_kind = TapeDebugKind::None;
        QGraphicsEllipseItem *marker_item = nullptr;
        QLabel *value_label = nullptr;
    };

    struct TelemetryPinnedRow {
        QTreeWidgetItem *item = nullptr;
        QLabel *source_label = nullptr;
    };

    QGraphicsScene *scene = nullptr;
    QGraphicsView *view = nullptr;
    QGraphicsPolygonItem *robotItem = nullptr;
    QLabel *xValueLabel = nullptr;
    QLabel *yValueLabel = nullptr;
    QLabel *yawValueLabel = nullptr;
    QLabel *mazeNameValueLabel = nullptr;
    QLabel *mazeFileValueLabel = nullptr;
    QLabel *mazeWidthValueLabel = nullptr;
    QLabel *mazeHeightValueLabel = nullptr;
    QLabel *mazeCellSizeValueLabel = nullptr;
    QLabel *navFloorFrontValueLabel = nullptr;
    QLabel *navFloorRearValueLabel = nullptr;
    QLabel *navYawValueLabel = nullptr;
    QLabel *navYawRateValueLabel = nullptr;
    QLabel *navYawZeroValueLabel = nullptr;
    QLabel *navStateValueLabel = nullptr;
    QLabel *navActionValueLabel = nullptr;
    QLabel *navActionStartYawValueLabel = nullptr;
    QLabel *navActionTargetYawValueLabel = nullptr;
    QLabel *navWallFrontValueLabel = nullptr;
    QLabel *navWallLeftValueLabel = nullptr;
    QLabel *navWallRightValueLabel = nullptr;
    QLabel *navWallDiagLeftValueLabel = nullptr;
    QLabel *navWallDiagRightValueLabel = nullptr;
    QLabel *navWallFrontThresholdValueLabel = nullptr;
    QLabel *navWallSideThresholdValueLabel = nullptr;
    QLabel *navWallDiagThresholdValueLabel = nullptr;
    QLabel *turnDebugSetpointValueLabel = nullptr;
    QLabel *turnDebugMeasuredValueLabel = nullptr;
    QLabel *turnDebugErrorValueLabel = nullptr;
    QLabel *turnDebugOutputValueLabel = nullptr;
    QLabel *turnDebugCorrectionValueLabel = nullptr;
    QLabel *turnDebugIntegralValueLabel = nullptr;
    QLabel *turnDebugKpValueLabel = nullptr;
    QLabel *turnDebugKiValueLabel = nullptr;
    QLabel *turnDebugKdValueLabel = nullptr;
    QLabel *turnDebugOutputLimitValueLabel = nullptr;
    QLabel *turnDebugSmoothTargetValueLabel = nullptr;
    QLabel *turnDebugSmoothRightLeftBaseValueLabel = nullptr;
    QLabel *turnDebugSmoothRightRightBaseValueLabel = nullptr;
    QLabel *turnDebugSmoothLeftLeftBaseValueLabel = nullptr;
    QLabel *turnDebugSmoothLeftRightBaseValueLabel = nullptr;
    QLabel *turnDebugSmoothLeftBaseValueLabel = nullptr;
    QLabel *turnDebugSmoothRightBaseValueLabel = nullptr;
    QLabel *turnDebugSmoothPhaseValueLabel = nullptr;
    QLabel *turnDebugSmoothDoneReasonValueLabel = nullptr;
    QLabel *turnDebugSmoothPostYawElapsedValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalGuidanceSourceValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalHoldInitializedValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalHoldSourceValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalHoldRecaptureCountValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalLeftHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalRightHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalCenterDiffHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalWallErrorValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalWallCorrectionValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalYawCorrectionValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalYawHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalYawHoldInitializedValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalYawHoldRecaptureCountValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalYawErrorValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalAppliedCorrectionValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagLeftValidValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagRightValidValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagLeftValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagRightValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagTargetValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagErrorScaleValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagModeValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagHoldInitializedValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagLeftHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagRightHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagCenterDiffHoldValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagHoldRecaptureCountValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagRawErrorValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalDiagErrorValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalFollowLeftValidValueLabel = nullptr;
    QLabel *turnDebugSmoothFinalFollowRightValidValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryEnabledValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryPendingValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryUsedValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryOffsetValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryEntryYawValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryExitYawValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryDiagUsedValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryCandidateSourceValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryRejectedReasonValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryOnlySetpointValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryRequireDiagValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryAllowAdvancePreviewValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryMinAbsValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryMaxAbsValueLabel = nullptr;
    QLabel *turnDebugSmoothYawCarryScaleValueLabel = nullptr;
    QLabel *turnDebugAdvancePhaseValueLabel = nullptr;
    QLabel *turnDebugAdvanceDoneReasonValueLabel = nullptr;
    QLabel *turnDebugRearBlackForLineValueLabel = nullptr;
    QLabel *turnDebugFloorRearRealValueLabel = nullptr;
    QLabel *turnDebugAdvanceStartedOnRearLineValueLabel = nullptr;
    QLabel *turnDebugAdvanceStartModeValueLabel = nullptr;
    QLabel *turnDebugAdvanceCenteredWaitingRearWhiteValueLabel = nullptr;
    QLabel *turnDebugSpecialCandidateValueLabel = nullptr;
    QLabel *turnDebugSpecialConfirmedValueLabel = nullptr;
    QLabel *turnDebugSpecialIgnoreRearValueLabel = nullptr;
    QLabel *turnDebugSpecialStartedOnRearLineValueLabel = nullptr;
    QLabel *turnDebugSpecialEnabledForMotionValueLabel = nullptr;
    QLabel *turnDebugSpecialDetectionContextValueLabel = nullptr;
    QLabel *turnDebugSpecialAuxEnabledValueLabel = nullptr;
    QLabel *turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel = nullptr;
    QLabel *turnDebugInitialSpecialSnapshotPendingValueLabel = nullptr;
    QLabel *turnDebugInitialSpecialSnapshotDoneValueLabel = nullptr;
    QLabel *turnDebugRearLineTrustedValueLabel = nullptr;
    QLabel *turnDebugRearLineTrustSourceValueLabel = nullptr;
    QLabel *turnDebugSpecialMarkTargetCellValueLabel = nullptr;
    QLabel *turnDebugSpecialMarkTargetSourceValueLabel = nullptr;
    QLabel *turnDebugLastSpecialMarkActionValueLabel = nullptr;
    QLabel *turnDebugAdvanceElapsedSinceLeaveValueLabel = nullptr;
    QLabel *turnDebugSpecialDetectMinValueLabel = nullptr;
    QLabel *turnDebugSpecialDetectMaxValueLabel = nullptr;
    QLabel *turnDebugApproachFrontPhaseValueLabel = nullptr;
    QLabel *turnDebugApproachFrontDoneReasonValueLabel = nullptr;
    QLabel *turnDebugApproachFrontTargetValueLabel = nullptr;
    QLabel *turnDebugApproachFrontLeftValueLabel = nullptr;
    QLabel *turnDebugApproachFrontRightValueLabel = nullptr;
    QLabel *turnDebugApproachFrontElapsedValueLabel = nullptr;
    QLabel *turnDebugApproachFrontBrakeElapsedValueLabel = nullptr;
    QLabel *turnDebugApproachFrontBaseLeftValueLabel = nullptr;
    QLabel *turnDebugApproachFrontBaseRightValueLabel = nullptr;
    QLabel *turnDebugApproachFrontCorrectionValueLabel = nullptr;
    QLabel *turnDebugCenterPivotPhaseValueLabel = nullptr;
    QLabel *turnDebugCenterPivotDoneReasonValueLabel = nullptr;
    QLabel *turnDebugCenterPivotElapsedValueLabel = nullptr;
    QLabel *turnDebugCenterPivotBrakeElapsedValueLabel = nullptr;
    QLabel *turnDebugCenterPivotBaseLeftValueLabel = nullptr;
    QLabel *turnDebugCenterPivotBaseRightValueLabel = nullptr;
    QLabel *turnDebugCenterPivotCorrectionValueLabel = nullptr;
    QLabel *turnDebugCenterPivotFrontBlackValueLabel = nullptr;
    QLabel *turnDebugCenterPivotRearBlackValueLabel = nullptr;
    QLabel *turnDebugCenterPivotFrontSeenWhiteValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawSetpointValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawMeasuredValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawOutputValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawCorrectionValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawHoldValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawHoldInitializedValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawHoldRecaptureCountValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawHoldErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceGuidanceLastSourceValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawKpValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawKiValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawKdValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawOutputLimitValueLabel = nullptr;
    QLabel *turnDebugAdvanceBaseLeftValueLabel = nullptr;
    QLabel *turnDebugAdvanceBaseRightValueLabel = nullptr;
    QLabel *turnDebugAdvanceGuidanceModeValueLabel = nullptr;
    QLabel *turnDebugAdvanceCorrectionSourceValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagPreviewArmedValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagPreviewLatchedValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagPreviewActiveValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagSourceValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagRawErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceFrontDiagRightValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallRightValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceDiagLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceDiagRightValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceFollowLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceFollowRightValidValueLabel = nullptr;
    QLabel *turnDebugWallCautionEnabledValueLabel = nullptr;
    QLabel *turnDebugWallLeftConfidenceValueLabel = nullptr;
    QLabel *turnDebugWallRightConfidenceValueLabel = nullptr;
    QLabel *turnDebugWallLeftCautionElapsedValueLabel = nullptr;
    QLabel *turnDebugWallRightCautionElapsedValueLabel = nullptr;
    QLabel *turnDebugWallLeftCautionHoldValueLabel = nullptr;
    QLabel *turnDebugWallRightCautionHoldValueLabel = nullptr;
    QLabel *turnDebugWallLeftCautionDeltaValueLabel = nullptr;
    QLabel *turnDebugWallRightCautionDeltaValueLabel = nullptr;
    QLabel *turnDebugWallCautionCorrectionValueLabel = nullptr;
    QLabel *turnDebugWallCautionLossReasonValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallLeftValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallRightValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallRawErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallErrorAfterDeadbandValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallPrevErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallErrorDeltaValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallPTermValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallDTermValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallRawCorrectionValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallLimitedCorrectionValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallCorrectionValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceKpValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceKdValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceLimitValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceScaleNumValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceScaleDenValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceTargetValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceSmoothFinalModeValueLabel = nullptr;
    QLabel *turnDebugDiagGuidancePTermValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceDTermValueLabel = nullptr;
    QLabel *turnDebugDiagGuidanceCorrectionValueLabel = nullptr;
    QLabel *turnDebugWallKpValueLabel = nullptr;
    QLabel *turnDebugWallKdValueLabel = nullptr;
    QLabel *turnDebugWallDeadbandValueLabel = nullptr;
    QLabel *turnDebugWallTargetLeftValueLabel = nullptr;
    QLabel *turnDebugWallTargetRightValueLabel = nullptr;
    QLabel *turnDebugWallCorrectionLimitValueLabel = nullptr;
    QLabel *turnDebugWallSingleSideErrorScaleValueLabel = nullptr;
    QLabel *turnDebugLastCompletedActionValueLabel = nullptr;
    QLabel *turnDebugLastSmoothDoneReasonValueLabel = nullptr;
    QLabel *turnDebugLastSmoothFinalYawValueLabel = nullptr;
    QLabel *turnDebugLastSmoothFinalRearValueLabel = nullptr;
    QLabel *turnDebugLastAdvanceDoneReasonValueLabel = nullptr;
    QLabel *turnDebugLastAdvanceFinalYawValueLabel = nullptr;
    QLabel *turnDebugLastAdvanceFinalRearValueLabel = nullptr;
    QLabel *turnDebugLastApproachFrontDoneReasonValueLabel = nullptr;
    QLabel *turnDebugLastCenterPivotDoneReasonValueLabel = nullptr;
    QLabel *navLeftMotorValueLabel = nullptr;
    QLabel *navRightMotorValueLabel = nullptr;
    QLabel *simulationRunningValueLabel = nullptr;
    QLabel *simulationDtValueLabel = nullptr;
    QLabel *autoModeValueLabel = nullptr;
    QLabel *simulationStepCountValueLabel = nullptr;
    QLabel *simulationTimeValueLabel = nullptr;
    QLabel *performanceDebugEnabledValueLabel = nullptr;
    QLabel *perfSceneItemCountValueLabel = nullptr;
    QLabel *perfSceneOverlayItemCountValueLabel = nullptr;
    QLabel *perfSceneTextItemCountValueLabel = nullptr;
    QLabel *perfSceneStaticOtherItemCountValueLabel = nullptr;
    QLabel *perfSimulationStepMsValueLabel = nullptr;
    QLabel *perfSimulationStepAvgMsValueLabel = nullptr;
    QLabel *perfSimulationStepMaxMsValueLabel = nullptr;
    QLabel *perfNavUpdateMsValueLabel = nullptr;
    QLabel *perfSensorUpdateMsValueLabel = nullptr;
    QLabel *perfVisualUpdateMsValueLabel = nullptr;
    QLabel *perfOverlayUpdateMsValueLabel = nullptr;
    QLabel *perfTelemetryUpdateMsValueLabel = nullptr;
    QLabel *perfFpsEstimateValueLabel = nullptr;
    QLabel *motorTestModeValueLabel = nullptr;
    QLabel *sequenceEnabledValueLabel = nullptr;
    QLabel *sequenceIndexValueLabel = nullptr;
    QLabel *sequenceLengthValueLabel = nullptr;
    QLabel *sequenceCurrentActionValueLabel = nullptr;
    QLabel *sequenceWaitingNextTickValueLabel = nullptr;
    QLabel *centerPivotSequenceActiveValueLabel = nullptr;
    QLabel *centerPivotSequencePhaseValueLabel = nullptr;
    QLabel *centerPivotSequenceLastCenterReasonValueLabel = nullptr;
    QLabel *planExecutionEnabledValueLabel = nullptr;
    QLabel *planQueueCountValueLabel = nullptr;
    QLabel *planCurrentActionValueLabel = nullptr;
    QLabel *planNextActionValueLabel = nullptr;
    QLabel *planLastExecutedActionValueLabel = nullptr;
    QLabel *planActionsExecutedCountValueLabel = nullptr;
    QLabel *planQueueOverflowValueLabel = nullptr;
    QLabel *planCompositeActionActiveValueLabel = nullptr;
    QLabel *planCompositeActionPhaseValueLabel = nullptr;
    QLabel *planCompositePrepareMethodValueLabel = nullptr;
    QLabel *planCompositeWallFrontAtStartValueLabel = nullptr;
    QLabel *planCompositeLastCenterReasonValueLabel = nullptr;
    QLabel *planCompositeLastApproachReasonValueLabel = nullptr;
    QLabel *planAdvanceAfterCenterPivotValueLabel = nullptr;
    QLabel *planNextAdvanceFromCenteredPoseValueLabel = nullptr;
    QLabel *routeStatusValueLabel = nullptr;
    QLabel *routeTargetCellValueLabel = nullptr;
    QLabel *routeStartCellValueLabel = nullptr;
    QLabel *routeStartDirValueLabel = nullptr;
    QLabel *routeLengthValueLabel = nullptr;
    QLabel *routeExpandedStatesValueLabel = nullptr;
    QLabel *routeFirstActionValueLabel = nullptr;
    QLabel *routeLastActionValueLabel = nullptr;
    QLabel *routeLoadedIntoPlanQueueValueLabel = nullptr;
    QLabel *frontierRouteStatusValueLabel = nullptr;
    QLabel *frontierTargetCellValueLabel = nullptr;
    QLabel *frontierTargetDirValueLabel = nullptr;
    QLabel *frontierExitDirAbsoluteValueLabel = nullptr;
    QLabel *frontierExitRelativeValueLabel = nullptr;
    QLabel *frontierNeighborCellValueLabel = nullptr;
    QLabel *frontierRouteLengthValueLabel = nullptr;
    QLabel *frontierExpandedStatesValueLabel = nullptr;
    QLabel *frontierLoadedIntoPlanQueueValueLabel = nullptr;
    QLabel *frontierCountFoundValueLabel = nullptr;
    QLabel *routeExecuteStatusValueLabel = nullptr;
    QLabel *routeStartPhysicalValidValueLabel = nullptr;
    QLabel *routeStartFloorRearBlackValueLabel = nullptr;
    QLabel *routePlanReadyToExecuteValueLabel = nullptr;
    QLabel *navAutonomyEnabledValueLabel = nullptr;
    QLabel *navPolicyValueLabel = nullptr;
    QLabel *navRecommendedActionValueLabel = nullptr;
    QLabel *navLastDecisionValueLabel = nullptr;
    QLabel *navDecisionWallFrontValueLabel = nullptr;
    QLabel *navDecisionWallLeftValueLabel = nullptr;
    QLabel *navDecisionWallRightValueLabel = nullptr;
    QLabel *navDecisionPointValidValueLabel = nullptr;
    QLabel *deadEndRecoveryActiveValueLabel = nullptr;
    QLabel *deadEndRecoveryPhaseValueLabel = nullptr;
    QLabel *deadEndRecoveryLastApproachReasonValueLabel = nullptr;
    QLabel *deadEndRecoveryPendingPivotValueLabel = nullptr;
    QLabel *navMapCandidateRightCellValueLabel = nullptr;
    QLabel *navMapCandidateFrontCellValueLabel = nullptr;
    QLabel *navMapCandidateLeftCellValueLabel = nullptr;
    QLabel *navMapCandidateRightVisitedValueLabel = nullptr;
    QLabel *navMapCandidateFrontVisitedValueLabel = nullptr;
    QLabel *navMapCandidateLeftVisitedValueLabel = nullptr;
    QLabel *navMapUsedUnvisitedPreferenceValueLabel = nullptr;
    QLabel *smartRecognitionStateValueLabel = nullptr;
    QLabel *smartLastFrontierStatusValueLabel = nullptr;
    QLabel *smartFrontierPlanRequestedCountValueLabel = nullptr;
    QLabel *smartFrontierRoutesExecutedCountValueLabel = nullptr;
    QLabel *smartNoFrontierCountValueLabel = nullptr;
    QLabel *smartLocalActionValueLabel = nullptr;
    QLabel *supervisorSmartStateValueLabel = nullptr;
    QLabel *supervisorSmartDecisionReasonValueLabel = nullptr;
    QLabel *supervisorRequestedActionValueLabel = nullptr;
    QLabel *supervisorRequestPlanToFrontierValueLabel = nullptr;
    QLabel *supervisorRequestExecutePlanValueLabel = nullptr;
    QLabel *supervisorSmartFrontierStatusValueLabel = nullptr;
    QLabel *supervisorSmartLocalActionValueLabel = nullptr;
    QLabel *supervisorSmartBlockedByMissionValueLabel = nullptr;
    QLabel *supervisorSmartNavReadyValueLabel = nullptr;
    QLabel *supervisorSmartPlanExecutionEnabledValueLabel = nullptr;
    QLabel *supervisorSmartActionInProgressValueLabel = nullptr;
    QLabel *supervisorSmartActiveAsSourceValueLabel = nullptr;
    QLabel *supervisorSmartLocalControlEnabledValueLabel = nullptr;
    QLabel *supervisorSmartLocalControlAppliedValueLabel = nullptr;
    QLabel *supervisorSmartLocalControlActionValueLabel = nullptr;
    QLabel *supervisorSmartLocalControlMapOkValueLabel = nullptr;
    QLabel *supervisorSmartLocalControlFallbackLegacyValueLabel = nullptr;
    QLabel *supervisorFrontierPlanNotifiedValueLabel = nullptr;
    QLabel *supervisorFrontierPlanLoadedValueLabel = nullptr;
    QLabel *supervisorFrontierPlanRequestPulseCountValueLabel = nullptr;
    QLabel *mainwindowSmartStateValueLabel = nullptr;
    QLabel *mainwindowSmartLocalActionValueLabel = nullptr;
    QLabel *mode1MissionEnabledValueLabel = nullptr;
    QLabel *mode1MissionStateValueLabel = nullptr;
    QLabel *mode1RequiredSpecialCountValueLabel = nullptr;
    QLabel *mode1FoundSpecialCountValueLabel = nullptr;
    QLabel *mode1RequiredSpecialsReachedValueLabel = nullptr;
    QLabel *mode1ReturnRequestedValueLabel = nullptr;
    QLabel *mode1WaitingActionDoneValueLabel = nullptr;
    QLabel *mode1SearchCompleteLatchedAtCountValueLabel = nullptr;
    QLabel *mode1PlanCancelledAfterRequiredFoundValueLabel = nullptr;
    QLabel *mode1NavReadyForReturnValueLabel = nullptr;
    QLabel *mode1PlanWasActiveWhenRequiredFoundValueLabel = nullptr;
    QLabel *mode1PlanQueueCountWhenRequiredFoundValueLabel = nullptr;
    QLabel *mode1StartCellValueLabel = nullptr;
    QLabel *mode1ReturnRouteStatusValueLabel = nullptr;
    QLabel *mode1ReturnPlanLoadedValueLabel = nullptr;
    QLabel *mode1ReturnToStartActiveValueLabel = nullptr;
    QLabel *mode1AtStartCellValueLabel = nullptr;
    QLabel *mode1DoneReasonValueLabel = nullptr;
    QLabel *supervisorStateValueLabel = nullptr;
    QLabel *supervisorDoneReasonValueLabel = nullptr;
    QLabel *supervisorRequiredSpecialsReachedValueLabel = nullptr;
    QLabel *supervisorReturnRequestedValueLabel = nullptr;
    QLabel *supervisorWaitingActionDoneValueLabel = nullptr;
    QLabel *supervisorRequestClearPlanValueLabel = nullptr;
    QLabel *supervisorRequestPlanReturnValueLabel = nullptr;
    QLabel *supervisorRequestExecuteReturnValueLabel = nullptr;
    QLabel *supervisorBlockSmartActionsValueLabel = nullptr;
    QLabel *supervisorActiveAsSourceValueLabel = nullptr;
    QLabel *floodStatusValueLabel = nullptr;
    QLabel *floodValidValueLabel = nullptr;
    QLabel *floodGoalCellValueLabel = nullptr;
    QLabel *floodCurrentCellCostValueLabel = nullptr;
    QLabel *floodReachedCountValueLabel = nullptr;
    QLabel *floodExpandedCountValueLabel = nullptr;
    QLabel *floodFrontierEvalValidValueLabel = nullptr;
    QLabel *floodFrontierCandidateCountValueLabel = nullptr;
    QLabel *floodFrontierCandidateEdgeCountValueLabel = nullptr;
    QLabel *floodFrontierCandidateCellCountValueLabel = nullptr;
    QLabel *floodFrontierCandidateNeighborCellCountValueLabel = nullptr;
    QLabel *floodFrontierBestFoundValueLabel = nullptr;
    QLabel *floodFrontierBestCellValueLabel = nullptr;
    QLabel *floodFrontierBestNeighborCellValueLabel = nullptr;
    QLabel *floodFrontierBestExitDirValueLabel = nullptr;
    QLabel *floodFrontierBestCostToStartValueLabel = nullptr;
    QLabel *floodFrontierBestNeighborManhattanValueLabel = nullptr;
    QLabel *floodFrontierBestScoreValueLabel = nullptr;
    QLabel *floodFrontierSafeReturnCostValueLabel = nullptr;
    QLabel *floodFrontierBestScoreDebugValueLabel = nullptr;
    QLabel *floodFrontierScoreImprovementValueLabel = nullptr;
    QLabel *floodFrontierScoreMarginValueLabel = nullptr;
    QLabel *floodFrontierDecisionValueLabel = nullptr;
    QLabel *floodFrontierDecisionReasonValueLabel = nullptr;
    QLabel *floodFrontierEntryRequiredDirValueLabel = nullptr;
    QLabel *floodFrontierEntryRelativeFromCurrentDirValueLabel = nullptr;
    QLabel *floodFrontierEntryActionFromCurrentDirValueLabel = nullptr;
    QLabel *floodFrontierEntrySupportedFromCurrentDirValueLabel = nullptr;
    QLabel *floodFrontierEntryNorthValueLabel = nullptr;
    QLabel *floodFrontierEntryEastValueLabel = nullptr;
    QLabel *floodFrontierEntrySouthValueLabel = nullptr;
    QLabel *floodFrontierEntryWestValueLabel = nullptr;
    QLabel *floodFrontierEntryPreferredArrivalDirValueLabel = nullptr;
    QLabel *floodFrontierEntryPreferredActionValueLabel = nullptr;
    QLabel *floodFrontierEntryPreferredSupportedValueLabel = nullptr;
    QLabel *mapEnabledValueLabel = nullptr;
    QLabel *mapWidthValueLabel = nullptr;
    QLabel *mapHeightValueLabel = nullptr;
    QLabel *mapCellXValueLabel = nullptr;
    QLabel *mapCellYValueLabel = nullptr;
    QLabel *mapDirValueLabel = nullptr;
    QLabel *mapCurrentCellVisitedValueLabel = nullptr;
    QLabel *mapCurrentCellSpecialValueLabel = nullptr;
    QLabel *mapCurrentCellWallsKnownValueLabel = nullptr;
    QLabel *mapCurrentCellWallsPresentValueLabel = nullptr;
    QLabel *mapLastPoseUpdateActionValueLabel = nullptr;
    QLabel *mapLastWallUpdateActionValueLabel = nullptr;
    QLabel *mapInitialWallSnapshotPendingValueLabel = nullptr;
    QLabel *mapUpdateCountValueLabel = nullptr;
    QLabel *mapWallUpdateCountValueLabel = nullptr;
    QLabel *mapSpecialCellsFoundCountValueLabel = nullptr;
    QLabel *mapLastSpecialCellValueLabel = nullptr;
    QLabel *mapOverlayEnabledValueLabel = nullptr;
    QLabel *overlayItemsCreatedLastUpdateValueLabel = nullptr;
    QLabel *overlayItemsDeletedLastUpdateValueLabel = nullptr;
    QLabel *overlayItemsCurrentValueLabel = nullptr;
    QLabel *overlayRebuildCountValueLabel = nullptr;
    QLabel *overlayDirtyValueLabel = nullptr;
    QLabel *simLeftMotorGainValueLabel = nullptr;
    QLabel *simRightMotorGainValueLabel = nullptr;
    QLabel *simPivotCenterCorrectionEnabledValueLabel = nullptr;
    QLabel *simPivotCenterLocalXValueLabel = nullptr;
    QLabel *simPivotCenterLocalYValueLabel = nullptr;
    QLabel *simLastMotionPivotLikeValueLabel = nullptr;
    QLabel *simFloorRearGlobalXValueLabel = nullptr;
    QLabel *simFloorRearGlobalYValueLabel = nullptr;
    QLabel *motorTestLeftValueLabel = nullptr;
    QLabel *motorTestRightValueLabel = nullptr;
    QTimer *simulationTimer = nullptr;
    bool simulationRunning = false;
    bool autoModeEnabled = false;
    bool motorTestModeEnabled = false;
    bool performanceDebugEnabled = false;
    int perfSceneItemCount = 0;
    int perfSceneOverlayItemCount = 0;
    int perfSceneTextItemCount = 0;
    int perfSceneStaticOtherItemCount = 0;
    double perfSimulationStepMs = 0.0;
    double perfSimulationStepAvgMs = 0.0;
    double perfSimulationStepMaxMs = 0.0;
    double perfNavUpdateMs = 0.0;
    double perfSensorUpdateMs = 0.0;
    double perfVisualUpdateMs = 0.0;
    double perfOverlayUpdateMs = 0.0;
    double perfTelemetryUpdateMs = 0.0;
    double perfFpsEstimate = 0.0;
    double perfWindowStepAccumMs = 0.0;
    double perfWindowStepMaxMs = 0.0;
    int perfWindowStepCount = 0;
    qint64 perfWindowStartElapsedMs = 0;
    QElapsedTimer perfElapsedTimer;
    bool testSequenceEnabled = false;
    bool testSequenceWaitingNextTick = false;
    int testSequenceIndex = 0;
    CenterPivotSequencePhase centerPivotSequencePhase = CenterPivotSequencePhase::None;
    NavCenterPivotDoneReason centerPivotSequenceLastCenterReason = NAV_CENTER_PIVOT_DONE_NONE;
    bool planExecutionEnabled = false;
    NavPlanAction planCurrentAction = NAV_PLAN_ACTION_NONE;
    NavPlanAction planLastExecutedAction = NAV_PLAN_ACTION_NONE;
    uint16_t planActionsExecutedCount = 0;
    CenterPivotSequencePhase planCompositeActionPhase = CenterPivotSequencePhase::None;
    PlanCompositePrepareMethod planCompositePrepareMethod = PlanCompositePrepareMethod::None;
    bool planCompositeWallFrontAtStart = false;
    NavCenterPivotDoneReason planCompositeLastCenterReason = NAV_CENTER_PIVOT_DONE_NONE;
    NavApproachFrontDoneReason planCompositeLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
    bool planAdvanceStartedAfterCenterPivotDiagnostic = false;
    bool planNextAdvanceFromCenteredPose = false;
    RouteExecuteStatus routeExecuteStatus = RouteExecuteStatus::Idle;
    bool routeStartPhysicalValid = false;
    bool routeStartFloorRearBlack = false;
    bool routePlanReadyToExecute = false;
    bool basicNavAutonomyEnabled = false;
    NavRecommendedAction basicNavRecommendedAction = NAV_RECOMMENDED_NONE;
    NavRecommendedAction basicNavLastDecision = NAV_RECOMMENDED_NONE;
    QString basicNavLastDecisionText = "NONE";
    bool basicNavDecisionWallFront = false;
    bool basicNavDecisionWallLeft = false;
    bool basicNavDecisionWallRight = false;
    bool basicNavDecisionPointValid = false;
    // Vista/cache de telemetria SMART derivada de nav_supervisor; MainWindow solo adapta.
    SmartRecognitionState smartRecognitionState = SmartRecognitionState::Idle;
    NavRouteStatus smartLastFrontierStatus = NAV_ROUTE_STATUS_IDLE;
    uint16_t smartFrontierPlanRequestedCount = 0;
    uint16_t smartFrontierRoutesExecutedCount = 0;
    uint16_t smartNoFrontierCount = 0;
    NavRecommendedAction smartLocalAction = NAV_RECOMMENDED_NONE;
    NavSupervisorSmartOutput supervisorSmartLastOutput = {};
    bool supervisorSmartActiveAsSource = false;
    bool supervisorSmartLastNavReady = false;
    bool supervisorSmartLastPlanExecutionEnabled = false;
    bool supervisorSmartActionInProgress = false;
    bool supervisorSmartLocalControlEnabled = true;
    bool supervisorSmartLocalControlApplied = false;
    NavSupervisorRequestedAction supervisorSmartLocalControlAction =
        NAV_SUPERVISOR_REQUESTED_ACTION_NONE;
    bool supervisorSmartLocalControlMapOk = false;
    bool supervisorSmartLocalControlFallbackLegacy = false;
    bool mode1MissionEnabled = true;
    // Vista/cache de telemetria derivada de nav_supervisor; no es fuente de verdad.
    Mode1MissionState mode1MissionState = Mode1MissionState::Disabled;
    Mode1MissionDoneReason mode1MissionDoneReason = Mode1MissionDoneReason::None;
    uint16_t mode1RequiredSpecialCount = 3;
    uint16_t mode1FoundSpecialCount = 0;
    bool mode1PlanCancelledAfterRequiredFound = false;
    bool mode1PlanWasActiveWhenRequiredFound = false;
    uint16_t mode1SearchCompleteLatchedAtCount = 0;
    uint8_t mode1PlanQueueCountWhenRequiredFound = 0;
    int8_t mode1StartCellX = 0;
    int8_t mode1StartCellY = 0;
    NavMapDirection mode1StartDir = NAV_DIR_EAST;
    bool mode1StartCellValid = false;
    NavRouteStatus mode1ReturnRouteStatus = NAV_ROUTE_STATUS_IDLE;
    bool mode1ReturnPlanLoaded = false;
    NavSupervisorOutput supervisorLastOutput = {};
    bool supervisorActiveAsSource = false;
    bool floodFrontierEvalValid = false;
    uint16_t floodFrontierCandidateCount = 0;
    uint16_t floodFrontierCandidateEdgeCount = 0;
    uint16_t floodFrontierCandidateCellCount = 0;
    uint16_t floodFrontierCandidateNeighborCellCount = 0;
    bool floodFrontierBestFound = false;
    int8_t floodFrontierBestCellX = -1;
    int8_t floodFrontierBestCellY = -1;
    int8_t floodFrontierBestNeighborCellX = -1;
    int8_t floodFrontierBestNeighborCellY = -1;
    NavMapDirection floodFrontierBestExitDir = NAV_DIR_NORTH;
    uint16_t floodFrontierBestCostToStart = NAV_FLOOD_COST_INF;
    uint16_t floodFrontierBestNeighborManhattan = 0;
    uint16_t floodFrontierBestScore = NAV_FLOOD_COST_INF;
    uint16_t floodFrontierSafeReturnCost = NAV_FLOOD_COST_INF;
    int32_t floodFrontierScoreImprovement = 0;
    uint16_t floodFrontierScoreMargin = 0;
    FloodFrontierDecision floodFrontierDecision = FloodFrontierDecision::None;
    FloodFrontierDecisionReason floodFrontierDecisionReason =
        FloodFrontierDecisionReason::None;
    NavMapDirection floodFrontierEntryRequiredDir = NAV_DIR_NORTH;
    FloodFrontierEntryRelative floodFrontierEntryRelativeFromCurrentDir =
        FloodFrontierEntryRelative::None;
    FloodFrontierEntryAction floodFrontierEntryActionFromCurrentDir =
        FloodFrontierEntryAction::None;
    bool floodFrontierEntrySupportedFromCurrentDir = false;
    FloodFrontierEntryRelative floodFrontierEntryRelativeByArrivalDir[4] = {
        FloodFrontierEntryRelative::None,
        FloodFrontierEntryRelative::None,
        FloodFrontierEntryRelative::None,
        FloodFrontierEntryRelative::None
    };
    FloodFrontierEntryAction floodFrontierEntryActionByArrivalDir[4] = {
        FloodFrontierEntryAction::None,
        FloodFrontierEntryAction::None,
        FloodFrontierEntryAction::None,
        FloodFrontierEntryAction::None
    };
    NavMapDirection floodFrontierEntryPreferredArrivalDir = NAV_DIR_NORTH;
    FloodFrontierEntryAction floodFrontierEntryPreferredAction =
        FloodFrontierEntryAction::None;
    bool floodFrontierEntryPreferredSupported = false;
    DeadEndRecoveryPhase deadEndRecoveryPhase = DeadEndRecoveryPhase::None;
    NavApproachFrontDoneReason deadEndRecoveryLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
    bool deadEndRecoveryPendingPivot = false;
    bool shadowMapOverlayEnabled = true;
    bool shadowMapOverlayDirty = true;
    uint32_t shadowMapOverlayLastMapUpdateCount = UINT32_MAX;
    uint32_t shadowMapOverlayLastWallUpdateCount = UINT32_MAX;
    uint16_t shadowMapOverlayLastSpecialCount = UINT16_MAX;
    bool shadowMapOverlayLastFloodValid = false;
    uint32_t floodOverlayRevision = 0;
    uint32_t shadowMapOverlayLastFloodRevision = UINT32_MAX;
    uint32_t frontierOverlayRevision = 0;
    uint32_t shadowMapOverlayLastFrontierRevision = UINT32_MAX;
    uint32_t overlayRebuildCount = 0;
    int overlayItemsCreatedLastUpdate = 0;
    int overlayItemsDeletedLastUpdate = 0;
    int overlayItemsCurrent = 0;
    uint64_t simulationStepCount = 0;
    double simulationTimeS = 0.0;
    double navYawZeroDeg = 0.0;
    RobotCommand lastNavCommand = {0, 0};
    RobotCommand motorTestCommand = {0, 0};
    SimWorld world;
    SimRobot robot;
    std::vector<IrSensor> irSensors;
    std::vector<FloorSensor> floorSensors;
    std::vector<TelemetryPinnedRow> telemetryPinnedRows;
    std::vector<QGraphicsItem *> shadowMapOverlayItems;
};

#endif // MAINWINDOW_H
