#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>
#include <QString>

#include <cstdint>
#include <vector>

#include "nav_core.h"
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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    enum class DeadEndRecoveryPhase {
        None,
        ApproachFront,
        Pivot180
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
    void clearShadowMapOverlay();
    void updateShadowMapOverlay();
    void drawShadowMapOverlay();
    void rebuildSceneItems();
    void initializeIrSensors();
    void initializeFloorSensors();
    void createTelemetryPanel();
    void updateTelemetryPanel();
    void showControlsHelp();
    void showControlTuningDialog();
    void loadMazeFromDialog();
    bool loadMazeFile(const QString &path);
    void adjustSmoothTargetYawRate(int delta_deg_s);
    void promptSmoothTargetYawRate();
    void toggleTestSequence();
    void cancelTestSequence();
    void startCurrentTestSequenceStep();
    void advanceTestSequenceIfNeeded();
    void toggleBasicNavAutonomy();
    void setBasicNavAutonomyEnabled(bool enabled);
    void toggleNavPolicy();
    void advanceBasicNavAutonomyIfNeeded();
    void startBasicNavRecommendedAction(NavRecommendedAction action,
                                        const RobotSensors &sensors);
    void cancelDeadEndRecovery();
    bool advanceDeadEndRecoveryIfNeeded();
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
    void setSimulationRunning(bool running);
    void resetNavigationYawReference();
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
    QLabel *turnDebugAdvancePhaseValueLabel = nullptr;
    QLabel *turnDebugAdvanceDoneReasonValueLabel = nullptr;
    QLabel *turnDebugSpecialCandidateValueLabel = nullptr;
    QLabel *turnDebugSpecialConfirmedValueLabel = nullptr;
    QLabel *turnDebugSpecialIgnoreRearValueLabel = nullptr;
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
    QLabel *turnDebugAdvanceYawSetpointValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawMeasuredValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawErrorValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawOutputValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawCorrectionValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawKpValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawKiValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawKdValueLabel = nullptr;
    QLabel *turnDebugAdvanceYawOutputLimitValueLabel = nullptr;
    QLabel *turnDebugAdvanceBaseLeftValueLabel = nullptr;
    QLabel *turnDebugAdvanceBaseRightValueLabel = nullptr;
    QLabel *turnDebugAdvanceGuidanceModeValueLabel = nullptr;
    QLabel *turnDebugAdvanceCorrectionSourceValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceWallRightValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceDiagLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceDiagRightValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceFollowLeftValidValueLabel = nullptr;
    QLabel *turnDebugAdvanceFollowRightValidValueLabel = nullptr;
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
    QLabel *navLeftMotorValueLabel = nullptr;
    QLabel *navRightMotorValueLabel = nullptr;
    QLabel *simulationRunningValueLabel = nullptr;
    QLabel *simulationDtValueLabel = nullptr;
    QLabel *autoModeValueLabel = nullptr;
    QLabel *simulationStepCountValueLabel = nullptr;
    QLabel *simulationTimeValueLabel = nullptr;
    QLabel *motorTestModeValueLabel = nullptr;
    QLabel *sequenceEnabledValueLabel = nullptr;
    QLabel *sequenceIndexValueLabel = nullptr;
    QLabel *sequenceLengthValueLabel = nullptr;
    QLabel *sequenceCurrentActionValueLabel = nullptr;
    QLabel *sequenceWaitingNextTickValueLabel = nullptr;
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
    bool testSequenceEnabled = false;
    bool testSequenceWaitingNextTick = false;
    int testSequenceIndex = 0;
    bool basicNavAutonomyEnabled = false;
    NavRecommendedAction basicNavRecommendedAction = NAV_RECOMMENDED_NONE;
    NavRecommendedAction basicNavLastDecision = NAV_RECOMMENDED_NONE;
    QString basicNavLastDecisionText = "NONE";
    bool basicNavDecisionWallFront = false;
    bool basicNavDecisionWallLeft = false;
    bool basicNavDecisionWallRight = false;
    bool basicNavDecisionPointValid = false;
    DeadEndRecoveryPhase deadEndRecoveryPhase = DeadEndRecoveryPhase::None;
    NavApproachFrontDoneReason deadEndRecoveryLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
    bool deadEndRecoveryPendingPivot = false;
    bool shadowMapOverlayEnabled = true;
    uint64_t simulationStepCount = 0;
    double simulationTimeS = 0.0;
    double navYawZeroDeg = 0.0;
    RobotCommand lastNavCommand = {0, 0};
    RobotCommand motorTestCommand = {0, 0};
    SimWorld world;
    SimRobot robot;
    std::vector<IrSensor> irSensors;
    std::vector<FloorSensor> floorSensors;
    std::vector<QGraphicsItem *> shadowMapOverlayItems;
};

#endif // MAINWINDOW_H
