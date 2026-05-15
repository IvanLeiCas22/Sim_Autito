#include "mainwindow.h"

#include "nav_core.h"

#include <QAction>
#include <QBrush>
#include <QChar>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFont>
#include <QFontDatabase>
#include <QFileDialog>
#include <QFileInfo>
#include <QGraphicsEllipseItem>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QGroupBox>
#include <QHeaderView>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QInputDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStringList>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kManualMoveStepMm = 10.0;
constexpr double kManualRotateStepDeg = 10.0;
constexpr double kIrMaxDistanceMm = 150.0;
constexpr double kIrHitRadiusMm = 5.0;
constexpr double kFloorSensorRadiusMm = 7.0;
constexpr int kSimulationIntervalMs = 10;
constexpr double kSimulationDtS = kSimulationIntervalMs / 1000.0;
constexpr int kWallCorrectionLimitPwmMax = 4000;

enum class TestSequenceStep {
    AdvanceUntilRearBlack,
    SmoothTurnRight,
    SmoothTurnLeft
};

constexpr int kTestSequenceLength = 5;
constexpr TestSequenceStep kTestSequence[kTestSequenceLength] = {
    TestSequenceStep::AdvanceUntilRearBlack,
    TestSequenceStep::SmoothTurnRight,
    TestSequenceStep::AdvanceUntilRearBlack,
    TestSequenceStep::SmoothTurnLeft,
    TestSequenceStep::AdvanceUntilRearBlack
};

double normalizeAngleSignedDeg(double angleDeg)
{
    double normalized = std::fmod(angleDeg + 180.0, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }

    return normalized - 180.0;
}

QString floorSensorText(bool isBlack, TapeDebugKind kind)
{
    if (!isBlack) {
        return "white";
    }

    switch (kind) {
    case TapeDebugKind::Boundary:
        return "BLACK (boundary)";
    case TapeDebugKind::Target:
        return "BLACK (target)";
    case TapeDebugKind::BoundaryAndTarget:
        return "BLACK (boundary+target)";
    case TapeDebugKind::None:
        break;
    }

    return "BLACK";
}

q16_16_t toQ16(double value)
{
    const double scaled = std::round(value * 65536.0);
    if (scaled > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return std::numeric_limits<int32_t>::max();
    }
    if (scaled < static_cast<double>(std::numeric_limits<int32_t>::min())) {
        return std::numeric_limits<int32_t>::min();
    }

    return static_cast<q16_16_t>(scaled);
}

double fromQ16(q16_16_t value)
{
    return static_cast<double>(value) / 65536.0;
}

QString navStateText(NavState state)
{
    switch (state) {
    case NAV_STATE_IDLE:
        return "IDLE";
    case NAV_STATE_ADVANCING_UNTIL_REAR_BLACK:
        return "ADVANCING";
    case NAV_STATE_APPROACHING_FRONT_WALL_FOR_PIVOT:
        return "APPROACH_FRONT";
    case NAV_STATE_CENTERING_IN_CELL_FOR_PIVOT:
        return "CENTER_IN_CELL_FOR_PIVOT";
    case NAV_STATE_SMOOTH_TURNING:
        return "SMOOTH_TURNING";
    case NAV_STATE_PIVOT_TURNING:
        return "PIVOT_TURNING";
    case NAV_STATE_DONE:
        return "DONE";
    }

    return "UNKNOWN";
}

QString navActionText(NavAction action)
{
    switch (action) {
    case NAV_ACTION_NONE:
        return "NONE";
    case NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK:
        return "ADVANCE_LINE";
    case NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        return "APPROACH_FRONT_WALL_FOR_PIVOT";
    case NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE:
        return "CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE";
    case NAV_ACTION_SMOOTH_TURN_LEFT:
        return "SMOOTH_LEFT";
    case NAV_ACTION_SMOOTH_TURN_RIGHT:
        return "SMOOTH_RIGHT";
    case NAV_ACTION_PIVOT_TURN_LEFT:
        return "PIVOT_LEFT";
    case NAV_ACTION_PIVOT_TURN_RIGHT:
        return "PIVOT_RIGHT";
    case NAV_ACTION_PIVOT_TURN_180:
        return "PIVOT_180";
    }

    return "UNKNOWN";
}

QString smoothPhaseText(NavSmoothPhase phase)
{
    switch (phase) {
    case NAV_SMOOTH_PHASE_NONE:
        return "NONE";
    case NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE:
        return "WAIT_LEAVE_START_LINE";
    case NAV_SMOOTH_PHASE_SEEK_TARGET_LINE:
        return "SEEK_TARGET_LINE";
    case NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE:
        return "POST_YAW_SEEK_REAR_LINE";
    case NAV_SMOOTH_PHASE_DONE:
        return "DONE";
    }

    return "UNKNOWN";
}

QString specialMarkTargetSourceText(NavSpecialMarkTargetSource source)
{
    switch (source) {
    case NAV_SPECIAL_MARK_TARGET_INVALID:
        return "INVALID";
    case NAV_SPECIAL_MARK_TARGET_CURRENT_CELL:
        return "CURRENT_CELL";
    case NAV_SPECIAL_MARK_TARGET_SMOOTH_DESTINATION:
        return "SMOOTH_DESTINATION";
    case NAV_SPECIAL_MARK_TARGET_AUX_CURRENT_CELL:
        return "AUX_CURRENT_CELL";
    }

    return "UNKNOWN";
}

QString specialDetectionContextText(NavSpecialDetectionContext context)
{
    switch (context) {
    case NAV_SPECIAL_DETECT_DISABLED:
        return "DISABLED";
    case NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL:
        return "TRANSLATION_TO_NEXT_CELL";
    case NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION:
        return "IN_CELL_AUX_TRANSLATION";
    }

    return "UNKNOWN";
}

QString smoothDoneReasonText(NavSmoothDoneReason reason)
{
    switch (reason) {
    case NAV_SMOOTH_DONE_NONE:
        return "NONE";
    case NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE:
        return "REAR_SENSOR_TARGET_LINE";
    case NAV_SMOOTH_DONE_YAW_FALLBACK:
        return "YAW_FALLBACK";
    case NAV_SMOOTH_DONE_TIMEOUT:
        return "TIMEOUT";
    }

    return "UNKNOWN";
}

QString advancePhaseText(NavAdvancePhase phase)
{
    switch (phase) {
    case NAV_ADVANCE_PHASE_NONE:
        return "NONE";
    case NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE:
        return "WAIT_LEAVE_START_LINE";
    case NAV_ADVANCE_PHASE_SEEK_TARGET_LINE:
        return "SEEK_TARGET_LINE";
    }

    return "UNKNOWN";
}

QString advanceDoneReasonText(NavAdvanceDoneReason reason)
{
    switch (reason) {
    case NAV_ADVANCE_DONE_NONE:
        return "NONE";
    case NAV_ADVANCE_DONE_REAR_SENSOR_TARGET_LINE:
        return "REAR_SENSOR_TARGET_LINE";
    }

    return "UNKNOWN";
}

QString advanceStartModeText(NavAdvanceStartMode mode)
{
    switch (mode) {
    case NAV_ADVANCE_START_REAR_LINE:
        return "REAR_LINE";
    case NAV_ADVANCE_START_CENTERED_POSE:
        return "CENTERED_POSE";
    }

    return "UNKNOWN";
}

QString rearLineTrustSourceText(NavRearLineTrustSource source)
{
    switch (source) {
    case NAV_REAR_LINE_TRUST_NONE:
        return "NONE";
    case NAV_REAR_LINE_TRUST_INITIAL_REAR_LINE:
        return "INITIAL_REAR_LINE";
    case NAV_REAR_LINE_TRUST_ADVANCE_DONE:
        return "ADVANCE_DONE";
    case NAV_REAR_LINE_TRUST_SMOOTH_DONE:
        return "SMOOTH_DONE";
    case NAV_REAR_LINE_TRUST_CENTERED_ADVANCE_DONE:
        return "CENTERED_ADVANCE_DONE";
    case NAV_REAR_LINE_TRUST_OTHER:
        return "OTHER";
    }

    return "UNKNOWN";
}

QString approachFrontPhaseText(NavApproachFrontPhase phase)
{
    switch (phase) {
    case NAV_APPROACH_FRONT_PHASE_NONE:
        return "NONE";
    case NAV_APPROACH_FRONT_PHASE_DRIVE:
        return "DRIVE";
    case NAV_APPROACH_FRONT_PHASE_BRAKE_SETTLE:
        return "BRAKE_SETTLE";
    case NAV_APPROACH_FRONT_PHASE_DONE:
        return "DONE";
    }

    return "UNKNOWN";
}

QString approachFrontDoneReasonText(NavApproachFrontDoneReason reason)
{
    switch (reason) {
    case NAV_APPROACH_FRONT_DONE_NONE:
        return "NONE";
    case NAV_APPROACH_FRONT_DONE_TARGET_DISTANCE:
        return "TARGET_DISTANCE";
    case NAV_APPROACH_FRONT_DONE_TIMEOUT:
        return "TIMEOUT";
    }

    return "UNKNOWN";
}

QString centerPivotPhaseText(NavCenterPivotPhase phase)
{
    switch (phase) {
    case NAV_CENTER_PIVOT_PHASE_NONE:
        return "NONE";
    case NAV_CENTER_PIVOT_PHASE_INIT:
        return "INIT";
    case NAV_CENTER_PIVOT_PHASE_WAIT_LEAVE_START_LINE:
        return "WAIT_LEAVE_START_LINE";
    case NAV_CENTER_PIVOT_PHASE_WAIT_FRONT_WHITE:
        return "WAIT_FRONT_WHITE";
    case NAV_CENTER_PIVOT_PHASE_SEEK_FRONT_LINE:
        return "SEEK_FRONT_LINE";
    case NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE:
        return "BRAKE_SETTLE";
    case NAV_CENTER_PIVOT_PHASE_DONE:
        return "DONE";
    }

    return "UNKNOWN";
}

QString centerPivotDoneReasonText(NavCenterPivotDoneReason reason)
{
    switch (reason) {
    case NAV_CENTER_PIVOT_DONE_NONE:
        return "NONE";
    case NAV_CENTER_PIVOT_DONE_FRONT_LINE:
        return "FRONT_LINE";
    case NAV_CENTER_PIVOT_DONE_TIMEOUT:
        return "TIMEOUT";
    case NAV_CENTER_PIVOT_DONE_START_NOT_ON_REAR_LINE:
        return "START_NOT_ON_REAR_LINE";
    }

    return "UNKNOWN";
}

QString advanceGuidanceModeText(NavAdvanceGuidanceMode mode)
{
    switch (mode) {
    case NAV_ADVANCE_GUIDANCE_YAW_ONLY:
        return "YAW_ONLY";
    case NAV_ADVANCE_GUIDANCE_WALL_ASSIST:
        return "WALL_ASSIST";
    }

    return "UNKNOWN";
}

QString advanceCorrectionSourceText(NavAdvanceCorrectionSource source)
{
    switch (source) {
    case NAV_ADVANCE_CORRECTION_YAW_PD:
        return "YAW_PD";
    case NAV_ADVANCE_CORRECTION_WALL_LEFT:
        return "WALL_LEFT";
    case NAV_ADVANCE_CORRECTION_WALL_RIGHT:
        return "WALL_RIGHT";
    case NAV_ADVANCE_CORRECTION_WALL_CENTER:
        return "WALL_CENTER";
    case NAV_ADVANCE_CORRECTION_WALL_LEFT_CAUTION:
        return "WALL_LEFT_CAUTION";
    case NAV_ADVANCE_CORRECTION_WALL_RIGHT_CAUTION:
        return "WALL_RIGHT_CAUTION";
    case NAV_ADVANCE_CORRECTION_DIAG_LEFT:
        return "DIAG_LEFT";
    case NAV_ADVANCE_CORRECTION_DIAG_RIGHT:
        return "DIAG_RIGHT";
    case NAV_ADVANCE_CORRECTION_DIAG_CENTER:
        return "DIAG_CENTER";
    }

    return "UNKNOWN";
}

QString wallCautionConfidenceText(NavWallCautionConfidence confidence)
{
    switch (confidence) {
    case NAV_WALL_CAUTION_CONFIDENCE_LOST:
        return "LOST";
    case NAV_WALL_CAUTION_CONFIDENCE_CONFIRMED:
        return "CONFIRMED";
    case NAV_WALL_CAUTION_CONFIDENCE_CAUTION:
        return "CAUTION";
    }
    return "UNKNOWN";
}

QString wallCautionLossReasonText(NavWallCautionLossReason reason)
{
    switch (reason) {
    case NAV_WALL_CAUTION_LOSS_NONE:
        return "NONE";
    case NAV_WALL_CAUTION_LOSS_DISABLED:
        return "DISABLED";
    case NAV_WALL_CAUTION_LOSS_LATERAL_LOST:
        return "LATERAL_LOST";
    case NAV_WALL_CAUTION_LOSS_DELTA_MAX:
        return "DELTA_MAX";
    case NAV_WALL_CAUTION_LOSS_TIMEOUT:
        return "TIMEOUT";
    case NAV_WALL_CAUTION_LOSS_ACTION_END:
        return "ACTION_END";
    }
    return "UNKNOWN";
}

QString advanceFrontDiagSourceText(NavAdvanceFrontDiagSource source)
{
    switch (source) {
    case NAV_ADVANCE_FRONT_DIAG_NONE:
        return "NONE";
    case NAV_ADVANCE_FRONT_DIAG_LEFT:
        return "DIAG_LEFT";
    case NAV_ADVANCE_FRONT_DIAG_RIGHT:
        return "DIAG_RIGHT";
    case NAV_ADVANCE_FRONT_DIAG_CENTER:
        return "DIAG_CENTER";
    }

    return "UNKNOWN";
}

QString mapDirectionText(NavMapDirection dir)
{
    switch (dir) {
    case NAV_DIR_NORTH:
        return "NORTH";
    case NAV_DIR_EAST:
        return "EAST";
    case NAV_DIR_SOUTH:
        return "SOUTH";
    case NAV_DIR_WEST:
        return "WEST";
    }

    return "UNKNOWN";
}

QString mapActionText(NavMapAction action)
{
    switch (action) {
    case NAV_MAP_ACTION_NONE:
        return "NONE";
    case NAV_MAP_ACTION_INITIAL_SNAPSHOT:
        return "INITIAL_SNAPSHOT";
    case NAV_MAP_ACTION_ADVANCE_LINE:
        return "ADVANCE_LINE";
    case NAV_MAP_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        return "APPROACH_FRONT_WALL_FOR_PIVOT";
    case NAV_MAP_ACTION_SMOOTH_TURN_LEFT:
        return "SMOOTH_LEFT";
    case NAV_MAP_ACTION_SMOOTH_TURN_RIGHT:
        return "SMOOTH_RIGHT";
    case NAV_MAP_ACTION_PIVOT_TURN_LEFT:
        return "PIVOT_LEFT";
    case NAV_MAP_ACTION_PIVOT_TURN_RIGHT:
        return "PIVOT_RIGHT";
    case NAV_MAP_ACTION_PIVOT_TURN_180:
        return "PIVOT_180";
    }

    return "UNKNOWN";
}

QString smoothFinalGuidanceSourceText(NavSmoothFinalGuidanceSource source)
{
    switch (source) {
    case NAV_SMOOTH_FINAL_GUIDANCE_NONE:
        return "NONE";
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER:
        return "DIAG_CENTER";
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT:
        return "DIAG_LEFT";
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT:
        return "DIAG_RIGHT";
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER_HOLD:
        return "DIAG_CENTER_HOLD";
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT_HOLD:
        return "DIAG_LEFT_HOLD";
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT_HOLD:
        return "DIAG_RIGHT_HOLD";
    case NAV_SMOOTH_FINAL_GUIDANCE_WALL_CENTER_HOLD:
        return "WALL_CENTER_HOLD";
    case NAV_SMOOTH_FINAL_GUIDANCE_WALL_LEFT_HOLD:
        return "WALL_LEFT_HOLD";
    case NAV_SMOOTH_FINAL_GUIDANCE_WALL_RIGHT_HOLD:
        return "WALL_RIGHT_HOLD";
    case NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY:
        return "YAW_ONLY";
    case NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK:
        return "YAW_ONLY_FALLBACK";
    }

    return "UNKNOWN";
}

QString smoothFinalDiagonalModeText(NavSmoothFinalDiagonalMode mode)
{
    switch (mode) {
    case NAV_SMOOTH_FINAL_DIAG_MODE_HOLD_RELATIVE:
        return "HOLD_RELATIVE";
    case NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT:
        return "SETPOINT";
    }
    return "UNKNOWN";
}

QString smoothYawCarryRejectedReasonText(NavSmoothYawCarryRejectedReason reason)
{
    switch (reason) {
    case NAV_SMOOTH_YAW_CARRY_REJECT_NONE:
        return "NONE";
    case NAV_SMOOTH_YAW_CARRY_REJECT_DISABLED:
        return "DISABLED";
    case NAV_SMOOTH_YAW_CARRY_REJECT_NOT_NEXT_SMOOTH:
        return "NOT_NEXT_SMOOTH";
    case NAV_SMOOTH_YAW_CARRY_REJECT_NOT_SETPOINT:
        return "NOT_SETPOINT";
    case NAV_SMOOTH_YAW_CARRY_REJECT_NO_DIAG_USED:
        return "NO_DIAG_USED";
    case NAV_SMOOTH_YAW_CARRY_REJECT_OFFSET_TOO_SMALL:
        return "OFFSET_TOO_SMALL";
    case NAV_SMOOTH_YAW_CARRY_REJECT_OFFSET_TOO_LARGE:
        return "OFFSET_TOO_LARGE";
    }
    return "UNKNOWN";
}

QString yawCarryCandidateSourceText(NavYawCarryCandidateSource source)
{
    switch (source) {
    case NAV_YAW_CARRY_SOURCE_NONE:
        return "NONE";
    case NAV_YAW_CARRY_SOURCE_SMOOTH_FINAL_DIAG:
        return "SMOOTH_FINAL_DIAG";
    case NAV_YAW_CARRY_SOURCE_ADVANCE_FRONT_DIAG_PREVIEW:
        return "ADVANCE_FRONT_DIAG_PREVIEW";
    }
    return "UNKNOWN";
}

QString wallMaskText(uint8_t mask)
{
    QStringList parts;
    if ((mask & NAV_MAP_WALL_NORTH) != 0) {
        parts << "N";
    }
    if ((mask & NAV_MAP_WALL_EAST) != 0) {
        parts << "E";
    }
    if ((mask & NAV_MAP_WALL_SOUTH) != 0) {
        parts << "S";
    }
    if ((mask & NAV_MAP_WALL_WEST) != 0) {
        parts << "W";
    }

    return parts.isEmpty() ? "-" : parts.join("");
}

NavMapDirection directionFromYawDeg(double yawDeg)
{
    double normalized = std::fmod(yawDeg, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }

    const int quadrant = static_cast<int>(std::floor((normalized + 45.0) / 90.0)) & 3;
    switch (quadrant) {
    case 0:
        return NAV_DIR_EAST;
    case 1:
        return NAV_DIR_SOUTH;
    case 2:
        return NAV_DIR_WEST;
    case 3:
    default:
        return NAV_DIR_NORTH;
    }
}

void configureTelemetryValueLabel(QLabel *label)
{
    if (!label) {
        return;
    }

    QFont font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    font.setPointSize(8);
    label->setFont(font);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setMinimumWidth(220);
    label->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
}

QString telemetrySectionText(QLabel *label)
{
    QString text = label ? label->text() : QString();
    text.remove("<b>");
    text.remove("</b>");
    return text.trimmed();
}

QString compactTelemetryName(QString name)
{
    if (name.endsWith(':')) {
        name.chop(1);
    }

    static const std::pair<const char *, const char *> replacements[] = {
        {"special_detection_", "special_"},
        {"special_mark_target_", "mark_"},
        {"advance_wall_", "wall_"},
        {"advance_yaw_", "yaw_"},
        {"advance_", "adv_"},
        {"approach_front_", "approach_"},
        {"center_pivot_", "center_"},
        {"smooth_", "sm_"},
        {"frontier_", "fr_"},
        {"plan_composite_", "comp_"},
        {"nav_map_candidate_", "cand_"},
        {"smart_recognition_", "smart_"}
    };

    for (const auto &replacement : replacements) {
        name.replace(replacement.first, replacement.second);
    }

    return name;
}

class TelemetryTreeBuilder {
public:
    explicit TelemetryTreeBuilder(QTreeWidget *tree)
        : tree_(tree)
    {
    }

    void addRow(QLabel *sectionLabel)
    {
        currentSection_ = new QTreeWidgetItem(tree_);
        currentSection_->setText(0, telemetrySectionText(sectionLabel));
        QFont font = currentSection_->font(0);
        font.setBold(true);
        currentSection_->setFont(0, font);
        currentSection_->setFirstColumnSpanned(true);
        currentSection_->setExpanded(true);
    }

    void addRow(const QString &name, QLabel *valueLabel)
    {
        auto *item = new QTreeWidgetItem(currentSection_ ? currentSection_ : tree_->invisibleRootItem());
        const QString fullName = name.endsWith(':') ? name.left(name.size() - 1) : name;
        item->setText(0, compactTelemetryName(name));
        item->setToolTip(0, fullName);
        if (valueLabel) {
            valueLabel->setMinimumWidth(0);
            valueLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
            tree_->setItemWidget(item, 1, valueLabel);
        }
    }

private:
    QTreeWidget *tree_ = nullptr;
    QTreeWidgetItem *currentSection_ = nullptr;
};

QString sequenceStepText(TestSequenceStep step)
{
    switch (step) {
    case TestSequenceStep::AdvanceUntilRearBlack:
        return "ADVANCE_UNTIL_REAR_BLACK";
    case TestSequenceStep::SmoothTurnRight:
        return "SMOOTH_TURN_RIGHT";
    case TestSequenceStep::SmoothTurnLeft:
        return "SMOOTH_TURN_LEFT";
    }

    return "UNKNOWN";
}

QString planActionText(NavPlanAction action)
{
    switch (action) {
    case NAV_PLAN_ACTION_NONE:
        return "NONE";
    case NAV_PLAN_ACTION_ADVANCE_LINE:
        return "ADVANCE_LINE";
    case NAV_PLAN_ACTION_SMOOTH_LEFT:
        return "SMOOTH_LEFT";
    case NAV_PLAN_ACTION_SMOOTH_RIGHT:
        return "SMOOTH_RIGHT";
    case NAV_PLAN_ACTION_PIVOT_180:
        return "PIVOT_180";
    case NAV_PLAN_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        return "APPROACH_FRONT_WALL_FOR_PIVOT";
    case NAV_PLAN_ACTION_CENTER_AND_PIVOT_180:
        return "CENTER_AND_PIVOT_180";
    }

    return "UNKNOWN";
}

QString routeStatusText(NavRouteStatus status)
{
    switch (status) {
    case NAV_ROUTE_STATUS_IDLE:
        return "IDLE";
    case NAV_ROUTE_STATUS_FOUND:
        return "FOUND";
    case NAV_ROUTE_STATUS_FRONTIER_ALREADY_HERE:
        return "FRONTIER_ALREADY_HERE";
    case NAV_ROUTE_STATUS_NO_PATH:
        return "NO_PATH";
    case NAV_ROUTE_STATUS_NO_FRONTIER:
        return "NO_FRONTIER";
    case NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS:
        return "TARGET_OUT_OF_BOUNDS";
    case NAV_ROUTE_STATUS_TARGET_NOT_VISITED:
        return "TARGET_NOT_VISITED";
    case NAV_ROUTE_STATUS_ROUTE_TOO_LONG:
        return "ROUTE_TOO_LONG";
    case NAV_ROUTE_STATUS_QUEUE_OVERFLOW:
        return "QUEUE_OVERFLOW";
    }

    return "UNKNOWN";
}

QString frontierExitRelativeText(NavFrontierExitRelative relative)
{
    switch (relative) {
    case NAV_FRONTIER_EXIT_NONE:
        return "NONE";
    case NAV_FRONTIER_EXIT_FRONT:
        return "FRONT";
    case NAV_FRONTIER_EXIT_RIGHT:
        return "RIGHT";
    case NAV_FRONTIER_EXIT_LEFT:
        return "LEFT";
    }

    return "UNKNOWN";
}

QString routeExecuteStatusText(MainWindow::RouteExecuteStatus status)
{
    switch (status) {
    case MainWindow::RouteExecuteStatus::Idle:
        return "IDLE";
    case MainWindow::RouteExecuteStatus::NoRouteLoaded:
        return "NO_ROUTE_LOADED";
    case MainWindow::RouteExecuteStatus::NavBusy:
        return "NAV_BUSY";
    case MainWindow::RouteExecuteStatus::StartNotOnRearLine:
        return "START_NOT_ON_REAR_LINE";
    case MainWindow::RouteExecuteStatus::Started:
        return "STARTED";
    case MainWindow::RouteExecuteStatus::Running:
        return "RUNNING";
    case MainWindow::RouteExecuteStatus::Completed:
        return "COMPLETED";
    case MainWindow::RouteExecuteStatus::Cancelled:
        return "CANCELLED";
    }

    return "UNKNOWN";
}

QString recommendedActionText(NavRecommendedAction action)
{
    switch (action) {
    case NAV_RECOMMENDED_NONE:
        return "NONE";
    case NAV_RECOMMENDED_ACQUIRE_REAR_LINE:
        return "ACQUIRE_REAR_LINE";
    case NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED:
        return "RECOVERY_PIVOT_180_FRONT_BLOCKED";
    case NAV_RECOMMENDED_ADVANCE_LINE:
        return "ADVANCE_LINE";
    case NAV_RECOMMENDED_SMOOTH_LEFT:
        return "SMOOTH_LEFT";
    case NAV_RECOMMENDED_SMOOTH_RIGHT:
        return "SMOOTH_RIGHT";
    case NAV_RECOMMENDED_PIVOT_180:
        return "PIVOT_180";
    }

    return "UNKNOWN";
}

QString navPolicyText(NavPolicy policy)
{
    switch (policy) {
    case NAV_POLICY_RIGHT_HAND_RULE:
        return "RIGHT_HAND_RULE";
    case NAV_POLICY_MAP_PREFER_UNVISITED:
        return "MAP_PREFER_UNVISITED";
    case NAV_POLICY_SMART_RECOGNITION:
        return "SMART_RECOGNITION";
    }

    return "UNKNOWN";
}

QString smartRecognitionStateText(MainWindow::SmartRecognitionState state)
{
    switch (state) {
    case MainWindow::SmartRecognitionState::Idle:
        return "IDLE";
    case MainWindow::SmartRecognitionState::LocalUnvisited:
        return "LOCAL_UNVISITED";
    case MainWindow::SmartRecognitionState::PlanToFrontier:
        return "PLAN_TO_FRONTIER";
    case MainWindow::SmartRecognitionState::ExecutingFrontierRoute:
        return "EXECUTING_FRONTIER_ROUTE";
    case MainWindow::SmartRecognitionState::FrontierAlreadyHere:
        return "FRONTIER_ALREADY_HERE";
    case MainWindow::SmartRecognitionState::NoFrontier:
        return "NO_FRONTIER";
    case MainWindow::SmartRecognitionState::Error:
        return "ERROR";
    }

    return "UNKNOWN";
}

QString mode1MissionStateText(MainWindow::Mode1MissionState state)
{
    switch (state) {
    case MainWindow::Mode1MissionState::Disabled:
        return "DISABLED";
    case MainWindow::Mode1MissionState::SearchSpecials:
        return "SEARCH_SPECIALS";
    case MainWindow::Mode1MissionState::FoundRequiredSpecialsWaitActionDone:
        return "FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE";
    case MainWindow::Mode1MissionState::ReturnToStartPlan:
        return "RETURN_TO_START_PLAN";
    case MainWindow::Mode1MissionState::ReturnToStartExecute:
        return "RETURN_TO_START_EXECUTE";
    case MainWindow::Mode1MissionState::Done:
        return "DONE";
    case MainWindow::Mode1MissionState::Error:
        return "ERROR";
    }

    return "UNKNOWN";
}

QString mode1MissionDoneReasonText(MainWindow::Mode1MissionDoneReason reason)
{
    switch (reason) {
    case MainWindow::Mode1MissionDoneReason::None:
        return "NONE";
    case MainWindow::Mode1MissionDoneReason::FoundRequiredSpecialsAndReturned:
        return "FOUND_REQUIRED_SPECIALS_AND_RETURNED";
    case MainWindow::Mode1MissionDoneReason::NoReturnRoute:
        return "NO_RETURN_ROUTE";
    case MainWindow::Mode1MissionDoneReason::ReturnRouteTooLong:
        return "RETURN_ROUTE_TOO_LONG";
    case MainWindow::Mode1MissionDoneReason::ReturnQueueOverflow:
        return "RETURN_QUEUE_OVERFLOW";
    case MainWindow::Mode1MissionDoneReason::StartCellInvalid:
        return "START_CELL_INVALID";
    case MainWindow::Mode1MissionDoneReason::NoFrontierBeforeRequiredSpecials:
        return "NO_FRONTIER_BEFORE_REQUIRED_SPECIALS";
    case MainWindow::Mode1MissionDoneReason::Cancelled:
        return "CANCELLED";
    }

    return "UNKNOWN";
}

QString mapCandidateCellText(int8_t cellX, int8_t cellY, bool valid)
{
    if (!valid) {
        return "invalid";
    }

    return QString("(%1,%2)").arg(cellX).arg(cellY);
}

QString deadEndRecoveryPhaseText(MainWindow::DeadEndRecoveryPhase phase)
{
    switch (phase) {
    case MainWindow::DeadEndRecoveryPhase::None:
        return "NONE";
    case MainWindow::DeadEndRecoveryPhase::ApproachFront:
        return "APPROACH_FRONT";
    case MainWindow::DeadEndRecoveryPhase::Pivot180:
        return "PIVOT_180";
    }

    return "UNKNOWN";
}

QString centerPivotSequencePhaseText(MainWindow::CenterPivotSequencePhase phase)
{
    switch (phase) {
    case MainWindow::CenterPivotSequencePhase::None:
        return "NONE";
    case MainWindow::CenterPivotSequencePhase::Centering:
        return "CENTERING";
    case MainWindow::CenterPivotSequencePhase::ApproachFront:
        return "APPROACH_FRONT";
    case MainWindow::CenterPivotSequencePhase::Pivot180:
        return "PIVOT_180";
    case MainWindow::CenterPivotSequencePhase::Done:
        return "DONE";
    case MainWindow::CenterPivotSequencePhase::Failed:
        return "FAILED";
    }

    return "UNKNOWN";
}

QString planCompositePrepareMethodText(MainWindow::PlanCompositePrepareMethod method)
{
    switch (method) {
    case MainWindow::PlanCompositePrepareMethod::None:
        return "NONE";
    case MainWindow::PlanCompositePrepareMethod::FrontLine:
        return "FRONT_LINE";
    case MainWindow::PlanCompositePrepareMethod::FrontWall:
        return "FRONT_WALL";
    }

    return "UNKNOWN";
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Sim Autito");
    resize(900, 700);
    setFocusPolicy(Qt::StrongFocus);

    nav_core_init();
    simulationTimer = new QTimer(this);
    simulationTimer->setInterval(kSimulationIntervalMs);
    connect(simulationTimer, &QTimer::timeout, this, &MainWindow::simulationStep);

    initializeIrSensors();
    initializeFloorSensors();
    resetRobotPoseToWorldStart();
    setupScene();
    createTelemetryPanel();

    QMenu *fileMenu = menuBar()->addMenu("File");
    QAction *loadMazeAction = fileMenu->addAction("Load Maze...");
    loadMazeAction->setShortcut(QKeySequence(Qt::Key_O));
    connect(loadMazeAction, &QAction::triggered, this, &MainWindow::loadMazeFromDialog);

    QMenu *tuningMenu = menuBar()->addMenu("Tuning");
    QAction *controlTuningAction = tuningMenu->addAction("Control Tuning...");
    controlTuningAction->setShortcut(QKeySequence(Qt::Key_F3));
    connect(controlTuningAction, &QAction::triggered, this, &MainWindow::showControlTuningDialog);

    QMenu *helpMenu = menuBar()->addMenu("Help");
    QAction *controlsAction = helpMenu->addAction("Controls");
    connect(controlsAction, &QAction::triggered, this, &MainWindow::showControlsHelp);

    QTimer::singleShot(0, this, [this]() { fitSceneToView(); });
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    bool handled = true;
    bool robotPoseChanged = true;

    switch (event->key()) {
    case Qt::Key_W:
    case Qt::Key_Up:
        robot.moveForward(kManualMoveStepMm);
        break;
    case Qt::Key_S:
    case Qt::Key_Down:
        robot.moveForward(-kManualMoveStepMm);
        break;
    case Qt::Key_A:
    case Qt::Key_Left:
        robot.rotate(-kManualRotateStepDeg);
        break;
    case Qt::Key_D:
    case Qt::Key_Right:
        robot.rotate(kManualRotateStepDeg);
        break;
    case Qt::Key_R:
        resetRobotPoseToWorldStart();
        break;
    case Qt::Key_Space:
        setSimulationRunning(!simulationRunning);
        robotPoseChanged = false;
        break;
    case Qt::Key_N:
        if (!simulationRunning) {
            simulationStep();
        }
        robotPoseChanged = false;
        break;
    case Qt::Key_M:
        autoModeEnabled = !autoModeEnabled;
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_T:
        planRouteToNearestFrontier();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_I:
        motorTestCommand = {3000, 3000};
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_K:
        promptRoutePlanToCell();
        robotPoseChanged = false;
        break;
    case Qt::Key_J:
        executeLoadedRouteIfSafe();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_L:
        startCenterPivotSequence();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_U:
        loadAndStartTestPlan();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
        adjustSmoothTargetYawRate(5);
        robotPoseChanged = false;
        break;
    case Qt::Key_Minus:
        adjustSmoothTargetYawRate(-5);
        robotPoseChanged = false;
        break;
    case Qt::Key_F2:
        promptSmoothTargetYawRate();
        robotPoseChanged = false;
        break;
    case Qt::Key_F3:
        showControlTuningDialog();
        robotPoseChanged = false;
        break;
    case Qt::Key_C: {
        const NavAdvanceGuidanceMode mode = nav_core_get_advance_guidance_mode();
        nav_core_set_advance_guidance_mode(mode == NAV_ADVANCE_GUIDANCE_WALL_ASSIST
                                               ? NAV_ADVANCE_GUIDANCE_YAW_ONLY
                                               : NAV_ADVANCE_GUIDANCE_WALL_ASSIST);
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_Y:
        shadowMapOverlayEnabled = !shadowMapOverlayEnabled;
        updateShadowMapOverlay();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_O:
        loadMazeFromDialog();
        robotPoseChanged = false;
        break;
    case Qt::Key_V:
        toggleTestSequence();
        robotPoseChanged = false;
        break;
    case Qt::Key_B:
        toggleBasicNavAutonomy();
        robotPoseChanged = false;
        break;
    case Qt::Key_P:
        toggleNavPolicy();
        robotPoseChanged = false;
        break;
    case Qt::Key_G:
        resetNavigationYawReference();
        nav_core_start_advance_until_rear_black();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_F:
        resetNavigationYawReference();
        nav_core_start_approach_front_wall_for_pivot();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_H:
        resetNavigationYawReference();
        nav_core_start_center_in_cell_for_pivot_by_front_line();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_X:
        cancelTestSequence();
        cancelCenterPivotSequence();
        cancelPlanExecution();
        cancelDeadEndRecovery();
        cancelMode1Mission(Mode1MissionDoneReason::Cancelled);
        setBasicNavAutonomyEnabled(false);
        nav_core_stop();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_Q: {
        resetNavigationYawReferenceForSmoothStart();
        RobotSensors sensors = buildRobotSensorsSnapshot();
        nav_core_start_smooth_turn_left(&sensors);
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_E: {
        resetNavigationYawReferenceForSmoothStart();
        RobotSensors sensors = buildRobotSensorsSnapshot();
        nav_core_start_smooth_turn_right(&sensors);
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_1: {
        resetNavigationYawReference();
        RobotSensors sensors = buildRobotSensorsSnapshot();
        nav_core_start_pivot_turn_left(&sensors);
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_2: {
        resetNavigationYawReference();
        RobotSensors sensors = buildRobotSensorsSnapshot();
        nav_core_start_pivot_turn_right(&sensors);
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_3: {
        resetNavigationYawReference();
        RobotSensors sensors = buildRobotSensorsSnapshot();
        nav_core_start_pivot_turn_180(&sensors);
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_Z:
        resetNavigationYawReference();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_F1:
        showControlsHelp();
        robotPoseChanged = false;
        break;
    default:
        handled = false;
        robotPoseChanged = false;
        break;
    }

    if (handled) {
        if (robotPoseChanged) {
            updateRobotGraphics();
        }
        event->accept();
        return;
    }

    QMainWindow::keyPressEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    fitSceneToView();
}

void MainWindow::setupScene()
{
    scene = new QGraphicsScene(this);
    scene->setBackgroundBrush(QBrush(Qt::white));

    view = new QGraphicsView(scene, this);
    view->setRenderHint(QPainter::Antialiasing, true);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    view->setFocusPolicy(Qt::NoFocus);
    setCentralWidget(view);

    drawReferenceGrid();
    drawBlackTape();
    drawWorldWalls();
    updateShadowMapOverlay();
    createRobotItem();
    createIrSensorItems();
    createFloorSensorItems();
    updateRobotGraphics();
}

void MainWindow::rebuildSceneItems()
{
    if (!scene) {
        return;
    }

    scene->clear();
    shadowMapOverlayItems.clear();
    robotItem = nullptr;
    for (IrSensor &sensor : irSensors) {
        sensor.ray_item = nullptr;
        sensor.hit_item = nullptr;
    }
    for (FloorSensor &sensor : floorSensors) {
        sensor.marker_item = nullptr;
    }

    drawReferenceGrid();
    drawBlackTape();
    drawWorldWalls();
    updateShadowMapOverlay();
    createRobotItem();
    createIrSensorItems();
    createFloorSensorItems();
    updateRobotGraphics();
    fitSceneToView();
}

void MainWindow::drawReferenceGrid()
{
    const double cellSizeMm = world.cellSizeMm();
    const double mazeWidthMm = world.widthMm();
    const double mazeHeightMm = world.heightMm();
    scene->setSceneRect(0, 0, mazeWidthMm, mazeHeightMm);

    const QPen gridPen(QColor(160, 160, 160), 1);

    for (int col = 0; col <= world.cols(); ++col) {
        const double x = col * cellSizeMm;
        scene->addLine(x, 0, x, mazeHeightMm, gridPen);
    }

    for (int row = 0; row <= world.rows(); ++row) {
        const double y = row * cellSizeMm;
        scene->addLine(0, y, mazeWidthMm, y, gridPen);
    }
}

void MainWindow::drawBlackTape()
{
    const QPen tapePen(Qt::NoPen);
    const QBrush boundaryTapeBrush(QColor(95, 95, 95, 150));
    const QBrush targetTapeBrush(QColor(35, 35, 35, 210));

    for (const SimRect &rect : world.boundaryTapeRects()) {
        QGraphicsRectItem *item = scene->addRect(rect.x_mm,
                                                rect.y_mm,
                                                rect.w_mm,
                                                rect.h_mm,
                                                tapePen,
                                                boundaryTapeBrush);
        item->setZValue(1.0);
    }

    for (const SimRect &rect : world.targetTapeRects()) {
        QGraphicsRectItem *item = scene->addRect(rect.x_mm,
                                                rect.y_mm,
                                                rect.w_mm,
                                                rect.h_mm,
                                                tapePen,
                                                targetTapeBrush);
        item->setZValue(1.0);
    }
}

void MainWindow::drawWorldWalls()
{
    const double cellSizeMm = world.cellSizeMm();
    QPen wallPen(QColor(0, 0, 0), 12);
    wallPen.setCapStyle(Qt::SquareCap);

    for (int row = 0; row < world.rows(); ++row) {
        for (int col = 0; col < world.cols(); ++col) {
            const double x0 = col * cellSizeMm;
            const double y0 = row * cellSizeMm;
            const double x1 = x0 + cellSizeMm;
            const double y1 = y0 + cellSizeMm;

            if (world.hasWall(row, col, WallDir::North)) {
                QGraphicsLineItem *wallItem = scene->addLine(x0, y0, x1, y0, wallPen);
                wallItem->setZValue(4.0);
            }
            if (world.hasWall(row, col, WallDir::West)) {
                QGraphicsLineItem *wallItem = scene->addLine(x0, y0, x0, y1, wallPen);
                wallItem->setZValue(4.0);
            }
            if (col == world.cols() - 1 && world.hasWall(row, col, WallDir::East)) {
                QGraphicsLineItem *wallItem = scene->addLine(x1, y0, x1, y1, wallPen);
                wallItem->setZValue(4.0);
            }
            if (row == world.rows() - 1 && world.hasWall(row, col, WallDir::South)) {
                QGraphicsLineItem *wallItem = scene->addLine(x0, y1, x1, y1, wallPen);
                wallItem->setZValue(4.0);
            }
        }
    }
}

void MainWindow::clearShadowMapOverlay()
{
    if (!scene) {
        shadowMapOverlayItems.clear();
        return;
    }

    for (QGraphicsItem *item : shadowMapOverlayItems) {
        if (item && item->scene() == scene) {
            scene->removeItem(item);
            delete item;
        }
    }
    shadowMapOverlayItems.clear();
}

void MainWindow::updateShadowMapOverlay()
{
    clearShadowMapOverlay();
    if (shadowMapOverlayEnabled) {
        drawShadowMapOverlay();
    }
}

void MainWindow::drawShadowMapOverlay()
{
    if (!scene) {
        return;
    }

    NavMapDebugSnapshot mapDebug = {};
    nav_core_get_map_debug(&mapDebug);
    if (!mapDebug.enabled) {
        return;
    }

    const double cellSizeMm = world.cellSizeMm();
    if (cellSizeMm <= 0.0) {
        return;
    }

    const int cols = std::min<int>(world.cols(), mapDebug.width);
    const int rows = std::min<int>(world.rows(), mapDebug.height);
    const QPen noPen(Qt::NoPen);
    const QBrush visitedBrush(QColor(40, 145, 255, 34));
    QPen currentCellPen(QColor(0, 150, 220, 210), 3.0);
    currentCellPen.setJoinStyle(Qt::MiterJoin);
    QPen presentWallPen(QColor(220, 40, 40, 220), 5.0);
    presentWallPen.setCapStyle(Qt::SquareCap);
    QPen absentWallPen(QColor(40, 160, 90, 95), 1.0, Qt::DashLine);
    absentWallPen.setCapStyle(Qt::SquareCap);
    QPen specialCellPen(QColor(255, 210, 0, 230), 5.0);
    specialCellPen.setJoinStyle(Qt::MiterJoin);
    QPen arrowPen(QColor(0, 70, 180, 230), 4.0);
    arrowPen.setCapStyle(Qt::RoundCap);
    const QBrush arrowBrush(QColor(0, 70, 180, 230));

    const auto remember = [this](QGraphicsItem *item, double zValue) {
        if (!item) {
            return;
        }
        item->setZValue(zValue);
        shadowMapOverlayItems.push_back(item);
    };

    const auto drawKnownWall = [&](double x0, double y0, double x1, double y1, bool present) {
        remember(scene->addLine(x0, y0, x1, y1, present ? presentWallPen : absentWallPen),
                 present ? 3.4 : 2.8);
    };

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            NavMapCell cell = {};
            if (!nav_core_get_map_cell(static_cast<int8_t>(x),
                                       static_cast<int8_t>(y),
                                       &cell)) {
                continue;
            }

            if (!cell.visited && cell.walls_known == 0) {
                continue;
            }

            const double x0 = x * cellSizeMm;
            const double y0 = y * cellSizeMm;
            const double x1 = x0 + cellSizeMm;
            const double y1 = y0 + cellSizeMm;

            if (cell.visited) {
                const double inset = std::max(3.0, cellSizeMm * 0.04);
                remember(scene->addRect(x0 + inset,
                                        y0 + inset,
                                        cellSizeMm - inset * 2.0,
                                        cellSizeMm - inset * 2.0,
                                        noPen,
                                        visitedBrush),
                         2.0);
            }

            if (cell.special_detected) {
                const double inset = std::max(6.0, cellSizeMm * 0.08);
                remember(scene->addRect(x0 + inset,
                                        y0 + inset,
                                        cellSizeMm - inset * 2.0,
                                        cellSizeMm - inset * 2.0,
                                        specialCellPen,
                                        Qt::NoBrush),
                         3.0);

                QGraphicsTextItem *label = scene->addText("S");
                QFont font = label->font();
                font.setBold(true);
                font.setPointSizeF(std::max(10.0, cellSizeMm * 0.13));
                label->setFont(font);
                label->setDefaultTextColor(QColor(255, 210, 0, 240));
                const QRectF labelBounds = label->boundingRect();
                label->setPos(x0 + cellSizeMm * 0.5 - labelBounds.width() * 0.5,
                              y0 + cellSizeMm * 0.5 - labelBounds.height() * 0.5);
                remember(label, 3.05);
            }

            if ((cell.walls_known & NAV_MAP_WALL_NORTH) != 0) {
                drawKnownWall(x0, y0, x1, y0, (cell.walls_present & NAV_MAP_WALL_NORTH) != 0);
            }
            if ((cell.walls_known & NAV_MAP_WALL_EAST) != 0) {
                drawKnownWall(x1, y0, x1, y1, (cell.walls_present & NAV_MAP_WALL_EAST) != 0);
            }
            if ((cell.walls_known & NAV_MAP_WALL_SOUTH) != 0) {
                drawKnownWall(x0, y1, x1, y1, (cell.walls_present & NAV_MAP_WALL_SOUTH) != 0);
            }
            if ((cell.walls_known & NAV_MAP_WALL_WEST) != 0) {
                drawKnownWall(x0, y0, x0, y1, (cell.walls_present & NAV_MAP_WALL_WEST) != 0);
            }
        }
    }

    if (mapDebug.cell_x < 0 || mapDebug.cell_y < 0
        || mapDebug.cell_x >= cols || mapDebug.cell_y >= rows) {
        return;
    }

    const double currentX = mapDebug.cell_x * cellSizeMm;
    const double currentY = mapDebug.cell_y * cellSizeMm;
    remember(scene->addRect(currentX + 2.0,
                            currentY + 2.0,
                            cellSizeMm - 4.0,
                            cellSizeMm - 4.0,
                            currentCellPen,
                            Qt::NoBrush),
             3.1);

    const double centerX = currentX + cellSizeMm * 0.5;
    const double centerY = currentY + cellSizeMm * 0.5;
    double dx = 0.0;
    double dy = 0.0;
    switch (mapDebug.dir) {
    case NAV_DIR_NORTH:
        dy = -1.0;
        break;
    case NAV_DIR_EAST:
        dx = 1.0;
        break;
    case NAV_DIR_SOUTH:
        dy = 1.0;
        break;
    case NAV_DIR_WEST:
        dx = -1.0;
        break;
    }

    const double arrowLength = cellSizeMm * 0.28;
    const double tipX = centerX + dx * arrowLength;
    const double tipY = centerY + dy * arrowLength;
    remember(scene->addLine(centerX, centerY, tipX, tipY, arrowPen), 3.2);

    const double headLength = cellSizeMm * 0.08;
    const double normalX = -dy;
    const double normalY = dx;
    QPolygonF head;
    head << QPointF(tipX, tipY)
         << QPointF(tipX - dx * headLength + normalX * headLength * 0.7,
                    tipY - dy * headLength + normalY * headLength * 0.7)
         << QPointF(tipX - dx * headLength - normalX * headLength * 0.7,
                    tipY - dy * headLength - normalY * headLength * 0.7);
    remember(scene->addPolygon(head, noPen, arrowBrush), 3.2);
}

void MainWindow::initializeIrSensors()
{
    irSensors = {
        {"front_left", 45.0, -20.0, 0.0, kIrMaxDistanceMm},
        {"front_right", 45.0, 20.0, 0.0, kIrMaxDistanceMm},
        {"left", 0.0, -40.0, -90.0, kIrMaxDistanceMm},
        {"right", 0.0, 40.0, 90.0, kIrMaxDistanceMm},
        {"diag_left", 30.0, -30.0, -45.0, kIrMaxDistanceMm},
        {"diag_right", 30.0, 30.0, 45.0, kIrMaxDistanceMm}
    };
}

void MainWindow::initializeFloorSensors()
{
    floorSensors = {
        {"floor_front", 42.0, 0.0},
        {"floor_rear", -42.0, 0.0}
    };
}

void MainWindow::createTelemetryPanel()
{
    auto *dock = new QDockWidget("Telemetry", this);
    auto *tree = new QTreeWidget(dock);
    auto *panel = tree;
    TelemetryTreeBuilder telemetryLayout(tree);
    auto *layout = &telemetryLayout;
    telemetryPinnedRows.clear();

    dock->setMinimumWidth(340);
    tree->setColumnCount(2);
    tree->setHeaderLabels({"Name", "Value"});
    tree->setRootIsDecorated(true);
    tree->setAlternatingRowColors(true);
    tree->setUniformRowHeights(true);
    tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    tree->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    tree->setTextElideMode(Qt::ElideRight);
    tree->header()->setStretchLastSection(true);
    tree->header()->setSectionResizeMode(0, QHeaderView::Interactive);
    tree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    tree->header()->resizeSection(0, 145);

    auto *poseTitle = new QLabel("<b>Pose</b>", panel);
    auto *mazeTitle = new QLabel("<b>Maze</b>", panel);
    auto *irTitle = new QLabel("<b>IR sensors</b>", panel);
    auto *floorTitle = new QLabel("<b>Floor sensors</b>", panel);
    auto *navPerceptionTitle = new QLabel("<b>Nav perception</b>", panel);
    auto *navTitle = new QLabel("<b>Nav snapshot</b>", panel);
    auto *turnDebugTitle = new QLabel("<b>Turn PI debug</b>", panel);
    auto *navCommandTitle = new QLabel("<b>Nav command</b>", panel);
    auto *simulationTitle = new QLabel("<b>Simulation</b>", panel);
    auto *sequenceTitle = new QLabel("<b>Test sequence</b>", panel);
    auto *planTitle = new QLabel("<b>Planned action queue</b>", panel);
    auto *navAutonomyTitle = new QLabel("<b>Basic nav autonomy</b>", panel);
    auto *mapTitle = new QLabel("<b>Shadow logical map</b>", panel);
    auto *motorTestTitle = new QLabel("<b>Motor test command</b>", panel);

    xValueLabel = new QLabel(panel);
    yValueLabel = new QLabel(panel);
    yawValueLabel = new QLabel(panel);
    mazeNameValueLabel = new QLabel(panel);
    mazeFileValueLabel = new QLabel(panel);
    mazeWidthValueLabel = new QLabel(panel);
    mazeHeightValueLabel = new QLabel(panel);
    mazeCellSizeValueLabel = new QLabel(panel);
    configureTelemetryValueLabel(xValueLabel);
    configureTelemetryValueLabel(yValueLabel);
    configureTelemetryValueLabel(yawValueLabel);
    configureTelemetryValueLabel(mazeNameValueLabel);
    configureTelemetryValueLabel(mazeFileValueLabel);
    configureTelemetryValueLabel(mazeWidthValueLabel);
    configureTelemetryValueLabel(mazeHeightValueLabel);
    configureTelemetryValueLabel(mazeCellSizeValueLabel);

    layout->addRow(poseTitle);
    layout->addRow("x:", xValueLabel);
    layout->addRow("y:", yValueLabel);
    layout->addRow("yaw:", yawValueLabel);

    layout->addRow(mazeTitle);
    layout->addRow("maze_name:", mazeNameValueLabel);
    layout->addRow("maze_file:", mazeFileValueLabel);
    layout->addRow("maze_width:", mazeWidthValueLabel);
    layout->addRow("maze_height:", mazeHeightValueLabel);
    layout->addRow("cell_size_mm:", mazeCellSizeValueLabel);

    layout->addRow(irTitle);
    for (IrSensor &sensor : irSensors) {
        sensor.value_label = new QLabel(panel);
        configureTelemetryValueLabel(sensor.value_label);
        layout->addRow(sensor.name, sensor.value_label);
    }

    layout->addRow(floorTitle);
    for (FloorSensor &sensor : floorSensors) {
        sensor.value_label = new QLabel(panel);
        configureTelemetryValueLabel(sensor.value_label);
        layout->addRow(sensor.name, sensor.value_label);
    }

    navFloorFrontValueLabel = new QLabel(panel);
    navFloorRearValueLabel = new QLabel(panel);
    navYawValueLabel = new QLabel(panel);
    navYawRateValueLabel = new QLabel(panel);
    navYawZeroValueLabel = new QLabel(panel);
    navStateValueLabel = new QLabel(panel);
    navActionValueLabel = new QLabel(panel);
    navActionStartYawValueLabel = new QLabel(panel);
    navActionTargetYawValueLabel = new QLabel(panel);
    navWallFrontValueLabel = new QLabel(panel);
    navWallLeftValueLabel = new QLabel(panel);
    navWallRightValueLabel = new QLabel(panel);
    navWallDiagLeftValueLabel = new QLabel(panel);
    navWallDiagRightValueLabel = new QLabel(panel);
    navWallFrontThresholdValueLabel = new QLabel(panel);
    navWallSideThresholdValueLabel = new QLabel(panel);
    navWallDiagThresholdValueLabel = new QLabel(panel);
    turnDebugSetpointValueLabel = new QLabel(panel);
    turnDebugMeasuredValueLabel = new QLabel(panel);
    turnDebugErrorValueLabel = new QLabel(panel);
    turnDebugOutputValueLabel = new QLabel(panel);
    turnDebugCorrectionValueLabel = new QLabel(panel);
    turnDebugIntegralValueLabel = new QLabel(panel);
    turnDebugKpValueLabel = new QLabel(panel);
    turnDebugKiValueLabel = new QLabel(panel);
    turnDebugKdValueLabel = new QLabel(panel);
    turnDebugOutputLimitValueLabel = new QLabel(panel);
    turnDebugSmoothTargetValueLabel = new QLabel(panel);
    turnDebugSmoothRightLeftBaseValueLabel = new QLabel(panel);
    turnDebugSmoothRightRightBaseValueLabel = new QLabel(panel);
    turnDebugSmoothLeftLeftBaseValueLabel = new QLabel(panel);
    turnDebugSmoothLeftRightBaseValueLabel = new QLabel(panel);
    turnDebugSmoothLeftBaseValueLabel = new QLabel(panel);
    turnDebugSmoothRightBaseValueLabel = new QLabel(panel);
    turnDebugSmoothPhaseValueLabel = new QLabel(panel);
    turnDebugSmoothDoneReasonValueLabel = new QLabel(panel);
    turnDebugSmoothPostYawElapsedValueLabel = new QLabel(panel);
    turnDebugSmoothFinalGuidanceSourceValueLabel = new QLabel(panel);
    turnDebugSmoothFinalHoldInitializedValueLabel = new QLabel(panel);
    turnDebugSmoothFinalHoldSourceValueLabel = new QLabel(panel);
    turnDebugSmoothFinalHoldRecaptureCountValueLabel = new QLabel(panel);
    turnDebugSmoothFinalLeftHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalRightHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalCenterDiffHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalWallErrorValueLabel = new QLabel(panel);
    turnDebugSmoothFinalWallCorrectionValueLabel = new QLabel(panel);
    turnDebugSmoothFinalYawCorrectionValueLabel = new QLabel(panel);
    turnDebugSmoothFinalYawHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalYawHoldInitializedValueLabel = new QLabel(panel);
    turnDebugSmoothFinalYawHoldRecaptureCountValueLabel = new QLabel(panel);
    turnDebugSmoothFinalYawErrorValueLabel = new QLabel(panel);
    turnDebugSmoothFinalAppliedCorrectionValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagLeftValidValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagRightValidValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagLeftValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagRightValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagTargetValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagErrorScaleValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagModeValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagHoldInitializedValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagLeftHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagRightHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagCenterDiffHoldValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagHoldRecaptureCountValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagRawErrorValueLabel = new QLabel(panel);
    turnDebugSmoothFinalDiagErrorValueLabel = new QLabel(panel);
    turnDebugSmoothFinalFollowLeftValidValueLabel = new QLabel(panel);
    turnDebugSmoothFinalFollowRightValidValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryEnabledValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryPendingValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryUsedValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryOffsetValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryEntryYawValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryExitYawValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryDiagUsedValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryCandidateSourceValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryRejectedReasonValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryOnlySetpointValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryRequireDiagValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryAllowAdvancePreviewValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryMinAbsValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryMaxAbsValueLabel = new QLabel(panel);
    turnDebugSmoothYawCarryScaleValueLabel = new QLabel(panel);
    turnDebugAdvancePhaseValueLabel = new QLabel(panel);
    turnDebugAdvanceDoneReasonValueLabel = new QLabel(panel);
    turnDebugRearBlackForLineValueLabel = new QLabel(panel);
    turnDebugFloorRearRealValueLabel = new QLabel(panel);
    turnDebugAdvanceStartedOnRearLineValueLabel = new QLabel(panel);
    turnDebugAdvanceStartModeValueLabel = new QLabel(panel);
    turnDebugAdvanceCenteredWaitingRearWhiteValueLabel = new QLabel(panel);
    turnDebugSpecialCandidateValueLabel = new QLabel(panel);
    turnDebugSpecialConfirmedValueLabel = new QLabel(panel);
    turnDebugSpecialIgnoreRearValueLabel = new QLabel(panel);
    turnDebugSpecialStartedOnRearLineValueLabel = new QLabel(panel);
    turnDebugSpecialEnabledForMotionValueLabel = new QLabel(panel);
    turnDebugSpecialDetectionContextValueLabel = new QLabel(panel);
    turnDebugSpecialAuxEnabledValueLabel = new QLabel(panel);
    turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel = new QLabel(panel);
    turnDebugInitialSpecialSnapshotPendingValueLabel = new QLabel(panel);
    turnDebugInitialSpecialSnapshotDoneValueLabel = new QLabel(panel);
    turnDebugRearLineTrustedValueLabel = new QLabel(panel);
    turnDebugRearLineTrustSourceValueLabel = new QLabel(panel);
    turnDebugSpecialMarkTargetCellValueLabel = new QLabel(panel);
    turnDebugSpecialMarkTargetSourceValueLabel = new QLabel(panel);
    turnDebugLastSpecialMarkActionValueLabel = new QLabel(panel);
    turnDebugAdvanceElapsedSinceLeaveValueLabel = new QLabel(panel);
    turnDebugSpecialDetectMinValueLabel = new QLabel(panel);
    turnDebugSpecialDetectMaxValueLabel = new QLabel(panel);
    turnDebugApproachFrontPhaseValueLabel = new QLabel(panel);
    turnDebugApproachFrontDoneReasonValueLabel = new QLabel(panel);
    turnDebugApproachFrontTargetValueLabel = new QLabel(panel);
    turnDebugApproachFrontLeftValueLabel = new QLabel(panel);
    turnDebugApproachFrontRightValueLabel = new QLabel(panel);
    turnDebugApproachFrontElapsedValueLabel = new QLabel(panel);
    turnDebugApproachFrontBrakeElapsedValueLabel = new QLabel(panel);
    turnDebugApproachFrontBaseLeftValueLabel = new QLabel(panel);
    turnDebugApproachFrontBaseRightValueLabel = new QLabel(panel);
    turnDebugApproachFrontCorrectionValueLabel = new QLabel(panel);
    turnDebugCenterPivotPhaseValueLabel = new QLabel(panel);
    turnDebugCenterPivotDoneReasonValueLabel = new QLabel(panel);
    turnDebugCenterPivotElapsedValueLabel = new QLabel(panel);
    turnDebugCenterPivotBrakeElapsedValueLabel = new QLabel(panel);
    turnDebugCenterPivotBaseLeftValueLabel = new QLabel(panel);
    turnDebugCenterPivotBaseRightValueLabel = new QLabel(panel);
    turnDebugCenterPivotCorrectionValueLabel = new QLabel(panel);
    turnDebugCenterPivotFrontBlackValueLabel = new QLabel(panel);
    turnDebugCenterPivotRearBlackValueLabel = new QLabel(panel);
    turnDebugCenterPivotFrontSeenWhiteValueLabel = new QLabel(panel);
    turnDebugAdvanceYawSetpointValueLabel = new QLabel(panel);
    turnDebugAdvanceYawMeasuredValueLabel = new QLabel(panel);
    turnDebugAdvanceYawErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceYawOutputValueLabel = new QLabel(panel);
    turnDebugAdvanceYawCorrectionValueLabel = new QLabel(panel);
    turnDebugAdvanceYawHoldValueLabel = new QLabel(panel);
    turnDebugAdvanceYawHoldInitializedValueLabel = new QLabel(panel);
    turnDebugAdvanceYawHoldRecaptureCountValueLabel = new QLabel(panel);
    turnDebugAdvanceYawHoldErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceGuidanceLastSourceValueLabel = new QLabel(panel);
    turnDebugAdvanceYawKpValueLabel = new QLabel(panel);
    turnDebugAdvanceYawKiValueLabel = new QLabel(panel);
    turnDebugAdvanceYawKdValueLabel = new QLabel(panel);
    turnDebugAdvanceYawOutputLimitValueLabel = new QLabel(panel);
    turnDebugAdvanceBaseLeftValueLabel = new QLabel(panel);
    turnDebugAdvanceBaseRightValueLabel = new QLabel(panel);
    turnDebugAdvanceGuidanceModeValueLabel = new QLabel(panel);
    turnDebugAdvanceCorrectionSourceValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagPreviewArmedValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagPreviewLatchedValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagPreviewActiveValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagSourceValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagRawErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceFrontDiagRightValidValueLabel = new QLabel(panel);
    turnDebugAdvanceWallLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRightValidValueLabel = new QLabel(panel);
    turnDebugAdvanceDiagLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceDiagRightValidValueLabel = new QLabel(panel);
    turnDebugAdvanceFollowLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceFollowRightValidValueLabel = new QLabel(panel);
    turnDebugWallCautionEnabledValueLabel = new QLabel(panel);
    turnDebugWallLeftConfidenceValueLabel = new QLabel(panel);
    turnDebugWallRightConfidenceValueLabel = new QLabel(panel);
    turnDebugWallLeftCautionElapsedValueLabel = new QLabel(panel);
    turnDebugWallRightCautionElapsedValueLabel = new QLabel(panel);
    turnDebugWallLeftCautionHoldValueLabel = new QLabel(panel);
    turnDebugWallRightCautionHoldValueLabel = new QLabel(panel);
    turnDebugWallLeftCautionDeltaValueLabel = new QLabel(panel);
    turnDebugWallRightCautionDeltaValueLabel = new QLabel(panel);
    turnDebugWallCautionCorrectionValueLabel = new QLabel(panel);
    turnDebugWallCautionLossReasonValueLabel = new QLabel(panel);
    turnDebugAdvanceWallLeftValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRightValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRawErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceWallErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceWallErrorAfterDeadbandValueLabel = new QLabel(panel);
    turnDebugAdvanceWallPrevErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceWallErrorDeltaValueLabel = new QLabel(panel);
    turnDebugAdvanceWallPTermValueLabel = new QLabel(panel);
    turnDebugAdvanceWallDTermValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRawCorrectionValueLabel = new QLabel(panel);
    turnDebugAdvanceWallLimitedCorrectionValueLabel = new QLabel(panel);
    turnDebugAdvanceWallCorrectionValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceKpValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceKdValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceLimitValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceScaleNumValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceScaleDenValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceTargetValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceSmoothFinalModeValueLabel = new QLabel(panel);
    turnDebugDiagGuidancePTermValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceDTermValueLabel = new QLabel(panel);
    turnDebugDiagGuidanceCorrectionValueLabel = new QLabel(panel);
    turnDebugWallKpValueLabel = new QLabel(panel);
    turnDebugWallKdValueLabel = new QLabel(panel);
    turnDebugWallDeadbandValueLabel = new QLabel(panel);
    turnDebugWallTargetLeftValueLabel = new QLabel(panel);
    turnDebugWallTargetRightValueLabel = new QLabel(panel);
    turnDebugWallCorrectionLimitValueLabel = new QLabel(panel);
    turnDebugWallSingleSideErrorScaleValueLabel = new QLabel(panel);
    turnDebugLastCompletedActionValueLabel = new QLabel(panel);
    turnDebugLastSmoothDoneReasonValueLabel = new QLabel(panel);
    turnDebugLastSmoothFinalYawValueLabel = new QLabel(panel);
    turnDebugLastSmoothFinalRearValueLabel = new QLabel(panel);
    turnDebugLastAdvanceDoneReasonValueLabel = new QLabel(panel);
    turnDebugLastAdvanceFinalYawValueLabel = new QLabel(panel);
    turnDebugLastAdvanceFinalRearValueLabel = new QLabel(panel);
    turnDebugLastApproachFrontDoneReasonValueLabel = new QLabel(panel);
    turnDebugLastCenterPivotDoneReasonValueLabel = new QLabel(panel);
    navLeftMotorValueLabel = new QLabel(panel);
    navRightMotorValueLabel = new QLabel(panel);
    simulationRunningValueLabel = new QLabel(panel);
    simulationDtValueLabel = new QLabel(panel);
    autoModeValueLabel = new QLabel(panel);
    simulationStepCountValueLabel = new QLabel(panel);
    simulationTimeValueLabel = new QLabel(panel);
    motorTestModeValueLabel = new QLabel(panel);
    sequenceEnabledValueLabel = new QLabel(panel);
    sequenceIndexValueLabel = new QLabel(panel);
    sequenceLengthValueLabel = new QLabel(panel);
    sequenceCurrentActionValueLabel = new QLabel(panel);
    sequenceWaitingNextTickValueLabel = new QLabel(panel);
    centerPivotSequenceActiveValueLabel = new QLabel(panel);
    centerPivotSequencePhaseValueLabel = new QLabel(panel);
    centerPivotSequenceLastCenterReasonValueLabel = new QLabel(panel);
    planExecutionEnabledValueLabel = new QLabel(panel);
    planQueueCountValueLabel = new QLabel(panel);
    planCurrentActionValueLabel = new QLabel(panel);
    planNextActionValueLabel = new QLabel(panel);
    planLastExecutedActionValueLabel = new QLabel(panel);
    planActionsExecutedCountValueLabel = new QLabel(panel);
    planQueueOverflowValueLabel = new QLabel(panel);
    planCompositeActionActiveValueLabel = new QLabel(panel);
    planCompositeActionPhaseValueLabel = new QLabel(panel);
    planCompositePrepareMethodValueLabel = new QLabel(panel);
    planCompositeWallFrontAtStartValueLabel = new QLabel(panel);
    planCompositeLastCenterReasonValueLabel = new QLabel(panel);
    planCompositeLastApproachReasonValueLabel = new QLabel(panel);
    planAdvanceAfterCenterPivotValueLabel = new QLabel(panel);
    planNextAdvanceFromCenteredPoseValueLabel = new QLabel(panel);
    routeStatusValueLabel = new QLabel(panel);
    routeTargetCellValueLabel = new QLabel(panel);
    routeStartCellValueLabel = new QLabel(panel);
    routeStartDirValueLabel = new QLabel(panel);
    routeLengthValueLabel = new QLabel(panel);
    routeExpandedStatesValueLabel = new QLabel(panel);
    routeFirstActionValueLabel = new QLabel(panel);
    routeLastActionValueLabel = new QLabel(panel);
    routeLoadedIntoPlanQueueValueLabel = new QLabel(panel);
    frontierRouteStatusValueLabel = new QLabel(panel);
    frontierTargetCellValueLabel = new QLabel(panel);
    frontierTargetDirValueLabel = new QLabel(panel);
    frontierExitDirAbsoluteValueLabel = new QLabel(panel);
    frontierExitRelativeValueLabel = new QLabel(panel);
    frontierNeighborCellValueLabel = new QLabel(panel);
    frontierRouteLengthValueLabel = new QLabel(panel);
    frontierExpandedStatesValueLabel = new QLabel(panel);
    frontierLoadedIntoPlanQueueValueLabel = new QLabel(panel);
    frontierCountFoundValueLabel = new QLabel(panel);
    routeExecuteStatusValueLabel = new QLabel(panel);
    routeStartPhysicalValidValueLabel = new QLabel(panel);
    routeStartFloorRearBlackValueLabel = new QLabel(panel);
    routePlanReadyToExecuteValueLabel = new QLabel(panel);
    navAutonomyEnabledValueLabel = new QLabel(panel);
    navPolicyValueLabel = new QLabel(panel);
    navRecommendedActionValueLabel = new QLabel(panel);
    navLastDecisionValueLabel = new QLabel(panel);
    navDecisionWallFrontValueLabel = new QLabel(panel);
    navDecisionWallLeftValueLabel = new QLabel(panel);
    navDecisionWallRightValueLabel = new QLabel(panel);
    navDecisionPointValidValueLabel = new QLabel(panel);
    deadEndRecoveryActiveValueLabel = new QLabel(panel);
    deadEndRecoveryPhaseValueLabel = new QLabel(panel);
    deadEndRecoveryLastApproachReasonValueLabel = new QLabel(panel);
    deadEndRecoveryPendingPivotValueLabel = new QLabel(panel);
    navMapCandidateRightCellValueLabel = new QLabel(panel);
    navMapCandidateFrontCellValueLabel = new QLabel(panel);
    navMapCandidateLeftCellValueLabel = new QLabel(panel);
    navMapCandidateRightVisitedValueLabel = new QLabel(panel);
    navMapCandidateFrontVisitedValueLabel = new QLabel(panel);
    navMapCandidateLeftVisitedValueLabel = new QLabel(panel);
    navMapUsedUnvisitedPreferenceValueLabel = new QLabel(panel);
    smartRecognitionStateValueLabel = new QLabel(panel);
    smartLastFrontierStatusValueLabel = new QLabel(panel);
    smartFrontierPlanRequestedCountValueLabel = new QLabel(panel);
    smartFrontierRoutesExecutedCountValueLabel = new QLabel(panel);
    smartNoFrontierCountValueLabel = new QLabel(panel);
    smartLocalActionValueLabel = new QLabel(panel);
    mode1MissionEnabledValueLabel = new QLabel(panel);
    mode1MissionStateValueLabel = new QLabel(panel);
    mode1RequiredSpecialCountValueLabel = new QLabel(panel);
    mode1FoundSpecialCountValueLabel = new QLabel(panel);
    mode1RequiredSpecialsReachedValueLabel = new QLabel(panel);
    mode1ReturnRequestedValueLabel = new QLabel(panel);
    mode1WaitingActionDoneValueLabel = new QLabel(panel);
    mode1SearchCompleteLatchedAtCountValueLabel = new QLabel(panel);
    mode1PlanCancelledAfterRequiredFoundValueLabel = new QLabel(panel);
    mode1NavReadyForReturnValueLabel = new QLabel(panel);
    mode1PlanWasActiveWhenRequiredFoundValueLabel = new QLabel(panel);
    mode1PlanQueueCountWhenRequiredFoundValueLabel = new QLabel(panel);
    mode1StartCellValueLabel = new QLabel(panel);
    mode1ReturnRouteStatusValueLabel = new QLabel(panel);
    mode1ReturnPlanLoadedValueLabel = new QLabel(panel);
    mode1ReturnToStartActiveValueLabel = new QLabel(panel);
    mode1AtStartCellValueLabel = new QLabel(panel);
    mode1DoneReasonValueLabel = new QLabel(panel);
    mapEnabledValueLabel = new QLabel(panel);
    mapWidthValueLabel = new QLabel(panel);
    mapHeightValueLabel = new QLabel(panel);
    mapCellXValueLabel = new QLabel(panel);
    mapCellYValueLabel = new QLabel(panel);
    mapDirValueLabel = new QLabel(panel);
    mapCurrentCellVisitedValueLabel = new QLabel(panel);
    mapCurrentCellSpecialValueLabel = new QLabel(panel);
    mapCurrentCellWallsKnownValueLabel = new QLabel(panel);
    mapCurrentCellWallsPresentValueLabel = new QLabel(panel);
    mapLastPoseUpdateActionValueLabel = new QLabel(panel);
    mapLastWallUpdateActionValueLabel = new QLabel(panel);
    mapInitialWallSnapshotPendingValueLabel = new QLabel(panel);
    mapUpdateCountValueLabel = new QLabel(panel);
    mapWallUpdateCountValueLabel = new QLabel(panel);
    mapSpecialCellsFoundCountValueLabel = new QLabel(panel);
    mapLastSpecialCellValueLabel = new QLabel(panel);
    mapOverlayEnabledValueLabel = new QLabel(panel);
    simLeftMotorGainValueLabel = new QLabel(panel);
    simRightMotorGainValueLabel = new QLabel(panel);
    simPivotCenterCorrectionEnabledValueLabel = new QLabel(panel);
    simPivotCenterLocalXValueLabel = new QLabel(panel);
    simPivotCenterLocalYValueLabel = new QLabel(panel);
    simLastMotionPivotLikeValueLabel = new QLabel(panel);
    simFloorRearGlobalXValueLabel = new QLabel(panel);
    simFloorRearGlobalYValueLabel = new QLabel(panel);
    motorTestLeftValueLabel = new QLabel(panel);
    motorTestRightValueLabel = new QLabel(panel);
    configureTelemetryValueLabel(navFloorFrontValueLabel);
    configureTelemetryValueLabel(navFloorRearValueLabel);
    configureTelemetryValueLabel(navYawValueLabel);
    configureTelemetryValueLabel(navYawRateValueLabel);
    configureTelemetryValueLabel(navYawZeroValueLabel);
    configureTelemetryValueLabel(navStateValueLabel);
    configureTelemetryValueLabel(navActionValueLabel);
    configureTelemetryValueLabel(navActionStartYawValueLabel);
    configureTelemetryValueLabel(navActionTargetYawValueLabel);
    configureTelemetryValueLabel(navWallFrontValueLabel);
    configureTelemetryValueLabel(navWallLeftValueLabel);
    configureTelemetryValueLabel(navWallRightValueLabel);
    configureTelemetryValueLabel(navWallDiagLeftValueLabel);
    configureTelemetryValueLabel(navWallDiagRightValueLabel);
    configureTelemetryValueLabel(navWallFrontThresholdValueLabel);
    configureTelemetryValueLabel(navWallSideThresholdValueLabel);
    configureTelemetryValueLabel(navWallDiagThresholdValueLabel);
    configureTelemetryValueLabel(turnDebugSetpointValueLabel);
    configureTelemetryValueLabel(turnDebugMeasuredValueLabel);
    configureTelemetryValueLabel(turnDebugErrorValueLabel);
    configureTelemetryValueLabel(turnDebugOutputValueLabel);
    configureTelemetryValueLabel(turnDebugCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugIntegralValueLabel);
    configureTelemetryValueLabel(turnDebugKpValueLabel);
    configureTelemetryValueLabel(turnDebugKiValueLabel);
    configureTelemetryValueLabel(turnDebugKdValueLabel);
    configureTelemetryValueLabel(turnDebugOutputLimitValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothTargetValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothRightLeftBaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothRightRightBaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothLeftLeftBaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothLeftRightBaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothLeftBaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothRightBaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothPhaseValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothPostYawElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalGuidanceSourceValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalHoldInitializedValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalHoldSourceValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalHoldRecaptureCountValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalLeftHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalRightHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalCenterDiffHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalWallErrorValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalWallCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalYawCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalYawHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalYawHoldInitializedValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalYawHoldRecaptureCountValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalYawErrorValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalAppliedCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagLeftValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagRightValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagTargetValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagErrorScaleValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagModeValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagHoldInitializedValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagLeftHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagRightHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagCenterDiffHoldValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagHoldRecaptureCountValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagRawErrorValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalDiagErrorValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalFollowLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothFinalFollowRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryEnabledValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryPendingValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryUsedValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryOffsetValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryEntryYawValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryExitYawValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryDiagUsedValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryCandidateSourceValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryRejectedReasonValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryOnlySetpointValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryRequireDiagValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryAllowAdvancePreviewValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryMinAbsValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryMaxAbsValueLabel);
    configureTelemetryValueLabel(turnDebugSmoothYawCarryScaleValueLabel);
    configureTelemetryValueLabel(turnDebugAdvancePhaseValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugRearBlackForLineValueLabel);
    configureTelemetryValueLabel(turnDebugFloorRearRealValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceStartedOnRearLineValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceStartModeValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceCenteredWaitingRearWhiteValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialCandidateValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialConfirmedValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialIgnoreRearValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialStartedOnRearLineValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialEnabledForMotionValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialDetectionContextValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialAuxEnabledValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel);
    configureTelemetryValueLabel(turnDebugInitialSpecialSnapshotPendingValueLabel);
    configureTelemetryValueLabel(turnDebugInitialSpecialSnapshotDoneValueLabel);
    configureTelemetryValueLabel(turnDebugRearLineTrustedValueLabel);
    configureTelemetryValueLabel(turnDebugRearLineTrustSourceValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialMarkTargetCellValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialMarkTargetSourceValueLabel);
    configureTelemetryValueLabel(turnDebugLastSpecialMarkActionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceElapsedSinceLeaveValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialDetectMinValueLabel);
    configureTelemetryValueLabel(turnDebugSpecialDetectMaxValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontPhaseValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontTargetValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontLeftValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontRightValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontBrakeElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontBaseLeftValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontBaseRightValueLabel);
    configureTelemetryValueLabel(turnDebugApproachFrontCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotPhaseValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotBrakeElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotBaseLeftValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotBaseRightValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotFrontBlackValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotRearBlackValueLabel);
    configureTelemetryValueLabel(turnDebugCenterPivotFrontSeenWhiteValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawSetpointValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawMeasuredValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawOutputValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawHoldValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawHoldInitializedValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawHoldRecaptureCountValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawHoldErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceGuidanceLastSourceValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawKpValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawKiValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawKdValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawOutputLimitValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceBaseLeftValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceBaseRightValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceGuidanceModeValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceCorrectionSourceValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagPreviewArmedValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagPreviewLatchedValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagPreviewActiveValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagSourceValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagRawErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFrontDiagRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceDiagLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceDiagRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFollowLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFollowRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugWallCautionEnabledValueLabel);
    configureTelemetryValueLabel(turnDebugWallLeftConfidenceValueLabel);
    configureTelemetryValueLabel(turnDebugWallRightConfidenceValueLabel);
    configureTelemetryValueLabel(turnDebugWallLeftCautionElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugWallRightCautionElapsedValueLabel);
    configureTelemetryValueLabel(turnDebugWallLeftCautionHoldValueLabel);
    configureTelemetryValueLabel(turnDebugWallRightCautionHoldValueLabel);
    configureTelemetryValueLabel(turnDebugWallLeftCautionDeltaValueLabel);
    configureTelemetryValueLabel(turnDebugWallRightCautionDeltaValueLabel);
    configureTelemetryValueLabel(turnDebugWallCautionCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugWallCautionLossReasonValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallLeftValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRightValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRawErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallErrorAfterDeadbandValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallPrevErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallErrorDeltaValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallPTermValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallDTermValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRawCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallLimitedCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceKpValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceKdValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceLimitValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceScaleNumValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceScaleDenValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceTargetValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceSmoothFinalModeValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidancePTermValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceDTermValueLabel);
    configureTelemetryValueLabel(turnDebugDiagGuidanceCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugWallKpValueLabel);
    configureTelemetryValueLabel(turnDebugWallKdValueLabel);
    configureTelemetryValueLabel(turnDebugWallDeadbandValueLabel);
    configureTelemetryValueLabel(turnDebugWallTargetLeftValueLabel);
    configureTelemetryValueLabel(turnDebugWallTargetRightValueLabel);
    configureTelemetryValueLabel(turnDebugWallCorrectionLimitValueLabel);
    configureTelemetryValueLabel(turnDebugWallSingleSideErrorScaleValueLabel);
    configureTelemetryValueLabel(turnDebugLastCompletedActionValueLabel);
    configureTelemetryValueLabel(turnDebugLastSmoothDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugLastSmoothFinalYawValueLabel);
    configureTelemetryValueLabel(turnDebugLastSmoothFinalRearValueLabel);
    configureTelemetryValueLabel(turnDebugLastAdvanceDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugLastAdvanceFinalYawValueLabel);
    configureTelemetryValueLabel(turnDebugLastAdvanceFinalRearValueLabel);
    configureTelemetryValueLabel(turnDebugLastApproachFrontDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugLastCenterPivotDoneReasonValueLabel);
    configureTelemetryValueLabel(navLeftMotorValueLabel);
    configureTelemetryValueLabel(navRightMotorValueLabel);
    configureTelemetryValueLabel(simulationRunningValueLabel);
    configureTelemetryValueLabel(simulationDtValueLabel);
    configureTelemetryValueLabel(autoModeValueLabel);
    configureTelemetryValueLabel(simulationStepCountValueLabel);
    configureTelemetryValueLabel(simulationTimeValueLabel);
    configureTelemetryValueLabel(motorTestModeValueLabel);
    configureTelemetryValueLabel(sequenceEnabledValueLabel);
    configureTelemetryValueLabel(sequenceIndexValueLabel);
    configureTelemetryValueLabel(sequenceLengthValueLabel);
    configureTelemetryValueLabel(sequenceCurrentActionValueLabel);
    configureTelemetryValueLabel(sequenceWaitingNextTickValueLabel);
    configureTelemetryValueLabel(centerPivotSequenceActiveValueLabel);
    configureTelemetryValueLabel(centerPivotSequencePhaseValueLabel);
    configureTelemetryValueLabel(centerPivotSequenceLastCenterReasonValueLabel);
    configureTelemetryValueLabel(planExecutionEnabledValueLabel);
    configureTelemetryValueLabel(planQueueCountValueLabel);
    configureTelemetryValueLabel(planCurrentActionValueLabel);
    configureTelemetryValueLabel(planNextActionValueLabel);
    configureTelemetryValueLabel(planLastExecutedActionValueLabel);
    configureTelemetryValueLabel(planActionsExecutedCountValueLabel);
    configureTelemetryValueLabel(planQueueOverflowValueLabel);
    configureTelemetryValueLabel(planCompositeActionActiveValueLabel);
    configureTelemetryValueLabel(planCompositeActionPhaseValueLabel);
    configureTelemetryValueLabel(planCompositePrepareMethodValueLabel);
    configureTelemetryValueLabel(planCompositeWallFrontAtStartValueLabel);
    configureTelemetryValueLabel(planCompositeLastCenterReasonValueLabel);
    configureTelemetryValueLabel(planCompositeLastApproachReasonValueLabel);
    configureTelemetryValueLabel(planAdvanceAfterCenterPivotValueLabel);
    configureTelemetryValueLabel(planNextAdvanceFromCenteredPoseValueLabel);
    configureTelemetryValueLabel(routeStatusValueLabel);
    configureTelemetryValueLabel(routeTargetCellValueLabel);
    configureTelemetryValueLabel(routeStartCellValueLabel);
    configureTelemetryValueLabel(routeStartDirValueLabel);
    configureTelemetryValueLabel(routeLengthValueLabel);
    configureTelemetryValueLabel(routeExpandedStatesValueLabel);
    configureTelemetryValueLabel(routeFirstActionValueLabel);
    configureTelemetryValueLabel(routeLastActionValueLabel);
    configureTelemetryValueLabel(routeLoadedIntoPlanQueueValueLabel);
    configureTelemetryValueLabel(frontierRouteStatusValueLabel);
    configureTelemetryValueLabel(frontierTargetCellValueLabel);
    configureTelemetryValueLabel(frontierTargetDirValueLabel);
    configureTelemetryValueLabel(frontierExitDirAbsoluteValueLabel);
    configureTelemetryValueLabel(frontierExitRelativeValueLabel);
    configureTelemetryValueLabel(frontierNeighborCellValueLabel);
    configureTelemetryValueLabel(frontierRouteLengthValueLabel);
    configureTelemetryValueLabel(frontierExpandedStatesValueLabel);
    configureTelemetryValueLabel(frontierLoadedIntoPlanQueueValueLabel);
    configureTelemetryValueLabel(frontierCountFoundValueLabel);
    configureTelemetryValueLabel(routeExecuteStatusValueLabel);
    configureTelemetryValueLabel(routeStartPhysicalValidValueLabel);
    configureTelemetryValueLabel(routeStartFloorRearBlackValueLabel);
    configureTelemetryValueLabel(routePlanReadyToExecuteValueLabel);
    configureTelemetryValueLabel(navAutonomyEnabledValueLabel);
    configureTelemetryValueLabel(navPolicyValueLabel);
    configureTelemetryValueLabel(navRecommendedActionValueLabel);
    configureTelemetryValueLabel(navLastDecisionValueLabel);
    configureTelemetryValueLabel(navDecisionWallFrontValueLabel);
    configureTelemetryValueLabel(navDecisionWallLeftValueLabel);
    configureTelemetryValueLabel(navDecisionWallRightValueLabel);
    configureTelemetryValueLabel(navDecisionPointValidValueLabel);
    configureTelemetryValueLabel(deadEndRecoveryActiveValueLabel);
    configureTelemetryValueLabel(deadEndRecoveryPhaseValueLabel);
    configureTelemetryValueLabel(deadEndRecoveryLastApproachReasonValueLabel);
    configureTelemetryValueLabel(deadEndRecoveryPendingPivotValueLabel);
    configureTelemetryValueLabel(navMapCandidateRightCellValueLabel);
    configureTelemetryValueLabel(navMapCandidateFrontCellValueLabel);
    configureTelemetryValueLabel(navMapCandidateLeftCellValueLabel);
    configureTelemetryValueLabel(navMapCandidateRightVisitedValueLabel);
    configureTelemetryValueLabel(navMapCandidateFrontVisitedValueLabel);
    configureTelemetryValueLabel(navMapCandidateLeftVisitedValueLabel);
    configureTelemetryValueLabel(navMapUsedUnvisitedPreferenceValueLabel);
    configureTelemetryValueLabel(smartRecognitionStateValueLabel);
    configureTelemetryValueLabel(smartLastFrontierStatusValueLabel);
    configureTelemetryValueLabel(smartFrontierPlanRequestedCountValueLabel);
    configureTelemetryValueLabel(smartFrontierRoutesExecutedCountValueLabel);
    configureTelemetryValueLabel(smartNoFrontierCountValueLabel);
    configureTelemetryValueLabel(smartLocalActionValueLabel);
    configureTelemetryValueLabel(mode1MissionEnabledValueLabel);
    configureTelemetryValueLabel(mode1MissionStateValueLabel);
    configureTelemetryValueLabel(mode1RequiredSpecialCountValueLabel);
    configureTelemetryValueLabel(mode1FoundSpecialCountValueLabel);
    configureTelemetryValueLabel(mode1RequiredSpecialsReachedValueLabel);
    configureTelemetryValueLabel(mode1ReturnRequestedValueLabel);
    configureTelemetryValueLabel(mode1WaitingActionDoneValueLabel);
    configureTelemetryValueLabel(mode1SearchCompleteLatchedAtCountValueLabel);
    configureTelemetryValueLabel(mode1PlanCancelledAfterRequiredFoundValueLabel);
    configureTelemetryValueLabel(mode1NavReadyForReturnValueLabel);
    configureTelemetryValueLabel(mode1PlanWasActiveWhenRequiredFoundValueLabel);
    configureTelemetryValueLabel(mode1PlanQueueCountWhenRequiredFoundValueLabel);
    configureTelemetryValueLabel(mode1StartCellValueLabel);
    configureTelemetryValueLabel(mode1ReturnRouteStatusValueLabel);
    configureTelemetryValueLabel(mode1ReturnPlanLoadedValueLabel);
    configureTelemetryValueLabel(mode1ReturnToStartActiveValueLabel);
    configureTelemetryValueLabel(mode1AtStartCellValueLabel);
    configureTelemetryValueLabel(mode1DoneReasonValueLabel);
    configureTelemetryValueLabel(mapEnabledValueLabel);
    configureTelemetryValueLabel(mapWidthValueLabel);
    configureTelemetryValueLabel(mapHeightValueLabel);
    configureTelemetryValueLabel(mapCellXValueLabel);
    configureTelemetryValueLabel(mapCellYValueLabel);
    configureTelemetryValueLabel(mapDirValueLabel);
    configureTelemetryValueLabel(mapCurrentCellVisitedValueLabel);
    configureTelemetryValueLabel(mapCurrentCellSpecialValueLabel);
    configureTelemetryValueLabel(mapCurrentCellWallsKnownValueLabel);
    configureTelemetryValueLabel(mapCurrentCellWallsPresentValueLabel);
    configureTelemetryValueLabel(mapLastPoseUpdateActionValueLabel);
    configureTelemetryValueLabel(mapLastWallUpdateActionValueLabel);
    configureTelemetryValueLabel(mapInitialWallSnapshotPendingValueLabel);
    configureTelemetryValueLabel(mapUpdateCountValueLabel);
    configureTelemetryValueLabel(mapWallUpdateCountValueLabel);
    configureTelemetryValueLabel(mapSpecialCellsFoundCountValueLabel);
    configureTelemetryValueLabel(mapLastSpecialCellValueLabel);
    configureTelemetryValueLabel(mapOverlayEnabledValueLabel);
    configureTelemetryValueLabel(simLeftMotorGainValueLabel);
    configureTelemetryValueLabel(simRightMotorGainValueLabel);
    configureTelemetryValueLabel(simPivotCenterCorrectionEnabledValueLabel);
    configureTelemetryValueLabel(simPivotCenterLocalXValueLabel);
    configureTelemetryValueLabel(simPivotCenterLocalYValueLabel);
    configureTelemetryValueLabel(simLastMotionPivotLikeValueLabel);
    configureTelemetryValueLabel(simFloorRearGlobalXValueLabel);
    configureTelemetryValueLabel(simFloorRearGlobalYValueLabel);
    configureTelemetryValueLabel(motorTestLeftValueLabel);
    configureTelemetryValueLabel(motorTestRightValueLabel);

    layout->addRow(navPerceptionTitle);
    layout->addRow("wall_front:", navWallFrontValueLabel);
    layout->addRow("wall_left:", navWallLeftValueLabel);
    layout->addRow("wall_right:", navWallRightValueLabel);
    layout->addRow("wall_diag_left:", navWallDiagLeftValueLabel);
    layout->addRow("wall_diag_right:", navWallDiagRightValueLabel);
    layout->addRow("wall_front_threshold_mm:", navWallFrontThresholdValueLabel);
    layout->addRow("wall_side_threshold_mm:", navWallSideThresholdValueLabel);
    layout->addRow("wall_diag_threshold_mm:", navWallDiagThresholdValueLabel);

    layout->addRow(navTitle);
    layout->addRow("floor_front_black:", navFloorFrontValueLabel);
    layout->addRow("floor_rear_black:", navFloorRearValueLabel);
    layout->addRow("yaw_deg:", navYawValueLabel);
    layout->addRow("yaw_rate_deg_s:", navYawRateValueLabel);
    layout->addRow("yaw_zero_global:", navYawZeroValueLabel);
    layout->addRow("nav_state:", navStateValueLabel);
    layout->addRow("nav_action:", navActionValueLabel);
    layout->addRow("action_start_yaw:", navActionStartYawValueLabel);
    layout->addRow("action_target_yaw:", navActionTargetYawValueLabel);

    layout->addRow(turnDebugTitle);
    layout->addRow("setpoint_deg_s:", turnDebugSetpointValueLabel);
    layout->addRow("measured_deg_s:", turnDebugMeasuredValueLabel);
    layout->addRow("error_deg_s:", turnDebugErrorValueLabel);
    layout->addRow("pid_output_pwm:", turnDebugOutputValueLabel);
    layout->addRow("correction_pwm:", turnDebugCorrectionValueLabel);
    layout->addRow("integral:", turnDebugIntegralValueLabel);
    layout->addRow("turn_kp:", turnDebugKpValueLabel);
    layout->addRow("turn_ki:", turnDebugKiValueLabel);
    layout->addRow("turn_kd:", turnDebugKdValueLabel);
    layout->addRow("turn_output_limit_pwm:", turnDebugOutputLimitValueLabel);
    layout->addRow("smooth_target_yaw_rate_deg_s:", turnDebugSmoothTargetValueLabel);
    layout->addRow("smooth_right_left_base_pwm:", turnDebugSmoothRightLeftBaseValueLabel);
    layout->addRow("smooth_right_right_base_pwm:", turnDebugSmoothRightRightBaseValueLabel);
    layout->addRow("smooth_left_left_base_pwm:", turnDebugSmoothLeftLeftBaseValueLabel);
    layout->addRow("smooth_left_right_base_pwm:", turnDebugSmoothLeftRightBaseValueLabel);
    layout->addRow("smooth_left_base_pwm:", turnDebugSmoothLeftBaseValueLabel);
    layout->addRow("smooth_right_base_pwm:", turnDebugSmoothRightBaseValueLabel);
    layout->addRow("smooth_phase:", turnDebugSmoothPhaseValueLabel);
    layout->addRow("smooth_done_reason:", turnDebugSmoothDoneReasonValueLabel);
    layout->addRow("smooth_post_yaw_elapsed_ms:", turnDebugSmoothPostYawElapsedValueLabel);
    layout->addRow("smooth_final_guidance_source:", turnDebugSmoothFinalGuidanceSourceValueLabel);
    layout->addRow("smooth_final_hold_initialized:",
                   turnDebugSmoothFinalHoldInitializedValueLabel);
    layout->addRow("smooth_final_hold_source:", turnDebugSmoothFinalHoldSourceValueLabel);
    layout->addRow("smooth_final_hold_recapture_count:",
                   turnDebugSmoothFinalHoldRecaptureCountValueLabel);
    layout->addRow("smooth_final_left_hold_mm:", turnDebugSmoothFinalLeftHoldValueLabel);
    layout->addRow("smooth_final_right_hold_mm:", turnDebugSmoothFinalRightHoldValueLabel);
    layout->addRow("smooth_final_center_diff_hold_mm:",
                   turnDebugSmoothFinalCenterDiffHoldValueLabel);
    layout->addRow("smooth_final_wall_error_mm:", turnDebugSmoothFinalWallErrorValueLabel);
    layout->addRow("smooth_final_wall_correction_pwm:",
                   turnDebugSmoothFinalWallCorrectionValueLabel);
    layout->addRow("smooth_final_yaw_correction_pwm:",
                   turnDebugSmoothFinalYawCorrectionValueLabel);
    layout->addRow("smooth_final_yaw_hold_deg:", turnDebugSmoothFinalYawHoldValueLabel);
    layout->addRow("smooth_final_yaw_hold_initialized:",
                   turnDebugSmoothFinalYawHoldInitializedValueLabel);
    layout->addRow("smooth_final_yaw_hold_recapture_count:",
                   turnDebugSmoothFinalYawHoldRecaptureCountValueLabel);
    layout->addRow("smooth_final_yaw_error_deg:", turnDebugSmoothFinalYawErrorValueLabel);
    layout->addRow("smooth_final_applied_correction_pwm:",
                   turnDebugSmoothFinalAppliedCorrectionValueLabel);
    layout->addRow("smooth_final_diag_left_valid:",
                   turnDebugSmoothFinalDiagLeftValidValueLabel);
    layout->addRow("smooth_final_diag_right_valid:",
                   turnDebugSmoothFinalDiagRightValidValueLabel);
    layout->addRow("smooth_final_diag_left_mm:", turnDebugSmoothFinalDiagLeftValueLabel);
    layout->addRow("smooth_final_diag_right_mm:", turnDebugSmoothFinalDiagRightValueLabel);
    layout->addRow("smooth_final_diag_target_mm:", turnDebugSmoothFinalDiagTargetValueLabel);
    layout->addRow("smooth_final_diag_error_scale:",
                   turnDebugSmoothFinalDiagErrorScaleValueLabel);
    layout->addRow("smooth_final_diag_mode:", turnDebugSmoothFinalDiagModeValueLabel);
    layout->addRow("smooth_final_diag_hold_initialized:",
                   turnDebugSmoothFinalDiagHoldInitializedValueLabel);
    layout->addRow("smooth_final_diag_left_hold_mm:",
                   turnDebugSmoothFinalDiagLeftHoldValueLabel);
    layout->addRow("smooth_final_diag_right_hold_mm:",
                   turnDebugSmoothFinalDiagRightHoldValueLabel);
    layout->addRow("smooth_final_diag_center_diff_hold_mm:",
                   turnDebugSmoothFinalDiagCenterDiffHoldValueLabel);
    layout->addRow("smooth_final_diag_hold_recapture_count:",
                   turnDebugSmoothFinalDiagHoldRecaptureCountValueLabel);
    layout->addRow("smooth_final_diag_raw_error_mm:",
                   turnDebugSmoothFinalDiagRawErrorValueLabel);
    layout->addRow("smooth_final_diag_error_mm:", turnDebugSmoothFinalDiagErrorValueLabel);
    layout->addRow("smooth_final_follow_left_valid:",
                   turnDebugSmoothFinalFollowLeftValidValueLabel);
    layout->addRow("smooth_final_follow_right_valid:",
                   turnDebugSmoothFinalFollowRightValidValueLabel);
    layout->addRow("smooth_yaw_carry_enabled:", turnDebugSmoothYawCarryEnabledValueLabel);
    layout->addRow("smooth_yaw_carry_pending:", turnDebugSmoothYawCarryPendingValueLabel);
    layout->addRow("smooth_yaw_carry_used:", turnDebugSmoothYawCarryUsedValueLabel);
    layout->addRow("smooth_yaw_carry_offset_deg:", turnDebugSmoothYawCarryOffsetValueLabel);
    layout->addRow("smooth_yaw_carry_entry_yaw_deg:",
                   turnDebugSmoothYawCarryEntryYawValueLabel);
    layout->addRow("smooth_yaw_carry_exit_yaw_deg:",
                   turnDebugSmoothYawCarryExitYawValueLabel);
    layout->addRow("smooth_yaw_carry_diag_used:", turnDebugSmoothYawCarryDiagUsedValueLabel);
    layout->addRow("smooth_yaw_carry_candidate_source:",
                   turnDebugSmoothYawCarryCandidateSourceValueLabel);
    layout->addRow("smooth_yaw_carry_rejected_reason:",
                   turnDebugSmoothYawCarryRejectedReasonValueLabel);
    layout->addRow("smooth_yaw_carry_only_setpoint:",
                   turnDebugSmoothYawCarryOnlySetpointValueLabel);
    layout->addRow("smooth_yaw_carry_require_diag:",
                   turnDebugSmoothYawCarryRequireDiagValueLabel);
    layout->addRow("smooth_yaw_carry_allow_advance_preview:",
                   turnDebugSmoothYawCarryAllowAdvancePreviewValueLabel);
    layout->addRow("smooth_yaw_carry_min_abs_deg:", turnDebugSmoothYawCarryMinAbsValueLabel);
    layout->addRow("smooth_yaw_carry_max_abs_deg:", turnDebugSmoothYawCarryMaxAbsValueLabel);
    layout->addRow("smooth_yaw_carry_offset_scale:", turnDebugSmoothYawCarryScaleValueLabel);
    layout->addRow("advance_phase:", turnDebugAdvancePhaseValueLabel);
    layout->addRow("advance_done_reason:", turnDebugAdvanceDoneReasonValueLabel);
    layout->addRow("rear_black_for_line:", turnDebugRearBlackForLineValueLabel);
    layout->addRow("floor_rear_black_real:", turnDebugFloorRearRealValueLabel);
    layout->addRow("advance_started_on_rear_line:",
                   turnDebugAdvanceStartedOnRearLineValueLabel);
    layout->addRow("advance_start_mode:", turnDebugAdvanceStartModeValueLabel);
    layout->addRow("advance_from_centered_waiting_rear_white:",
                   turnDebugAdvanceCenteredWaitingRearWhiteValueLabel);
    layout->addRow("special_candidate:", turnDebugSpecialCandidateValueLabel);
    layout->addRow("special_confirmed:", turnDebugSpecialConfirmedValueLabel);
    layout->addRow("special_ignore_rear_until_white:", turnDebugSpecialIgnoreRearValueLabel);
    layout->addRow("special_detection_started_on_rear_line:",
                   turnDebugSpecialStartedOnRearLineValueLabel);
    layout->addRow("special_detection_enabled_for_current_motion:",
                   turnDebugSpecialEnabledForMotionValueLabel);
    layout->addRow("special_detection_context:", turnDebugSpecialDetectionContextValueLabel);
    layout->addRow("special_aux_detection_enabled:", turnDebugSpecialAuxEnabledValueLabel);
    layout->addRow("special_aux_started_after_rear_line_left:",
                   turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel);
    layout->addRow("initial_special_snapshot_pending:",
                   turnDebugInitialSpecialSnapshotPendingValueLabel);
    layout->addRow("initial_special_snapshot_done:",
                   turnDebugInitialSpecialSnapshotDoneValueLabel);
    layout->addRow("rear_line_trusted_for_decision:", turnDebugRearLineTrustedValueLabel);
    layout->addRow("rear_line_trust_source:", turnDebugRearLineTrustSourceValueLabel);
    layout->addRow("special_mark_target_cell:", turnDebugSpecialMarkTargetCellValueLabel);
    layout->addRow("special_mark_target_source:", turnDebugSpecialMarkTargetSourceValueLabel);
    layout->addRow("last_special_mark_action:", turnDebugLastSpecialMarkActionValueLabel);
    layout->addRow("advance_elapsed_since_leave_start_line_ms:",
                   turnDebugAdvanceElapsedSinceLeaveValueLabel);
    layout->addRow("special_detect_min_ms:", turnDebugSpecialDetectMinValueLabel);
    layout->addRow("special_detect_max_ms:", turnDebugSpecialDetectMaxValueLabel);
    layout->addRow("approach_front_phase:", turnDebugApproachFrontPhaseValueLabel);
    layout->addRow("approach_front_done_reason:", turnDebugApproachFrontDoneReasonValueLabel);
    layout->addRow("approach_front_target_mm:", turnDebugApproachFrontTargetValueLabel);
    layout->addRow("approach_front_left_mm:", turnDebugApproachFrontLeftValueLabel);
    layout->addRow("approach_front_right_mm:", turnDebugApproachFrontRightValueLabel);
    layout->addRow("approach_front_elapsed_ms:", turnDebugApproachFrontElapsedValueLabel);
    layout->addRow("approach_front_brake_elapsed_ms:", turnDebugApproachFrontBrakeElapsedValueLabel);
    layout->addRow("approach_front_base_left_pwm:", turnDebugApproachFrontBaseLeftValueLabel);
    layout->addRow("approach_front_base_right_pwm:", turnDebugApproachFrontBaseRightValueLabel);
    layout->addRow("approach_front_correction_pwm:", turnDebugApproachFrontCorrectionValueLabel);
    layout->addRow("center_pivot_phase:", turnDebugCenterPivotPhaseValueLabel);
    layout->addRow("center_pivot_done_reason:", turnDebugCenterPivotDoneReasonValueLabel);
    layout->addRow("center_pivot_elapsed_ms:", turnDebugCenterPivotElapsedValueLabel);
    layout->addRow("center_pivot_brake_elapsed_ms:", turnDebugCenterPivotBrakeElapsedValueLabel);
    layout->addRow("center_pivot_base_left_pwm:", turnDebugCenterPivotBaseLeftValueLabel);
    layout->addRow("center_pivot_base_right_pwm:", turnDebugCenterPivotBaseRightValueLabel);
    layout->addRow("center_pivot_correction_pwm:", turnDebugCenterPivotCorrectionValueLabel);
    layout->addRow("center_pivot_front_black:", turnDebugCenterPivotFrontBlackValueLabel);
    layout->addRow("center_pivot_rear_black:", turnDebugCenterPivotRearBlackValueLabel);
    layout->addRow("center_pivot_front_seen_white:", turnDebugCenterPivotFrontSeenWhiteValueLabel);
    layout->addRow("advance_yaw_setpoint:", turnDebugAdvanceYawSetpointValueLabel);
    layout->addRow("advance_yaw_measured:", turnDebugAdvanceYawMeasuredValueLabel);
    layout->addRow("advance_yaw_error:", turnDebugAdvanceYawErrorValueLabel);
    layout->addRow("advance_yaw_pid_output:", turnDebugAdvanceYawOutputValueLabel);
    layout->addRow("advance_yaw_correction:", turnDebugAdvanceYawCorrectionValueLabel);
    layout->addRow("advance_yaw_hold_deg:", turnDebugAdvanceYawHoldValueLabel);
    layout->addRow("advance_yaw_hold_initialized:",
                   turnDebugAdvanceYawHoldInitializedValueLabel);
    layout->addRow("advance_yaw_hold_recapture_count:",
                   turnDebugAdvanceYawHoldRecaptureCountValueLabel);
    layout->addRow("advance_yaw_hold_error_deg:", turnDebugAdvanceYawHoldErrorValueLabel);
    layout->addRow("advance_guidance_last_source:",
                   turnDebugAdvanceGuidanceLastSourceValueLabel);
    layout->addRow("advance_yaw_kp:", turnDebugAdvanceYawKpValueLabel);
    layout->addRow("advance_yaw_ki:", turnDebugAdvanceYawKiValueLabel);
    layout->addRow("advance_yaw_kd:", turnDebugAdvanceYawKdValueLabel);
    layout->addRow("advance_yaw_output_limit_pwm:", turnDebugAdvanceYawOutputLimitValueLabel);
    layout->addRow("advance_base_left_pwm:", turnDebugAdvanceBaseLeftValueLabel);
    layout->addRow("advance_base_right_pwm:", turnDebugAdvanceBaseRightValueLabel);
    layout->addRow("advance_guidance_mode:", turnDebugAdvanceGuidanceModeValueLabel);
    layout->addRow("advance_final_correction_source:", turnDebugAdvanceCorrectionSourceValueLabel);
    layout->addRow("advance_front_diag_preview_armed:",
                   turnDebugAdvanceFrontDiagPreviewArmedValueLabel);
    layout->addRow("advance_front_diag_preview_latched:",
                   turnDebugAdvanceFrontDiagPreviewLatchedValueLabel);
    layout->addRow("advance_front_diag_preview_active:",
                   turnDebugAdvanceFrontDiagPreviewActiveValueLabel);
    layout->addRow("advance_front_diag_source:", turnDebugAdvanceFrontDiagSourceValueLabel);
    layout->addRow("advance_front_diag_raw_error_mm:",
                   turnDebugAdvanceFrontDiagRawErrorValueLabel);
    layout->addRow("advance_front_diag_error_mm:", turnDebugAdvanceFrontDiagErrorValueLabel);
    layout->addRow("advance_front_diag_left_valid:",
                   turnDebugAdvanceFrontDiagLeftValidValueLabel);
    layout->addRow("advance_front_diag_right_valid:",
                   turnDebugAdvanceFrontDiagRightValidValueLabel);
    layout->addRow("advance_wall_left_valid:", turnDebugAdvanceWallLeftValidValueLabel);
    layout->addRow("advance_wall_right_valid:", turnDebugAdvanceWallRightValidValueLabel);
    layout->addRow("advance_diag_left_valid:", turnDebugAdvanceDiagLeftValidValueLabel);
    layout->addRow("advance_diag_right_valid:", turnDebugAdvanceDiagRightValidValueLabel);
    layout->addRow("advance_follow_left_valid:", turnDebugAdvanceFollowLeftValidValueLabel);
    layout->addRow("advance_follow_right_valid:", turnDebugAdvanceFollowRightValidValueLabel);
    layout->addRow("wall_caution_enabled:", turnDebugWallCautionEnabledValueLabel);
    layout->addRow("wall_left_confidence:", turnDebugWallLeftConfidenceValueLabel);
    layout->addRow("wall_right_confidence:", turnDebugWallRightConfidenceValueLabel);
    layout->addRow("wall_left_caution_elapsed_ms:",
                   turnDebugWallLeftCautionElapsedValueLabel);
    layout->addRow("wall_right_caution_elapsed_ms:",
                   turnDebugWallRightCautionElapsedValueLabel);
    layout->addRow("wall_left_caution_hold_mm:", turnDebugWallLeftCautionHoldValueLabel);
    layout->addRow("wall_right_caution_hold_mm:", turnDebugWallRightCautionHoldValueLabel);
    layout->addRow("wall_left_caution_delta_mm:", turnDebugWallLeftCautionDeltaValueLabel);
    layout->addRow("wall_right_caution_delta_mm:", turnDebugWallRightCautionDeltaValueLabel);
    layout->addRow("wall_caution_correction_pwm:", turnDebugWallCautionCorrectionValueLabel);
    layout->addRow("wall_caution_loss_reason:", turnDebugWallCautionLossReasonValueLabel);
    layout->addRow("advance_wall_left_mm:", turnDebugAdvanceWallLeftValueLabel);
    layout->addRow("advance_wall_right_mm:", turnDebugAdvanceWallRightValueLabel);
    layout->addRow("advance_wall_raw_error_mm:", turnDebugAdvanceWallRawErrorValueLabel);
    layout->addRow("advance_wall_error_mm:", turnDebugAdvanceWallErrorValueLabel);
    layout->addRow("advance_wall_error_after_deadband_mm:", turnDebugAdvanceWallErrorAfterDeadbandValueLabel);
    layout->addRow("advance_wall_prev_error_mm:", turnDebugAdvanceWallPrevErrorValueLabel);
    layout->addRow("advance_wall_error_delta_mm:", turnDebugAdvanceWallErrorDeltaValueLabel);
    layout->addRow("advance_wall_p_term_pwm:", turnDebugAdvanceWallPTermValueLabel);
    layout->addRow("advance_wall_d_term_pwm:", turnDebugAdvanceWallDTermValueLabel);
    layout->addRow("advance_wall_raw_correction_pwm:", turnDebugAdvanceWallRawCorrectionValueLabel);
    layout->addRow("advance_wall_limited_correction_pwm:", turnDebugAdvanceWallLimitedCorrectionValueLabel);
    layout->addRow("advance_wall_correction_pwm:", turnDebugAdvanceWallCorrectionValueLabel);
    layout->addRow("diag_guidance_kp:", turnDebugDiagGuidanceKpValueLabel);
    layout->addRow("diag_guidance_kd:", turnDebugDiagGuidanceKdValueLabel);
    layout->addRow("diag_guidance_correction_limit_pwm:",
                   turnDebugDiagGuidanceLimitValueLabel);
    layout->addRow("diag_guidance_error_scale_num:",
                   turnDebugDiagGuidanceScaleNumValueLabel);
    layout->addRow("diag_guidance_error_scale_den:",
                   turnDebugDiagGuidanceScaleDenValueLabel);
    layout->addRow("diag_guidance_target_mm:", turnDebugDiagGuidanceTargetValueLabel);
    layout->addRow("diag_guidance_smooth_final_mode:",
                   turnDebugDiagGuidanceSmoothFinalModeValueLabel);
    layout->addRow("diag_guidance_p_term_pwm:", turnDebugDiagGuidancePTermValueLabel);
    layout->addRow("diag_guidance_d_term_pwm:", turnDebugDiagGuidanceDTermValueLabel);
    layout->addRow("diag_guidance_correction_pwm:", turnDebugDiagGuidanceCorrectionValueLabel);
    layout->addRow("wall_kp:", turnDebugWallKpValueLabel);
    layout->addRow("wall_kd:", turnDebugWallKdValueLabel);
    layout->addRow("wall_error_deadband_mm:", turnDebugWallDeadbandValueLabel);
    layout->addRow("wall_follow_target_left_mm:", turnDebugWallTargetLeftValueLabel);
    layout->addRow("wall_follow_target_right_mm:", turnDebugWallTargetRightValueLabel);
    layout->addRow("wall_correction_limit_pwm:", turnDebugWallCorrectionLimitValueLabel);
    layout->addRow("wall_single_side_error_scale:", turnDebugWallSingleSideErrorScaleValueLabel);
    layout->addRow("last_completed_action:", turnDebugLastCompletedActionValueLabel);
    layout->addRow("last_smooth_done_reason:", turnDebugLastSmoothDoneReasonValueLabel);
    layout->addRow("last_smooth_final_yaw:", turnDebugLastSmoothFinalYawValueLabel);
    layout->addRow("last_smooth_final_rear:", turnDebugLastSmoothFinalRearValueLabel);
    layout->addRow("last_advance_done_reason:", turnDebugLastAdvanceDoneReasonValueLabel);
    layout->addRow("last_advance_final_yaw:", turnDebugLastAdvanceFinalYawValueLabel);
    layout->addRow("last_advance_final_rear:", turnDebugLastAdvanceFinalRearValueLabel);
    layout->addRow("last_approach_front_done_reason:", turnDebugLastApproachFrontDoneReasonValueLabel);
    layout->addRow("last_center_pivot_done_reason:",
                   turnDebugLastCenterPivotDoneReasonValueLabel);

    layout->addRow(navCommandTitle);
    layout->addRow("left_motor_pwm:", navLeftMotorValueLabel);
    layout->addRow("right_motor_pwm:", navRightMotorValueLabel);

    layout->addRow(simulationTitle);
    layout->addRow("running:", simulationRunningValueLabel);
    layout->addRow("dt_ms:", simulationDtValueLabel);
    layout->addRow("auto_mode:", autoModeValueLabel);
    layout->addRow("step_count:", simulationStepCountValueLabel);
    layout->addRow("sim_time_s:", simulationTimeValueLabel);
    layout->addRow("motor_test:", motorTestModeValueLabel);
    layout->addRow("sim_left_motor_gain:", simLeftMotorGainValueLabel);
    layout->addRow("sim_right_motor_gain:", simRightMotorGainValueLabel);
    layout->addRow("sim_pivot_center_correction_enabled:", simPivotCenterCorrectionEnabledValueLabel);
    layout->addRow("sim_pivot_center_local_x_mm:", simPivotCenterLocalXValueLabel);
    layout->addRow("sim_pivot_center_local_y_mm:", simPivotCenterLocalYValueLabel);
    layout->addRow("sim_last_motion_was_pivot_like:", simLastMotionPivotLikeValueLabel);
    layout->addRow("floor_rear_global_x:", simFloorRearGlobalXValueLabel);
    layout->addRow("floor_rear_global_y:", simFloorRearGlobalYValueLabel);

    layout->addRow(sequenceTitle);
    layout->addRow("sequence_enabled:", sequenceEnabledValueLabel);
    layout->addRow("sequence_index:", sequenceIndexValueLabel);
    layout->addRow("sequence_length:", sequenceLengthValueLabel);
    layout->addRow("sequence_current_action:", sequenceCurrentActionValueLabel);
    layout->addRow("sequence_waiting_next_tick:", sequenceWaitingNextTickValueLabel);
    layout->addRow("center_pivot_sequence_active:", centerPivotSequenceActiveValueLabel);
    layout->addRow("center_pivot_sequence_phase:", centerPivotSequencePhaseValueLabel);
    layout->addRow("center_pivot_sequence_last_center_reason:",
                   centerPivotSequenceLastCenterReasonValueLabel);

    layout->addRow(planTitle);
    layout->addRow("plan_execution_enabled:", planExecutionEnabledValueLabel);
    layout->addRow("plan_queue_count:", planQueueCountValueLabel);
    layout->addRow("plan_current_action:", planCurrentActionValueLabel);
    layout->addRow("plan_next_action:", planNextActionValueLabel);
    layout->addRow("plan_last_executed_action:", planLastExecutedActionValueLabel);
    layout->addRow("plan_actions_executed_count:", planActionsExecutedCountValueLabel);
    layout->addRow("plan_queue_overflow:", planQueueOverflowValueLabel);
    layout->addRow("plan_composite_action_active:", planCompositeActionActiveValueLabel);
    layout->addRow("plan_composite_action_phase:", planCompositeActionPhaseValueLabel);
    layout->addRow("plan_composite_prepare_method:",
                   planCompositePrepareMethodValueLabel);
    layout->addRow("plan_composite_wall_front_at_start:",
                   planCompositeWallFrontAtStartValueLabel);
    layout->addRow("plan_composite_last_center_reason:",
                   planCompositeLastCenterReasonValueLabel);
    layout->addRow("plan_composite_last_approach_reason:",
                   planCompositeLastApproachReasonValueLabel);
    layout->addRow("plan_advance_after_center_and_pivot_180:",
                   planAdvanceAfterCenterPivotValueLabel);
    layout->addRow("plan_next_advance_from_centered_pose:",
                   planNextAdvanceFromCenteredPoseValueLabel);
    layout->addRow("route_status:", routeStatusValueLabel);
    layout->addRow("route_target_cell:", routeTargetCellValueLabel);
    layout->addRow("route_start_cell:", routeStartCellValueLabel);
    layout->addRow("route_start_dir:", routeStartDirValueLabel);
    layout->addRow("route_length:", routeLengthValueLabel);
    layout->addRow("route_expanded_states:", routeExpandedStatesValueLabel);
    layout->addRow("route_first_action:", routeFirstActionValueLabel);
    layout->addRow("route_last_action:", routeLastActionValueLabel);
    layout->addRow("route_loaded_into_plan_queue:", routeLoadedIntoPlanQueueValueLabel);
    layout->addRow("frontier_route_status:", frontierRouteStatusValueLabel);
    layout->addRow("frontier_target_cell:", frontierTargetCellValueLabel);
    layout->addRow("frontier_target_dir:", frontierTargetDirValueLabel);
    layout->addRow("frontier_exit_dir_absolute:", frontierExitDirAbsoluteValueLabel);
    layout->addRow("frontier_exit_relative:", frontierExitRelativeValueLabel);
    layout->addRow("frontier_neighbor_cell:", frontierNeighborCellValueLabel);
    layout->addRow("frontier_route_length:", frontierRouteLengthValueLabel);
    layout->addRow("frontier_expanded_states:", frontierExpandedStatesValueLabel);
    layout->addRow("frontier_loaded_into_plan_queue:", frontierLoadedIntoPlanQueueValueLabel);
    layout->addRow("frontier_count_found:", frontierCountFoundValueLabel);
    layout->addRow("route_execute_status:", routeExecuteStatusValueLabel);
    layout->addRow("route_start_physical_valid:", routeStartPhysicalValidValueLabel);
    layout->addRow("route_start_floor_rear_black:", routeStartFloorRearBlackValueLabel);
    layout->addRow("route_plan_ready_to_execute:", routePlanReadyToExecuteValueLabel);

    layout->addRow(navAutonomyTitle);
    layout->addRow("nav_autonomy_enabled:", navAutonomyEnabledValueLabel);
    layout->addRow("nav_policy:", navPolicyValueLabel);
    layout->addRow("nav_recommended_action:", navRecommendedActionValueLabel);
    layout->addRow("nav_last_decision:", navLastDecisionValueLabel);
    layout->addRow("nav_decision_wall_front:", navDecisionWallFrontValueLabel);
    layout->addRow("nav_decision_wall_left:", navDecisionWallLeftValueLabel);
    layout->addRow("nav_decision_wall_right:", navDecisionWallRightValueLabel);
    layout->addRow("nav_decision_point_valid:", navDecisionPointValidValueLabel);
    layout->addRow("dead_end_recovery_active:", deadEndRecoveryActiveValueLabel);
    layout->addRow("dead_end_recovery_phase:", deadEndRecoveryPhaseValueLabel);
    layout->addRow("dead_end_recovery_last_approach_reason:",
                   deadEndRecoveryLastApproachReasonValueLabel);
    layout->addRow("dead_end_recovery_pending_pivot:", deadEndRecoveryPendingPivotValueLabel);
    layout->addRow("nav_map_candidate_right_cell:", navMapCandidateRightCellValueLabel);
    layout->addRow("nav_map_candidate_front_cell:", navMapCandidateFrontCellValueLabel);
    layout->addRow("nav_map_candidate_left_cell:", navMapCandidateLeftCellValueLabel);
    layout->addRow("nav_map_candidate_right_visited:", navMapCandidateRightVisitedValueLabel);
    layout->addRow("nav_map_candidate_front_visited:", navMapCandidateFrontVisitedValueLabel);
    layout->addRow("nav_map_candidate_left_visited:", navMapCandidateLeftVisitedValueLabel);
    layout->addRow("nav_map_used_unvisited_preference:",
                   navMapUsedUnvisitedPreferenceValueLabel);
    layout->addRow("smart_recognition_state:", smartRecognitionStateValueLabel);
    layout->addRow("smart_last_frontier_status:", smartLastFrontierStatusValueLabel);
    layout->addRow("smart_frontier_plan_requested_count:",
                   smartFrontierPlanRequestedCountValueLabel);
    layout->addRow("smart_frontier_routes_executed_count:",
                   smartFrontierRoutesExecutedCountValueLabel);
    layout->addRow("smart_no_frontier_count:", smartNoFrontierCountValueLabel);
    layout->addRow("smart_local_action:", smartLocalActionValueLabel);
    layout->addRow("mode1_mission_enabled:", mode1MissionEnabledValueLabel);
    layout->addRow("mode1_mission_state:", mode1MissionStateValueLabel);
    layout->addRow("mode1_required_special_count:", mode1RequiredSpecialCountValueLabel);
    layout->addRow("mode1_found_special_count:", mode1FoundSpecialCountValueLabel);
    layout->addRow("mode1_required_specials_reached:",
                   mode1RequiredSpecialsReachedValueLabel);
    layout->addRow("mode1_return_requested:", mode1ReturnRequestedValueLabel);
    layout->addRow("mode1_waiting_action_done:", mode1WaitingActionDoneValueLabel);
    layout->addRow("mode1_search_complete_latched_at_count:",
                   mode1SearchCompleteLatchedAtCountValueLabel);
    layout->addRow("mode1_plan_cancelled_after_required_found:",
                   mode1PlanCancelledAfterRequiredFoundValueLabel);
    layout->addRow("mode1_nav_ready_for_return:", mode1NavReadyForReturnValueLabel);
    layout->addRow("mode1_plan_was_active_when_required_found:",
                   mode1PlanWasActiveWhenRequiredFoundValueLabel);
    layout->addRow("mode1_plan_queue_count_when_required_found:",
                   mode1PlanQueueCountWhenRequiredFoundValueLabel);
    layout->addRow("mode1_start_cell:", mode1StartCellValueLabel);
    layout->addRow("mode1_return_route_status:", mode1ReturnRouteStatusValueLabel);
    layout->addRow("mode1_return_plan_loaded:", mode1ReturnPlanLoadedValueLabel);
    layout->addRow("mode1_return_to_start_active:", mode1ReturnToStartActiveValueLabel);
    layout->addRow("mode1_at_start_cell:", mode1AtStartCellValueLabel);
    layout->addRow("mode1_done_reason:", mode1DoneReasonValueLabel);

    layout->addRow(mapTitle);
    layout->addRow("map_enabled:", mapEnabledValueLabel);
    layout->addRow("map_width:", mapWidthValueLabel);
    layout->addRow("map_height:", mapHeightValueLabel);
    layout->addRow("logical_cell_x:", mapCellXValueLabel);
    layout->addRow("logical_cell_y:", mapCellYValueLabel);
    layout->addRow("logical_dir:", mapDirValueLabel);
    layout->addRow("current_cell_visited:", mapCurrentCellVisitedValueLabel);
    layout->addRow("current_cell_special:", mapCurrentCellSpecialValueLabel);
    layout->addRow("current_cell_walls_known:", mapCurrentCellWallsKnownValueLabel);
    layout->addRow("current_cell_walls_present:", mapCurrentCellWallsPresentValueLabel);
    layout->addRow("map_last_pose_update_action:", mapLastPoseUpdateActionValueLabel);
    layout->addRow("map_last_wall_update_action:", mapLastWallUpdateActionValueLabel);
    layout->addRow("map_initial_wall_snapshot_pending:", mapInitialWallSnapshotPendingValueLabel);
    layout->addRow("map_update_count:", mapUpdateCountValueLabel);
    layout->addRow("map_wall_update_count:", mapWallUpdateCountValueLabel);
    layout->addRow("special_cells_found_count:", mapSpecialCellsFoundCountValueLabel);
    layout->addRow("last_special_cell:", mapLastSpecialCellValueLabel);
    layout->addRow("map_overlay_enabled:", mapOverlayEnabledValueLabel);

    layout->addRow(motorTestTitle);
    layout->addRow("test_left_pwm:", motorTestLeftValueLabel);
    layout->addRow("test_right_pwm:", motorTestRightValueLabel);

    auto *pinnedTitle = new QTreeWidgetItem();
    pinnedTitle->setText(0, "Pinned debug");
    QFont pinnedFont = pinnedTitle->font(0);
    pinnedFont.setBold(true);
    pinnedTitle->setFont(0, pinnedFont);
    pinnedTitle->setFirstColumnSpanned(true);
    pinnedTitle->setExpanded(true);

    auto addPinnedRow = [this, pinnedTitle](const QString &name, QLabel *sourceLabel) {
        auto *item = new QTreeWidgetItem(pinnedTitle);
        item->setText(0, compactTelemetryName(name));
        item->setToolTip(0, name);
        item->setText(1, sourceLabel ? sourceLabel->text() : QString());
        item->setToolTip(1, item->text(1));
        telemetryPinnedRows.push_back({item, sourceLabel});
    };

    addPinnedRow("special_candidate", turnDebugSpecialCandidateValueLabel);
    addPinnedRow("special_confirmed", turnDebugSpecialConfirmedValueLabel);
    addPinnedRow("special_mark_target_cell", turnDebugSpecialMarkTargetCellValueLabel);
    addPinnedRow("special_mark_target_source", turnDebugSpecialMarkTargetSourceValueLabel);
    addPinnedRow("last_special_mark_action", turnDebugLastSpecialMarkActionValueLabel);
    addPinnedRow("special_ignore_rear_until_white", turnDebugSpecialIgnoreRearValueLabel);
    addPinnedRow("special_detection_started_on_rear_line",
                 turnDebugSpecialStartedOnRearLineValueLabel);
    addPinnedRow("special_detection_enabled_for_current_motion",
                 turnDebugSpecialEnabledForMotionValueLabel);
    addPinnedRow("special_detection_context", turnDebugSpecialDetectionContextValueLabel);
    addPinnedRow("special_aux_detection_enabled", turnDebugSpecialAuxEnabledValueLabel);
    addPinnedRow("special_aux_started_after_rear_line_left",
                 turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel);
    addPinnedRow("initial_special_snapshot_pending",
                 turnDebugInitialSpecialSnapshotPendingValueLabel);
    addPinnedRow("initial_special_snapshot_done",
                 turnDebugInitialSpecialSnapshotDoneValueLabel);
    addPinnedRow("rear_line_trusted_for_decision", turnDebugRearLineTrustedValueLabel);
    addPinnedRow("rear_line_trust_source", turnDebugRearLineTrustSourceValueLabel);
    addPinnedRow("current_cell_special", mapCurrentCellSpecialValueLabel);
    addPinnedRow("special_cells_found_count", mapSpecialCellsFoundCountValueLabel);
    addPinnedRow("logical_cell_x", mapCellXValueLabel);
    addPinnedRow("logical_cell_y", mapCellYValueLabel);
    addPinnedRow("logical_dir", mapDirValueLabel);
    addPinnedRow("nav_action", navActionValueLabel);
    addPinnedRow("smooth_phase", turnDebugSmoothPhaseValueLabel);
    addPinnedRow("smooth_done_reason", turnDebugSmoothDoneReasonValueLabel);
    addPinnedRow("smooth_final_guidance_source", turnDebugSmoothFinalGuidanceSourceValueLabel);
    addPinnedRow("smooth_final_diag_mode", turnDebugSmoothFinalDiagModeValueLabel);
    addPinnedRow("smooth_final_diag_hold_initialized",
                 turnDebugSmoothFinalDiagHoldInitializedValueLabel);
    addPinnedRow("smooth_final_diag_error_mm", turnDebugSmoothFinalDiagErrorValueLabel);
    addPinnedRow("diag_guidance_correction_pwm", turnDebugDiagGuidanceCorrectionValueLabel);
    addPinnedRow("smooth_final_yaw_hold_deg", turnDebugSmoothFinalYawHoldValueLabel);
    addPinnedRow("smooth_final_yaw_error_deg", turnDebugSmoothFinalYawErrorValueLabel);
    addPinnedRow("smooth_final_applied_correction_pwm",
                 turnDebugSmoothFinalAppliedCorrectionValueLabel);
    addPinnedRow("smooth_yaw_carry_pending", turnDebugSmoothYawCarryPendingValueLabel);
    addPinnedRow("smooth_yaw_carry_used", turnDebugSmoothYawCarryUsedValueLabel);
    addPinnedRow("smooth_yaw_carry_offset_deg", turnDebugSmoothYawCarryOffsetValueLabel);
    addPinnedRow("smooth_yaw_carry_candidate_source",
                 turnDebugSmoothYawCarryCandidateSourceValueLabel);
    addPinnedRow("smooth_yaw_carry_rejected_reason",
                 turnDebugSmoothYawCarryRejectedReasonValueLabel);
    addPinnedRow("advance_final_correction_source", turnDebugAdvanceCorrectionSourceValueLabel);
    addPinnedRow("advance_front_diag_preview_armed",
                 turnDebugAdvanceFrontDiagPreviewArmedValueLabel);
    addPinnedRow("advance_front_diag_preview_latched",
                 turnDebugAdvanceFrontDiagPreviewLatchedValueLabel);
    addPinnedRow("advance_front_diag_preview_active",
                 turnDebugAdvanceFrontDiagPreviewActiveValueLabel);
    addPinnedRow("advance_front_diag_source", turnDebugAdvanceFrontDiagSourceValueLabel);
    addPinnedRow("wall_left_confidence", turnDebugWallLeftConfidenceValueLabel);
    addPinnedRow("wall_right_confidence", turnDebugWallRightConfidenceValueLabel);
    addPinnedRow("wall_caution_correction_pwm", turnDebugWallCautionCorrectionValueLabel);
    addPinnedRow("advance_front_diag_error_mm", turnDebugAdvanceFrontDiagErrorValueLabel);
    addPinnedRow("advance_yaw_hold_deg", turnDebugAdvanceYawHoldValueLabel);
    addPinnedRow("advance_yaw_hold_error_deg", turnDebugAdvanceYawHoldErrorValueLabel);
    addPinnedRow("plan_current_action", planCurrentActionValueLabel);
    addPinnedRow("plan_next_action", planNextActionValueLabel);
    addPinnedRow("smart_recognition_state", smartRecognitionStateValueLabel);
    addPinnedRow("mode1_mission_state", mode1MissionStateValueLabel);
    addPinnedRow("mode1_found_special_count", mode1FoundSpecialCountValueLabel);
    addPinnedRow("mode1_required_specials_reached",
                 mode1RequiredSpecialsReachedValueLabel);
    addPinnedRow("mode1_return_requested", mode1ReturnRequestedValueLabel);
    addPinnedRow("mode1_waiting_action_done", mode1WaitingActionDoneValueLabel);
    addPinnedRow("mode1_plan_cancelled_after_required_found",
                 mode1PlanCancelledAfterRequiredFoundValueLabel);
    addPinnedRow("mode1_plan_was_active_when_required_found",
                 mode1PlanWasActiveWhenRequiredFoundValueLabel);
    addPinnedRow("mode1_plan_queue_count_when_required_found",
                 mode1PlanQueueCountWhenRequiredFoundValueLabel);
    addPinnedRow("mode1_return_route_status", mode1ReturnRouteStatusValueLabel);
    addPinnedRow("mode1_done_reason", mode1DoneReasonValueLabel);
    tree->insertTopLevelItem(0, pinnedTitle);

    dock->setWidget(tree);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    updateTelemetryPanel();
}

void MainWindow::updateTelemetryPanel()
{
    if (xValueLabel) {
        xValueLabel->setText(QString("%1 mm").arg(robot.xMm(), 0, 'f', 1));
    }
    if (yValueLabel) {
        yValueLabel->setText(QString("%1 mm").arg(robot.yMm(), 0, 'f', 1));
    }
    if (yawValueLabel) {
        yawValueLabel->setText(QString("%1 deg").arg(robot.yawDeg(), 0, 'f', 1));
    }
    if (mazeNameValueLabel) {
        mazeNameValueLabel->setText(world.mazeName());
    }
    if (mazeFileValueLabel) {
        mazeFileValueLabel->setText(world.mazeFilePath().isEmpty()
                                        ? "built-in default"
                                        : world.mazeFilePath());
    }
    if (mazeWidthValueLabel) {
        mazeWidthValueLabel->setText(
            QString("%1 cells (%2 mm)").arg(world.cols()).arg(world.widthMm(), 0, 'f', 1));
    }
    if (mazeHeightValueLabel) {
        mazeHeightValueLabel->setText(
            QString("%1 cells (%2 mm)").arg(world.rows()).arg(world.heightMm(), 0, 'f', 1));
    }
    if (mazeCellSizeValueLabel) {
        mazeCellSizeValueLabel->setText(QString("%1 mm").arg(world.cellSizeMm(), 0, 'f', 1));
    }

    for (IrSensor &sensor : irSensors) {
        if (sensor.value_label) {
            sensor.value_label->setText(QString("%1 mm").arg(sensor.last_distance_mm, 0, 'f', 1));
        }
    }

    for (FloorSensor &sensor : floorSensors) {
        if (sensor.value_label) {
            sensor.value_label->setText(floorSensorText(sensor.is_black, sensor.debug_kind));
        }
    }

    const RobotSensors sensors = buildRobotSensorsSnapshot();
    NavWallPerception wallPerception = {};
    nav_core_get_wall_perception(&wallPerception);
    if (navWallFrontValueLabel) {
        navWallFrontValueLabel->setText(wallPerception.wall_front ? "true" : "false");
    }
    if (navWallLeftValueLabel) {
        navWallLeftValueLabel->setText(wallPerception.wall_left ? "true" : "false");
    }
    if (navWallRightValueLabel) {
        navWallRightValueLabel->setText(wallPerception.wall_right ? "true" : "false");
    }
    if (navWallDiagLeftValueLabel) {
        navWallDiagLeftValueLabel->setText(wallPerception.wall_diag_left ? "true" : "false");
    }
    if (navWallDiagRightValueLabel) {
        navWallDiagRightValueLabel->setText(wallPerception.wall_diag_right ? "true" : "false");
    }
    if (navWallFrontThresholdValueLabel) {
        navWallFrontThresholdValueLabel->setText(
            QString("%1 mm").arg(fromQ16(wallPerception.front_threshold_mm_q16), 0, 'f', 1));
    }
    if (navWallSideThresholdValueLabel) {
        navWallSideThresholdValueLabel->setText(
            QString("%1 mm").arg(fromQ16(wallPerception.side_threshold_mm_q16), 0, 'f', 1));
    }
    if (navWallDiagThresholdValueLabel) {
        navWallDiagThresholdValueLabel->setText(
            QString("%1 mm").arg(fromQ16(wallPerception.diag_threshold_mm_q16), 0, 'f', 1));
    }
    if (navFloorFrontValueLabel) {
        navFloorFrontValueLabel->setText(sensors.floor_front_black ? "true" : "false");
    }
    if (navFloorRearValueLabel) {
        navFloorRearValueLabel->setText(sensors.floor_rear_black ? "true" : "false");
    }
    if (navYawValueLabel) {
        navYawValueLabel->setText(QString("%1 deg").arg(fromQ16(sensors.yaw_deg_q16), 0, 'f', 1));
    }
    if (navYawRateValueLabel) {
        navYawRateValueLabel->setText(QString("%1 deg/s").arg(fromQ16(sensors.yaw_rate_deg_s_q16), 0, 'f', 1));
    }
    if (navYawZeroValueLabel) {
        navYawZeroValueLabel->setText(QString("%1 deg").arg(navYawZeroDeg, 0, 'f', 1));
    }
    if (navStateValueLabel) {
        navStateValueLabel->setText(navStateText(nav_core_state()));
    }
    if (navActionValueLabel) {
        navActionValueLabel->setText(navActionText(nav_core_action()));
    }
    if (navActionStartYawValueLabel) {
        navActionStartYawValueLabel->setText(
            QString("%1 deg").arg(fromQ16(nav_core_action_start_yaw_q16()), 0, 'f', 1));
    }
    if (navActionTargetYawValueLabel) {
        navActionTargetYawValueLabel->setText(
            QString("%1 deg").arg(fromQ16(nav_core_action_target_yaw_q16()), 0, 'f', 1));
    }

    NavTurnDebug turnDebug = {};
    nav_core_get_turn_debug(&turnDebug);
    NavSmoothTurnConfig smoothConfig = {};
    nav_core_get_smooth_turn_config(&smoothConfig);
    if (turnDebugSetpointValueLabel) {
        turnDebugSetpointValueLabel->setText(
            QString("%1 deg/s").arg(fromQ16(turnDebug.yaw_rate_setpoint_deg_s_q16), 0, 'f', 1));
    }
    if (turnDebugMeasuredValueLabel) {
        turnDebugMeasuredValueLabel->setText(
            QString("%1 deg/s").arg(fromQ16(turnDebug.yaw_rate_measured_deg_s_q16), 0, 'f', 1));
    }
    if (turnDebugErrorValueLabel) {
        turnDebugErrorValueLabel->setText(
            QString("%1 deg/s").arg(fromQ16(turnDebug.yaw_rate_error_deg_s_q16), 0, 'f', 1));
    }
    if (turnDebugOutputValueLabel) {
        turnDebugOutputValueLabel->setText(
            QString("%1 PWM").arg(fromQ16(turnDebug.pid_output_q16), 0, 'f', 1));
    }
    if (turnDebugCorrectionValueLabel) {
        turnDebugCorrectionValueLabel->setText(QString::number(turnDebug.correction_pwm));
    }
    if (turnDebugIntegralValueLabel) {
        turnDebugIntegralValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.integral_q16), 0, 'f', 3));
    }
    if (turnDebugKpValueLabel) {
        turnDebugKpValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.turn_kp_q16), 0, 'f', 4));
    }
    if (turnDebugKiValueLabel) {
        turnDebugKiValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.turn_ki_q16), 0, 'f', 4));
    }
    if (turnDebugKdValueLabel) {
        turnDebugKdValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.turn_kd_q16), 0, 'f', 4));
    }
    if (turnDebugOutputLimitValueLabel) {
        turnDebugOutputLimitValueLabel->setText(QString::number(turnDebug.turn_output_limit_pwm));
    }
    if (turnDebugSmoothTargetValueLabel) {
        turnDebugSmoothTargetValueLabel->setText(
            QString("%1 deg/s").arg(smoothConfig.target_yaw_rate_deg_s));
    }
    if (turnDebugSmoothRightLeftBaseValueLabel) {
        turnDebugSmoothRightLeftBaseValueLabel->setText(
            QString::number(smoothConfig.right_left_base_pwm));
    }
    if (turnDebugSmoothRightRightBaseValueLabel) {
        turnDebugSmoothRightRightBaseValueLabel->setText(
            QString::number(smoothConfig.right_right_base_pwm));
    }
    if (turnDebugSmoothLeftLeftBaseValueLabel) {
        turnDebugSmoothLeftLeftBaseValueLabel->setText(
            QString::number(smoothConfig.left_left_base_pwm));
    }
    if (turnDebugSmoothLeftRightBaseValueLabel) {
        turnDebugSmoothLeftRightBaseValueLabel->setText(
            QString::number(smoothConfig.left_right_base_pwm));
    }
    if (turnDebugSmoothLeftBaseValueLabel) {
        turnDebugSmoothLeftBaseValueLabel->setText(QString::number(turnDebug.smooth_left_base_pwm));
    }
    if (turnDebugSmoothRightBaseValueLabel) {
        turnDebugSmoothRightBaseValueLabel->setText(QString::number(turnDebug.smooth_right_base_pwm));
    }
    if (turnDebugSmoothPhaseValueLabel) {
        turnDebugSmoothPhaseValueLabel->setText(smoothPhaseText(turnDebug.smooth_phase));
    }
    if (turnDebugSmoothDoneReasonValueLabel) {
        turnDebugSmoothDoneReasonValueLabel->setText(
            smoothDoneReasonText(turnDebug.smooth_done_reason));
    }
    if (turnDebugSmoothPostYawElapsedValueLabel) {
        turnDebugSmoothPostYawElapsedValueLabel->setText(
            QString::number(turnDebug.smooth_post_yaw_elapsed_ms));
    }
    if (turnDebugSmoothFinalGuidanceSourceValueLabel) {
        turnDebugSmoothFinalGuidanceSourceValueLabel->setText(
            smoothFinalGuidanceSourceText(turnDebug.smooth_final_guidance_source));
    }
    if (turnDebugSmoothFinalHoldInitializedValueLabel) {
        turnDebugSmoothFinalHoldInitializedValueLabel->setText(
            turnDebug.smooth_final_hold_initialized ? "true" : "false");
    }
    if (turnDebugSmoothFinalHoldSourceValueLabel) {
        turnDebugSmoothFinalHoldSourceValueLabel->setText(
            smoothFinalGuidanceSourceText(turnDebug.smooth_final_hold_source));
    }
    if (turnDebugSmoothFinalHoldRecaptureCountValueLabel) {
        turnDebugSmoothFinalHoldRecaptureCountValueLabel->setText(
            QString::number(turnDebug.smooth_final_hold_recapture_count));
    }
    if (turnDebugSmoothFinalLeftHoldValueLabel) {
        turnDebugSmoothFinalLeftHoldValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_left_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalRightHoldValueLabel) {
        turnDebugSmoothFinalRightHoldValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_right_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalCenterDiffHoldValueLabel) {
        turnDebugSmoothFinalCenterDiffHoldValueLabel->setText(
            QString("%1 mm")
                .arg(fromQ16(turnDebug.smooth_final_center_diff_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalWallErrorValueLabel) {
        turnDebugSmoothFinalWallErrorValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_wall_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalWallCorrectionValueLabel) {
        turnDebugSmoothFinalWallCorrectionValueLabel->setText(
            QString::number(turnDebug.smooth_final_wall_correction_pwm));
    }
    if (turnDebugSmoothFinalYawCorrectionValueLabel) {
        turnDebugSmoothFinalYawCorrectionValueLabel->setText(
            QString::number(turnDebug.smooth_final_yaw_correction_pwm));
    }
    if (turnDebugSmoothFinalYawHoldValueLabel) {
        turnDebugSmoothFinalYawHoldValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_final_yaw_hold_deg_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalYawHoldInitializedValueLabel) {
        turnDebugSmoothFinalYawHoldInitializedValueLabel->setText(
            turnDebug.smooth_final_yaw_hold_initialized ? "true" : "false");
    }
    if (turnDebugSmoothFinalYawHoldRecaptureCountValueLabel) {
        turnDebugSmoothFinalYawHoldRecaptureCountValueLabel->setText(
            QString::number(turnDebug.smooth_final_yaw_hold_recapture_count));
    }
    if (turnDebugSmoothFinalYawErrorValueLabel) {
        turnDebugSmoothFinalYawErrorValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_final_yaw_error_deg_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalAppliedCorrectionValueLabel) {
        turnDebugSmoothFinalAppliedCorrectionValueLabel->setText(
            QString::number(turnDebug.smooth_final_applied_correction_pwm));
    }
    if (turnDebugSmoothFinalDiagLeftValidValueLabel) {
        turnDebugSmoothFinalDiagLeftValidValueLabel->setText(
            turnDebug.smooth_final_diag_left_valid ? "true" : "false");
    }
    if (turnDebugSmoothFinalDiagRightValidValueLabel) {
        turnDebugSmoothFinalDiagRightValidValueLabel->setText(
            turnDebug.smooth_final_diag_right_valid ? "true" : "false");
    }
    if (turnDebugSmoothFinalDiagLeftValueLabel) {
        turnDebugSmoothFinalDiagLeftValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_diag_left_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagRightValueLabel) {
        turnDebugSmoothFinalDiagRightValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_diag_right_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagTargetValueLabel) {
        turnDebugSmoothFinalDiagTargetValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_diag_target_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagErrorScaleValueLabel) {
        turnDebugSmoothFinalDiagErrorScaleValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.smooth_final_diag_error_scale_q16), 0, 'f', 2));
    }
    if (turnDebugSmoothFinalDiagModeValueLabel) {
        turnDebugSmoothFinalDiagModeValueLabel->setText(
            smoothFinalDiagonalModeText(turnDebug.smooth_final_diag_mode));
    }
    if (turnDebugSmoothFinalDiagHoldInitializedValueLabel) {
        turnDebugSmoothFinalDiagHoldInitializedValueLabel->setText(
            turnDebug.smooth_final_diag_hold_initialized ? "true" : "false");
    }
    if (turnDebugSmoothFinalDiagLeftHoldValueLabel) {
        turnDebugSmoothFinalDiagLeftHoldValueLabel->setText(
            QString("%1 mm")
                .arg(fromQ16(turnDebug.smooth_final_diag_left_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagRightHoldValueLabel) {
        turnDebugSmoothFinalDiagRightHoldValueLabel->setText(
            QString("%1 mm")
                .arg(fromQ16(turnDebug.smooth_final_diag_right_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagCenterDiffHoldValueLabel) {
        turnDebugSmoothFinalDiagCenterDiffHoldValueLabel->setText(
            QString("%1 mm")
                .arg(fromQ16(turnDebug.smooth_final_diag_center_diff_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagHoldRecaptureCountValueLabel) {
        turnDebugSmoothFinalDiagHoldRecaptureCountValueLabel->setText(
            QString::number(turnDebug.smooth_final_diag_hold_recapture_count));
    }
    if (turnDebugSmoothFinalDiagRawErrorValueLabel) {
        turnDebugSmoothFinalDiagRawErrorValueLabel->setText(
            QString("%1 mm")
                .arg(fromQ16(turnDebug.smooth_final_diag_raw_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalDiagErrorValueLabel) {
        turnDebugSmoothFinalDiagErrorValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.smooth_final_diag_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugSmoothFinalFollowLeftValidValueLabel) {
        turnDebugSmoothFinalFollowLeftValidValueLabel->setText(
            turnDebug.smooth_final_follow_left_valid ? "true" : "false");
    }
    if (turnDebugSmoothFinalFollowRightValidValueLabel) {
        turnDebugSmoothFinalFollowRightValidValueLabel->setText(
            turnDebug.smooth_final_follow_right_valid ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryEnabledValueLabel) {
        turnDebugSmoothYawCarryEnabledValueLabel->setText(
            turnDebug.smooth_yaw_carry_enabled ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryPendingValueLabel) {
        turnDebugSmoothYawCarryPendingValueLabel->setText(
            turnDebug.smooth_yaw_carry_pending ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryUsedValueLabel) {
        turnDebugSmoothYawCarryUsedValueLabel->setText(
            turnDebug.smooth_yaw_carry_used ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryOffsetValueLabel) {
        turnDebugSmoothYawCarryOffsetValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_yaw_carry_offset_deg_q16),
                                  0,
                                  'f',
                                  1));
    }
    if (turnDebugSmoothYawCarryEntryYawValueLabel) {
        turnDebugSmoothYawCarryEntryYawValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_yaw_carry_entry_yaw_deg_q16),
                                  0,
                                  'f',
                                  1));
    }
    if (turnDebugSmoothYawCarryExitYawValueLabel) {
        turnDebugSmoothYawCarryExitYawValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_yaw_carry_exit_yaw_deg_q16),
                                  0,
                                  'f',
                                  1));
    }
    if (turnDebugSmoothYawCarryDiagUsedValueLabel) {
        turnDebugSmoothYawCarryDiagUsedValueLabel->setText(
            turnDebug.smooth_yaw_carry_diag_used ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryCandidateSourceValueLabel) {
        turnDebugSmoothYawCarryCandidateSourceValueLabel->setText(
            yawCarryCandidateSourceText(turnDebug.smooth_yaw_carry_candidate_source));
    }
    if (turnDebugSmoothYawCarryRejectedReasonValueLabel) {
        turnDebugSmoothYawCarryRejectedReasonValueLabel->setText(
            smoothYawCarryRejectedReasonText(turnDebug.smooth_yaw_carry_rejected_reason));
    }
    if (turnDebugSmoothYawCarryOnlySetpointValueLabel) {
        turnDebugSmoothYawCarryOnlySetpointValueLabel->setText(
            turnDebug.smooth_yaw_carry_only_setpoint ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryRequireDiagValueLabel) {
        turnDebugSmoothYawCarryRequireDiagValueLabel->setText(
            turnDebug.smooth_yaw_carry_require_diag ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryAllowAdvancePreviewValueLabel) {
        turnDebugSmoothYawCarryAllowAdvancePreviewValueLabel->setText(
            turnDebug.smooth_yaw_carry_allow_advance_preview ? "true" : "false");
    }
    if (turnDebugSmoothYawCarryMinAbsValueLabel) {
        turnDebugSmoothYawCarryMinAbsValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_yaw_carry_min_abs_deg_q16),
                                  0,
                                  'f',
                                  1));
    }
    if (turnDebugSmoothYawCarryMaxAbsValueLabel) {
        turnDebugSmoothYawCarryMaxAbsValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.smooth_yaw_carry_max_abs_deg_q16),
                                  0,
                                  'f',
                                  1));
    }
    if (turnDebugSmoothYawCarryScaleValueLabel) {
        turnDebugSmoothYawCarryScaleValueLabel->setText(
            QString("%1/%2")
                .arg(turnDebug.smooth_yaw_carry_offset_scale_num)
                .arg(turnDebug.smooth_yaw_carry_offset_scale_den));
    }
    if (turnDebugAdvancePhaseValueLabel) {
        turnDebugAdvancePhaseValueLabel->setText(advancePhaseText(turnDebug.advance_phase));
    }
    if (turnDebugAdvanceDoneReasonValueLabel) {
        turnDebugAdvanceDoneReasonValueLabel->setText(
            advanceDoneReasonText(turnDebug.advance_done_reason));
    }
    if (turnDebugRearBlackForLineValueLabel) {
        turnDebugRearBlackForLineValueLabel->setText(
            turnDebug.rear_black_for_line ? "true" : "false");
    }
    if (turnDebugFloorRearRealValueLabel) {
        turnDebugFloorRearRealValueLabel->setText(
            turnDebug.floor_rear_black ? "true" : "false");
    }
    if (turnDebugAdvanceStartedOnRearLineValueLabel) {
        turnDebugAdvanceStartedOnRearLineValueLabel->setText(
            turnDebug.advance_started_on_rear_line ? "true" : "false");
    }
    if (turnDebugAdvanceStartModeValueLabel) {
        turnDebugAdvanceStartModeValueLabel->setText(
            advanceStartModeText(turnDebug.advance_start_mode));
    }
    if (turnDebugAdvanceCenteredWaitingRearWhiteValueLabel) {
        turnDebugAdvanceCenteredWaitingRearWhiteValueLabel->setText(
            turnDebug.advance_from_centered_waiting_rear_white ? "true" : "false");
    }
    if (turnDebugSpecialCandidateValueLabel) {
        turnDebugSpecialCandidateValueLabel->setText(turnDebug.special_candidate ? "true" : "false");
    }
    if (turnDebugSpecialConfirmedValueLabel) {
        turnDebugSpecialConfirmedValueLabel->setText(turnDebug.special_confirmed ? "true" : "false");
    }
    if (turnDebugSpecialIgnoreRearValueLabel) {
        turnDebugSpecialIgnoreRearValueLabel->setText(
            turnDebug.special_ignore_rear_until_white ? "true" : "false");
    }
    if (turnDebugSpecialStartedOnRearLineValueLabel) {
        turnDebugSpecialStartedOnRearLineValueLabel->setText(
            turnDebug.special_detection_started_on_rear_line ? "true" : "false");
    }
    if (turnDebugSpecialEnabledForMotionValueLabel) {
        turnDebugSpecialEnabledForMotionValueLabel->setText(
            turnDebug.special_detection_enabled_for_current_motion ? "true" : "false");
    }
    if (turnDebugSpecialDetectionContextValueLabel) {
        turnDebugSpecialDetectionContextValueLabel->setText(
            specialDetectionContextText(turnDebug.special_detection_context));
    }
    if (turnDebugSpecialAuxEnabledValueLabel) {
        turnDebugSpecialAuxEnabledValueLabel->setText(
            turnDebug.special_aux_detection_enabled ? "true" : "false");
    }
    if (turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel) {
        turnDebugSpecialAuxStartedAfterRearLineLeftValueLabel->setText(
            turnDebug.special_aux_started_after_rear_line_left ? "true" : "false");
    }
    if (turnDebugInitialSpecialSnapshotPendingValueLabel) {
        turnDebugInitialSpecialSnapshotPendingValueLabel->setText(
            turnDebug.initial_special_snapshot_pending ? "true" : "false");
    }
    if (turnDebugInitialSpecialSnapshotDoneValueLabel) {
        turnDebugInitialSpecialSnapshotDoneValueLabel->setText(
            turnDebug.initial_special_snapshot_done ? "true" : "false");
    }
    if (turnDebugRearLineTrustedValueLabel) {
        turnDebugRearLineTrustedValueLabel->setText(
            turnDebug.rear_line_trusted_for_decision ? "true" : "false");
    }
    if (turnDebugRearLineTrustSourceValueLabel) {
        turnDebugRearLineTrustSourceValueLabel->setText(
            rearLineTrustSourceText(turnDebug.rear_line_trust_source));
    }
    if (turnDebugSpecialMarkTargetCellValueLabel) {
        if (turnDebug.special_mark_target_cell_x >= 0
            && turnDebug.special_mark_target_cell_y >= 0) {
            turnDebugSpecialMarkTargetCellValueLabel->setText(
                QString("(%1,%2)")
                    .arg(turnDebug.special_mark_target_cell_x)
                    .arg(turnDebug.special_mark_target_cell_y));
        } else {
            turnDebugSpecialMarkTargetCellValueLabel->setText("(-,-)");
        }
    }
    if (turnDebugSpecialMarkTargetSourceValueLabel) {
        turnDebugSpecialMarkTargetSourceValueLabel->setText(
            specialMarkTargetSourceText(turnDebug.special_mark_target_source));
    }
    if (turnDebugLastSpecialMarkActionValueLabel) {
        turnDebugLastSpecialMarkActionValueLabel->setText(
            navActionText(turnDebug.last_special_mark_action));
    }
    if (turnDebugAdvanceElapsedSinceLeaveValueLabel) {
        turnDebugAdvanceElapsedSinceLeaveValueLabel->setText(
            QString::number(turnDebug.advance_elapsed_since_leave_start_line_ms));
    }
    if (turnDebugSpecialDetectMinValueLabel) {
        turnDebugSpecialDetectMinValueLabel->setText(QString::number(turnDebug.special_detect_min_ms));
    }
    if (turnDebugSpecialDetectMaxValueLabel) {
        turnDebugSpecialDetectMaxValueLabel->setText(QString::number(turnDebug.special_detect_max_ms));
    }
    if (turnDebugApproachFrontPhaseValueLabel) {
        turnDebugApproachFrontPhaseValueLabel->setText(
            approachFrontPhaseText(turnDebug.approach_front_phase));
    }
    if (turnDebugApproachFrontDoneReasonValueLabel) {
        turnDebugApproachFrontDoneReasonValueLabel->setText(
            approachFrontDoneReasonText(turnDebug.approach_front_done_reason));
    }
    if (turnDebugApproachFrontTargetValueLabel) {
        turnDebugApproachFrontTargetValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.approach_front_target_mm_q16), 0, 'f', 1));
    }
    if (turnDebugApproachFrontLeftValueLabel) {
        turnDebugApproachFrontLeftValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.approach_front_left_mm_q16), 0, 'f', 1));
    }
    if (turnDebugApproachFrontRightValueLabel) {
        turnDebugApproachFrontRightValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.approach_front_right_mm_q16), 0, 'f', 1));
    }
    if (turnDebugApproachFrontElapsedValueLabel) {
        turnDebugApproachFrontElapsedValueLabel->setText(
            QString::number(turnDebug.approach_front_elapsed_ms));
    }
    if (turnDebugApproachFrontBrakeElapsedValueLabel) {
        turnDebugApproachFrontBrakeElapsedValueLabel->setText(
            QString::number(turnDebug.approach_front_brake_elapsed_ms));
    }
    if (turnDebugApproachFrontBaseLeftValueLabel) {
        turnDebugApproachFrontBaseLeftValueLabel->setText(
            QString::number(turnDebug.approach_front_base_left_pwm));
    }
    if (turnDebugApproachFrontBaseRightValueLabel) {
        turnDebugApproachFrontBaseRightValueLabel->setText(
            QString::number(turnDebug.approach_front_base_right_pwm));
    }
    if (turnDebugApproachFrontCorrectionValueLabel) {
        turnDebugApproachFrontCorrectionValueLabel->setText(
            QString::number(turnDebug.approach_front_correction_pwm));
    }
    if (turnDebugCenterPivotPhaseValueLabel) {
        turnDebugCenterPivotPhaseValueLabel->setText(
            centerPivotPhaseText(turnDebug.center_pivot_phase));
    }
    if (turnDebugCenterPivotDoneReasonValueLabel) {
        turnDebugCenterPivotDoneReasonValueLabel->setText(
            centerPivotDoneReasonText(turnDebug.center_pivot_done_reason));
    }
    if (turnDebugCenterPivotElapsedValueLabel) {
        turnDebugCenterPivotElapsedValueLabel->setText(
            QString::number(turnDebug.center_pivot_elapsed_ms));
    }
    if (turnDebugCenterPivotBrakeElapsedValueLabel) {
        turnDebugCenterPivotBrakeElapsedValueLabel->setText(
            QString::number(turnDebug.center_pivot_brake_elapsed_ms));
    }
    if (turnDebugCenterPivotBaseLeftValueLabel) {
        turnDebugCenterPivotBaseLeftValueLabel->setText(
            QString::number(turnDebug.center_pivot_base_left_pwm));
    }
    if (turnDebugCenterPivotBaseRightValueLabel) {
        turnDebugCenterPivotBaseRightValueLabel->setText(
            QString::number(turnDebug.center_pivot_base_right_pwm));
    }
    if (turnDebugCenterPivotCorrectionValueLabel) {
        turnDebugCenterPivotCorrectionValueLabel->setText(
            QString::number(turnDebug.center_pivot_correction_pwm));
    }
    if (turnDebugCenterPivotFrontBlackValueLabel) {
        turnDebugCenterPivotFrontBlackValueLabel->setText(
            turnDebug.center_pivot_front_black ? "true" : "false");
    }
    if (turnDebugCenterPivotRearBlackValueLabel) {
        turnDebugCenterPivotRearBlackValueLabel->setText(
            turnDebug.center_pivot_rear_black ? "true" : "false");
    }
    if (turnDebugCenterPivotFrontSeenWhiteValueLabel) {
        turnDebugCenterPivotFrontSeenWhiteValueLabel->setText(
            turnDebug.center_pivot_front_seen_white ? "true" : "false");
    }
    if (turnDebugAdvanceYawSetpointValueLabel) {
        turnDebugAdvanceYawSetpointValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.advance_yaw_setpoint_deg_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceYawMeasuredValueLabel) {
        turnDebugAdvanceYawMeasuredValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.advance_yaw_measured_deg_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceYawErrorValueLabel) {
        turnDebugAdvanceYawErrorValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.advance_yaw_error_deg_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceYawOutputValueLabel) {
        turnDebugAdvanceYawOutputValueLabel->setText(
            QString("%1 PWM").arg(fromQ16(turnDebug.advance_yaw_pid_output_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceYawCorrectionValueLabel) {
        turnDebugAdvanceYawCorrectionValueLabel->setText(
            QString::number(turnDebug.advance_yaw_correction_pwm));
    }
    if (turnDebugAdvanceYawHoldValueLabel) {
        turnDebugAdvanceYawHoldValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.advance_yaw_hold_deg_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceYawHoldInitializedValueLabel) {
        turnDebugAdvanceYawHoldInitializedValueLabel->setText(
            turnDebug.advance_yaw_hold_initialized ? "true" : "false");
    }
    if (turnDebugAdvanceYawHoldRecaptureCountValueLabel) {
        turnDebugAdvanceYawHoldRecaptureCountValueLabel->setText(
            QString::number(turnDebug.advance_yaw_hold_recapture_count));
    }
    if (turnDebugAdvanceYawHoldErrorValueLabel) {
        turnDebugAdvanceYawHoldErrorValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.advance_yaw_hold_error_deg_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceGuidanceLastSourceValueLabel) {
        turnDebugAdvanceGuidanceLastSourceValueLabel->setText(
            advanceCorrectionSourceText(turnDebug.advance_guidance_last_source));
    }
    if (turnDebugAdvanceYawKpValueLabel) {
        turnDebugAdvanceYawKpValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.advance_yaw_kp_q16), 0, 'f', 4));
    }
    if (turnDebugAdvanceYawKiValueLabel) {
        turnDebugAdvanceYawKiValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.advance_yaw_ki_q16), 0, 'f', 4));
    }
    if (turnDebugAdvanceYawKdValueLabel) {
        turnDebugAdvanceYawKdValueLabel->setText(
            QString("%1").arg(fromQ16(turnDebug.advance_yaw_kd_q16), 0, 'f', 4));
    }
    if (turnDebugAdvanceYawOutputLimitValueLabel) {
        turnDebugAdvanceYawOutputLimitValueLabel->setText(
            QString::number(turnDebug.advance_yaw_output_limit_pwm));
    }
    if (turnDebugAdvanceBaseLeftValueLabel) {
        turnDebugAdvanceBaseLeftValueLabel->setText(
            QString::number(turnDebug.advance_base_left_pwm));
    }
    if (turnDebugAdvanceBaseRightValueLabel) {
        turnDebugAdvanceBaseRightValueLabel->setText(
            QString::number(turnDebug.advance_base_right_pwm));
    }
    if (turnDebugAdvanceGuidanceModeValueLabel) {
        turnDebugAdvanceGuidanceModeValueLabel->setText(
            advanceGuidanceModeText(nav_core_get_advance_guidance_mode()));
    }
    if (turnDebugAdvanceCorrectionSourceValueLabel) {
        turnDebugAdvanceCorrectionSourceValueLabel->setText(
            advanceCorrectionSourceText(turnDebug.advance_final_correction_source));
    }
    if (turnDebugAdvanceFrontDiagPreviewArmedValueLabel) {
        turnDebugAdvanceFrontDiagPreviewArmedValueLabel->setText(
            turnDebug.advance_front_diag_preview_armed ? "true" : "false");
    }
    if (turnDebugAdvanceFrontDiagPreviewLatchedValueLabel) {
        turnDebugAdvanceFrontDiagPreviewLatchedValueLabel->setText(
            turnDebug.advance_front_diag_preview_latched ? "true" : "false");
    }
    if (turnDebugAdvanceFrontDiagPreviewActiveValueLabel) {
        turnDebugAdvanceFrontDiagPreviewActiveValueLabel->setText(
            turnDebug.advance_front_diag_preview_active ? "true" : "false");
    }
    if (turnDebugAdvanceFrontDiagSourceValueLabel) {
        turnDebugAdvanceFrontDiagSourceValueLabel->setText(
            advanceFrontDiagSourceText(turnDebug.advance_front_diag_source));
    }
    if (turnDebugAdvanceFrontDiagRawErrorValueLabel) {
        turnDebugAdvanceFrontDiagRawErrorValueLabel->setText(
            QString("%1 mm")
                .arg(fromQ16(turnDebug.advance_front_diag_raw_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceFrontDiagErrorValueLabel) {
        turnDebugAdvanceFrontDiagErrorValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_front_diag_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceFrontDiagLeftValidValueLabel) {
        turnDebugAdvanceFrontDiagLeftValidValueLabel->setText(
            turnDebug.advance_front_diag_left_valid ? "true" : "false");
    }
    if (turnDebugAdvanceFrontDiagRightValidValueLabel) {
        turnDebugAdvanceFrontDiagRightValidValueLabel->setText(
            turnDebug.advance_front_diag_right_valid ? "true" : "false");
    }
    if (turnDebugAdvanceWallLeftValidValueLabel) {
        turnDebugAdvanceWallLeftValidValueLabel->setText(
            turnDebug.advance_wall_left_valid ? "true" : "false");
    }
    if (turnDebugAdvanceWallRightValidValueLabel) {
        turnDebugAdvanceWallRightValidValueLabel->setText(
            turnDebug.advance_wall_right_valid ? "true" : "false");
    }
    if (turnDebugAdvanceDiagLeftValidValueLabel) {
        turnDebugAdvanceDiagLeftValidValueLabel->setText(
            turnDebug.advance_diag_left_valid ? "true" : "false");
    }
    if (turnDebugAdvanceDiagRightValidValueLabel) {
        turnDebugAdvanceDiagRightValidValueLabel->setText(
            turnDebug.advance_diag_right_valid ? "true" : "false");
    }
    if (turnDebugAdvanceFollowLeftValidValueLabel) {
        turnDebugAdvanceFollowLeftValidValueLabel->setText(
            turnDebug.advance_follow_left_valid ? "true" : "false");
    }
    if (turnDebugAdvanceFollowRightValidValueLabel) {
        turnDebugAdvanceFollowRightValidValueLabel->setText(
            turnDebug.advance_follow_right_valid ? "true" : "false");
    }
    if (turnDebugWallCautionEnabledValueLabel) {
        turnDebugWallCautionEnabledValueLabel->setText(
            turnDebug.wall_caution_enabled ? "true" : "false");
    }
    if (turnDebugWallLeftConfidenceValueLabel) {
        turnDebugWallLeftConfidenceValueLabel->setText(
            wallCautionConfidenceText(turnDebug.wall_left_confidence));
    }
    if (turnDebugWallRightConfidenceValueLabel) {
        turnDebugWallRightConfidenceValueLabel->setText(
            wallCautionConfidenceText(turnDebug.wall_right_confidence));
    }
    if (turnDebugWallLeftCautionElapsedValueLabel) {
        turnDebugWallLeftCautionElapsedValueLabel->setText(
            QString("%1 ms").arg(turnDebug.wall_left_caution_elapsed_ms));
    }
    if (turnDebugWallRightCautionElapsedValueLabel) {
        turnDebugWallRightCautionElapsedValueLabel->setText(
            QString("%1 ms").arg(turnDebug.wall_right_caution_elapsed_ms));
    }
    if (turnDebugWallLeftCautionHoldValueLabel) {
        turnDebugWallLeftCautionHoldValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_left_caution_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallRightCautionHoldValueLabel) {
        turnDebugWallRightCautionHoldValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_right_caution_hold_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallLeftCautionDeltaValueLabel) {
        turnDebugWallLeftCautionDeltaValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_left_caution_delta_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallRightCautionDeltaValueLabel) {
        turnDebugWallRightCautionDeltaValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_right_caution_delta_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallCautionCorrectionValueLabel) {
        turnDebugWallCautionCorrectionValueLabel->setText(
            QString::number(turnDebug.wall_caution_correction_pwm));
    }
    if (turnDebugWallCautionLossReasonValueLabel) {
        turnDebugWallCautionLossReasonValueLabel->setText(
            wallCautionLossReasonText(turnDebug.wall_caution_loss_reason));
    }
    if (turnDebugAdvanceWallLeftValueLabel) {
        turnDebugAdvanceWallLeftValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_left_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallRightValueLabel) {
        turnDebugAdvanceWallRightValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_right_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallRawErrorValueLabel) {
        turnDebugAdvanceWallRawErrorValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_raw_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallErrorValueLabel) {
        turnDebugAdvanceWallErrorValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallErrorAfterDeadbandValueLabel) {
        turnDebugAdvanceWallErrorAfterDeadbandValueLabel->setText(
            QString("%1 mm").arg(
                fromQ16(turnDebug.advance_wall_error_after_deadband_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallPrevErrorValueLabel) {
        turnDebugAdvanceWallPrevErrorValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_prev_error_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallErrorDeltaValueLabel) {
        turnDebugAdvanceWallErrorDeltaValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_error_delta_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallPTermValueLabel) {
        turnDebugAdvanceWallPTermValueLabel->setText(
            QString::number(turnDebug.advance_wall_p_term_pwm));
    }
    if (turnDebugAdvanceWallDTermValueLabel) {
        turnDebugAdvanceWallDTermValueLabel->setText(
            QString::number(turnDebug.advance_wall_d_term_pwm));
    }
    if (turnDebugAdvanceWallRawCorrectionValueLabel) {
        turnDebugAdvanceWallRawCorrectionValueLabel->setText(
            QString::number(turnDebug.advance_wall_raw_correction_pwm));
    }
    if (turnDebugAdvanceWallLimitedCorrectionValueLabel) {
        turnDebugAdvanceWallLimitedCorrectionValueLabel->setText(
            QString::number(turnDebug.advance_wall_limited_correction_pwm));
    }
    if (turnDebugAdvanceWallCorrectionValueLabel) {
        turnDebugAdvanceWallCorrectionValueLabel->setText(
            QString::number(turnDebug.advance_wall_correction_pwm));
    }
    if (turnDebugDiagGuidanceKpValueLabel) {
        turnDebugDiagGuidanceKpValueLabel->setText(
            QString::number(turnDebug.diag_guidance_kp_pwm_per_mm));
    }
    if (turnDebugDiagGuidanceKdValueLabel) {
        turnDebugDiagGuidanceKdValueLabel->setText(
            QString::number(turnDebug.diag_guidance_kd_pwm_per_mm_per_tick));
    }
    if (turnDebugDiagGuidanceLimitValueLabel) {
        turnDebugDiagGuidanceLimitValueLabel->setText(
            QString::number(turnDebug.diag_guidance_correction_limit_pwm));
    }
    if (turnDebugDiagGuidanceScaleNumValueLabel) {
        turnDebugDiagGuidanceScaleNumValueLabel->setText(
            QString::number(turnDebug.diag_guidance_error_scale_num));
    }
    if (turnDebugDiagGuidanceScaleDenValueLabel) {
        turnDebugDiagGuidanceScaleDenValueLabel->setText(
            QString::number(turnDebug.diag_guidance_error_scale_den));
    }
    if (turnDebugDiagGuidanceTargetValueLabel) {
        turnDebugDiagGuidanceTargetValueLabel->setText(
            QString("%1 mm").arg(turnDebug.diag_guidance_target_mm));
    }
    if (turnDebugDiagGuidanceSmoothFinalModeValueLabel) {
        turnDebugDiagGuidanceSmoothFinalModeValueLabel->setText(
            smoothFinalDiagonalModeText(turnDebug.diag_guidance_smooth_final_mode));
    }
    if (turnDebugDiagGuidancePTermValueLabel) {
        turnDebugDiagGuidancePTermValueLabel->setText(
            QString::number(turnDebug.diag_guidance_p_term_pwm));
    }
    if (turnDebugDiagGuidanceDTermValueLabel) {
        turnDebugDiagGuidanceDTermValueLabel->setText(
            QString::number(turnDebug.diag_guidance_d_term_pwm));
    }
    if (turnDebugDiagGuidanceCorrectionValueLabel) {
        turnDebugDiagGuidanceCorrectionValueLabel->setText(
            QString::number(turnDebug.diag_guidance_correction_pwm));
    }
    if (turnDebugWallKpValueLabel) {
        turnDebugWallKpValueLabel->setText(QString::number(turnDebug.wall_kp_pwm_per_mm));
    }
    if (turnDebugWallKdValueLabel) {
        turnDebugWallKdValueLabel->setText(QString::number(turnDebug.wall_kd_pwm_per_mm_per_tick));
    }
    if (turnDebugWallDeadbandValueLabel) {
        turnDebugWallDeadbandValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_error_deadband_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallTargetLeftValueLabel) {
        turnDebugWallTargetLeftValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_follow_target_left_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallTargetRightValueLabel) {
        turnDebugWallTargetRightValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.wall_follow_target_right_mm_q16), 0, 'f', 1));
    }
    if (turnDebugWallCorrectionLimitValueLabel) {
        turnDebugWallCorrectionLimitValueLabel->setText(
            QString::number(turnDebug.wall_correction_limit_pwm));
    }
    if (turnDebugWallSingleSideErrorScaleValueLabel) {
        turnDebugWallSingleSideErrorScaleValueLabel->setText(
            QString::number(turnDebug.wall_single_side_error_scale));
    }
    if (turnDebugLastCompletedActionValueLabel) {
        turnDebugLastCompletedActionValueLabel->setText(
            navActionText(turnDebug.last_completed_action));
    }
    if (turnDebugLastSmoothDoneReasonValueLabel) {
        turnDebugLastSmoothDoneReasonValueLabel->setText(
            smoothDoneReasonText(turnDebug.last_smooth_done_reason));
    }
    if (turnDebugLastSmoothFinalYawValueLabel) {
        turnDebugLastSmoothFinalYawValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.last_smooth_final_yaw_deg_q16), 0, 'f', 1));
    }
    if (turnDebugLastSmoothFinalRearValueLabel) {
        turnDebugLastSmoothFinalRearValueLabel->setText(
            turnDebug.last_smooth_final_floor_rear_black ? "true" : "false");
    }
    if (turnDebugLastAdvanceDoneReasonValueLabel) {
        turnDebugLastAdvanceDoneReasonValueLabel->setText(
            advanceDoneReasonText(turnDebug.last_advance_done_reason));
    }
    if (turnDebugLastAdvanceFinalYawValueLabel) {
        turnDebugLastAdvanceFinalYawValueLabel->setText(
            QString("%1 deg").arg(fromQ16(turnDebug.last_advance_final_yaw_deg_q16), 0, 'f', 1));
    }
    if (turnDebugLastAdvanceFinalRearValueLabel) {
        turnDebugLastAdvanceFinalRearValueLabel->setText(
            turnDebug.last_advance_final_floor_rear_black ? "true" : "false");
    }
    if (turnDebugLastApproachFrontDoneReasonValueLabel) {
        turnDebugLastApproachFrontDoneReasonValueLabel->setText(
            approachFrontDoneReasonText(turnDebug.last_approach_front_done_reason));
    }
    if (turnDebugLastCenterPivotDoneReasonValueLabel) {
        turnDebugLastCenterPivotDoneReasonValueLabel->setText(
            centerPivotDoneReasonText(turnDebug.last_center_pivot_done_reason));
    }

    if (navLeftMotorValueLabel) {
        navLeftMotorValueLabel->setText(QString::number(lastNavCommand.left_motor_pwm));
    }
    if (navRightMotorValueLabel) {
        navRightMotorValueLabel->setText(QString::number(lastNavCommand.right_motor_pwm));
    }
    if (simulationRunningValueLabel) {
        simulationRunningValueLabel->setText(simulationRunning ? "true" : "false");
    }
    if (simulationDtValueLabel) {
        simulationDtValueLabel->setText(QString::number(kSimulationIntervalMs));
    }
    if (autoModeValueLabel) {
        autoModeValueLabel->setText(autoModeEnabled ? "true" : "false");
    }
    if (simulationStepCountValueLabel) {
        simulationStepCountValueLabel->setText(QString::number(simulationStepCount));
    }
    if (simulationTimeValueLabel) {
        simulationTimeValueLabel->setText(QString("%1").arg(simulationTimeS, 0, 'f', 3));
    }
    if (motorTestModeValueLabel) {
        motorTestModeValueLabel->setText(motorTestModeEnabled ? "true" : "false");
    }
    if (sequenceEnabledValueLabel) {
        sequenceEnabledValueLabel->setText(testSequenceEnabled ? "true" : "false");
    }
    if (sequenceIndexValueLabel) {
        sequenceIndexValueLabel->setText(QString::number(testSequenceIndex));
    }
    if (sequenceLengthValueLabel) {
        sequenceLengthValueLabel->setText(QString::number(kTestSequenceLength));
    }
    if (sequenceCurrentActionValueLabel) {
        const bool hasSequenceAction = testSequenceEnabled
            && testSequenceIndex >= 0
            && testSequenceIndex < kTestSequenceLength;
        sequenceCurrentActionValueLabel->setText(
            hasSequenceAction ? sequenceStepText(kTestSequence[testSequenceIndex]) : "NONE");
    }
    if (sequenceWaitingNextTickValueLabel) {
        sequenceWaitingNextTickValueLabel->setText(
            testSequenceWaitingNextTick ? "true" : "false");
    }
    if (centerPivotSequenceActiveValueLabel) {
        const bool active = centerPivotSequencePhase == CenterPivotSequencePhase::Centering
            || centerPivotSequencePhase == CenterPivotSequencePhase::Pivot180;
        centerPivotSequenceActiveValueLabel->setText(active ? "true" : "false");
    }
    if (centerPivotSequencePhaseValueLabel) {
        centerPivotSequencePhaseValueLabel->setText(
            centerPivotSequencePhaseText(centerPivotSequencePhase));
    }
    if (centerPivotSequenceLastCenterReasonValueLabel) {
        centerPivotSequenceLastCenterReasonValueLabel->setText(
            centerPivotDoneReasonText(centerPivotSequenceLastCenterReason));
    }
    NavPlanDebugSnapshot planDebug = {};
    nav_core_plan_debug_snapshot(&planDebug);
    if (planExecutionEnabledValueLabel) {
        planExecutionEnabledValueLabel->setText(planExecutionEnabled ? "true" : "false");
    }
    if (planQueueCountValueLabel) {
        planQueueCountValueLabel->setText(
            QString("%1 / %2").arg(planDebug.count).arg(planDebug.capacity));
    }
    if (planCurrentActionValueLabel) {
        planCurrentActionValueLabel->setText(planActionText(planCurrentAction));
    }
    if (planNextActionValueLabel) {
        planNextActionValueLabel->setText(planActionText(planDebug.next_action));
    }
    if (planLastExecutedActionValueLabel) {
        planLastExecutedActionValueLabel->setText(planActionText(planLastExecutedAction));
    }
    if (planActionsExecutedCountValueLabel) {
        planActionsExecutedCountValueLabel->setText(QString::number(planActionsExecutedCount));
    }
    if (planQueueOverflowValueLabel) {
        planQueueOverflowValueLabel->setText(planDebug.overflow ? "true" : "false");
    }
    if (planCompositeActionActiveValueLabel) {
        const bool active = planCompositeActionPhase == CenterPivotSequencePhase::Centering
            || planCompositeActionPhase == CenterPivotSequencePhase::ApproachFront
            || planCompositeActionPhase == CenterPivotSequencePhase::Pivot180;
        planCompositeActionActiveValueLabel->setText(active ? "true" : "false");
    }
    if (planCompositeActionPhaseValueLabel) {
        planCompositeActionPhaseValueLabel->setText(
            centerPivotSequencePhaseText(planCompositeActionPhase));
    }
    if (planCompositePrepareMethodValueLabel) {
        planCompositePrepareMethodValueLabel->setText(
            planCompositePrepareMethodText(planCompositePrepareMethod));
    }
    if (planCompositeWallFrontAtStartValueLabel) {
        planCompositeWallFrontAtStartValueLabel->setText(
            planCompositeWallFrontAtStart ? "true" : "false");
    }
    if (planCompositeLastCenterReasonValueLabel) {
        planCompositeLastCenterReasonValueLabel->setText(
            centerPivotDoneReasonText(planCompositeLastCenterReason));
    }
    if (planCompositeLastApproachReasonValueLabel) {
        planCompositeLastApproachReasonValueLabel->setText(
            approachFrontDoneReasonText(planCompositeLastApproachReason));
    }
    if (planAdvanceAfterCenterPivotValueLabel) {
        planAdvanceAfterCenterPivotValueLabel->setText(
            planAdvanceStartedAfterCenterPivotDiagnostic ? "true" : "false");
    }
    if (planNextAdvanceFromCenteredPoseValueLabel) {
        planNextAdvanceFromCenteredPoseValueLabel->setText(
            planNextAdvanceFromCenteredPose ? "true" : "false");
    }
    NavRouteDebugSnapshot routeDebug = {};
    nav_core_get_route_debug(&routeDebug);
    if (routeStatusValueLabel) {
        routeStatusValueLabel->setText(routeStatusText(routeDebug.status));
    }
    if (routeTargetCellValueLabel) {
        routeTargetCellValueLabel->setText(
            QString("(%1,%2)").arg(routeDebug.target_cell_x).arg(routeDebug.target_cell_y));
    }
    if (routeStartCellValueLabel) {
        routeStartCellValueLabel->setText(
            QString("(%1,%2)").arg(routeDebug.start_cell_x).arg(routeDebug.start_cell_y));
    }
    if (routeStartDirValueLabel) {
        routeStartDirValueLabel->setText(mapDirectionText(routeDebug.start_dir));
    }
    if (routeLengthValueLabel) {
        routeLengthValueLabel->setText(QString::number(routeDebug.route_length));
    }
    if (routeExpandedStatesValueLabel) {
        routeExpandedStatesValueLabel->setText(QString::number(routeDebug.expanded_states));
    }
    if (routeFirstActionValueLabel) {
        routeFirstActionValueLabel->setText(planActionText(routeDebug.first_action));
    }
    if (routeLastActionValueLabel) {
        routeLastActionValueLabel->setText(planActionText(routeDebug.last_action));
    }
    if (routeLoadedIntoPlanQueueValueLabel) {
        routeLoadedIntoPlanQueueValueLabel->setText(
            routeDebug.loaded_into_plan_queue ? "true" : "false");
    }
    if (frontierRouteStatusValueLabel) {
        frontierRouteStatusValueLabel->setText(
            routeDebug.frontier_mode ? routeStatusText(routeDebug.status) : "IDLE");
    }
    if (frontierTargetCellValueLabel) {
        frontierTargetCellValueLabel->setText(
            QString("(%1,%2)")
                .arg(routeDebug.frontier_target_cell_x)
                .arg(routeDebug.frontier_target_cell_y));
    }
    if (frontierTargetDirValueLabel) {
        frontierTargetDirValueLabel->setText(mapDirectionText(routeDebug.frontier_target_dir));
    }
    if (frontierExitDirAbsoluteValueLabel) {
        frontierExitDirAbsoluteValueLabel->setText(
            mapDirectionText(routeDebug.frontier_exit_dir_absolute));
    }
    if (frontierExitRelativeValueLabel) {
        frontierExitRelativeValueLabel->setText(
            frontierExitRelativeText(routeDebug.frontier_exit_relative));
    }
    if (frontierNeighborCellValueLabel) {
        frontierNeighborCellValueLabel->setText(
            QString("(%1,%2)")
                .arg(routeDebug.frontier_neighbor_cell_x)
                .arg(routeDebug.frontier_neighbor_cell_y));
    }
    if (frontierRouteLengthValueLabel) {
        frontierRouteLengthValueLabel->setText(
            QString::number(routeDebug.frontier_mode ? routeDebug.route_length : 0));
    }
    if (frontierExpandedStatesValueLabel) {
        frontierExpandedStatesValueLabel->setText(
            QString::number(routeDebug.frontier_mode ? routeDebug.expanded_states : 0));
    }
    if (frontierLoadedIntoPlanQueueValueLabel) {
        frontierLoadedIntoPlanQueueValueLabel->setText(
            routeDebug.frontier_mode && routeDebug.loaded_into_plan_queue ? "true" : "false");
    }
    if (frontierCountFoundValueLabel) {
        frontierCountFoundValueLabel->setText(QString::number(routeDebug.frontier_count_found));
    }
    routePlanReadyToExecute =
        routeDebug.status == NAV_ROUTE_STATUS_FOUND
        && routeDebug.loaded_into_plan_queue
        && planDebug.count > 0;
    if (routeExecuteStatusValueLabel) {
        routeExecuteStatusValueLabel->setText(routeExecuteStatusText(routeExecuteStatus));
    }
    if (routeStartPhysicalValidValueLabel) {
        routeStartPhysicalValidValueLabel->setText(routeStartPhysicalValid ? "true" : "false");
    }
    if (routeStartFloorRearBlackValueLabel) {
        routeStartFloorRearBlackValueLabel->setText(routeStartFloorRearBlack ? "true" : "false");
    }
    if (routePlanReadyToExecuteValueLabel) {
        routePlanReadyToExecuteValueLabel->setText(routePlanReadyToExecute ? "true" : "false");
    }
    if (navAutonomyEnabledValueLabel) {
        navAutonomyEnabledValueLabel->setText(basicNavAutonomyEnabled ? "true" : "false");
    }
    if (navPolicyValueLabel) {
        navPolicyValueLabel->setText(navPolicyText(nav_core_get_policy()));
    }
    if (navRecommendedActionValueLabel) {
        navRecommendedActionValueLabel->setText(
            recommendedActionText(basicNavRecommendedAction));
    }
    if (navLastDecisionValueLabel) {
        navLastDecisionValueLabel->setText(basicNavLastDecisionText);
    }
    if (navDecisionWallFrontValueLabel) {
        navDecisionWallFrontValueLabel->setText(basicNavDecisionWallFront ? "true" : "false");
    }
    if (navDecisionWallLeftValueLabel) {
        navDecisionWallLeftValueLabel->setText(basicNavDecisionWallLeft ? "true" : "false");
    }
    if (navDecisionWallRightValueLabel) {
        navDecisionWallRightValueLabel->setText(basicNavDecisionWallRight ? "true" : "false");
    }
    if (navDecisionPointValidValueLabel) {
        navDecisionPointValidValueLabel->setText(
            basicNavDecisionPointValid ? "true" : "false");
    }
    if (deadEndRecoveryActiveValueLabel) {
        deadEndRecoveryActiveValueLabel->setText(
            deadEndRecoveryPhase != DeadEndRecoveryPhase::None ? "true" : "false");
    }
    if (deadEndRecoveryPhaseValueLabel) {
        deadEndRecoveryPhaseValueLabel->setText(deadEndRecoveryPhaseText(deadEndRecoveryPhase));
    }
    if (deadEndRecoveryLastApproachReasonValueLabel) {
        deadEndRecoveryLastApproachReasonValueLabel->setText(
            approachFrontDoneReasonText(deadEndRecoveryLastApproachReason));
    }
    if (deadEndRecoveryPendingPivotValueLabel) {
        deadEndRecoveryPendingPivotValueLabel->setText(
            deadEndRecoveryPendingPivot ? "true" : "false");
    }
    NavMapCandidateDebug candidateDebug = {};
    nav_core_get_map_candidate_debug(&candidateDebug);
    if (navMapCandidateRightCellValueLabel) {
        navMapCandidateRightCellValueLabel->setText(
            mapCandidateCellText(candidateDebug.right_cell_x,
                                 candidateDebug.right_cell_y,
                                 candidateDebug.right_cell_valid));
    }
    if (navMapCandidateFrontCellValueLabel) {
        navMapCandidateFrontCellValueLabel->setText(
            mapCandidateCellText(candidateDebug.front_cell_x,
                                 candidateDebug.front_cell_y,
                                 candidateDebug.front_cell_valid));
    }
    if (navMapCandidateLeftCellValueLabel) {
        navMapCandidateLeftCellValueLabel->setText(
            mapCandidateCellText(candidateDebug.left_cell_x,
                                 candidateDebug.left_cell_y,
                                 candidateDebug.left_cell_valid));
    }
    if (navMapCandidateRightVisitedValueLabel) {
        navMapCandidateRightVisitedValueLabel->setText(
            candidateDebug.right_cell_visited ? "true" : "false");
    }
    if (navMapCandidateFrontVisitedValueLabel) {
        navMapCandidateFrontVisitedValueLabel->setText(
            candidateDebug.front_cell_visited ? "true" : "false");
    }
    if (navMapCandidateLeftVisitedValueLabel) {
        navMapCandidateLeftVisitedValueLabel->setText(
            candidateDebug.left_cell_visited ? "true" : "false");
    }
    if (navMapUsedUnvisitedPreferenceValueLabel) {
        navMapUsedUnvisitedPreferenceValueLabel->setText(
            candidateDebug.used_unvisited_preference ? "true" : "false");
    }
    if (smartRecognitionStateValueLabel) {
        smartRecognitionStateValueLabel->setText(
            smartRecognitionStateText(smartRecognitionState));
    }
    if (smartLastFrontierStatusValueLabel) {
        smartLastFrontierStatusValueLabel->setText(routeStatusText(smartLastFrontierStatus));
    }
    if (smartFrontierPlanRequestedCountValueLabel) {
        smartFrontierPlanRequestedCountValueLabel->setText(
            QString::number(smartFrontierPlanRequestedCount));
    }
    if (smartFrontierRoutesExecutedCountValueLabel) {
        smartFrontierRoutesExecutedCountValueLabel->setText(
            QString::number(smartFrontierRoutesExecutedCount));
    }
    if (smartNoFrontierCountValueLabel) {
        smartNoFrontierCountValueLabel->setText(QString::number(smartNoFrontierCount));
    }
    if (smartLocalActionValueLabel) {
        smartLocalActionValueLabel->setText(recommendedActionText(smartLocalAction));
    }
    NavMapDebugSnapshot mapDebug = {};
    nav_core_get_map_debug(&mapDebug);
    if (mapEnabledValueLabel) {
        mapEnabledValueLabel->setText(mapDebug.enabled ? "true" : "false");
    }
    if (mapWidthValueLabel) {
        mapWidthValueLabel->setText(QString::number(mapDebug.width));
    }
    if (mapHeightValueLabel) {
        mapHeightValueLabel->setText(QString::number(mapDebug.height));
    }
    if (mapCellXValueLabel) {
        mapCellXValueLabel->setText(QString::number(mapDebug.cell_x));
    }
    if (mapCellYValueLabel) {
        mapCellYValueLabel->setText(QString::number(mapDebug.cell_y));
    }
    if (mapDirValueLabel) {
        mapDirValueLabel->setText(mapDirectionText(mapDebug.dir));
    }
    if (mapCurrentCellVisitedValueLabel) {
        mapCurrentCellVisitedValueLabel->setText(
            mapDebug.current_cell_visited ? "true" : "false");
    }
    if (mapCurrentCellSpecialValueLabel) {
        mapCurrentCellSpecialValueLabel->setText(
            mapDebug.current_cell_special ? "true" : "false");
    }
    if (mapCurrentCellWallsKnownValueLabel) {
        mapCurrentCellWallsKnownValueLabel->setText(
            QString("0x%1 (%2)")
                .arg(mapDebug.current_cell_walls_known, 2, 16, QChar('0'))
                .arg(wallMaskText(mapDebug.current_cell_walls_known)));
    }
    if (mapCurrentCellWallsPresentValueLabel) {
        mapCurrentCellWallsPresentValueLabel->setText(
            QString("0x%1 (%2)")
                .arg(mapDebug.current_cell_walls_present, 2, 16, QChar('0'))
                .arg(wallMaskText(mapDebug.current_cell_walls_present)));
    }
    if (mapLastPoseUpdateActionValueLabel) {
        mapLastPoseUpdateActionValueLabel->setText(
            mapActionText(mapDebug.last_pose_update_action));
    }
    if (mapLastWallUpdateActionValueLabel) {
        mapLastWallUpdateActionValueLabel->setText(
            mapActionText(mapDebug.last_wall_update_action));
    }
    if (mapInitialWallSnapshotPendingValueLabel) {
        mapInitialWallSnapshotPendingValueLabel->setText(
            mapDebug.initial_wall_snapshot_pending ? "true" : "false");
    }
    if (mapUpdateCountValueLabel) {
        mapUpdateCountValueLabel->setText(QString::number(mapDebug.update_count));
    }
    if (mapWallUpdateCountValueLabel) {
        mapWallUpdateCountValueLabel->setText(QString::number(mapDebug.wall_update_count));
    }
    if (mapSpecialCellsFoundCountValueLabel) {
        mapSpecialCellsFoundCountValueLabel->setText(
            QString::number(mapDebug.special_cells_found_count));
    }
    if (mapLastSpecialCellValueLabel) {
        if (mapDebug.last_special_cell_x >= 0 && mapDebug.last_special_cell_y >= 0) {
            mapLastSpecialCellValueLabel->setText(
                QString("(%1,%2)").arg(mapDebug.last_special_cell_x).arg(mapDebug.last_special_cell_y));
        } else {
            mapLastSpecialCellValueLabel->setText("none");
        }
    }
    mode1FoundSpecialCount = mapDebug.special_cells_found_count;
    const bool mode1AtStartCell = mode1MissionAtStartCell(mapDebug);
    if (mode1MissionEnabledValueLabel) {
        mode1MissionEnabledValueLabel->setText(mode1MissionEnabled ? "true" : "false");
    }
    if (mode1MissionStateValueLabel) {
        mode1MissionStateValueLabel->setText(mode1MissionStateText(mode1MissionState));
    }
    if (mode1RequiredSpecialCountValueLabel) {
        mode1RequiredSpecialCountValueLabel->setText(
            QString::number(mode1RequiredSpecialCount));
    }
    if (mode1FoundSpecialCountValueLabel) {
        mode1FoundSpecialCountValueLabel->setText(QString::number(mode1FoundSpecialCount));
    }
    if (mode1RequiredSpecialsReachedValueLabel) {
        mode1RequiredSpecialsReachedValueLabel->setText(
            mode1RequiredSpecialsReached ? "true" : "false");
    }
    if (mode1ReturnRequestedValueLabel) {
        mode1ReturnRequestedValueLabel->setText(mode1ReturnRequested ? "true" : "false");
    }
    if (mode1WaitingActionDoneValueLabel) {
        mode1WaitingActionDoneValueLabel->setText(
            mode1MissionState == Mode1MissionState::FoundRequiredSpecialsWaitActionDone
                ? "true"
                : "false");
    }
    if (mode1SearchCompleteLatchedAtCountValueLabel) {
        mode1SearchCompleteLatchedAtCountValueLabel->setText(
            QString::number(mode1SearchCompleteLatchedAtCount));
    }
    if (mode1PlanCancelledAfterRequiredFoundValueLabel) {
        mode1PlanCancelledAfterRequiredFoundValueLabel->setText(
            mode1PlanCancelledAfterRequiredFound ? "true" : "false");
    }
    if (mode1NavReadyForReturnValueLabel) {
        mode1NavReadyForReturnValueLabel->setText(
            mode1NavReadyForReturn ? "true" : "false");
    }
    if (mode1PlanWasActiveWhenRequiredFoundValueLabel) {
        mode1PlanWasActiveWhenRequiredFoundValueLabel->setText(
            mode1PlanWasActiveWhenRequiredFound ? "true" : "false");
    }
    if (mode1PlanQueueCountWhenRequiredFoundValueLabel) {
        mode1PlanQueueCountWhenRequiredFoundValueLabel->setText(
            QString::number(mode1PlanQueueCountWhenRequiredFound));
    }
    if (mode1StartCellValueLabel) {
        mode1StartCellValueLabel->setText(
            mode1StartCellValid
                ? QString("(%1,%2) %3")
                      .arg(mode1StartCellX)
                      .arg(mode1StartCellY)
                      .arg(mapDirectionText(mode1StartDir))
                : "invalid");
    }
    if (mode1ReturnRouteStatusValueLabel) {
        mode1ReturnRouteStatusValueLabel->setText(routeStatusText(mode1ReturnRouteStatus));
    }
    if (mode1ReturnPlanLoadedValueLabel) {
        mode1ReturnPlanLoadedValueLabel->setText(mode1ReturnPlanLoaded ? "true" : "false");
    }
    if (mode1ReturnToStartActiveValueLabel) {
        mode1ReturnToStartActiveValueLabel->setText(
            mode1MissionState == Mode1MissionState::ReturnToStartPlan
                    || mode1MissionState == Mode1MissionState::ReturnToStartExecute
                ? "true"
                : "false");
    }
    if (mode1AtStartCellValueLabel) {
        mode1AtStartCellValueLabel->setText(mode1AtStartCell ? "true" : "false");
    }
    if (mode1DoneReasonValueLabel) {
        mode1DoneReasonValueLabel->setText(
            mode1MissionDoneReasonText(mode1MissionDoneReason));
    }
    if (mapOverlayEnabledValueLabel) {
        mapOverlayEnabledValueLabel->setText(shadowMapOverlayEnabled ? "true" : "false");
    }
    if (simLeftMotorGainValueLabel) {
        simLeftMotorGainValueLabel->setText(QString("%1").arg(robot.leftMotorGain(), 0, 'f', 3));
    }
    if (simRightMotorGainValueLabel) {
        simRightMotorGainValueLabel->setText(QString("%1").arg(robot.rightMotorGain(), 0, 'f', 3));
    }
    if (simPivotCenterCorrectionEnabledValueLabel) {
        simPivotCenterCorrectionEnabledValueLabel->setText(
            robot.usePivotCenterCorrection() ? "true" : "false");
    }
    if (simPivotCenterLocalXValueLabel) {
        simPivotCenterLocalXValueLabel->setText(
            QString("%1 mm").arg(robot.pivotCenterLocalXmm(), 0, 'f', 1));
    }
    if (simPivotCenterLocalYValueLabel) {
        simPivotCenterLocalYValueLabel->setText(
            QString("%1 mm").arg(robot.pivotCenterLocalYmm(), 0, 'f', 1));
    }
    if (simLastMotionPivotLikeValueLabel) {
        simLastMotionPivotLikeValueLabel->setText(
            robot.lastMotionWasPivotLike() ? "true" : "false");
    }
    double floorRearGlobalX = 0.0;
    double floorRearGlobalY = 0.0;
    double floorRearLocalX = robot.pivotCenterLocalXmm();
    double floorRearLocalY = robot.pivotCenterLocalYmm();
    for (const FloorSensor &sensor : floorSensors) {
        if (QString(sensor.name) == "floor_rear") {
            floorRearLocalX = sensor.local_x_mm;
            floorRearLocalY = sensor.local_y_mm;
            break;
        }
    }
    robotLocalToWorld(floorRearLocalX, floorRearLocalY, &floorRearGlobalX, &floorRearGlobalY);
    if (simFloorRearGlobalXValueLabel) {
        simFloorRearGlobalXValueLabel->setText(QString("%1 mm").arg(floorRearGlobalX, 0, 'f', 1));
    }
    if (simFloorRearGlobalYValueLabel) {
        simFloorRearGlobalYValueLabel->setText(QString("%1 mm").arg(floorRearGlobalY, 0, 'f', 1));
    }
    if (motorTestLeftValueLabel) {
        motorTestLeftValueLabel->setText(QString::number(motorTestCommand.left_motor_pwm));
    }
    if (motorTestRightValueLabel) {
        motorTestRightValueLabel->setText(QString::number(motorTestCommand.right_motor_pwm));
    }
    syncPinnedTelemetryRows();
}

void MainWindow::syncPinnedTelemetryRows()
{
    for (TelemetryPinnedRow &row : telemetryPinnedRows) {
        if (!row.item || !row.source_label) {
            continue;
        }
        row.item->setText(1, row.source_label->text());
        row.item->setToolTip(1, row.source_label->text());
    }
}

void MainWindow::toggleTestSequence()
{
    if (testSequenceEnabled) {
        cancelTestSequence();
        nav_core_stop();
        lastNavCommand = {0, 0};
        updateTelemetryPanel();
        return;
    }

    setBasicNavAutonomyEnabled(false);
    testSequenceEnabled = true;
    testSequenceWaitingNextTick = false;
    testSequenceIndex = 0;
    cancelCenterPivotSequence();
    startCurrentTestSequenceStep();
    updateNavCorePipeline();
    updateTelemetryPanel();
}

void MainWindow::cancelTestSequence()
{
    testSequenceEnabled = false;
    testSequenceWaitingNextTick = false;
    testSequenceIndex = 0;
}

void MainWindow::startCurrentTestSequenceStep()
{
    if (!testSequenceEnabled
        || testSequenceIndex < 0
        || testSequenceIndex >= kTestSequenceLength) {
        cancelTestSequence();
        return;
    }

    const TestSequenceStep step = kTestSequence[testSequenceIndex];
    if (step == TestSequenceStep::AdvanceUntilRearBlack) {
        resetNavigationYawReference();
        nav_core_start_advance_until_rear_black();
        return;
    }

    resetNavigationYawReferenceForSmoothStart();
    RobotSensors sensors = buildRobotSensorsSnapshot();
    if (step == TestSequenceStep::SmoothTurnRight) {
        nav_core_start_smooth_turn_right(&sensors);
    } else if (step == TestSequenceStep::SmoothTurnLeft) {
        nav_core_start_smooth_turn_left(&sensors);
    }
}

void MainWindow::advanceTestSequenceIfNeeded()
{
    if (!testSequenceEnabled) {
        return;
    }

    if (testSequenceWaitingNextTick) {
        testSequenceWaitingNextTick = false;
        startCurrentTestSequenceStep();
        return;
    }

    if (nav_core_state() == NAV_STATE_DONE && nav_core_action() == NAV_ACTION_NONE) {
        ++testSequenceIndex;
        if (testSequenceIndex >= kTestSequenceLength) {
            cancelTestSequence();
            return;
        }

        testSequenceWaitingNextTick = true;
    }
}

void MainWindow::startCenterPivotSequence()
{
    setBasicNavAutonomyEnabled(false);
    cancelTestSequence();
    cancelPlanExecution();
    cancelPlanCompositeAction();
    cancelDeadEndRecovery();
    centerPivotSequencePhase = CenterPivotSequencePhase::Centering;
    centerPivotSequenceLastCenterReason = NAV_CENTER_PIVOT_DONE_NONE;
    resetNavigationYawReference();
    nav_core_start_center_in_cell_for_pivot_by_front_line();
    updateNavCorePipeline();
}

void MainWindow::cancelCenterPivotSequence()
{
    centerPivotSequencePhase = CenterPivotSequencePhase::None;
    centerPivotSequenceLastCenterReason = NAV_CENTER_PIVOT_DONE_NONE;
}

void MainWindow::advanceCenterPivotSequenceIfNeeded()
{
    if (centerPivotSequencePhase == CenterPivotSequencePhase::None
        || centerPivotSequencePhase == CenterPivotSequencePhase::Done
        || centerPivotSequencePhase == CenterPivotSequencePhase::Failed) {
        return;
    }

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);
    mode1NavReadyForReturn = navReady;
    if (!navReady) {
        return;
    }

    if (centerPivotSequencePhase == CenterPivotSequencePhase::Centering) {
        NavTurnDebug turnDebug = {};
        nav_core_get_turn_debug(&turnDebug);
        centerPivotSequenceLastCenterReason = turnDebug.last_center_pivot_done_reason;
        if (centerPivotSequenceLastCenterReason != NAV_CENTER_PIVOT_DONE_FRONT_LINE) {
            centerPivotSequencePhase = CenterPivotSequencePhase::Failed;
            return;
        }

        RobotSensors sensors = buildRobotSensorsSnapshot();
        resetNavigationYawReference();
        nav_core_start_pivot_turn_180(&sensors);
        centerPivotSequencePhase = CenterPivotSequencePhase::Pivot180;
        return;
    }

    if (centerPivotSequencePhase == CenterPivotSequencePhase::Pivot180) {
        centerPivotSequencePhase = CenterPivotSequencePhase::Done;
    }
}

void MainWindow::loadAndStartTestPlan()
{
    setBasicNavAutonomyEnabled(false);
    cancelTestSequence();
    cancelCenterPivotSequence();
    cancelPlanCompositeAction();
    cancelDeadEndRecovery();
    nav_core_plan_clear();

    const NavPlanAction testPlan[] = {
        NAV_PLAN_ACTION_ADVANCE_LINE,
        NAV_PLAN_ACTION_SMOOTH_RIGHT,
        NAV_PLAN_ACTION_ADVANCE_LINE,
        NAV_PLAN_ACTION_SMOOTH_LEFT,
        NAV_PLAN_ACTION_ADVANCE_LINE
    };

    bool loaded = true;
    for (NavPlanAction action : testPlan) {
        loaded = nav_core_plan_push(action) && loaded;
    }

    planExecutionEnabled = loaded && !nav_core_plan_is_empty();
    planCurrentAction = NAV_PLAN_ACTION_NONE;
    planLastExecutedAction = NAV_PLAN_ACTION_NONE;
    planActionsExecutedCount = 0;
    planAdvanceStartedAfterCenterPivotDiagnostic = false;
    planNextAdvanceFromCenteredPose = false;
    routeExecuteStatus = RouteExecuteStatus::Idle;
    routeStartPhysicalValid = false;
    routeStartFloorRearBlack = false;
    lastNavCommand = {0, 0};

    if (planExecutionEnabled) {
        advancePlanExecutionIfNeeded();
        updateNavCorePipeline();
    }
}

void MainWindow::cancelPlanExecution()
{
    planExecutionEnabled = false;
    planCurrentAction = NAV_PLAN_ACTION_NONE;
    planAdvanceStartedAfterCenterPivotDiagnostic = false;
    planNextAdvanceFromCenteredPose = false;
    cancelPlanCompositeAction();
    nav_core_plan_clear();
    lastNavCommand = {0, 0};
    routeExecuteStatus = RouteExecuteStatus::Cancelled;
    routeStartPhysicalValid = false;
    routeStartFloorRearBlack = false;
    routePlanReadyToExecute = false;
}

void MainWindow::setMode1MissionEnabled(bool enabled)
{
    mode1MissionEnabled = enabled;
    mode1ReturnRouteStatus = NAV_ROUTE_STATUS_IDLE;
    mode1ReturnPlanLoaded = false;
    if (mode1MissionEnabled) {
        mode1MissionState = Mode1MissionState::SearchSpecials;
        mode1MissionDoneReason = Mode1MissionDoneReason::None;
        mode1RequiredSpecialsReached = false;
        mode1ReturnRequested = false;
        mode1PlanCancelledAfterRequiredFound = false;
        mode1NavReadyForReturn = false;
        mode1PlanWasActiveWhenRequiredFound = false;
        mode1SearchCompleteLatchedAtCount = 0;
        mode1PlanQueueCountWhenRequiredFound = 0;
        nav_core_set_policy(NAV_POLICY_SMART_RECOGNITION);
    } else if (mode1MissionDoneReason != Mode1MissionDoneReason::Cancelled) {
        if (mode1MissionState == Mode1MissionState::ReturnToStartPlan
            || mode1MissionState == Mode1MissionState::ReturnToStartExecute) {
            cancelPlanExecution();
        }
        mode1MissionState = Mode1MissionState::Disabled;
        mode1MissionDoneReason = Mode1MissionDoneReason::None;
    }
}

void MainWindow::cancelMode1Mission(Mode1MissionDoneReason reason)
{
    mode1MissionEnabled = false;
    mode1MissionState = Mode1MissionState::Disabled;
    mode1MissionDoneReason = reason;
    mode1ReturnRouteStatus = NAV_ROUTE_STATUS_IDLE;
    mode1ReturnPlanLoaded = false;
    mode1ReturnRequested = false;
    mode1NavReadyForReturn = false;
}

bool MainWindow::mode1MissionAtStartCell(const NavMapDebugSnapshot &mapDebug) const
{
    return mode1StartCellValid
        && mapDebug.enabled
        && mapDebug.cell_x == mode1StartCellX
        && mapDebug.cell_y == mode1StartCellY;
}

bool MainWindow::advanceMode1MissionIfNeeded()
{
    if (!mode1MissionEnabled) {
        return false;
    }

    NavMapDebugSnapshot mapDebug = {};
    nav_core_get_map_debug(&mapDebug);
    mode1FoundSpecialCount = mapDebug.special_cells_found_count;

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);

    if (mode1MissionState == Mode1MissionState::Disabled) {
        mode1MissionState = Mode1MissionState::SearchSpecials;
        mode1MissionDoneReason = Mode1MissionDoneReason::None;
    }

    if (mode1MissionState == Mode1MissionState::SearchSpecials) {
        if (nav_core_get_policy() != NAV_POLICY_SMART_RECOGNITION) {
            nav_core_set_policy(NAV_POLICY_SMART_RECOGNITION);
        }
        if (mode1FoundSpecialCount < mode1RequiredSpecialCount) {
            return false;
        }
        mode1RequiredSpecialsReached = true;
        mode1ReturnRequested = true;
        mode1SearchCompleteLatchedAtCount = mode1FoundSpecialCount;
        NavPlanDebugSnapshot planDebug = {};
        nav_core_plan_debug_snapshot(&planDebug);
        mode1PlanWasActiveWhenRequiredFound = planExecutionEnabled;
        mode1PlanQueueCountWhenRequiredFound = planDebug.count;
        planExecutionEnabled = false;
        planCurrentAction = NAV_PLAN_ACTION_NONE;
        planNextAdvanceFromCenteredPose = false;
        nav_core_plan_clear();
        mode1PlanCancelledAfterRequiredFound =
            mode1PlanWasActiveWhenRequiredFound || mode1PlanQueueCountWhenRequiredFound > 0;
        mode1MissionState = Mode1MissionState::FoundRequiredSpecialsWaitActionDone;
        mode1ReturnRouteStatus = NAV_ROUTE_STATUS_IDLE;
        mode1ReturnPlanLoaded = false;
        routeExecuteStatus = RouteExecuteStatus::Idle;
        return true;
    }

    if (mode1MissionState == Mode1MissionState::FoundRequiredSpecialsWaitActionDone) {
        if (!navReady) {
            return true;
        }
        mode1MissionState = Mode1MissionState::ReturnToStartPlan;
    }

    if (mode1MissionState == Mode1MissionState::ReturnToStartPlan) {
        if (!mode1StartCellValid) {
            mode1MissionState = Mode1MissionState::Error;
            mode1MissionDoneReason = Mode1MissionDoneReason::StartCellInvalid;
            setBasicNavAutonomyEnabled(false);
            lastNavCommand = {0, 0};
            return true;
        }
        if (!navReady) {
            return true;
        }
        if (mode1MissionAtStartCell(mapDebug)) {
            mode1MissionState = Mode1MissionState::Done;
            mode1MissionDoneReason =
                Mode1MissionDoneReason::FoundRequiredSpecialsAndReturned;
            setBasicNavAutonomyEnabled(false);
            lastNavCommand = {0, 0};
            return true;
        }

        planExecutionEnabled = false;
        planCurrentAction = NAV_PLAN_ACTION_NONE;
        planNextAdvanceFromCenteredPose = false;
        cancelPlanCompositeAction();
        nav_core_plan_clear();

        mode1ReturnRouteStatus =
            nav_core_route_plan_to_cell(mode1StartCellX, mode1StartCellY);
        NavPlanDebugSnapshot planDebug = {};
        nav_core_plan_debug_snapshot(&planDebug);
        mode1ReturnPlanLoaded =
            mode1ReturnRouteStatus == NAV_ROUTE_STATUS_FOUND && planDebug.count > 0;

        if (mode1ReturnRouteStatus == NAV_ROUTE_STATUS_FOUND && planDebug.count > 0) {
            planExecutionEnabled = true;
            routeExecuteStatus = RouteExecuteStatus::Running;
            mode1MissionState = Mode1MissionState::ReturnToStartExecute;
            advancePlanExecutionIfNeeded();
            return true;
        }

        if (mode1ReturnRouteStatus == NAV_ROUTE_STATUS_FOUND
            && mode1MissionAtStartCell(mapDebug)) {
            mode1MissionState = Mode1MissionState::Done;
            mode1MissionDoneReason =
                Mode1MissionDoneReason::FoundRequiredSpecialsAndReturned;
            setBasicNavAutonomyEnabled(false);
            lastNavCommand = {0, 0};
            return true;
        }

        mode1MissionState = Mode1MissionState::Error;
        switch (mode1ReturnRouteStatus) {
        case NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS:
        case NAV_ROUTE_STATUS_TARGET_NOT_VISITED:
            mode1MissionDoneReason = Mode1MissionDoneReason::StartCellInvalid;
            break;
        case NAV_ROUTE_STATUS_ROUTE_TOO_LONG:
            mode1MissionDoneReason = Mode1MissionDoneReason::ReturnRouteTooLong;
            break;
        case NAV_ROUTE_STATUS_QUEUE_OVERFLOW:
            mode1MissionDoneReason = Mode1MissionDoneReason::ReturnQueueOverflow;
            break;
        case NAV_ROUTE_STATUS_IDLE:
        case NAV_ROUTE_STATUS_FOUND:
        case NAV_ROUTE_STATUS_FRONTIER_ALREADY_HERE:
        case NAV_ROUTE_STATUS_NO_PATH:
        case NAV_ROUTE_STATUS_NO_FRONTIER:
        default:
            mode1MissionDoneReason = Mode1MissionDoneReason::NoReturnRoute;
            break;
        }
        setBasicNavAutonomyEnabled(false);
        lastNavCommand = {0, 0};
        return true;
    }

    if (mode1MissionState == Mode1MissionState::ReturnToStartExecute) {
        advancePlanExecutionIfNeeded();
        nav_core_get_map_debug(&mapDebug);
        if (!planExecutionEnabled) {
            if (mode1MissionAtStartCell(mapDebug)) {
                mode1MissionState = Mode1MissionState::Done;
                mode1MissionDoneReason =
                    Mode1MissionDoneReason::FoundRequiredSpecialsAndReturned;
            } else {
                mode1MissionState = Mode1MissionState::Error;
                mode1MissionDoneReason = Mode1MissionDoneReason::NoReturnRoute;
            }
            setBasicNavAutonomyEnabled(false);
            lastNavCommand = {0, 0};
        }
        return true;
    }

    return mode1MissionState == Mode1MissionState::Done
        || mode1MissionState == Mode1MissionState::Error;
}

void MainWindow::cancelPlanCompositeAction()
{
    planCompositeActionPhase = CenterPivotSequencePhase::None;
    planCompositePrepareMethod = PlanCompositePrepareMethod::None;
    planCompositeWallFrontAtStart = false;
    planCompositeLastCenterReason = NAV_CENTER_PIVOT_DONE_NONE;
    planCompositeLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
}

void MainWindow::startPlanCompositeCenterAndPivot180()
{
    NavWallPerception perception = {};
    nav_core_get_wall_perception(&perception);
    planCompositeWallFrontAtStart = perception.wall_front;
    planCompositePrepareMethod = planCompositeWallFrontAtStart
        ? PlanCompositePrepareMethod::FrontWall
        : PlanCompositePrepareMethod::FrontLine;
    planCompositeActionPhase = planCompositeWallFrontAtStart
        ? CenterPivotSequencePhase::ApproachFront
        : CenterPivotSequencePhase::Centering;
    planCompositeLastCenterReason = NAV_CENTER_PIVOT_DONE_NONE;
    planCompositeLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
    planAdvanceStartedAfterCenterPivotDiagnostic = false;
    planNextAdvanceFromCenteredPose = false;
    planCurrentAction = NAV_PLAN_ACTION_CENTER_AND_PIVOT_180;
    planLastExecutedAction = NAV_PLAN_ACTION_CENTER_AND_PIVOT_180;
    ++planActionsExecutedCount;
    resetNavigationYawReference();
    if (planCompositeWallFrontAtStart) {
        nav_core_start_approach_front_wall_for_pivot();
    } else {
        nav_core_start_center_in_cell_for_pivot_by_front_line();
    }
}

bool MainWindow::advancePlanCompositeActionIfNeeded()
{
    if (planCompositeActionPhase == CenterPivotSequencePhase::None) {
        return false;
    }

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);
    if (!navReady) {
        return true;
    }

    if (planCompositeActionPhase == CenterPivotSequencePhase::ApproachFront) {
        NavTurnDebug turnDebug = {};
        nav_core_get_turn_debug(&turnDebug);
        planCompositeLastApproachReason = turnDebug.last_approach_front_done_reason;
        if (planCompositeLastApproachReason != NAV_APPROACH_FRONT_DONE_TARGET_DISTANCE
            && planCompositeLastApproachReason != NAV_APPROACH_FRONT_DONE_TIMEOUT) {
            planCompositeActionPhase = CenterPivotSequencePhase::Failed;
            planExecutionEnabled = false;
            nav_core_stop();
            lastNavCommand = {0, 0};
            return true;
        }

        RobotSensors sensors = buildRobotSensorsSnapshot();
        resetNavigationYawReference();
        nav_core_start_pivot_turn_180(&sensors);
        planCompositeActionPhase = CenterPivotSequencePhase::Pivot180;
        return true;
    }

    if (planCompositeActionPhase == CenterPivotSequencePhase::Centering) {
        NavTurnDebug turnDebug = {};
        nav_core_get_turn_debug(&turnDebug);
        planCompositeLastCenterReason = turnDebug.last_center_pivot_done_reason;
        if (planCompositeLastCenterReason != NAV_CENTER_PIVOT_DONE_FRONT_LINE) {
            planCompositeActionPhase = CenterPivotSequencePhase::Failed;
            planExecutionEnabled = false;
            nav_core_stop();
            lastNavCommand = {0, 0};
            return true;
        }

        RobotSensors sensors = buildRobotSensorsSnapshot();
        resetNavigationYawReference();
        nav_core_start_pivot_turn_180(&sensors);
        planCompositeActionPhase = CenterPivotSequencePhase::Pivot180;
        return true;
    }

    if (planCompositeActionPhase == CenterPivotSequencePhase::Pivot180) {
        planCompositeActionPhase = CenterPivotSequencePhase::Done;
        planCurrentAction = NAV_PLAN_ACTION_NONE;
        planNextAdvanceFromCenteredPose = true;
        return false;
    }

    return false;
}

void MainWindow::executeLoadedRouteIfSafe()
{
    updateIrSensors();
    updateFloorSensors();

    NavPlanDebugSnapshot planDebug = {};
    nav_core_plan_debug_snapshot(&planDebug);
    NavRouteDebugSnapshot routeDebug = {};
    nav_core_get_route_debug(&routeDebug);

    routePlanReadyToExecute =
        routeDebug.status == NAV_ROUTE_STATUS_FOUND
        && routeDebug.loaded_into_plan_queue
        && planDebug.count > 0;
    routeStartFloorRearBlack = buildRobotSensorsSnapshot().floor_rear_black;
    routeStartPhysicalValid =
        routePlanReadyToExecute
        && routeStartFloorRearBlack
        && nav_core_rear_line_trusted_for_decision();

    if (!routePlanReadyToExecute) {
        routeExecuteStatus = RouteExecuteStatus::NoRouteLoaded;
        return;
    }

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);
    if (!navReady) {
        routeExecuteStatus = RouteExecuteStatus::NavBusy;
        return;
    }

    if (!routeStartFloorRearBlack || !nav_core_rear_line_trusted_for_decision()) {
        routeExecuteStatus = RouteExecuteStatus::StartNotOnRearLine;
        return;
    }

    setBasicNavAutonomyEnabled(false);
    cancelTestSequence();
    cancelCenterPivotSequence();
    cancelDeadEndRecovery();
    planExecutionEnabled = true;
    routeExecuteStatus = RouteExecuteStatus::Started;
    advancePlanExecutionIfNeeded();
    if (planExecutionEnabled) {
        routeExecuteStatus = RouteExecuteStatus::Running;
    }
    updateNavCorePipeline();
}

bool MainWindow::startPlanAction(NavPlanAction action)
{
    if (action == NAV_PLAN_ACTION_NONE) {
        return false;
    }

    const bool startedAfterCenterAndPivot =
        action == NAV_PLAN_ACTION_ADVANCE_LINE
        && planNextAdvanceFromCenteredPose;

    if (planNextAdvanceFromCenteredPose && action != NAV_PLAN_ACTION_ADVANCE_LINE) {
        planNextAdvanceFromCenteredPose = false;
        return false;
    }

    const bool actionIsSmooth =
        action == NAV_PLAN_ACTION_SMOOTH_LEFT || action == NAV_PLAN_ACTION_SMOOTH_RIGHT;
    if (actionIsSmooth) {
        resetNavigationYawReferenceForSmoothStart();
    } else {
        resetNavigationYawReference();
    }
    RobotSensors sensors = buildRobotSensorsSnapshot();
    switch (action) {
    case NAV_PLAN_ACTION_ADVANCE_LINE:
        if (startedAfterCenterAndPivot) {
            nav_core_start_advance_until_rear_black_from_centered_pose();
            planNextAdvanceFromCenteredPose = false;
        } else {
            nav_core_start_advance_until_rear_black();
        }
        break;
    case NAV_PLAN_ACTION_SMOOTH_LEFT:
        nav_core_start_smooth_turn_left(&sensors);
        break;
    case NAV_PLAN_ACTION_SMOOTH_RIGHT:
        nav_core_start_smooth_turn_right(&sensors);
        break;
    case NAV_PLAN_ACTION_PIVOT_180:
        nav_core_start_pivot_turn_180(&sensors);
        break;
    case NAV_PLAN_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        nav_core_start_approach_front_wall_for_pivot();
        break;
    case NAV_PLAN_ACTION_CENTER_AND_PIVOT_180:
        startPlanCompositeCenterAndPivot180();
        return true;
    case NAV_PLAN_ACTION_NONE:
        return false;
    }

    planAdvanceStartedAfterCenterPivotDiagnostic = startedAfterCenterAndPivot;
    planCurrentAction = action;
    planLastExecutedAction = action;
    ++planActionsExecutedCount;
    return true;
}

void MainWindow::advancePlanExecutionIfNeeded()
{
    if (!planExecutionEnabled) {
        return;
    }

    if (advancePlanCompositeActionIfNeeded()) {
        return;
    }

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);
    if (!navReady) {
        return;
    }

    planCurrentAction = NAV_PLAN_ACTION_NONE;
    const NavPlanAction nextAction = nav_core_plan_pop_next();
    if (nextAction == NAV_PLAN_ACTION_NONE) {
        planExecutionEnabled = false;
        planNextAdvanceFromCenteredPose = false;
        cancelPlanCompositeAction();
        lastNavCommand = {0, 0};
        if (routeExecuteStatus == RouteExecuteStatus::Running
            || routeExecuteStatus == RouteExecuteStatus::Started) {
            routeExecuteStatus = RouteExecuteStatus::Completed;
        }
        return;
    }

    if (!startPlanAction(nextAction)) {
        planExecutionEnabled = false;
        cancelPlanCompositeAction();
        lastNavCommand = {0, 0};
    }
}

void MainWindow::loadMazeFromDialog()
{
    const QString startDir = world.mazeFilePath().isEmpty()
        ? QStringLiteral("data")
        : QFileInfo(world.mazeFilePath()).absolutePath();
    const QString path = QFileDialog::getOpenFileName(this,
                                                      "Load Maze",
                                                      startDir,
                                                      "Maze JSON (*.json);;All files (*.*)");
    if (path.isEmpty()) {
        return;
    }

    loadMazeFile(path);
}

bool MainWindow::loadMazeFile(const QString &path)
{
    const bool wasRunning = simulationRunning;
    setSimulationRunning(false);

    SimWorld loadedWorld = world;
    if (!loadedWorld.loadFromJsonFile(path)) {
        if (wasRunning) {
            setSimulationRunning(true);
        }
        QMessageBox::warning(this,
                             "Load Maze",
                             QString("Could not load maze JSON:\n%1").arg(path));
        return false;
    }

    cancelTestSequence();
    cancelCenterPivotSequence();
    cancelPlanExecution();
    setBasicNavAutonomyEnabled(false);
    nav_core_stop();
    lastNavCommand = {0, 0};
    world = loadedWorld;
    resetRobotPoseToWorldStart();
    rebuildSceneItems();
    updateNavCorePipeline();
    updateTelemetryPanel();
    return true;
}

void MainWindow::toggleBasicNavAutonomy()
{
    const bool enable = !basicNavAutonomyEnabled;
    setBasicNavAutonomyEnabled(enable);
    if (!enable) {
        nav_core_stop();
        updateNavCorePipeline();
    }
    if (basicNavAutonomyEnabled) {
        if (mode1MissionEnabled) {
            mode1MissionState = Mode1MissionState::SearchSpecials;
            mode1MissionDoneReason = Mode1MissionDoneReason::None;
            mode1RequiredSpecialsReached = false;
            mode1ReturnRequested = false;
            mode1PlanCancelledAfterRequiredFound = false;
            mode1NavReadyForReturn = false;
            mode1PlanWasActiveWhenRequiredFound = false;
            mode1SearchCompleteLatchedAtCount = 0;
            mode1PlanQueueCountWhenRequiredFound = 0;
            mode1ReturnRouteStatus = NAV_ROUTE_STATUS_IDLE;
            mode1ReturnPlanLoaded = false;
            nav_core_set_policy(NAV_POLICY_SMART_RECOGNITION);
        }
        cancelTestSequence();
        cancelCenterPivotSequence();
        cancelPlanExecution();
        updateIrSensors();
        updateFloorSensors();
        updateNavCorePipeline();
        advanceBasicNavAutonomyIfNeeded();
    }
    updateTelemetryPanel();
}

void MainWindow::setBasicNavAutonomyEnabled(bool enabled)
{
    basicNavAutonomyEnabled = enabled;
    if (!basicNavAutonomyEnabled) {
        basicNavRecommendedAction = NAV_RECOMMENDED_NONE;
        basicNavLastDecision = NAV_RECOMMENDED_NONE;
        basicNavLastDecisionText = "NONE";
        smartRecognitionState = SmartRecognitionState::Idle;
        smartLocalAction = NAV_RECOMMENDED_NONE;
        basicNavDecisionWallFront = false;
        basicNavDecisionWallLeft = false;
        basicNavDecisionWallRight = false;
        basicNavDecisionPointValid = false;
        cancelDeadEndRecovery();
    }
}

void MainWindow::toggleNavPolicy()
{
    const NavPolicy currentPolicy = nav_core_get_policy();
    NavPolicy nextPolicy = NAV_POLICY_RIGHT_HAND_RULE;
    switch (currentPolicy) {
    case NAV_POLICY_RIGHT_HAND_RULE:
        nextPolicy = NAV_POLICY_MAP_PREFER_UNVISITED;
        break;
    case NAV_POLICY_MAP_PREFER_UNVISITED:
        nextPolicy = NAV_POLICY_SMART_RECOGNITION;
        break;
    case NAV_POLICY_SMART_RECOGNITION:
        nextPolicy = NAV_POLICY_RIGHT_HAND_RULE;
        break;
    }
    nav_core_set_policy(nextPolicy);
    updateTelemetryPanel();
}

void MainWindow::cancelDeadEndRecovery()
{
    deadEndRecoveryPhase = DeadEndRecoveryPhase::None;
    deadEndRecoveryLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
    deadEndRecoveryPendingPivot = false;
}

bool MainWindow::advanceDeadEndRecoveryIfNeeded()
{
    if (deadEndRecoveryPhase == DeadEndRecoveryPhase::None) {
        return false;
    }

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);
    if (!navReady) {
        return true;
    }

    if (deadEndRecoveryPhase == DeadEndRecoveryPhase::ApproachFront) {
        NavTurnDebug turnDebug = {};
        nav_core_get_turn_debug(&turnDebug);
        deadEndRecoveryLastApproachReason = turnDebug.last_approach_front_done_reason;
        deadEndRecoveryPendingPivot = false;

        RobotSensors sensors = buildRobotSensorsSnapshot();
        resetNavigationYawReference();
        nav_core_start_pivot_turn_180(&sensors);
        deadEndRecoveryPhase = DeadEndRecoveryPhase::Pivot180;
        basicNavLastDecision = NAV_RECOMMENDED_PIVOT_180;
        basicNavLastDecisionText = "DEAD_END_PIVOT_180";
        return true;
    }

    deadEndRecoveryPhase = DeadEndRecoveryPhase::None;
    deadEndRecoveryPendingPivot = false;
    return false;
}

void MainWindow::advanceBasicNavAutonomyIfNeeded()
{
    if (!basicNavAutonomyEnabled) {
        return;
    }

    if (planCompositeActionPhase == CenterPivotSequencePhase::Centering
        || planCompositeActionPhase == CenterPivotSequencePhase::ApproachFront
        || planCompositeActionPhase == CenterPivotSequencePhase::Pivot180) {
        return;
    }

    if (deadEndRecoveryPhase != DeadEndRecoveryPhase::None
        && advanceDeadEndRecoveryIfNeeded()) {
        return;
    }

    if (advanceMode1MissionIfNeeded()) {
        return;
    }

    if (nav_core_get_policy() == NAV_POLICY_SMART_RECOGNITION && planExecutionEnabled) {
        smartRecognitionState = SmartRecognitionState::ExecutingFrontierRoute;
        return;
    }

    const bool navReady =
        (nav_core_action() == NAV_ACTION_NONE)
        && (nav_core_state() == NAV_STATE_IDLE || nav_core_state() == NAV_STATE_DONE);
    if (!navReady) {
        return;
    }

    NavWallPerception perception = {};
    nav_core_get_wall_perception(&perception);
    basicNavDecisionWallFront = perception.wall_front;
    basicNavDecisionWallLeft = perception.wall_left;
    basicNavDecisionWallRight = perception.wall_right;

    RobotSensors sensors = buildRobotSensorsSnapshot();
    const bool rearLineTrusted = nav_core_rear_line_trusted_for_decision();
    basicNavDecisionPointValid = sensors.floor_rear_black && rearLineTrusted;
    basicNavRecommendedAction = nav_core_recommend_basic_action(&sensors);
    smartLocalAction = basicNavRecommendedAction;

    if (nav_core_get_policy() == NAV_POLICY_SMART_RECOGNITION) {
        NavMapCandidateDebug candidateDebug = {};
        nav_core_get_map_candidate_debug(&candidateDebug);
        if (!basicNavDecisionPointValid
            || candidateDebug.used_unvisited_preference
            || basicNavRecommendedAction == NAV_RECOMMENDED_ACQUIRE_REAR_LINE
            || basicNavRecommendedAction == NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED) {
            smartRecognitionState = candidateDebug.used_unvisited_preference
                ? SmartRecognitionState::LocalUnvisited
                : SmartRecognitionState::Idle;
            startBasicNavRecommendedAction(basicNavRecommendedAction, sensors);
            return;
        }

        ++smartFrontierPlanRequestedCount;
        smartRecognitionState = SmartRecognitionState::PlanToFrontier;
        const NavRouteStatus frontierStatus = nav_core_route_plan_to_nearest_frontier();
        smartLastFrontierStatus = frontierStatus;
        NavPlanDebugSnapshot planDebug = {};
        nav_core_plan_debug_snapshot(&planDebug);
        if (frontierStatus == NAV_ROUTE_STATUS_FOUND && planDebug.count > 0) {
            planExecutionEnabled = true;
            ++smartFrontierRoutesExecutedCount;
            smartRecognitionState = SmartRecognitionState::ExecutingFrontierRoute;
            routeExecuteStatus = RouteExecuteStatus::Running;
            advancePlanExecutionIfNeeded();
            return;
        }
        if (frontierStatus == NAV_ROUTE_STATUS_FRONTIER_ALREADY_HERE) {
            smartRecognitionState = SmartRecognitionState::FrontierAlreadyHere;
            return;
        }
        if (frontierStatus == NAV_ROUTE_STATUS_NO_FRONTIER) {
            ++smartNoFrontierCount;
            if (mode1MissionEnabled
                && mode1MissionState == Mode1MissionState::SearchSpecials) {
                mode1MissionState = Mode1MissionState::Error;
                mode1MissionDoneReason =
                    Mode1MissionDoneReason::NoFrontierBeforeRequiredSpecials;
            }
            setBasicNavAutonomyEnabled(false);
            smartRecognitionState = SmartRecognitionState::NoFrontier;
            return;
        }

        smartRecognitionState = SmartRecognitionState::Error;
        return;
    }

    startBasicNavRecommendedAction(basicNavRecommendedAction, sensors);
}

void MainWindow::startBasicNavRecommendedAction(NavRecommendedAction action,
                                                const RobotSensors &sensors)
{
    if (action == NAV_RECOMMENDED_NONE) {
        return;
    }

    const bool actionIsSmooth =
        action == NAV_RECOMMENDED_SMOOTH_LEFT || action == NAV_RECOMMENDED_SMOOTH_RIGHT;
    if (actionIsSmooth) {
        resetNavigationYawReferenceForSmoothStart();
    } else {
        resetNavigationYawReference();
    }
    const RobotSensors startSensors = actionIsSmooth ? buildRobotSensorsSnapshot() : sensors;
    switch (action) {
    case NAV_RECOMMENDED_ACQUIRE_REAR_LINE:
        if (sensors.floor_rear_black && !nav_core_rear_line_trusted_for_decision()) {
            nav_core_start_advance_until_rear_black_from_centered_pose();
        } else {
            nav_core_start_advance_until_rear_black();
        }
        break;
    case NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED:
        nav_core_start_pivot_turn_180(&startSensors);
        break;
    case NAV_RECOMMENDED_ADVANCE_LINE:
        nav_core_start_advance_until_rear_black();
        break;
    case NAV_RECOMMENDED_SMOOTH_LEFT:
        nav_core_start_smooth_turn_left(&startSensors);
        break;
    case NAV_RECOMMENDED_SMOOTH_RIGHT:
        nav_core_start_smooth_turn_right(&startSensors);
        break;
    case NAV_RECOMMENDED_PIVOT_180:
        nav_core_start_approach_front_wall_for_pivot();
        deadEndRecoveryPhase = DeadEndRecoveryPhase::ApproachFront;
        deadEndRecoveryLastApproachReason = NAV_APPROACH_FRONT_DONE_NONE;
        deadEndRecoveryPendingPivot = true;
        basicNavLastDecision = action;
        basicNavLastDecisionText = "DEAD_END_APPROACH_FRONT";
        return;
    case NAV_RECOMMENDED_NONE:
        break;
    }

    basicNavLastDecision = action;
    basicNavLastDecisionText = recommendedActionText(action);
}

void MainWindow::resetRobotPoseToWorldStart()
{
    robot.setPose(world.startXMm(), world.startYMm(), world.startYawDeg());
    resetNavigationYawReference();
    initializeNavMapFromWorldStart();
    if (mode1MissionEnabled) {
        mode1MissionState = Mode1MissionState::SearchSpecials;
        mode1MissionDoneReason = Mode1MissionDoneReason::None;
        mode1RequiredSpecialsReached = false;
        mode1ReturnRequested = false;
        mode1PlanCancelledAfterRequiredFound = false;
        mode1NavReadyForReturn = false;
        mode1PlanWasActiveWhenRequiredFound = false;
        mode1SearchCompleteLatchedAtCount = 0;
        mode1PlanQueueCountWhenRequiredFound = 0;
        mode1ReturnRouteStatus = NAV_ROUTE_STATUS_IDLE;
        mode1ReturnPlanLoaded = false;
    }
}

void MainWindow::initializeNavMapFromWorldStart()
{
    const double cellSizeMm = world.cellSizeMm();
    int cellX = 0;
    int cellY = 0;
    if (cellSizeMm > 0.0) {
        cellX = static_cast<int>(std::floor(world.startXMm() / cellSizeMm));
        cellY = static_cast<int>(std::floor(world.startYMm() / cellSizeMm));
    }

    cellX = std::clamp(cellX, 0, std::max(0, world.cols() - 1));
    cellY = std::clamp(cellY, 0, std::max(0, world.rows() - 1));
    mode1StartCellX = static_cast<int8_t>(cellX);
    mode1StartCellY = static_cast<int8_t>(cellY);
    mode1StartDir = directionFromYawDeg(world.startYawDeg());
    mode1StartCellValid = world.cols() > 0
        && world.rows() > 0
        && cellX < NAV_MAP_MAX_WIDTH
        && cellY < NAV_MAP_MAX_HEIGHT;
    nav_core_map_init(static_cast<uint8_t>(std::clamp(world.cols(), 1, NAV_MAP_MAX_WIDTH)),
                      static_cast<uint8_t>(std::clamp(world.rows(), 1, NAV_MAP_MAX_HEIGHT)),
                      mode1StartCellX,
                      mode1StartCellY,
                      mode1StartDir);
    updateShadowMapOverlay();
}

void MainWindow::adjustSmoothTargetYawRate(int delta_deg_s)
{
    nav_core_set_smooth_target_yaw_rate_deg_s(
        nav_core_get_smooth_target_yaw_rate_deg_s() + delta_deg_s);
    updateTelemetryPanel();
}

void MainWindow::promptSmoothTargetYawRate()
{
    bool accepted = false;
    const int target = QInputDialog::getInt(this,
                                            "Smooth target yaw rate",
                                            "Target yaw rate (deg/s):",
                                            nav_core_get_smooth_target_yaw_rate_deg_s(),
                                            60,
                                            120,
                                            5,
                                            &accepted);
    if (!accepted) {
        return;
    }

    nav_core_set_smooth_target_yaw_rate_deg_s(target);
    updateTelemetryPanel();
}

void MainWindow::promptRoutePlanToCell()
{
    NavMapDebugSnapshot mapDebug = {};
    nav_core_get_map_debug(&mapDebug);

    bool accepted = false;
    const int targetX = QInputDialog::getInt(this,
                                             "Dry-run route plan",
                                             "Target cell X:",
                                             mapDebug.cell_x,
                                             0,
                                             std::max(0, static_cast<int>(mapDebug.width) - 1),
                                             1,
                                             &accepted);
    if (!accepted) {
        return;
    }

    const int targetY = QInputDialog::getInt(this,
                                             "Dry-run route plan",
                                             "Target cell Y:",
                                             mapDebug.cell_y,
                                             0,
                                             std::max(0, static_cast<int>(mapDebug.height) - 1),
                                             1,
                                             &accepted);
    if (!accepted) {
        return;
    }

    planExecutionEnabled = false;
    planCurrentAction = NAV_PLAN_ACTION_NONE;
    planAdvanceStartedAfterCenterPivotDiagnostic = false;
    planNextAdvanceFromCenteredPose = false;
    nav_core_route_plan_to_cell(static_cast<int16_t>(targetX),
                                static_cast<int16_t>(targetY));
    routeExecuteStatus = RouteExecuteStatus::Idle;
    routeStartPhysicalValid = false;
    routeStartFloorRearBlack = false;
    updateTelemetryPanel();
}

void MainWindow::planRouteToNearestFrontier()
{
    planExecutionEnabled = false;
    planCurrentAction = NAV_PLAN_ACTION_NONE;
    planAdvanceStartedAfterCenterPivotDiagnostic = false;
    planNextAdvanceFromCenteredPose = false;
    routeExecuteStatus = RouteExecuteStatus::Idle;
    routeStartPhysicalValid = false;
    routeStartFloorRearBlack = false;
    nav_core_route_plan_to_nearest_frontier();
    updateTelemetryPanel();
}

void MainWindow::showControlTuningDialog()
{
    QDialog dialog(this);
    dialog.setWindowTitle("Control Tuning");

    auto *rootLayout = new QVBoxLayout(&dialog);
    auto *scrollArea = new QScrollArea(&dialog);
    auto *scrollContent = new QWidget(scrollArea);
    auto *scrollLayout = new QVBoxLayout(scrollContent);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto *turnGroup = new QGroupBox("Turn yaw-rate PI", &dialog);
    auto *turnLayout = new QFormLayout(turnGroup);
    auto *turnKpSpin = new QDoubleSpinBox(turnGroup);
    auto *turnKiSpin = new QDoubleSpinBox(turnGroup);
    auto *turnKdSpin = new QDoubleSpinBox(turnGroup);
    auto *turnLimitSpin = new QSpinBox(turnGroup);

    auto *advanceYawGroup = new QGroupBox("Advance yaw PD", &dialog);
    auto *advanceYawLayout = new QFormLayout(advanceYawGroup);
    auto *advanceYawKpSpin = new QDoubleSpinBox(advanceYawGroup);
    auto *advanceYawKiSpin = new QDoubleSpinBox(advanceYawGroup);
    auto *advanceYawKdSpin = new QDoubleSpinBox(advanceYawGroup);
    auto *advanceYawLimitSpin = new QSpinBox(advanceYawGroup);

    auto *wallGroup = new QGroupBox("Advance wall PD", &dialog);
    auto *wallLayout = new QFormLayout(wallGroup);
    auto *wallKpSpin = new QSpinBox(wallGroup);
    auto *wallKdSpin = new QSpinBox(wallGroup);
    auto *wallLimitSpin = new QSpinBox(wallGroup);
    auto *wallDeadbandSpin = new QSpinBox(wallGroup);
    auto *wallTargetLeftSpin = new QSpinBox(wallGroup);
    auto *wallTargetRightSpin = new QSpinBox(wallGroup);

    auto *diagGroup = new QGroupBox("Diagonal guidance", &dialog);
    auto *diagLayout = new QFormLayout(diagGroup);
    auto *diagKpSpin = new QSpinBox(diagGroup);
    auto *diagKdSpin = new QSpinBox(diagGroup);
    auto *diagLimitSpin = new QSpinBox(diagGroup);
    auto *diagScaleNumSpin = new QSpinBox(diagGroup);
    auto *diagScaleDenSpin = new QSpinBox(diagGroup);
    auto *diagTargetSpin = new QSpinBox(diagGroup);
    auto *diagSmoothFinalModeCombo = new QComboBox(diagGroup);
    auto *smoothYawCarryEnabledCheck = new QCheckBox(diagGroup);
    auto *smoothYawCarryOnlySetpointCheck = new QCheckBox(diagGroup);
    auto *smoothYawCarryRequireDiagCheck = new QCheckBox(diagGroup);
    auto *smoothYawCarryAllowAdvancePreviewCheck = new QCheckBox(diagGroup);
    auto *smoothYawCarryMinAbsSpin = new QDoubleSpinBox(diagGroup);
    auto *smoothYawCarryMaxAbsSpin = new QDoubleSpinBox(diagGroup);
    auto *smoothYawCarryScaleNumSpin = new QSpinBox(diagGroup);
    auto *smoothYawCarryScaleDenSpin = new QSpinBox(diagGroup);

    auto *wallCautionGroup = new QGroupBox("Wall caution", &dialog);
    auto *wallCautionLayout = new QFormLayout(wallCautionGroup);
    auto *wallCautionEnabledCheck = new QCheckBox(wallCautionGroup);
    auto *wallCautionTimeoutSpin = new QSpinBox(wallCautionGroup);
    auto *wallCautionDeltaMaxSpin = new QSpinBox(wallCautionGroup);
    auto *wallCautionKpSpin = new QSpinBox(wallCautionGroup);
    auto *wallCautionKdSpin = new QSpinBox(wallCautionGroup);
    auto *wallCautionLimitSpin = new QSpinBox(wallCautionGroup);

    auto *mode1MissionGroup = new QGroupBox("Mode 1 mission", &dialog);
    auto *mode1MissionLayout = new QFormLayout(mode1MissionGroup);
    auto *mode1MissionEnabledCheck = new QCheckBox(mode1MissionGroup);
    auto *mode1RequiredSpecialCountSpin = new QSpinBox(mode1MissionGroup);

    const auto configureGainSpin = [](QDoubleSpinBox *spin) {
        spin->setRange(0.0, 600.0);
        spin->setDecimals(4);
        spin->setSingleStep(0.1);
    };
    const auto configureLimitSpin = [](QSpinBox *spin) {
        spin->setRange(0, 9999);
        spin->setSingleStep(100);
    };

    configureGainSpin(turnKpSpin);
    configureGainSpin(turnKiSpin);
    configureGainSpin(turnKdSpin);
    configureLimitSpin(turnLimitSpin);
    configureGainSpin(advanceYawKpSpin);
    configureGainSpin(advanceYawKiSpin);
    configureGainSpin(advanceYawKdSpin);
    configureLimitSpin(advanceYawLimitSpin);
    wallKpSpin->setRange(0, 600);
    wallKdSpin->setRange(0, 600);
    wallLimitSpin->setRange(0, kWallCorrectionLimitPwmMax);
    wallLimitSpin->setSingleStep(50);
    wallDeadbandSpin->setRange(0, 50);
    wallTargetLeftSpin->setRange(1, 200);
    wallTargetRightSpin->setRange(1, 200);
    diagKpSpin->setRange(0, 600);
    diagKdSpin->setRange(0, 600);
    diagLimitSpin->setRange(0, kWallCorrectionLimitPwmMax);
    diagLimitSpin->setSingleStep(50);
    diagScaleNumSpin->setRange(0, 100);
    diagScaleDenSpin->setRange(1, 100);
    diagTargetSpin->setRange(50, 160);
    diagSmoothFinalModeCombo->addItem("Hold relative",
                                      static_cast<int>(NAV_SMOOTH_FINAL_DIAG_MODE_HOLD_RELATIVE));
    diagSmoothFinalModeCombo->addItem("Setpoint",
                                      static_cast<int>(NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT));
    smoothYawCarryMinAbsSpin->setRange(0.0, 30.0);
    smoothYawCarryMinAbsSpin->setDecimals(2);
    smoothYawCarryMinAbsSpin->setSingleStep(0.5);
    smoothYawCarryMaxAbsSpin->setRange(0.0, 30.0);
    smoothYawCarryMaxAbsSpin->setDecimals(2);
    smoothYawCarryMaxAbsSpin->setSingleStep(0.5);
    smoothYawCarryScaleNumSpin->setRange(0, 100);
    smoothYawCarryScaleDenSpin->setRange(1, 100);
    wallCautionTimeoutSpin->setRange(0, 1000);
    wallCautionTimeoutSpin->setSingleStep(10);
    wallCautionDeltaMaxSpin->setRange(0, 100);
    wallCautionKpSpin->setRange(0, 600);
    wallCautionKdSpin->setRange(0, 600);
    wallCautionLimitSpin->setRange(0, kWallCorrectionLimitPwmMax);
    wallCautionLimitSpin->setSingleStep(50);
    mode1RequiredSpecialCountSpin->setRange(1, 16);

    turnLayout->addRow("Kp:", turnKpSpin);
    turnLayout->addRow("Ki:", turnKiSpin);
    turnLayout->addRow("Kd:", turnKdSpin);
    turnLayout->addRow("output_limit_pwm:", turnLimitSpin);
    advanceYawLayout->addRow("Kp:", advanceYawKpSpin);
    advanceYawLayout->addRow("Ki:", advanceYawKiSpin);
    advanceYawLayout->addRow("Kd:", advanceYawKdSpin);
    advanceYawLayout->addRow("output_limit_pwm:", advanceYawLimitSpin);
    wallLayout->addRow("Kp:", wallKpSpin);
    wallLayout->addRow("Kd:", wallKdSpin);
    wallLayout->addRow("correction_limit_pwm:", wallLimitSpin);
    wallLayout->addRow("error_deadband_mm:", wallDeadbandSpin);
    wallLayout->addRow("target_left_mm:", wallTargetLeftSpin);
    wallLayout->addRow("target_right_mm:", wallTargetRightSpin);
    diagLayout->addRow("diag_kp:", diagKpSpin);
    diagLayout->addRow("diag_kd:", diagKdSpin);
    diagLayout->addRow("diag_correction_limit_pwm:", diagLimitSpin);
    diagLayout->addRow("diag_error_scale_num:", diagScaleNumSpin);
    diagLayout->addRow("diag_error_scale_den:", diagScaleDenSpin);
    diagLayout->addRow("diag_target_mm:", diagTargetSpin);
    diagLayout->addRow("smooth_final_mode:", diagSmoothFinalModeCombo);
    diagLayout->addRow("smooth_yaw_carry_enabled:", smoothYawCarryEnabledCheck);
    diagLayout->addRow("smooth_yaw_carry_only_setpoint:", smoothYawCarryOnlySetpointCheck);
    diagLayout->addRow("smooth_yaw_carry_require_diag:", smoothYawCarryRequireDiagCheck);
    diagLayout->addRow("smooth_yaw_carry_allow_advance_preview:",
                       smoothYawCarryAllowAdvancePreviewCheck);
    diagLayout->addRow("smooth_yaw_carry_min_abs_deg:", smoothYawCarryMinAbsSpin);
    diagLayout->addRow("smooth_yaw_carry_max_abs_deg:", smoothYawCarryMaxAbsSpin);
    diagLayout->addRow("smooth_yaw_carry_offset_scale_num:", smoothYawCarryScaleNumSpin);
    diagLayout->addRow("smooth_yaw_carry_offset_scale_den:", smoothYawCarryScaleDenSpin);
    wallCautionLayout->addRow("enabled:", wallCautionEnabledCheck);
    wallCautionLayout->addRow("timeout_ms:", wallCautionTimeoutSpin);
    wallCautionLayout->addRow("delta_max_mm:", wallCautionDeltaMaxSpin);
    wallCautionLayout->addRow("kp:", wallCautionKpSpin);
    wallCautionLayout->addRow("kd:", wallCautionKdSpin);
    wallCautionLayout->addRow("correction_limit_pwm:", wallCautionLimitSpin);
    mode1MissionLayout->addRow("mode1_mission_enabled:", mode1MissionEnabledCheck);
    mode1MissionLayout->addRow("required_special_count:", mode1RequiredSpecialCountSpin);

    scrollLayout->addWidget(turnGroup);
    scrollLayout->addWidget(advanceYawGroup);
    scrollLayout->addWidget(wallGroup);
    scrollLayout->addWidget(diagGroup);
    scrollLayout->addWidget(wallCautionGroup);
    scrollLayout->addWidget(mode1MissionGroup);
    scrollLayout->addStretch(1);
    scrollArea->setWidget(scrollContent);
    rootLayout->addWidget(scrollArea, 1);

    auto *buttons = new QDialogButtonBox(&dialog);
    QPushButton *applyButton = buttons->addButton(QDialogButtonBox::Apply);
    QPushButton *resetButton = buttons->addButton(QDialogButtonBox::Reset);
    QPushButton *closeButton = buttons->addButton(QDialogButtonBox::Close);
    rootLayout->addWidget(buttons);
    dialog.resize(560, 760);

    const auto loadCurrentValues = [&]() {
        NavTurnPidConfig turnConfig = {};
        nav_core_get_turn_pid_config(&turnConfig);
        turnKpSpin->setValue(fromQ16(turnConfig.kp_q16));
        turnKiSpin->setValue(fromQ16(turnConfig.ki_q16));
        turnKdSpin->setValue(fromQ16(turnConfig.kd_q16));
        turnLimitSpin->setValue(static_cast<int>(turnConfig.output_limit_pwm));

        NavAdvanceYawPidConfig advanceYawConfig = {};
        nav_core_get_advance_yaw_pid_config(&advanceYawConfig);
        advanceYawKpSpin->setValue(fromQ16(advanceYawConfig.kp_q16));
        advanceYawKiSpin->setValue(fromQ16(advanceYawConfig.ki_q16));
        advanceYawKdSpin->setValue(fromQ16(advanceYawConfig.kd_q16));
        advanceYawLimitSpin->setValue(static_cast<int>(advanceYawConfig.output_limit_pwm));

        NavAdvanceWallConfig wallConfig = {};
        nav_core_get_advance_wall_config(&wallConfig);
        wallKpSpin->setValue(wallConfig.kp_pwm_per_mm);
        wallKdSpin->setValue(wallConfig.kd_pwm_per_mm_per_tick);
        wallLimitSpin->setValue(wallConfig.correction_limit_pwm);
        wallDeadbandSpin->setValue(wallConfig.error_deadband_mm);
        wallTargetLeftSpin->setValue(wallConfig.target_left_mm);
        wallTargetRightSpin->setValue(wallConfig.target_right_mm);

        NavDiagonalGuidanceConfig diagConfig = {};
        nav_core_get_diagonal_guidance_config(&diagConfig);
        diagKpSpin->setValue(diagConfig.kp_pwm_per_mm);
        diagKdSpin->setValue(diagConfig.kd_pwm_per_mm_per_tick);
        diagLimitSpin->setValue(diagConfig.correction_limit_pwm);
        diagScaleNumSpin->setValue(diagConfig.error_scale_num);
        diagScaleDenSpin->setValue(diagConfig.error_scale_den);
        diagTargetSpin->setValue(diagConfig.target_mm);
        const int modeIndex =
            diagSmoothFinalModeCombo->findData(static_cast<int>(diagConfig.smooth_final_mode));
        diagSmoothFinalModeCombo->setCurrentIndex(modeIndex >= 0 ? modeIndex : 0);

        NavSmoothYawCarryConfig carryConfig = {};
        nav_core_get_smooth_yaw_carry_config(&carryConfig);
        smoothYawCarryEnabledCheck->setChecked(carryConfig.enabled);
        smoothYawCarryOnlySetpointCheck->setChecked(carryConfig.only_setpoint);
        smoothYawCarryRequireDiagCheck->setChecked(carryConfig.require_diag);
        smoothYawCarryAllowAdvancePreviewCheck->setChecked(carryConfig.allow_advance_preview);
        smoothYawCarryMinAbsSpin->setValue(fromQ16(carryConfig.min_abs_deg_q16));
        smoothYawCarryMaxAbsSpin->setValue(fromQ16(carryConfig.max_abs_deg_q16));
        smoothYawCarryScaleNumSpin->setValue(carryConfig.offset_scale_num);
        smoothYawCarryScaleDenSpin->setValue(carryConfig.offset_scale_den);

        NavWallCautionConfig wallCautionConfig = {};
        nav_core_get_wall_caution_config(&wallCautionConfig);
        wallCautionEnabledCheck->setChecked(wallCautionConfig.enabled);
        wallCautionTimeoutSpin->setValue(static_cast<int>(wallCautionConfig.timeout_ms));
        wallCautionDeltaMaxSpin->setValue(wallCautionConfig.delta_max_mm);
        wallCautionKpSpin->setValue(wallCautionConfig.kp_pwm_per_mm);
        wallCautionKdSpin->setValue(wallCautionConfig.kd_pwm_per_mm_per_tick);
        wallCautionLimitSpin->setValue(wallCautionConfig.correction_limit_pwm);

        mode1MissionEnabledCheck->setChecked(mode1MissionEnabled);
        mode1RequiredSpecialCountSpin->setValue(static_cast<int>(mode1RequiredSpecialCount));
    };

    const auto applyValues = [&]() {
        NavTurnPidConfig turnConfig = {
            toQ16(turnKpSpin->value()),
            toQ16(turnKiSpin->value()),
            toQ16(turnKdSpin->value()),
            turnLimitSpin->value()
        };
        nav_core_set_turn_pid_config(&turnConfig);

        NavAdvanceYawPidConfig advanceYawConfig = {
            toQ16(advanceYawKpSpin->value()),
            toQ16(advanceYawKiSpin->value()),
            toQ16(advanceYawKdSpin->value()),
            advanceYawLimitSpin->value()
        };
        nav_core_set_advance_yaw_pid_config(&advanceYawConfig);

        NavAdvanceWallConfig wallConfig = {
            static_cast<int16_t>(wallKpSpin->value()),
            static_cast<int16_t>(wallKdSpin->value()),
            static_cast<int16_t>(wallLimitSpin->value()),
            static_cast<int16_t>(wallDeadbandSpin->value()),
            static_cast<int16_t>(wallTargetLeftSpin->value()),
            static_cast<int16_t>(wallTargetRightSpin->value())
        };
        nav_core_set_advance_wall_config(&wallConfig);

        NavDiagonalGuidanceConfig diagConfig = {
            static_cast<int16_t>(diagKpSpin->value()),
            static_cast<int16_t>(diagKdSpin->value()),
            static_cast<int16_t>(diagLimitSpin->value()),
            static_cast<int16_t>(diagScaleNumSpin->value()),
            static_cast<int16_t>(diagScaleDenSpin->value()),
            static_cast<int16_t>(diagTargetSpin->value()),
            static_cast<NavSmoothFinalDiagonalMode>(
                diagSmoothFinalModeCombo->currentData().toInt())
        };
        nav_core_set_diagonal_guidance_config(&diagConfig);

        NavSmoothYawCarryConfig carryConfig = {
            smoothYawCarryEnabledCheck->isChecked(),
            smoothYawCarryOnlySetpointCheck->isChecked(),
            smoothYawCarryRequireDiagCheck->isChecked(),
            smoothYawCarryAllowAdvancePreviewCheck->isChecked(),
            toQ16(smoothYawCarryMinAbsSpin->value()),
            toQ16(smoothYawCarryMaxAbsSpin->value()),
            static_cast<int16_t>(smoothYawCarryScaleNumSpin->value()),
            static_cast<int16_t>(smoothYawCarryScaleDenSpin->value())
        };
        nav_core_set_smooth_yaw_carry_config(&carryConfig);

        NavWallCautionConfig wallCautionConfig = {
            wallCautionEnabledCheck->isChecked(),
            static_cast<uint16_t>(wallCautionTimeoutSpin->value()),
            static_cast<int16_t>(wallCautionDeltaMaxSpin->value()),
            static_cast<int16_t>(wallCautionKpSpin->value()),
            static_cast<int16_t>(wallCautionKdSpin->value()),
            static_cast<int16_t>(wallCautionLimitSpin->value())
        };
        nav_core_set_wall_caution_config(&wallCautionConfig);

        mode1RequiredSpecialCount =
            static_cast<uint16_t>(mode1RequiredSpecialCountSpin->value());
        setMode1MissionEnabled(mode1MissionEnabledCheck->isChecked());
        updateTelemetryPanel();
    };

    connect(applyButton, &QPushButton::clicked, this, applyValues);
    connect(resetButton, &QPushButton::clicked, this, [&]() {
        nav_core_reset_turn_pid_defaults();
        nav_core_reset_advance_yaw_pid_defaults();
        nav_core_reset_advance_wall_defaults();
        nav_core_reset_diagonal_guidance_defaults();
        nav_core_reset_smooth_yaw_carry_defaults();
        nav_core_reset_wall_caution_defaults();
        mode1RequiredSpecialCount = 3;
        setMode1MissionEnabled(true);
        loadCurrentValues();
        updateTelemetryPanel();
    });
    connect(closeButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    loadCurrentValues();
    dialog.exec();
}

void MainWindow::showControlsHelp()
{
    QMessageBox::information(
        this,
        "Controls",
        "Manual:\n"
        "- W / Up: avanzar 10 mm\n"
        "- S / Down: retroceder 10 mm\n"
        "- A / Left: rotar -10 grados\n"
        "- D / Right: rotar +10 grados\n"
        "- R: reset pose\n"
        "\n"
        "Simulation:\n"
        "- Space: Play/Pause\n"
        "- N: Step once when paused. If auto_mode=true, it advances one simulation tick.\n"
        "- M: Toggle auto mode\n"
        "- O: Load maze JSON\n"
        "- V: Toggle fixed test sequence\n"
        "- U: Load and execute planned test action queue\n"
        "- J: Execute route currently loaded by K, only when rear sensor is on line\n"
        "- K: Dry-run route plan to target cell\n"
        "- T: Dry-run route plan to nearest exploration frontier\n"
        "- L: Test CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE -> PIVOT_180 sequence\n"
        "- B: Toggle basic autonomous navigation; dead-ends use approach-front then PIVOT_180\n"
        "- P: Toggle nav policy RIGHT_HAND_RULE / MAP_PREFER_UNVISITED / SMART_RECOGNITION\n"
        "- C: Toggle ADVANCE guidance WALL_ASSIST / YAW_ONLY\n"
        "- Y: Toggle shadow logical map overlay\n"
        "\n"
        "Motor test:\n"
        "- I: test PWM {3000, 3000} avanzar\n"
        "\n"
        "Smooth tuning:\n"
        "- +: increase smooth target yaw rate by 5 deg/s\n"
        "- -: decrease smooth target yaw rate by 5 deg/s\n"
        "- F2: set exact smooth target yaw rate\n"
        "- F3: open Control Tuning window, including optional Mode 1 mission\n"
        "\n"
        "Navigation test:\n"
        "- G: start ADVANCE_LINE until rear floor sensor detects black\n"
        "- F: start APPROACH_FRONT_WALL_FOR_PIVOT test\n"
        "- H: start CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE test\n"
        "- Q: start SMOOTH_LEFT test, resets nav yaw reference\n"
        "- E: start SMOOTH_RIGHT test, resets nav yaw reference\n"
        "- 1: start PIVOT_LEFT test, resets nav yaw reference\n"
        "- 2: start PIVOT_RIGHT test, resets nav yaw reference\n"
        "- 3: start PIVOT_180 test, resets nav yaw reference\n"
        "- X: stop navigation action and cancel autonomy/plan\n"
        "- Z: reset navigation yaw reference\n"
        "\n"
        "Notas:\n"
        "- Auto movement only applies when running=true and auto_mode=true.\n"
        "- Motor test command only overrides nav command when motor_test=true.\n"
        "- nav_core only returns movement PWM while a navigation action is active."
    );
}

void MainWindow::createRobotItem()
{
    const double halfLength = robot.lengthMm() / 2.0;
    const double halfWidth = robot.widthMm() / 2.0;

    QPolygonF shape;
    shape << QPointF(halfLength, 0.0)
          << QPointF(halfLength * 0.35, halfWidth)
          << QPointF(-halfLength, halfWidth)
          << QPointF(-halfLength, -halfWidth)
          << QPointF(halfLength * 0.35, -halfWidth);

    robotItem = scene->addPolygon(shape, QPen(QColor(20, 70, 120), 2), QBrush(QColor(40, 140, 220)));
    robotItem->setTransformOriginPoint(0.0, 0.0);
    robotItem->setZValue(10.0);
}

void MainWindow::createIrSensorItems()
{
    const QPen rayPen(QColor(230, 90, 20), 2);
    const QPen hitPen(QColor(160, 40, 0), 1);
    const QBrush hitBrush(QColor(255, 120, 40));

    for (IrSensor &sensor : irSensors) {
        sensor.ray_item = scene->addLine(0.0, 0.0, 0.0, 0.0, rayPen);
        sensor.ray_item->setZValue(8.0);

        sensor.hit_item = scene->addEllipse(-kIrHitRadiusMm,
                                            -kIrHitRadiusMm,
                                            kIrHitRadiusMm * 2.0,
                                            kIrHitRadiusMm * 2.0,
                                            hitPen,
                                            hitBrush);
        sensor.hit_item->setZValue(9.0);
        sensor.hit_item->setVisible(false);
    }
}

void MainWindow::createFloorSensorItems()
{
    for (FloorSensor &sensor : floorSensors) {
        sensor.marker_item = scene->addEllipse(-kFloorSensorRadiusMm,
                                               -kFloorSensorRadiusMm,
                                               kFloorSensorRadiusMm * 2.0,
                                               kFloorSensorRadiusMm * 2.0,
                                               QPen(QColor(30, 30, 30), 2),
                                               QBrush(Qt::white));
        sensor.marker_item->setZValue(12.0);
    }
}

void MainWindow::updateRobotVisualOnly()
{
    if (!robotItem) {
        return;
    }

    robotItem->setPos(robot.xMm(), robot.yMm());
    robotItem->setRotation(robot.yawDeg());
}

void MainWindow::updateRobotGraphics()
{
    updateRobotVisualOnly();
    updateIrSensors();
    updateFloorSensors();
    updateNavCorePipeline();
    updateTelemetryPanel();
}

void MainWindow::updateIrSensors()
{
    for (IrSensor &sensor : irSensors) {
        if (!sensor.ray_item || !sensor.hit_item) {
            continue;
        }

        double originX = 0.0;
        double originY = 0.0;
        robotLocalToWorld(sensor.local_x_mm, sensor.local_y_mm, &originX, &originY);

        const double sensorAngleDeg = robot.yawDeg() + sensor.angle_offset_deg;
        const RaycastHit hit = world.castRay(originX,
                                             originY,
                                             sensorAngleDeg,
                                             sensor.max_distance_mm);
        sensor.last_distance_mm = hit.hit ? hit.distance_mm : sensor.max_distance_mm;
        sensor.ray_item->setLine(originX, originY, hit.hit_x_mm, hit.hit_y_mm);
        sensor.hit_item->setPos(hit.hit_x_mm, hit.hit_y_mm);
        sensor.hit_item->setVisible(hit.hit);
    }
}

void MainWindow::updateFloorSensors()
{
    for (FloorSensor &sensor : floorSensors) {
        if (!sensor.marker_item) {
            continue;
        }

        double sensorX = 0.0;
        double sensorY = 0.0;
        robotLocalToWorld(sensor.local_x_mm, sensor.local_y_mm, &sensorX, &sensorY);

        sensor.is_black = world.isBlackTapeAt(sensorX, sensorY);
        sensor.debug_kind = world.debugTapeKindAt(sensorX, sensorY);
        sensor.marker_item->setPos(sensorX, sensorY);
        sensor.marker_item->setBrush(sensor.is_black ? QBrush(Qt::black) : QBrush(Qt::white));
    }
}

void MainWindow::updateNavCorePipeline()
{
    RobotSensors sensors = buildRobotSensorsSnapshot();
    lastNavCommand = nav_core_update(&sensors);
    updateShadowMapOverlay();
}

void MainWindow::simulationStep()
{
    ++simulationStepCount;
    simulationTimeS += kSimulationDtS;

    advanceTestSequenceIfNeeded();
    advanceCenterPivotSequenceIfNeeded();
    advancePlanExecutionIfNeeded();
    updateIrSensors();
    updateFloorSensors();
    updateNavCorePipeline();
    advanceTestSequenceIfNeeded();
    advanceCenterPivotSequenceIfNeeded();
    advancePlanExecutionIfNeeded();
    advanceBasicNavAutonomyIfNeeded();

    if (autoModeEnabled) {
        const RobotCommand command = motorTestModeEnabled ? motorTestCommand : lastNavCommand;
        robot.applyDifferentialDrive(command.left_motor_pwm,
                                     command.right_motor_pwm,
                                     kSimulationDtS);
        updateRobotVisualOnly();
        updateIrSensors();
        updateFloorSensors();
    }

    updateTelemetryPanel();
}

void MainWindow::setSimulationRunning(bool running)
{
    simulationRunning = running;

    if (simulationRunning) {
        simulationTimer->start();
    } else {
        simulationTimer->stop();
    }

    updateTelemetryPanel();
}

void MainWindow::resetNavigationYawReference()
{
    nav_core_prepare_smooth_yaw_carry_for_next_action(false);
    navYawZeroDeg = robot.yawDeg();
}

void MainWindow::resetNavigationYawReferenceForSmoothStart()
{
    nav_core_prepare_smooth_yaw_carry_for_next_action(true);
    q16_16_t carryOffsetQ16 = 0;
    if (nav_core_has_smooth_yaw_carry_pending()) {
        carryOffsetQ16 = nav_core_consume_smooth_yaw_carry_offset_q16();
    }
    navYawZeroDeg = robot.yawDeg() - fromQ16(carryOffsetQ16);
}

double MainWindow::navigationYawDeg() const
{
    return normalizeAngleSignedDeg(robot.yawDeg() - navYawZeroDeg);
}

RobotSensors MainWindow::buildRobotSensorsSnapshot() const
{
    RobotSensors sensors = {};

    for (const IrSensor &sensor : irSensors) {
        const QString name(sensor.name);
        if (name == "front_left") {
            sensors.ir_front_left_mm_q16 = toQ16(sensor.last_distance_mm);
        } else if (name == "front_right") {
            sensors.ir_front_right_mm_q16 = toQ16(sensor.last_distance_mm);
        } else if (name == "left") {
            sensors.ir_left_mm_q16 = toQ16(sensor.last_distance_mm);
        } else if (name == "right") {
            sensors.ir_right_mm_q16 = toQ16(sensor.last_distance_mm);
        } else if (name == "diag_left") {
            sensors.ir_diag_left_mm_q16 = toQ16(sensor.last_distance_mm);
        } else if (name == "diag_right") {
            sensors.ir_diag_right_mm_q16 = toQ16(sensor.last_distance_mm);
        }
    }

    sensors.yaw_deg_q16 = toQ16(navigationYawDeg());
    sensors.yaw_rate_deg_s_q16 = toQ16(robot.yawRateDegS());

    for (const FloorSensor &sensor : floorSensors) {
        const QString name(sensor.name);
        if (name == "floor_front") {
            sensors.floor_front_black = sensor.is_black;
        } else if (name == "floor_rear") {
            sensors.floor_rear_black = sensor.is_black;
        }
    }

    return sensors;
}

void MainWindow::robotLocalToWorld(double local_x_mm,
                                   double local_y_mm,
                                   double *global_x_mm,
                                   double *global_y_mm) const
{
    const double yawRad = robot.yawDeg() * kDegToRad;
    const double cosYaw = std::cos(yawRad);
    const double sinYaw = std::sin(yawRad);

    *global_x_mm = robot.xMm() + cosYaw * local_x_mm - sinYaw * local_y_mm;
    *global_y_mm = robot.yMm() + sinYaw * local_x_mm + cosYaw * local_y_mm;
}

void MainWindow::fitSceneToView()
{
    if (!view || !scene || scene->items().isEmpty()) {
        return;
    }

    view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}
