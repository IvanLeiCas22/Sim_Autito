#include "mainwindow.h"

#include "nav_core.h"

#include <QAction>
#include <QBrush>
#include <QColor>
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
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QGroupBox>
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
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>
#include <limits>

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

enum BasicNavDecision {
    BasicNavDecisionNone = 0,
    BasicNavDecisionAcquireRearLine,
    BasicNavDecisionRecoveryPivot180FrontBlocked,
    BasicNavDecisionSmoothRight,
    BasicNavDecisionAdvance,
    BasicNavDecisionSmoothLeft,
    BasicNavDecisionPivot180
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
    }

    return "UNKNOWN";
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

QString basicNavDecisionText(int decision)
{
    switch (decision) {
    case BasicNavDecisionNone:
        return "NONE";
    case BasicNavDecisionAcquireRearLine:
        return "ACQUIRE_REAR_LINE";
    case BasicNavDecisionRecoveryPivot180FrontBlocked:
        return "RECOVERY_PIVOT_180_FRONT_BLOCKED";
    case BasicNavDecisionSmoothRight:
        return "SMOOTH_TURN_RIGHT";
    case BasicNavDecisionAdvance:
        return "ADVANCE_UNTIL_REAR_BLACK";
    case BasicNavDecisionSmoothLeft:
        return "SMOOTH_TURN_LEFT";
    case BasicNavDecisionPivot180:
        return "PIVOT_TURN_180";
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
        motorTestModeEnabled = !motorTestModeEnabled;
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_I:
        motorTestCommand = {3000, 3000};
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_K:
        motorTestCommand = {-3000, -3000};
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_J:
        motorTestCommand = {-1500, 1500};
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_L:
        motorTestCommand = {1500, -1500};
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_U:
        motorTestCommand = {0, 0};
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
    case Qt::Key_G:
        resetNavigationYawReference();
        nav_core_start_advance_until_rear_black();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_X:
        cancelTestSequence();
        setBasicNavAutonomyEnabled(false);
        nav_core_stop();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_Q: {
        resetNavigationYawReference();
        RobotSensors sensors = buildRobotSensorsSnapshot();
        nav_core_start_smooth_turn_left(&sensors);
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    }
    case Qt::Key_E: {
        resetNavigationYawReference();
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
    case Qt::Key_H:
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
        {"floor_front", 35.0, 0.0},
        {"floor_rear", -35.0, 0.0}
    };
}

void MainWindow::createTelemetryPanel()
{
    auto *dock = new QDockWidget("Telemetry", this);
    auto *panel = new QWidget(dock);
    auto *layout = new QFormLayout(panel);
    auto *scrollArea = new QScrollArea(dock);
    dock->setMinimumWidth(260);
    dock->setMaximumWidth(260);
    panel->setMinimumWidth(460);
    layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

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
    auto *navAutonomyTitle = new QLabel("<b>Basic nav autonomy</b>", panel);
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
    turnDebugAdvancePhaseValueLabel = new QLabel(panel);
    turnDebugAdvanceDoneReasonValueLabel = new QLabel(panel);
    turnDebugAdvanceYawSetpointValueLabel = new QLabel(panel);
    turnDebugAdvanceYawMeasuredValueLabel = new QLabel(panel);
    turnDebugAdvanceYawErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceYawOutputValueLabel = new QLabel(panel);
    turnDebugAdvanceYawCorrectionValueLabel = new QLabel(panel);
    turnDebugAdvanceYawKpValueLabel = new QLabel(panel);
    turnDebugAdvanceYawKiValueLabel = new QLabel(panel);
    turnDebugAdvanceYawKdValueLabel = new QLabel(panel);
    turnDebugAdvanceYawOutputLimitValueLabel = new QLabel(panel);
    turnDebugAdvanceBaseLeftValueLabel = new QLabel(panel);
    turnDebugAdvanceBaseRightValueLabel = new QLabel(panel);
    turnDebugAdvanceGuidanceModeValueLabel = new QLabel(panel);
    turnDebugAdvanceCorrectionSourceValueLabel = new QLabel(panel);
    turnDebugAdvanceWallLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRightValidValueLabel = new QLabel(panel);
    turnDebugAdvanceDiagLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceDiagRightValidValueLabel = new QLabel(panel);
    turnDebugAdvanceFollowLeftValidValueLabel = new QLabel(panel);
    turnDebugAdvanceFollowRightValidValueLabel = new QLabel(panel);
    turnDebugAdvanceWallLeftValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRightValueLabel = new QLabel(panel);
    turnDebugAdvanceWallErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceWallErrorAfterDeadbandValueLabel = new QLabel(panel);
    turnDebugAdvanceWallPrevErrorValueLabel = new QLabel(panel);
    turnDebugAdvanceWallErrorDeltaValueLabel = new QLabel(panel);
    turnDebugAdvanceWallPTermValueLabel = new QLabel(panel);
    turnDebugAdvanceWallDTermValueLabel = new QLabel(panel);
    turnDebugAdvanceWallRawCorrectionValueLabel = new QLabel(panel);
    turnDebugAdvanceWallLimitedCorrectionValueLabel = new QLabel(panel);
    turnDebugAdvanceWallCorrectionValueLabel = new QLabel(panel);
    turnDebugWallKpValueLabel = new QLabel(panel);
    turnDebugWallKdValueLabel = new QLabel(panel);
    turnDebugWallDeadbandValueLabel = new QLabel(panel);
    turnDebugWallTargetLeftValueLabel = new QLabel(panel);
    turnDebugWallTargetRightValueLabel = new QLabel(panel);
    turnDebugWallCorrectionLimitValueLabel = new QLabel(panel);
    turnDebugLastCompletedActionValueLabel = new QLabel(panel);
    turnDebugLastSmoothDoneReasonValueLabel = new QLabel(panel);
    turnDebugLastSmoothFinalYawValueLabel = new QLabel(panel);
    turnDebugLastSmoothFinalRearValueLabel = new QLabel(panel);
    turnDebugLastAdvanceDoneReasonValueLabel = new QLabel(panel);
    turnDebugLastAdvanceFinalYawValueLabel = new QLabel(panel);
    turnDebugLastAdvanceFinalRearValueLabel = new QLabel(panel);
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
    navAutonomyEnabledValueLabel = new QLabel(panel);
    navPolicyValueLabel = new QLabel(panel);
    navLastDecisionValueLabel = new QLabel(panel);
    navDecisionWallFrontValueLabel = new QLabel(panel);
    navDecisionWallLeftValueLabel = new QLabel(panel);
    navDecisionWallRightValueLabel = new QLabel(panel);
    navDecisionPointValidValueLabel = new QLabel(panel);
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
    configureTelemetryValueLabel(turnDebugAdvancePhaseValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawSetpointValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawMeasuredValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawOutputValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawKpValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawKiValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawKdValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceYawOutputLimitValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceBaseLeftValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceBaseRightValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceGuidanceModeValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceCorrectionSourceValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceDiagLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceDiagRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFollowLeftValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceFollowRightValidValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallLeftValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRightValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallErrorAfterDeadbandValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallPrevErrorValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallErrorDeltaValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallPTermValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallDTermValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallRawCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallLimitedCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugAdvanceWallCorrectionValueLabel);
    configureTelemetryValueLabel(turnDebugWallKpValueLabel);
    configureTelemetryValueLabel(turnDebugWallKdValueLabel);
    configureTelemetryValueLabel(turnDebugWallDeadbandValueLabel);
    configureTelemetryValueLabel(turnDebugWallTargetLeftValueLabel);
    configureTelemetryValueLabel(turnDebugWallTargetRightValueLabel);
    configureTelemetryValueLabel(turnDebugWallCorrectionLimitValueLabel);
    configureTelemetryValueLabel(turnDebugLastCompletedActionValueLabel);
    configureTelemetryValueLabel(turnDebugLastSmoothDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugLastSmoothFinalYawValueLabel);
    configureTelemetryValueLabel(turnDebugLastSmoothFinalRearValueLabel);
    configureTelemetryValueLabel(turnDebugLastAdvanceDoneReasonValueLabel);
    configureTelemetryValueLabel(turnDebugLastAdvanceFinalYawValueLabel);
    configureTelemetryValueLabel(turnDebugLastAdvanceFinalRearValueLabel);
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
    configureTelemetryValueLabel(navAutonomyEnabledValueLabel);
    configureTelemetryValueLabel(navPolicyValueLabel);
    configureTelemetryValueLabel(navLastDecisionValueLabel);
    configureTelemetryValueLabel(navDecisionWallFrontValueLabel);
    configureTelemetryValueLabel(navDecisionWallLeftValueLabel);
    configureTelemetryValueLabel(navDecisionWallRightValueLabel);
    configureTelemetryValueLabel(navDecisionPointValidValueLabel);
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
    layout->addRow("advance_phase:", turnDebugAdvancePhaseValueLabel);
    layout->addRow("advance_done_reason:", turnDebugAdvanceDoneReasonValueLabel);
    layout->addRow("advance_yaw_setpoint:", turnDebugAdvanceYawSetpointValueLabel);
    layout->addRow("advance_yaw_measured:", turnDebugAdvanceYawMeasuredValueLabel);
    layout->addRow("advance_yaw_error:", turnDebugAdvanceYawErrorValueLabel);
    layout->addRow("advance_yaw_pid_output:", turnDebugAdvanceYawOutputValueLabel);
    layout->addRow("advance_yaw_correction:", turnDebugAdvanceYawCorrectionValueLabel);
    layout->addRow("advance_yaw_kp:", turnDebugAdvanceYawKpValueLabel);
    layout->addRow("advance_yaw_ki:", turnDebugAdvanceYawKiValueLabel);
    layout->addRow("advance_yaw_kd:", turnDebugAdvanceYawKdValueLabel);
    layout->addRow("advance_yaw_output_limit_pwm:", turnDebugAdvanceYawOutputLimitValueLabel);
    layout->addRow("advance_base_left_pwm:", turnDebugAdvanceBaseLeftValueLabel);
    layout->addRow("advance_base_right_pwm:", turnDebugAdvanceBaseRightValueLabel);
    layout->addRow("advance_guidance_mode:", turnDebugAdvanceGuidanceModeValueLabel);
    layout->addRow("advance_final_correction_source:", turnDebugAdvanceCorrectionSourceValueLabel);
    layout->addRow("advance_wall_left_valid:", turnDebugAdvanceWallLeftValidValueLabel);
    layout->addRow("advance_wall_right_valid:", turnDebugAdvanceWallRightValidValueLabel);
    layout->addRow("advance_diag_left_valid:", turnDebugAdvanceDiagLeftValidValueLabel);
    layout->addRow("advance_diag_right_valid:", turnDebugAdvanceDiagRightValidValueLabel);
    layout->addRow("advance_follow_left_valid:", turnDebugAdvanceFollowLeftValidValueLabel);
    layout->addRow("advance_follow_right_valid:", turnDebugAdvanceFollowRightValidValueLabel);
    layout->addRow("advance_wall_left_mm:", turnDebugAdvanceWallLeftValueLabel);
    layout->addRow("advance_wall_right_mm:", turnDebugAdvanceWallRightValueLabel);
    layout->addRow("advance_wall_error_mm:", turnDebugAdvanceWallErrorValueLabel);
    layout->addRow("advance_wall_error_after_deadband_mm:", turnDebugAdvanceWallErrorAfterDeadbandValueLabel);
    layout->addRow("advance_wall_prev_error_mm:", turnDebugAdvanceWallPrevErrorValueLabel);
    layout->addRow("advance_wall_error_delta_mm:", turnDebugAdvanceWallErrorDeltaValueLabel);
    layout->addRow("advance_wall_p_term_pwm:", turnDebugAdvanceWallPTermValueLabel);
    layout->addRow("advance_wall_d_term_pwm:", turnDebugAdvanceWallDTermValueLabel);
    layout->addRow("advance_wall_raw_correction_pwm:", turnDebugAdvanceWallRawCorrectionValueLabel);
    layout->addRow("advance_wall_limited_correction_pwm:", turnDebugAdvanceWallLimitedCorrectionValueLabel);
    layout->addRow("advance_wall_correction_pwm:", turnDebugAdvanceWallCorrectionValueLabel);
    layout->addRow("wall_kp:", turnDebugWallKpValueLabel);
    layout->addRow("wall_kd:", turnDebugWallKdValueLabel);
    layout->addRow("wall_error_deadband_mm:", turnDebugWallDeadbandValueLabel);
    layout->addRow("wall_follow_target_left_mm:", turnDebugWallTargetLeftValueLabel);
    layout->addRow("wall_follow_target_right_mm:", turnDebugWallTargetRightValueLabel);
    layout->addRow("wall_correction_limit_pwm:", turnDebugWallCorrectionLimitValueLabel);
    layout->addRow("last_completed_action:", turnDebugLastCompletedActionValueLabel);
    layout->addRow("last_smooth_done_reason:", turnDebugLastSmoothDoneReasonValueLabel);
    layout->addRow("last_smooth_final_yaw:", turnDebugLastSmoothFinalYawValueLabel);
    layout->addRow("last_smooth_final_rear:", turnDebugLastSmoothFinalRearValueLabel);
    layout->addRow("last_advance_done_reason:", turnDebugLastAdvanceDoneReasonValueLabel);
    layout->addRow("last_advance_final_yaw:", turnDebugLastAdvanceFinalYawValueLabel);
    layout->addRow("last_advance_final_rear:", turnDebugLastAdvanceFinalRearValueLabel);

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

    layout->addRow(navAutonomyTitle);
    layout->addRow("nav_autonomy_enabled:", navAutonomyEnabledValueLabel);
    layout->addRow("nav_policy:", navPolicyValueLabel);
    layout->addRow("nav_last_decision:", navLastDecisionValueLabel);
    layout->addRow("nav_decision_wall_front:", navDecisionWallFrontValueLabel);
    layout->addRow("nav_decision_wall_left:", navDecisionWallLeftValueLabel);
    layout->addRow("nav_decision_wall_right:", navDecisionWallRightValueLabel);
    layout->addRow("nav_decision_point_valid:", navDecisionPointValidValueLabel);

    layout->addRow(motorTestTitle);
    layout->addRow("test_left_pwm:", motorTestLeftValueLabel);
    layout->addRow("test_right_pwm:", motorTestRightValueLabel);

    panel->setLayout(layout);
    scrollArea->setWidgetResizable(true);
    scrollArea->setWidget(panel);
    dock->setWidget(scrollArea);
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
    if (turnDebugAdvancePhaseValueLabel) {
        turnDebugAdvancePhaseValueLabel->setText(advancePhaseText(turnDebug.advance_phase));
    }
    if (turnDebugAdvanceDoneReasonValueLabel) {
        turnDebugAdvanceDoneReasonValueLabel->setText(
            advanceDoneReasonText(turnDebug.advance_done_reason));
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
    if (turnDebugAdvanceWallLeftValueLabel) {
        turnDebugAdvanceWallLeftValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_left_mm_q16), 0, 'f', 1));
    }
    if (turnDebugAdvanceWallRightValueLabel) {
        turnDebugAdvanceWallRightValueLabel->setText(
            QString("%1 mm").arg(fromQ16(turnDebug.advance_wall_right_mm_q16), 0, 'f', 1));
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
    if (navAutonomyEnabledValueLabel) {
        navAutonomyEnabledValueLabel->setText(basicNavAutonomyEnabled ? "true" : "false");
    }
    if (navPolicyValueLabel) {
        navPolicyValueLabel->setText("RIGHT_HAND_RULE");
    }
    if (navLastDecisionValueLabel) {
        navLastDecisionValueLabel->setText(basicNavDecisionText(basicNavLastDecision));
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

    resetNavigationYawReference();
    const TestSequenceStep step = kTestSequence[testSequenceIndex];
    if (step == TestSequenceStep::AdvanceUntilRearBlack) {
        nav_core_start_advance_until_rear_black();
        return;
    }

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
    setBasicNavAutonomyEnabled(!basicNavAutonomyEnabled);
    if (basicNavAutonomyEnabled) {
        cancelTestSequence();
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
        basicNavLastDecision = BasicNavDecisionNone;
        basicNavDecisionWallFront = false;
        basicNavDecisionWallLeft = false;
        basicNavDecisionWallRight = false;
        basicNavDecisionPointValid = false;
    }
}

void MainWindow::advanceBasicNavAutonomyIfNeeded()
{
    if (!basicNavAutonomyEnabled) {
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
    startBasicNavActionFromPerception(perception);
}

void MainWindow::startBasicNavActionFromPerception(const NavWallPerception &perception)
{
    basicNavDecisionWallFront = perception.wall_front;
    basicNavDecisionWallLeft = perception.wall_left;
    basicNavDecisionWallRight = perception.wall_right;

    resetNavigationYawReference();
    RobotSensors sensors = buildRobotSensorsSnapshot();
    basicNavDecisionPointValid = sensors.floor_rear_black;
    if (!basicNavDecisionPointValid) {
        if (perception.wall_front) {
            nav_core_start_pivot_turn_180(&sensors);
            basicNavLastDecision = BasicNavDecisionRecoveryPivot180FrontBlocked;
            return;
        }

        nav_core_start_advance_until_rear_black();
        basicNavLastDecision = BasicNavDecisionAcquireRearLine;
        return;
    }

    if (!perception.wall_right) {
        nav_core_start_smooth_turn_right(&sensors);
        basicNavLastDecision = BasicNavDecisionSmoothRight;
        return;
    }

    if (!perception.wall_front) {
        nav_core_start_advance_until_rear_black();
        basicNavLastDecision = BasicNavDecisionAdvance;
        return;
    }

    if (!perception.wall_left) {
        nav_core_start_smooth_turn_left(&sensors);
        basicNavLastDecision = BasicNavDecisionSmoothLeft;
        return;
    }

    nav_core_start_pivot_turn_180(&sensors);
    basicNavLastDecision = BasicNavDecisionPivot180;
}

void MainWindow::resetRobotPoseToWorldStart()
{
    robot.setPose(world.startXMm(), world.startYMm(), world.startYawDeg());
    resetNavigationYawReference();
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

void MainWindow::showControlTuningDialog()
{
    QDialog dialog(this);
    dialog.setWindowTitle("Control Tuning");

    auto *rootLayout = new QVBoxLayout(&dialog);

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

    rootLayout->addWidget(turnGroup);
    rootLayout->addWidget(advanceYawGroup);
    rootLayout->addWidget(wallGroup);

    auto *buttons = new QDialogButtonBox(&dialog);
    QPushButton *applyButton = buttons->addButton(QDialogButtonBox::Apply);
    QPushButton *resetButton = buttons->addButton(QDialogButtonBox::Reset);
    QPushButton *closeButton = buttons->addButton(QDialogButtonBox::Close);
    rootLayout->addWidget(buttons);

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
        updateTelemetryPanel();
    };

    connect(applyButton, &QPushButton::clicked, this, applyValues);
    connect(resetButton, &QPushButton::clicked, this, [&]() {
        nav_core_reset_turn_pid_defaults();
        nav_core_reset_advance_yaw_pid_defaults();
        nav_core_reset_advance_wall_defaults();
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
        "- B: Toggle basic right-hand autonomous navigation; if not on rear line, acquire line first\n"
        "- C: Toggle ADVANCE guidance WALL_ASSIST / YAW_ONLY\n"
        "\n"
        "Motor test:\n"
        "- T: Toggle motor test mode\n"
        "- I: test PWM {3000, 3000} avanzar\n"
        "- K: test PWM {-3000, -3000} retroceder\n"
        "- J: test PWM {-1500, 1500} girar izquierda\n"
        "- L: test PWM {1500, -1500} girar derecha\n"
        "- U: stop test motors {0, 0}\n"
        "\n"
        "Smooth tuning:\n"
        "- +: increase smooth target yaw rate by 5 deg/s\n"
        "- -: decrease smooth target yaw rate by 5 deg/s\n"
        "- F2: set exact smooth target yaw rate\n"
        "- F3: open Control Tuning window\n"
        "\n"
        "Navigation test:\n"
        "- G: start ADVANCE_LINE until rear floor sensor detects black\n"
        "- Q: start SMOOTH_LEFT test, resets nav yaw reference\n"
        "- E: start SMOOTH_RIGHT test, resets nav yaw reference\n"
        "- 1: start PIVOT_LEFT test, resets nav yaw reference\n"
        "- 2: start PIVOT_RIGHT test, resets nav yaw reference\n"
        "- 3: start PIVOT_180 test, resets nav yaw reference\n"
        "- X: stop navigation action\n"
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
}

void MainWindow::simulationStep()
{
    ++simulationStepCount;
    simulationTimeS += kSimulationDtS;

    advanceTestSequenceIfNeeded();
    updateIrSensors();
    updateFloorSensors();
    updateNavCorePipeline();
    advanceTestSequenceIfNeeded();
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
    navYawZeroDeg = robot.yawDeg();
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
