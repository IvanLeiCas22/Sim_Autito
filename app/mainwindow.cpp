#include "mainwindow.h"

#include "nav_core.h"

#include <QAction>
#include <QBrush>
#include <QColor>
#include <QDockWidget>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QKeyEvent>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QTimer>
#include <QWidget>

#include <cmath>
#include <limits>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kManualMoveStepMm = 10.0;
constexpr double kManualRotateStepDeg = 10.0;
constexpr double kIrMaxDistanceMm = 130.0;
constexpr double kIrHitRadiusMm = 5.0;
constexpr double kFloorSensorRadiusMm = 7.0;
constexpr int kSimulationIntervalMs = 10;
constexpr double kSimulationDtS = kSimulationIntervalMs / 1000.0;

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
    setupScene();
    createTelemetryPanel();

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
        robot.resetPose();
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
    case Qt::Key_G:
        nav_core_start_advance_until_rear_black();
        updateNavCorePipeline();
        updateTelemetryPanel();
        robotPoseChanged = false;
        break;
    case Qt::Key_X:
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

void MainWindow::drawReferenceGrid()
{
    const double cellSizeMm = world.cellSizeMm();
    const double mazeWidthMm = world.cols() * cellSizeMm;
    const double mazeHeightMm = world.rows() * cellSizeMm;
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
    dock->setMinimumWidth(260);
    dock->setMaximumWidth(260);
    panel->setMinimumWidth(240);

    auto *poseTitle = new QLabel("<b>Pose</b>", panel);
    auto *irTitle = new QLabel("<b>IR sensors</b>", panel);
    auto *floorTitle = new QLabel("<b>Floor sensors</b>", panel);
    auto *navTitle = new QLabel("<b>Nav snapshot</b>", panel);
    auto *navCommandTitle = new QLabel("<b>Nav command</b>", panel);
    auto *simulationTitle = new QLabel("<b>Simulation</b>", panel);
    auto *motorTestTitle = new QLabel("<b>Motor test command</b>", panel);

    xValueLabel = new QLabel(panel);
    yValueLabel = new QLabel(panel);
    yawValueLabel = new QLabel(panel);
    xValueLabel->setMinimumWidth(150);
    yValueLabel->setMinimumWidth(150);
    yawValueLabel->setMinimumWidth(150);

    layout->addRow(poseTitle);
    layout->addRow("x:", xValueLabel);
    layout->addRow("y:", yValueLabel);
    layout->addRow("yaw:", yawValueLabel);

    layout->addRow(irTitle);
    for (IrSensor &sensor : irSensors) {
        sensor.value_label = new QLabel(panel);
        sensor.value_label->setMinimumWidth(150);
        layout->addRow(sensor.name, sensor.value_label);
    }

    layout->addRow(floorTitle);
    for (FloorSensor &sensor : floorSensors) {
        sensor.value_label = new QLabel(panel);
        sensor.value_label->setMinimumWidth(150);
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
    navLeftMotorValueLabel = new QLabel(panel);
    navRightMotorValueLabel = new QLabel(panel);
    simulationRunningValueLabel = new QLabel(panel);
    simulationDtValueLabel = new QLabel(panel);
    autoModeValueLabel = new QLabel(panel);
    simulationStepCountValueLabel = new QLabel(panel);
    simulationTimeValueLabel = new QLabel(panel);
    motorTestModeValueLabel = new QLabel(panel);
    motorTestLeftValueLabel = new QLabel(panel);
    motorTestRightValueLabel = new QLabel(panel);
    navFloorFrontValueLabel->setMinimumWidth(150);
    navFloorRearValueLabel->setMinimumWidth(150);
    navYawValueLabel->setMinimumWidth(150);
    navYawRateValueLabel->setMinimumWidth(150);
    navYawZeroValueLabel->setMinimumWidth(150);
    navStateValueLabel->setMinimumWidth(150);
    navActionValueLabel->setMinimumWidth(150);
    navActionStartYawValueLabel->setMinimumWidth(150);
    navActionTargetYawValueLabel->setMinimumWidth(150);
    navLeftMotorValueLabel->setMinimumWidth(150);
    navRightMotorValueLabel->setMinimumWidth(150);
    simulationRunningValueLabel->setMinimumWidth(150);
    simulationDtValueLabel->setMinimumWidth(150);
    autoModeValueLabel->setMinimumWidth(150);
    simulationStepCountValueLabel->setMinimumWidth(150);
    simulationTimeValueLabel->setMinimumWidth(150);
    motorTestModeValueLabel->setMinimumWidth(150);
    motorTestLeftValueLabel->setMinimumWidth(150);
    motorTestRightValueLabel->setMinimumWidth(150);

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

    layout->addRow(motorTestTitle);
    layout->addRow("test_left_pwm:", motorTestLeftValueLabel);
    layout->addRow("test_right_pwm:", motorTestRightValueLabel);

    panel->setLayout(layout);
    dock->setWidget(panel);
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
    if (motorTestLeftValueLabel) {
        motorTestLeftValueLabel->setText(QString::number(motorTestCommand.left_motor_pwm));
    }
    if (motorTestRightValueLabel) {
        motorTestRightValueLabel->setText(QString::number(motorTestCommand.right_motor_pwm));
    }
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
        "\n"
        "Motor test:\n"
        "- T: Toggle motor test mode\n"
        "- I: test PWM {3000, 3000} avanzar\n"
        "- K: test PWM {-3000, -3000} retroceder\n"
        "- J: test PWM {-1500, 1500} girar izquierda\n"
        "- L: test PWM {1500, -1500} girar derecha\n"
        "- U: stop test motors {0, 0}\n"
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

    updateIrSensors();
    updateFloorSensors();
    updateNavCorePipeline();

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
