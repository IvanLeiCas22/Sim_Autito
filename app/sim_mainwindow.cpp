#include "sim_mainwindow.h"

#include <QAction>
#include <QBrush>
#include <QColor>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QKeySequence>
#include <QHBoxLayout>
#include <QFont>
#include <QMenuBar>
#include <QPainter>
#include <QPen>
#include <QResizeEvent>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRobotLengthMm = 90.0;
constexpr double kRobotWidthMm = 80.0;
constexpr double kIrMaxDistanceMm = 150.0;
constexpr double kFloorSensorRadiusMm = 7.0;
constexpr int kSimulationIntervalMs = 10;

QString boolText(bool value)
{
    return value ? QStringLiteral("true") : QStringLiteral("false");
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    irReadings_ = {{
        {QStringLiteral("front_left"),  35.0, -20.0,   0.0, false, kIrMaxDistanceMm, 0.0, 0.0},
        {QStringLiteral("front_right"), 35.0,  20.0,   0.0, false, kIrMaxDistanceMm, 0.0, 0.0},
        {QStringLiteral("left"),         0.0, -38.0, -90.0, false, kIrMaxDistanceMm, 0.0, 0.0},
        {QStringLiteral("right"),        0.0,  38.0,  90.0, false, kIrMaxDistanceMm, 0.0, 0.0},
        {QStringLiteral("diag_left"),   25.0, -30.0, -45.0, false, kIrMaxDistanceMm, 0.0, 0.0},
        {QStringLiteral("diag_right"),  25.0,  30.0,  45.0, false, kIrMaxDistanceMm, 0.0, 0.0},
    }};

    floorFront_.name = QStringLiteral("front");
    floorFront_.local_x_mm = 42.0;
    floorFront_.local_y_mm = 0.0;

    floorRear_.name = QStringLiteral("rear");
    floorRear_.local_x_mm = -42.0;
    floorRear_.local_y_mm = 0.0;

    setupUi();
    setupActions();

    simulationTimer_ = new QTimer(this);
    simulationTimer_->setInterval(kSimulationIntervalMs);
    connect(simulationTimer_, &QTimer::timeout, this, [this]() {
        simulationStep();
    });

    resetSimulation();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    fitViewToWorld();
}

void MainWindow::setupUi()
{
    scene_ = new QGraphicsScene(this);
    view_ = new QGraphicsView(scene_, this);
    view_->setRenderHint(QPainter::Antialiasing, true);
    view_->setDragMode(QGraphicsView::ScrollHandDrag);

    setCentralWidget(view_);

    telemetryText_ = new QPlainTextEdit(this);
    telemetryText_->setReadOnly(true);
    telemetryText_->setMaximumBlockCount(2000);

    QFont telemetryFont = telemetryText_->font();
    telemetryFont.setFamily(QStringLiteral("Consolas"));
    telemetryFont.setPointSize(9);
    telemetryText_->setFont(telemetryFont);

    auto *dock = new QDockWidget(QStringLiteral("Telemetry"), this);
    dock->setWidget(telemetryText_);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    statusLabel_ = new QLabel(this);
    statusBar()->addPermanentWidget(statusLabel_);

    setWindowTitle(QStringLiteral("Sim_Autito - Firmware Bridge Stub"));
    resize(1200, 800);
}

void MainWindow::setupActions()
{
    auto *toolbar = addToolBar(QStringLiteral("Simulation"));
    toolbar->setMovable(false);

    auto *loadAction = new QAction(QStringLiteral("Load map"), this);
    auto *resetAction = new QAction(QStringLiteral("Reset"), this);
    auto *startAction = new QAction(QStringLiteral("Start"), this);
    auto *stopAction = new QAction(QStringLiteral("Stop"), this);
    auto *toggleAction = new QAction(QStringLiteral("Start/Stop"), this);

    loadAction->setShortcut(QKeySequence::Open);
    resetAction->setShortcut(QKeySequence(QStringLiteral("R")));
    startAction->setShortcut(QKeySequence(QStringLiteral("Space")));
    stopAction->setShortcut(QKeySequence(QStringLiteral("S")));
    toggleAction->setShortcut(QKeySequence(QStringLiteral("M")));

    connect(loadAction, &QAction::triggered, this, [this]() { loadMap(); });
    connect(resetAction, &QAction::triggered, this, [this]() { resetSimulation(); });
    connect(startAction, &QAction::triggered, this, [this]() { startSimulation(); });
    connect(stopAction, &QAction::triggered, this, [this]() { stopSimulation(); });
    connect(toggleAction, &QAction::triggered, this, [this]() { toggleSimulation(); });

    toolbar->addAction(loadAction);
    toolbar->addAction(resetAction);
    toolbar->addSeparator();
    toolbar->addAction(startAction);
    toolbar->addAction(stopAction);

    auto *fileMenu = menuBar()->addMenu(QStringLiteral("&File"));
    fileMenu->addAction(loadAction);

    auto *simulationMenu = menuBar()->addMenu(QStringLiteral("&Simulation"));
    simulationMenu->addAction(resetAction);
    simulationMenu->addAction(startAction);
    simulationMenu->addAction(stopAction);
    simulationMenu->addAction(toggleAction);

    addAction(loadAction);
    addAction(resetAction);
    addAction(startAction);
    addAction(stopAction);
    addAction(toggleAction);
}

void MainWindow::loadMap()
{
    QString startDir;
#ifdef SIM_AUTITO_SOURCE_DIR
    startDir = QDir(QStringLiteral(SIM_AUTITO_SOURCE_DIR)).filePath(QStringLiteral("data"));
#else
    startDir = QDir::current().filePath(QStringLiteral("data"));
#endif

    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Load maze JSON"),
        startDir,
        QStringLiteral("Maze JSON (*.json);;All files (*.*)"));

    if (path.isEmpty()) {
        return;
    }

    if (world_.loadFromJsonFile(path)) {
        resetSimulation();
    }
}

void MainWindow::resetSimulation()
{
    stopSimulation();

    robot_.setPose(world_.startXMm(), world_.startYMm(), world_.startYawDeg());
    firmwareBridge_.reset();
    lastCommand_ = FirmwareSimBridge::Command{};

    updateSensors();
    refreshScene();
    refreshTelemetry();
}

void MainWindow::startSimulation()
{
    if (simulationRunning_) {
        return;
    }

    simulationRunning_ = true;
    firmwareBridge_.start();
    simulationTimer_->start();
    refreshTelemetry();
}

void MainWindow::stopSimulation()
{
    if (!simulationRunning_) {
        firmwareBridge_.stop();
        return;
    }

    simulationRunning_ = false;
    simulationTimer_->stop();
    firmwareBridge_.stop();
    refreshTelemetry();
}

void MainWindow::toggleSimulation()
{
    if (simulationRunning_) {
        stopSimulation();
    } else {
        startSimulation();
    }
}

void MainWindow::simulationStep()
{
    updateSensors();

    const FirmwareSimBridge::SensorSnapshot snapshot = buildBridgeSnapshot();
    lastCommand_ = firmwareBridge_.tick(snapshot);

    if (simulationRunning_) {
        robot_.applyDifferentialDrive(
            static_cast<int16_t>(lastCommand_.left_pwm),
            static_cast<int16_t>(lastCommand_.right_pwm),
            kSimulationIntervalMs / 1000.0);
    }

    updateSensors();
    refreshScene();
    refreshTelemetry();
}

void MainWindow::updateSensors()
{
    for (IrSensorReading &reading : irReadings_) {
        const QPointF origin = robotLocalToWorld(reading.local_x_mm, reading.local_y_mm);
        const double rayAngleDeg = robot_.yawDeg() + reading.local_angle_deg;
        const RaycastHit hit = world_.castRay(origin.x(), origin.y(), rayAngleDeg, kIrMaxDistanceMm);

        reading.hit = hit.hit;
        reading.distance_mm = hit.distance_mm;
        reading.hit_x_mm = hit.hit_x_mm;
        reading.hit_y_mm = hit.hit_y_mm;
    }

    const QPointF front = robotLocalToWorld(floorFront_.local_x_mm, floorFront_.local_y_mm);
    floorFront_.world_x_mm = front.x();
    floorFront_.world_y_mm = front.y();
    floorFront_.black = world_.isBlackTapeAt(front.x(), front.y());

    const QPointF rear = robotLocalToWorld(floorRear_.local_x_mm, floorRear_.local_y_mm);
    floorRear_.world_x_mm = rear.x();
    floorRear_.world_y_mm = rear.y();
    floorRear_.black = world_.isBlackTapeAt(rear.x(), rear.y());
}

FirmwareSimBridge::SensorSnapshot MainWindow::buildBridgeSnapshot() const
{
    FirmwareSimBridge::SensorSnapshot snapshot;
    snapshot.dt_ms = kSimulationIntervalMs;
    snapshot.floor_front_black = floorFront_.black;
    snapshot.floor_rear_black = floorRear_.black;
    snapshot.yaw_deg = robot_.yawDeg();
    snapshot.yaw_rate_deg_s = robot_.yawRateDegS();

    for (int i = 0; i < kIrSensorCount; ++i) {
        snapshot.ir_distance_mm[static_cast<std::size_t>(i)] = irReadings_[static_cast<std::size_t>(i)].distance_mm;
    }

    return snapshot;
}

void MainWindow::refreshScene()
{
    scene_->clear();
    scene_->setSceneRect(0.0, 0.0, world_.widthMm(), world_.heightMm());

    const QPen gridPen(QColor(220, 220, 220));
    for (int col = 0; col <= world_.cols(); ++col) {
        const double x = col * world_.cellSizeMm();
        scene_->addLine(x, 0.0, x, world_.heightMm(), gridPen);
    }
    for (int row = 0; row <= world_.rows(); ++row) {
        const double y = row * world_.cellSizeMm();
        scene_->addLine(0.0, y, world_.widthMm(), y, gridPen);
    }

    const QBrush boundaryTapeBrush(QColor(30, 30, 30));
    const QBrush targetTapeBrush(QColor(80, 80, 80));
    const QPen noPen(Qt::NoPen);

    for (const SimRect &rect : world_.boundaryTapeRects()) {
        scene_->addRect(rect.x_mm, rect.y_mm, rect.w_mm, rect.h_mm, noPen, boundaryTapeBrush);
    }
    for (const SimRect &rect : world_.targetTapeRects()) {
        scene_->addRect(rect.x_mm, rect.y_mm, rect.w_mm, rect.h_mm, noPen, targetTapeBrush);
    }

    const QPen wallPen(QColor(20, 20, 20), 6.0, Qt::SolidLine, Qt::RoundCap);
    for (const SimLineSegment &segment : world_.wallSegments()) {
        scene_->addLine(segment.x1_mm, segment.y1_mm, segment.x2_mm, segment.y2_mm, wallPen);
    }

    const QPen rayPen(QColor(0, 120, 200), 1.0, Qt::DashLine);
    for (const IrSensorReading &reading : irReadings_) {
        const QPointF origin = robotLocalToWorld(reading.local_x_mm, reading.local_y_mm);
        scene_->addLine(origin.x(), origin.y(), reading.hit_x_mm, reading.hit_y_mm, rayPen);
        scene_->addEllipse(origin.x() - 2.0, origin.y() - 2.0, 4.0, 4.0, QPen(Qt::NoPen), QBrush(QColor(0, 120, 200)));
    }

    const QBrush floorFrontBrush(floorFront_.black ? QColor(0, 0, 0) : QColor(240, 240, 240));
    const QBrush floorRearBrush(floorRear_.black ? QColor(0, 0, 0) : QColor(240, 240, 240));
    scene_->addEllipse(floorFront_.world_x_mm - kFloorSensorRadiusMm,
                       floorFront_.world_y_mm - kFloorSensorRadiusMm,
                       2.0 * kFloorSensorRadiusMm,
                       2.0 * kFloorSensorRadiusMm,
                       QPen(QColor(20, 20, 20)),
                       floorFrontBrush);
    scene_->addEllipse(floorRear_.world_x_mm - kFloorSensorRadiusMm,
                       floorRear_.world_y_mm - kFloorSensorRadiusMm,
                       2.0 * kFloorSensorRadiusMm,
                       2.0 * kFloorSensorRadiusMm,
                       QPen(QColor(20, 20, 20)),
                       floorRearBrush);

    scene_->addPolygon(robotPolygon(), QPen(QColor(0, 70, 0), 2.0), QBrush(QColor(120, 220, 120, 160)));

    const QPointF nose = robotLocalToWorld(kRobotLengthMm / 2.0, 0.0);
    scene_->addLine(robot_.xMm(), robot_.yMm(), nose.x(), nose.y(), QPen(QColor(0, 100, 0), 3.0));

    fitViewToWorld();
}

void MainWindow::refreshTelemetry()
{
    const FirmwareSimBridge::Debug debug = firmwareBridge_.debug();

    QString text;
    text += QStringLiteral("Simulation\n");
    text += QStringLiteral("  running: %1\n").arg(boolText(simulationRunning_));
    text += QStringLiteral("  interval_ms: %1\n\n").arg(kSimulationIntervalMs);

    text += QStringLiteral("World\n");
    text += QStringLiteral("  map: %1\n").arg(world_.mazeName());
    text += QStringLiteral("  size: %1 cols x %2 rows\n").arg(world_.cols()).arg(world_.rows());
    text += QStringLiteral("  cell_size_mm: %1\n\n").arg(world_.cellSizeMm(), 0, 'f', 1);

    text += QStringLiteral("Robot pose\n");
    text += QStringLiteral("  x_mm: %1\n").arg(robot_.xMm(), 0, 'f', 2);
    text += QStringLiteral("  y_mm: %1\n").arg(robot_.yMm(), 0, 'f', 2);
    text += QStringLiteral("  yaw_deg: %1\n").arg(robot_.yawDeg(), 0, 'f', 2);
    text += QStringLiteral("  yaw_rate_deg_s: %1\n\n").arg(robot_.yawRateDegS(), 0, 'f', 2);

    text += QStringLiteral("FirmwareSimBridge\n");
    text += QStringLiteral("  state: %1\n").arg(debug.state);
    text += QStringLiteral("  reason: %1\n").arg(debug.reason);
    text += QStringLiteral("  enabled: %1\n").arg(boolText(debug.enabled));
    text += QStringLiteral("  left_pwm: %1\n").arg(lastCommand_.left_pwm);
    text += QStringLiteral("  right_pwm: %1\n\n").arg(lastCommand_.right_pwm);

    text += QStringLiteral("IR sensors\n");
    for (const IrSensorReading &reading : irReadings_) {
        text += QStringLiteral("  %1: hit=%2 dist_mm=%3\n")
            .arg(reading.name)
            .arg(boolText(reading.hit))
            .arg(reading.distance_mm, 0, 'f', 1);
    }

    text += QStringLiteral("\nFloor sensors\n");
    text += QStringLiteral("  front: black=%1 x=%2 y=%3\n")
        .arg(boolText(floorFront_.black))
        .arg(floorFront_.world_x_mm, 0, 'f', 1)
        .arg(floorFront_.world_y_mm, 0, 'f', 1);
    text += QStringLiteral("  rear: black=%1 x=%2 y=%3\n")
        .arg(boolText(floorRear_.black))
        .arg(floorRear_.world_x_mm, 0, 'f', 1)
        .arg(floorRear_.world_y_mm, 0, 'f', 1);

    telemetryText_->setPlainText(text);
    statusLabel_->setText(simulationRunning_
        ? QStringLiteral("Running")
        : QStringLiteral("Stopped"));
}

QPointF MainWindow::robotLocalToWorld(double local_x_mm, double local_y_mm) const
{
    const double yawRad = robot_.yawDeg() * kDegToRad;
    const double cosYaw = std::cos(yawRad);
    const double sinYaw = std::sin(yawRad);

    return QPointF(
        robot_.xMm() + cosYaw * local_x_mm - sinYaw * local_y_mm,
        robot_.yMm() + sinYaw * local_x_mm + cosYaw * local_y_mm);
}

QPolygonF MainWindow::robotPolygon() const
{
    QPolygonF polygon;
    polygon << robotLocalToWorld(kRobotLengthMm / 2.0, -kRobotWidthMm / 2.0);
    polygon << robotLocalToWorld(kRobotLengthMm / 2.0,  kRobotWidthMm / 2.0);
    polygon << robotLocalToWorld(-kRobotLengthMm / 2.0, kRobotWidthMm / 2.0);
    polygon << robotLocalToWorld(-kRobotLengthMm / 2.0, -kRobotWidthMm / 2.0);
    return polygon;
}

void MainWindow::fitViewToWorld()
{
    if (!view_ || !scene_) {
        return;
    }

    const QRectF sceneRect = scene_->sceneRect();
    if (!sceneRect.isEmpty()) {
        view_->fitInView(sceneRect.adjusted(-40.0, -40.0, 40.0, 40.0), Qt::KeepAspectRatio);
    }
}
