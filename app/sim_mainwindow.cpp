#include "sim_mainwindow.h"

#include <QAction>
#include <QBrush>
#include <QColor>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QGroupBox>
#include <QKeySequence>
#include <QList>
#include <QHBoxLayout>
#include <QFont>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPen>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QShowEvent>
#include <QSpinBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRobotLengthMm = 90.0;
constexpr double kRobotWidthMm = 80.0;
constexpr double kIrMaxDistanceMm = 150.0;
constexpr double kFloorSensorRadiusMm = 7.0;
constexpr int kSimulationIntervalMs = 10;
constexpr double kManualJogLinearStepMm = 20.0;
constexpr double kManualJogLinearFastStepMm = 100.0;
constexpr double kManualJogLinearFineStepMm = 5.0;
constexpr double kManualJogTurnStepDeg = 5.0;
constexpr double kManualJogTurnFastStepDeg = 20.0;
constexpr double kManualJogTurnFineStepDeg = 1.0;

QString boolText(bool value)
{
    return value ? QStringLiteral("true") : QStringLiteral("false");
}

QString tapeKindText(TapeDebugKind kind)
{
    switch (kind) {
    case TapeDebugKind::None:
        return QStringLiteral("none");
    case TapeDebugKind::Boundary:
        return QStringLiteral("boundary");
    case TapeDebugKind::Target:
        return QStringLiteral("target");
    case TapeDebugKind::BoundaryAndTarget:
        return QStringLiteral("boundary+target");
    }

    return QStringLiteral("unknown");
}

int q16ToHundredths(int32_t q16)
{
    return static_cast<int>(std::llround(static_cast<double>(q16) * 100.0 / 65536.0));
}

int32_t hundredthsToQ16(int value_x100)
{
    const double q16 = std::llround(static_cast<double>(value_x100) * 65536.0 / 100.0);
    return static_cast<int32_t>(std::clamp(q16, -2147483648.0, 2147483647.0));
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

    // The first fit must run after Qt has completed the initial layout.
    // Otherwise fitInView() can use a not-yet-final viewport and leave the
    // maze rendered as a tiny item until the user resizes/maximizes the window.
    QTimer::singleShot(0, this, [this]() {
        fitViewToWorld();
    });
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    QTimer::singleShot(0, this, [this]() {
        fitViewToWorld();
    });
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    fitViewToWorld();
}

void MainWindow::setupUi()
{
    scene_ = new QGraphicsScene(this);
    scene_->setBackgroundBrush(QBrush(Qt::white));

    view_ = new QGraphicsView(scene_, this);
    view_->setRenderHint(QPainter::Antialiasing, true);
    view_->setDragMode(QGraphicsView::ScrollHandDrag);
    view_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

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
    auto *fitAction = new QAction(QStringLiteral("Fit map"), this);
    auto *startStraightYawHoldAction = new QAction(QStringLiteral("Start straight yaw-hold"), this);
    auto *startWallFollowAdvanceAction = new QAction(QStringLiteral("Start wall-follow advance"), this);
    auto *stopFirmwareControlAction = new QAction(QStringLiteral("Stop firmware control"), this);
    auto *tuneFirmwareConfigAction = new QAction(QStringLiteral("Tune firmware PID/config"), this);

    loadAction->setShortcut(QKeySequence::Open);
    resetAction->setShortcut(QKeySequence(QStringLiteral("R")));
    startAction->setShortcut(QKeySequence(QStringLiteral("Space")));
    stopAction->setShortcut(QKeySequence(QStringLiteral("Esc")));
    toggleAction->setShortcut(QKeySequence(QStringLiteral("M")));
    fitAction->setShortcut(QKeySequence(QStringLiteral("F")));

    connect(loadAction, &QAction::triggered, this, [this]() { loadMap(); });
    connect(resetAction, &QAction::triggered, this, [this]() { resetSimulation(); });
    connect(startAction, &QAction::triggered, this, [this]() { startSimulation(); });
    connect(stopAction, &QAction::triggered, this, [this]() { stopSimulation(); });
    connect(toggleAction, &QAction::triggered, this, [this]() { toggleSimulation(); });
    connect(fitAction, &QAction::triggered, this, [this]() { fitViewToWorld(); });
    connect(startStraightYawHoldAction, &QAction::triggered, this, [this]() { startStraightYawHoldControl(); });
    connect(startWallFollowAdvanceAction, &QAction::triggered, this, [this]() { startWallFollowAdvanceControl(); });
    connect(stopFirmwareControlAction, &QAction::triggered, this, [this]() { stopFirmwareControl(); });
    connect(tuneFirmwareConfigAction, &QAction::triggered, this, [this]() { tuneFirmwareConfig(); });

    toolbar->addAction(loadAction);
    toolbar->addAction(resetAction);
    toolbar->addSeparator();
    toolbar->addAction(startAction);
    toolbar->addAction(stopAction);
    toolbar->addSeparator();
    toolbar->addAction(fitAction);

    auto *fileMenu = menuBar()->addMenu(QStringLiteral("&File"));
    fileMenu->addAction(loadAction);

    auto *simulationMenu = menuBar()->addMenu(QStringLiteral("&Simulation"));
    simulationMenu->addAction(resetAction);
    simulationMenu->addAction(startAction);
    simulationMenu->addAction(stopAction);
    simulationMenu->addAction(toggleAction);

    auto *viewMenu = menuBar()->addMenu(QStringLiteral("&View"));
    viewMenu->addAction(fitAction);

    auto *firmwareMenu = menuBar()->addMenu(QStringLiteral("&Firmware"));
    firmwareMenu->addAction(startStraightYawHoldAction);
    firmwareMenu->addAction(startWallFollowAdvanceAction);
    firmwareMenu->addAction(stopFirmwareControlAction);
    firmwareMenu->addSeparator();
    firmwareMenu->addAction(tuneFirmwareConfigAction);

    auto *manualMenu = menuBar()->addMenu(QStringLiteral("&Manual"));
    auto makeManualAction = [this, manualMenu](const QString &text,
                                               std::initializer_list<const char *> shortcuts,
                                               double distance_mm,
                                               double delta_yaw_deg) {
        auto *action = new QAction(text, this);
        QList<QKeySequence> sequences;
        for (const char *shortcut : shortcuts) {
            sequences.append(QKeySequence(QString::fromLatin1(shortcut)));
        }
        action->setShortcuts(sequences);
        action->setShortcutContext(Qt::ApplicationShortcut);
        connect(action, &QAction::triggered, this, [this, distance_mm, delta_yaw_deg]() {
            manualJog(distance_mm, delta_yaw_deg);
        });
        manualMenu->addAction(action);
        addAction(action);
        return action;
    };

    makeManualAction(QStringLiteral("Jog forward"), {"W", "Up"}, kManualJogLinearStepMm, 0.0);
    makeManualAction(QStringLiteral("Jog backward"), {"S", "Down"}, -kManualJogLinearStepMm, 0.0);
    makeManualAction(QStringLiteral("Turn left"), {"A", "Left"}, 0.0, -kManualJogTurnStepDeg);
    makeManualAction(QStringLiteral("Turn right"), {"D", "Right"}, 0.0, kManualJogTurnStepDeg);

    manualMenu->addSeparator();
    makeManualAction(QStringLiteral("Jog forward fast"), {"Shift+W", "Shift+Up"}, kManualJogLinearFastStepMm, 0.0);
    makeManualAction(QStringLiteral("Jog backward fast"), {"Shift+S", "Shift+Down"}, -kManualJogLinearFastStepMm, 0.0);
    makeManualAction(QStringLiteral("Turn left fast"), {"Shift+A", "Shift+Left"}, 0.0, -kManualJogTurnFastStepDeg);
    makeManualAction(QStringLiteral("Turn right fast"), {"Shift+D", "Shift+Right"}, 0.0, kManualJogTurnFastStepDeg);

    manualMenu->addSeparator();
    makeManualAction(QStringLiteral("Jog forward fine"), {"Ctrl+Up"}, kManualJogLinearFineStepMm, 0.0);
    makeManualAction(QStringLiteral("Jog backward fine"), {"Ctrl+Down"}, -kManualJogLinearFineStepMm, 0.0);
    makeManualAction(QStringLiteral("Turn left fine"), {"Ctrl+Left"}, 0.0, -kManualJogTurnFineStepDeg);
    makeManualAction(QStringLiteral("Turn right fine"), {"Ctrl+Right"}, 0.0, kManualJogTurnFineStepDeg);

    addAction(loadAction);
    addAction(resetAction);
    addAction(startAction);
    addAction(stopAction);
    addAction(toggleAction);
    addAction(fitAction);
    addAction(startStraightYawHoldAction);
    addAction(startWallFollowAdvanceAction);
    addAction(stopFirmwareControlAction);
    addAction(tuneFirmwareConfigAction);
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
    lastManualJogDescription_ = QStringLiteral("none");

    updateSensors();
    lastCommand_ = firmwareBridge_.tick(buildBridgeSnapshot());
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

void MainWindow::startStraightYawHoldControl()
{
    updateSensors();
    firmwareBridge_.startStraightYawHold(robot_.yawDeg());

    if (firmwareBridge_.debug().enabled) {
        simulationRunning_ = true;
        simulationTimer_->start();
        lastCommand_ = firmwareBridge_.tick(buildBridgeSnapshot());
    } else {
        simulationRunning_ = false;
        simulationTimer_->stop();
        lastCommand_ = FirmwareSimBridge::Command{};
    }

    refreshScene();
    refreshTelemetry();
}

void MainWindow::startWallFollowAdvanceControl()
{
    updateSensors();
    firmwareBridge_.startWallFollowAdvance();

    if (firmwareBridge_.debug().enabled) {
        simulationRunning_ = true;
        simulationTimer_->start();
        lastCommand_ = firmwareBridge_.tick(buildBridgeSnapshot());
    } else {
        simulationRunning_ = false;
        simulationTimer_->stop();
        lastCommand_ = FirmwareSimBridge::Command{};
    }

    refreshScene();
    refreshTelemetry();
}

void MainWindow::stopFirmwareControl()
{
    simulationRunning_ = false;
    simulationTimer_->stop();
    firmwareBridge_.stopControl();
    lastCommand_ = FirmwareSimBridge::Command{};

    updateSensors();
    lastCommand_ = firmwareBridge_.tick(buildBridgeSnapshot());
    refreshScene();
    refreshTelemetry();
}

void MainWindow::tuneFirmwareConfig()
{
    FirmwareSimBridge::FirmwareConfig currentConfig;
    if (!firmwareBridge_.getFirmwareConfig(&currentConfig)) {
        QMessageBox::information(this,
                                 QStringLiteral("Firmware config"),
                                 QStringLiteral("Firmware core not available"));
        return;
    }

    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("Tune firmware PID/config"));
    dialog.resize(520, 720);

    auto *rootLayout = new QVBoxLayout(&dialog);
    auto *scrollArea = new QScrollArea(&dialog);
    scrollArea->setWidgetResizable(true);

    auto *content = new QWidget(scrollArea);
    auto *contentLayout = new QVBoxLayout(content);

    auto makeGroup = [contentLayout](const QString &title) {
        auto *group = new QGroupBox(title);
        auto *layout = new QFormLayout(group);
        contentLayout->addWidget(group);
        return layout;
    };

    auto makeSpin = [](int minimum, int maximum, int value) {
        auto *spin = new QSpinBox;
        spin->setRange(minimum, maximum);
        spin->setValue(value);
        return spin;
    };

    auto addSpin = [makeSpin](QFormLayout *layout,
                              const QString &label,
                              int minimum,
                              int maximum,
                              int value) {
        QSpinBox *spin = makeSpin(minimum, maximum, value);
        layout->addRow(label, spin);
        return spin;
    };

    struct PidEditors
    {
        QSpinBox *kp = nullptr;
        QSpinBox *ki = nullptr;
        QSpinBox *kd = nullptr;
        QSpinBox *limit = nullptr;
    };

    auto addPidGroup = [addSpin](QFormLayout *layout,
                                 int32_t kp_q16,
                                 int32_t ki_q16,
                                 int32_t kd_q16,
                                 int32_t limit_pwm) {
        PidEditors editors;
        editors.kp = addSpin(layout, QStringLiteral("Kp x100"), -100000, 100000, q16ToHundredths(kp_q16));
        editors.ki = addSpin(layout, QStringLiteral("Ki x100"), -100000, 100000, q16ToHundredths(ki_q16));
        editors.kd = addSpin(layout, QStringLiteral("Kd x100"), -100000, 100000, q16ToHundredths(kd_q16));
        editors.limit = addSpin(layout, QStringLiteral("output limit pwm"), 0, 20000, limit_pwm);
        return editors;
    };

    QFormLayout *advanceLayout = makeGroup(QStringLiteral("Advance / yaw-hold"));
    PidEditors advancePid = addPidGroup(advanceLayout,
                                        currentConfig.advance_pid_kp_q16,
                                        currentConfig.advance_pid_ki_q16,
                                        currentConfig.advance_pid_kd_q16,
                                        currentConfig.advance_pid_output_limit_pwm);

    QFormLayout *smoothLayout = makeGroup(QStringLiteral("Smooth turn"));
    PidEditors smoothPid = addPidGroup(smoothLayout,
                                       currentConfig.smooth_turn_pid_kp_q16,
                                       currentConfig.smooth_turn_pid_ki_q16,
                                       currentConfig.smooth_turn_pid_kd_q16,
                                       currentConfig.smooth_turn_pid_output_limit_pwm);

    QFormLayout *pivotLayout = makeGroup(QStringLiteral("Pivot turn"));
    PidEditors pivotPid = addPidGroup(pivotLayout,
                                      currentConfig.pivot_turn_pid_kp_q16,
                                      currentConfig.pivot_turn_pid_ki_q16,
                                      currentConfig.pivot_turn_pid_kd_q16,
                                      currentConfig.pivot_turn_pid_output_limit_pwm);

    QFormLayout *brakingLayout = makeGroup(QStringLiteral("Braking"));
    PidEditors brakingPid = addPidGroup(brakingLayout,
                                        currentConfig.braking_pid_kp_q16,
                                        currentConfig.braking_pid_ki_q16,
                                        currentConfig.braking_pid_kd_q16,
                                        currentConfig.braking_pid_output_limit_pwm);
    QSpinBox *brakingMinSpeed = addSpin(brakingLayout,
                                        QStringLiteral("braking min speed pwm"),
                                        -32768,
                                        32767,
                                        currentConfig.braking_min_speed_pwm);

    QFormLayout *baseLayout = makeGroup(QStringLiteral("Bases PWM"));
    QSpinBox *leftBase = addSpin(baseLayout,
                                 QStringLiteral("left motor base speed"),
                                 0,
                                 65535,
                                 currentConfig.left_motor_base_speed);
    QSpinBox *rightBase = addSpin(baseLayout,
                                  QStringLiteral("right motor base speed"),
                                  0,
                                  65535,
                                  currentConfig.right_motor_base_speed);
    QSpinBox *fasterSmooth = addSpin(baseLayout,
                                     QStringLiteral("faster smooth turn speed"),
                                     0,
                                     65535,
                                     currentConfig.faster_motor_smooth_turn_speed);
    QSpinBox *slowerSmooth = addSpin(baseLayout,
                                     QStringLiteral("slower smooth turn speed"),
                                     0,
                                     65535,
                                     currentConfig.slower_motor_smooth_turn_speed);
    QSpinBox *turnTarget = addSpin(baseLayout,
                                   QStringLiteral("turn target dps"),
                                   0,
                                   65535,
                                   currentConfig.turn_target_dps);
    QSpinBox *pivotTurnTarget = addSpin(baseLayout,
                                        QStringLiteral("pivot turn target dps"),
                                        0,
                                        65535,
                                        currentConfig.pivot_turn_target_dps);

    contentLayout->addStretch();
    scrollArea->setWidget(content);
    rootLayout->addWidget(scrollArea);

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok
                                         | QDialogButtonBox::Cancel
                                         | QDialogButtonBox::Apply,
                                         &dialog);
    QPushButton *resetButton = buttons->addButton(QStringLiteral("Reset simulation defaults"),
                                                  QDialogButtonBox::ResetRole);
    rootLayout->addWidget(buttons);

    auto collectConfig = [&]() {
        FirmwareSimBridge::FirmwareConfig config = currentConfig;
        config.advance_pid_kp_q16 = hundredthsToQ16(advancePid.kp->value());
        config.advance_pid_ki_q16 = hundredthsToQ16(advancePid.ki->value());
        config.advance_pid_kd_q16 = hundredthsToQ16(advancePid.kd->value());
        config.advance_pid_output_limit_pwm = advancePid.limit->value();

        config.smooth_turn_pid_kp_q16 = hundredthsToQ16(smoothPid.kp->value());
        config.smooth_turn_pid_ki_q16 = hundredthsToQ16(smoothPid.ki->value());
        config.smooth_turn_pid_kd_q16 = hundredthsToQ16(smoothPid.kd->value());
        config.smooth_turn_pid_output_limit_pwm = smoothPid.limit->value();

        config.pivot_turn_pid_kp_q16 = hundredthsToQ16(pivotPid.kp->value());
        config.pivot_turn_pid_ki_q16 = hundredthsToQ16(pivotPid.ki->value());
        config.pivot_turn_pid_kd_q16 = hundredthsToQ16(pivotPid.kd->value());
        config.pivot_turn_pid_output_limit_pwm = pivotPid.limit->value();

        config.braking_pid_kp_q16 = hundredthsToQ16(brakingPid.kp->value());
        config.braking_pid_ki_q16 = hundredthsToQ16(brakingPid.ki->value());
        config.braking_pid_kd_q16 = hundredthsToQ16(brakingPid.kd->value());
        config.braking_pid_output_limit_pwm = brakingPid.limit->value();
        config.braking_min_speed_pwm = static_cast<int16_t>(brakingMinSpeed->value());

        config.left_motor_base_speed = static_cast<uint16_t>(leftBase->value());
        config.right_motor_base_speed = static_cast<uint16_t>(rightBase->value());
        config.faster_motor_smooth_turn_speed = static_cast<uint16_t>(fasterSmooth->value());
        config.slower_motor_smooth_turn_speed = static_cast<uint16_t>(slowerSmooth->value());
        config.turn_target_dps = static_cast<uint16_t>(turnTarget->value());
        config.pivot_turn_target_dps = static_cast<uint16_t>(pivotTurnTarget->value());
        return config;
    };

    auto loadConfigIntoWidgets = [&](const FirmwareSimBridge::FirmwareConfig &config) {
        currentConfig = config;
        advancePid.kp->setValue(q16ToHundredths(config.advance_pid_kp_q16));
        advancePid.ki->setValue(q16ToHundredths(config.advance_pid_ki_q16));
        advancePid.kd->setValue(q16ToHundredths(config.advance_pid_kd_q16));
        advancePid.limit->setValue(config.advance_pid_output_limit_pwm);

        smoothPid.kp->setValue(q16ToHundredths(config.smooth_turn_pid_kp_q16));
        smoothPid.ki->setValue(q16ToHundredths(config.smooth_turn_pid_ki_q16));
        smoothPid.kd->setValue(q16ToHundredths(config.smooth_turn_pid_kd_q16));
        smoothPid.limit->setValue(config.smooth_turn_pid_output_limit_pwm);

        pivotPid.kp->setValue(q16ToHundredths(config.pivot_turn_pid_kp_q16));
        pivotPid.ki->setValue(q16ToHundredths(config.pivot_turn_pid_ki_q16));
        pivotPid.kd->setValue(q16ToHundredths(config.pivot_turn_pid_kd_q16));
        pivotPid.limit->setValue(config.pivot_turn_pid_output_limit_pwm);

        brakingPid.kp->setValue(q16ToHundredths(config.braking_pid_kp_q16));
        brakingPid.ki->setValue(q16ToHundredths(config.braking_pid_ki_q16));
        brakingPid.kd->setValue(q16ToHundredths(config.braking_pid_kd_q16));
        brakingPid.limit->setValue(config.braking_pid_output_limit_pwm);
        brakingMinSpeed->setValue(config.braking_min_speed_pwm);

        leftBase->setValue(config.left_motor_base_speed);
        rightBase->setValue(config.right_motor_base_speed);
        fasterSmooth->setValue(config.faster_motor_smooth_turn_speed);
        slowerSmooth->setValue(config.slower_motor_smooth_turn_speed);
        turnTarget->setValue(config.turn_target_dps);
        pivotTurnTarget->setValue(config.pivot_turn_target_dps);
    };

    auto applyConfig = [&]() {
        const FirmwareSimBridge::FirmwareConfig config = collectConfig();
        if (!firmwareBridge_.setFirmwareConfig(config)) {
            QMessageBox::warning(&dialog,
                                 QStringLiteral("Firmware config"),
                                 QStringLiteral("Firmware core not available"));
            return false;
        }

        currentConfig = config;
        refreshTelemetry();
        return true;
    };

    connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked, &dialog, [&]() {
        applyConfig();
    });
    connect(buttons, &QDialogButtonBox::accepted, &dialog, [&]() {
        if (applyConfig()) {
            dialog.accept();
        }
    });
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    connect(resetButton, &QPushButton::clicked, &dialog, [&]() {
        if (!firmwareBridge_.resetFirmwareConfigToSimulationDefaults()
            || !firmwareBridge_.getFirmwareConfig(&currentConfig)) {
            QMessageBox::warning(&dialog,
                                 QStringLiteral("Firmware config"),
                                 QStringLiteral("Firmware core not available"));
            return;
        }

        loadConfigIntoWidgets(currentConfig);
        refreshTelemetry();
    });

    dialog.exec();
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

void MainWindow::manualJog(double distance_mm, double delta_yaw_deg)
{
    const bool perturbationMode = simulationRunning_ && firmwareBridge_.isFirmwareControlActive();

    if (simulationRunning_ && !perturbationMode) {
        stopSimulation();
    }

    if (std::abs(distance_mm) > 0.001) {
        robot_.moveForward(distance_mm);
    }

    if (std::abs(delta_yaw_deg) > 0.001) {
        robot_.rotate(delta_yaw_deg);
    }

    lastCommand_ = FirmwareSimBridge::Command{};

    if (std::abs(distance_mm) > 0.001) {
        lastManualJogDescription_ = QStringLiteral("linear %1 mm").arg(distance_mm, 0, 'f', 1);
    } else if (std::abs(delta_yaw_deg) > 0.001) {
        lastManualJogDescription_ = QStringLiteral("turn %1 deg").arg(delta_yaw_deg, 0, 'f', 1);
    } else {
        lastManualJogDescription_ = QStringLiteral("none");
    }

    updateSensors();
    lastCommand_ = firmwareBridge_.tick(buildBridgeSnapshot());
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
    floorFront_.debug_kind = world_.debugTapeKindAt(front.x(), front.y());

    const QPointF rear = robotLocalToWorld(floorRear_.local_x_mm, floorRear_.local_y_mm);
    floorRear_.world_x_mm = rear.x();
    floorRear_.world_y_mm = rear.y();
    floorRear_.black = world_.isBlackTapeAt(rear.x(), rear.y());
    floorRear_.debug_kind = world_.debugTapeKindAt(rear.x(), rear.y());
}

FirmwareSimBridge::SensorSnapshot MainWindow::buildBridgeSnapshot() const
{
    FirmwareSimBridge::SensorSnapshot snapshot;
    snapshot.dt_ms = kSimulationIntervalMs;
    snapshot.floor_front_black = floorFront_.black;
    snapshot.floor_rear_black = floorRear_.black;
    snapshot.yaw_deg = robot_.yawDeg();
    snapshot.yaw_rate_deg_s = robot_.yawRateDegS();
    snapshot.left_motor_gain = robot_.leftMotorGain();
    snapshot.right_motor_gain = robot_.rightMotorGain();

    for (int i = 0; i < kIrSensorCount; ++i) {
        snapshot.ir_distance_mm[static_cast<std::size_t>(i)] = irReadings_[static_cast<std::size_t>(i)].distance_mm;
    }

    return snapshot;
}

void MainWindow::refreshScene()
{
    scene_->clear();
    scene_->setSceneRect(0.0, 0.0, world_.widthMm(), world_.heightMm());

    // Visual convention:
    //   reference grid       = medium gray, thin
    //   boundary tape lines  = translucent gray
    //   target/special tape  = darker translucent gray
    //   physical walls       = solid black, thick, drawn above tape
    //   IR rays              = orange, with a visible hit marker only on hit
    const QPen gridPen(QColor(160, 160, 160), 1.0);
    for (int col = 0; col <= world_.cols(); ++col) {
        const double x = col * world_.cellSizeMm();
        QGraphicsLineItem *item = scene_->addLine(x, 0.0, x, world_.heightMm(), gridPen);
        item->setZValue(0.0);
    }
    for (int row = 0; row <= world_.rows(); ++row) {
        const double y = row * world_.cellSizeMm();
        QGraphicsLineItem *item = scene_->addLine(0.0, y, world_.widthMm(), y, gridPen);
        item->setZValue(0.0);
    }

    const QBrush boundaryTapeBrush(QColor(95, 95, 95, 150));
    const QBrush targetTapeBrush(QColor(35, 35, 35, 210));
    const QPen noPen(Qt::NoPen);

    for (const SimRect &rect : world_.boundaryTapeRects()) {
        QGraphicsRectItem *item = scene_->addRect(rect.x_mm, rect.y_mm, rect.w_mm, rect.h_mm, noPen, boundaryTapeBrush);
        item->setZValue(1.0);
    }
    for (const SimRect &rect : world_.targetTapeRects()) {
        QGraphicsRectItem *item = scene_->addRect(rect.x_mm, rect.y_mm, rect.w_mm, rect.h_mm, noPen, targetTapeBrush);
        item->setZValue(2.0);
    }

    QPen wallPen(QColor(0, 0, 0), 12.0, Qt::SolidLine, Qt::SquareCap);
    for (const SimLineSegment &segment : world_.wallSegments()) {
        QGraphicsLineItem *item = scene_->addLine(segment.x1_mm, segment.y1_mm, segment.x2_mm, segment.y2_mm, wallPen);
        item->setZValue(4.0);
    }

    const QPen rayPen(QColor(230, 90, 20), 2.0);
    const QPen hitPen(QColor(160, 40, 0), 1.0);
    const QBrush hitBrush(QColor(255, 120, 40));

    for (const IrSensorReading &reading : irReadings_) {
        const QPointF origin = robotLocalToWorld(reading.local_x_mm, reading.local_y_mm);

        QGraphicsLineItem *rayItem = scene_->addLine(origin.x(), origin.y(), reading.hit_x_mm, reading.hit_y_mm, rayPen);
        rayItem->setZValue(8.0);

        QGraphicsEllipseItem *originItem = scene_->addEllipse(origin.x() - 2.0,
                                                              origin.y() - 2.0,
                                                              4.0,
                                                              4.0,
                                                              QPen(Qt::NoPen),
                                                              QBrush(QColor(230, 90, 20)));
        originItem->setZValue(9.0);

        if (reading.hit) {
            QGraphicsEllipseItem *hitItem = scene_->addEllipse(reading.hit_x_mm - 4.0,
                                                               reading.hit_y_mm - 4.0,
                                                               8.0,
                                                               8.0,
                                                               hitPen,
                                                               hitBrush);
            hitItem->setZValue(9.0);
        }
    }

    const QBrush floorFrontBrush(floorFront_.black ? QColor(0, 0, 0) : QColor(255, 255, 255));
    const QBrush floorRearBrush(floorRear_.black ? QColor(0, 0, 0) : QColor(255, 255, 255));

    QGraphicsEllipseItem *frontFloorItem = scene_->addEllipse(floorFront_.world_x_mm - kFloorSensorRadiusMm,
                                                              floorFront_.world_y_mm - kFloorSensorRadiusMm,
                                                              2.0 * kFloorSensorRadiusMm,
                                                              2.0 * kFloorSensorRadiusMm,
                                                              QPen(QColor(30, 30, 30), 2.0),
                                                              floorFrontBrush);
    frontFloorItem->setZValue(12.0);

    QGraphicsEllipseItem *rearFloorItem = scene_->addEllipse(floorRear_.world_x_mm - kFloorSensorRadiusMm,
                                                             floorRear_.world_y_mm - kFloorSensorRadiusMm,
                                                             2.0 * kFloorSensorRadiusMm,
                                                             2.0 * kFloorSensorRadiusMm,
                                                             QPen(QColor(30, 30, 30), 2.0),
                                                             floorRearBrush);
    rearFloorItem->setZValue(12.0);

    QGraphicsPolygonItem *robotItem = scene_->addPolygon(robotPolygon(),
                                                         QPen(QColor(20, 70, 120), 2.0),
                                                         QBrush(QColor(40, 140, 220)));
    robotItem->setZValue(10.0);

    fitViewToWorld();
}

void MainWindow::refreshTelemetry()
{
    const FirmwareSimBridge::Debug debug = firmwareBridge_.debug();
    const bool manualJogPerturbationMode = simulationRunning_ && firmwareBridge_.isFirmwareControlActive();

    QString text;
    text += QStringLiteral("Simulation\n");
    text += QStringLiteral("  running: %1\n").arg(boolText(simulationRunning_));
    text += QStringLiteral("  interval_ms: %1\n\n").arg(kSimulationIntervalMs);

    text += QStringLiteral("World\n");
    text += QStringLiteral("  map: %1\n").arg(world_.mazeName());
    text += QStringLiteral("  size: %1 cols x %2 rows\n").arg(world_.cols()).arg(world_.rows());
    text += QStringLiteral("  cell_size_mm: %1\n\n").arg(world_.cellSizeMm(), 0, 'f', 1);

    text += QStringLiteral("Rendering\n");
    text += QStringLiteral("  boundary_tape: translucent gray\n");
    text += QStringLiteral("  target/special: dark gray square\n");
    text += QStringLiteral("  walls: solid black thick lines\n");
    text += QStringLiteral("  ir_rays: orange\n\n");

    text += QStringLiteral("Robot pose\n");
    text += QStringLiteral("  x_mm: %1\n").arg(robot_.xMm(), 0, 'f', 2);
    text += QStringLiteral("  y_mm: %1\n").arg(robot_.yMm(), 0, 'f', 2);
    text += QStringLiteral("  yaw_deg: %1\n").arg(robot_.yawDeg(), 0, 'f', 2);
    text += QStringLiteral("  yaw_rate_deg_s: %1\n\n").arg(robot_.yawRateDegS(), 0, 'f', 2);

    text += QStringLiteral("Manual jog\n");
    text += QStringLiteral("  keys: W/S/A/D or arrows\n");
    text += QStringLiteral("  fast: Shift + movement key\n");
    text += QStringLiteral("  fine: Ctrl + arrows\n");
    text += manualJogPerturbationMode
        ? QStringLiteral("  note: manual jog perturbs pose without stopping firmware control\n")
        : QStringLiteral("  note: manual jog stops the timer before moving\n");
    text += QStringLiteral("  last: %1\n\n").arg(lastManualJogDescription_);

    text += QStringLiteral("FirmwareSimBridge\n");
    text += QStringLiteral("  state: %1\n").arg(debug.state);
    text += QStringLiteral("  reason: %1\n").arg(debug.reason);
    text += QStringLiteral("  enabled: %1\n").arg(boolText(debug.enabled));
    text += QStringLiteral("  firmware_control_mode: %1\n").arg(debug.control_mode);
    text += QStringLiteral("  sim_config_left_base: %1\n").arg(debug.sim_config_left_base);
    text += QStringLiteral("  sim_config_right_base: %1\n").arg(debug.sim_config_right_base);
    text += QStringLiteral("  left_pwm: %1\n").arg(lastCommand_.left_pwm);
    text += QStringLiteral("  right_pwm: %1\n\n").arg(lastCommand_.right_pwm);

    text += QStringLiteral("FW decision\n");
    text += QStringLiteral("  random_value: %1\n")
        .arg(static_cast<unsigned int>(debug.decision_random_value));
    text += QStringLiteral("  available_options_mask: 0x%1\n")
        .arg(static_cast<unsigned int>(debug.available_options_mask), 2, 16, QLatin1Char('0'));
    text += QStringLiteral("  valid_option_count: %1\n")
        .arg(static_cast<unsigned int>(debug.valid_option_count));
    text += QStringLiteral("  recommended_action: %1 (%2)\n\n")
        .arg(debug.recommended_action)
        .arg(debug.recommended_action_text);

    text += QStringLiteral("FW perception\n");
    text += QStringLiteral("  floor_front_black: %1\n").arg(boolText(debug.floor_front_black));
    text += QStringLiteral("  floor_rear_black: %1\n").arg(boolText(debug.floor_rear_black));
    text += QStringLiteral("  wall_front: %1\n").arg(boolText(debug.wall_front));
    text += QStringLiteral("  wall_left: %1\n").arg(boolText(debug.wall_left));
    text += QStringLiteral("  wall_right: %1\n").arg(boolText(debug.wall_right));
    text += QStringLiteral("  wall_diag_left: %1\n").arg(boolText(debug.wall_diag_left));
    text += QStringLiteral("  wall_diag_right: %1\n").arg(boolText(debug.wall_diag_right));
    text += QStringLiteral("  dist_front_left_mm: %1\n").arg(debug.dist_front_left_mm);
    text += QStringLiteral("  dist_front_right_mm: %1\n").arg(debug.dist_front_right_mm);
    text += QStringLiteral("  dist_left_lat_mm: %1\n").arg(debug.dist_left_lat_mm);
    text += QStringLiteral("  dist_right_lat_mm: %1\n").arg(debug.dist_right_lat_mm);
    text += QStringLiteral("  dist_diagonal_left_mm: %1\n").arg(debug.dist_diagonal_left_mm);
    text += QStringLiteral("  dist_diagonal_right_mm: %1\n").arg(debug.dist_diagonal_right_mm);
    text += QStringLiteral("  adc_floor_front: %1\n").arg(debug.adc_floor_front);
    text += QStringLiteral("  adc_floor_rear: %1\n\n").arg(debug.adc_floor_rear);

    text += QStringLiteral("IR sensors\n");
    for (const IrSensorReading &reading : irReadings_) {
        text += QStringLiteral("  %1: hit=%2 dist_mm=%3\n")
            .arg(reading.name)
            .arg(boolText(reading.hit))
            .arg(reading.distance_mm, 0, 'f', 1);
    }

    text += QStringLiteral("\nFloor sensors\n");
    text += QStringLiteral("  front: black=%1 kind=%2 x=%3 y=%4\n")
        .arg(boolText(floorFront_.black))
        .arg(tapeKindText(floorFront_.debug_kind))
        .arg(floorFront_.world_x_mm, 0, 'f', 1)
        .arg(floorFront_.world_y_mm, 0, 'f', 1);
    text += QStringLiteral("  rear: black=%1 kind=%2 x=%3 y=%4\n")
        .arg(boolText(floorRear_.black))
        .arg(tapeKindText(floorRear_.debug_kind))
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
    const double halfLength = kRobotLengthMm / 2.0;
    const double halfWidth = kRobotWidthMm / 2.0;

    QPolygonF polygon;
    polygon << robotLocalToWorld(halfLength, 0.0);
    polygon << robotLocalToWorld(halfLength * 0.35, halfWidth);
    polygon << robotLocalToWorld(-halfLength, halfWidth);
    polygon << robotLocalToWorld(-halfLength, -halfWidth);
    polygon << robotLocalToWorld(halfLength * 0.35, -halfWidth);
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
