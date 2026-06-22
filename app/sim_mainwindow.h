#ifndef SIM_MAINWINDOW_H
#define SIM_MAINWINDOW_H

#include "firmware_sim_bridge.h"
#include "sim_unerbus_link.h"
#include "sim_robot.h"
#include "sim_world.h"

#include <array>

#include <QByteArray>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QList>
#include <QMainWindow>
#include <QShowEvent>
#include <QResizeEvent>
#include <QPlainTextEdit>
#include <QTimer>

class QAction;

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void showEvent(QShowEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    static constexpr int kIrSensorCount = FirmwareSimBridge::kIrSensorCount;

    struct IrSensorReading
    {
        QString name;
        double local_x_mm = 0.0;
        double local_y_mm = 0.0;
        double local_angle_deg = 0.0;
        bool hit = false;
        double distance_mm = 0.0;
        double hit_x_mm = 0.0;
        double hit_y_mm = 0.0;
    };

    struct FloorSensorReading
    {
        QString name;
        double local_x_mm = 0.0;
        double local_y_mm = 0.0;
        bool black = false;
        TapeDebugKind debug_kind = TapeDebugKind::None;
        double world_x_mm = 0.0;
        double world_y_mm = 0.0;
    };

    void setupUi();
    void setupActions();

    void loadMap();
    void resetSimulation();
    void startSimulation();
    void stopSimulation();
    void toggleSimulation();
    void startStraightYawHoldControl();
    void startWallFollowAdvanceControl();
    void startSmoothTurnLeftControl();
    void startSmoothTurnRightControl();
    void startPivotLeft90Control();
    void startPivotRight90Control();
    void startPivot180Control();
    void startSupervisorV1Control();
    void startGoToBControl();
    void stopFirmwareControl();
    void clearSupervisorLearnedMap();
    void tuneFirmwareConfig();
    void configureRealHmiLink();
    void toggleRealHmiLink(bool enabled);
    void realHmiTimerStep();
    void simulationStep();
    void manualJog(double distance_mm, double delta_yaw_deg);
    void rotateManualJog(double delta_yaw_deg);

    void updateSensors();
    FirmwareSimBridge::SensorSnapshot buildBridgeSnapshot() const;
    bool computeFirmwareInitialMazePose(uint8_t *x, uint8_t *y, uint8_t *heading) const;

    void refreshScene();
    void clearNonFirmwareMazeOverlayItems();
    void clearFirmwareMazeOverlay();
    void resetFirmwareMazeOverlayCache();
    void updateFirmwareMazeOverlay();
    QByteArray buildFirmwareMazeOverlaySignature(const FirmwareSimBridge::Debug &debug) const;
    bool firmwareMazeLogicalToWorldCell(uint8_t logical_x, uint8_t logical_y, int *world_col, int *world_row) const;
    void addFirmwareMazeOverlayItem(QGraphicsItem *item);
    void refreshTelemetry();

    QPointF robotLocalToWorld(double local_x_mm, double local_y_mm) const;
    QPolygonF robotPolygon() const;
    void fitViewToWorld();

    SimWorld world_;
    SimRobot robot_;
    FirmwareSimBridge firmwareBridge_;
    SimUnerbusLink realHmiLink_;

    QGraphicsScene *scene_ = nullptr;
    QGraphicsView *view_ = nullptr;
    QTimer *simulationTimer_ = nullptr;
    QTimer *realHmiTimer_ = nullptr;
    QPlainTextEdit *telemetryText_ = nullptr;
    QLabel *statusLabel_ = nullptr;

    bool simulationRunning_ = false;
    bool rotateJogAroundRearAxle_ = true;
    bool showFirmwareMazeOverlay_ = true;
    QAction *enableRealHmiLinkAction_ = nullptr;
    QByteArray firmwareMazeOverlaySignature_;
    QList<QGraphicsItem *> firmwareMazeOverlayItems_;
    FirmwareSimBridge::Command lastCommand_;
    QString lastManualJogDescription_ = QStringLiteral("none");

    std::array<IrSensorReading, kIrSensorCount> irReadings_;
    FloorSensorReading floorFront_;
    FloorSensorReading floorRear_;
};

#endif // SIM_MAINWINDOW_H
