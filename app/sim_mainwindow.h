#ifndef SIM_MAINWINDOW_H
#define SIM_MAINWINDOW_H

#include "firmware_sim_bridge.h"
#include "sim_robot.h"
#include "sim_world.h"

#include <array>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QTimer>

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
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
    void simulationStep();

    void updateSensors();
    FirmwareSimBridge::SensorSnapshot buildBridgeSnapshot() const;

    void refreshScene();
    void refreshTelemetry();

    QPointF robotLocalToWorld(double local_x_mm, double local_y_mm) const;
    QPolygonF robotPolygon() const;
    void fitViewToWorld();

    SimWorld world_;
    SimRobot robot_;
    FirmwareSimBridge firmwareBridge_;

    QGraphicsScene *scene_ = nullptr;
    QGraphicsView *view_ = nullptr;
    QTimer *simulationTimer_ = nullptr;
    QPlainTextEdit *telemetryText_ = nullptr;
    QLabel *statusLabel_ = nullptr;

    bool simulationRunning_ = false;
    FirmwareSimBridge::Command lastCommand_;

    std::array<IrSensorReading, kIrSensorCount> irReadings_;
    FloorSensorReading floorFront_;
    FloorSensorReading floorRear_;
};

#endif // SIM_MAINWINDOW_H
