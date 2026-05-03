#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>

#include <cstdint>
#include <vector>

#include "nav_types.h"
#include "sim_robot.h"
#include "sim_world.h"

class QLabel;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsPolygonItem;
class QKeyEvent;
class QResizeEvent;
class QTimer;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    void setupScene();
    void drawReferenceGrid();
    void drawBlackTape();
    void drawWorldWalls();
    void initializeIrSensors();
    void initializeFloorSensors();
    void createTelemetryPanel();
    void updateTelemetryPanel();
    void showControlsHelp();
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
    QLabel *navFloorFrontValueLabel = nullptr;
    QLabel *navFloorRearValueLabel = nullptr;
    QLabel *navYawValueLabel = nullptr;
    QLabel *navYawRateValueLabel = nullptr;
    QLabel *navYawZeroValueLabel = nullptr;
    QLabel *navStateValueLabel = nullptr;
    QLabel *navActionValueLabel = nullptr;
    QLabel *navActionStartYawValueLabel = nullptr;
    QLabel *navActionTargetYawValueLabel = nullptr;
    QLabel *navLeftMotorValueLabel = nullptr;
    QLabel *navRightMotorValueLabel = nullptr;
    QLabel *simulationRunningValueLabel = nullptr;
    QLabel *simulationDtValueLabel = nullptr;
    QLabel *autoModeValueLabel = nullptr;
    QLabel *simulationStepCountValueLabel = nullptr;
    QLabel *simulationTimeValueLabel = nullptr;
    QLabel *motorTestModeValueLabel = nullptr;
    QLabel *motorTestLeftValueLabel = nullptr;
    QLabel *motorTestRightValueLabel = nullptr;
    QTimer *simulationTimer = nullptr;
    bool simulationRunning = false;
    bool autoModeEnabled = false;
    bool motorTestModeEnabled = false;
    uint64_t simulationStepCount = 0;
    double simulationTimeS = 0.0;
    double navYawZeroDeg = 0.0;
    RobotCommand lastNavCommand = {0, 0};
    RobotCommand motorTestCommand = {0, 0};
    SimWorld world;
    SimRobot robot;
    std::vector<IrSensor> irSensors;
    std::vector<FloorSensor> floorSensors;
};

#endif // MAINWINDOW_H
