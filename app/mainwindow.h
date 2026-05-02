#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>

#include "sim_robot.h"

class QGraphicsPolygonItem;
class QKeyEvent;
class QResizeEvent;

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
    void createRobotItem();
    void updateRobotGraphics();
    void fitSceneToView();

    QGraphicsScene *scene = nullptr;
    QGraphicsView *view = nullptr;
    QGraphicsPolygonItem *robotItem = nullptr;
    SimRobot robot;
};

#endif // MAINWINDOW_H
