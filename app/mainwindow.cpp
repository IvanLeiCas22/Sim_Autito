#include "mainwindow.h"

#include <QBrush>
#include <QColor>
#include <QGraphicsPolygonItem>
#include <QKeyEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QTimer>

namespace {
constexpr int kGridCols = 8;
constexpr int kGridRows = 8;
constexpr int kCellSizeMm = 200;
constexpr double kManualMoveStepMm = 10.0;
constexpr double kManualRotateStepDeg = 10.0;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Sim Autito");
    resize(900, 700);
    setFocusPolicy(Qt::StrongFocus);

    setupScene();
    QTimer::singleShot(0, this, [this]() { fitSceneToView(); });
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    bool handled = true;

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
    default:
        handled = false;
        break;
    }

    if (handled) {
        updateRobotGraphics();
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
    createRobotItem();
    updateRobotGraphics();
}

void MainWindow::drawReferenceGrid()
{
    const int mazeWidthMm = kGridCols * kCellSizeMm;
    const int mazeHeightMm = kGridRows * kCellSizeMm;
    scene->setSceneRect(0, 0, mazeWidthMm, mazeHeightMm);

    const QPen gridPen(QColor(160, 160, 160), 1);
    const QPen borderPen(QColor(45, 45, 45), 2);

    for (int col = 0; col <= kGridCols; ++col) {
        const int x = col * kCellSizeMm;
        scene->addLine(x, 0, x, mazeHeightMm, gridPen);
    }

    for (int row = 0; row <= kGridRows; ++row) {
        const int y = row * kCellSizeMm;
        scene->addLine(0, y, mazeWidthMm, y, gridPen);
    }

    scene->addRect(0, 0, mazeWidthMm, mazeHeightMm, borderPen);
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

void MainWindow::updateRobotGraphics()
{
    if (!robotItem) {
        return;
    }

    robotItem->setPos(robot.xMm(), robot.yMm());
    robotItem->setRotation(robot.yawDeg());
}

void MainWindow::fitSceneToView()
{
    if (!view || !scene || scene->items().isEmpty()) {
        return;
    }

    view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}
