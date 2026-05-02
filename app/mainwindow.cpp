#include "mainwindow.h"

#include <QBrush>
#include <QColor>
#include <QPainter>
#include <QPen>
#include <QTimer>

namespace {
constexpr int kGridCells = 5;
constexpr int kCellSizeMm = 200;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Sim Autito");
    resize(900, 700);

    setupScene();
    QTimer::singleShot(0, this, [this]() { fitSceneToView(); });
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
    setCentralWidget(view);

    drawReferenceGrid();
}

void MainWindow::drawReferenceGrid()
{
    const int mazeSizeMm = kGridCells * kCellSizeMm;
    scene->setSceneRect(0, 0, mazeSizeMm, mazeSizeMm);

    const QPen gridPen(QColor(160, 160, 160), 1);
    const QPen borderPen(QColor(45, 45, 45), 2);

    for (int i = 0; i <= kGridCells; ++i) {
        const int pos = i * kCellSizeMm;
        scene->addLine(pos, 0, pos, mazeSizeMm, gridPen);
        scene->addLine(0, pos, mazeSizeMm, pos, gridPen);
    }

    scene->addRect(0, 0, mazeSizeMm, mazeSizeMm, borderPen);
}

void MainWindow::fitSceneToView()
{
    if (!view || !scene || scene->items().isEmpty()) {
        return;
    }

    view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}
