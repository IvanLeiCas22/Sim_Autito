#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>

class QResizeEvent;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    void setupScene();
    void drawReferenceGrid();
    void fitSceneToView();

    QGraphicsScene *scene = nullptr;
    QGraphicsView *view = nullptr;
};

#endif // MAINWINDOW_H
