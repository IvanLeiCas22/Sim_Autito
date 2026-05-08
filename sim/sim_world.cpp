#include "sim_world.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QIODevice>
#include <QDebug>
#include <QStringList>

#include <cmath>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kEpsilon = 1.0e-9;
constexpr double kTapeWidthMm = 20.0;
constexpr double kTapeHalfWidthMm = kTapeWidthMm / 2.0;
}

SimWorld::SimWorld()
{
    if (!loadDefaultJsonMaze()) {
        initializeDefaultMaze();
    }
}

int SimWorld::rows() const
{
    return rows_;
}

int SimWorld::cols() const
{
    return cols_;
}

double SimWorld::cellSizeMm() const
{
    return cellSizeMm_;
}

double SimWorld::widthMm() const
{
    return cols_ * cellSizeMm_;
}

double SimWorld::heightMm() const
{
    return rows_ * cellSizeMm_;
}

QString SimWorld::mazeName() const
{
    return mazeName_;
}

QString SimWorld::mazeFilePath() const
{
    return mazeFilePath_;
}

double SimWorld::startXMm() const
{
    return startXMm_;
}

double SimWorld::startYMm() const
{
    return startYMm_;
}

double SimWorld::startYawDeg() const
{
    return startYawDeg_;
}

bool SimWorld::hasWall(int row, int col, WallDir dir) const
{
    if (!isInside(row, col)) {
        return false;
    }

    return cells_[row * cols_ + col].walls[wallIndex(dir)];
}

void SimWorld::setWall(int row, int col, WallDir dir, bool enabled)
{
    if (!isInside(row, col)) {
        return;
    }

    cells_[row * cols_ + col].walls[wallIndex(dir)] = enabled;

    const int nextRow = neighborRow(row, dir);
    const int nextCol = neighborCol(col, dir);
    if (isInside(nextRow, nextCol)) {
        cells_[nextRow * cols_ + nextCol].walls[wallIndex(oppositeDir(dir))] = enabled;
    }
}

void SimWorld::addWall(int row, int col, WallDir dir)
{
    setWall(row, col, dir, true);
}

std::vector<SimLineSegment> SimWorld::wallSegments() const
{
    std::vector<SimLineSegment> segments;

    for (int row = 0; row < rows_; ++row) {
        for (int col = 0; col < cols_; ++col) {
            const double x0 = col * cellSizeMm_;
            const double y0 = row * cellSizeMm_;
            const double x1 = x0 + cellSizeMm_;
            const double y1 = y0 + cellSizeMm_;

            if (hasWall(row, col, WallDir::North)) {
                segments.push_back({x0, y0, x1, y0});
            }
            if (hasWall(row, col, WallDir::West)) {
                segments.push_back({x0, y0, x0, y1});
            }
            if (col == cols_ - 1 && hasWall(row, col, WallDir::East)) {
                segments.push_back({x1, y0, x1, y1});
            }
            if (row == rows_ - 1 && hasWall(row, col, WallDir::South)) {
                segments.push_back({x0, y1, x1, y1});
            }
        }
    }

    return segments;
}

RaycastHit SimWorld::castRay(double origin_x_mm,
                             double origin_y_mm,
                             double angle_deg,
                             double max_distance_mm) const
{
    const double angleRad = angle_deg * kDegToRad;
    const double dx = std::cos(angleRad);
    const double dy = std::sin(angleRad);

    RaycastHit closestHit = {
        false,
        max_distance_mm,
        origin_x_mm + dx * max_distance_mm,
        origin_y_mm + dy * max_distance_mm
    };

    const std::vector<SimLineSegment> segments = wallSegments();
    for (const SimLineSegment &segment : segments) {
        const double sx = segment.x2_mm - segment.x1_mm;
        const double sy = segment.y2_mm - segment.y1_mm;
        const double qpx = segment.x1_mm - origin_x_mm;
        const double qpy = segment.y1_mm - origin_y_mm;
        const double denominator = dx * sy - dy * sx;

        if (std::abs(denominator) < kEpsilon) {
            continue;
        }

        const double rayDistance = (qpx * sy - qpy * sx) / denominator;
        const double segmentRatio = (qpx * dy - qpy * dx) / denominator;

        if (rayDistance < 0.0 || rayDistance > closestHit.distance_mm) {
            continue;
        }
        if (segmentRatio < 0.0 || segmentRatio > 1.0) {
            continue;
        }

        closestHit.hit = true;
        closestHit.distance_mm = rayDistance;
        closestHit.hit_x_mm = origin_x_mm + dx * rayDistance;
        closestHit.hit_y_mm = origin_y_mm + dy * rayDistance;
    }

    return closestHit;
}

bool SimWorld::isBlackTapeAt(double x_mm, double y_mm) const
{
    return debugTapeKindAt(x_mm, y_mm) != TapeDebugKind::None;
}

TapeDebugKind SimWorld::debugTapeKindAt(double x_mm, double y_mm) const
{
    const bool isBoundary = isBoundaryTapeAt(x_mm, y_mm);
    const bool isTarget = isTargetTapeAt(x_mm, y_mm);

    if (isBoundary && isTarget) {
        return TapeDebugKind::BoundaryAndTarget;
    }
    if (isBoundary) {
        return TapeDebugKind::Boundary;
    }
    if (isTarget) {
        return TapeDebugKind::Target;
    }

    return TapeDebugKind::None;
}

bool SimWorld::isBoundaryTapeAt(double x_mm, double y_mm) const
{
    const double mazeWidthMm = widthMm();
    const double mazeHeightMm = heightMm();
    if (x_mm < 0.0 || x_mm > mazeWidthMm || y_mm < 0.0 || y_mm > mazeHeightMm) {
        return false;
    }

    for (int col = 1; col < cols_; ++col) {
        const double lineX = col * cellSizeMm_;
        if (std::abs(x_mm - lineX) <= kTapeHalfWidthMm) {
            return true;
        }
    }

    for (int row = 1; row < rows_; ++row) {
        const double lineY = row * cellSizeMm_;
        if (std::abs(y_mm - lineY) <= kTapeHalfWidthMm) {
            return true;
        }
    }

    return false;
}

bool SimWorld::isTargetTapeAt(double x_mm, double y_mm) const
{
    (void)x_mm;
    (void)y_mm;
    return false;
}

std::vector<SimRect> SimWorld::boundaryTapeRects() const
{
    std::vector<SimRect> rects;
    const double mazeWidthMm = widthMm();
    const double mazeHeightMm = heightMm();

    for (int col = 1; col < cols_; ++col) {
        const double x = col * cellSizeMm_ - kTapeHalfWidthMm;
        rects.push_back({x, 0.0, kTapeWidthMm, mazeHeightMm});
    }

    for (int row = 1; row < rows_; ++row) {
        const double y = row * cellSizeMm_ - kTapeHalfWidthMm;
        rects.push_back({0.0, y, mazeWidthMm, kTapeWidthMm});
    }

    return rects;
}

std::vector<SimRect> SimWorld::targetTapeRects() const
{
    return {};
}

std::vector<SimRect> SimWorld::blackTapeRects() const
{
    return boundaryTapeRects();
}

void SimWorld::initializeDefaultMaze()
{
    resizeMaze(8, 8, kCellSizeMm);
    mazeName_ = "default";
    mazeFilePath_.clear();
    startXMm_ = 100.0;
    startYMm_ = 100.0;
    startYawDeg_ = 0.0;
    addBoundaryWalls();

    // Temporary internal walls to validate rendering before JSON loading exists.
    addWall(0, 1, WallDir::East);
    addWall(1, 1, WallDir::South);
    addWall(2, 3, WallDir::East);
    addWall(3, 4, WallDir::South);
}

bool SimWorld::loadDefaultJsonMaze()
{
    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
#ifdef SIM_AUTITO_SOURCE_DIR
        QDir(QStringLiteral(SIM_AUTITO_SOURCE_DIR)).filePath("data/maze_01.json"),
#endif
        QDir::current().filePath("data/maze_01.json"),
        QDir(appDir).filePath("data/maze_01.json"),
        QDir(appDir).filePath("../data/maze_01.json"),
        QDir(appDir).filePath("../../data/maze_01.json")
    };

    for (const QString &path : candidates) {
        if (QFile::exists(path) && loadFromJsonFile(path)) {
            qInfo() << "Loaded maze JSON:" << QDir::toNativeSeparators(path)
                    << "name:" << mazeName_
                    << "cols:" << cols_
                    << "rows:" << rows_
                    << "cell_size_mm:" << cellSizeMm_;
            return true;
        }
    }

    qWarning() << "Could not load data/maze_01.json; using built-in default maze";
    return false;
}

bool SimWorld::loadFromJsonFile(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open maze JSON:" << QDir::toNativeSeparators(path);
        return false;
    }

    QJsonParseError parseError = {};
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        qWarning() << "Invalid maze JSON:" << parseError.errorString();
        return false;
    }

    const QJsonObject root = document.object();
    mazeName_ = root.value("name").toString("maze_01");
    mazeFilePath_ = path;
    const QJsonObject cells = root.value("cells").toObject();
    const int cols = root.contains("cols")
        ? root.value("cols").toInt()
        : cells.value("width").toInt(8);
    const int rows = root.contains("rows")
        ? root.value("rows").toInt()
        : cells.value("height").toInt(8);
    const double cellSizeMm = root.contains("cell_size_mm")
        ? root.value("cell_size_mm").toDouble(kCellSizeMm)
        : cells.value("cell_size_mm").toDouble(kCellSizeMm);

    if (rows <= 0 || cols <= 0 || cellSizeMm <= 0.0) {
        qWarning() << "Invalid maze dimensions in JSON";
        return false;
    }

    resizeMaze(rows, cols, cellSizeMm);

    const QJsonObject start = root.value("start").toObject();
    startXMm_ = start.value("x_mm").toDouble(cellSizeMm_ / 2.0);
    startYMm_ = start.value("y_mm").toDouble(cellSizeMm_ / 2.0);
    startYawDeg_ = start.value("yaw_deg").toDouble(0.0);

    addBoundaryWalls();

    const QJsonArray walls = root.value("walls").toArray();
    for (const QJsonValue &wallValue : walls) {
        const QJsonObject wall = wallValue.toObject();
        const int col = wall.contains("cell_x")
            ? wall.value("cell_x").toInt(-1)
            : wall.value("col").toInt(-1);
        const int row = wall.contains("cell_y")
            ? wall.value("cell_y").toInt(-1)
            : wall.value("row").toInt(-1);
        WallDir dir = WallDir::North;
        if (!parseWallDir(wall.value("dir").toString(), &dir)) {
            qWarning() << "Skipping maze wall with invalid dir";
            continue;
        }
        if (!isInside(row, col)) {
            qWarning() << "Skipping maze wall outside bounds at row/col:" << row << col;
            continue;
        }

        addWall(row, col, dir);
    }

    addBoundaryWalls();
    return true;
}

void SimWorld::resizeMaze(int rows, int cols, double cell_size_mm)
{
    rows_ = rows;
    cols_ = cols;
    cellSizeMm_ = cell_size_mm;
    cells_.assign(rows_ * cols_, Cell{});
}

void SimWorld::addBoundaryWalls()
{
    for (int col = 0; col < cols_; ++col) {
        addWall(0, col, WallDir::North);
        addWall(rows_ - 1, col, WallDir::South);
    }

    for (int row = 0; row < rows_; ++row) {
        addWall(row, 0, WallDir::West);
        addWall(row, cols_ - 1, WallDir::East);
    }
}

bool SimWorld::parseWallDir(const QString &text, WallDir *dir) const
{
    if (dir == nullptr) {
        return false;
    }

    const QString normalized = text.trimmed().toUpper();
    if (normalized == "N" || normalized == "NORTH") {
        *dir = WallDir::North;
        return true;
    }
    if (normalized == "E" || normalized == "EAST") {
        *dir = WallDir::East;
        return true;
    }
    if (normalized == "S" || normalized == "SOUTH") {
        *dir = WallDir::South;
        return true;
    }
    if (normalized == "W" || normalized == "WEST") {
        *dir = WallDir::West;
        return true;
    }

    return false;
}

bool SimWorld::isInside(int row, int col) const
{
    return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

int SimWorld::wallIndex(WallDir dir) const
{
    return static_cast<int>(dir);
}

WallDir SimWorld::oppositeDir(WallDir dir) const
{
    switch (dir) {
    case WallDir::North:
        return WallDir::South;
    case WallDir::East:
        return WallDir::West;
    case WallDir::South:
        return WallDir::North;
    case WallDir::West:
        return WallDir::East;
    }

    return WallDir::North;
}

int SimWorld::neighborRow(int row, WallDir dir) const
{
    switch (dir) {
    case WallDir::North:
        return row - 1;
    case WallDir::South:
        return row + 1;
    case WallDir::East:
    case WallDir::West:
        return row;
    }

    return row;
}

int SimWorld::neighborCol(int col, WallDir dir) const
{
    switch (dir) {
    case WallDir::East:
        return col + 1;
    case WallDir::West:
        return col - 1;
    case WallDir::North:
    case WallDir::South:
        return col;
    }

    return col;
}
