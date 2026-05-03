#include "sim_world.h"

#include <cmath>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kEpsilon = 1.0e-9;
constexpr double kTapeWidthMm = 20.0;
constexpr double kTapeHalfWidthMm = kTapeWidthMm / 2.0;
constexpr double kTargetTapeSizeMm = 90.0;

struct TargetCell {
    int row;
    int col;
};

constexpr TargetCell kTargetCells[] = {
    {2, 2},
    {4, 5},
    {6, 1}
};
}

SimWorld::SimWorld()
    : cells_(kRows * kCols)
{
    initializeDefaultMaze();
}

int SimWorld::rows() const
{
    return kRows;
}

int SimWorld::cols() const
{
    return kCols;
}

double SimWorld::cellSizeMm() const
{
    return kCellSizeMm;
}

bool SimWorld::hasWall(int row, int col, WallDir dir) const
{
    if (!isInside(row, col)) {
        return false;
    }

    return cells_[row * kCols + col].walls[wallIndex(dir)];
}

void SimWorld::setWall(int row, int col, WallDir dir, bool enabled)
{
    if (!isInside(row, col)) {
        return;
    }

    cells_[row * kCols + col].walls[wallIndex(dir)] = enabled;

    const int nextRow = neighborRow(row, dir);
    const int nextCol = neighborCol(col, dir);
    if (isInside(nextRow, nextCol)) {
        cells_[nextRow * kCols + nextCol].walls[wallIndex(oppositeDir(dir))] = enabled;
    }
}

void SimWorld::addWall(int row, int col, WallDir dir)
{
    setWall(row, col, dir, true);
}

std::vector<SimLineSegment> SimWorld::wallSegments() const
{
    std::vector<SimLineSegment> segments;

    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            const double x0 = col * kCellSizeMm;
            const double y0 = row * kCellSizeMm;
            const double x1 = x0 + kCellSizeMm;
            const double y1 = y0 + kCellSizeMm;

            if (hasWall(row, col, WallDir::North)) {
                segments.push_back({x0, y0, x1, y0});
            }
            if (hasWall(row, col, WallDir::West)) {
                segments.push_back({x0, y0, x0, y1});
            }
            if (col == kCols - 1 && hasWall(row, col, WallDir::East)) {
                segments.push_back({x1, y0, x1, y1});
            }
            if (row == kRows - 1 && hasWall(row, col, WallDir::South)) {
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
    const double mazeWidthMm = kCols * kCellSizeMm;
    const double mazeHeightMm = kRows * kCellSizeMm;
    if (x_mm < 0.0 || x_mm > mazeWidthMm || y_mm < 0.0 || y_mm > mazeHeightMm) {
        return false;
    }

    for (int col = 1; col < kCols; ++col) {
        const double lineX = col * kCellSizeMm;
        if (std::abs(x_mm - lineX) <= kTapeHalfWidthMm) {
            return true;
        }
    }

    for (int row = 1; row < kRows; ++row) {
        const double lineY = row * kCellSizeMm;
        if (std::abs(y_mm - lineY) <= kTapeHalfWidthMm) {
            return true;
        }
    }

    return false;
}

bool SimWorld::isTargetTapeAt(double x_mm, double y_mm) const
{
    const double mazeWidthMm = kCols * kCellSizeMm;
    const double mazeHeightMm = kRows * kCellSizeMm;
    if (x_mm < 0.0 || x_mm > mazeWidthMm || y_mm < 0.0 || y_mm > mazeHeightMm) {
        return false;
    }

    for (const TargetCell &target : kTargetCells) {
        const double centerX = (target.col + 0.5) * kCellSizeMm;
        const double centerY = (target.row + 0.5) * kCellSizeMm;
        const double halfSize = kTargetTapeSizeMm / 2.0;

        if (x_mm >= centerX - halfSize && x_mm <= centerX + halfSize
            && y_mm >= centerY - halfSize && y_mm <= centerY + halfSize) {
            return true;
        }
    }

    return false;
}

std::vector<SimRect> SimWorld::boundaryTapeRects() const
{
    std::vector<SimRect> rects;
    const double mazeWidthMm = kCols * kCellSizeMm;
    const double mazeHeightMm = kRows * kCellSizeMm;

    for (int col = 1; col < kCols; ++col) {
        const double x = col * kCellSizeMm - kTapeHalfWidthMm;
        rects.push_back({x, 0.0, kTapeWidthMm, mazeHeightMm});
    }

    for (int row = 1; row < kRows; ++row) {
        const double y = row * kCellSizeMm - kTapeHalfWidthMm;
        rects.push_back({0.0, y, mazeWidthMm, kTapeWidthMm});
    }

    return rects;
}

std::vector<SimRect> SimWorld::targetTapeRects() const
{
    std::vector<SimRect> rects;

    for (const TargetCell &target : kTargetCells) {
        const double centerX = (target.col + 0.5) * kCellSizeMm;
        const double centerY = (target.row + 0.5) * kCellSizeMm;
        rects.push_back({
            centerX - kTargetTapeSizeMm / 2.0,
            centerY - kTargetTapeSizeMm / 2.0,
            kTargetTapeSizeMm,
            kTargetTapeSizeMm
        });
    }

    return rects;
}

std::vector<SimRect> SimWorld::blackTapeRects() const
{
    std::vector<SimRect> rects = boundaryTapeRects();
    const std::vector<SimRect> targets = targetTapeRects();
    rects.insert(rects.end(), targets.begin(), targets.end());
    return rects;
}

void SimWorld::initializeDefaultMaze()
{
    for (int col = 0; col < kCols; ++col) {
        addWall(0, col, WallDir::North);
        addWall(kRows - 1, col, WallDir::South);
    }

    for (int row = 0; row < kRows; ++row) {
        addWall(row, 0, WallDir::West);
        addWall(row, kCols - 1, WallDir::East);
    }

    // Temporary internal walls to validate rendering before JSON loading exists.
    addWall(0, 1, WallDir::East);
    addWall(1, 1, WallDir::South);
    addWall(2, 3, WallDir::East);
    addWall(3, 4, WallDir::South);
}

bool SimWorld::isInside(int row, int col) const
{
    return row >= 0 && row < kRows && col >= 0 && col < kCols;
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
