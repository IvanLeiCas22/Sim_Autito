#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include <array>
#include <vector>

enum class WallDir
{
    North,
    East,
    South,
    West
};

enum class TapeDebugKind
{
    None,
    Boundary,
    Target,
    BoundaryAndTarget
};

struct SimLineSegment
{
    double x1_mm;
    double y1_mm;
    double x2_mm;
    double y2_mm;
};

struct RaycastHit
{
    bool hit;
    double distance_mm;
    double hit_x_mm;
    double hit_y_mm;
};

struct SimRect
{
    double x_mm;
    double y_mm;
    double w_mm;
    double h_mm;
};

class SimWorld
{
public:
    SimWorld();

    int rows() const;
    int cols() const;
    double cellSizeMm() const;
    bool hasWall(int row, int col, WallDir dir) const;
    void setWall(int row, int col, WallDir dir, bool enabled);
    void addWall(int row, int col, WallDir dir);
    std::vector<SimLineSegment> wallSegments() const;
    RaycastHit castRay(double origin_x_mm,
                       double origin_y_mm,
                       double angle_deg,
                       double max_distance_mm) const;
    bool isBlackTapeAt(double x_mm, double y_mm) const;
    TapeDebugKind debugTapeKindAt(double x_mm, double y_mm) const;
    std::vector<SimRect> boundaryTapeRects() const;
    std::vector<SimRect> targetTapeRects() const;
    std::vector<SimRect> blackTapeRects() const;

private:
    static constexpr int kRows = 8;
    static constexpr int kCols = 8;
    static constexpr double kCellSizeMm = 200.0;
    static constexpr int kWallDirCount = 4;

    struct Cell {
        std::array<bool, kWallDirCount> walls = {};
    };

    void initializeDefaultMaze();
    bool isBoundaryTapeAt(double x_mm, double y_mm) const;
    bool isTargetTapeAt(double x_mm, double y_mm) const;
    bool isInside(int row, int col) const;
    int wallIndex(WallDir dir) const;
    WallDir oppositeDir(WallDir dir) const;
    int neighborRow(int row, WallDir dir) const;
    int neighborCol(int col, WallDir dir) const;

    std::vector<Cell> cells_;

    // TODO: Add maze loading from JSON in a later step.
};

#endif // SIM_WORLD_H
