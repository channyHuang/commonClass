#ifndef POISSONSAMPLER_H
#define POISSONSAMPLER_H

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <time.h>

#include "commonMath/vector2i.h"
#include "commonMath/vector2.h"
#include "commonMath/box.h"
#include "geometrymath.h"

class PoissonDiscSampler {
public:
    struct SampleGrid {
        Box boundingBox;
        Vector2i vSize = Vector2i(0, 0);
        float fStepSize = 0;
        std::vector<int> grid;

        SampleGrid(const Box &extents, float cellsize) {
            boundingBox = extents;
            fStepSize = cellsize;
            Vector2 vFloatSize = Vector2(boundingBox.getDX(), boundingBox.getDY()) / fStepSize;
            vFloatSize = vFloatSize.ceil();
            vSize = Vector2i((int)vFloatSize.x, (int)vFloatSize.y);
            grid = std::vector<int>(vSize.x * vSize.y, -1);
        }

        int getFlatIndex(int i, int j) {
            return i + j * vSize.x;
        }

        int getSample(const Vector2i &g) {
            return getSample(g.x, g.y);
        }

        int getSample(int i, int j) {
            if (i < 0 || i > vSize.x || j < 0 || j > vSize.y) {
                return -1;
            }
            return grid[getFlatIndex(i, j)];
        }

        void setSample(const Vector2i &g, int s) {
            setSample(g.x, g.y, s);
        }

        void setSample(int i, int j, int s) {
            if (i < 0 || i > vSize.x || j < 0 || j > vSize.y) {
                return;
            }
            grid[getFlatIndex(i, j)] = s;
        }

        Vector2i getCell(const Vector2 &p) {
            return getCell(p.x, p.y);
        }

        Vector2i getCell(float x, float y) {
            x -= boundingBox.vMin.x;
            y -= boundingBox.vMin.y;
            return Vector2i((int)floor(x / fStepSize), (int)floor(y / fStepSize));
        }
    };

    static std::vector<Vector2> generateSamples(const Box &bounds, float r, int k);

    static Vector2 randomPoint(const Box &extents);
    static Vector2 randomDiscPoint(const Vector2 &center, float r);
    static bool findDiscPoint(const Vector2 &center, float r, int k,
        const std::vector<Vector2> &points, SampleGrid &grid, Vector2 *p);
    static bool isSampleValid(const Vector2 &p, float r,
        const std::vector<Vector2> &points, SampleGrid &grid);
};

#endif
