#include "poissonsampler.h"

/*
    Poisson disc sampling method based on "Fast Poisson Disk Sampling in
    Arbitrary Dimensions" by Robert Bridson.

    https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
*/
std::vector<Vector2> PoissonDiscSampler::generateSamples(const Box &bounds,
       real r, int k) {
       real dx = r / (real)sqrt(2);
        SampleGrid grid(bounds, dx);

       Vector2 seed = randomPoint(bounds);
        std::vector<Vector2> points;
        points.push_back(seed);

        std::vector<int> activeList;
        activeList.push_back(0);

       Vector2i g = grid.getCell(seed);
        grid.setSample(g, 0);

        while (!activeList.empty()) {
            int randidx = (int)Math::IntervalRandom(0, (int)activeList.size() - 1);
            int pidx = activeList[randidx];
           Vector2 p = points[pidx];

           Vector2 newPoint;

           real nr =Math::IntervalRandom(r * 0.2f, r);

            bool isFound = findDiscPoint(p, nr, k, points, grid, &newPoint);

            if (!isFound) {
                activeList.erase(activeList.begin() + randidx);
                continue;
            }

            int newidx = (int)points.size();
            activeList.push_back(newidx);
            points.push_back(newPoint);

           Vector2i g = grid.getCell(newPoint);
            grid.setSample(g, newidx);
        }

        return points;
    }

   Vector2 PoissonDiscSampler::randomPoint(const Box &extents) {
       real px =Math::IntervalRandom(extents.vMin.x, extents.vMax.x);
       real py =Math::IntervalRandom(extents.vMin.y, extents.vMax.y);
        return Vector2(px, py);
    }

   Vector2 PoissonDiscSampler::randomDiscPoint(const Vector2 &center,real r) {
       real angle =Math::IntervalRandom(0.0f, 2 *Math::PI);
       real rl =Math::IntervalRandom(r, 2 * r);
       Vector2 vDir =Vector2(sin(angle), cos(angle));
        return center + vDir * rl;
    }

    bool PoissonDiscSampler::findDiscPoint(const Vector2 &center,real r, int k,
        const std::vector<Vector2> &points,
        SampleGrid &grid,Vector2 *p) {
        for (int i = 0; i < k; i++) {
           Vector2 sample = randomDiscPoint(center, r);
            if (!grid.boundingBox.isPointInside(Vector3(sample.x, sample.y, 0))) {
                continue;
            }

            if (isSampleValid(sample, r, points, grid)) {
                *p = sample;
                return true;
            }
        }

        return false;
    }

    bool PoissonDiscSampler::isSampleValid(const Vector2 &p,real r,
        const std::vector<Vector2> &points,
        SampleGrid &grid) {
       Vector2i g = grid.getCell(p);
        int sampleid = grid.getSample(g);
        if (sampleid != -1) {
            return false;
        }

       Vector2i vMin =Vector2i((int)fmax(g.x - 2, 0), (int)fmax(g.y - 2, 0));
       Vector2i vMax =Vector2i((int)fmin(g.x + 2, grid.vSize.x - 1), (int)fmin(g.y + 2, grid.vSize.y - 1));

       real rsq = r * r;
        for (int j = vMin.y; j <= vMax.y; j++) {
            for (int i = vMin.x; i <= vMax.x; i++) {
                sampleid = grid.getSample(i, j);
                if (sampleid == -1) {
                    continue;
                }

               Vector2 o = points[sampleid];
               Vector2 dv = p - o;
               real distsq = dv.lenSqr();

                if (distsq < rsq) {
                    return false;
                }
            }
        }

        return true;
    }
