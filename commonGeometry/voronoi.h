#ifndef VORONOI_H
#define VORONOI_H

#include <vector>
#include <set>
#include <cmath>

#include "noise.h"
#include "geometrymath.h"
#include "kdtree.h"
#include "delaunay.h"

#define Voronoi_Cell_Level 7

namespace Graph_Geometry {
    class Voronoi
    {
    public:
        struct StVoronoiGridData {
            real fDist;
            int nLabel;
            StVoronoiGridData(real _fDist, int _nLabel)
                :fDist(_fDist), nLabel(_nLabel) {}
        };

        explicit Voronoi(const Vector3i &vSize = Vector3i(1, 1, 1));
        virtual ~Voronoi();

        void genVoronoiMulLevel(int nCellNumber, int nLevel, real& fMinDist, real& fMaxDist);
        void genVoronoiKnn(int nCellNumber, int nLevel = 0);
        GraphGeometry voronoi(std::vector<Vector2> &points);
        GraphGeometry delaunayToVoronoi(GraphGeometry &T);

        bool bValidPos(int x, int y) {
            if (x < 0 || x >= m_vSize.x || y < 0 || y >= m_vSize.z) {
                return false;
            }
            return true;
        }

        int vector2Index(int x, int y) {
            return x + y * m_vSize.x;
        }

        real getValue(int x, int y) {
            if (x < 0 || x >= m_vSize.x || y < 0 || y >= m_vSize.y) return 0.f;
            return m_vData[x][y].fDist;
        }

    private:
        void genRandomCenters(int nCellNumber, int nLevel);

        typedef std::vector<std::vector<std::pair<int, int> > > VertexEdgeTable;

        void createVoronoiVertices(GraphGeometry &T, GraphGeometry &V, std::vector<int> &vertexToFaceTable);
        Vector2 computeVoronoiVertex(GraphGeometry &T, const Face &f);
        bool isBoundaryVertex(GraphGeometry &T, const Vertex2D &v);
        VertexEdgeTable initVertexEdgeTable(
            GraphGeometry &T, GraphGeometry &V, std::vector<int> &delaunayFaceToVertexTable);
        void initVertexIncidentEdges(GraphGeometry &V, const VertexEdgeTable &vertexEdges);
        void getVoronoiCellEdgeLoop(const Vertex2D &delaunayVertex,
            GraphGeometry &T, GraphGeometry &V,
            const std::vector<int> &delaunayFaceToVertexTable,
            const VertexEdgeTable &vertexEdgeTable,
            std::vector<HalfEdge> &edgeLoop);
        void initVoronoiFaceFromEdgeLoop(const std::vector<HalfEdge> &edgeLoop,
            GraphGeometry &V,
            VertexEdgeTable &vertexEdges);

        //data
        std::vector<std::vector<StVoronoiGridData>> m_vData;
        //size
        Vector3i m_vSize;
        int m_nMeshSize;
        int m_nVolume;
        //voronoi params
        fast_noise_lite::FastNoiseLite noiseLite;
        std::vector<std::vector<real>> m_gridNoise;

        std::vector<Vector3> m_vCenters[Voronoi_Cell_Level];
        std::vector<int> m_vCenterNumber;
        int m_nCellNumber;
        int m_nCellLevel;
    };
}

#endif
