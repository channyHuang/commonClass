#include "voronoi.h"

namespace Graph_Geometry {
    Voronoi::Voronoi(const Vector3i &vSize)
    {
        m_vSize = vSize;
        m_nMeshSize = m_vSize.x * m_vSize.z;
        m_nVolume = m_vSize.y * m_nMeshSize;

        noiseLite.SetSeed(rand());
        noiseLite.SetNoiseType(fast_noise_lite::FastNoiseLite::NoiseType_Cellular);
        noiseLite.SetCellularDistanceFunction(fast_noise_lite::FastNoiseLite::CellularDistanceFunction_Hybrid);

        m_vData.resize(m_vSize.x, std::vector<StVoronoiGridData>(m_vSize.z, StVoronoiGridData(0, 0)));
    }

    Voronoi::~Voronoi() {}

    void Voronoi::genRandomCenters(int nCellNumber, int nLevel) {
        m_nCellLevel = MathFuncs::Clamp(nLevel, 1, Voronoi_Cell_Level);

        m_gridNoise.resize(m_vSize.x, std::vector<float>(m_vSize.z, 0));
        {
#pragma omp parallel for num_threads(30)
            for (int i = 0; i < m_nMeshSize; i++) {
                int y = i / m_vSize.x;
                int x = i % m_vSize.x;
                m_gridNoise[x][y] = noiseLite.GetNoise<float>((float)x, (float)y);
            }
        }

        //generate random center to seperate area
        m_nCellNumber = nCellNumber;
        int nLevelNumber = m_nCellNumber;
        for (int l = 0; l < m_nCellLevel; l++) {
            m_vCenters[l].resize(nLevelNumber);
            for (int i = 0; i < nLevelNumber; i++) {
                Vector2 vCenter = GeometryMath::randomPoint(Boxi(Vector3i(0, 0, 0), m_vSize));

                m_vCenters[l][i].x = vCenter.x;
                m_vCenters[l][i].z = vCenter.y;
            }
            nLevelNumber <<= 1;
        }
    }

    void Voronoi::genVoronoiMulLevel(int nCellNumber, int nLevel, float& fMinDist, float& fMaxDist) {
        genRandomCenters(nCellNumber, nLevel);

        fMaxDist = 0;
        fMinDist = (float)m_vSize.lenSqr();

#pragma omp parallel for
        for (int i = 0; i < m_nMeshSize; i++) {
            int y = i / m_vSize.x;
            int x = i % m_vSize.x;

            Vector3 target = Vector3((float)x, (float)y, 0);
            for (int l = nLevel - 1; l >= 0; l--) {
                float dmin = (float)m_nVolume;
                int j = 0;
                for (unsigned int i = 0; i < m_vCenters[l].size(); i++) {
                    float dist = m_vCenters[l][i].distanceTo(target);
                    dist += m_gridNoise[x][y];

                    if (dist < dmin) {
                        dmin = dist;
                        j = i;
                    }
                }

                if (l == 0) {
                    float dist = m_vCenters[l][j].distanceTo(Vector3((float)x, (float)y, 0));
                    dist += m_gridNoise[(int)target.x][(int)target.y];

                    fMaxDist = std::max(fMaxDist, dist);
                    fMinDist = std::min(fMinDist, dist);
                    m_vData[x][y] = StVoronoiGridData(dist, j);
                }
                else {
                    target = m_vCenters[l][j];
                }
            }
        }
    }

    void Voronoi::genVoronoiKnn(int nCellNumber, int nLevel) {
        genRandomCenters(nCellNumber, nLevel + 1);
        unsigned int k = 1;
        std::vector<float> coeffs{ -1, 1 };
        KdTree tree(m_vCenters[nLevel], (int)m_vCenters[nLevel].size());
        std::vector<int> vLabel;
        std::vector<float> vDist;

        int idx = m_nMeshSize;
        float fMaxDist = 0;
        float fMinDist = MathFuncs::POS_INFINITY;
        while (idx) {
            int x = idx % m_vSize.x;
            int y = idx / m_vSize.z;
            tree.knnSearch(Vector3((float)x, (float)y, 0), 3, vLabel, vDist);

            float fPixelValue = 0;
            //vLabel[i]: label, vDist[i]: distance
            for (unsigned int i = 0; i < k; i++) {			// print summary
                //dists[i] = sqrt(dists[i]);			// unsquare distance

                if (i < coeffs.size()) {
                    fPixelValue += (coeffs[i] * vDist[i]);
                }
            }

            fPixelValue *= m_gridNoise[x][y];

            fMaxDist = std::max(fMaxDist, fPixelValue);
            fMinDist = std::min(fMinDist, fPixelValue);

            m_vData[x][y] = StVoronoiGridData(fPixelValue, vLabel[0]);
            idx--;
        }

        for (int x = 0; x < m_vSize.x; x++) {
            for (int y = 0; y < m_vSize.z; y++) {
                float fPixelValue = m_vData[x][y].fDist;
                fPixelValue -= fMinDist;
                fPixelValue /= (fMaxDist - fMinDist);
                fPixelValue = 256 * (1 - fPixelValue);
                m_vData[x][y].fDist = fPixelValue;
            }
        }
    }

    GraphGeometry Voronoi::voronoi(std::vector<Vector2> &points) {
        if (points.size() == 0) {
            return GraphGeometry();
        }

        Delaunay cDelaunay;
        GraphGeometry V = cDelaunay.triangulate(points);
        return delaunayToVoronoi(V);
    }

    GraphGeometry Voronoi::delaunayToVoronoi(GraphGeometry &T) {
        GraphGeometry V;
        std::vector<int> voronoiVertexToFaceTable;
        createVoronoiVertices(T, V, voronoiVertexToFaceTable);

        std::vector<int> delaunayFaceToVertexTable(T.faces.size(), -1);
        for (unsigned int i = 0; i < voronoiVertexToFaceTable.size(); i++) {
            delaunayFaceToVertexTable[voronoiVertexToFaceTable[i]] = i;
        }

        VertexEdgeTable vertexEdges;
        vertexEdges = initVertexEdgeTable(T, V, delaunayFaceToVertexTable);
        initVertexIncidentEdges(V, vertexEdges);

        std::vector<Face> incidentFaces;
        std::vector<HalfEdge> edgeLoop;
        for (unsigned int vidx = 0; vidx < T.vertices.size(); vidx++) {
            Vertex2D v = T.vertices[vidx];
            if (isBoundaryVertex(T, v)) {
                continue;
            }

            edgeLoop.clear();
            getVoronoiCellEdgeLoop(v, T, V,
                delaunayFaceToVertexTable,
                vertexEdges,
                edgeLoop);
            initVoronoiFaceFromEdgeLoop(edgeLoop, V, vertexEdges);
        }

        return V;
    }

    void Voronoi::createVoronoiVertices(GraphGeometry &T,
        GraphGeometry &V, std::vector<int> &vertexToFaceTable) {

        for (unsigned int i = 0; i < T.faces.size(); i++) {
            if (T.faces[i].outerComponent == -1) {
                continue;
            }

            Vector2 p = computeVoronoiVertex(T, T.faces[i]);
            V.createVertex(p);
            vertexToFaceTable.push_back(i);
        }
    }

    Vector2 Voronoi::computeVoronoiVertex(GraphGeometry &T, const Face &f) {
        // A voronoi Vertex2D is a center of a circle that passes through
        // points p0, pi, pj
        HalfEdge h = T.outerComponent(f);
        Vector2 p0 = T.origin(h).vPos;
        Vector2 pi = T.origin(T.next(h)).vPos;
        Vector2 pj = T.origin(T.prev(h)).vPos;

        Vector2 p(0.5f * (pi.x + pj.x), 0.5f * (pi.y + pj.y));
        Vector2 r(-(pj.y - pi.y), pj.x - pi.x);

        Vector2 q(0.5f * (pi.x + p0.x), 0.5f * (pi.y + p0.y));
        Vector2 s(-(p0.y - pi.y), p0.x - pi.x);

        Vector2 center;
        bool isIntersection = GeometryMath::lineIntersection(p, r, q, s, &center);
        if (!isIntersection) {
            return p0;
        }

        return center;
    }

    bool Voronoi::isBoundaryVertex(GraphGeometry &T, const Vertex2D &v) {
        if (v.incidentEdge == -1) {
            return true;
        }

        HalfEdge h = T.incidentEdge(v);
        int startid = h.id;

        do {
            if (h.incidentFace == -1) {
                return true;
            }

            h = T.twin(h);
            if (h.next == -1) {
                return true;
            }

            h = T.next(h);
        } while (h.id != startid);

        return false;
    }

    Voronoi::VertexEdgeTable Voronoi::initVertexEdgeTable(
        GraphGeometry &T, GraphGeometry &V,
        std::vector<int> &delaunayFaceToVertexTable) {
        VertexEdgeTable vertexEdges;
        vertexEdges.reserve(V.vertices.size());
        for (unsigned int i = 0; i < V.vertices.size(); i++) {
            vertexEdges.push_back(std::vector<std::pair<int, int> >());
            vertexEdges[i].reserve(3);
        }

        std::vector<Face> incidentFaces;
        for (unsigned int vidx = 0; vidx < T.vertices.size(); vidx++) {
            Vertex2D v = T.vertices[vidx];
            if (isBoundaryVertex(T, v)) {
                continue;
            }

            incidentFaces.clear();
            T.getIncidentFaces(v, incidentFaces);

            for (unsigned int fidx = 0; fidx < incidentFaces.size(); fidx++) {
                Face fi = incidentFaces[fidx];
                Face fj = fidx == 0 ? incidentFaces.back() : incidentFaces[fidx - 1];

                int refi = delaunayFaceToVertexTable[fi.id];
                int refj = delaunayFaceToVertexTable[fj.id];
                Vertex2D vi = V.getVertex(refi);
                Vertex2D vj = V.getVertex(refj);

                HalfEdge eij = V.createHalfEdge();
                eij.origin = vi.id;
                V.updateHalfEdge(eij);

                std::pair<int, int> vertexEdge(vj.id, eij.id);
                vertexEdges[vi.id].push_back(vertexEdge);
            }
        }

        return vertexEdges;
    }

    void Voronoi::initVertexIncidentEdges(GraphGeometry &V, const VertexEdgeTable &vertexEdges) {
        for (unsigned int i = 0; i < vertexEdges.size(); i++) {
            if (vertexEdges[i].size() <= 0) {
                continue;
            }
            int refeij(vertexEdges[i][0].second);
            HalfEdge eij = V.getHalfEdge(refeij);
            Vertex2D vi = V.getVertex(i);
            vi.incidentEdge = eij.id;
            V.updateVertex(vi);
        }
    }

    void Voronoi::getVoronoiCellEdgeLoop(const Vertex2D &delaunayVertex,
        GraphGeometry &T, GraphGeometry &V,
        const std::vector<int> &delaunayFaceToVertexTable,
        const VertexEdgeTable &vertexEdges,
        std::vector<HalfEdge> &edgeLoop) {

        std::vector<Face> incidentFaces;
        incidentFaces.reserve(6);
        T.getIncidentFaces(delaunayVertex, incidentFaces);

        for (unsigned int fidx = 0; fidx < incidentFaces.size(); fidx++) {
            Face fi = incidentFaces[fidx];
            Face fj = fidx == 0 ? incidentFaces.back() : incidentFaces[fidx - 1];

            int refi = delaunayFaceToVertexTable[fi.id];
            int refj = delaunayFaceToVertexTable[fj.id];
            Vertex2D vi = V.getVertex(refi);
            Vertex2D vj = V.getVertex(refj);

            HalfEdge eij;
            for (unsigned int eidx = 0; eidx < vertexEdges[vi.id].size(); eidx++) {
                std::pair<int, int> vertexEdge = vertexEdges[vi.id][eidx];
                if (vertexEdge.first == vj.id) {
                    int refeij(vertexEdge.second);
                    eij = V.getHalfEdge(refeij);
                }
            }

            edgeLoop.push_back(eij);
        }
    }

    void Voronoi::initVoronoiFaceFromEdgeLoop(const std::vector<HalfEdge> &edgeLoop,
        GraphGeometry &V,
        VertexEdgeTable &vertexEdges) {

        Face cellface = V.createFace();
        cellface.outerComponent = edgeLoop[0].id;
        V.updateFace(cellface);

        HalfEdge eij, ejk, eri, eji;
        Vertex2D vi, vj;
        for (unsigned hidx = 0; hidx < edgeLoop.size(); hidx++) {
            eij = edgeLoop[hidx];
            ejk = hidx == 0 ? edgeLoop.back() : edgeLoop[hidx - 1];
            eri = hidx == edgeLoop.size() - 1 ? edgeLoop[0] : edgeLoop[hidx + 1];
            vi = V.origin(eij);
            vj = V.origin(ejk);

            bool isHalfEdgeFound = false;
            for (unsigned int eidx = 0; eidx < vertexEdges[vj.id].size(); eidx++) {
                std::pair<int, int> vertexEdge = vertexEdges[vj.id][eidx];
                if (vertexEdge.first == vi.id) {
                    eji = V.getHalfEdge(vertexEdge.second);
                    isHalfEdgeFound = true;
                    break;
                }
            }

            if (!isHalfEdgeFound) {
                eji = V.createHalfEdge();
                eji.origin = vj.id;
                eji.twin = eij.id;
                V.updateHalfEdge(eji);

                std::pair<int, int> vertexEdge(vi.id, eji.id);
                vertexEdges[vj.id].push_back(vertexEdge);
            }

            eij.origin = vi.id;
            eij.twin = eji.id;
            eij.incidentFace = cellface.id;
            eij.next = ejk.id;
            eij.prev = eri.id;
            V.updateHalfEdge(eij);
        }
    }
}
