#include "vertexmap.h"

namespace Graph_Geometry {
    VertexMap::VertexMap() {}

    VertexMap::VertexMap(GraphGeometry *V, Box extents) :
        m_cGraphGeometry(V), m_cBoundingBox(extents) {
        vertices.reserve(m_cGraphGeometry->vertices.size());
        interior.reserve(m_cGraphGeometry->vertices.size());
        m_vertexTypes.reserve(m_cGraphGeometry->vertices.size());
        m_vertexIdToMapIndex = std::vector<int>(m_cGraphGeometry->vertices.size(), -1);

        Vertex2D v;
        for (unsigned int i = 0; i < m_cGraphGeometry->vertices.size(); i++) {
            v = m_cGraphGeometry->vertices[i];
            if (!extents.isPointInside(Vector3(v.vPos.x, v.vPos.y, 0)) || isBoundaryVertex(v)) {
                continue;
            }

            vertices.push_back(v);
            m_vertexIdToMapIndex[v.id] = int(vertices.size() - 1);

            if (getVertexType(v) == VertexType::interior) {
                interior.push_back(v);
                m_vertexTypes.push_back(VertexType::interior);
            }
            else {
                edge.push_back(v);
                m_vertexTypes.push_back(VertexType::edge);
            }
        }
    }

    unsigned int VertexMap::size() {
        return (unsigned int)vertices.size();
    }

    void VertexMap::getNeighbours(const Vertex2D &v,
        std::vector<Vertex2D> &nbs) {
        HalfEdge h = m_cGraphGeometry->incidentEdge(v);
        int startRef = h.id;

        HalfEdge twin;
        Vertex2D n;
        do {
            twin = m_cGraphGeometry->twin(h);
            n = m_cGraphGeometry->origin(twin);
            if (isVertex(n)) {
                nbs.push_back(n);
            }
            h = m_cGraphGeometry->next(twin);
        } while (h.id != startRef);
    }

    void VertexMap::getNeighbourIndices(const Vertex2D &v, std::vector<int> &nbs) {
        HalfEdge h = m_cGraphGeometry->incidentEdge(v);
        int startRef = h.id;

        HalfEdge twin;
        Vertex2D n;
        do {
            twin = m_cGraphGeometry->twin(h);
            n = m_cGraphGeometry->origin(twin);
            if (isVertex(n)) {
                nbs.push_back(getVertexIndex(n));
            }
            h = m_cGraphGeometry->next(twin);
        } while (h.id != startRef);
    }

    int VertexMap::getVertexIndex(const Vertex2D &v) {
        if (!isInRange(v.id)) {
            return -1;
        }
        return m_vertexIdToMapIndex[v.id];
    }

    bool VertexMap::isVertex(const Vertex2D &v) {
        if (!isInRange(v.id)) {
            return false;
        }
        return getVertexIndex(v) != -1;
    }

    bool VertexMap::isEdge(const Vertex2D &v) {
        if (!isInRange(v.id)) {
            return false;
        }
        return m_vertexTypes[getVertexIndex(v)] == VertexType::edge;
    }

    bool VertexMap::isInterior(const Vertex2D &v) {
        if (!isInRange(v.id)) {
            return false;
        }
        return m_vertexTypes[getVertexIndex(v)] == VertexType::interior;
    }

    bool VertexMap::isBoundaryVertex(const Vertex2D &v) {
        if (v.incidentEdge == -1) {
            return true;
        }

        HalfEdge h = m_cGraphGeometry->incidentEdge(v);
        int startid = h.id;

        do {
            if (h.incidentFace == -1) {
                return true;
            }

            h = m_cGraphGeometry->twin(h);
            if (h.next == -1) {
                return true;
            }

            h = m_cGraphGeometry->next(h);
        } while (h.id != startid);

        return false;
    }

    VertexMap::VertexType VertexMap::getVertexType(const Vertex2D &v) {
        HalfEdge h = m_cGraphGeometry->incidentEdge(v);
        int startRef = h.id;

        HalfEdge twin;
        Vertex2D n;
        int ncount = 0;
        do {
            twin = m_cGraphGeometry->twin(h);
            n = m_cGraphGeometry->origin(twin);
            if (m_cBoundingBox.isPointInside(Vector3(n.vPos.x, n.vPos.y, 0)) && !isBoundaryVertex(n)) {
                ncount++;
            }
            h = m_cGraphGeometry->next(twin);
        } while (h.id != startRef);

        if (ncount < 3) {
            return VertexType::edge;
        }
        else {
            return VertexType::interior;
        }
    }

    bool VertexMap::isInRange(int id) {
        return id >= 0 && id < (int)m_vertexIdToMapIndex.size();
    }
}
