#include "graphgeometry.h"

namespace Graph_Geometry {
    void GraphGeometry::getOuterComponents(const Face &f, std::vector<HalfEdge> &edges) {
        HalfEdge h = outerComponent(f);
        int startid = h.id;

        do {
            edges.push_back(h);
            h = next(h);
        } while (h.id != startid);
    }

    void GraphGeometry::getOuterComponents(const Face &f, std::vector<int> &edges) {
        HalfEdge h = outerComponent(f);
        int startid = h.id;

        do {
            edges.push_back(h.id);
            h = next(h);
        } while (h.id != startid);
    }

    void GraphGeometry::getIncidentEdges(const Vertex2D &v, std::vector<HalfEdge> &edges) {
        HalfEdge h = incidentEdge(v);
        int startid = h.id;

        do {
            edges.push_back(h);
            h = next(twin(h));
        } while (h.id != startid);
    }

    void GraphGeometry::getIncidentEdges(const Vertex2D &v, std::vector<int> &edges) {
        HalfEdge h = incidentEdge(v);
        int startid = h.id;

        do {
            edges.push_back(h.id);
            h = next(twin(h));
        } while (h.id != startid);
    }

    void GraphGeometry::getIncidentFaces(const Vertex2D &v, std::vector<Face> &faces) {
        HalfEdge h = incidentEdge(v);
        int startid = h.id;

        do {
            if (!isBoundary(h)) {
                faces.push_back(incidentFace(h));
            }
            h = next(twin(h));
        } while (h.id != startid);
    }

    void GraphGeometry::getIncidentFaces(const Vertex2D &v, std::vector<int> &faces) {
        HalfEdge h = incidentEdge(v);
        int startid = h.id;

        do {
            if (!isBoundary(h)) {
                faces.push_back(incidentFace(h).id);
            }
            h = next(twin(h));
        } while (h.id != startid);
    }

    bool GraphGeometry::isBoundary(const HalfEdge &h) {
        return h.incidentFace == -1;
    }

}
