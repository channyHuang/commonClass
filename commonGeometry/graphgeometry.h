#ifndef GRAPHGEOMETRY_H
#define GRAPHGEOMETRY_H

#include "vertex.h"
#include "face.h"
#include "halfedge.h"

namespace Graph_Geometry {
class GraphGeometry
{
public:
    GraphGeometry() {}
    ~GraphGeometry() {}

    inline Vertex2D const createVertex(const Vector2 &p) {
        return createVertex(p.x, p.y);
    }

    inline Vertex2D const createVertex(float px, float py) {
        Vertex2D vert(px, py);
        vert.id = (int)vertices.size();
        vertices.push_back(vert);
        return vert;
    }

    inline HalfEdge const createHalfEdge() {
        HalfEdge edge;
        edge.id = (int)edges.size();
        edges.push_back(edge);
        return edge;
    }

    inline Face const createFace() {
        Face face;
        face.id = (int)faces.size();
        faces.push_back(face);
        return face;
    }


    inline Vertex2D const getVertex(int id) {
        if (!isVertexInRange(id)) {
            return Vertex2D();
        }
        return vertices[id];
    }

    inline HalfEdge const getHalfEdge(int id) {
        if (!isHalfEdgeInRange(id)) {
            return HalfEdge();
        }
        return edges[id];
    }


    inline Face getFace(int id) {
        if (!isFaceInRange(id)) {
            return Face();
        }
        return faces[id];
    }

    inline void updateVertex(const Vertex2D &v) {
        if (!isVertexInRange(v)) {
            return;
        }
        vertices[v.id] = v;
    }

    inline void updateHalfEdge(const HalfEdge &e) {
        if (!isHalfEdgeInRange(e)) {
            return;
        }
        edges[e.id] = e;
    }

    inline void updateFace(const Face &f) {
        if (!isFaceInRange(f)) {
            return;
        }
        faces[f.id] = f;
    }

    inline Vertex2D const origin(const HalfEdge &h) {
        if (!isVertexInRange(h.origin)) {
            return Vertex2D();
        }
        return vertices[h.origin];
    }

    inline HalfEdge const twin(const HalfEdge &h) {
        if (!isHalfEdgeInRange(h.twin)) {
            return HalfEdge();
        }
        return edges[h.twin];
    }

    inline Face const incidentFace(const HalfEdge &h) {
        if (!isFaceInRange(h.incidentFace)) {
            return Face();
        }
        return faces[h.incidentFace];
    }

    inline HalfEdge const next(const HalfEdge &h) {
        if (!isHalfEdgeInRange(h.next)) {
            return HalfEdge();
        }
        return edges[h.next];
    }

    inline HalfEdge const prev(const HalfEdge &h) {
        if (!isHalfEdgeInRange(h.prev)) {
            return HalfEdge();
        }
        return edges[h.prev];
    }

    inline HalfEdge const outerComponent(const Face &f) {
        if (!isHalfEdgeInRange(f.outerComponent)) {
            return HalfEdge();
        }
        return edges[f.outerComponent];
    }

    inline HalfEdge const incidentEdge(const Vertex2D &v) {
        if (!isHalfEdgeInRange(v.incidentEdge)) {
            return HalfEdge();
        }
        return edges[v.incidentEdge];
    }

    inline bool const isVertexInRange(const Vertex2D &v) {
        return v.id >= 0 && v.id < (int)vertices.size();
    }

    inline bool const isVertexInRange(int id) {
        return id >= 0 && id < (int)vertices.size();
    }

    inline bool isHalfEdgeInRange(const HalfEdge &h) {
        return h.id >= 0 && h.id < (int)edges.size();
    }

    inline bool isHalfEdgeInRange(int id) {
        return id >= 0 && id < (int)edges.size();
    }

    inline bool isFaceInRange(const Face &f) {
        return f.id >= 0 && f.id < (int)faces.size();
    }

    inline bool isFaceInRange(int id) {
        return id >= 0 && id < (int)faces.size();
    }

    void getOuterComponents(const Face &f, std::vector<HalfEdge> &edges);
    void getOuterComponents(const Face &f, std::vector<int> &edges);
    void getIncidentEdges(const Vertex2D &v, std::vector<HalfEdge> &edges);
    void getIncidentEdges(const Vertex2D &v, std::vector<int> &edges);
    void getIncidentFaces(const Vertex2D &v, std::vector<Face> &faces);
    void getIncidentFaces(const Vertex2D &v, std::vector<int> &faces);
    bool isBoundary(const HalfEdge &h);

    std::vector<Vertex2D> vertices;
    std::vector<HalfEdge> edges;
    std::vector<Face> faces;
};
}

#endif
