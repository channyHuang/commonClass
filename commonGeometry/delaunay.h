#ifndef DELAUNAY_H
#define DELAUNAY_H

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdlib.h>

#include "vector2.h"
#include "vertex.h"
#include "halfedge.h"
#include "face.h"

#include "graphgeometry.h"
#include "geometrymath.h"

namespace Graph_Geometry {
    class Delaunay {
    public:
        Delaunay() {};

        GraphGeometry triangulate(std::vector<Vector2> &points);

    private:
        void getSuperTriangle(const std::vector<Vector2> &points,
            Vector2 *p1, Vector2 *p2, Vector2 *p3);
        GraphGeometry initTriangulation(const std::vector<Vector2> &points);
        Face locateTriangleAtPoint(Vector2 &p, GraphGeometry &T);
        Vector2 computeTriangleCentroid(const Face &f, GraphGeometry &T);
        bool isSegmentIntersectingEdge(const Vector2 &p0, const Vector2 &p1, const HalfEdge &h, GraphGeometry &T);
        bool isPointInsideTriangle(const Vector2 &p, const Face &f, GraphGeometry &T);
        real pointToEdgeDistance(Vector2 &p, HalfEdge &h, GraphGeometry &T);
        void insertPointIntoTriangulation(Vector2 p, Face f, GraphGeometry &T);
        void insertPointIntoTriangle(Vector2 p, Face f, GraphGeometry &T);
        void insertPointIntoTriangleEdge(Vector2 p, Face f, HalfEdge h, GraphGeometry &T);
        bool isEdgeLegal(Vertex2D p0, HalfEdge e, GraphGeometry &T);
        void legalizeEdge(Vertex2D p, HalfEdge h, GraphGeometry &T);
        void cleanup(GraphGeometry &T);
        void getCleanupInvalidFaces(GraphGeometry &T, std::vector<Face> &invalidFaces);
        void getCleanupInvalidEdges(GraphGeometry &T, std::vector<HalfEdge> &invalidEdges,
            std::vector<HalfEdge> &invalidTwins);
        void getCleanupUpdateVertices(GraphGeometry &T,
            std::vector<HalfEdge> &invalidEdges,
            std::vector<Vertex2D> &vertices);
        void updateCleanupVertices(GraphGeometry &T,
            std::vector<Vertex2D> &updateVertices,
            std::vector<uint8_t> &invalidEdgeTable,
            std::vector<uint8_t> &invalidFaceTable);
        void removeInvalidCleanupComponents(GraphGeometry &T,
            std::vector<HalfEdge> &invalidEdges,
            std::vector<Face> &invalidFaces);

    };
}

#endif
