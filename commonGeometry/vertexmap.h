#ifndef VERTEXMAP_H
#define VERTEXMAP_H

#include <stdio.h>
#include <iostream>
#include <vector>

#include "commonMath/box.h"
#include "graphgeometry.h"

namespace Graph_Geometry {
    //record voronoi base on vertex
    class VertexMap {
    public:
        enum class VertexType : char {
            edge = 0x00,
            interior = 0x01
        };

        VertexMap();
        VertexMap(GraphGeometry *V, Box extents);

        unsigned int size();
        void getNeighbours(const Vertex2D &v, std::vector<Vertex2D> &nbs);
        void getNeighbourIndices(const Vertex2D &v, std::vector<int> &nbs);
        int getVertexIndex(const Vertex2D &v);
        bool isVertex(const Vertex2D &v);
        bool isEdge(const Vertex2D &v);
        bool isInterior(const Vertex2D &v);

        std::vector<Vertex2D> vertices;
        std::vector<Vertex2D> edge;
        std::vector<Vertex2D> interior;

    private:
        bool isBoundaryVertex(const Vertex2D &v);
        bool isInRange(int id);
        VertexType getVertexType(const Vertex2D &v);

        GraphGeometry *m_cGraphGeometry;
        Box m_cBoundingBox;
        std::vector<int> m_vertexIdToMapIndex;
        std::vector<VertexType> m_vertexTypes;
    };
}

#endif
