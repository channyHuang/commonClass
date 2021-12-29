#pragma once

#include "vertexmap.h"

namespace Graph_Geometry {
    //record height of each vertices in voronoi
    template <class T>
    class NodeMap {

    public:
        NodeMap() {}
        NodeMap(VertexMap *vertexMap) : m_vertexMap(vertexMap) {
            initializeNodes();
        }

        NodeMap(VertexMap *vertexMap, T fillval) : m_vertexMap(vertexMap) {
            initializeNodes();
            fill(fillval);
        }

        unsigned int size() {
            return m_vertexMap->size();
        }

        T min() {
            T minval = m_nodes[0];
            for (unsigned int i = 0; i < m_nodes.size(); i++) {
                if (m_nodes[i] < minval) {
                    minval = m_nodes[i];
                }
            }
            return minval;
        }

        T max() {
            T maxval = m_nodes[0];
            for (unsigned int i = 0; i < m_nodes.size(); i++) {
                if (m_nodes[i] > maxval) {
                    maxval = m_nodes[i];
                }
            }
            return maxval;
        }

        int getNodeIndex(const Vertex2D &v) {
            return m_vertexMap->getVertexIndex(v);
        }

        void fill(T fillval) {
            for (unsigned int i = 0; i < m_nodes.size(); i++) {
                m_nodes[i] = fillval;
            }
        }

        T operator()(int idx) {
            if (!isInRange(idx)) {
                return T(0);
            }
            return m_nodes[idx];
        }

        T operator()(const Vertex2D &v) {
            int idx = getNodeIndex(v);
            if (idx == -1) {
                return T(0);
            }
            return m_nodes[idx];
        }

        T* getPointer(int idx) {
            if (!isInRange(idx)) {
                return &m_nodes[idx];
            }
            return &m_nodes[idx];
        }

        T* getPointer(const Vertex2D &v) {
            int idx = getNodeIndex(v);
            if (idx == -1) {
                return &m_nodes[0];
            }
            return &m_nodes[idx];
        }

        void set(int idx, T val) {
            if (!isInRange(idx)) {
                return;
            }
            m_nodes[idx] = val;
        }

        void set(const Vertex2D &v, T val) {
            int idx = getNodeIndex(v);
            if (idx == -1 || !isInRange(idx)) {
                return;
            }
            m_nodes[idx] = val;
        }

        void getNeighbours(int idx, std::vector<T> &nbs) {
            if (!isInRange(idx)) {
                return;
            }
            std::vector<int> indices;
            indices.reserve(3);
            m_vertexMap->getNeighbourIndices(m_vertexMap->vertices[idx], indices);

            for (unsigned int i = 0; i < indices.size(); i++) {
                nbs.push_back(m_nodes[indices[i]]);
            }
        }

        void getNeighbours(const Vertex2D &v, std::vector<T> &nbs) {
            int idx = getNodeIndex(v);
            if (idx == -1) {
                return;
            }
            std::vector<int> indices;
            indices.reserve(3);
            m_vertexMap->getNeighbourIndices(v, indices);

            for (unsigned int i = 0; i < indices.size(); i++) {
                nbs.push_back(m_nodes[indices[i]]);
            }
        }

        bool isNode(int idx) {
            return isInRange(idx);
        }

        bool isNode(const Vertex2D &v) {
            int idx = getNodeIndex(v);
            return idx != -1;
        }

        bool isEdge(int idx) {
            if (!isInRange(idx)) {
                return false;
            }
            return m_vertexMap->isEdge(m_vertexMap->vertices[idx]);
        }

        bool isEdge(const Vertex2D &v) {
            return m_vertexMap->isEdge(v);
        }

        bool isInterior(int idx) {
            if (!isInRange(idx)) {
                return false;
            }
            return m_vertexMap->isInterior(m_vertexMap->vertices[idx]);
        }

        bool isInterior(const Vertex2D &v) {
            return m_vertexMap->isInterior(v);
        }

        // Normalize height map values to range [0, 1]
        void normalize() {
            real min = std::numeric_limits<real>::infinity();
            real max = -std::numeric_limits<real>::infinity();
            for (unsigned int i = 0; i < size(); i++) {
                min = fmin(min, (*this)(i));
                max = fmax(max, (*this)(i));
            }

            for (unsigned int i = 0; i < size(); i++) {
                real val = (*this)(i);
                real normalized = (val - min) / (max - min);
                set(i, normalized);
            }
        }

        // Normalize height map and square root the values
        void round() {
            normalize();
            for (unsigned int i = 0; i < size(); i++) {
                real rounded = sqrt((*this)(i));
                set(i, rounded);
            }
        }

        //  Replace height with average of its neighbours
        void relax() {
            std::vector<real> averages;
            averages.reserve(size());

            std::vector<real> nbs;
            for (unsigned int i = 0; i < size(); i++) {
                nbs.clear();
                getNeighbours(i, nbs);
                if (nbs.size() == 0) {
                    continue;
                }

                real sum = 0.0;
                for (unsigned int nidx = 0; nidx < nbs.size(); nidx++) {
                    sum += nbs[nidx];
                }
                averages.push_back(sum / nbs.size());
            }

            for (unsigned int i = 0; i < size(); i++) {
                if (i >= averages.size()) break;
                set(i, averages[i]);
            }
        }


    private:
        void initializeNodes() {
            m_nodes = std::vector<T>(size(), T());
        }

        bool isInRange(int idx) {
            return idx >= 0 && idx < (int)m_nodes.size();
        }

        VertexMap *m_vertexMap;
        std::vector<T> m_nodes;
    };
}
