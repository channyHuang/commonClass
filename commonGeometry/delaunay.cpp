#include "delaunay.h"

namespace Graph_Geometry {
    GraphGeometry Delaunay::triangulate(std::vector<Vector2> &points) {
        if (points.size() == 0) {
            return GraphGeometry();
        }

        GraphGeometry T = initTriangulation(points);
        while (!points.empty()) {
            Vector2 p = points.back();
            points.pop_back();

            Face f = locateTriangleAtPoint(p, T);
            if (f.id != -1) {
                insertPointIntoTriangulation(p, f, T);

                mapIndex[T.vertices.size() - 1] = points.size();
            }
        }

        cleanup(T);

        return T;
    }

    void Delaunay::getSuperTriangle(const std::vector<Vector2> &points,
        Vector2 *p1, Vector2 *p2, Vector2 *p3) {
        real eps = 1e-3f;
        real minx = points[0].x;
        real miny = points[0].y;
        real maxx = minx + eps;
        real maxy = miny + eps;

        for (unsigned int i = 0; i < points.size(); i++) {
            Vector2 p = points[i];
            if (p.x < minx) { minx = p.x; }
            if (p.y < miny) { miny = p.y; }
            if (p.x > maxx) { maxx = p.x; }
            if (p.y > maxy) { maxy = p.y; }
        }

        real expand = fmax(0.1f * (maxx - minx), 0.1f * (maxy - miny));
        minx -= expand;
        miny -= 5 * expand;
        maxx += expand;
        maxy += expand;

        p1->x = 0.5f * (minx + maxx);
        p1->y = maxy + 0.5f * (maxy - miny);

        real m = (maxy - p1->y) / (maxx - p1->x);
        p2->x = (1.0f / m) * (miny - p1->y + m * p1->x);
        p2->y = miny;

        m = (maxy - p1->y) / (minx - p1->x);
        p3->x = (1.0f / m) * (miny - p1->y + m * p1->x);
        p3->y = miny;
    }

    GraphGeometry Delaunay::initTriangulation(const std::vector<Vector2> &points) {
        Vector2 s1, s2, s3;
        getSuperTriangle(points, &s1, &s2, &s3);

        GraphGeometry T;
        Vertex2D p1 = T.createVertex(s1);
        Vertex2D p2 = T.createVertex(s2);
        Vertex2D p3 = T.createVertex(s3);
        HalfEdge e12 = T.createHalfEdge();
        HalfEdge e23 = T.createHalfEdge();
        HalfEdge e31 = T.createHalfEdge();
        HalfEdge e13 = T.createHalfEdge();
        HalfEdge e32 = T.createHalfEdge();
        HalfEdge e21 = T.createHalfEdge();
        Face f0 = T.createFace();

        p1.incidentEdge = e12.id;
        p2.incidentEdge = e23.id;
        p3.incidentEdge = e31.id;
        T.updateVertex(p1);
        T.updateVertex(p2);
        T.updateVertex(p3);

        e12.origin = p1.id;
        e12.twin = e21.id;
        e12.incidentFace = f0.id;
        e12.next = e23.id;
        e12.prev = e31.id;
        T.updateHalfEdge(e12);

        e23.origin = p2.id;
        e23.twin = e32.id;
        e23.incidentFace = f0.id;
        e23.next = e31.id;
        e23.prev = e12.id;
        T.updateHalfEdge(e23);

        e31.origin = p3.id;
        e31.twin = e13.id;
        e31.incidentFace = f0.id;
        e31.next = e12.id;
        e31.prev = e23.id;
        T.updateHalfEdge(e31);

        e13.origin = p1.id;
        e13.twin = e31.id;
        e13.next = e32.id;
        e13.prev = e21.id;
        T.updateHalfEdge(e13);

        e32.origin = p3.id;
        e32.twin = e23.id;
        e32.next = e21.id;
        e32.prev = e13.id;
        T.updateHalfEdge(e32);

        e21.origin = p2.id;
        e21.twin = e12.id;
        e21.next = e13.id;
        e21.prev = e32.id;
        T.updateHalfEdge(e21);

        f0.outerComponent = e12.id;
        T.updateFace(f0);

        return T;
    }

    // Choose a random triangle, walk toward p until the containing
    // triangle is found.
    Face Delaunay::locateTriangleAtPoint(Vector2 &p, GraphGeometry &T) {
        int randfidx(rand() % T.faces.size());
        Face f = T.getFace(randfidx);

        int count = 0;
        int maxcount = (int)(2.0*sqrt(T.faces.size()));

        int faceHistory[3] = { -1, -1, -1 };
        HalfEdge h;
        Vector2 p0;
        for (;;) {
            if (isPointInsideTriangle(p, f, T)) {
                return f;
            }

            p0 = computeTriangleCentroid(f, T);
            bool isNeighbourFound = false;
            h = T.outerComponent(f);
            for (int i = 0; i < 3; i++) {
                if (isSegmentIntersectingEdge(p0, p, h, T)) {
                    f = T.incidentFace(T.twin(h));
                    isNeighbourFound = true;
                }
                h = T.next(h);
            }

            if (!isNeighbourFound) {
                break;
            }

            // Due to numerical error, a point may not report to be contained
            // in any triangle. In this case, the search will travel back and
            // forth between two faces. Tracking the last two face id's will
            // allow cycles to be detected
            faceHistory[2] = faceHistory[1];
            faceHistory[1] = faceHistory[0];
            faceHistory[0] = f.id;
            if (faceHistory[0] == faceHistory[2]) {
                break;
            }

            count++;
            if (count > maxcount) {
                break;
            }
        }

        return Face();
    }

    bool Delaunay::isPointInsideTriangle(const Vector2 &p, const Face &f, GraphGeometry &T) {
        HalfEdge h = T.outerComponent(f);
        const Vector2 p0 = T.origin(h).vPos;
        h = T.next(h);
        Vector2 p1 = T.origin(h).vPos;
        Vector2 p2 = T.origin(T.next(h)).vPos;

        real area = 0.5f * (-p1.y*p2.x + p0.y*(-p1.x + p2.x) + p0.x*(p1.y - p2.y) + p1.x*p2.y);
        real s = 1.0f / (2.0f * area)*(p0.y*p2.x - p0.x*p2.y + (p2.y - p0.y)*p.x + (p0.x - p2.x)*p.y);
        real t = 1.0f / (2.0f * area)*(p0.x*p1.y - p0.y*p1.x + (p0.y - p1.y)*p.x + (p1.x - p0.x)*p.y);

        return s >= 0 && t >= 0 && 1 - s - t >= 0;
    }

    Vector2 Delaunay::computeTriangleCentroid(const Face &f, GraphGeometry &T) {
        HalfEdge h = T.outerComponent(f);
        Vector2 p0 = T.origin(h).vPos;
        Vector2 p1 = T.origin(T.next(h)).vPos;
        Vector2 p2 = T.origin(T.prev(h)).vPos;

        real frac = 1.0f / 3.0f;
        return Vector2(frac*(p0.x + p1.x + p2.x), frac*(p0.y + p1.y + p2.y));
    }

    bool Delaunay::isSegmentIntersectingEdge(const Vector2 &A, const Vector2 &B,
        const HalfEdge &h, GraphGeometry &T) {
        Vector2 C = T.origin(h).vPos;
        Vector2 D = T.origin(T.twin(h)).vPos;
        return lineSegmentIntersection(A, B, C, D);
    }

    real Delaunay::pointToEdgeDistance(Vector2 &p0, HalfEdge &h, GraphGeometry &T) {
        Vector2 p1 = T.origin(h).vPos;
        Vector2 p2 = T.origin(T.twin(h)).vPos;

        real vx = p2.x - p1.x;
        real vy = p2.y - p1.y;
        real len = sqrt(vx*vx + vy * vy);

        return fabs((vx*(p1.y - p0.y) - (p1.x - p0.x)*vy) / len);
    }

    void Delaunay::insertPointIntoTriangulation(Vector2 p, Face f, GraphGeometry &T) {
        real eps = 1e-9f;
        int closeEdgeCount = 0;
        HalfEdge closeEdge;

        HalfEdge h = T.outerComponent(f);
        for (int i = 0; i < 3; i++) {
            real dist = pointToEdgeDistance(p, h, T);
            if (dist < eps) {
                closeEdge = h;
                closeEdgeCount++;

                if (closeEdgeCount == 2) {
                    // too close to an existing vertex
                    return;
                }
            }

            if (i < 2) {
                h = T.next(h);
            }
        }

        if (closeEdgeCount == 0) {
            insertPointIntoTriangle(p, f, T);
        }
        else {
            insertPointIntoTriangleEdge(p, f, closeEdge, T);
        }
    }

    void Delaunay::insertPointIntoTriangle(Vector2 p, Face f, GraphGeometry &T) {
        // existing components
        HalfEdge eij = T.outerComponent(f);
        HalfEdge ejk = T.next(eij);
        HalfEdge eki = T.next(ejk);

        Face f1 = f;

        Vertex2D pi = T.origin(eij);
        Vertex2D pj = T.origin(ejk);
        Vertex2D pk = T.origin(eki);

        // new components
        HalfEdge eri = T.createHalfEdge();
        HalfEdge eir = T.createHalfEdge();
        HalfEdge erj = T.createHalfEdge();
        HalfEdge ejr = T.createHalfEdge();
        HalfEdge erk = T.createHalfEdge();
        HalfEdge ekr = T.createHalfEdge();

        Face f2 = T.createFace();
        Face f3 = T.createFace();

        Vertex2D pr = T.createVertex(p);

        // update existing components
        eij.next = ejr.id;
        eij.prev = eri.id;
        T.updateHalfEdge(eij);

        ejk.incidentFace = f2.id;
        ejk.next = ekr.id;
        ejk.prev = erj.id;
        T.updateHalfEdge(ejk);

        eki.incidentFace = f3.id;
        eki.next = eir.id;
        eki.prev = erk.id;
        T.updateHalfEdge(eki);

        f1.outerComponent = eij.id;
        T.updateFace(f1);

        // initialize new components
        eri.origin = pr.id;
        eri.twin = eir.id;
        eri.incidentFace = f1.id;
        eri.next = eij.id;
        eri.prev = ejr.id;
        T.updateHalfEdge(eri);

        eir.origin = pi.id;
        eir.twin = eri.id;
        eir.incidentFace = f3.id;
        eir.next = erk.id;
        eir.prev = eki.id;
        T.updateHalfEdge(eir);

        erj.origin = pr.id;
        erj.twin = ejr.id;
        erj.incidentFace = f2.id;
        erj.next = ejk.id;
        erj.prev = ekr.id;
        T.updateHalfEdge(erj);

        ejr.origin = pj.id;
        ejr.twin = erj.id;
        ejr.incidentFace = f1.id;
        ejr.next = eri.id;
        ejr.prev = eij.id;
        T.updateHalfEdge(ejr);

        erk.origin = pr.id;
        erk.twin = ekr.id;
        erk.incidentFace = f3.id;
        erk.next = eki.id;
        erk.prev = eir.id;
        T.updateHalfEdge(erk);

        ekr.origin = pk.id;
        ekr.twin = erk.id;
        ekr.incidentFace = f2.id;
        ekr.next = erj.id;
        ekr.prev = ejk.id;
        T.updateHalfEdge(ekr);

        f2.outerComponent = ejk.id;
        T.updateFace(f2);

        f3.outerComponent = eki.id;
        T.updateFace(f3);

        pr.incidentEdge = eri.id;
        T.updateVertex(pr);

        legalizeEdge(pr, eij, T);
        legalizeEdge(pr, ejk, T);
        legalizeEdge(pr, eki, T);
    }

    void Delaunay::insertPointIntoTriangleEdge(Vector2 p, Face f,
        HalfEdge h, GraphGeometry &T) {
        // existing components
        HalfEdge eij = h;
        HalfEdge ejk = T.next(eij);
        HalfEdge eki = T.next(ejk);

        HalfEdge eji = T.twin(eij);
        HalfEdge eil = T.next(eji);
        HalfEdge elj = T.next(eil);

        Face f1 = T.incidentFace(eij);
        Face f2 = T.incidentFace(eji);

        Vertex2D pj = T.origin(ejk);
        Vertex2D pk = T.origin(eki);
        Vertex2D pl = T.origin(elj);

        // component replacements
        HalfEdge eir = eij;
        HalfEdge eri = eji;

        // new components
        HalfEdge erj = T.createHalfEdge();
        HalfEdge ejr = T.createHalfEdge();
        HalfEdge erk = T.createHalfEdge();
        HalfEdge ekr = T.createHalfEdge();
        HalfEdge erl = T.createHalfEdge();
        HalfEdge elr = T.createHalfEdge();

        Face f3 = T.createFace();
        Face f4 = T.createFace();

        Vertex2D pr = T.createVertex(p);

        // update existing components
        ejk.incidentFace = f4.id;
        ejk.next = ekr.id;
        ejk.prev = erj.id;
        T.updateHalfEdge(ejk);

        eki.next = eir.id;
        eki.prev = erk.id;
        T.updateHalfEdge(eki);

        eil.next = elr.id;
        eil.prev = eri.id;
        T.updateHalfEdge(eil);

        elj.incidentFace = f3.id;
        elj.next = ejr.id;
        elj.prev = erl.id;
        T.updateHalfEdge(elj);

        f1.outerComponent = eki.id;
        T.updateFace(f1);

        f2.outerComponent = eil.id;
        T.updateFace(f2);

        pj.incidentEdge = ejk.id;
        T.updateVertex(pj);

        // update replacement components
        eir.next = erk.id;
        eir.prev = eki.id;
        T.updateHalfEdge(eir);

        eri.origin = pr.id;
        eri.next = eil.id;
        eri.prev = elr.id;
        T.updateHalfEdge(eri);

        // initialize new components
        erj.origin = pr.id;
        erj.twin = ejr.id;
        erj.incidentFace = f4.id;
        erj.next = ejk.id;
        erj.prev = ekr.id;
        T.updateHalfEdge(erj);

        ejr.origin = pj.id;
        ejr.twin = erj.id;
        ejr.incidentFace = f3.id;
        ejr.next = erl.id;
        ejr.prev = elj.id;
        T.updateHalfEdge(ejr);

        erk.origin = pr.id;
        erk.twin = ekr.id;
        erk.incidentFace = f1.id;
        erk.next = eki.id;
        erk.prev = eir.id;
        T.updateHalfEdge(erk);

        ekr.origin = pk.id;
        ekr.twin = erk.id;
        ekr.incidentFace = f4.id;
        ekr.next = erj.id;
        ekr.prev = ejk.id;
        T.updateHalfEdge(ekr);

        erl.origin = pr.id;
        erl.twin = elr.id;
        erl.incidentFace = f3.id;
        erl.next = elj.id;
        erl.prev = ejr.id;
        T.updateHalfEdge(erl);

        elr.origin = pl.id;
        elr.twin = erl.id;
        elr.incidentFace = f2.id;
        elr.next = eri.id;
        elr.prev = eil.id;
        T.updateHalfEdge(elr);

        f3.outerComponent = elj.id;
        T.updateFace(f3);

        f4.outerComponent = ejk.id;
        T.updateFace(f4);

        pr.incidentEdge = eri.id;
        T.updateVertex(pr);

        legalizeEdge(pr, eil, T);
        legalizeEdge(pr, elj, T);
        legalizeEdge(pr, ejk, T);
        legalizeEdge(pr, eki, T);
    }

    bool Delaunay::isEdgeLegal(Vertex2D v, HalfEdge e, GraphGeometry &T) {
        if (T.isBoundary(T.twin(e))) {
            return true;
        }

        Vector2 p0 = v.vPos;
        Vector2 pi = T.origin(e).vPos;
        Vector2 pj = T.origin(T.twin(e)).vPos;
        Vector2 pk = T.origin(T.prev(T.twin(e))).vPos;

        /*
            Computational Geometry Algorithms and Applications
            by Mark de Berg, Otfried Cheong, Marc van Kreveld, Mark Overmars

            Theorem 9.4:

            Let edge pipj be incident to triangles p0pipj and pipjpk, and let
            C be the circle through p0, pi, and pj. The edge pipj is illegal if
            and only if the point pk lies in the interior of C.
        */

        Vector2 p(0.5f * (pi.x + pj.x), 0.5f * (pi.y + pj.y));
        Vector2 r(-(pj.y - pi.y), pj.x - pi.x);

        Vector2 q(0.5f * (pi.x + p0.x), 0.5f * (pi.y + p0.y));
        Vector2 s(-(p0.y - pi.y), p0.x - pi.x);

        Vector2 center;
        bool isIntersection = lineIntersection(p, r, q, s, &center);
        if (!isIntersection) {
            return false;
        }

        real dvx = p0.x - center.x;
        real dvy = p0.y - center.y;
        real crsq = dvx * dvx + dvy * dvy;

        real dkx = pk.x - center.x;
        real dky = pk.y - center.y;
        real distsq = dkx * dkx + dky * dky;

        return distsq >= crsq;
    }

    void Delaunay::legalizeEdge(Vertex2D pr, HalfEdge eij, GraphGeometry &T) {
        if (isEdgeLegal(pr, eij, T)) {
            return;
        }

        HalfEdge ejr = T.next(eij);
        HalfEdge eri = T.next(ejr);
        HalfEdge eji = T.twin(eij);
        HalfEdge eik = T.next(eji);
        HalfEdge ekj = T.next(eik);

        Face f1 = T.incidentFace(eij);
        Face f2 = T.incidentFace(eji);

        Vertex2D pi = T.origin(eij);
        Vertex2D pj = T.origin(eji);
        Vertex2D pk = T.origin(ekj);

        // replacement components
        HalfEdge erk = eij;
        HalfEdge ekr = eji;

        // update existing components
        ejr.incidentFace = f2.id;
        ejr.next = erk.id;
        ejr.prev = ekj.id;
        T.updateHalfEdge(ejr);

        eri.next = eik.id;
        eri.prev = ekr.id;
        T.updateHalfEdge(eri);

        eik.incidentFace = f1.id;
        eik.next = ekr.id;
        eik.prev = eri.id;
        T.updateHalfEdge(eik);

        ekj.next = ejr.id;
        ekj.prev = erk.id;
        T.updateHalfEdge(ekj);

        f1.outerComponent = ekr.id;
        T.updateFace(f1);

        f2.outerComponent = erk.id;
        T.updateFace(f2);

        pi.incidentEdge = eik.id;
        T.updateVertex(pi);

        pj.incidentEdge = ejr.id;
        T.updateVertex(pj);

        pk.incidentEdge = ekr.id;
        T.updateVertex(pk);

        pr.incidentEdge = erk.id;
        T.updateVertex(pr);

        // update replacement components
        erk.origin = pr.id;
        erk.twin = ekr.id;
        erk.incidentFace = f2.id;
        erk.next = ekj.id;
        erk.prev = ejr.id;
        T.updateHalfEdge(erk);

        ekr.origin = pk.id;
        ekr.twin = erk.id;
        ekr.incidentFace = f1.id;
        ekr.next = eri.id;
        ekr.prev = eik.id;
        T.updateHalfEdge(ekr);

        legalizeEdge(pr, eik, T);
        legalizeEdge(pr, ekj, T);
    }

    void Delaunay::getCleanupInvalidFaces(GraphGeometry &T, std::vector<Face> &invalidFaces) {
        Vertex2D p1 = T.vertices[0];
        Vertex2D p2 = T.vertices[1];
        Vertex2D p3 = T.vertices[2];
        T.getIncidentFaces(p1, invalidFaces);
        T.getIncidentFaces(p2, invalidFaces);
        T.getIncidentFaces(p3, invalidFaces);
    }

    void Delaunay::getCleanupInvalidEdges(GraphGeometry &T,
        std::vector<HalfEdge> &invalidEdges,
        std::vector<HalfEdge> &invalidTwins) {
        Vertex2D p1 = T.vertices[0];
        Vertex2D p2 = T.vertices[1];
        Vertex2D p3 = T.vertices[2];
        T.getIncidentEdges(p1, invalidEdges);
        T.getIncidentEdges(p2, invalidEdges);
        T.getIncidentEdges(p3, invalidEdges);

        invalidTwins.reserve(invalidEdges.size());
        for (unsigned int i = 0; i < invalidEdges.size(); i++) {
            invalidTwins.push_back(T.twin(invalidEdges[i]));
        }
    }

    void Delaunay::getCleanupUpdateVertices(GraphGeometry &T,
        std::vector<HalfEdge> &invalidEdges,
        std::vector<Vertex2D> &vertices) {
        Vertex2D p1 = T.vertices[0];
        Vertex2D p2 = T.vertices[1];
        Vertex2D p3 = T.vertices[2];

        std::vector<uint8_t> updateVertexTable(T.vertices.size(), false);
        for (unsigned int i = 0; i < invalidEdges.size(); i++) {
            Vertex2D v = T.origin(T.twin(invalidEdges[i]));
            if (v.id == p1.id || v.id == p2.id || v.id == p3.id) {
                continue;
            }
            updateVertexTable[v.id] = true;
        }

        for (unsigned int i = 0; i < updateVertexTable.size(); i++) {
            if (updateVertexTable[i]) {
                vertices.push_back(T.vertices[i]);
            }
        }
    }

    void Delaunay::updateCleanupVertices(GraphGeometry &T,
        std::vector<Vertex2D> &updateVertices,
        std::vector<uint8_t> &invalidEdgeTable,
        std::vector<uint8_t> &invalidFaceTable) {

        std::vector<int> incidentEdges;
        for (unsigned int i = 0; i < updateVertices.size(); i++) {
            Vertex2D v = updateVertices[i];

            incidentEdges.clear();
            T.getIncidentEdges(v, incidentEdges);

            HalfEdge h;
            for (unsigned int hidx = 0; hidx < incidentEdges.size(); hidx++) {
                h = T.getHalfEdge(incidentEdges[hidx]);
                int ref = h.id;
                if (!invalidEdgeTable[ref] && invalidFaceTable[h.incidentFace]) {
                    h.incidentFace = -1;
                    T.updateHalfEdge(h);
                }
            }

            if (invalidEdgeTable[v.incidentEdge]) {
                for (unsigned int hidx = 0; hidx < incidentEdges.size(); hidx++) {
                    h = T.getHalfEdge(incidentEdges[hidx]);
                    if (!invalidEdgeTable[h.id]) {
                        v.incidentEdge = h.id;
                        T.updateVertex(v);
                    }
                }
            }
        }
    }

    void Delaunay::removeInvalidCleanupComponents(GraphGeometry &T,
        std::vector<HalfEdge> &invalidEdges,
        std::vector<Face> &invalidFaces) {
        for (unsigned int i = 0; i < invalidEdges.size(); i++) {
            HalfEdge h = invalidEdges[i];
            HalfEdge twin = T.twin(h);

            int id = h.id;
            h = HalfEdge();
            h.id = id;
            T.updateHalfEdge(h);

            id = twin.id;
            twin = HalfEdge();
            twin.id = id;
            T.updateHalfEdge(twin);
        }

        for (unsigned int i = 0; i < invalidFaces.size(); i++) {
            Face f = invalidFaces[i];
            f.outerComponent = -1;
            T.updateFace(f);
        }

        Vertex2D p1 = T.vertices[0];
        Vertex2D p2 = T.vertices[1];
        Vertex2D p3 = T.vertices[2];
        p1.incidentEdge = -1;
        p2.incidentEdge = -1;
        p3.incidentEdge = -1;
        T.updateVertex(p1);
        T.updateVertex(p2);
        T.updateVertex(p3);
    }

    void Delaunay::cleanup(GraphGeometry &T) {

        std::vector<Face> invalidFaces;
        getCleanupInvalidFaces(T, invalidFaces);
        std::vector<uint8_t> invalidFaceTable(T.faces.size(), false);
        for (unsigned int i = 0; i < invalidFaces.size(); i++) {
            invalidFaceTable[invalidFaces[i].id] = true;
        }

        std::vector<HalfEdge> invalidEdges;
        std::vector<HalfEdge> invalidTwinEdges;
        getCleanupInvalidEdges(T, invalidEdges, invalidTwinEdges);

        std::vector<uint8_t> invalidEdgeTable(T.edges.size(), false);
        for (unsigned int i = 0; i < invalidEdges.size(); i++) {
            invalidEdgeTable[invalidEdges[i].id] = true;
        }
        for (unsigned int i = 0; i < invalidTwinEdges.size(); i++) {
            invalidEdgeTable[invalidTwinEdges[i].id] = true;
        }

        std::vector<Vertex2D> updateVertices;
        getCleanupUpdateVertices(T, invalidEdges, updateVertices);
        updateCleanupVertices(T, updateVertices, invalidEdgeTable, invalidFaceTable);

        removeInvalidCleanupComponents(T, invalidEdges, invalidFaces);
    }
}
