#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include "vector3.h"
#include "box.h"
#include "math_funcs.h"

#define ERR 0.001
#define MAXERR ((1 + ERR) * Math::EPSILON)

class KdTreeNode {
public:
    KdTreeNode() {}
    virtual ~KdTreeNode() {}
    virtual void knnSearch(const Vector3 vTarget, int k, std::vector<int>& vLabel, std::vector<float>& vDist, float fBoxDist) {};
};

class TreeLeaf : public KdTreeNode {
public:
    TreeLeaf(int nNumOfPoints, int nStPos, int nEndPos, std::vector<Vector3>& vPoints, std::vector<int>& vBuckets) {
        m_nNumOfPoints = nNumOfPoints;
        m_nStPos = nStPos;
        m_nEndPos = nEndPos;
        m_vPoints = vPoints;
        m_vBuckets = vBuckets;
    };
    virtual ~TreeLeaf() {}
    virtual void knnSearch(const Vector3 &vTarget, unsigned int k, std::vector<int>& vLabel, std::vector<float>& vDist, float fBoxDist);

private:
    int m_nNumOfPoints;
    std::vector<int> m_vBuckets;
    std::vector<Vector3> m_vPoints;
    int m_nStPos, m_nEndPos;
};

class TreeSplit : public KdTreeNode {
public:
    TreeSplit(int nCuttingDim, float fCutValue, float fLowValue, float fHighValue, KdTreeNode* low = nullptr, KdTreeNode* high = nullptr);
    virtual ~TreeSplit() {}
    void knnSearch(const Vector3 &vTarget, int k, std::vector<int>& vLabel, std::vector<float>& vDist, float fBoxDist);

private:
    int m_nCutDim;
    float m_fCutValue;
    float m_fLow, m_fHigh;
    KdTreeNode *m_pLow, *m_pHigh;
};

class KdTree
{
public:
    KdTree(const std::vector<Vector3>& points, int nNumOfPoints, int nBucketSize = 1);


    static void calcBoundingBox(const std::vector<Vector3>& points, int nNumOfPoints, Box& stBoundingBox);
    void knnSearch(Vector3 vTarget, int k, std::vector<int>& vLabel, std::vector<float>& vDist);

    static float calcBoxDistance(const Vector3 &vTarget, const Vector3 &vLow, const Vector3 &vHigh);
    //get value of m_vPoints[m_vLabels[nIndex]][nTargetDim]
    static float getPointDimValue(const std::vector<Vector3>& vPoints, const std::vector<int>& vLabels, unsigned int nIndex, unsigned int nTargetDim);
private:
    void SkeletonTree(int nNumOfPoints, int nBucketSize);
    KdTreeNode* rKdTree(int nLabelStPos, int nLabelEndPos, int nBucketSize, Box& stBoundingBox);

    //get min and max value of m_vPoints[m_vLabels[st...end]] in cutting dim
    void calcMaxMin(int nLabelStPos, int nLabelEndPos, int nCuttingDim, float& minn, float& maxn);
    //calc max - min of above
    float calcSpread(int nLabelStPos, int nLabelEndPos, int nCuttingDim);
    void splitter(int nLabelStPos, int nLabelEndPos,
        int& nCuttingDim, float& fCutValue, int& nNumOfLow, const Box& stBoundingBox);
    void planeSplit(int nLabelStPos, int nLabelEndPos, int nCuttingDim, float fCutValue, int& nBreakNum, int& nBreakNum2);

    void swapLabel(int i, int j);

    int m_nNumOfPoints;
    int m_nBucketSize;

    Box m_stBoundingBox;
    KdTreeNode* m_pRoot;
    std::vector<int> m_vLabels;
    std::vector<Vector3> m_vPoints;
};

#endif
