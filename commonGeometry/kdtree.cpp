#include "kdtree.h"

TreeSplit::TreeSplit(int nCuttingDim, float fCutValue, float fLowValue, float fHighValue, KdTreeNode* low, KdTreeNode* high) {
    m_nCutDim = nCuttingDim;
    m_fCutValue = fCutValue;
    m_fLow = fLowValue;
    m_fHigh = fHighValue;

    m_pLow = low;
    m_pHigh = high;
}

void TreeSplit::knnSearch(const Vector3 &vTarget, int k, std::vector<int>& vLabel, std::vector<float>& vDist, float fBoxDist) {
    float diff = vTarget[m_nCutDim] - m_fCutValue;
    if (diff < 0) {
        m_pLow->knnSearch(vTarget, k, vLabel, vDist, fBoxDist);
        float fBoundDiff = m_fLow - vTarget[m_nCutDim];
        if (fBoundDiff < 0) {
            fBoundDiff = 0;
        }
        fBoxDist += (diff * diff - fBoundDiff * fBoundDiff);
        if (fBoundDiff * MathFuncs::EPSILON < (vDist.size() == k ? vDist[k - 1] : MathFuncs::POS_INFINITY)) {
            m_pHigh->knnSearch(vTarget, k, vLabel, vDist, fBoxDist);
        }
    } else {
        m_pHigh->knnSearch(vTarget, k, vLabel, vDist, fBoxDist);
        float fBoundDiff = m_fHigh - vTarget[m_nCutDim];
        if (fBoundDiff < 0) {
            fBoundDiff = 0;
        }
        fBoxDist += (diff * diff - fBoundDiff * fBoundDiff);
        if (fBoundDiff * MathFuncs::EPSILON < (vDist.size() == k ? vDist[k - 1] : MathFuncs::POS_INFINITY)) {
            m_pLow->knnSearch(vTarget, k, vLabel, vDist, fBoxDist);
        }
    }
}

void TreeLeaf::knnSearch(const Vector3 &vTarget, unsigned int k, std::vector<int>& vLabel, std::vector<float>& vDist, float fBoxDist) {
    float fMinDist = (vDist.size() == k ? vDist[k - 1] : MathFuncs::POS_INFINITY);
    for (int i = m_nStPos, d; i <= m_nEndPos; i++) {
        float dist = 0;
        for (d = 0; d < 3; d++) {
            float t = (vTarget[d] - KdTree::getPointDimValue(m_vPoints, m_vBuckets, i, d));

            dist += t * t;
            if (dist > fMinDist) break;
        }

        if (d >= 3 && dist != 0) {
            if (vDist.size() < k) {
                vDist.push_back(dist);
                vLabel.push_back(m_vBuckets[i]);
            }
            int pos = int((vDist.size() < k ? vDist.size() : k) - 1);

            while (pos > 0 && vDist[pos - 1] > dist) {
                vDist[pos] = vDist[pos - 1];
                vLabel[pos] = vLabel[pos - 1];
                pos--;
            }
            vDist[pos] = dist;
            vLabel[pos] = m_vBuckets[i];

            fMinDist = dist;
        }
    }
}

KdTree::KdTree(const std::vector<Vector3>& points, int nNumOfPoints, int nBucketSize) {
    SkeletonTree(nNumOfPoints, nBucketSize);
    if (nNumOfPoints == 0) {
        return;
    }
    m_vPoints = points;
    calcBoundingBox(points, nNumOfPoints, m_stBoundingBox);
    Box stBoundingBox;
    stBoundingBox.vMin = m_stBoundingBox.vMin;
    stBoundingBox.vMax = m_stBoundingBox.vMax;
    m_pRoot = rKdTree(0, nNumOfPoints - 1, nBucketSize, stBoundingBox);
}

void KdTree::SkeletonTree(int nNumOfPoints, int nBucketSize) {
    m_nNumOfPoints = nNumOfPoints;
    m_nBucketSize = nBucketSize;
    for (int i = 0; i < nNumOfPoints; i++) {
        m_vLabels.push_back(i);
    }
    m_pRoot = nullptr;
}

KdTreeNode* KdTree::rKdTree(int nLabelStPos, int nLabelEndPos, int nBucketSize, Box& stBoundingBox) {
    int nNumOfPoints = (nLabelEndPos - nLabelStPos + 1);
    if (nNumOfPoints <= nBucketSize) {
        if (nNumOfPoints == 0) {
            return new TreeLeaf(0, 0, 0, m_vPoints, m_vLabels);;
        }
        return new TreeLeaf(nNumOfPoints, nLabelStPos, nLabelEndPos, m_vPoints, m_vLabels);
    }
    KdTreeNode *low, *high;
    int nLabelPos, nCuttingDim;
    float fCutValue, fLowValue, fHighValue;
    splitter(nLabelStPos, nLabelEndPos, nCuttingDim, fCutValue, nLabelPos, stBoundingBox);

    fLowValue = stBoundingBox.vMin[nCuttingDim];
    fHighValue = stBoundingBox.vMax[nCuttingDim];

    stBoundingBox.vMax[nCuttingDim] = fCutValue;
    low = rKdTree(nLabelStPos, nLabelPos, nBucketSize, stBoundingBox);
    stBoundingBox.vMax[nCuttingDim] = fHighValue;
    stBoundingBox.vMin[nCuttingDim] = fCutValue;
    high = rKdTree(nLabelPos + 1, nLabelEndPos, nBucketSize, stBoundingBox);
    stBoundingBox.vMin[nCuttingDim] = fLowValue;

    TreeSplit* pNode = new TreeSplit(nCuttingDim, fCutValue, fLowValue, fHighValue, low, high);
    return pNode;
}

void KdTree::calcBoundingBox(const std::vector<Vector3>& points, int nNumOfPoints, Box& stBoundingBox) {
    stBoundingBox.vMin = points[0];
    stBoundingBox.vMax = points[0];

    for (int i = 0; i < nNumOfPoints; i++) {
        for (int j = 0; j < 3; j++) {
            if (stBoundingBox.vMin[j] > points[i][j]) {
                stBoundingBox.vMin[j] = points[i][j];
            }
            else if (stBoundingBox.vMax[j] < points[i][j]) {
                stBoundingBox.vMax[j] = points[i][j];
            }
        }
    }
}

float KdTree::calcSpread(int nLabelStPos, int nLabelEndPos, int nCuttingDim) {
    float minn, maxn;
    calcMaxMin(nLabelStPos, nLabelEndPos, nCuttingDim, minn, maxn);
    return (maxn - minn);
}

void KdTree::calcMaxMin(int nLabelStPos, int nLabelEndPos, int nCuttingDim, float& minn, float& maxn) {
    minn = getPointDimValue(m_vPoints, m_vLabels, nLabelStPos, nCuttingDim);
    maxn = getPointDimValue(m_vPoints, m_vLabels, nLabelStPos, nCuttingDim);

    for (int i = nLabelStPos; i <= nLabelEndPos; i++) {
        if (minn > getPointDimValue(m_vPoints, m_vLabels, i, nCuttingDim)) {
            minn = getPointDimValue(m_vPoints, m_vLabels, i, nCuttingDim);
        }
        else if (maxn < getPointDimValue(m_vPoints, m_vLabels, i, nCuttingDim)) {
            maxn = getPointDimValue(m_vPoints, m_vLabels, i, nCuttingDim);
        }
    }
}

void KdTree::splitter(int nLabelStPos, int nLabelEndPos,
    int& nCuttingDim, float& fCutValue, int& nLabelPos, const Box& stBoundingBox) {
    float fMaxLength = stBoundingBox.vMax[0] - stBoundingBox.vMin[0];
    float fLength = std::max(stBoundingBox.vMax[1] - stBoundingBox.vMin[1], stBoundingBox.vMax[2] - stBoundingBox.vMin[2]);
    fMaxLength = std::max(fMaxLength, fLength);

    float maxSpread = -1;
    //calculate nCuttingDim
    for (int i = 0; i < 3; i++) {
        if ((stBoundingBox.vMax[i] - stBoundingBox.vMin[i]) < (1 - MathFuncs::LOWEPSILON) * fMaxLength) continue;
        float spread = calcSpread(nLabelStPos, nLabelEndPos, i);
        if (spread > maxSpread) {
            maxSpread = spread;
            nCuttingDim = i;
        }
    }
    //
    float minn, maxn;
    calcMaxMin(nLabelStPos, nLabelEndPos, nCuttingDim, minn, maxn);

    float fIdealCutValue = (stBoundingBox.vMax[nCuttingDim] + stBoundingBox.vMin[nCuttingDim]) / 2;
    if (fIdealCutValue < minn) fCutValue = minn;
    else if (fIdealCutValue > maxn) fCutValue = maxn;
    else fCutValue = fIdealCutValue;

    //
    int nBreakNum, nBreakNum2;
    planeSplit(nLabelStPos, nLabelEndPos, nCuttingDim, fCutValue, nBreakNum, nBreakNum2);

    nLabelPos = (nLabelEndPos + nLabelStPos) / 2;
    if (fIdealCutValue < minn) nLabelPos = nLabelStPos;
    else if (fIdealCutValue > maxn) nLabelPos = nLabelEndPos;
    else if (nBreakNum > nLabelPos) nLabelPos = nBreakNum;
    else if (nBreakNum2 < nLabelPos) nLabelPos = nBreakNum2;
}

void KdTree::swapLabel(int i, int j) {
    int tmp = m_vLabels[i];
    m_vLabels[i] = m_vLabels[j];
    m_vLabels[j] = tmp;
}

float KdTree::getPointDimValue(const std::vector<Vector3>& vPoints, const std::vector<int>& vLabels, unsigned int nIndex, unsigned int nTargetDim) {
    if (nIndex < 0 || nIndex >= vLabels.size() || nTargetDim < 0 || nTargetDim >= 3) return 0;
    return vPoints[vLabels[nIndex]][nTargetDim];
}

void KdTree::planeSplit(int nLabelStPos, int nLabelEndPos, int nCuttingDim, float fCutValue, int& nBreakNum, int& nBreakNum2) {
    int l = nLabelStPos, r = nLabelEndPos;
    while (l <= r) {
        while (l <= r && getPointDimValue(m_vPoints, m_vLabels, l, nCuttingDim) < fCutValue) l++;
        while (l <= r && getPointDimValue(m_vPoints, m_vLabels, r, nCuttingDim) >= fCutValue) r--;
        if (l > r) break;
        swapLabel(l, r);
        l++;
        r--;
    }
    nBreakNum = l;
    r = m_nNumOfPoints - 1;
    while (1 <= r) {
        while (l <= r && getPointDimValue(m_vPoints, m_vLabels, l, nCuttingDim) <= fCutValue) l++;
        while (l <= r && getPointDimValue(m_vPoints, m_vLabels, r, nCuttingDim) > fCutValue) r--;
        if (l > r) break;
        swapLabel(l, r);
        l++;
        r--;
    }
    nBreakNum2 = l;
}

void KdTree::knnSearch(Vector3 vTarget, int k, std::vector<int>& vLabel, std::vector<float>& vDist) {
    if (k > m_nNumOfPoints) {
        return;
    }
    m_pRoot->knnSearch(vTarget, k, vLabel, vDist, 0);
}

float KdTree::calcBoxDistance(const Vector3 &vTarget, const Vector3 &vLow, const Vector3 &vHigh) {
    float dist = 0;
    float t = 0;
    for (int i = 0; i < 3; i++) {
        if (vTarget[i] < vLow[i]) {
            t = vTarget[i] - vLow[i];
            dist += t * t;
        } else if (vTarget[i] > vHigh[i]) {
            t = vTarget[i] - vHigh[i];
            dist += t * t;
        }
    }
    return dist;
}
