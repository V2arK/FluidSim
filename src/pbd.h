#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
#include <unordered_map>

/* ----------------- Physical Constants -------------------- */

const float PARTICLE_RADIUS = 0.005f;                   // Radius of a single particle
const float SUPPORT_RADIUS = 5.0f * PARTICLE_RADIUS;    // Support radius for smoothing kernels
const float PARTICLE_DIAMETER = PARTICLE_RADIUS * 2.0f; // Diameter of a particle
const float GRAVITY = 9.8f;                             // Gravitational acceleration
const glm::vec3 GRAVITY_DIR = glm::vec3(0.0, -1.0, 0.0);
const float REFERENCE_DENSITY = 1000.0f;   // Reference density
const float STIFFNESS_CONSTANT = 50.0f;    // Stiffness constant for pressure calculation
const float PRESSURE_EXPONENT = 7.0f;      // Exponent for the equation of state
const float VISCOSITY_COEFFICIENT = 8e-6f; // Viscosity coefficient
const float DELTA_T = 0.003f;              // delta for euler intergration
const float MAX_VELOCITY = 100.0f;         // max velocity of particle
const float VELOCITY_ATTENUATION = 0.9f;   // for when bouncing off the border

namespace Para
{
    const float gravity = 9.8f;
    const float density0 = 1000.0f;
    const float stiffness = 50.0f;
    const float exponent = 7.0f;
    const float viscosity = 0.05f;
    const float dt = 2e-4;
    const int substep = 10;
}
/* ----------------- Random Generator -------------------- */

#include <stdint.h>
namespace PCG32
{
    static uint64_t state = 0xcafef00dd15ea5e5u; // Must be odd
    static uint64_t const multiplier = 6364136223846793005u;

    // Generate a pseudo-random 32-bit integer
    uint32_t generate()
    {
        uint64_t x = state;
        const unsigned count = (unsigned)(x >> 61);
        state = x * multiplier;
        x ^= x >> 22;
        return (uint32_t)(x >> (22 + count));
    }

    // Generate a pseudo-random float between 0 and 1
    float randomFloat()
    {
        return float(double(generate()) / 4294967296.0);
    }
}

inline unsigned int computeHash(unsigned int x, unsigned int y, unsigned int z)
{
    // these magic number is just 3 large primes,
    // so we can map the 3d coordinate into a hash.
    const unsigned int p1 = 73856093;
    const unsigned int p2 = 19349663;
    const unsigned int p3 = 83492791;
    return (x * p1) ^ (y * p2) ^ (z * p3);
}

/* ----------------- Cubic Spline Kernel -------------------- */

class WCubicSpline2d
{
public:
    WCubicSpline2d() = delete;

    explicit WCubicSpline2d(float h)
    {
        mH = h;
        mH2 = h * h;
        mSigma = 40.0 / (7.0 * glm::pi<float>() * mH2);

        mBufferSize = glm::uvec2(128, 128);
        mGradBuffer = std::vector<std::vector<glm::vec2>>(mBufferSize.x, std::vector<glm::vec2>(mBufferSize.y));
        mValueBuffer = std::vector<float>(mBufferSize.x);

        for (int i = 0; i < mBufferSize.x; i++)
        {
            for (int j = 0; j < mBufferSize.y; j++)
            {
                float x = ((float)i + 0.5f) * mH / mBufferSize.x;
                float y = ((float)j + 0.5f) * mH / mBufferSize.y;
                glm::vec2 radius(x, y);
                mGradBuffer[i][j] = CalculateGrad(radius);
            }
        }

        for (int i = 0; i < mBufferSize.x; i++)
        {
            float distance = ((float)i + 0.5f) * mH / mBufferSize.x;
            mValueBuffer[i] = CalculateValue(distance);
        }
    }
    ~WCubicSpline2d()
    {
    }

    float Value(float distance)
    {
        float res = 0;
        int i = (std::abs(distance) * mBufferSize.x / mH);
        if (i >= mBufferSize.x)
        {
            return res;
        }
        res = mValueBuffer[i];
        return res;
    }

    glm::vec2 Grad(glm::vec2 radius) {
        glm::vec2 res(0.0f, 0.0f);

        int i = (std::abs(radius.x) * mBufferSize.x / mH);
        int j = (std::abs(radius.y) * mBufferSize.x / mH);

        if (i >= mBufferSize.x || j >= mBufferSize.y) {
            return res;
        }

        res = mGradBuffer[i][j];

        if (radius.x < 0) {
            res.x = -res.x;
        }
        if (radius.y < 0) {
            res.y = -res.y;
        }

        return res;
    }

private:
    float CalculateValue(float distance) {
        float r = std::abs(distance);
        float q = r / mH;
        float q2 = q * q;
        float q3 = q * q2;
        float res = 0.0f;
        if (q < 0.5f) {
            res = 6.0f * (q3 - q2) + 1.0f;
            res *= mSigma;
            return res;
        }
        else if (q >= 0.5f && q < 1.0f) {
            res = 1.0f - q;
            res = std::pow(res, 3) * 2.0f;
            res *= mSigma;
            return res;
        }
        return res;
    }

    glm::vec2 CalculateGrad(glm::vec2 radius) {
        glm::vec2 res(0.0f, 0.0f);
        float distance = glm::length(radius);
        if (distance < 1e-5) {
            return res;
        }

        float q = distance / mH;
        glm::vec2 qGrad = radius / (mH * distance);

        if (q < 0.5f) {
            res = 6.0f * (3.0f * q * q - 2.0f * q) * mSigma * qGrad;
            return res;
        }
        else if (q >= 0.5 && q < 1.0f) {
            res = -6.0f * std::powf(1.0f - q, 2) * mSigma * qGrad;
            return res;
        }
        return res;
    }

private:
    float mH;
    float mH2;
    float mSigma;
    glm::uvec2 mBufferSize;
    std::vector<std::vector<glm::vec2>> mGradBuffer;
    std::vector<float> mValueBuffer;
};

/* ----------------- Particle System -------------------- */

struct NeighborInfo
{
    int index;
    float distance;
    float distance2;
    glm::vec2 radius;
};

class ParticalSystem
{
public:
    ParticalSystem(){

    }
    
    ~ParticalSystem(){

    }

    void SetContainerSize(glm::vec2 corner, glm::vec2 size)
    {
        mLowerBound = corner;
        mUpperBound = corner + size;
        mContainerCenter = corner + 0.5f * size;

        mBlockRowNum = floor(size.y / mSupportRadius);
        mBlockColNum = floor(size.x / mSupportRadius);
        mBlockSize = glm::vec2(size.x / mBlockColNum, size.y / mBlockRowNum);

        mPositions.clear();
        mVelocity.clear();
        mAccleration.clear();

        mStartIndex = 0;

        // mStartIndex += AddBoundary(mLowerBound, glm::vec2(size.x, mSupportRadius));
        // mStartIndex += AddBoundary(glm::vec2(mLowerBound.x, mUpperBound.y - mSupportRadius), glm::vec2(size.x, mSupportRadius));
        // mStartIndex += AddBoundary(glm::vec2(mLowerBound.x, mLowerBound.y + mSupportRadius), glm::vec2(mSupportRadius, size.y - 2.0f * mSupportRadius));
        // mStartIndex += AddBoundary(glm::vec2(mUpperBound.x - mSupportRadius, mLowerBound.y + mSupportRadius), glm::vec2(mSupportRadius, size.y - 2.0f * mSupportRadius));
    }

    int32_t AddFluidBlock(glm::vec2 corner, glm::vec2 size, glm::vec2 v0, float particalSpace)
    {
        glm::vec2 blockLowerBound = corner;
        glm::vec2 blockUpperBound = corner + size;

        if (blockLowerBound.x < mLowerBound.x ||
            blockLowerBound.y < mLowerBound.y ||
            blockUpperBound.x > mUpperBound.x ||
            blockUpperBound.y > mUpperBound.y)
        {
            return -1;
        }

        int width = size.x / particalSpace;
        int height = size.y / particalSpace;

        std::vector<glm::vec2> position(width * height);
        std::vector<glm::vec2> velocity(width * height, v0);
        std::vector<glm::vec2> accleration(width * height, glm::vec2(0.0f, 0.0f));

        int p = 0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                position[p] = corner + glm::vec2((j + 0.5) * particalSpace, (i + 0.5) * particalSpace);
                if (i % 2)
                {
                    position[p].x += mParticalRadius;
                }
                p++;
            }
        }

        mPositions.insert(mPositions.end(), position.begin(), position.end());
        mVelocity.insert(mVelocity.end(), velocity.begin(), velocity.end());
        mAccleration.insert(mAccleration.end(), accleration.begin(), accleration.end());
        return position.size();
    }

    void SearchNeighbors()
    {
        BuildBlockStructure();

        mNeighbors = std::vector<std::vector<NeighborInfo>>(mPositions.size(), std::vector<NeighborInfo>(0));

        for (int i = mStartIndex; i < mPositions.size(); i++)
        { // ���������Ӳ����ھ�
            glm::vec2 deltePos = mPositions[i] - mLowerBound;
            uint32_t bc = floor(deltePos.x / mBlockSize.x);
            uint32_t br = floor(deltePos.y / mBlockSize.y);

            int bIdi = GetBlockIdByPosition(mPositions[i]);

            // ����������Block
            for (int dr = -1; dr <= 1; dr++)
            {
                for (int dc = -1; dc <= 1; dc++)
                {
                    if (bc + dc < 0 || bc + dc >= mBlockColNum ||
                        br + dr < 0 || br + dr >= mBlockRowNum)
                    {
                        continue;
                    }

                    int bIdj = bIdi + dr * mBlockColNum + dc;
                    std::vector<int> &block = mBlocks[bIdj];
                    for (int j : block)
                    { // ��Block����������
                        if (i == j)
                        {
                            continue;
                        }
                        NeighborInfo nInfo{};
                        nInfo.radius = mPositions[i] - mPositions[j];
                        nInfo.distance = glm::length(nInfo.radius);
                        nInfo.distance2 = nInfo.distance * nInfo.distance;
                        nInfo.index = j;
                        if (nInfo.distance <= mSupportRadius)
                        {
                            mNeighbors[i].push_back(nInfo);
                        }
                    }
                }
            }
        }
    }

    size_t GetBlockIdByPosition(glm::vec2 position)
    {
        if (position.x < mLowerBound.x ||
            position.y < mLowerBound.y ||
            position.x > mUpperBound.x ||
            position.y > mUpperBound.y)
        {
            return -1;
        }

        glm::vec2 deltePos = position - mLowerBound;
        uint32_t c = floor(deltePos.x / mBlockSize.x);
        uint32_t r = floor(deltePos.y / mBlockSize.y);
        return r * mBlockColNum + c;
    }

    void BuildBlockStructure()
    {
        mBlocks = std::vector<std::vector<int>>(mBlockColNum * mBlockRowNum, std::vector<int>(0));

        for (int i = 0; i < mPositions.size(); i++)
        { // �������ӷ����Լ��ļ�
            int bId = GetBlockIdByPosition(mPositions[i]);
            mBlocks[bId].push_back(i);
        }
    }

private:
    int32_t AddBoundary(glm::vec2 corner, glm::vec2 size)
    {
        float space = mParticalRadius / 4.0f;
        int rows = floor(size.y / space);
        int cols = floor(size.x / space);

        float rowOffset = (size.y - ((float)rows - 1.0f) * space) / 2.0f;
        float colOffset = (size.x - ((float)cols - 1.0f) * space) / 2.0f;

        std::vector<glm::vec2> position(rows * cols);
        std::vector<glm::vec2> velocity(rows * cols, glm::vec2(0.0f, 0.0f));
        std::vector<glm::vec2> accleration(rows * cols, glm::vec2(0.0f, 0.0f));

        int p = 0;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                float x = colOffset + j * space;
                float y = rowOffset + i * space;
                position[p] = corner + glm::vec2(x, y);
                p++;
            }
        }

        mPositions.insert(mPositions.end(), position.begin(), position.end());
        mVelocity.insert(mVelocity.end(), velocity.begin(), velocity.end());
        mAccleration.insert(mAccleration.end(), accleration.begin(), accleration.end());
        return position.size();
    }

public:
    // 粒子参数
    float mSupportRadius = 0.025; // 支撑半径
    float mSupportRadius2 = mSupportRadius * mSupportRadius;
    float mParticalRadius = 0.005; // 粒子半径
    float mParticalDiameter = 2.0 * mParticalRadius;
    float mVolume = 0.8 * mParticalDiameter * mParticalDiameter; // 体积
    float mMass = Para::density0 * mVolume;                      // 质量
    float mViscosity = 0.01;                                     // 粘度系数
    float mExponent = 7.0f;                                      // 压力指数
    int mStiffness = 50.0f;                                      // 刚度

    int mStartIndex = 0;
    std::vector<glm::vec2> mPositions;
    std::vector<glm::vec2> mAccleration;
    std::vector<glm::vec2> mVelocity;
    std::vector<float> mDensity;
    std::vector<float> mPressure;
    std::vector<std::vector<NeighborInfo>> mNeighbors;

    // 容器参数
    glm::vec2 mLowerBound = glm::vec2(-1.0f, -1.0f);
    glm::vec2 mUpperBound = glm::vec2(1.0f, 1.0f);
    glm::vec2 mContainerCenter = glm::vec2(0.0f, 0.0f);
    std::vector<std::vector<int>> mBlocks;
    glm::vec2 mBlockSize = glm::vec2(0.5, 0.5);
    uint32_t mBlockRowNum = 4;
    uint32_t mBlockColNum = 4;
};

class Solver
{
public:
    explicit Solver(ParticalSystem& ps) : mPs(ps), mW(ps.mSupportRadius)
    {

    }

    ~Solver(){

    }

    void Iterate() {
        UpdateDensityAndPressure();
        InitAccleration();
        UpdateViscosityAccleration();
        UpdatePressureAccleration();
        EulerIntegrate();
        BoundaryCondition();
        //std::cout << "solve time = " << timer.GetTime() << std::endl;
    }

private:
    void UpdateDensityAndPressure() {
        mPs.mDensity = std::vector<float>(mPs.mPositions.size(), Para::density0);
        mPs.mPressure = std::vector<float>(mPs.mPositions.size(), 0.0f);
        for (int i = 0; i < mPs.mPositions.size(); i++) {    // 对所有粒子
            if (mPs.mNeighbors.size() != 0) {    // 有邻居
                std::vector<NeighborInfo>& neighbors = mPs.mNeighbors[i];
                float density = 0;
                for (auto& nInfo : neighbors) {
                    density += mW.Value(nInfo.distance);
                }
                density *= (mPs.mVolume * Para::density0);
                mPs.mDensity[i] = density;
                mPs.mDensity[i] = std::max(density, Para::density0);        // 禁止膨胀
            }
            // 更新压强
            mPs.mPressure[i] = mPs.mStiffness* (std::powf(mPs.mDensity[i] / Para::density0, mPs.mExponent) - 1.0f);
        }
    }

    void InitAccleration() {
        std::fill(mPs.mAccleration.begin() + mPs.mStartIndex, mPs.mAccleration.end(), glm::vec2(0.0f, -Para::gravity));
    }

    void UpdateViscosityAccleration() {
        float dim = 2.0f;
        float constFactor = 2.0f * (dim + 2.0f) * mPs.mViscosity;
        for (int i = mPs.mStartIndex; i < mPs.mPositions.size(); i++) {    // 对所有粒子
            if (mPs.mNeighbors.size() != 0) {    // 有邻居
                std::vector<NeighborInfo>& neighbors = mPs.mNeighbors[i];
                glm::vec2 viscosityForce(0.0f, 0.0f);
                for (auto& nInfo : neighbors) {
                    int j = nInfo.index;
                    float dotDvToRad = glm::dot(mPs.mVelocity[i] - mPs.mVelocity[j], nInfo.radius);

                    float denom = nInfo.distance2 + 0.01f * mPs.mSupportRadius2;
                    viscosityForce += (mPs.mMass / mPs.mDensity[j]) * dotDvToRad * mW.Grad(nInfo.radius) / denom;
                }
                viscosityForce *= constFactor;
                mPs.mAccleration[i] += viscosityForce;
            }
        }
    }
    void UpdatePressureAccleration() {
        std::vector<float> pressDivDens2(mPs.mPositions.size(), 0);        // p/(dens^2)
        for (int i = 0; i < mPs.mPositions.size(); i++) {
            pressDivDens2[i] = mPs.mPressure[i] / std::powf(mPs.mDensity[i], 2);
        }

        for (int i = mPs.mStartIndex; i < mPs.mPositions.size(); i++) {    // 对所有粒子
            if (mPs.mNeighbors.size() != 0) {    // 有邻居
                std::vector<NeighborInfo>& neighbors = mPs.mNeighbors[i];
                glm::vec2 pressureForce(0.0f, 0.0f);
                for (auto& nInfo : neighbors) {
                    int j = nInfo.index;
                    pressureForce += mPs.mDensity[j] * (pressDivDens2[i] + pressDivDens2[j]) * mW.Grad(nInfo.radius);
                }
                mPs.mAccleration[i] -= pressureForce * mPs.mVolume;
            }
        }
    }

    void EulerIntegrate() {
        for (int i = mPs.mStartIndex; i < mPs.mPositions.size(); i++) {    // 对所有粒子
            mPs.mVelocity[i] += Para::dt * mPs.mAccleration[i];
            mPs.mVelocity[i] = glm::clamp(mPs.mVelocity[i], glm::vec2(-100.0f), glm::vec2(100.0f));
            mPs.mPositions[i] += Para::dt * mPs.mVelocity[i];
        }
    }

    void BoundaryCondition() {
        for (int i = 0; i < mPs.mPositions.size(); i++) {    // 对所有粒子
            glm::vec2& position = mPs.mPositions[i];
            bool invFlag = false;
            if (position.y < mPs.mLowerBound.y + mPs.mSupportRadius) {
                mPs.mVelocity[i].y = std::abs(mPs.mVelocity[i].y);
                invFlag = true;
            }

            if (position.y > mPs.mUpperBound.y - mPs.mSupportRadius) {
                mPs.mVelocity[i].y = -std::abs(mPs.mVelocity[i].y);
                invFlag = true;
            }

            if (position.x < mPs.mLowerBound.x + mPs.mSupportRadius) {
                mPs.mVelocity[i].x = std::abs(mPs.mVelocity[i].x);
                invFlag = true;
            }

            if (position.x > mPs.mUpperBound.x - mPs.mSupportRadius) {
                mPs.mVelocity[i].x = -std::abs(mPs.mVelocity[i].x);
                invFlag = true;
            }

            if (invFlag) {
                mPs.mPositions[i] += Para::dt * mPs.mVelocity[i];
                mPs.mVelocity[i] = glm::clamp(mPs.mVelocity[i], glm::vec2(-100.0f), glm::vec2(100.0f));    // 速度限制
            }
        }
    }

private:
    ParticalSystem &mPs;
    WCubicSpline2d mW;
};

#endif // !PARTICLE_SYSTEM_H
