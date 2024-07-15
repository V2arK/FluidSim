#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <stdint.h>

/* ----------------- Physical Constants -------------------- */

// Define physical constants used in the simulation
const float REFERENCE_DENSITY = 1000.0f;   // Reference density for fluid
const float STIFFNESS_CONSTANT = 50.0f;    // Stiffness constant for pressure calculation
const float PRESSURE_EXPONENT = 7.0f;      // Exponent used in the equation of state for pressure
const float VISCOSITY_COEFFICIENT = 0.01f; // Viscosity coefficient for fluid
const float DELTA_T = 0.0005f;             // Time step for Euler integration
const float MAX_VELOCITY = 100.0f;         // Maximum allowable particle velocity
const float VELOCITY_ATTENUATION = 0.9f;   // Velocity attenuation factor for particle collisions

const float GRAVITY = 9.8f;                         // Gravitational acceleration
const glm::vec2 GRAVITY_DIR = glm::vec2(0.0, -1.0); // Direction of gravity

// Particle and support radius definitions
const float PARTICLE_RADIUS = 0.005f;                                 // Radius of each particle
const float SUPPORT_RADIUS = 5.0f * PARTICLE_RADIUS;                  // Support radius for smoothing kernels
const float SUPPORT_RADIUS2 = SUPPORT_RADIUS * SUPPORT_RADIUS;        // Square of the support radius
const float PARTICLE_DIAMETER = PARTICLE_RADIUS * 2.0f;               // Diameter of a particle
const float PARTICLE_VOLUME = 0.8f * std::powf(PARTICLE_DIAMETER, 2); // Volume of a particle
const float PARTICLE_MASS = REFERENCE_DENSITY * PARTICLE_VOLUME;      // Mass of a particle

/* ----------------- Random Generator -------------------- */

namespace PCG32
{
    static uint64_t state = 0xcafef00dd15ea5e5u;             // Initial state for the random generator
    static uint64_t const multiplier = 6364136223846793005u; // Multiplier for random generation

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

// Compute a hash value from 3D coordinates
inline unsigned int computeHash(unsigned int x = 0, unsigned int y = 0, unsigned int z = 0)
{
    // These magic numbers are just 3 large primes to map the 3D coordinate into a hash.
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
        : mH(h), mH2(h * h), mSigma(40.0 / (7.0 * glm::pi<float>() * mH2)),
          mBufferSize(128, 128),
          mGradBuffer(mBufferSize.x, std::vector<glm::vec2>(mBufferSize.y)),
          mValueBuffer(mBufferSize.x)
    {
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

    ~WCubicSpline2d() = default;

    float Value(float distance) const
    {
        float res = 0;
        int i = static_cast<int>(std::abs(distance) * mBufferSize.x / mH);
        if (i < mBufferSize.x)
        {
            res = mValueBuffer[i];
        }
        return res;
    }

    glm::vec2 Grad(glm::vec2 radius) const
    {
        glm::vec2 res(0.0f, 0.0f);

        int i = static_cast<int>(std::abs(radius.x) * mBufferSize.x / mH);
        int j = static_cast<int>(std::abs(radius.y) * mBufferSize.y / mH);

        if (i < mBufferSize.x && j < mBufferSize.y)
        {
            res = mGradBuffer[i][j];

            if (radius.x < 0)
                res.x = -res.x;
            if (radius.y < 0)
                res.y = -res.y;
        }

        return res;
    }

private:
    float CalculateValue(float distance) const
    {
        float r = std::abs(distance);
        float q = r / mH;
        float q2 = q * q;
        float q3 = q * q2;
        float res = 0.0f;

        if (q < 0.5f)
        {
            res = 6.0f * (q3 - q2) + 1.0f;
        }
        else if (q < 1.0f)
        {
            res = 2.0f * std::powf(1.0f - q, 3);
        }

        return res * mSigma;
    }

    glm::vec2 CalculateGrad(glm::vec2 radius) const
    {
        glm::vec2 res(0.0f, 0.0f);
        float distance = glm::length(radius);
        if (distance < 1e-5)
            return res;

        float q = distance / mH;
        glm::vec2 qGrad = radius / (mH * distance);

        if (q < 0.5f)
        {
            res = 6.0f * (3.0f * q * q - 2.0f * q) * mSigma * qGrad;
        }
        else if (q < 1.0f)
        {
            res = -6.0f * std::powf(1.0f - q, 2) * mSigma * qGrad;
        }

        return res;
    }

    float mH;                                        // Smoothing radius
    float mH2;                                       // Square of the smoothing radius
    float mSigma;                                    // Normalization constant for the kernel
    glm::uvec2 mBufferSize;                          // Buffer size for precomputed values
    std::vector<std::vector<glm::vec2>> mGradBuffer; // Gradient buffer for precomputed values
    std::vector<float> mValueBuffer;                 // Value buffer for precomputed values
};

/* ----------------- Particle System -------------------- */

struct NeighborInfo
{
    int index;        // Index of the neighboring particle
    float distance;   // Distance to the neighboring particle
    float distance2;  // Square of the distance to the neighboring particle
    glm::vec2 radius; // Vector pointing to the neighboring particle
};

class ParticleSystem
{
public:
    ParticleSystem() = default;
    ~ParticleSystem() = default;

    // Set the size of the container where particles are confined
    void SetContainerSize(glm::vec2 corner, glm::vec2 size)
    {
        mLowerBound = corner;
        mUpperBound = corner + size;
        mContainerCenter = corner + 0.5f * size;

        mBlockRowNum = static_cast<uint32_t>(floor(size.y / SUPPORT_RADIUS));
        mBlockColNum = static_cast<uint32_t>(floor(size.x / SUPPORT_RADIUS));
        mBlockSize = glm::vec2(size.x / mBlockColNum, size.y / mBlockRowNum);

        mPositions.clear();
        mVelocity.clear();
        mAcceleration.clear();

        mStartIndex = 0;
    }

    // Add a block of fluid particles to the system
    unsigned int AddFluidBlock(glm::vec2 corner, glm::vec2 size, glm::vec2 v0, float particleSpace)
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

        int width = static_cast<int>(size.x / particleSpace);
        int height = static_cast<int>(size.y / particleSpace);

        std::vector<glm::vec2> position(width * height);
        std::vector<glm::vec2> velocity(width * height, v0);
        std::vector<glm::vec2> acceleration(width * height, glm::vec2(0.0f, 0.0f));

        int p = 0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                position[p] = corner + glm::vec2((j + PCG32::randomFloat()) * particleSpace, (i + PCG32::randomFloat()) * particleSpace);
                p++;
            }
        }

        mPositions.insert(mPositions.end(), position.begin(), position.end());
        mVelocity.insert(mVelocity.end(), velocity.begin(), velocity.end());
        mAcceleration.insert(mAcceleration.end(), acceleration.begin(), acceleration.end());
        return position.size();
    }

    // Search for neighboring particles within the support radius
    void SearchNeighbors()
    {
        BuildBlockStructure();

        mNeighbors = std::vector<std::vector<NeighborInfo>>(mPositions.size(), std::vector<NeighborInfo>(0));

        for (unsigned int i = mStartIndex; i < mPositions.size(); i++)
        {
            glm::vec2 deltaPos = mPositions[i] - mLowerBound;
            uint32_t bc = static_cast<uint32_t>(floor(deltaPos.x / mBlockSize.x));
            uint32_t br = static_cast<uint32_t>(floor(deltaPos.y / mBlockSize.y));

            int blockId = GetBlockIdByPosition(mPositions[i]);

            for (int dr = -1; dr <= 1; dr++)
            {
                for (int dc = -1; dc <= 1; dc++)
                {
                    if (bc + dc < 0 || bc + dc >= mBlockColNum ||
                        br + dr < 0 || br + dr >= mBlockRowNum)
                    {
                        continue;
                    }

                    int neighborBlockId = blockId + dr * mBlockColNum + dc;
                    auto &block = mBlocks[neighborBlockId];
                    for (int j : block)
                    {
                        if (i == j)
                            continue;

                        NeighborInfo nInfo{};
                        nInfo.radius = mPositions[i] - mPositions[j];
                        nInfo.distance = glm::length(nInfo.radius);
                        nInfo.distance2 = nInfo.distance * nInfo.distance;
                        nInfo.index = j;
                        if (nInfo.distance <= SUPPORT_RADIUS)
                        {
                            mNeighbors[i].push_back(nInfo);
                        }
                    }
                }
            }
        }
    }

    // Get block ID based on particle position
    unsigned int GetBlockIdByPosition(glm::vec2 position) const
    {
        if (position.x < mLowerBound.x ||
            position.y < mLowerBound.y ||
            position.x > mUpperBound.x ||
            position.y > mUpperBound.y)
        {
            return static_cast<unsigned int>(-1);
        }

        glm::vec2 deltaPos = position - mLowerBound;
        uint32_t x = static_cast<uint32_t>(floor(deltaPos.x / mBlockSize.x));
        uint32_t y = static_cast<uint32_t>(floor(deltaPos.y / mBlockSize.y));
        return y * mBlockColNum + x;
    }

    // Build the block structure for spatial partitioning
    void BuildBlockStructure()
    {
        mBlocks = std::vector<std::vector<unsigned int>>(mBlockColNum * mBlockRowNum, std::vector<unsigned int>(0));

        for (unsigned int i = 0; i < mPositions.size(); i++)
        {
            int blockId = GetBlockIdByPosition(mPositions[i]);
            mBlocks[blockId].push_back(static_cast<int>(i));
        }
    }

private:
    unsigned int AddBoundary(glm::vec2 corner, glm::vec2 size)
    {
        float space = PARTICLE_RADIUS / 4.0f;
        int rows = static_cast<int>(floor(size.y / space));
        int cols = static_cast<int>(floor(size.x / space));

        float rowOffset = (size.y - ((float)rows - 1.0f) * space) / 2.0f;
        float colOffset = (size.x - ((float)cols - 1.0f) * space) / 2.0f;

        std::vector<glm::vec2> position(rows * cols);
        std::vector<glm::vec2> velocity(rows * cols, glm::vec2(0.0f, 0.0f));
        std::vector<glm::vec2> acceleration(rows * cols, glm::vec2(0.0f, 0.0f));

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
        mAcceleration.insert(mAcceleration.end(), acceleration.begin(), acceleration.end());
        return position.size();
    }

public:
    int mStartIndex = 0;                               // Starting index for particles
    std::vector<glm::vec2> mPositions;                 // Positions of particles
    std::vector<glm::vec2> mAcceleration;              // Accelerations of particles
    std::vector<glm::vec2> mVelocity;                  // Velocities of particles
    std::vector<float> mDensity;                       // Densities of particles
    std::vector<float> mPressure;                      // Pressures of particles
    std::vector<std::vector<NeighborInfo>> mNeighbors; // Neighbor information for particles

    // Container parameters
    glm::vec2 mLowerBound = glm::vec2(-1.0f, -1.0f);    // Lower bound of the container
    glm::vec2 mUpperBound = glm::vec2(1.0f, 1.0f);      // Upper bound of the container
    glm::vec2 mContainerCenter = glm::vec2(0.0f, 0.0f); // Center of the container
    std::vector<std::vector<unsigned int>> mBlocks;     // Blocks for spatial partitioning
    glm::vec2 mBlockSize = glm::vec2(0.5f, 0.5f);       // Size of each block
    uint32_t mBlockRowNum = 4;                          // Number of block rows
    uint32_t mBlockColNum = 4;                          // Number of block columns
};

class Solver
{
public:
    explicit Solver(ParticleSystem &ps)
        : mPs(ps), mW(SUPPORT_RADIUS) {}

    ~Solver() = default;

    // Perform solver iteration to update particle states
    void Iterate()
    {
        UpdateDensityAndPressure();    // Update densities and pressures of particles
        InitAcceleration();            // Initialize accelerations of particles
        UpdateViscosityAcceleration(); // Update accelerations due to viscosity
        UpdatePressureAcceleration();  // Update accelerations due to pressure
        EulerIntegrate();              // Integrate positions and velocities using Euler method
        BoundaryCondition();           // Apply boundary conditions to particles
    }

private:
    // Update density and pressure for all particles
    void UpdateDensityAndPressure()
    {
        mPs.mDensity.assign(mPs.mPositions.size(), REFERENCE_DENSITY); // Initialize densities to reference value
        mPs.mPressure.assign(mPs.mPositions.size(), 0.0f);             // Initialize pressures to zero

        for (unsigned int i = 0; i < mPs.mPositions.size(); i++)
        {
            if (!mPs.mNeighbors.empty())
            {
                float density = 0;
                for (const auto &nInfo : mPs.mNeighbors[i])
                {
                    density += mW.Value(nInfo.distance); // Sum contributions from neighboring particles
                }
                density *= (PARTICLE_VOLUME * REFERENCE_DENSITY);       // Scale density by particle volume and reference density
                mPs.mDensity[i] = std::max(density, REFERENCE_DENSITY); // Prevent expansion
            }

            // Update pressure using the equation of state
            mPs.mPressure[i] = STIFFNESS_CONSTANT * (std::powf(mPs.mDensity[i] / REFERENCE_DENSITY, PRESSURE_EXPONENT) - 1.0f);
        }
    }

    // Initialize acceleration for all particles
    void InitAcceleration()
    {
        std::fill(mPs.mAcceleration.begin() + mPs.mStartIndex, mPs.mAcceleration.end(), glm::vec2(0.0f, -GRAVITY)); // Initialize accelerations to gravity
    }

    // Update viscosity acceleration for all particles
    void UpdateViscosityAcceleration()
    {
        float dim = 2.0f;                                                // Dimensionality of the simulation
        float constFactor = 2.0f * (dim + 2.0f) * VISCOSITY_COEFFICIENT; // Constant factor for viscosity calculation

        for (unsigned int i = mPs.mStartIndex; i < mPs.mPositions.size(); i++)
        {
            if (!mPs.mNeighbors.empty())
            {
                glm::vec2 viscosityForce(0.0f, 0.0f);
                for (const auto &nInfo : mPs.mNeighbors[i])
                {
                    int j = nInfo.index;
                    float dotDvToRad = glm::dot(mPs.mVelocity[i] - mPs.mVelocity[j], nInfo.radius);                   // Dot product of velocity difference and radius
                    float denom = nInfo.distance2 + 0.01f * SUPPORT_RADIUS2;                                          // Denominator for viscosity calculation
                    viscosityForce += (PARTICLE_MASS / mPs.mDensity[j]) * dotDvToRad * mW.Grad(nInfo.radius) / denom; // Sum viscosity forces
                }
                viscosityForce *= constFactor;          // Scale viscosity force
                mPs.mAcceleration[i] += viscosityForce; // Update acceleration with viscosity force
            }
        }
    }

    // Update pressure acceleration for all particles
    void UpdatePressureAcceleration()
    {
        std::vector<float> pressDivDens2(mPs.mPositions.size(), 0);

        for (unsigned int i = 0; i < mPs.mPositions.size(); i++)
        {
            pressDivDens2[i] = mPs.mPressure[i] / std::powf(mPs.mDensity[i], 2); // Precompute pressure divided by density squared
        }

        for (unsigned int i = mPs.mStartIndex; i < mPs.mPositions.size(); i++)
        {
            if (!mPs.mNeighbors.empty())
            {
                glm::vec2 pressureForce(0.0f, 0.0f);
                for (const auto &nInfo : mPs.mNeighbors[i])
                {
                    int j = nInfo.index;
                    pressureForce += mPs.mDensity[j] * (pressDivDens2[i] + pressDivDens2[j]) * mW.Grad(nInfo.radius); // Sum pressure forces
                }
                mPs.mAcceleration[i] -= pressureForce * PARTICLE_VOLUME; // Update acceleration with pressure force
            }
        }
    }

    // Integrate particle velocities and positions using Euler integration
    void EulerIntegrate()
    {
        for (unsigned int i = mPs.mStartIndex; i < mPs.mPositions.size(); i++)
        {
            mPs.mVelocity[i] += DELTA_T * mPs.mAcceleration[i];                                     // Update velocity using acceleration
            mPs.mVelocity[i] = glm::clamp(mPs.mVelocity[i], glm::vec2(-100.0f), glm::vec2(100.0f)); // Clamp velocity to maximum allowable value
            mPs.mPositions[i] += DELTA_T * mPs.mVelocity[i];                                        // Update position using velocity
        }
    }

    // Apply boundary conditions to ensure particles stay within the container
    void BoundaryCondition()
    {
        for (unsigned int i = 0; i < mPs.mPositions.size(); i++)
        {
            glm::vec2 &position = mPs.mPositions[i];
            bool invFlag = false;

            if (position.y < mPs.mLowerBound.y + SUPPORT_RADIUS)
            {
                mPs.mVelocity[i].y = std::abs(mPs.mVelocity[i].y); // Reflect velocity if particle hits lower bound
                invFlag = true;
            }
            if (position.y > mPs.mUpperBound.y - SUPPORT_RADIUS)
            {
                mPs.mVelocity[i].y = -std::abs(mPs.mVelocity[i].y); // Reflect velocity if particle hits upper bound
                invFlag = true;
            }
            if (position.x < mPs.mLowerBound.x + SUPPORT_RADIUS)
            {
                mPs.mVelocity[i].x = std::abs(mPs.mVelocity[i].x); // Reflect velocity if particle hits left bound
                invFlag = true;
            }
            if (position.x > mPs.mUpperBound.x - SUPPORT_RADIUS)
            {
                mPs.mVelocity[i].x = -std::abs(mPs.mVelocity[i].x); // Reflect velocity if particle hits right bound
                invFlag = true;
            }

            if (invFlag)
            {
                mPs.mPositions[i] += DELTA_T * mPs.mVelocity[i];                                        // Update position after reflection
                mPs.mVelocity[i] = glm::clamp(mPs.mVelocity[i], glm::vec2(-100.0f), glm::vec2(100.0f)); // Clamp velocity to maximum allowable value
            }
        }
    }

private:
    ParticleSystem &mPs; // Reference to the particle system
    WCubicSpline2d mW;   // Cubic spline kernel for smoothing
};

#endif // PARTICLE_SYSTEM_H
