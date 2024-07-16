#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <stdint.h>
#include <thread>

/* ----------------- Multi thread Constants -------------------- */

const unsigned int THREAD_COUNT = std::thread::hardware_concurrency();

/* ----------------- Physical Constants -------------------- */

const float REFERENCE_DENSITY = 1000.0f;   // Reference density for fluid
const float STIFFNESS_CONSTANT = 50.0f;    // Stiffness constant for pressure calculation
const float PRESSURE_EXPONENT = 7.0f;      // Exponent used in the equation of state for pressure
const float VISCOSITY_COEFFICIENT = 0.01f; // Viscosity coefficient for fluid
const float TIME_STEP = 0.0005f;           // Time step for Euler integration
const float MAX_VELOCITY = 100.0f;         // Maximum allowable particle velocity
const float VELOCITY_ATTENUATION = 0.9f;   // Velocity attenuation factor for particle collisions

const float GRAVITY = 9.8f;                               // Gravitational acceleration
const glm::vec2 GRAVITY_DIRECTION = glm::vec2(0.0, -1.0); // Direction of gravity

const float PARTICLE_RADIUS = 0.005f;                                     // Radius of each particle
const float SUPPORT_RADIUS = 5.0f * PARTICLE_RADIUS;                      // Support radius for smoothing kernels
const float SUPPORT_RADIUS_SQUARED = SUPPORT_RADIUS * SUPPORT_RADIUS;     // Square of the support radius
const float PARTICLE_DIAMETER = PARTICLE_RADIUS * 2.0f;                   // Diameter of a particle
const float PARTICLE_AREA = 0.8f * PARTICLE_DIAMETER * PARTICLE_DIAMETER; // Area of a particle
const float PARTICLE_MASS = REFERENCE_DENSITY * PARTICLE_AREA;            // Mass of a particle

/* ----------------- Random Generator -------------------- */

namespace PCG32
{
    static uint64_t state = 0xcafef00dd15ea5e5u;             // Initial state for the random generator
    static uint64_t const multiplier = 6364136223846793005u; // Multiplier for random generation

    uint32_t generate()
    {
        uint64_t x = state;
        const unsigned count = (unsigned)(x >> 61);
        state = x * multiplier;
        x ^= x >> 22;
        return (uint32_t)(x >> (22 + count));
    }

    float randomFloat()
    {
        return float(double(generate()) / 4294967296.0);
    }
}

inline unsigned int computeHash(unsigned int x = 0, unsigned int y = 0, unsigned int z = 0)
{
    const unsigned int prime1 = 73856093;
    const unsigned int prime2 = 19349663;
    const unsigned int prime3 = 83492791;
    return (x * prime1) ^ (y * prime2) ^ (z * prime3);
}

/* ----------------- Cubic Spline Kernel -------------------- */

class CubicSplineKernel
{
public:
    CubicSplineKernel() = delete;

    explicit CubicSplineKernel(float smoothingRadius)
        : smoothingRadius_(smoothingRadius),
          smoothingRadiusSquared_(smoothingRadius * smoothingRadius),
          normalizationConstant_(40.0 / (7.0 * glm::pi<float>() * smoothingRadiusSquared_)),
          bufferSize_(128, 128),
          gradientBuffer_(bufferSize_.x, std::vector<glm::vec2>(bufferSize_.y)),
          valueBuffer_(bufferSize_.x)
    {
        for (int i = 0; i < bufferSize_.x; i++)
        {
            for (int j = 0; j < bufferSize_.y; j++)
            {
                float x = ((float)i + 0.5f) * smoothingRadius_ / bufferSize_.x;
                float y = ((float)j + 0.5f) * smoothingRadius_ / bufferSize_.y;
                glm::vec2 radius(x, y);
                gradientBuffer_[i][j] = CalculateGradient(radius);
            }
        }

        for (int i = 0; i < bufferSize_.x; i++)
        {
            float distance = ((float)i + 0.5f) * smoothingRadius_ / bufferSize_.x;
            valueBuffer_[i] = CalculateValue(distance);
        }
    }

    ~CubicSplineKernel() = default;

    float Value(float distance) const
    {
        float result = 0;
        int index = static_cast<int>(std::abs(distance) * bufferSize_.x / smoothingRadius_);
        if (index < bufferSize_.x)
        {
            result = valueBuffer_[index];
        }
        return result;
    }

    glm::vec2 Gradient(glm::vec2 radius) const
    {
        glm::vec2 result(0.0f, 0.0f);

        int i = static_cast<int>(std::abs(radius.x) * bufferSize_.x / smoothingRadius_);
        int j = static_cast<int>(std::abs(radius.y) * bufferSize_.y / smoothingRadius_);

        if (i < bufferSize_.x && j < bufferSize_.y)
        {
            result = gradientBuffer_[i][j];

            if (radius.x < 0)
                result.x = -result.x;
            if (radius.y < 0)
                result.y = -result.y;
        }

        return result;
    }

private:
    float CalculateValue(float distance) const
    {
        float radius = std::abs(distance);
        float q = radius / smoothingRadius_;
        float qSquared = q * q;
        float qCubed = q * qSquared;
        float result = 0.0f;

        if (q < 0.5f)
        {
            result = 6.0f * (qCubed - qSquared) + 1.0f;
        }
        else if (q < 1.0f)
        {
            result = 2.0f * std::powf(1.0f - q, 3);
        }

        return result * normalizationConstant_;
    }

    glm::vec2 CalculateGradient(glm::vec2 radius) const
    {
        glm::vec2 result(0.0f, 0.0f);
        float distance = glm::length(radius);
        if (distance < 1e-5)
            return result;

        float q = distance / smoothingRadius_;
        glm::vec2 qGrad = radius / (smoothingRadius_ * distance);

        if (q < 0.5f)
        {
            result = 6.0f * (3.0f * q * q - 2.0f * q) * normalizationConstant_ * qGrad;
        }
        else if (q < 1.0f)
        {
            result = -6.0f * std::powf(1.0f - q, 2) * normalizationConstant_ * qGrad;
        }

        return result;
    }

    float smoothingRadius_;                              // Smoothing radius
    float smoothingRadiusSquared_;                       // Square of the smoothing radius
    float normalizationConstant_;                        // Normalization constant for the kernel
    glm::uvec2 bufferSize_;                              // Buffer size for precomputed values
    std::vector<std::vector<glm::vec2>> gradientBuffer_; // Gradient buffer for precomputed values
    std::vector<float> valueBuffer_;                     // Value buffer for precomputed values
};

/* ----------------- Particle System -------------------- */

struct NeighborInfo
{
    int particleIndex;      // Index of the neighboring particle
    float distance;         // Distance to the neighboring particle
    float distanceSquared;  // Square of the distance to the neighboring particle
    glm::vec2 radiusVector; // Vector pointing to the neighboring particle
};

class ParticleSystem
{
public:
    ParticleSystem() : kernel_(SUPPORT_RADIUS) {}
    ~ParticleSystem() = default;

    void SetContainerSize(glm::vec2 lowerCorner, glm::vec2 upperCorner)
    {
        lowerBound_ = lowerCorner;
        upperBound_ = upperCorner;

        glm::vec2 size = upperBound_ - lowerBound_;

        blockRowCount_ = static_cast<uint32_t>(floor(size.y / SUPPORT_RADIUS));
        blockColumnCount_ = static_cast<uint32_t>(floor(size.x / SUPPORT_RADIUS));
        blockSize_ = glm::vec2(size.x / blockColumnCount_, size.y / blockRowCount_);

        particlePositions_.clear();
        particleVelocities_.clear();
        particleAccelerations_.clear();
    }

    unsigned int AddFluidBlock(glm::vec2 blockLowerBound, glm::vec2 blockUpperBound, glm::vec2 initialVelocity = glm::vec2(0.0f, 0.0f), float particleSpacing = 0.007f)
    {

        glm::vec2 size = blockUpperBound - blockLowerBound;

        if (blockLowerBound.x < lowerBound_.x ||
            blockLowerBound.y < lowerBound_.y ||
            blockUpperBound.x > upperBound_.x ||
            blockUpperBound.y > upperBound_.y)
        {
            return -1;
        }

        int width = static_cast<int>(size.x / particleSpacing);
        int height = static_cast<int>(size.y / particleSpacing);

        std::vector<glm::vec2> positions(width * height);
        std::vector<glm::vec2> velocities(width * height, initialVelocity);
        std::vector<glm::vec2> accelerations(width * height, glm::vec2(0.0f, 0.0f));

        int particleIndex = 0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                positions[particleIndex] = blockLowerBound + glm::vec2((j + PCG32::randomFloat()) * particleSpacing, (i + PCG32::randomFloat()) * particleSpacing);
                particleIndex++;
            }
        }

        particlePositions_.insert(particlePositions_.end(), positions.begin(), positions.end());
        particleVelocities_.insert(particleVelocities_.end(), velocities.begin(), velocities.end());
        particleAccelerations_.insert(particleAccelerations_.end(), accelerations.begin(), accelerations.end());
        return positions.size();
    }

    void GridSearch()
    {
        blocks_ = std::vector<std::vector<unsigned int>>(blockColumnCount_ * blockRowCount_, std::vector<unsigned int>(0));

        for (unsigned int i = 0; i < particlePositions_.size(); i++)
        {
            int blockId = GetBlockIdByPosition(particlePositions_[i]);
            blocks_[blockId].push_back(static_cast<int>(i));
        }

        neighbors_ = std::vector<std::vector<NeighborInfo>>(particlePositions_.size(), std::vector<NeighborInfo>(0));

        std::vector<std::thread> threads;

        auto worker = [this](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                glm::vec2 deltaPosition = particlePositions_[i] - lowerBound_;
                uint32_t blockColumn = static_cast<uint32_t>(floor(deltaPosition.x / blockSize_.x));
                uint32_t blockRow = static_cast<uint32_t>(floor(deltaPosition.y / blockSize_.y));

                int blockId = GetBlockIdByPosition(particlePositions_[i]);

                for (int rowOffset = -1; rowOffset <= 1; rowOffset++)
                {
                    for (int columnOffset = -1; columnOffset <= 1; columnOffset++)
                    {
                        if (blockColumn + columnOffset < 0 || blockColumn + columnOffset >= blockColumnCount_ ||
                            blockRow + rowOffset < 0 || blockRow + rowOffset >= blockRowCount_)
                        {
                            continue;
                        }

                        int neighborBlockId = blockId + rowOffset * blockColumnCount_ + columnOffset;
                        auto &block = blocks_[neighborBlockId];
                        for (int j : block)
                        {
                            if (i == j)
                                continue;

                            NeighborInfo neighborInfo{};
                            neighborInfo.radiusVector = particlePositions_[i] - particlePositions_[j];
                            neighborInfo.distance = glm::length(neighborInfo.radiusVector);
                            neighborInfo.distanceSquared = neighborInfo.distance * neighborInfo.distance;
                            neighborInfo.particleIndex = j;
                            if (neighborInfo.distance <= SUPPORT_RADIUS)
                            {
                                neighbors_[i].push_back(neighborInfo);
                            }
                        }
                    }
                }
            }
        };

        const unsigned int particlesPerThread = particlePositions_.size() / THREAD_COUNT;
        unsigned int start = 0;

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    unsigned int GetBlockIdByPosition(glm::vec2 position) const
    {
        if (position.x < lowerBound_.x ||
            position.y < lowerBound_.y ||
            position.x > upperBound_.x ||
            position.y > upperBound_.y)
        {
            return static_cast<unsigned int>(-1);
        }

        glm::vec2 deltaPosition = position - lowerBound_;
        uint32_t x = static_cast<uint32_t>(floor(deltaPosition.x / blockSize_.x));
        uint32_t y = static_cast<uint32_t>(floor(deltaPosition.y / blockSize_.y));
        return y * blockColumnCount_ + x;
    }

    void clearParticle()
    {
        particlePositions_.clear();
        particleAccelerations_.clear();
        particleVelocities_.clear();
        particleDensities_.clear();
        particlePressures_.clear();
        neighbors_.clear();
    }

    /* ----------------- SPH Implementation -------------------- */

    void Iterate()
    {
        ResetAcceleration();           // Initialize accelerations of particles
        GridSearch();             // build neighbour
        DensityAndPressure();    // Update densities and pressures of particles
        ViscosityAcceleration(); // Update accelerations due to viscosity
        PressureAcceleration();  // Update accelerations due to pressure
        MoveStep();              // Integrate positions and velocities using Euler method
        CheckBoundary();     // Apply boundary conditions to particles
    }

    void DensityAndPressure()
    {
        particleDensities_.assign(particlePositions_.size(), REFERENCE_DENSITY); // Initialize densities to reference value
        particlePressures_.assign(particlePositions_.size(), 0.0f);              // Initialize pressures to zero

        std::vector<std::thread> threads;

        auto worker = [this](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                if (!neighbors_.empty())
                {
                    float density = 0;
                    for (const auto &neighborInfo : neighbors_[i])
                    {
                        density += kernel_.Value(neighborInfo.distance); // Sum contributions from neighboring particles
                    }
                    density *= (PARTICLE_AREA * REFERENCE_DENSITY);               // Scale density by particle area and reference density
                    particleDensities_[i] = std::max(density, REFERENCE_DENSITY); // Prevent expansion
                }

                particlePressures_[i] = STIFFNESS_CONSTANT * (std::powf(particleDensities_[i] / REFERENCE_DENSITY, PRESSURE_EXPONENT) - 1.0f);
            }
        };

        const unsigned int particlesPerThread = particlePositions_.size() / THREAD_COUNT;
        unsigned int start = 0;

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    void ResetAcceleration()
    {
        std::fill(particleAccelerations_.begin() + 0, particleAccelerations_.end(), glm::vec2(0.0f, -GRAVITY)); // Initialize accelerations to gravity
    }

    void ViscosityAcceleration()
    {
        float dimension = 2.0f;                                                    // Dimensionality of the simulation
        float viscosityFactor = 2.0f * (dimension + 2.0f) * VISCOSITY_COEFFICIENT; // Constant factor for viscosity
        std::vector<std::thread> threads;

        auto worker = [this, viscosityFactor](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                if (!neighbors_.empty())
                {
                    glm::vec2 viscosityForce(0.0f, 0.0f);
                    for (const auto &neighborInfo : neighbors_[i])
                    {
                        int j = neighborInfo.particleIndex;
                        float velocityDifferenceDotRadius = glm::dot(particleVelocities_[i] - particleVelocities_[j], neighborInfo.radiusVector);                            // Dot product of velocity difference and radius
                        float denominator = neighborInfo.distanceSquared + 0.01f * SUPPORT_RADIUS_SQUARED;                                                                   // Denominator for viscosity calculation
                        viscosityForce += (PARTICLE_MASS / particleDensities_[j]) * velocityDifferenceDotRadius * kernel_.Gradient(neighborInfo.radiusVector) / denominator; // Sum viscosity forces
                    }
                    viscosityForce *= viscosityFactor;           // Scale viscosity force
                    particleAccelerations_[i] += viscosityForce; // Update acceleration with viscosity force
                }
            }
        };

        const unsigned int particlesPerThread = particlePositions_.size() / THREAD_COUNT;
        unsigned int start = 0;

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    void PressureAcceleration()
    {
        std::vector<float> pressureOverDensitySquared(particlePositions_.size(), 0);

        std::vector<std::thread> threads;

        auto worker1 = [this, &pressureOverDensitySquared](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                pressureOverDensitySquared[i] = particlePressures_[i] / std::powf(particleDensities_[i], 2); // Precompute pressure divided by density squared
            }
        };

        const unsigned int particlesPerThread = particlePositions_.size() / THREAD_COUNT;
        unsigned int start = 0;

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker1, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }

        auto worker2 = [this, &pressureOverDensitySquared](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                if (!neighbors_.empty())
                {
                    glm::vec2 pressureForce(0.0f, 0.0f);
                    for (const auto &neighborInfo : neighbors_[i])
                    {
                        int j = neighborInfo.particleIndex;
                        pressureForce += particleDensities_[j] * (pressureOverDensitySquared[i] + pressureOverDensitySquared[j]) * kernel_.Gradient(neighborInfo.radiusVector); // Sum pressure forces
                    }
                    particleAccelerations_[i] -= pressureForce * PARTICLE_AREA; // Update acceleration with pressure force
                }
            }
        };

        start = 0;
        threads.clear();

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker2, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    void MoveStep()
    {

        std::vector<std::thread> threads;

        auto worker = [this](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                particleVelocities_[i] += TIME_STEP * particleAccelerations_[i];                                    // Update velocity using acceleration
                particleVelocities_[i] = glm::clamp(particleVelocities_[i], glm::vec2(-100.0f), glm::vec2(100.0f)); // Clamp velocity to maximum allowable value
                particlePositions_[i] += TIME_STEP * particleVelocities_[i];                                        // Update position using velocity
            }
        };

        const unsigned int particlesPerThread = particlePositions_.size() / THREAD_COUNT;
        unsigned int start = 0;

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    void CheckBoundary()
    {

        std::vector<std::thread> threads;

        auto worker = [this](unsigned int start, unsigned int end)
        {
            for (unsigned int i = start; i < end; i++)
            {
                glm::vec2 &position = particlePositions_[i];
                bool inversionFlag = false;

                if (position.y < lowerBound_.y + SUPPORT_RADIUS)
                {
                    particleVelocities_[i].y = std::abs(particleVelocities_[i].y); // Reflect velocity if particle hits lower bound
                    inversionFlag = true;
                }
                if (position.y > upperBound_.y - SUPPORT_RADIUS)
                {
                    particleVelocities_[i].y = -std::abs(particleVelocities_[i].y); // Reflect velocity if particle hits upper bound
                    inversionFlag = true;
                }
                if (position.x < lowerBound_.x + SUPPORT_RADIUS)
                {
                    particleVelocities_[i].x = std::abs(particleVelocities_[i].x); // Reflect velocity if particle hits left bound
                    inversionFlag = true;
                }
                if (position.x > upperBound_.x - SUPPORT_RADIUS)
                {
                    particleVelocities_[i].x = -std::abs(particleVelocities_[i].x); // Reflect velocity if particle hits right bound
                    inversionFlag = true;
                }

                if (inversionFlag)
                {
                    particlePositions_[i] += TIME_STEP * particleVelocities_[i];                                        // Update position after reflection
                    particleVelocities_[i] = glm::clamp(particleVelocities_[i], glm::vec2(-100.0f), glm::vec2(100.0f)); // Clamp velocity to maximum allowable value
                }
            }
        };

        const unsigned int particlesPerThread = particlePositions_.size() / THREAD_COUNT;
        unsigned int start = 0;

        for (unsigned int t = 0; t < THREAD_COUNT; t++)
        {
            unsigned int end = (t == THREAD_COUNT - 1) ? particlePositions_.size() : start + particlesPerThread;
            threads.emplace_back(worker, start, end);
            start = end;
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

private:
    unsigned int AddBoundary(glm::vec2 corner, glm::vec2 size)
    {
        float spacing = PARTICLE_RADIUS / 4.0f;
        int rows = static_cast<int>(floor(size.y / spacing));
        int cols = static_cast<int>(floor(size.x / spacing));

        float rowOffset = (size.y - ((float)rows - 1.0f) * spacing) / 2.0f;
        float colOffset = (size.x - ((float)cols - 1.0f) * spacing) / 2.0f;

        std::vector<glm::vec2> positions(rows * cols);
        std::vector<glm::vec2> velocities(rows * cols, glm::vec2(0.0f, 0.0f));
        std::vector<glm::vec2> accelerations(rows * cols, glm::vec2(0.0f, 0.0f));

        int particleIndex = 0;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                float x = colOffset + j * spacing;
                float y = rowOffset + i * spacing;
                positions[particleIndex] = corner + glm::vec2(x, y);
                particleIndex++;
            }
        }

        particlePositions_.insert(particlePositions_.end(), positions.begin(), positions.end());
        particleVelocities_.insert(particleVelocities_.end(), velocities.begin(), velocities.end());
        particleAccelerations_.insert(particleAccelerations_.end(), accelerations.begin(), accelerations.end());
        return positions.size();
    }

    CubicSplineKernel kernel_; // Cubic spline kernel for smoothing

public:
    std::vector<glm::vec2> particlePositions_;         // Positions of particles
    std::vector<glm::vec2> particleAccelerations_;     // Accelerations of particles
    std::vector<glm::vec2> particleVelocities_;        // Velocities of particles
    std::vector<float> particleDensities_;             // Densities of particles
    std::vector<float> particlePressures_;             // Pressures of particles
    std::vector<std::vector<NeighborInfo>> neighbors_; // Neighbor information for particles

    glm::vec2 lowerBound_;                          // Lower bound of the container
    glm::vec2 upperBound_;                          // Upper bound of the container
    std::vector<std::vector<unsigned int>> blocks_; // Blocks for spatial partitioning
    glm::vec2 blockSize_;                           // Size of each block
    uint32_t blockRowCount_ = 4;                    // Number of block rows
    uint32_t blockColumnCount_ = 4;                 // Number of block columns
};

#endif // PARTICLE_SYSTEM_H
