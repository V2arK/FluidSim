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

class CubicSplineKernel
{
public:
    CubicSplineKernel() = delete; // Delete default constructor

    // Constructor with support radius parameter
    explicit CubicSplineKernel(float h)
    {
        supportRadius = h;
        supportRadiusSquared = h * h;
        supportRadiusCubed = supportRadiusSquared * h;
        normalizationFactor = 8.0 / (glm::pi<float>() * supportRadiusCubed);

        bufferSize = 128;
        valueAndGradientBuffer = std::vector<glm::vec2>(bufferSize);

        // Precompute values and gradient factors
        for (uint32_t i = 0; i < bufferSize; i++)
        {
            float distance = ((float)i + 0.5f) * supportRadius / bufferSize; // [0, h]
            valueAndGradientBuffer[i].x = calculateValue(distance);
            valueAndGradientBuffer[i].y = calculateGradientFactor(distance);
        }
    }
    ~CubicSplineKernel() {}

    // Get precomputed value and gradient based on input distance
    glm::vec2 getValue(float distance)
    {
        if (distance < 0.0f || distance > supportRadius)
        {
            return glm::vec2(0.0f, 0.0f);
        }

        float index = distance / supportRadius * bufferSize;
        uint32_t lowerIndex = static_cast<uint32_t>(index);
        uint32_t upperIndex = lowerIndex + 1;

        if (upperIndex >= bufferSize)
        {
            upperIndex = bufferSize - 1;
        }

        float t = index - lowerIndex;
        glm::vec2 lowerValue = valueAndGradientBuffer[lowerIndex];
        glm::vec2 upperValue = valueAndGradientBuffer[upperIndex];

        return glm::mix(lowerValue, upperValue, t);
    }

    // Get size of the buffer
    uint32_t getBufferSize() { return bufferSize; }

    // Calculate spline value for a given distance
    float calculateValue(float distance)
    {
        float r = std::abs(distance);
        float q = r / supportRadius;
        float q2 = q * q;
        float q3 = q * q2;
        float result = 0.0f;
        if (q < 0.5f)
        {
            result = 6.0f * (q3 - q2) + 1.0f;
            result *= normalizationFactor;
            return result;
        }
        else if (q >= 0.5f && q < 1.0f)
        {
            result = 1.0f - q;
            result = std::pow(result, 3) * 2.0f;
            result *= normalizationFactor;
            return result;
        }
        return result;
    }

    // Calculate gradient factor for a given distance
    float calculateGradientFactor(float distance)
    {
        float result = 0.0f;
        if (distance < 1e-5)
        {
            return result;
        }

        float q = distance / supportRadius;

        if (q < 0.5f)
        {
            result = 6.0f * (3.0f * q * q - 2.0f * q) * normalizationFactor / (supportRadius * distance);
            return result;
        }
        else if (q >= 0.5 && q < 1.0f)
        {
            result = -6.0f * std::powf(1.0f - q, 2) * normalizationFactor / (supportRadius * distance);
            return result;
        }
        return result;
    }

    float supportRadius;                           // Support radius
    float supportRadiusSquared;                    // Support radius squared
    float supportRadiusCubed;                      // Support radius cubed
    float normalizationFactor;                     // Normalization factor
    uint32_t bufferSize;                           // Buffer size
    std::vector<glm::vec2> valueAndGradientBuffer; // Buffer for values and gradient factors
};

/* ----------------- Particle System -------------------- */

// Structure to hold particle information
struct ParticleInfo
{
    alignas(16) glm::vec3 position;
    alignas(16) glm::vec3 velocity;
    alignas(16) glm::vec3 acceleration;
    alignas(4) float density;
    alignas(4) float pressure;
    alignas(4) float pressureOverDensitySquared;
    alignas(4) uint32_t blockId;
};

class ParticleSystem
{
public:
    ParticleSystem() {}
    ~ParticleSystem() {}

    /* ----------------- Physic Solver -------------------- */

    void EulerIntegration()
    {
        for (auto &particle : particles)
        {
            particle.velocity += DELTA_T * particle.acceleration;
            particle.velocity = glm::clamp(particle.velocity, glm::vec3(-MAX_VELOCITY), glm::vec3(MAX_VELOCITY));
            particle.position += DELTA_T * particle.velocity;
        }
    }

    void ComputeDensityAndPressure()
    {
        // Clear the densities and pressure before computation
        for (auto &particle : particles)
        {
            particle.density = REFERENCE_DENSITY;
            particle.pressure = 0.0f;
        }

        // Iterate over each particle to compute density
        for (auto &block : blockOfParticles)
        {
            for (auto cur_particle : block.second)
            {
                for (auto neighbor_particle : block.second)
                {
                    if (cur_particle == neighbor_particle)
                    {
                        continue;
                    }
                    // compute all distintive pair of neighbouring particles
                    glm::vec3 radius_ij = cur_particle->position - neighbor_particle->position;
                    float distance_ij = glm::length(radius_ij);

                    if (distance_ij < SUPPORT_RADIUS)
                    {
                        cur_particle->density += kernel.getValue(distance_ij / SUPPORT_RADIUS)[0];
                    }
                }

                cur_particle->density *= (volume * REFERENCE_DENSITY);
                // Ensure the fluid do not expand
                cur_particle->density = std::max(cur_particle->density, REFERENCE_DENSITY);

                // Calculate pressure using the equation of state
                cur_particle->pressure = STIFFNESS_CONSTANT * (std::powf(cur_particle->density / REFERENCE_DENSITY, PRESSURE_EXPONENT) - 1.0f);

                // Precompute pressure over density squared for later use in force calculations
                cur_particle->pressureOverDensitySquared = cur_particle->pressure / (cur_particle->density * cur_particle->density);
            }
        }
    }
    
    
    // Compute Acceleration
    void ComputeAcceleration()
    {

        for (auto it = particles.begin(); it != particles.end(); ++it)
        {
            // we add gravity here (initialize)
            it->acceleration = GRAVITY_DIR * GRAVITY;
        }

        for (auto &block : blockOfParticles)
        {
            for (auto cur_particle : block.second)
            {
                const float DIM = 3.0f;
                const float CONST_FACTOR = 2.0f * (DIM + 2.0f) * VISCOSITY_COEFFICIENT;
                glm::vec3 viscosityForce = glm::vec3(0.0f);
                glm::vec3 pressureForce = glm::vec3(0.0f);

                for (auto other_particle : block.second)
                {
                    if (cur_particle == other_particle)
                    {
                        continue;
                    }

                    glm::vec3 radiusIj = cur_particle->position - other_particle->position;
                    float distanceIj = glm::length(radiusIj);
                    if (distanceIj <= SUPPORT_RADIUS)
                    {
                        float dotDvToRad = glm::dot(cur_particle->velocity - other_particle->velocity, radiusIj);
                        float denom = distanceIj * distanceIj + 0.01f * SUPPORT_RADIUS * SUPPORT_RADIUS;
                        glm::vec3 wGrad = kernel.getValue(distanceIj / SUPPORT_RADIUS)[1] * radiusIj;
                        viscosityForce += (volume * REFERENCE_DENSITY / other_particle->density) * dotDvToRad * wGrad / denom;
                        pressureForce += other_particle->density * (cur_particle->pressureOverDensitySquared + other_particle->pressureOverDensitySquared) * wGrad;
                    }
                }
                cur_particle->acceleration += viscosityForce * CONST_FACTOR;
                cur_particle->acceleration -= pressureForce * volume;
            }
        }
    }

    void BoundaryCheck()
    {
        for (auto &particle : particles)
        {
            // Check x-axis
            if (particle.position.x > upperBound.x)
            {
                particle.position.x = upperBound.x;
                particle.velocity.x *= -VELOCITY_ATTENUATION;
            }
            else if (particle.position.x < lowerBound.x)
            {
                particle.position.x = lowerBound.x;
                particle.velocity.x *= -VELOCITY_ATTENUATION;
            }

            // Check y-axis
            if (particle.position.y > upperBound.y)
            {
                particle.position.y = upperBound.y;
                particle.velocity.y *= -VELOCITY_ATTENUATION;
            }
            else if (particle.position.y < lowerBound.y)
            {
                particle.position.y = lowerBound.y;
                particle.velocity.y *= -VELOCITY_ATTENUATION;
            }

            // Check z-axis
            if (particle.position.z > upperBound.z)
            {
                particle.position.z = upperBound.z;
                particle.velocity.z *= -VELOCITY_ATTENUATION;
            }
            else if (particle.position.z < lowerBound.z)
            {
                particle.position.z = lowerBound.z;
                particle.velocity.z *= -VELOCITY_ATTENUATION;
            }
        }
    }

    /* ----------------- Misc functions -------------------- */

    // Set container size and initialize blocks
    void setContainerSize(glm::vec3 corner, glm::vec3 size)
    {
        lowerBound = corner - SUPPORT_RADIUS + PARTICLE_DIAMETER;
        upperBound = corner + size + SUPPORT_RADIUS - PARTICLE_DIAMETER;
        containerCenter = (lowerBound + upperBound) / 2.0f;
        size = upperBound - lowerBound;

        // Number of blocks in each direction for computation
        blockCount.x = floor(size.x / SUPPORT_RADIUS);
        blockCount.y = floor(size.y / SUPPORT_RADIUS);
        blockCount.z = floor(size.z / SUPPORT_RADIUS);

        // Size of each block
        blockSize = glm::vec3(size.x / blockCount.x, size.y / blockCount.y, size.z / blockCount.z);

        particles.clear();
    }

    // Add a block of fluid particles
    int32_t addFluidBlock(glm::vec3 corner, glm::vec3 size, glm::vec3 initialVelocity, float particleSpacing)
    {
        glm::vec3 blockLowerBound = corner;
        glm::vec3 blockUpperBound = corner + size;

        // Check if block is within container bounds
        if (blockLowerBound.x < lowerBound.x ||
            blockLowerBound.y < lowerBound.y ||
            blockLowerBound.z < lowerBound.z ||
            blockUpperBound.x > upperBound.x ||
            blockUpperBound.y > upperBound.y ||
            blockUpperBound.z > upperBound.z)
        {
            return 0;
        }

        // Number of particles in each direction
        glm::uvec3 particleCount = glm::uvec3(size.x / particleSpacing, size.y / particleSpacing, size.z / particleSpacing);
        std::vector<ParticleInfo> newParticles(particleCount.x * particleCount.y * particleCount.z);

        // Initialize particle properties
        int index = 0;
        for (int x = 0; x < particleCount.x; x++)
        {
            for (int y = 0; y < particleCount.y; y++)
            {
                for (int z = 0; z < particleCount.z; z++)
                {
                    // populate vector newParticles with particles one by one
                    float offsetX = (x + PCG32::randomFloat()) * particleSpacing;
                    float offsetY = (y + PCG32::randomFloat()) * particleSpacing;
                    float offsetZ = (z + PCG32::randomFloat()) * particleSpacing;
                    newParticles[index].position = corner + glm::vec3(offsetX, offsetY, offsetZ);
                    newParticles[index].blockId = getBlockIdByPosition(newParticles[index].position);
                    newParticles[index].velocity = initialVelocity;
                    index++;
                }
            }
        }

        particles.insert(particles.end(), newParticles.begin(), newParticles.end());
        return newParticles.size();
    }

    // Get block ID based on particle position
    uint32_t getBlockIdByPosition(glm::vec3 position)
    {
        if (position.x < lowerBound.x ||
            position.y < lowerBound.y ||
            position.z < lowerBound.z ||
            position.x > upperBound.x ||
            position.y > upperBound.y ||
            position.z > upperBound.z)
        {
            return -1;
        }

        glm::vec3 deltaPosition = position - lowerBound;
        uint32_t column = floor(deltaPosition.x / blockSize.x);
        uint32_t row = floor(deltaPosition.y / blockSize.y);
        uint32_t height = floor(deltaPosition.z / blockSize.z);
        return computeHash(row, column, height);
    }

    void updateData()
    {

        // sort particles into blockOfParticles, key is computeHash()
        blockOfParticles.clear();
        for (auto it = particles.begin(); it != particles.end(); ++it)
        {

            unsigned int hashValue = getBlockIdByPosition(it->position);
            blockOfParticles[hashValue].push_back(&(*it));
        }

        ComputeDensityAndPressure();
        ComputeAcceleration();
        EulerIntegration();
        BoundaryCheck();
        
    }

public:
    // Particle parameters
    float supportRadius = SUPPORT_RADIUS; // Support radius
    float supportRadiusSquared = supportRadius * supportRadius;
    float particleRadius = PARTICLE_RADIUS; // Particle radius
    float particleDiameter = PARTICLE_DIAMETER;
    float volume = std::pow(particleDiameter, 3);       // Volume
    float mass = REFERENCE_DENSITY * volume;            // Mass
    float viscosityCoefficient = VISCOSITY_COEFFICIENT; // Viscosity coefficient
    float pressureExponent = PRESSURE_EXPONENT;         // Pressure exponent
    int stiffnessConstant = STIFFNESS_CONSTANT;         // Stiffness
    std::vector<ParticleInfo> particles;                // the particles lives here

    // each blocks of particles, key is from computeHash().
    std::unordered_map<unsigned int, std::vector<ParticleInfo *>> blockOfParticles;

    // Container parameters
    glm::vec3 lowerBound = glm::vec3(FLT_MAX);
    glm::vec3 upperBound = glm::vec3(-FLT_MAX);
    glm::vec3 containerCenter = glm::vec3(0.0f);
    glm::uvec3 blockCount = glm::uvec3(0); // Number of blocks in XYZ directions
    glm::vec3 blockSize = glm::vec3(0.0f);

    // Smoothing kernel function
    CubicSplineKernel kernel = CubicSplineKernel(supportRadius);
};

#endif // !PARTICLE_SYSTEM_H
