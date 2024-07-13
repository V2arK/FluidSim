#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <glm/glm.hpp>
#include <vector>
#include <algorithm>

/* ----------------- Physical Constants -------------------- */

const float SUPPORT_RADIUS = 0.025f;                    // Support radius for smoothing kernels
const float PARTICLE_RADIUS = 0.005f;                   // Radius of a single particle
const float PARTICLE_DIAMETER = PARTICLE_RADIUS * 2.0f; // Diameter of a particle
const float GRAVITY = 9.8f;                             // Gravitational acceleration
const float REFERENCE_DENSITY = 1000.0f;                // Reference density
const float STIFFNESS_CONSTANT = 5.0f;                  // Stiffness constant for pressure calculation
const float PRESSURE_EXPONENT = 7.0f;                   // Exponent for the equation of state
const float VISCOSITY_COEFFICIENT = 8e-6f;              // Viscosity coefficient

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

    // Get precomputed data
    float_t *getData()
    {
        return (float_t *)valueAndGradientBuffer.data();
    }

    // Get size of the buffer
    uint32_t getBufferSize() { return bufferSize; }

private:
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
struct ParticleInfo{
    alignas(16) glm::vec3 position;
    alignas(16) glm::vec3 velocity;
    alignas(16) glm::vec3 acceleration;
    alignas(4) float_t density;
    alignas(4) float_t pressure;
    alignas(4) float_t pressureOverDensitySquared;
    alignas(4) uint32_t blockId;
};

class ParticleSystem
{
public:
    ParticleSystem() {}
    ~ParticleSystem() {}

    // Set container size and initialize blocks
    void setContainerSize(glm::vec3 corner, glm::vec3 size)
    {
        lowerBound = corner - SUPPORT_RADIUS + PARTICLE_DIAMETER;
        upperBound = corner + size + SUPPORT_RADIUS - PARTICLE_DIAMETER;
        containerCenter = (lowerBound + upperBound) / 2.0f;
        size = upperBound - lowerBound;

        // Number of blocks in each direction
        blockCount.x = floor(size.x / SUPPORT_RADIUS);
        blockCount.y = floor(size.y / SUPPORT_RADIUS);
        blockCount.z = floor(size.z / SUPPORT_RADIUS);

        // Size of each block
        blockSize = glm::vec3(size.x / blockCount.x, size.y / blockCount.y, size.z / blockCount.z);

        blockIdOffsets.resize(27);
        int index = 0;
        for (int k = -1; k <= 1; k++)
        {
            for (int j = -1; j <= 1; j++)
            {
                for (int i = -1; i <= 1; i++)
                {
                    blockIdOffsets[index] = blockCount.x * blockCount.y * k + blockCount.x * j + i;
                    index++;
                }
            }
        }

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
        return height * blockCount.x * blockCount.y + row * blockCount.x + column;
    }

    // Update particle data and sort by block ID
    void updateData()
    {
        std::sort(particles.begin(), particles.end(),
                  [=](ParticleInfo &first, ParticleInfo &second)
                  {
                      return first.blockId < second.blockId;
                  });

        // Calculate block extents
        blockExtents = std::vector<glm::uvec2>(blockCount.x * blockCount.y * blockCount.z, glm::uvec2(0, 0));
        int currentBlockId = 0;
        int left = 0;
        int right;
        for (right = 0; right < particles.size(); right++)
        {
            if (particles[right].blockId != currentBlockId)
            {
                blockExtents[currentBlockId] = glm::uvec2(left, right); // Left inclusive, right exclusive
                left = right;
                currentBlockId = particles[right].blockId;
            }
        }
        blockExtents[currentBlockId] = glm::uvec2(left, right);
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
    std::vector<ParticleInfo> particles;
    int maxNeighbors = 512;

    // Container parameters
    glm::vec3 lowerBound = glm::vec3(FLT_MAX);
    glm::vec3 upperBound = glm::vec3(-FLT_MAX);
    glm::vec3 containerCenter = glm::vec3(0.0f);
    glm::uvec3 blockCount = glm::uvec3(0); // Number of blocks in XYZ directions
    glm::vec3 blockSize = glm::vec3(0.0f);
    std::vector<glm::uvec2> blockExtents;
    std::vector<int32_t> blockIdOffsets;

    // Smoothing kernel function
    CubicSplineKernel kernel = CubicSplineKernel(supportRadius);
};

#endif // !PARTICLE_SYSTEM_H
