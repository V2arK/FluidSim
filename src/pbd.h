// misc
#include <iostream>
#include <vector>
#include <cfloat>
#include <thread>
#include <algorithm>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// particle system related
const float deltaT = 0.002f; // get the firrst frame started
const bool globalEnableParticles = false;
const glm::vec3 globalGravity = glm::vec3(0.0f, -0.05f, 0.0f);
const int globalNumParticles = 1000;
const float G = 2e-3f; // Gravitational constant, particle mass = 1.0f
const float EpsilonGravity = 2e-2f;
const float sphereRadius = 0.03f;              // for the spherical particles

// fast random number generator based pcg32_fast
#include <stdint.h>
namespace PCG32
{
    static uint64_t mcg_state = 0xcafef00dd15ea5e5u; // must be odd
    static uint64_t const multiplier = 6364136223846793005u;
    uint32_t pcg32_fast(void)
    {
        uint64_t x = mcg_state;
        const unsigned count = (unsigned)(x >> 61);
        mcg_state = x * multiplier;
        x ^= x >> 22;
        return (uint32_t)(x >> (22 + count));
    }
    float rand()
    {
        return float(double(pcg32_fast()) / 4294967296.0);
    }
}

// ====== implement it in A3 ======
// fill in the missing parts

class Particle
{
public:
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 velocity = glm::vec3(0.0f);
    glm::vec3 prevPosition = position;

    glm::vec3 force = glm::vec3(0.0f); // Force accumulator
    float mass = 1.0f;                 // Particle mass
    float radius = sphereRadius;              // for the spherical particles

    bool resolveCollision(Particle &other)
    {
        glm::vec3 diff = position - other.position;
        float dist = glm::distance(position, other.position);
        float minDist = radius + other.radius;

        //printf("dist = %f, minDist = %f\n", dist, minDist);
        if (dist < minDist)
        {
            glm::vec3 collisionNormal = glm::normalize(diff);
            float penetrationDepth = minDist - dist;

            // Move particles apart to resolve the collision
            // half for each
            position += 0.5f * penetrationDepth * collisionNormal;
            other.position -= 0.5f * penetrationDepth * collisionNormal;

            // Calculate relative velocity`
            glm::vec3 relativeVelocity = velocity - other.velocity;

            // Calculate velocity along the collision normal
            float collisionImpulse = dot(relativeVelocity, collisionNormal);

            // Apply collision impulse
            if (collisionImpulse < 0)
            {
                force -= (collisionImpulse / mass) * collisionNormal;
                other.force += (collisionImpulse / other.mass) * collisionNormal;
            }
            //printf("collision!\n");
            return true;
        }
        //printf("no collision!\n");
        return false;
    }

    void reset()
    {
        position = glm::vec3(PCG32::rand(), PCG32::rand(), PCG32::rand()) - float(0.5f);
        velocity = 2.0f * glm::vec3((PCG32::rand() - 0.5f), 0.0f, (PCG32::rand() - 0.5f));

        prevPosition = position;
        position += velocity * deltaT;
    }

    void step(float deltaTime)
    {
        // Save the current position
        glm::vec3 temp = position;

        // Collision box [-0.5, 0.5] × [-0.5, 0.5] × [-0.5, 0.5]
		for (int axis = 0; axis < 3; ++axis)
		{
			float clampAmount = position[axis];
			// save the clamp amount for position,
			// gonna apply the same amount to prevPosition as well

			if (position[axis] > 0.5f)
			{
				// Reflect prevPosition
				temp[axis] = 2.0f * position[axis] - temp[axis];
				// Move position to the boundary]
				position[axis] = 0.5f;
			}
			else if (position[axis] < -0.5f)
			{
				// Reflect prevPosition
				temp[axis] = 2.0f * position[axis] - temp[axis];
				// Move position to the boundary
				position[axis] = -0.5f;
			}

			// move the prevPosition by the same amount
			clampAmount -= position[axis];
			temp[axis] -= clampAmount;
		}

        // Calculate the new position using Verlet integration
        glm::vec3 acceleration = force / mass;
        position = position + (position - prevPosition) + (acceleration + globalGravity) * (deltaTime * deltaTime);

        // Velocity
        // v(t + dt) = (r(t + dt) - r(t - dt)) / (2 * dt)
        velocity = (position - temp) / (2.0f * deltaTime);

        // Update the previous position
        prevPosition = temp;
    }
};

class ParticleSystem
{
public:
    std::vector<Particle> particles;
    float sphereSize = 0.0f;

    ParticleSystem() {};

    void initialize()
    {
        particles.resize(globalNumParticles);

        for (int i = 0; i < globalNumParticles; i++)
        {
            particles[i].reset();
        }
    }

    void step(float deltaTime)
    {

        for (auto &particle : particles)
        {
            particle.force = glm::vec3(0.0f); // Reset force accumulator
        }
        // Handle collisions
        for (int i = 0; i < globalNumParticles; i++)
        {
            for (int j = i + 1; j < globalNumParticles; j++)
            {
                particles[i].resolveCollision(particles[j]);
            }
        }

        // add some particle-particle interaction here
        // spherical particles can be implemented here
        for (int i = 0; i < globalNumParticles; i++)
        {
            particles[i].step(deltaTime);
        }
    }
};

static ParticleSystem globalParticleSystem;
