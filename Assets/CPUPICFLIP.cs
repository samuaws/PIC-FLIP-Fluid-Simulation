using UnityEngine;
using System.Collections.Generic;

public class CPUPICFLIP : MonoBehaviour
{
    public int gridWidth = 20;
    public int gridHeight = 20;
    public float cellSize = 0.5f;
    public int numParticles = 100;
    public float viscosityCoefficient = 0.01f; // Lower viscosity for water-like behavior
    public Vector2 gravity = new Vector2(0, -9.81f); // Gravity vector
    public float maxVelocity = 10f; // Maximum allowed particle velocity
    public float dampingFactor = 0.99f; // Damping factor to reduce particle energy
    public float maxAllowedTimeStep = 0.01f; // Max allowed timestep for stability
    public float correctionStrength = 0.25f; // Reduce the correction strength for stability

    // Visualization toggles
    public bool showGrid = true;
    public bool showBounds = true;
    public bool showVelocityField = true;

    private Grid grid;
    public List<Particle> particles;

    void Start()
    {
        grid = new Grid(gridWidth, gridHeight, cellSize);
        particles = new List<Particle>();

        // Initialize particles with random positions and velocities
        for (int i = 0; i < numParticles; i++)
        {
            Vector2 randomPosition = new Vector2(Random.Range(0.1f, gridWidth * cellSize - 0.1f), Random.Range(0.1f, gridHeight * cellSize - 0.1f));
            Vector2 randomVelocity = new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)); // Non-zero initial velocity
            particles.Add(new Particle(randomPosition, randomVelocity));
        }
    }

    void Update()
    {
        float dt = Mathf.Min(Time.deltaTime, maxAllowedTimeStep);

        TransferParticleVelocitiesToGrid();
        ApplyGravityToGrid(dt); // Apply gravity first
        SolvePressure(); // Pressure projection
        UpdateParticleVelocitiesFromGrid();
        AdvectParticles(dt);
        EnforceBoundaries();
        ResolveParticleOverlap(cellSize * 0.5f); // Ensure particles are at least half a cell apart
        // Optionally dampen particle velocities if necessary
    }

    void TransferParticleVelocitiesToGrid()
    {
        // Reset grid velocities and weights
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                grid.velocity[i, j] = Vector2.zero;
                grid.weights[i, j] = 0f; // Reset the weights
            }
        }

        // Transfer particle velocities to the grid
        foreach (Particle particle in particles)
        {
            // Grid cell coordinates
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            // Interpolation weights
            float tx = (particle.position.x / grid.cellSize) - x0;
            float ty = (particle.position.y / grid.cellSize) - y0;

            float w00 = (1 - tx) * (1 - ty); // Top-left weight
            float w10 = tx * (1 - ty);       // Top-right weight
            float w01 = (1 - tx) * ty;       // Bottom-left weight
            float w11 = tx * ty;             // Bottom-right weight

            // Add weighted velocities to grid points
            if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height)
            {
                grid.velocity[x0, y0] += particle.velocity * w00;
                grid.weights[x0, y0] += w00;
            }
            if (x1 >= 0 && x1 < grid.width && y0 >= 0 && y0 < grid.height)
            {
                grid.velocity[x1, y0] += particle.velocity * w10;
                grid.weights[x1, y0] += w10;
            }
            if (x0 >= 0 && x0 < grid.width && y1 >= 0 && y1 < grid.height)
            {
                grid.velocity[x0, y1] += particle.velocity * w01;
                grid.weights[x0, y1] += w01;
            }
            if (x1 >= 0 && x1 < grid.width && y1 >= 0 && y1 < grid.height)
            {
                grid.velocity[x1, y1] += particle.velocity * w11;
                grid.weights[x1, y1] += w11;
            }
        }

        // Normalize the grid velocities by their corresponding weights
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                if (grid.weights[i, j] > 0f)
                {
                    grid.velocity[i, j] /= grid.weights[i, j];
                }
            }
        }
    }

    void ApplyGravityToGrid(float dt)
    {
        // Apply gravity directly to grid velocities
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                grid.velocity[i, j] += gravity * dt;
            }
        }
    }

    void ApplyViscosity(float dt)
    {
        for (int i = 1; i < grid.width - 1; i++)
        {
            for (int j = 1; j < grid.height - 1; j++)
            {
                Vector2 velocity = grid.velocity[i, j];

                Vector2 laplacian = grid.velocity[i - 1, j] + grid.velocity[i + 1, j] +
                                    grid.velocity[i, j - 1] + grid.velocity[i, j + 1] -
                                    4 * velocity;

                grid.velocity[i, j] += viscosityCoefficient * laplacian * dt;
            }
        }
    }

    void SolvePressure()
    {
        int iterations = 60; // Number of iterations for Gauss-Seidel solver
        float alpha = 1.0f;
        float beta = 0.25f;

        // Initialize the pressure field with zero
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                grid.pressure[i, j] = 0f;
            }
        }

        for (int k = 0; k < iterations; k++)
        {
            for (int i = 1; i < grid.width - 1; i++)
            {
                for (int j = 1; j < grid.height - 1; j++)
                {
                    // Calculate divergence
                    float divergence = (grid.velocity[i + 1, j].x - grid.velocity[i, j].x +
                                        grid.velocity[i, j + 1].y - grid.velocity[i, j].y);

                    // Apply solid boundary conditions (s = 0 for solids, 1 for fluid)
                    float s = (IsSolid(i - 1, j) ? 0f : 1f) +
                              (IsSolid(i + 1, j) ? 0f : 1f) +
                              (IsSolid(i, j - 1) ? 0f : 1f) +
                              (IsSolid(i, j + 1) ? 0f : 1f);

                    if (s > 0)
                    {
                        // Gauss-Seidel iteration
                        grid.pressure[i, j] = (divergence - alpha * (GetPressure(i - 1, j) + GetPressure(i + 1, j) +
                                                                     GetPressure(i, j - 1) + GetPressure(i, j + 1))) / s;

                        // Update velocities based on the pressure
                        grid.velocity[i, j].x += divergence * (IsSolid(i - 1, j) ? 0f : 1f) / s;
                        grid.velocity[i + 1, j].x -= divergence * (IsSolid(i + 1, j) ? 0f : 1f) / s;
                        grid.velocity[i, j].y += divergence * (IsSolid(i, j - 1) ? 0f : 1f) / s;
                        grid.velocity[i, j + 1].y -= divergence * (IsSolid(i, j + 1) ? 0f : 1f) / s;
                    }
                }
            }
        }
    }

    float GetPressure(int i, int j)
    {
        if (i < 0 || i >= grid.width || j < 0 || j >= grid.height)
        {
            return 0f; // Assume solid boundaries outside the grid
        }
        return grid.pressure[i, j];
    }

    bool IsSolid(int i, int j)
    {
        // Assume no solid boundaries in this basic implementation
        // You can modify this method to add solid cells if needed
        return false;
    }

    void UpdateParticleVelocitiesFromGrid()
    {
        foreach (Particle particle in particles)
        {
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            // Interpolation weights
            float tx = (particle.position.x / grid.cellSize) - x0;
            float ty = (particle.position.y / grid.cellSize) - y0;

            float w00 = (1 - tx) * (1 - ty);
            float w10 = tx * (1 - ty);
            float w01 = (1 - tx) * ty;
            float w11 = tx * ty;

            // Update particle velocity using bilinear interpolation of grid velocities
            Vector2 newVelocity = Vector2.zero;

            if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height)
            {
                newVelocity += grid.velocity[x0, y0] * w00;
            }
            if (x1 >= 0 && x1 < grid.width && y0 >= 0 && y0 < grid.height)
            {
                newVelocity += grid.velocity[x1, y0] * w10;
            }
            if (x0 >= 0 && x0 < grid.width && y1 >= 0 && y1 < grid.height)
            {
                newVelocity += grid.velocity[x0, y1] * w01;
            }
            if (x1 >= 0 && x1 < grid.width && y1 >= 0 && y1 < grid.height)
            {
                newVelocity += grid.velocity[x1, y1] * w11;
            }

            particle.velocity = newVelocity;
        }
    }

    void AdvectParticles(float dt)
    {
        foreach (Particle particle in particles)
        {
            particle.position += particle.velocity * dt;

            // Clamp the particle velocity to prevent instability
            if (particle.velocity.magnitude > maxVelocity)
            {
                particle.velocity = particle.velocity.normalized * maxVelocity;
            }
        }
    }

    void EnforceBoundaries()
    {
        foreach (Particle particle in particles)
        {
            // Reflect particles off the left and right boundaries
            if (particle.position.x < 0)
            {
                particle.position.x = 0;
                particle.velocity.x *= -1;
            }
            else if (particle.position.x > gridWidth * cellSize)
            {
                particle.position.x = gridWidth * cellSize;
                particle.velocity.x *= -1;
            }

            // Reflect particles off the top and bottom boundaries
            if (particle.position.y < 0)
            {
                particle.position.y = 0;
                particle.velocity.y *= -1;
            }
            else if (particle.position.y > gridHeight * cellSize)
            {
                particle.position.y = gridHeight * cellSize;
                particle.velocity.y *= -1;
            }
        }
    }

    void ResolveParticleOverlap(float minDistance)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            for (int j = i + 1; j < particles.Count; j++)
            {
                Vector2 direction = particles[i].position - particles[j].position;
                float distance = direction.magnitude;

                if (distance < minDistance)
                {
                    Vector2 correction = direction.normalized * (minDistance - distance) * correctionStrength;
                    particles[i].position += correction;
                    particles[j].position -= correction;
                }
            }
        }
    }

    void DampenParticleVelocities()
    {
        foreach (Particle particle in particles)
        {
            particle.velocity *= dampingFactor; // Apply damping to reduce energy
        }
    }

    void OnDrawGizmos()
    {
        // Draw particles
        Gizmos.color = Color.blue;
        if (particles != null)
        {
            foreach (Particle particle in particles)
            {
                Gizmos.DrawSphere(particle.position, 0.1f);
            }
        }

        // Draw grid
        if (showGrid)
        {
            Gizmos.color = Color.gray;
            for (int i = 0; i <= grid.width; i++)
            {
                Gizmos.DrawLine(new Vector3(i * grid.cellSize, 0, 0), new Vector3(i * grid.cellSize, grid.height * grid.cellSize, 0));
            }
            for (int j = 0; j <= grid.height; j++)
            {
                Gizmos.DrawLine(new Vector3(0, j * grid.cellSize, 0), new Vector3(grid.width * grid.cellSize, j * grid.cellSize, 0));
            }
        }

        // Draw grid boundaries
        if (showBounds)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(new Vector3(0, 0, 0), new Vector3(grid.width * grid.cellSize, 0, 0));
            Gizmos.DrawLine(new Vector3(0, 0, 0), new Vector3(0, grid.height * grid.cellSize, 0));
            Gizmos.DrawLine(new Vector3(grid.width * grid.cellSize, 0, 0), new Vector3(grid.width * grid.cellSize, grid.height * grid.cellSize, 0));
            Gizmos.DrawLine(new Vector3(0, grid.height * grid.cellSize, 0), new Vector3(grid.width * grid.cellSize, grid.height * grid.cellSize, 0));
        }

        // Draw velocity field
        if (showVelocityField && grid != null && grid.velocity != null)
        {
            Gizmos.color = Color.red;
            for (int i = 0; i < grid.width; i++)
            {
                for (int j = 0; j < grid.height; j++)
                {
                    Vector3 cellCenter = new Vector3(i * grid.cellSize + grid.cellSize / 2, j * grid.cellSize + grid.cellSize / 2, 0);
                    Vector3 velocity = new Vector3(grid.velocity[i, j].x, grid.velocity[i, j].y, 0);
                    Gizmos.DrawLine(cellCenter, cellCenter + velocity * 0.5f); // Scale the velocity for better visualization
                }
            }
        }
    }
}

public class Grid
{
    public Vector2[,] velocity;
    public float[,] pressure;
    public float[,] weights; // New array to store the weights

    public int width, height;
    public float cellSize;

    public Grid(int width, int height, float cellSize)
    {
        this.width = width;
        this.height = height;
        this.cellSize = cellSize;
        velocity = new Vector2[width, height];
        pressure = new float[width, height];
        weights = new float[width, height]; // Initialize the weights array
    }
}

public class Particle
{
    public Vector2 position;
    public Vector2 velocity;

    public Particle(Vector2 position, Vector2 velocity)
    {
        this.position = position;
        this.velocity = velocity;
    }
}
