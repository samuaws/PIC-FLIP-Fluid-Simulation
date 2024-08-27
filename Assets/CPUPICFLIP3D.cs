using UnityEngine;
using System.Collections.Generic;

public class CPUPICFLIP3D : MonoBehaviour
{
    public int gridWidth = 20;
    public int gridHeight = 20;
    public int gridDepth = 20;
    public float cellSize = 0.5f;
    public int numParticles = 1000;
    public float viscosityCoefficient = 0.01f;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public float maxVelocity = 10f;
    public float dampingFactor = 0.99f;
    public float maxAllowedTimeStep = 0.01f;
    public float flipPicBlend = 0.95f;  // Blending factor between FLIP and PIC (1.0 is pure FLIP, 0.0 is pure PIC)

    public bool showGrid = true;
    public bool showBounds = true;
    public bool showVelocityField = true;

    private Grid3D grid;
    public List<Particle3D> particles;

    private Vector3[,,] previousGridVelocity;  // Store previous grid velocities for FLIP calculation

    void Start()
    {
        grid = new Grid3D(gridWidth, gridHeight, gridDepth, cellSize);
        particles = new List<Particle3D>();
        previousGridVelocity = new Vector3[gridWidth, gridHeight, gridDepth];

        for (int i = 0; i < numParticles; i++)
        {
            Vector3 randomPosition = new Vector3(
                Random.Range(0.1f, gridWidth * cellSize - 0.1f),
                Random.Range(0.1f, gridHeight * cellSize - 0.1f),
                Random.Range(0.1f, gridDepth * cellSize - 0.1f));
            Vector3 randomVelocity = new Vector3(Random.Range(-1f, 1f), Random.Range(-1f, 1f), Random.Range(-1f, 1f));
            particles.Add(new Particle3D(randomPosition, randomVelocity));
        }
    }

    void Update()
    {
        float dt = Mathf.Min(Time.deltaTime, maxAllowedTimeStep);

        TransferParticleVelocitiesToGrid();  // Transfer particle velocities to the grid
        ApplyGravityToGrid(dt);  // Apply gravity to the grid
        SolvePressure();  // Solve for pressure to ensure incompressibility
        UpdateParticleVelocitiesFromGrid();  // Update particle velocities from the grid (using FLIP and PIC)
        AdvectParticles(dt);  // Move particles based on their velocities
        EnforceBoundaries();  // Enforce boundary conditions on the particles
    }

    void OnDrawGizmos()
    {
        // Draw particles
        Gizmos.color = Color.blue;
        if (particles != null)
        {
            foreach (Particle3D particle in particles)
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
                for (int j = 0; j <= grid.height; j++)
                {
                    for (int k = 0; k <= grid.depth; k++)
                    {
                        Gizmos.DrawLine(new Vector3(i * grid.cellSize, j * grid.cellSize, 0), new Vector3(i * grid.cellSize, j * grid.cellSize, grid.depth * grid.cellSize));
                        Gizmos.DrawLine(new Vector3(i * grid.cellSize, 0, k * grid.cellSize), new Vector3(i * grid.cellSize, grid.height * grid.cellSize, k * grid.cellSize));
                        Gizmos.DrawLine(new Vector3(0, j * grid.cellSize, k * grid.cellSize), new Vector3(grid.width * grid.cellSize, j * grid.cellSize, k * grid.cellSize));
                    }
                }
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

            Gizmos.DrawLine(new Vector3(0, 0, 0), new Vector3(0, 0, grid.depth * grid.cellSize));
            Gizmos.DrawLine(new Vector3(grid.width * grid.cellSize, 0, 0), new Vector3(grid.width * grid.cellSize, 0, grid.depth * grid.cellSize));
            Gizmos.DrawLine(new Vector3(grid.width * grid.cellSize, grid.height * grid.cellSize, 0), new Vector3(grid.width * grid.cellSize, grid.height * grid.cellSize, grid.depth * grid.cellSize));
            Gizmos.DrawLine(new Vector3(0, grid.height * grid.cellSize, 0), new Vector3(0, grid.height * grid.cellSize, grid.depth * grid.cellSize));

            Gizmos.DrawLine(new Vector3(0, 0, grid.depth * grid.cellSize), new Vector3(grid.width * grid.cellSize, 0, grid.depth * grid.cellSize));
            Gizmos.DrawLine(new Vector3(0, grid.height * grid.cellSize, grid.depth * grid.cellSize), new Vector3(grid.width * grid.cellSize, grid.height * grid.cellSize, grid.depth * grid.cellSize));
        }

        // Draw velocity field
        if (showVelocityField && grid != null && grid.velocity != null)
        {
            Gizmos.color = Color.red;
            for (int i = 0; i < grid.width; i++)
            {
                for (int j = 0; j < grid.height; j++)
                {
                    for (int k = 0; k < grid.depth; k++)
                    {
                        Vector3 cellCenter = new Vector3(i * grid.cellSize + grid.cellSize / 2, j * grid.cellSize + grid.cellSize / 2, k * grid.cellSize + grid.cellSize / 2);
                        Vector3 velocity = grid.velocity[i, j, k];
                        Gizmos.DrawLine(cellCenter, cellCenter + velocity * 0.5f); // Scale the velocity for better visualization
                    }
                }
            }
        }
    }

    void TransferParticleVelocitiesToGrid()
    {
        // Store previous grid velocities for FLIP calculations
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    previousGridVelocity[i, j, k] = grid.velocity[i, j, k];
                    grid.velocity[i, j, k] = Vector3.zero;
                    grid.weights[i, j, k] = 0f;
                }
            }
        }

        // Transfer velocities from particles to grid using a weighted sum
        foreach (Particle3D particle in particles)
        {
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int z0 = Mathf.FloorToInt(particle.position.z / grid.cellSize);

            // Boundary check to avoid out-of-bounds access
            if (x0 < 0 || x0 >= grid.width || y0 < 0 || y0 >= grid.height || z0 < 0 || z0 >= grid.depth)
                continue;

            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;

            // Ensure the upper indices (x1, y1, z1) are within bounds
            if (x1 >= grid.width || y1 >= grid.height || z1 >= grid.depth)
                continue;

            float tx = (particle.position.x / grid.cellSize) - x0;
            float ty = (particle.position.y / grid.cellSize) - y0;
            float tz = (particle.position.z / grid.cellSize) - z0;

            float w000 = (1 - tx) * (1 - ty) * (1 - tz);
            float w100 = tx * (1 - ty) * (1 - tz);
            float w010 = (1 - tx) * ty * (1 - tz);
            float w110 = tx * ty * (1 - tz);
            float w001 = (1 - tx) * (1 - ty) * tz;
            float w101 = tx * (1 - ty) * tz;
            float w011 = (1 - tx) * ty * tz;
            float w111 = tx * ty * tz;

            // Ensure indices are within bounds before accessing the grid arrays
            grid.velocity[x0, y0, z0] += particle.velocity * w000;
            grid.weights[x0, y0, z0] += w000;

            grid.velocity[x1, y0, z0] += particle.velocity * w100;
            grid.weights[x1, y0, z0] += w100;

            grid.velocity[x0, y1, z0] += particle.velocity * w010;
            grid.weights[x0, y1, z0] += w010;

            grid.velocity[x1, y1, z0] += particle.velocity * w110;
            grid.weights[x1, y1, z0] += w110;

            grid.velocity[x0, y0, z1] += particle.velocity * w001;
            grid.weights[x0, y0, z1] += w001;

            grid.velocity[x1, y0, z1] += particle.velocity * w101;
            grid.weights[x1, y0, z1] += w101;

            grid.velocity[x0, y1, z1] += particle.velocity * w011;
            grid.weights[x0, y1, z1] += w011;

            grid.velocity[x1, y1, z1] += particle.velocity * w111;
            grid.weights[x1, y1, z1] += w111;
        }

        // Normalize velocities by the accumulated weights
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    if (grid.weights[i, j, k] > 0f)
                    {
                        grid.velocity[i, j, k] /= grid.weights[i, j, k];
                    }
                }
            }
        }
    }

    void ApplyGravityToGrid(float dt)
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.velocity[i, j, k] += gravity * dt;
                }
            }
        }
    }

    void SolvePressure()
    {
        int iterations = 60; // Number of iterations for pressure solver
        float alpha = -1.0f;  // This constant can be adjusted based on grid resolution and time step size
        float beta = 1.0f;  // Regularization term to prevent over-correction

        // Initialize pressure to zero
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.pressure[i, j, k] = 0f;
                }
            }
        }

        // Pressure solve iterations
        for (int l = 0; l < iterations; l++)
        {
            for (int i = 1; i < grid.width - 1; i++)
            {
                for (int j = 1; j < grid.height - 1; j++)
                {
                    for (int k = 1; k < grid.depth - 1; k++)
                    {
                        if (grid.weights[i, j, k] > 0f)
                        {
                            float divergence = ComputeDivergence(i, j, k);

                            float pressureCorrection = -divergence * alpha / (grid.weights[i, j, k] + beta);
                            grid.pressure[i, j, k] += pressureCorrection;

                            ApplyPressureGradient(i, j, k, pressureCorrection);
                        }
                    }
                }
            }
        }
    }

    float ComputeDivergence(int i, int j, int k)
    {
        float div = 0f;

        // Ensure valid indices for divergence computation
        if (i + 1 < grid.width)
            div += grid.velocity[i + 1, j, k].x - grid.velocity[i, j, k].x;

        if (j + 1 < grid.height)
            div += grid.velocity[i, j + 1, k].y - grid.velocity[i, j, k].y;

        if (k + 1 < grid.depth)
            div += grid.velocity[i, j, k + 1].z - grid.velocity[i, j, k].z;

        return div;
    }

    void ApplyPressureGradient(int i, int j, int k, float pressureCorrection)
    {
        if (i > 0 && i < grid.width - 1)
        {
            grid.velocity[i, j, k].x -= pressureCorrection;
            grid.velocity[i + 1, j, k].x += pressureCorrection;
        }

        if (j > 0 && j < grid.height - 1)
        {
            grid.velocity[i, j, k].y -= pressureCorrection;
            grid.velocity[i, j + 1, k].y += pressureCorrection;
        }

        if (k > 0 && k < grid.depth - 1)
        {
            grid.velocity[i, j, k].z -= pressureCorrection;
            grid.velocity[i, j, k + 1].z += pressureCorrection;
        }
    }

    void UpdateParticleVelocitiesFromGrid()
    {
        foreach (Particle3D particle in particles)
        {
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int z0 = Mathf.FloorToInt(particle.position.z / grid.cellSize);

            // Boundary check to avoid out-of-bounds access
            if (x0 < 0 || x0 >= grid.width || y0 < 0 || y0 >= grid.height || z0 < 0 || z0 >= grid.depth)
                continue;

            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;

            // Ensure the upper indices (x1, y1, z1) are within bounds
            if (x1 >= grid.width || y1 >= grid.height || z1 >= grid.depth)
                continue;

            float tx = (particle.position.x / grid.cellSize) - x0;
            float ty = (particle.position.y / grid.cellSize) - y0;
            float tz = (particle.position.z / grid.cellSize) - z0;

            float w000 = (1 - tx) * (1 - ty) * (1 - tz);
            float w100 = tx * (1 - ty) * (1 - tz);
            float w010 = (1 - tx) * ty * (1 - tz);
            float w110 = tx * ty * (1 - tz);
            float w001 = (1 - tx) * (1 - ty) * tz;
            float w101 = tx * (1 - ty) * tz;
            float w011 = (1 - tx) * ty * tz;
            float w111 = tx * ty * tz;

            Vector3 picVelocity = Vector3.zero;

            picVelocity += grid.velocity[x0, y0, z0] * w000;
            picVelocity += grid.velocity[x1, y0, z0] * w100;
            picVelocity += grid.velocity[x0, y1, z0] * w010;
            picVelocity += grid.velocity[x1, y1, z0] * w110;
            picVelocity += grid.velocity[x0, y0, z1] * w001;
            picVelocity += grid.velocity[x1, y0, z1] * w101;
            picVelocity += grid.velocity[x0, y1, z1] * w011;
            picVelocity += grid.velocity[x1, y1, z1] * w111;

            // FLIP Update: Calculate the difference between current and previous grid velocities
            Vector3 flipVelocity = particle.velocity;
            flipVelocity += (grid.velocity[x0, y0, z0] - previousGridVelocity[x0, y0, z0]) * w000;
            flipVelocity += (grid.velocity[x1, y0, z0] - previousGridVelocity[x1, y0, z0]) * w100;
            flipVelocity += (grid.velocity[x0, y1, z0] - previousGridVelocity[x0, y1, z0]) * w010;
            flipVelocity += (grid.velocity[x1, y1, z0] - previousGridVelocity[x1, y1, z0]) * w110;
            flipVelocity += (grid.velocity[x0, y0, z1] - previousGridVelocity[x0, y0, z1]) * w001;
            flipVelocity += (grid.velocity[x1, y0, z1] - previousGridVelocity[x1, y0, z1]) * w101;
            flipVelocity += (grid.velocity[x0, y1, z1] - previousGridVelocity[x0, y1, z1]) * w011;
            flipVelocity += (grid.velocity[x1, y1, z1] - previousGridVelocity[x1, y1, z1]) * w111;

            // Blending between PIC and FLIP velocities
            particle.velocity = (1 - flipPicBlend) * picVelocity + flipPicBlend * flipVelocity;
        }
    }

    void AdvectParticles(float dt)
    {
        foreach (Particle3D particle in particles)
        {
            particle.position += particle.velocity * dt;
            print(particle.velocity);

            if (particle.velocity.magnitude > maxVelocity)
            {
                particle.velocity = particle.velocity.normalized * maxVelocity;
            }
        }
    }

    void EnforceBoundaries()
    {
        foreach (Particle3D particle in particles)
        {
            if (particle.position.x < 0)
            {
                particle.position.x = 0;
                particle.velocity.x *= -dampingFactor;
            }
            else if (particle.position.x > gridWidth * cellSize)
            {
                particle.position.x = gridWidth * cellSize;
                particle.velocity.x *= -dampingFactor;
            }

            if (particle.position.y < 0)
            {
                particle.position.y = 0;
                particle.velocity.y *= -dampingFactor;
            }
            else if (particle.position.y > gridHeight * cellSize)
            {
                particle.position.y = gridHeight * cellSize;
                particle.velocity.y *= -dampingFactor;
            }

            if (particle.position.z < 0)
            {
                particle.position.z = 0;
                particle.velocity.z *= -dampingFactor;
            }
            else if (particle.position.z > gridDepth * cellSize)
            {
                particle.position.z = gridDepth * cellSize;
                particle.velocity.z *= -dampingFactor;
            }
        }
    }
}

public class Grid3D
{
    public Vector3[,,] velocity;
    public float[,,] pressure;
    public float[,,] weights;
    public int width, height, depth;
    public float cellSize;

    public Grid3D(int width, int height, int depth, float cellSize)
    {
        this.width = width;
        this.height = height;
        this.depth = depth;
        this.cellSize = cellSize;
        velocity = new Vector3[width, height, depth];
        pressure = new float[width, height, depth];
        weights = new float[width, height, depth];
    }
}

public class Particle3D
{
    public Vector3 position;
    public Vector3 velocity;

    public Particle3D(Vector3 position, Vector3 velocity)
    {
        this.position = position;
        this.velocity = velocity;
    }
}
