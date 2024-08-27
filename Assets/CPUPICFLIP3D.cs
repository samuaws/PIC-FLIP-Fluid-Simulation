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
    public float correctionStrength = 0.25f;

    public float desiredDensity = 1.0f; // Desired uniform density to enforce

    public bool showGrid = true;
    public bool showBounds = true;
    public bool showVelocityField = true;

    private Grid3D grid;
    public List<Particle3D> particles;

    void Start()
    {
        grid = new Grid3D(gridWidth, gridHeight, gridDepth, cellSize);
        particles = new List<Particle3D>();

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

        InitBuffer();
        InterlockedAddWeight();
        SetNeighborGridTypes();
        CalcGridWeight();

        CalculateDensity(); // Calculate density before solving pressure

        CorrectVelocitiesForDensity(); // Adjust velocities to enforce uniform density

        TransferParticleVelocitiesToGrid();
        ApplyGravityToGrid(dt);
        SolvePressure();
        UpdateParticleVelocitiesFromGrid();
        ProjectDensity();
        CalcPositionModify();
        UpdatePosition();
        AdvectParticles(dt);
        EnforceBoundaries();
        ResolveParticleOverlap(cellSize * 0.5f);
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

    void InitBuffer()
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.gridTypes[i, j, k] = GridType3D.Air;
                    grid.gridUIntWeights[i, j, k] = 0;
                }
            }
        }
    }

    void InterlockedAddWeight()
    {
        foreach (Particle3D particle in particles)
        {
            int x = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int z = Mathf.FloorToInt(particle.position.z / grid.cellSize);

            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
                        int nz = z + dz;

                        if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height && nz >= 0 && nz < grid.depth)
                        {
                            float weight = GetWeight(particle.position, new Vector3(nx * grid.cellSize, ny * grid.cellSize, nz * grid.cellSize), 1.0f / grid.cellSize);
                            grid.gridUIntWeights[nx, ny, nz] += ConvertFloatToUInt(weight);
                        }
                    }
                }
            }
        }
    }

    void SetNeighborGridTypes()
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.gridTypes[i, j, k] = GridType3D.Air;
                }
            }
        }
    }

    void CalcGridWeight()
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    float sumWeight = 0;

                    if (grid.gridTypes[i, j, k] == GridType3D.Fluid)
                    {
                        bool neighborAir = false;

                        for (int dx = -1; dx <= 1; dx++)
                        {
                            for (int dy = -1; dy <= 1; dy++)
                            {
                                for (int dz = -1; dz <= 1; dz++)
                                {
                                    int nx = i + dx;
                                    int ny = j + dy;
                                    int nz = k + dz;

                                    if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height && nz >= 0 && nz < grid.depth)
                                    {
                                        GridType3D neighborType = grid.gridTypes[nx, ny, nz];

                                        if (neighborType == GridType3D.Air)
                                        {
                                            neighborAir = true;
                                        }

                                        if (neighborType == GridType3D.Solid)
                                        {
                                            int distance = Mathf.Abs(dx) + Mathf.Abs(dy) + Mathf.Abs(dz);
                                            sumWeight += distance == 1 ? grid.ghostWeight.x : grid.ghostWeight.y;
                                        }
                                    }
                                }
                            }
                        }

                        if (!neighborAir)
                        {
                            sumWeight += ConvertUIntToFloat(grid.gridUIntWeights[i, j, k]);
                            sumWeight *= grid.invAverageWeight;
                            sumWeight = Mathf.Clamp(sumWeight, 0.5f, 1.5f);
                            sumWeight -= 1.0f;
                        }
                        else
                        {
                            sumWeight = 0;
                        }
                    }

                    grid.gridWeights[i, j, k] = sumWeight;
                }
            }
        }
    }

    void CalculateDensity()
    {
        // Reset density field
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.density[i, j, k] = 0f;
                }
            }
        }

        // Accumulate particle contributions to grid cells
        foreach (Particle3D particle in particles)
        {
            int x = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int z = Mathf.FloorToInt(particle.position.z / grid.cellSize);

            if (x >= 0 && x < grid.width && y >= 0 && y < grid.height && z >= 0 && z < grid.depth)
            {
                grid.density[x, y, z] += 1f; // Each particle contributes to the density
            }
        }

        // Normalize the density to account for grid cell size
        float cellVolume = grid.cellSize * grid.cellSize * grid.cellSize;
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.density[i, j, k] /= cellVolume;
                }
            }
        }
    }

    void CorrectVelocitiesForDensity()
    {
        // Iterate over the grid and adjust velocities to correct density
        for (int i = 1; i < grid.width - 1; i++)
        {
            for (int j = 1; j < grid.height - 1; j++)
            {
                for (int k = 1; k < grid.depth - 1; k++)
                {
                    // Calculate the density error
                    float densityError = grid.density[i, j, k] - desiredDensity;

                    // Apply a correction to the velocity field based on the density error
                    if (Mathf.Abs(densityError) > 0.01f) // Apply correction only if error is significant
                    {
                        Vector3 correction = Vector3.zero;

                        // Adjust velocities based on the error to reduce the density
                        if (densityError > 0)
                        {
                            // Density too high, push particles outwards
                            correction = new Vector3(
                                (grid.velocity[i + 1, j, k].x - grid.velocity[i - 1, j, k].x),
                                (grid.velocity[i, j + 1, k].y - grid.velocity[i, j - 1, k].y),
                                (grid.velocity[i, j, k + 1].z - grid.velocity[i, j, k - 1].z)
                            );
                        }
                        else
                        {
                            // Density too low, pull particles inwards
                            correction = new Vector3(
                                (grid.velocity[i - 1, j, k].x - grid.velocity[i + 1, j, k].x),
                                (grid.velocity[i, j - 1, k].y - grid.velocity[i, j + 1, k].y),
                                (grid.velocity[i, j, k - 1].z - grid.velocity[i, j, k + 1].z)
                            );
                        }

                        // Apply correction to the velocity field
                        grid.velocity[i, j, k] += correction * 0.1f; // The 0.1f is a damping factor to avoid instability
                    }
                }
            }
        }
    }

    void ProjectDensity()
    {
        for (int k = 0; k < 60; k++)
        {
            for (int i = 1; i < grid.width - 1; i++)
            {
                for (int j = 1; j < grid.height - 1; j++)
                {
                    for (int z = 1; z < grid.depth - 1; z++)
                    {
                        float pressure = 0;

                        if (grid.gridTypes[i, j, z] == GridType3D.Fluid)
                        {
                            int ixp = i - 1;
                            int ixn = i + 1;
                            int jyp = j - 1;
                            int jyn = j + 1;
                            int zzp = z - 1;
                            int zzn = z + 1;

                            pressure += grid.densityProjectionParam1.x * (grid.gridPressure[ixp, j, z] + grid.gridPressure[ixn, j, z]);
                            pressure += grid.densityProjectionParam1.y * (grid.gridPressure[i, jyp, z] + grid.gridPressure[i, jyn, z]);
                            pressure += grid.densityProjectionParam1.z * (grid.gridPressure[i, j, zzp] + grid.gridPressure[i, j, zzn]);
                            pressure += grid.densityProjectionParam1.w * grid.gridWeights[i, j, z];
                        }

                        grid.gridPressure[i, j, z] = pressure;
                    }
                }
            }
        }
    }

    void CalcPositionModify()
    {
        for (int i = 1; i < grid.width - 1; i++)
        {
            for (int j = 1; j < grid.height - 1; j++)
            {
                for (int z = 1; z < grid.depth - 1; z++)
                {
                    if (grid.gridTypes[i, j, z] == GridType3D.Fluid)
                    {
                        int ixp = i - 1;
                        int ixn = i + 1;
                        int jyp = j - 1;
                        int jyn = j + 1;
                        int zzp = z - 1;
                        int zzn = z + 1;

                        float p = grid.gridPressure[i, j, z];

                        Vector3 delPos = Vector3.zero;
                        delPos.x = grid.densityProjectionParam2.x * (p - grid.gridPressure[ixp, j, z]);
                        delPos.y = grid.densityProjectionParam2.y * (p - grid.gridPressure[i, jyp, z]);
                        delPos.z = grid.densityProjectionParam2.z * (p - grid.gridPressure[i, j, zzp]);

                        grid.gridPositionModify[i, j, z] = delPos;
                    }
                }
            }
        }
    }

    void UpdatePosition()
    {
        foreach (Particle3D particle in particles)
        {
            Vector3 newPosition = particle.position + SampleGridParam(particle.position, grid.gridPositionModify);
            newPosition = ClampPosition(newPosition, Vector3.zero, new Vector3(grid.width * grid.cellSize, grid.height * grid.cellSize, grid.depth * grid.cellSize));
            ClampPositionByObstacles(ref newPosition);

            particle.position = newPosition;
        }
    }

    void TransferParticleVelocitiesToGrid()
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                for (int k = 0; k < grid.depth; k++)
                {
                    grid.velocity[i, j, k] = Vector3.zero;
                    grid.weights[i, j, k] = 0f;
                }
            }
        }

        foreach (Particle3D particle in particles)
        {
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int z0 = Mathf.FloorToInt(particle.position.z / grid.cellSize);
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;

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
            if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                grid.velocity[x0, y0, z0] += particle.velocity * w000;
                grid.weights[x0, y0, z0] += w000;
            }
            if (x1 >= 0 && x1 < grid.width && y0 >= 0 && y0 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                grid.velocity[x1, y0, z0] += particle.velocity * w100;
                grid.weights[x1, y0, z0] += w100;
            }
            if (x0 >= 0 && x0 < grid.width && y1 >= 0 && y1 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                grid.velocity[x0, y1, z0] += particle.velocity * w010;
                grid.weights[x0, y1, z0] += w010;
            }
            if (x1 >= 0 && x1 < grid.width && y1 >= 0 && y1 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                grid.velocity[x1, y1, z0] += particle.velocity * w110;
                grid.weights[x1, y1, z0] += w110;
            }
            if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                grid.velocity[x0, y0, z1] += particle.velocity * w001;
                grid.weights[x0, y0, z1] += w001;
            }
            if (x1 >= 0 && x1 < grid.width && y0 >= 0 && y0 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                grid.velocity[x1, y0, z1] += particle.velocity * w101;
                grid.weights[x1, y0, z1] += w101;
            }
            if (x0 >= 0 && x0 < grid.width && y1 >= 0 && y1 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                grid.velocity[x0, y1, z1] += particle.velocity * w011;
                grid.weights[x0, y1, z1] += w011;
            }
            if (x1 >= 0 && x1 < grid.width && y1 >= 0 && y1 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                grid.velocity[x1, y1, z1] += particle.velocity * w111;
                grid.weights[x1, y1, z1] += w111;
            }
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
        float alpha = 1.0f;
        float beta = 0.25f;

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
                        if (grid.gridTypes[i, j, k] == GridType3D.Fluid)
                        {
                            float divergence = ComputeDivergence(i, j, k);

                            // Use density to determine the correction
                            float s = grid.density[i, j, k];
                            if (s > 0)
                            {
                                float pressureCorrection = -divergence / s;
                                grid.pressure[i, j, k] += alpha * pressureCorrection;

                                // Update the velocities based on the pressure gradient
                                ApplyPressureGradient(i, j, k, pressureCorrection);
                            }
                        }
                    }
                }
            }
        }
    }

    float ComputeDivergence(int i, int j, int k)
    {
        float div = 0f;

        div += grid.velocity[i + 1, j, k].x - grid.velocity[i, j, k].x;
        div += grid.velocity[i, j + 1, k].y - grid.velocity[i, j, k].y;
        div += grid.velocity[i, j, k + 1].z - grid.velocity[i, j, k].z;

        return div;
    }

    void ApplyPressureGradient(int i, int j, int k, float pressureCorrection)
    {
        if (!IsSolid(i - 1, j, k))
            grid.velocity[i, j, k].x -= pressureCorrection;
        if (!IsSolid(i + 1, j, k))
            grid.velocity[i + 1, j, k].x += pressureCorrection;

        if (!IsSolid(i, j - 1, k))
            grid.velocity[i, j, k].y -= pressureCorrection;
        if (!IsSolid(i, j + 1, k))
            grid.velocity[i, j + 1, k].y += pressureCorrection;

        if (!IsSolid(i, j, k - 1))
            grid.velocity[i, j, k].z -= pressureCorrection;
        if (!IsSolid(i, j, k + 1))
            grid.velocity[i, j, k + 1].z += pressureCorrection;
    }

    bool IsSolid(int i, int j, int k)
    {
        return false;
    }

    void UpdateParticleVelocitiesFromGrid()
    {
        foreach (Particle3D particle in particles)
        {
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int z0 = Mathf.FloorToInt(particle.position.z / grid.cellSize);
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;

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

            Vector3 newVelocity = Vector3.zero;

            if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                newVelocity += grid.velocity[x0, y0, z0] * w000;
            }
            if (x1 >= 0 && x1 < grid.width && y0 >= 0 && y0 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                newVelocity += grid.velocity[x1, y0, z0] * w100;
            }
            if (x0 >= 0 && x0 < grid.width && y1 >= 0 && y1 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                newVelocity += grid.velocity[x0, y1, z0] * w010;
            }
            if (x1 >= 0 && x1 < grid.width && y1 >= 0 && y1 < grid.height && z0 >= 0 && z0 < grid.depth)
            {
                newVelocity += grid.velocity[x1, y1, z0] * w110;
            }
            if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                newVelocity += grid.velocity[x0, y0, z1] * w001;
            }
            if (x1 >= 0 && x1 < grid.width && y0 >= 0 && y0 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                newVelocity += grid.velocity[x1, y0, z1] * w101;
            }
            if (x0 >= 0 && x0 < grid.width && y1 >= 0 && y1 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                newVelocity += grid.velocity[x0, y1, z1] * w011;
            }
            if (x1 >= 0 && x1 < grid.width && y1 >= 0 && y1 < grid.height && z1 >= 0 && z1 < grid.depth)
            {
                newVelocity += grid.velocity[x1, y1, z1] * w111;
            }

            particle.velocity = newVelocity;
        }
    }


    void AdvectParticles(float dt)
    {
        foreach (Particle3D particle in particles)
        {
            particle.position += particle.velocity * dt;

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
                particle.velocity.x *= -1;
            }
            else if (particle.position.x > gridWidth * cellSize)
            {
                particle.position.x = gridWidth * cellSize;
                particle.velocity.x *= -1;
            }

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

            if (particle.position.z < 0)
            {
                particle.position.z = 0;
                particle.velocity.z *= -1;
            }
            else if (particle.position.z > gridDepth * cellSize)
            {
                particle.position.z = gridDepth * cellSize;
                particle.velocity.z *= -1;
            }
        }
    }

    void ResolveParticleOverlap(float minDistance)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            for (int j = i + 1; j < particles.Count; j++)
            {
                Vector3 direction = particles[i].position - particles[j].position;
                float distance = direction.magnitude;

                if (distance < minDistance)
                {
                    Vector3 correction = direction.normalized * (minDistance - distance) * correctionStrength;
                    particles[i].position += correction;
                    particles[j].position -= correction;
                }
            }
        }
    }

    Vector3 SampleGridParam(Vector3 position, Vector3[,,] gridParam)
    {
        int x = Mathf.FloorToInt(position.x / grid.cellSize);
        int y = Mathf.FloorToInt(position.y / grid.cellSize);
        int z = Mathf.FloorToInt(position.z / grid.cellSize);

        x = Mathf.Clamp(x, 0, grid.width - 1);
        y = Mathf.Clamp(y, 0, grid.height - 1);
        z = Mathf.Clamp(z, 0, grid.depth - 1);

        return gridParam[x, y, z];
    }

    Vector3 ClampPosition(Vector3 position, Vector3 min, Vector3 max)
    {
        return new Vector3(
            Mathf.Clamp(position.x, min.x, max.x),
            Mathf.Clamp(position.y, min.y, max.y),
            Mathf.Clamp(position.z, min.z, max.z));
    }

    void ClampPositionByObstacles(ref Vector3 position)
    {
        // Implement obstacle clamping logic if needed
    }

    float ConvertUIntToFloat(int value)
    {
        return value * (1.0f / 16777216f); // 2^-24
    }

    int ConvertFloatToUInt(float value)
    {
        return Mathf.RoundToInt(value * 16777216f); // 2^24
    }

    float GetWeight(Vector3 pos, Vector3 cellPos, float invSpacing)
    {
        return Mathf.Exp(-Mathf.Pow((pos - cellPos).magnitude * invSpacing, 2));
    }
}

public class Grid3D
{
    public Vector3[,,] velocity;
    public float[,,] pressure;
    public float[,,] weights;
    public GridType3D[,,] gridTypes;
    public int[,,] gridUIntWeights;
    public float[,,] gridWeights;
    public float[,,] gridPressure;
    public Vector3[,,] gridPositionModify;
    public float[,,] density; // Density field

    public int width, height, depth;
    public float cellSize;

    public Vector2 ghostWeight = new Vector2(0.5f, 0.25f);
    public float invAverageWeight = 0.5f;
    public Vector4 densityProjectionParam1 = new Vector4(0.25f, 0.25f, 0.25f, 0.25f);
    public Vector3 densityProjectionParam2 = new Vector3(0.5f, 0.5f, 0.5f);

    public Grid3D(int width, int height, int depth, float cellSize)
    {
        this.width = width;
        this.height = height;
        this.depth = depth;
        this.cellSize = cellSize;
        velocity = new Vector3[width, height, depth];
        pressure = new float[width, height, depth];
        weights = new float[width, height, depth];
        gridTypes = new GridType3D[width, height, depth];
        gridUIntWeights = new int[width, height, depth];
        gridWeights = new float[width, height, depth];
        gridPressure = new float[width, height, depth];
        gridPositionModify = new Vector3[width, height, depth];
        density = new float[width, height, depth]; // Initialize density field
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

public enum GridType3D
{
    Air,
    Solid,
    Fluid
}
