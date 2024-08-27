using UnityEngine;
using System.Collections.Generic;

public class CPUPICFLIP : MonoBehaviour
{
    public int gridWidth = 20;
    public int gridHeight = 20;
    public float cellSize = 0.5f;
    public int numParticles = 100;
    public float viscosityCoefficient = 0.01f;
    public Vector2 gravity = new Vector2(0, -9.81f);
    public float maxVelocity = 10f;
    public float dampingFactor = 0.99f;
    public float maxAllowedTimeStep = 0.01f;
    public float correctionStrength = 0.25f;

    public bool showGrid = true;
    public bool showBounds = true;
    public bool showVelocityField = true;

    private Grid grid;
    public List<Particle> particles;

    void Start()
    {
        grid = new Grid(gridWidth, gridHeight, cellSize);
        particles = new List<Particle>();

        for (int i = 0; i < numParticles; i++)
        {
            Vector2 randomPosition = new Vector2(Random.Range(0.1f, gridWidth * cellSize - 0.1f), Random.Range(0.1f, gridHeight * cellSize - 0.1f));
            Vector2 randomVelocity = new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f));
            particles.Add(new Particle(randomPosition, randomVelocity));
        }
    }

    void Update()
    {
        float dt = Mathf.Min(Time.deltaTime, maxAllowedTimeStep);

        InitBuffer();
        InterlockedAddWeight();
        SetNeighborGridTypes();
        CalcGridWeight();

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

    void InitBuffer()
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                var gridType = grid.gridTypes[i, j];
                grid.gridTypes[i, j] = GridType.Air;
                grid.gridUIntWeights[i, j] = 0;
            }
        }
    }

    void InterlockedAddWeight()
    {
        foreach (Particle particle in particles)
        {
            int x = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y = Mathf.FloorToInt(particle.position.y / grid.cellSize);


            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height)
                    {
                        float weight = GetWeight(particle.position, new Vector2(nx * grid.cellSize, ny * grid.cellSize), 1.0f / grid.cellSize);
                        grid.gridUIntWeights[nx, ny] += ConvertFloatToUInt(weight);
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
                GridType myType = grid.gridTypes[i, j];
                GridType xpType = (i == 0) ? GridType.Solid : grid.gridTypes[i - 1, j];
                GridType xnType = (i == grid.width - 1) ? GridType.Solid : grid.gridTypes[i + 1, j];
                GridType ypType = (j == 0) ? GridType.Solid : grid.gridTypes[i, j - 1];
                GridType ynType = (j == grid.height - 1) ? GridType.Solid : grid.gridTypes[i, j + 1];

                grid.gridTypes[i, j] = myType;
            }
        }
    }

    void CalcGridWeight()
    {
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                float sumWeight = 0;

                if (grid.gridTypes[i, j] == GridType.Fluid)
                {
                    bool neighborAir = false;

                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            int nx = i + dx;
                            int ny = j + dy;

                            if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height)
                            {
                                GridType neighborType = grid.gridTypes[nx, ny];

                                if (neighborType == GridType.Air)
                                {
                                    neighborAir = true;
                                }

                                if (neighborType == GridType.Solid)
                                {
                                    int distance = Mathf.Abs(dx) + Mathf.Abs(dy);
                                    sumWeight += distance == 1 ? grid.ghostWeight.x : grid.ghostWeight.y;
                                }
                            }
                        }
                    }

                    if (!neighborAir)
                    {
                        sumWeight += ConvertUIntToFloat(grid.gridUIntWeights[i, j]);
                        sumWeight *= grid.invAverageWeight;
                        sumWeight = Mathf.Clamp(sumWeight, 0.5f, 1.5f);
                        sumWeight -= 1.0f;
                    }
                    else
                    {
                        sumWeight = 0;
                    }
                }

                grid.gridWeights[i, j] = sumWeight;
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
                    float pressure = 0;

                    if (grid.gridTypes[i, j] == GridType.Fluid)
                    {
                        int ixp = i - 1;
                        int ixn = i + 1;
                        int jyp = j - 1;
                        int jyn = j + 1;

                        pressure += grid.densityProjectionParam1.x * (grid.gridPressure[ixp, j] + grid.gridPressure[ixn, j]);
                        pressure += grid.densityProjectionParam1.y * (grid.gridPressure[i, jyp] + grid.gridPressure[i, jyn]);
                        pressure += grid.densityProjectionParam1.z * grid.gridWeights[i, j];
                    }

                    grid.gridPressure[i, j] = pressure;
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
                if (grid.gridTypes[i, j] == GridType.Fluid)
                {
                    int ixp = i - 1;
                    int ixn = i + 1;
                    int jyp = j - 1;
                    int jyn = j + 1;

                    float p = grid.gridPressure[i, j];

                    Vector2 delPos = Vector2.zero;
                    delPos.x = grid.densityProjectionParam2.x * (p - grid.gridPressure[ixp, j]);
                    delPos.y = grid.densityProjectionParam2.y * (p - grid.gridPressure[i, jyp]);

                    grid.gridPositionModify[i, j] = delPos;
                }
            }
        }
    }

    void UpdatePosition()
    {
        foreach (Particle particle in particles)
        {
            Vector2 newPosition = particle.position + SampleGridParam(particle.position, grid.gridPositionModify);
            newPosition = ClampPosition(newPosition, Vector2.zero, new Vector2(grid.width * grid.cellSize, grid.height * grid.cellSize));
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
                grid.velocity[i, j] = Vector2.zero;
                grid.weights[i, j] = 0f;
            }
        }

        foreach (Particle particle in particles)
        {
            int x0 = Mathf.FloorToInt(particle.position.x / grid.cellSize);
            int y0 = Mathf.FloorToInt(particle.position.y / grid.cellSize);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            float tx = (particle.position.x / grid.cellSize) - x0;
            float ty = (particle.position.y / grid.cellSize) - y0;

            float w00 = (1 - tx) * (1 - ty);
            float w10 = tx * (1 - ty);
            float w01 = (1 - tx) * ty;
            float w11 = tx * ty;

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
        for (int i = 0; i < grid.width; i++)
        {
            for (int j = 0; j < grid.height; j++)
            {
                grid.velocity[i, j] += gravity * dt;
            }
        }
    }

    void SolvePressure()
    {
        int iterations = 60;
        float alpha = 1.0f;
        float beta = 0.25f;

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
                    float divergence = (grid.velocity[i + 1, j].x - grid.velocity[i, j].x +
                                        grid.velocity[i, j + 1].y - grid.velocity[i, j].y);

                    float s = (IsSolid(i - 1, j) ? 0f : 1f) +
                              (IsSolid(i + 1, j) ? 0f : 1f) +
                              (IsSolid(i, j - 1) ? 0f : 1f) +
                              (IsSolid(i, j + 1) ? 0f : 1f);

                    if (s > 0)
                    {
                        grid.pressure[i, j] = (divergence - alpha * (GetPressure(i - 1, j) + GetPressure(i + 1, j) +
                                                                     GetPressure(i, j - 1) + GetPressure(i, j + 1))) / s;

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
            return 0f;
        }
        return grid.pressure[i, j];
    }

    bool IsSolid(int i, int j)
    {
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

            float tx = (particle.position.x / grid.cellSize) - x0;
            float ty = (particle.position.y / grid.cellSize) - y0;

            float w00 = (1 - tx) * (1 - ty);
            float w10 = tx * (1 - ty);
            float w01 = (1 - tx) * ty;
            float w11 = tx * ty;

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

    Vector2 SampleGridParam(Vector2 position, Vector2[,] gridParam)
    {
        int x = Mathf.FloorToInt(position.x / grid.cellSize);
        int y = Mathf.FloorToInt(position.y / grid.cellSize);

        x = Mathf.Clamp(x, 0, grid.width - 1);
        y = Mathf.Clamp(y, 0, grid.height - 1);

        return gridParam[x, y];
    }

    Vector2 ClampPosition(Vector2 position, Vector2 min, Vector2 max)
    {
        return new Vector2(Mathf.Clamp(position.x, min.x, max.x), Mathf.Clamp(position.y, min.y, max.y));
    }

    void ClampPositionByObstacles(ref Vector2 position)
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

    float GetWeight(Vector2 pos, Vector2 cellPos, float invSpacing)
    {
        return Mathf.Exp(-Mathf.Pow((pos - cellPos).magnitude * invSpacing, 2));
    }
}

public class Grid
{
    public Vector2[,] velocity;
    public float[,] pressure;
    public float[,] weights;
    public GridType[,] gridTypes;
    public int[,] gridUIntWeights;
    public float[,] gridWeights;
    public float[,] gridPressure;
    public Vector2[,] gridPositionModify;

    public int width, height;
    public float cellSize;

    public Vector2 ghostWeight = new Vector2(0.5f, 0.25f);
    public float invAverageWeight = 0.5f;
    public Vector4 densityProjectionParam1 = new Vector4(0.25f, 0.25f, 0.25f, 0.25f);
    public Vector2 densityProjectionParam2 = new Vector2(0.5f, 0.5f);

    public Grid(int width, int height, float cellSize)
    {
        this.width = width;
        this.height = height;
        this.cellSize = cellSize;
        velocity = new Vector2[width, height];
        pressure = new float[width, height];
        weights = new float[width, height];
        gridTypes = new GridType[width, height];
        gridUIntWeights = new int[width, height];
        gridWeights = new float[width, height];
        gridPressure = new float[width, height];
        gridPositionModify = new Vector2[width, height];
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

public enum GridType
{
    Air,
    Solid,
    Fluid
}

public struct GridTypeData
{
    public GridType MyType;
    public GridType XPrevType;
    public GridType XNextType;
    public GridType YPrevType;
    public GridType YNextType;

    public GridTypeData(GridType myType, GridType xPrevType, GridType xNextType, GridType yPrevType, GridType yNextType)
    {
        MyType = myType;
        XPrevType = xPrevType;
        XNextType = xNextType;
        YPrevType = yPrevType;
        YNextType = yNextType;
    }
}
