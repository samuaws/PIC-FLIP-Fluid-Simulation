using UnityEngine;

public class AdvancedPICFLIPSimulatioCPU : MonoBehaviour
{
    public int gridResolution = 128;
    public float cellSize = 1.0f / 128.0f;
    public int numParticles = 100000;
    public float particleRadius = 0.01f;
    public float timeStep = 0.01f;
    public float viscosity = 0.01f;

    private FluidParticleCPU[] particles;
    private Vector3[] velocityGrid;
    private Vector3[] newVelocityGrid;
    private float[] massGrid;
    private float[] divergenceGrid;
    private float[] pressureGrid;
    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    void Start()
    {
        InitializeGrids();
        InitializeParticles();
    }

    void InitializeGrids()
    {
        int gridSize = gridResolution * gridResolution * gridResolution;

        velocityGrid = new Vector3[gridSize];
        newVelocityGrid = new Vector3[gridSize];
        massGrid = new float[gridSize];
        divergenceGrid = new float[gridSize];
        pressureGrid = new float[gridSize];
    }

    void InitializeParticles()
    {
        particles = new FluidParticleCPU[numParticles];
        for (int i = 0; i < numParticles; i++)
        {
            particles[i] = new FluidParticleCPU
            {
                position = new Vector3(
                    Random.Range(0, gridResolution) * cellSize,
                    Random.Range(0, gridResolution) * cellSize,
                    Random.Range(0, gridResolution) * cellSize
                ),
                velocity = Vector3.zero
            };
        }
    }

    void ApplyExternalForces()
    {
        // Apply gravity to each particle's velocity
        for (int i = 0; i < numParticles; i++)
        {
            particles[i].velocity += gravity * timeStep;
        }
    }

    void TransferParticleToGrid()
    {
        // Clear mass and velocity grids
        System.Array.Clear(massGrid, 0, massGrid.Length);
        System.Array.Clear(velocityGrid, 0, velocityGrid.Length);

        // Accumulate mass and velocity contributions from particles to the grid
        for (int i = 0; i < numParticles; i++)
        {
            FluidParticleCPU p = particles[i];
            int xIndex = Mathf.FloorToInt(p.position.x / cellSize);
            int yIndex = Mathf.FloorToInt(p.position.y / cellSize);
            int zIndex = Mathf.FloorToInt(p.position.z / cellSize);

            Vector3 gridPosition = new Vector3(xIndex, yIndex, zIndex) * cellSize;
            Vector3 offset = (p.position - gridPosition) / cellSize;

            for (int iOffset = -1; iOffset <= 2; iOffset++)
            {
                for (int jOffset = -1; jOffset <= 2; jOffset++)
                {
                    for (int kOffset = -1; kOffset <= 2; kOffset++)
                    {
                        int gridX = xIndex + iOffset;
                        int gridY = yIndex + jOffset;
                        int gridZ = zIndex + kOffset;

                        if (gridX >= 0 && gridX < gridResolution &&
                            gridY >= 0 && gridY < gridResolution &&
                            gridZ >= 0 && gridZ < gridResolution)
                        {
                            int gridIndex = gridX + gridY * gridResolution + gridZ * gridResolution * gridResolution;
                            float weight = TricubicKernel(offset.x - iOffset) *
                                           TricubicKernel(offset.y - jOffset) *
                                           TricubicKernel(offset.z - kOffset);

                            massGrid[gridIndex] += weight;
                            velocityGrid[gridIndex] += p.velocity * weight;
                            print(velocityGrid[gridIndex]);
                        }
                    }
                }
            }
        }

        // Normalize the grid velocity by mass
        for (int i = 0; i < velocityGrid.Length; i++)
        {
            if (massGrid[i] > 0)
            {
                velocityGrid[i] /= massGrid[i];
            }
        }
    }

    void ComputeDivergence()
    {
        int gridSize = gridResolution * gridResolution * gridResolution;
        System.Array.Clear(divergenceGrid, 0, divergenceGrid.Length);

        for (int i = 0; i < gridSize; i++)
        {
            int x = i % gridResolution;
            int y = (i / gridResolution) % gridResolution;
            int z = i / (gridResolution * gridResolution);

            Vector3 gradient = Vector3.zero;

            if (x < gridResolution - 1) gradient.x += velocityGrid[i + 1].x - velocityGrid[i].x;
            if (x > 0) gradient.x -= velocityGrid[i].x - velocityGrid[i - 1].x;
            if (y < gridResolution - 1) gradient.y += velocityGrid[i + gridResolution].y - velocityGrid[i].y;
            if (y > 0) gradient.y -= velocityGrid[i].y - velocityGrid[i - gridResolution].y;
            if (z < gridResolution - 1) gradient.z += velocityGrid[i + gridResolution * gridResolution].z - velocityGrid[i].z;
            if (z > 0) gradient.z -= velocityGrid[i].z - velocityGrid[i - gridResolution * gridResolution].z;

            divergenceGrid[i] = (gradient.x + gradient.y + gradient.z) / cellSize;
        }
    }

    void EnforceBoundaries()
    {
        for (int i = 0; i < velocityGrid.Length; i++)
        {
            int x = i % gridResolution;
            int y = (i / gridResolution) % gridResolution;
            int z = i / (gridResolution * gridResolution);

            if (x == 0 || x == gridResolution - 1 ||
                y == 0 || y == gridResolution - 1 ||
                z == 0 || z == gridResolution - 1)
            {
                velocityGrid[i] = Vector3.zero;
            }
        }
    }

    void SolvePressure()
    {
        System.Array.Clear(pressureGrid, 0, pressureGrid.Length);

        for (int iter = 0; iter < 20; iter++)
        {
            for (int i = 0; i < pressureGrid.Length; i++)
            {
                int x = i % gridResolution;
                int y = (i / gridResolution) % gridResolution;
                int z = i / (gridResolution * gridResolution);

                float laplacian = 0f;

                if (x < gridResolution - 1) laplacian += pressureGrid[i + 1];
                if (x > 0) laplacian += pressureGrid[i - 1];
                if (y < gridResolution - 1) laplacian += pressureGrid[i + gridResolution];
                if (y > 0) laplacian += pressureGrid[i - gridResolution];
                if (z < gridResolution - 1) laplacian += pressureGrid[i + gridResolution * gridResolution];
                if (z > 0) laplacian += pressureGrid[i - gridResolution * gridResolution];

                laplacian -= 6f * pressureGrid[i];

                pressureGrid[i] = (divergenceGrid[i] - laplacian) / 6f;
            }
        }

        for (int i = 0; i < velocityGrid.Length; i++)
        {
            int x = i % gridResolution;
            int y = (i / gridResolution) % gridResolution;
            int z = i / (gridResolution * gridResolution);

            Vector3 gradient = Vector3.zero;

            if (x < gridResolution - 1) gradient.x += pressureGrid[i + 1] - pressureGrid[i];
            if (x > 0) gradient.x -= pressureGrid[i] - pressureGrid[i - 1];
            if (y < gridResolution - 1) gradient.y += pressureGrid[i + gridResolution] - pressureGrid[i];
            if (y > 0) gradient.y -= pressureGrid[i] - pressureGrid[i - gridResolution];
            if (z < gridResolution - 1) gradient.z += pressureGrid[i + gridResolution * gridResolution] - pressureGrid[i];
            if (z > 0) gradient.z -= pressureGrid[i] - pressureGrid[i - gridResolution * gridResolution];

            velocityGrid[i] -= gradient / cellSize;
        }
    }

    void ApplyViscosity()
    {
        for (int i = 0; i < velocityGrid.Length; i++)
        {
            Vector3 laplacian = GetLaplacian(velocityGrid, i);
            newVelocityGrid[i] = velocityGrid[i] + viscosity * laplacian * timeStep;
        }

        System.Array.Copy(newVelocityGrid, velocityGrid, velocityGrid.Length);
    }

    Vector3 GetLaplacian(Vector3[] field, int index)
    {
        int x = index % gridResolution;
        int y = (index / gridResolution) % gridResolution;
        int z = index / (gridResolution * gridResolution);

        Vector3 laplacian = Vector3.zero;

        if (x < gridResolution - 1) laplacian += field[index + 1];
        if (x > 0) laplacian += field[index - 1];
        if (y < gridResolution - 1) laplacian += field[index + gridResolution];
        if (y > 0) laplacian += field[index - gridResolution];
        if (z < gridResolution - 1) laplacian += field[index + gridResolution * gridResolution];
        if (z > 0) laplacian += field[index - gridResolution * gridResolution];

        laplacian -= 6f * field[index];

        return laplacian;
    }

    void UpdateParticleVelocities()
    {
        for (int i = 0; i < numParticles; i++)
        {
            particles[i].velocity = TricubicInterpolateVelocity(particles[i].position);
        }
    }

    Vector3 TricubicInterpolateVelocity(Vector3 particlePosition)
    {
        int xIndex = Mathf.FloorToInt(particlePosition.x / cellSize);
        int yIndex = Mathf.FloorToInt(particlePosition.y / cellSize);
        int zIndex = Mathf.FloorToInt(particlePosition.z / cellSize);

        Vector3 interpolatedVelocity = Vector3.zero;

        for (int i = -1; i <= 2; i++)
        {
            for (int j = -1; j <= 2; j++)
            {
                for (int k = -1; k <= 2; k++)
                {
                    int gridX = xIndex + i;
                    int gridY = yIndex + j;
                    int gridZ = zIndex + k;

                    if (gridX >= 0 && gridX < gridResolution &&
                        gridY >= 0 && gridY < gridResolution &&
                        gridZ >= 0 && gridZ < gridResolution)
                    {
                        int gridIndex = gridX + gridY * gridResolution + gridZ * gridResolution * gridResolution;
                        float weight = TricubicKernel((particlePosition.x / cellSize) - i) *
                                       TricubicKernel((particlePosition.y / cellSize) - j) *
                                       TricubicKernel((particlePosition.z / cellSize) - k);

                        interpolatedVelocity += velocityGrid[gridIndex] * weight;
                    }
                }
            }
        }

        return interpolatedVelocity;
    }

    void UpdateParticlePositions()
    {
        for (int i = 0; i < numParticles; i++)
        {
            particles[i].position += particles[i].velocity * timeStep;
        }
    }

    float TricubicKernel(float x)
    {
        x = Mathf.Abs(x);
        if (x < 1.0f)
        {
            return 1.5f * x * x * x - 2.5f * x * x + 1.0f;
        }
        else if (x < 2.0f)
        {
            return -0.5f * (x - 2.0f) * (x - 2.0f) * (x - 2.0f) + 2.5f * (x - 2.0f) * (x - 2.0f) - 4.0f * (x - 2.0f) + 2.0f;
        }
        else
        {
            return 0.0f;
        }
    }

    void Update()
    {
        ApplyExternalForces();
        TransferParticleToGrid();
        ComputeDivergence();
        EnforceBoundaries();
        //SolvePressure();
        ApplyViscosity();
        UpdateParticleVelocities();
        UpdateParticlePositions();
    }

    void OnDrawGizmos()
    {
        if (particles != null)
        {
            Gizmos.color = Color.cyan;
            for (int i = 0; i < numParticles; i++)
            {
                Gizmos.DrawSphere(particles[i].position, particleRadius);
            }
        }
    }
}

struct FluidParticleCPU
{
    public Vector3 position;
    public Vector3 velocity;
}
