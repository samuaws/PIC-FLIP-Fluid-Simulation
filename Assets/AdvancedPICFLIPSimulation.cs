using UnityEngine;

public class AdvancedPICFLIPSimulation : MonoBehaviour
{
    public ComputeShader divergenceShader;
    public ComputeShader conjugateGradientShader;
    public ComputeShader enforceBoundariesShader;
    public ComputeShader applyViscosityShader;
    public ComputeShader particleToGridShader;

    public int gridResolution = 128;
    public float cellSize = 1.0f / 128.0f;
    public int numParticles = 100000;
    public float particleRadius = 0.01f;
    public float timeStep = 0.01f;
    public float viscosity = 0.01f;

    private FluidParticle[] particles;
    private ComputeBuffer particleBuffer;
    private ComputeBuffer velocityBuffer;
    private ComputeBuffer massBuffer;
    private ComputeBuffer divergenceBuffer;
    private ComputeBuffer pressureBuffer;
    private ComputeBuffer residualBuffer;
    private ComputeBuffer directionBuffer;
    private ComputeBuffer ApBuffer;

    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    void Start()
    {
        InitializeBuffers();
        InitializeParticles();
    }

    void InitializeBuffers()
    {
        int gridSize = gridResolution * gridResolution * gridResolution;

        particleBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3 * 2); // Position + Velocity
        velocityBuffer = new ComputeBuffer(gridSize, sizeof(float) * 3);
        massBuffer = new ComputeBuffer(gridSize, sizeof(float));
        divergenceBuffer = new ComputeBuffer(gridSize, sizeof(float));
        pressureBuffer = new ComputeBuffer(gridSize, sizeof(float));
        residualBuffer = new ComputeBuffer(gridSize, sizeof(float));
        directionBuffer = new ComputeBuffer(gridSize, sizeof(float));
        ApBuffer = new ComputeBuffer(gridSize, sizeof(float));
    }

    void InitializeParticles()
    {
        particles = new FluidParticle[numParticles];
        for (int i = 0; i < numParticles; i++)
        {
            particles[i] = new FluidParticle
            {
                position = new Vector3(
                    Random.Range(0, gridResolution) * cellSize,
                    Random.Range(0, gridResolution) * cellSize,
                    Random.Range(0, gridResolution) * cellSize
                ),
                velocity = Vector3.zero
            };
        }
        particleBuffer.SetData(particles);
    }

    void ApplyExternalForces()
    {
        // Apply gravity to each particle's velocity
        for (int i = 0; i < numParticles; i++)
        {
            particles[i].velocity += gravity * timeStep;
        }

        // Update the particle buffer with the new velocities
        particleBuffer.SetData(particles);
    }

    void TransferParticleToGrid()
    {
        // Clear mass and velocity buffers
        massBuffer.SetData(new float[gridResolution * gridResolution * gridResolution]);
        velocityBuffer.SetData(new Vector3[gridResolution * gridResolution * gridResolution]);

        // Dispatch compute shader to handle particle to grid transfer
        particleToGridShader.SetInt("gridResolution", gridResolution);
        particleToGridShader.SetFloat("cellSize", cellSize);
        particleToGridShader.SetBuffer(0, "particles", particleBuffer);
        particleToGridShader.SetBuffer(0, "velocityBuffer", velocityBuffer);
        particleToGridShader.SetBuffer(0, "massBuffer", massBuffer);
        particleToGridShader.Dispatch(0, numParticles / 64, 1, 1);
    }

    void ComputeDivergence()
    {
        divergenceShader.SetInt("gridResolution", gridResolution);
        divergenceShader.SetFloat("cellSize", cellSize);
        divergenceShader.SetBuffer(0, "divergence", divergenceBuffer);
        divergenceShader.SetBuffer(0, "velocities", velocityBuffer);
        divergenceShader.Dispatch(0, gridResolution / 8, gridResolution / 8, gridResolution / 8);
    }

    void EnforceBoundaries()
    {
        enforceBoundariesShader.SetInt("gridResolution", gridResolution);
        enforceBoundariesShader.SetBuffer(0, "velocities", velocityBuffer);
        enforceBoundariesShader.Dispatch(0, gridResolution / 8, gridResolution / 8, gridResolution / 8);
    }

    void SolvePressure()
    {
        conjugateGradientShader.SetInt("gridResolution", gridResolution);
        conjugateGradientShader.SetFloat("cellSize", cellSize);
        conjugateGradientShader.SetFloat("timeStep", timeStep);
        conjugateGradientShader.SetBuffer(0, "pressure", pressureBuffer);
        conjugateGradientShader.SetBuffer(0, "divergence", divergenceBuffer);
        conjugateGradientShader.SetBuffer(0, "velocities", velocityBuffer);
        conjugateGradientShader.SetBuffer(0, "residual", residualBuffer);
        conjugateGradientShader.SetBuffer(0, "direction", directionBuffer);
        conjugateGradientShader.SetBuffer(0, "Ap", ApBuffer);

        for (int i = 0; i < 20; i++) // Conjugate Gradient iterations
        {
            conjugateGradientShader.Dispatch(0, gridResolution / 8, gridResolution / 8, gridResolution / 8);
        }
    }

    void ApplyViscosity()
    {
        applyViscosityShader.SetInt("gridResolution", gridResolution);
        applyViscosityShader.SetFloat("timeStep", timeStep);
        applyViscosityShader.SetFloat("viscosity", viscosity);
        applyViscosityShader.SetBuffer(0, "velocities", velocityBuffer);
        applyViscosityShader.Dispatch(0, gridResolution / 8, gridResolution / 8, gridResolution / 8);
    }

    void UpdateParticleVelocities()
    {
        // Get the data from the velocity buffer
        Vector3[] velocities = new Vector3[gridResolution * gridResolution * gridResolution];
        velocityBuffer.GetData(velocities);

        // Update particle velocities based on the grid velocities (using interpolation)
        particleBuffer.GetData(particles);

        for (int i = 0; i < numParticles; i++)
        {
            Vector3 particlePosition = particles[i].position;
            particles[i].velocity = TricubicInterpolateVelocity(particlePosition, velocities);
        }

        particleBuffer.SetData(particles);
    }

    Vector3 TricubicInterpolateVelocity(Vector3 particlePosition, Vector3[] velocities)
    {
        int xIndex = Mathf.FloorToInt(particlePosition.x / cellSize);
        int yIndex = Mathf.FloorToInt(particlePosition.y / cellSize);
        int zIndex = Mathf.FloorToInt(particlePosition.z / cellSize);

        // Calculate weights for tricubic interpolation
        float wx = (particlePosition.x / cellSize) - xIndex;
        float wy = (particlePosition.y / cellSize) - yIndex;
        float wz = (particlePosition.z / cellSize) - zIndex;

        Vector3 interpolatedVelocity = Vector3.zero;

        // Loop through 4x4x4 neighborhood
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
                        float weight = TricubicKernel(wx - i) * TricubicKernel(wy - j) * TricubicKernel(wz - k);

                        interpolatedVelocity += velocities[gridIndex] * weight;
                    }
                }
            }
        }

        return interpolatedVelocity;
    }

    float TricubicKernel(float x)
    {
        x = Mathf.Abs(x);
        if (x < 1)
        {
            return 1.5f * Mathf.Pow(x, 3) - 2.5f * Mathf.Pow(x, 2) + 1;
        }
        else if (x < 2)
        {
            return -0.5f * Mathf.Pow(x - 2, 3) + 2.5f * Mathf.Pow(x - 2, 2) - 4 * (x - 2) + 2;
        }
        else
        {
            return 0;
        }
    }

    void UpdateParticlePositions()
    {
        for (int i = 0; i < numParticles; i++)
        {
            particles[i].position += particles[i].velocity * timeStep;
        }

        particleBuffer.SetData(particles);
    }

    void Update()
    {
        ApplyExternalForces();  // Apply gravity before other steps
        TransferParticleToGrid();
        ComputeDivergence();
        EnforceBoundaries();
        SolvePressure();
        ApplyViscosity();
        UpdateParticleVelocities();
        UpdateParticlePositions();
    }

    void OnDestroy()
    {
        particleBuffer.Release();
        velocityBuffer.Release();
        massBuffer.Release();
        divergenceBuffer.Release();
        pressureBuffer.Release();
        residualBuffer.Release();
        directionBuffer.Release();
        ApBuffer.Release();
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

struct FluidParticle
{
    public Vector3 position;
    public Vector3 velocity;
}
