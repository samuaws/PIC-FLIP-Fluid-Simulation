using UnityEngine;
using System.Collections.Generic;

public class FlipPicFluidSimulation : MonoBehaviour
{
    // Visualization parameters
    [SerializeField] int resolutionX = 512;
    [SerializeField] int resolutionY = 512;
    [SerializeField] GameObject particlePrefab;
    [SerializeField] Vector2 boundsSize = new Vector2(10, 10);
    [SerializeField, Range(0.1f, 2.0f)] float particleSize = 0.5f;

    // Simulation parameters
    public int gridWidth = 40;
    public int gridHeight = 30;
    public float cellSize = 0.2f;
    public int particleCount = 10000;
    public float dt = 0.01f;
    public float viscosity = 0.1f;
    public int iterations = 20;
    public float gravity = -9.81f;

    private Vector2[,] gridVelocities;
    private Vector2[,] gridVelocitiesCopy;
    private float[,] weights;
    private List<Particle> particles;
    private float alpha = 0.01f; // 0 = PIC, 1 = FLIP

    void Start()
    {
        InitializeGridAndParticles();
    }

    void Update()
    {
        ApplyGravityToParticles();
        UpdateParticleSizes();
        TransferParticleVelocitiesToGrid();
        SolveIncompressibilityConstraint();
        TransferGridVelocitiesToParticles();
        UpdateParticlePositions();
    }

    private void InitializeGridAndParticles()
    {
        gridVelocities = new Vector2[gridWidth + 1, gridHeight + 1];
        gridVelocitiesCopy = new Vector2[gridWidth + 1, gridHeight + 1];
        weights = new float[gridWidth + 1, gridHeight + 1];
        particles = new List<Particle>();

        Vector2 halfBoundsSize = boundsSize / 2;

        for (int i = 0; i < particleCount; i++)
        {
            Vector2 randomPosition = new Vector2(
                Random.Range(-halfBoundsSize.x, halfBoundsSize.x),
                Random.Range(-halfBoundsSize.y, halfBoundsSize.y)
            );

            Particle particle = new Particle
            {
                position = randomPosition,
                velocity = Vector2.zero,
                mass = 1f,
                gameObject = Instantiate(particlePrefab, randomPosition, Quaternion.identity)
            };

            particle.gameObject.transform.localScale = new Vector3(particleSize, particleSize, particleSize);
            particles.Add(particle);
        }
    }

    private void ApplyGravityToParticles()
    {
        foreach (var particle in particles)
        {
            // Apply gravity to the y-component of the velocity
            particle.velocity += new Vector2(0, gravity) * dt;
        }
    }

    private void UpdateParticleSizes()
    {
        foreach (Particle particle in particles)
        {
            particle.gameObject.transform.localScale = new Vector3(particleSize, particleSize, particleSize);
        }
    }

    private void TransferParticleVelocitiesToGrid()
    {
        // Clear the grid velocities and weights
        for (int x = 0; x < gridWidth + 1; x++)
        {
            for (int y = 0; y < gridHeight + 1; y++)
            {
                gridVelocities[x, y] = Vector2.zero;
                gridVelocitiesCopy[x, y] = Vector2.zero;
                weights[x, y] = 0f;
            }
        }

        // Transfer particle velocities to grid
        foreach (var particle in particles)
        {
            Vector2 gridPos = particle.position / cellSize;
            int i = Mathf.FloorToInt(gridPos.x);
            int j = Mathf.FloorToInt(gridPos.y);

            // Calculate weights for bilinear interpolation
            float u = gridPos.x - i;
            float v = gridPos.y - j;

            // Boundary checks and velocity transfer
            if (i >= 0 && i < gridWidth && j >= 0 && j < gridHeight)
            {
                float weight = (1 - u) * (1 - v);
                gridVelocities[i, j] += weight * particle.velocity;
                weights[i, j] += weight;
            }
            if (i + 1 >= 0 && i + 1 <= gridWidth && j >= 0 && j < gridHeight)
            {
                float weight = u * (1 - v);
                gridVelocities[i + 1, j] += weight * particle.velocity;
                weights[i + 1, j] += weight;
            }
            if (i >= 0 && i < gridWidth && j + 1 >= 0 && j + 1 <= gridHeight)
            {
                float weight = (1 - u) * v;
                gridVelocities[i, j + 1] += weight * particle.velocity;
                weights[i, j + 1] += weight;
            }
            if (i + 1 >= 0 && i + 1 <= gridWidth && j + 1 >= 0 && j + 1 <= gridHeight)
            {
                float weight = u * v;
                gridVelocities[i + 1, j + 1] += weight * particle.velocity;
                weights[i + 1, j + 1] += weight;
            }
        }

        // Normalize grid velocities by their weights
        for (int x = 0; x < gridWidth + 1; x++)
        {
            for (int y = 0; y < gridHeight + 1; y++)
            {
                if (weights[x, y] > 0)
                {
                    gridVelocities[x, y] /= weights[x, y];
                }
                else
                {
                    gridVelocities[x, y] = Vector2.zero;
                }
            }
        }

        // Save a copy of the grid velocities for FLIP calculation
        System.Array.Copy(gridVelocities, gridVelocitiesCopy, gridVelocities.Length);
    }

    private void SolveIncompressibilityConstraint()
    {
        for (int k = 0; k < iterations; k++)
        {
            for (int i = 1; i < gridWidth; i++)
            {
                for (int j = 1; j < gridHeight; j++)
                {
                    if (weights[i, j] > 0)
                    {
                        float div = gridVelocities[i + 1, j].x - gridVelocities[i, j].x + gridVelocities[i, j + 1].y - gridVelocities[i, j].y;
                        float s = (i > 0 && i < gridWidth && j > 0 && j < gridHeight) ? 1 : 0;

                        gridVelocities[i, j].x += div * s;
                        gridVelocities[i + 1, j].x -= div * s;
                        gridVelocities[i, j].y += div * s;
                        gridVelocities[i, j + 1].y -= div * s;
                    }
                }
            }
        }
    }

    private void TransferGridVelocitiesToParticles()
    {
        foreach (var particle in particles)
        {
            Vector2 gridPos = particle.position / cellSize;
            int i = Mathf.FloorToInt(gridPos.x);
            int j = Mathf.FloorToInt(gridPos.y);

            // Calculate weights for bilinear interpolation
            float u = gridPos.x - i;
            float v = gridPos.y - j;

            // Ensure indices are within bounds and weights are non-zero
            if (i >= 0 && i < gridWidth && j >= 0 && j < gridHeight && weights[i, j] > 0)
            {
                // PIC velocity
                Vector2 picVelocity = Vector2.zero;
                if (weights[i, j] > 0)
                {
                    picVelocity += (1 - u) * (1 - v) * gridVelocities[i, j];
                }
                if (i + 1 <= gridWidth && weights[i + 1, j] > 0)
                {
                    picVelocity += u * (1 - v) * gridVelocities[i + 1, j];
                }
                if (j + 1 <= gridHeight && weights[i, j + 1] > 0)
                {
                    picVelocity += (1 - u) * v * gridVelocities[i, j + 1];
                }
                if (i + 1 <= gridWidth && j + 1 <= gridHeight && weights[i + 1, j + 1] > 0)
                {
                    picVelocity += u * v * gridVelocities[i + 1, j + 1];
                }

                // FLIP velocity
                Vector2 flipDelta = gridVelocities[i, j] - gridVelocitiesCopy[i, j];
                Vector2 flipVelocity = particle.velocity + flipDelta;

                // Combine PIC and FLIP
                particle.velocity = alpha * picVelocity + (1 - alpha) * flipVelocity;
            }
        }
    }

    private void UpdateParticlePositions()
    {
        Vector2 halfBoundsSize = boundsSize / 2;

        foreach (var particle in particles)
        {
            particle.position += particle.velocity * dt;

            // Handle boundary collisions
            if (particle.position.x < -halfBoundsSize.x || particle.position.x > halfBoundsSize.x)
            {
                particle.velocity.x *= -1;
                particle.position.x = Mathf.Clamp(particle.position.x, -halfBoundsSize.x, halfBoundsSize.x);
            }
            if (particle.position.y < -halfBoundsSize.y || particle.position.y > halfBoundsSize.y)
            {
                particle.velocity.y *= -1;
                particle.position.y = Mathf.Clamp(particle.position.y, -halfBoundsSize.y, halfBoundsSize.y);
            }

            // Update particle's visual position
            particle.gameObject.transform.position = new Vector3(particle.position.x, particle.position.y, 0);
        }
    }

    private class Particle
    {
        public Vector2 position;
        public Vector2 velocity;
        public float mass;
        public GameObject gameObject;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boundsSize.x, boundsSize.y, 0));
    }
}
