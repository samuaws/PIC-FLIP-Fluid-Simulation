using UnityEngine;
using System.Collections.Generic;

public class PICFLIPSim : MonoBehaviour
{
    public int gridWidth = 40;
    public int gridHeight = 30;
    public int numParticles = 10000;
    public float alpha = 0.01f;
    public float deltaTime = 0.01f;
    public GameObject particlePrefab;

    private MACGrid grid;
    private List<Particle> particles;
    private List<GameObject> particleObjects;

    void Start()
    {
        grid = new MACGrid(gridWidth, gridHeight);
        particles = InitializeParticles(numParticles, gridWidth, gridHeight);
        particleObjects = new List<GameObject>();

        foreach (var particle in particles)
        {
            var particleObj = Instantiate(particlePrefab, particle.Position, Quaternion.identity);
            particleObjects.Add(particleObj);
        }
    }

    void Update()
    {
        TransferVelocitiesToGrid(particles, grid);
        SolveDivergenceAndPressure(grid);
        TransferVelocitiesToParticles(particles, grid, alpha);
        UpdateParticlePositions(particles, deltaTime);
        UpdateParticleObjects();
    }

    List<Particle> InitializeParticles(int numParticles, int width, int height)
    {
        var particles = new List<Particle>();
        for (int i = 0; i < numParticles; i++)
        {
            Vector2 pos = new Vector2(Random.Range(0, width), Random.Range(0, height));
            Vector2 vel = new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f));
            particles.Add(new Particle(pos, vel, 1f));
        }
        return particles;
    }

    void TransferVelocitiesToGrid(List<Particle> particles, MACGrid grid)
    {
        foreach (var particle in particles)
        {
            int i = Mathf.FloorToInt(particle.Position.x);
            int j = Mathf.FloorToInt(particle.Position.y);

            // Ensure i and j are within valid range for U and V grids
            if (i >= 0 && i < grid.Width && j >= 0 && j < grid.Height)
            {
                // Bilinear interpolation for horizontal velocities (U)
                if (i + 1 < grid.Width)
                {
                    float uLeft = (i + 1 - particle.Position.x) * particle.Velocity.x;
                    float uRight = (particle.Position.x - i) * particle.Velocity.x;
                    grid.U[i, j] += uLeft;
                    grid.U[i + 1, j] += uRight;
                }

                // Bilinear interpolation for vertical velocities (V)
                if (j + 1 < grid.Height)
                {
                    float vBottom = (j + 1 - particle.Position.y) * particle.Velocity.y;
                    float vTop = (particle.Position.y - j) * particle.Velocity.y;
                    grid.V[i, j] += vBottom;
                    grid.V[i, j + 1] += vTop;
                }
            }
        }
    }

    void SolveDivergenceAndPressure(MACGrid grid)
    {
        // Step 1: Calculate divergence
        for (int i = 1; i < grid.Width - 1; i++)
        {
            for (int j = 1; j < grid.Height - 1; j++)
            {
                grid.P[i, j] = grid.U[i + 1, j] - grid.U[i, j] + grid.V[i, j + 1] - grid.V[i, j];
            }
        }

        // Step 2: Gauss-Seidel iteration for pressure
        for (int k = 0; k < 20; k++) // 20 iterations
        {
            for (int i = 1; i < grid.Width - 1; i++)
            {
                for (int j = 1; j < grid.Height - 1; j++)
                {
                    grid.P[i, j] = 0.25f * (grid.P[i - 1, j] + grid.P[i + 1, j] + grid.P[i, j - 1] + grid.P[i, j + 1] - grid.P[i, j]);
                }
            }
        }
    }

    void TransferVelocitiesToParticles(List<Particle> particles, MACGrid grid, float alpha)
    {
        foreach (var particle in particles)
        {
            int i = Mathf.FloorToInt(particle.Position.x);
            int j = Mathf.FloorToInt(particle.Position.y);

            // Ensure i and j are within valid range for U and V grids
            if (i >= 0 && i < grid.Width && j >= 0 && j < grid.Height)
            {
                // Bilinear interpolation for grid velocities
                float uInterp = Mathf.Lerp(grid.U[i, j], grid.U[i + 1, j], particle.Position.x - i);
                float vInterp = Mathf.Lerp(grid.V[i, j], grid.V[i, j + 1], particle.Position.y - j);

                Vector2 picVelocity = new Vector2(uInterp, vInterp);
                Vector2 flipVelocity = particle.Velocity + (picVelocity - particle.Velocity);

                particle.Velocity = alpha * picVelocity + (1 - alpha) * flipVelocity;
            }
        }
    }

    void UpdateParticlePositions(List<Particle> particles, float deltaTime)
    {
        foreach (var particle in particles)
        {
            particle.Position += particle.Velocity * deltaTime;
        }
    }

    void UpdateParticleObjects()
    {
        for (int i = 0; i < particles.Count; i++)
        {
            particleObjects[i].transform.position = particles[i].Position;
        }
    }

    public class Particle
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public float Mass;

        public Particle(Vector2 position, Vector2 velocity, float mass)
        {
            Position = position;
            Velocity = velocity;
            Mass = mass;
        }
    }

    public class MACGrid
    {
        public int Width;
        public int Height;
        public float[,] U; // Horizontal velocities
        public float[,] V; // Vertical velocities
        public float[,] P; // Pressure

        public MACGrid(int width, int height)
        {
            Width = width;
            Height = height;
            U = new float[width + 1, height];
            V = new float[width, height + 1];
            P = new float[width, height];
        }
    }
}
