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
    public Color gridColor = Color.green; // Color of the grid in Gizmos
    public bool usePCGSolver = false; // Option to use PCG solver

    private MACGrid grid;
    private List<Particle> particles;
    private List<GameObject> particleObjects;
    private Vector2 gridOrigin; // The origin point of the grid

    void Start()
    {
        grid = new MACGrid(gridWidth, gridHeight);
        particles = InitializeParticles(numParticles, gridWidth, gridHeight);
        particleObjects = new List<GameObject>();

        // Calculate the origin so that the grid is centered in the screen
        gridOrigin = new Vector2(-gridWidth / 2f, -gridHeight / 2f);

        foreach (var particle in particles)
        {
            var particleObj = Instantiate(particlePrefab, GridToWorldPosition(particle.Position), Quaternion.identity);
            particleObjects.Add(particleObj);
        }
    }

    void Update()
    {
        ClassifyGridCells(grid, particles);
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

    void ClassifyGridCells(MACGrid grid, List<Particle> particles)
    {
        for (int i = 0; i < grid.Width; i++)
        {
            for (int j = 0; j < grid.Height; j++)
            {
                grid.CellTypes[i, j] = MACGrid.CellType.Air;
            }
        }

        foreach (var particle in particles)
        {
            int i = Mathf.FloorToInt(particle.Position.x);
            int j = Mathf.FloorToInt(particle.Position.y);
            if (i >= 0 && i < grid.Width && j >= 0 && j < grid.Height)
            {
                grid.CellTypes[i, j] = MACGrid.CellType.Fluid;
            }
        }

        // Optionally mark solid boundaries as Solid type
        // Example: grid.CellTypes[0, j] = MACGrid.CellType.Solid; (for the left boundary)
    }

    void TransferVelocitiesToGrid(List<Particle> particles, MACGrid grid)
    {
        foreach (var particle in particles)
        {
            int i = Mathf.FloorToInt(particle.Position.x);
            int j = Mathf.FloorToInt(particle.Position.y);

            if (i >= 0 && i < grid.Width && j >= 0 && j < grid.Height)
            {
                float uLeft = (i + 1 - particle.Position.x) * particle.Velocity.x;
                float uRight = (particle.Position.x - i) * particle.Velocity.x;
                grid.U[i, j] += uLeft;
                grid.U[i + 1, j] += uRight;

                float vBottom = (j + 1 - particle.Position.y) * particle.Velocity.y;
                float vTop = (particle.Position.y - j) * particle.Velocity.y;
                grid.V[i, j] += vBottom;
                grid.V[i, j + 1] += vTop;
            }
        }

        // Enforce boundary conditions
        for (int i = 0; i < grid.Width; i++)
        {
            for (int j = 0; j < grid.Height; j++)
            {
                if (grid.CellTypes[i, j] == MACGrid.CellType.Solid)
                {
                    grid.U[i, j] = 0;
                    grid.V[i, j] = 0;
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
                if (grid.CellTypes[i, j] == MACGrid.CellType.Fluid)
                {
                    grid.P[i, j] = grid.U[i + 1, j] - grid.U[i, j] + grid.V[i, j + 1] - grid.V[i, j];
                }
            }
        }

        // Step 2: Solve for pressure
        if (usePCGSolver)
        {
            // Implement Preconditioned Conjugate Gradient (PCG) solver here
            // Placeholder: Implement or integrate a PCG solver library
        }
        else
        {
            // Gauss-Seidel iteration
            for (int k = 0; k < 20; k++) // 20 iterations
            {
                for (int i = 1; i < grid.Width - 1; i++)
                {
                    for (int j = 1; j < grid.Height - 1; j++)
                    {
                        if (grid.CellTypes[i, j] == MACGrid.CellType.Fluid)
                        {
                            grid.P[i, j] = 0.25f * (grid.P[i - 1, j] + grid.P[i + 1, j] + grid.P[i, j - 1] + grid.P[i, j + 1] - grid.P[i, j]);
                        }
                    }
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

            if (i >= 0 && i < grid.Width && j >= 0 && j < grid.Height)
            {
                float uInterp = Mathf.Lerp(grid.U[i, j], grid.U[i + 1, j], particle.Position.x - i);
                float vInterp = Mathf.Lerp(grid.V[i, j], grid.V[i, j + 1], particle.Position.y - j);

                // Check for NaN values
                if (float.IsNaN(uInterp) || float.IsNaN(vInterp))
                {
                    Debug.LogError($"NaN detected in interpolated velocity: uInterp = {uInterp}, vInterp = {vInterp}");
                    uInterp = vInterp = 0; // Reset to prevent further errors
                }

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

            // Check for NaN values
            if (float.IsNaN(particle.Position.x) || float.IsNaN(particle.Position.y))
            {
                Debug.LogError($"NaN detected in particle position: {particle.Position}, velocity: {particle.Velocity}");
                particle.Position = Vector2.zero; // Reset to a valid position to prevent further errors
            }

            // Boundary conditions
            if (particle.Position.x < 0)
            {
                particle.Position.x = 0;
                particle.Velocity.x *= -1;
            }
            else if (particle.Position.x >= grid.Width)
            {
                particle.Position.x = grid.Width - 1;
                particle.Velocity.x *= -1;
            }

            if (particle.Position.y < 0)
            {
                particle.Position.y = 0;
                particle.Velocity.y *= -1;
            }
            else if (particle.Position.y >= grid.Height)
            {
                particle.Position.y = grid.Height - 1;
                particle.Velocity.y *= -1;
            }
        }
    }

    void UpdateParticleObjects()
    {
        for (int i = 0; i < particles.Count; i++)
        {
            particleObjects[i].transform.position = GridToWorldPosition(particles[i].Position);
        }
    }

    Vector2 GridToWorldPosition(Vector2 gridPosition)
    {
        return gridOrigin + gridPosition;
    }

    void OnDrawGizmos()
    {
        if (grid == null) return;

        Gizmos.color = gridColor;

        for (int i = 0; i < grid.Width; i++)
        {
            for (int j = 0; j < grid.Height; j++)
            {
                Vector2 cellPos = GridToWorldPosition(new Vector2(i, j));
                Gizmos.DrawWireCube(cellPos, Vector3.one);
            }
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
        public enum CellType { Air, Fluid, Solid }

        public int Width;
        public int Height;
        public float[,] U; // Horizontal velocities
        public float[,] V; // Vertical velocities
        public float[,] P; // Pressure
        public CellType[,] CellTypes;

        public MACGrid(int width, int height)
        {
            Width = width;
            Height = height;
            U = new float[width + 1, height];
            V = new float[width, height + 1];
            P = new float[width, height];
            CellTypes = new CellType[width, height];
        }
    }
}
