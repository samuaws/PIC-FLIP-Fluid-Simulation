using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Solvers;

public class PICFLIPFluidSimulation : MonoBehaviour
{
    public int gridWidth = 40;
    public int gridHeight = 30;
    public int numParticles = 10000;
    public float cellSize = 0.2f;
    public float dt = 0.01f;
    public float viscosity = 0.1f;
    public float alpha = 0.01f; // PIC/FLIP blending factor
    public GameObject particlePrefab; // Prefab for particles

    private Particle[] particles;
    private GameObject[] particleObjects;
    private double[,] uGrid; // Horizontal velocities on vertical edges
    private double[,] vGrid; // Vertical velocities on horizontal edges
    private double[,] uPrevGrid;
    private double[,] vPrevGrid;
    private double[,] pressure;
    private int[,] cellTypes; // 0 = air, 1 = fluid, 2 = solid

    private void Start()
    {
        InitializeParticles();
        InitializeGrid();
    }

    private void Update()
    {
        TransferVelocitiesToGrid();
        SolveDivergence();
        UpdateParticleVelocities();
        UpdateParticlePositions();
    }

    private void InitializeParticles()
    {
        particles = new Particle[numParticles];
        particleObjects = new GameObject[numParticles];
        for (int i = 0; i < numParticles; i++)
        {
            Vector2 initialPosition = new Vector2(UnityEngine.Random.Range(0, gridWidth * cellSize),
                                                  UnityEngine.Random.Range(0, gridHeight * cellSize));
            particles[i] = new Particle
            {
                position = initialPosition,
                velocity = Vector2.zero,
                mass = 1.0f
            };
            // Instantiate the particle as a GameObject
            particleObjects[i] = Instantiate(particlePrefab, initialPosition, Quaternion.identity);
        }
    }

    private void InitializeGrid()
    {
        uGrid = new double[gridWidth + 1, gridHeight];
        vGrid = new double[gridWidth, gridHeight + 1];
        uPrevGrid = new double[gridWidth + 1, gridHeight];
        vPrevGrid = new double[gridWidth, gridHeight + 1];
        pressure = new double[gridWidth, gridHeight];
        cellTypes = new int[gridWidth, gridHeight];
    }

    private void TransferVelocitiesToGrid()
    {
        // Clear grid
        Array.Clear(uGrid, 0, uGrid.Length);
        Array.Clear(vGrid, 0, vGrid.Length);
        Array.Clear(cellTypes, 0, cellTypes.Length);

        // Transfer particle velocities to the grid
        foreach (var particle in particles)
        {
            int x = Mathf.FloorToInt(particle.position.x / cellSize);
            int y = Mathf.FloorToInt(particle.position.y / cellSize);

            double fx = (particle.position.x / cellSize) - x;
            double fy = (particle.position.y / cellSize) - y;

            // Bilinear interpolation weights
            double wx0 = 1 - fx;
            double wx1 = fx;
            double wy0 = 1 - fy;
            double wy1 = fy;

            // Transfer to uGrid (horizontal velocities)
            if (x < gridWidth && y < gridHeight)
            {
                uGrid[x, y] += particle.velocity.x * wx0 * wy0;
                if (x + 1 < gridWidth)
                    uGrid[x + 1, y] += particle.velocity.x * wx1 * wy0;
                if (y + 1 < gridHeight)
                    uGrid[x, y + 1] += particle.velocity.x * wx0 * wy1;
                if (x + 1 < gridWidth && y + 1 < gridHeight)
                    uGrid[x + 1, y + 1] += particle.velocity.x * wx1 * wy1;
            }

            // Transfer to vGrid (vertical velocities)
            if (x < gridWidth && y < gridHeight)
            {
                vGrid[x, y] += particle.velocity.y * wx0 * wy0;
                if (x + 1 < gridWidth)
                    vGrid[x + 1, y] += particle.velocity.y * wx1 * wy0;
                if (y + 1 < gridHeight)
                    vGrid[x, y + 1] += particle.velocity.y * wx0 * wy1;
                if (x + 1 < gridWidth && y + 1 < gridHeight)
                    vGrid[x + 1, y + 1] += particle.velocity.y * wx1 * wy1;
            }

            cellTypes[x, y] = 1; // Mark cell as fluid
        }

        // Copy velocity grid for FLIP calculation
        Array.Copy(uGrid, uPrevGrid, uGrid.Length);
        Array.Copy(vGrid, vPrevGrid, vGrid.Length);
    }
    private void SolveDivergence()
    {
        // Initialize pressure grid
        Array.Clear(pressure, 0, pressure.Length);

        // Create a linear system for the pressure solver
        var A = SparseMatrix.OfArray(new double[gridWidth * gridHeight, gridWidth * gridHeight]);
        var b = new DenseVector(gridWidth * gridHeight);
        var pressureVector = new DenseVector(gridWidth * gridHeight);

        // Build the linear system (A * pressure = b)
        for (int i = 1; i < gridWidth - 1; i++)
        {
            for (int j = 1; j < gridHeight - 1; j++)
            {
                if (cellTypes[i, j] == 1) // If cell is fluid
                {
                    int index = i * gridHeight + j;
                    b[index] = -(uGrid[i + 1, j] - uGrid[i, j] + vGrid[i, j + 1] - vGrid[i, j]) / cellSize;

                    // Center coefficient
                    A[index, index] = 4;

                    // Neighbor coefficients
                    A[index, (i - 1) * gridHeight + j] = -1;
                    A[index, (i + 1) * gridHeight + j] = -1;
                    A[index, i * gridHeight + (j - 1)] = -1;
                    A[index, i * gridHeight + (j + 1)] = -1;
                }
            }
        }

        // Set up the Conjugate Gradient solver and iterator
        var solver = new MathNet.Numerics.LinearAlgebra.Double.Solvers.GpBiCg();
        var iterator = new Iterator<double>(
            new IterationCountStopCriterion<double>(1000),
            new ResidualStopCriterion<double>(1e-10)
        );

        // Solve the linear system without a preconditioner
        solver.Solve(A, b, pressureVector, iterator, null);

        // Extract the pressure values back into the pressure grid
        for (int i = 0; i < gridWidth; i++)
        {
            for (int j = 0; j < gridHeight; j++)
            {
                pressure[i, j] = pressureVector[i * gridHeight + j];
            }
        }

        // Subtract pressure gradient from velocity to remove divergence
        for (int i = 1; i < gridWidth - 1; i++)
        {
            for (int j = 1; j < gridHeight - 1; j++)
            {
                if (cellTypes[i, j] == 1) // If cell is fluid
                {
                    uGrid[i, j] -= (pressure[i, j] - pressure[i - 1, j]) / cellSize;
                    uGrid[i + 1, j] -= (pressure[i + 1, j] - pressure[i, j]) / cellSize;
                    vGrid[i, j] -= (pressure[i, j] - pressure[i, j - 1]) / cellSize;
                    vGrid[i, j + 1] -= (pressure[i, j + 1] - pressure[i, j]) / cellSize;
                }
            }
        }
    }

    private void UpdateParticleVelocities()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Particle particle = particles[i];
            int x = Mathf.FloorToInt(particle.position.x / cellSize);
            int y = Mathf.FloorToInt(particle.position.y / cellSize);

            double fx = (particle.position.x / cellSize) - x;
            double fy = (particle.position.y / cellSize) - y;

            // Bilinear interpolation weights
            double wx0 = 1 - fx;
            double wx1 = fx;
            double wy0 = 1 - fy;
            double wy1 = fy;

            // PIC velocity
            double uPic = 0;
            double vPic = 0;

            if (x < gridWidth && y < gridHeight)
            {
                uPic = uGrid[x, y] * wx0 * wy0;

                if (x + 1 < gridWidth)
                    uPic += uGrid[x + 1, y] * wx1 * wy0;

                if (y + 1 < gridHeight)
                    uPic += uGrid[x, y + 1] * wx0 * wy1;

                if (x + 1 < gridWidth && y + 1 < gridHeight)
                    uPic += uGrid[x + 1, y + 1] * wx1 * wy1;

                vPic = vGrid[x, y] * wx0 * wy0;

                if (x + 1 < gridWidth)
                    vPic += vGrid[x + 1, y] * wx1 * wy0;

                if (y + 1 < gridHeight)
                    vPic += vGrid[x, y + 1] * wx0 * wy1;

                if (x + 1 < gridWidth && y + 1 < gridHeight)
                    vPic += vGrid[x + 1, y + 1] * wx1 * wy1;
            }

            Vector2 picVelocity = new Vector2((float)uPic, (float)vPic);

            // FLIP velocity
            double uFlip = particle.velocity.x;
            double vFlip = particle.velocity.y;

            if (x < gridWidth && y < gridHeight)
            {
                uFlip += (uGrid[x, y] - uPrevGrid[x, y]) * wx0 * wy0;

                if (x + 1 < gridWidth)
                    uFlip += (uGrid[x + 1, y] - uPrevGrid[x + 1, y]) * wx1 * wy0;

                if (y + 1 < gridHeight)
                    uFlip += (uGrid[x, y + 1] - uPrevGrid[x, y + 1]) * wx0 * wy1;

                if (x + 1 < gridWidth && y + 1 < gridHeight)
                    uFlip += (uGrid[x + 1, y + 1] - uPrevGrid[x + 1, y + 1]) * wx1 * wy1;

                vFlip += (vGrid[x, y] - vPrevGrid[x, y]) * wx0 * wy0;

                if (x + 1 < gridWidth)
                    vFlip += (vGrid[x + 1, y] - vPrevGrid[x + 1, y]) * wx1 * wy0;

                if (y + 1 < gridHeight)
                    vFlip += (vGrid[x, y + 1] - vPrevGrid[x, y + 1]) * wx0 * wy1;

                if (x + 1 < gridWidth && y + 1 < gridHeight)
                    vFlip += (vGrid[x + 1, y + 1] - vPrevGrid[x + 1, y + 1]) * wx1 * wy1;
            }

            Vector2 flipVelocity = new Vector2((float)uFlip, (float)vFlip);

            // Combine PIC and FLIP velocities
            particle.velocity = alpha * picVelocity + (1 - alpha) * flipVelocity;
        }
    }


    private void UpdateParticlePositions()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            // Update positions using symplectic Euler integration
            particles[i].position += particles[i].velocity * dt;

            // Update the corresponding GameObject's position
            particleObjects[i].transform.position = particles[i].position;

            // Handle boundaries (simple reflection)
            if (particles[i].position.x < 0 || particles[i].position.x > gridWidth * cellSize)
                particles[i].velocity.x = -particles[i].velocity.x;

            if (particles[i].position.y < 0 || particles[i].position.y > gridHeight * cellSize)
                particles[i].velocity.y = -particles[i].velocity.y;
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.gray;

        // Draw the grid
        for (int i = 0; i <= gridWidth; i++)
        {
            for (int j = 0; j <= gridHeight; j++)
            {
                Vector3 start = new Vector3(i * cellSize, 0, j * cellSize);
                Vector3 end = new Vector3(i * cellSize, 0, gridHeight * cellSize);
                Gizmos.DrawLine(start, end);

                start = new Vector3(0, 0, j * cellSize);
                end = new Vector3(gridWidth * cellSize, 0, j * cellSize);
                Gizmos.DrawLine(start, end);
            }
        }
    }

    private class Particle
    {
        public Vector2 position;
        public Vector2 velocity;
        public float mass;
    }
}
