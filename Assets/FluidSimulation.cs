using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidSimulation : MonoBehaviour
{
    public GameObject particlePrefab; // Reference to the particle prefab

    [Header("Simulation Settings")]
    public Vector2 boundsSize = new Vector2(10, 10); // Size of the simulation bounds

    [Range(0.1f, 2.0f)]
    public float particleSize = 0.5f; // Size of each particle

    public float gravity = -9.81f;
    public float cellWidth = 1.0f; // Width of one cell
    public int activeCells = 10; // Number of cells that will spawn particles
    public bool visualizeGrid = true;

    private List<Particle> particles = new List<Particle>();
    private Cell[,] grid;

    struct Particle
    {
        public Vector2 position;
        public Vector2 velocity;
        public GameObject gameObject;

        public Particle(Vector2 pos, Vector2 vel, GameObject obj)
        {
            position = pos;
            velocity = vel;
            gameObject = obj;
        }
    }

    struct Cell
    {
        public Vector2 velocity;
        public Vector2 centerPosition;

        public Cell(Vector2 vel, Vector2 center)
        {
            velocity = vel;
            centerPosition = center;
        }
    }

    void Start()
    {
        // Initialize the grid
        InitializeGrid();

        // Spawn particles in a grid pattern
        SpawnParticlesInGrid();
    }

    void Update()
    {
        // Update each particle's position based on its velocity
        for (int i = 0; i < particles.Count; i++)
        {
            Particle particle = particles[i];

            // Update velocity and position
            particle.velocity += Vector2.up * gravity * Time.deltaTime;
            particle.position += particle.velocity * Time.deltaTime;

            // Resolve collisions with the bounds
            ResolveCollisions(ref particle);

            // Update the GameObject's position
            particle.gameObject.transform.position = new Vector3(particle.position.x, particle.position.y, 0);

            // Update the particle in the list
            particles[i] = particle;
        }

        // Update grid cell velocities
        UpdateGridVelocities();
    }

    void ResolveCollisions(ref Particle particle)
    {
        Vector2 halfBoundsSize = boundsSize / 2 - Vector2.one * (particleSize * 0.1f);

        if (Mathf.Abs(particle.position.x) > halfBoundsSize.x)
        {
            particle.position.x = halfBoundsSize.x * Mathf.Sign(particle.position.x);
            particle.velocity.x *= -1;
        }

        if (Mathf.Abs(particle.position.y) > halfBoundsSize.y)
        {
            particle.position.y = halfBoundsSize.y * Mathf.Sign(particle.position.y);
            particle.velocity.y *= -1;
        }
    }

    void DrawCircle(Vector2 position, float size, Color color, ref Particle particle)
    {
        // Instantiate a new particle at the given position
        GameObject particleObject = Instantiate(particlePrefab, new Vector3(position.x, position.y, 0), Quaternion.identity);

        // Set the size of the particle
        size *= 0.1f;
        particleObject.transform.localScale = new Vector3(size, size, size);

        // Set the color of the particle (if the particle prefab has a SpriteRenderer component)
        SpriteRenderer spriteRenderer = particleObject.GetComponent<SpriteRenderer>();
        if (spriteRenderer != null)
        {
            spriteRenderer.color = color;
        }

        // Update the particle struct with the GameObject
        particle.gameObject = particleObject;
    }

    void OnDrawGizmos()
    {
        // Draw the bounds
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boundsSize.x, boundsSize.y, 0));

        // Visualize the grid
        if (grid != null && visualizeGrid)
        {
            VisualizeGrid();
        }
    }

    void InitializeGrid()
    {
        int cols = Mathf.CeilToInt(boundsSize.x / cellWidth);
        int rows = Mathf.CeilToInt(boundsSize.y / cellWidth);

        grid = new Cell[cols, rows];

        Vector2 offset = new Vector2(boundsSize.x / 2, boundsSize.y / 2);

        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                Vector2 centerPosition = new Vector2(x * cellWidth - offset.x + cellWidth / 2, y * cellWidth - offset.y + cellWidth / 2);
                grid[x, y] = new Cell(Vector2.zero, centerPosition);
            }
        }
    }

    void SpawnParticlesInGrid()
    {
        int cols = Mathf.CeilToInt(boundsSize.x / cellWidth);
        int rows = Mathf.CeilToInt(boundsSize.y / cellWidth);

        int totalCells = cols * rows;
        int cellsToActivate = Mathf.Min(activeCells, totalCells);

        // Randomly select cells to activate
        List<Vector2Int> activeCellIndices = new List<Vector2Int>();
        while (activeCellIndices.Count < cellsToActivate)
        {
            int x = Random.Range(0, cols);
            int y = Random.Range(0, rows);
            Vector2Int cellIndex = new Vector2Int(x, y);
            if (!activeCellIndices.Contains(cellIndex))
            {
                activeCellIndices.Add(cellIndex);
            }
        }

        foreach (Vector2Int cellIndex in activeCellIndices)
        {
            // Split each cell into 2x2 subcells
            float subCellWidth = cellWidth / 2.0f;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    // Randomly position the particle within the subcell
                    float xOffset = Random.Range(0, subCellWidth);
                    float yOffset = Random.Range(0, subCellWidth);

                    Vector2 cellCenter = grid[cellIndex.x, cellIndex.y].centerPosition;

                    Vector2 spawnPos = cellCenter + new Vector2(
                        i * subCellWidth + xOffset - subCellWidth / 2,
                        j * subCellWidth + yOffset - subCellWidth / 2
                    );

                    Particle newParticle = new Particle(spawnPos, Vector2.zero, null);
                    DrawCircle(spawnPos, particleSize, Color.cyan, ref newParticle);
                    particles.Add(newParticle);
                }
            }
        }
    }

    void UpdateGridVelocities()
    {
        int cols = grid.GetLength(0);
        int rows = grid.GetLength(1);

        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                Vector2 cellCenter = grid[x, y].centerPosition;
                Vector2 averageVelocity = Vector2.zero;
                int particleCount = 0;

                foreach (var particle in particles)
                {
                    if (Vector2.Distance(particle.position, cellCenter) <= cellWidth)
                    {
                        averageVelocity += particle.velocity;
                        particleCount++;
                    }
                }

                if (particleCount > 0)
                {
                    averageVelocity /= particleCount;
                }

                grid[x, y].velocity = averageVelocity;
            }
        }
    }

    void VisualizeGrid()
    {
        Gizmos.color = Color.white;

        int cols = Mathf.CeilToInt(boundsSize.x / cellWidth);
        int rows = Mathf.CeilToInt(boundsSize.y / cellWidth);

        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                Vector2 centerPosition = grid[x, y].centerPosition;
                Gizmos.DrawWireCube(new Vector3(centerPosition.x, centerPosition.y, 0), new Vector3(cellWidth, cellWidth, 0));
            }
        }
    }
}
