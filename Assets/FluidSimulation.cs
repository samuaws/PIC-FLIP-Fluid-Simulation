using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidSimulation : MonoBehaviour
{
    public GameObject particlePrefab; // Reference to the particle prefab
    public CellVisualizer cellVisualizer; // Reference to the CellVisualizer script

    [Header("Simulation Settings")]
    public Vector2 boundsSize = new Vector2(10, 10); // Size of the simulation bounds

    [Range(0.1f, 2.0f)]
    public float particleSize = 0.5f; // Size of each particle

    public float gravity = -9.81f;
    public float cellWidth = 1.0f; // Width of one cell
    public int activeCells = 10; // Number of cells that will spawn particles
    public float particleMass = 1.0f; // Mass of each particle
    public float smoothingLength = 1.0f; // Smoothing length for SPH
    public float viscosity = 1.0f; // Viscosity coefficient
    public float restDensity = 1000.0f; // Rest density
    public float stiffnessConstant = 3.0f; // Stiffness constant

    [Header("Visualization")]
    public bool visualizeGrid = true;
    public bool visualizeFluidCells = false;
    public bool visualizeCellsProperties = false;

    private List<Particle> particles = new List<Particle>();
    private Cell[,] grid;

    struct Particle
    {
        public Vector2 position;
        public Vector2 velocity;
        public GameObject gameObject;
        public float mass;
        public float density;
        public float pressure;

        public Particle(Vector2 pos, Vector2 vel, GameObject obj, float m)
        {
            position = pos;
            velocity = vel;
            gameObject = obj;
            mass = m;
            density = 0.0f;
            pressure = 0.0f;
        }
    }


    struct Cell
    {
        public Vector2 velocity;
        public Vector2 centerPosition;
        public bool hasFluid;
        public float distanceToBoundary;
        public bool isBoundary; // New field

        public Cell(Vector2 vel, Vector2 center, bool hasFluid)
        {
            velocity = vel;
            centerPosition = center;
            this.hasFluid = hasFluid;
            distanceToBoundary = float.MaxValue; // Initialize to a large value
            isBoundary = false; // Initialize as non-boundary
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

        // Update grid cell velocities
       // UpdateGridVelocities();

        // Recompute the distance field
       // ComputeDistanceField();

        // Extend the velocity field
        //ExtendVelocityField();

        // Calculate densities for all particles
        CalculateDensities();

        // Calculate viscosity forces and update velocities
        CalculateViscosityForcesAndUpdateVelocities();

        // Calculate pressures for all particles
        CalculatePressures(restDensity, stiffnessConstant);

        // Calculate pressure forces and update velocities
        CalculatePressureForcesAndUpdateVelocities();

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

        // Check for mouse click and handle cell visualization
        HandleMouseClick();
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
                grid[x, y] = new Cell(Vector2.zero, centerPosition, false);
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

                    Particle newParticle = new Particle(spawnPos, Vector2.zero, null , 1);
                    DrawCircle(spawnPos, particleSize, Color.blue, ref newParticle);
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
                    averageVelocity += Vector2.up * gravity;
                    grid[x, y].hasFluid = true;
                    grid[x, y].velocity = averageVelocity;
                }
                else
                {
                    grid[x, y].hasFluid = false;
                }
                

                // Check for boundary cells
                bool isBoundary = false;
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        if (i == 0 && j == 0) continue;
                        int nx = x + i;
                        int ny = y + j;
                        if (nx >= 0 && ny >= 0 && nx < cols && ny < rows)
                        {
                            if (grid[x, y].hasFluid != grid[nx, ny].hasFluid)
                            {
                                isBoundary = true;
                                break;
                            }
                        }
                    }
                    if (isBoundary) break;
                }

                grid[x, y].isBoundary = isBoundary;
                if (isBoundary)
                {
                    grid[x, y].distanceToBoundary = 0; // Initialize boundary cells with zero distance
                }
                else if (!grid[x, y].hasFluid)
                {
                    grid[x, y].distanceToBoundary = float.MaxValue; // Initialize non-fluid cells with max distance
                }
            }
        }
    }
    void ComputeDistanceField()
    {
        int cols = grid.GetLength(0);
        int rows = grid.GetLength(1);

        Queue<Vector2Int> queue = new Queue<Vector2Int>();

        // Enqueue all boundary cells
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                if (grid[x, y].isBoundary)
                {
                    queue.Enqueue(new Vector2Int(x, y));
                    grid[x, y].distanceToBoundary = 0; // Boundary cells have zero distance
                }
                else if (!grid[x, y].hasFluid)
                {
                    grid[x, y].distanceToBoundary = float.MaxValue; // Non-fluid cells initialize to max distance
                }
            }
        }

        // BFS to propagate distances
        int[] dx = { -1, 1, 0, 0, -1, -1, 1, 1 };
        int[] dy = { 0, 0, -1, 1, -1, 1, -1, 1 };

        while (queue.Count > 0)
        {
            Vector2Int cell = queue.Dequeue();
            int x = cell.x;
            int y = cell.y;

            for (int i = 0; i < 8; i++) // Use 8 directions for Euclidean distance
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx >= 0 && ny >= 0 && nx < cols && ny < rows && !grid[nx, ny].hasFluid)
                {
                    float newDist = grid[x, y].distanceToBoundary + Vector2.Distance(grid[x, y].centerPosition, grid[nx, ny].centerPosition);
                    if (newDist < grid[nx, ny].distanceToBoundary)
                    {
                        grid[nx, ny].distanceToBoundary = newDist;
                        queue.Enqueue(new Vector2Int(nx, ny));
                    }
                }
            }
        }
    }

    void ExtendVelocityField()
    {
        int cols = grid.GetLength(0);
        int rows = grid.GetLength(1);

        // Compute the gradient of the distance field
        Vector2[,] distanceGradient = new Vector2[cols, rows];
        for (int y = 1; y < rows - 1; y++)
        {
            for (int x = 1; x < cols - 1; x++)
            {
                float dox = (grid[x + 1, y].distanceToBoundary - grid[x - 1, y].distanceToBoundary) / (2 * cellWidth);
                float doy = (grid[x, y + 1].distanceToBoundary - grid[x, y - 1].distanceToBoundary) / (2 * cellWidth);
                distanceGradient[x, y] = new Vector2(dox, doy).normalized;
            }
        }

        // Extend the velocity field
        Queue<Vector2Int> queue = new Queue<Vector2Int>();

        // Initialize queue with fluid cells
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                if (grid[x, y].hasFluid)
                {
                    queue.Enqueue(new Vector2Int(x, y));
                }
            }
        }

        // Propagate velocities using BFS
        int[] dx = { -1, 1, 0, 0 };
        int[] dy = { 0, 0, -1, 1 };

        while (queue.Count > 0)
        {
            Vector2Int cell = queue.Dequeue();
            int x = cell.x;
            int y = cell.y;

            Vector2 baseVelocity = grid[x, y].velocity;

            for (int i = 0; i < 4; i++)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx >= 0 && ny >= 0 && nx < cols && ny < rows && !grid[nx, ny].hasFluid)
                {
                    Vector2 gradPhi = distanceGradient[nx, ny];
                    if (gradPhi != Vector2.zero)
                    {
                        Vector2 projectedVelocity = Vector2.Dot(baseVelocity, gradPhi) * gradPhi;
                        grid[nx, ny].velocity = projectedVelocity;
                        print(projectedVelocity);
                        // Mark the cell as processed by setting a small fluid amount (to avoid reprocessing)
                        //grid[nx, ny].hasFluid = true;
                        queue.Enqueue(new Vector2Int(nx, ny));
                    }
                }
            }
        }

        // Reset non-fluid cells after velocity extension
        //for (int y = 0; y < rows; y++)
        //{
        //    for (int x = 0; x < cols; x++)
        //    {
        //        if (grid[x, y].hasFluid && grid[x, y].velocity == Vector2.zero)
        //        {
        //            grid[x, y].hasFluid = false;
        //        }
        //    }
        //}
    }

    void CalculateDensities()
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Particle particle = particles[i];
            particle.density = 0.0f;

            for (int j = 0; j < particles.Count; j++)
            {
                Particle neighbor = particles[j];
                float distance = Vector2.Distance(particle.position, neighbor.position);
                if (distance < smoothingLength)
                {
                    //float q = distance / smoothingLength;
                    //float kernelValue = (1.0f - q) * (1.0f - q) * (1.0f - q);   
                    //float kernelValue = q*q*q;

                    float volume = Mathf.PI * Mathf.Pow(smoothingLength, 8) / 4;
                    float value = Mathf.Max(0, smoothingLength * smoothingLength - distance * distance);
                    float kernelValue = value * value * value / volume;
                    particle.density += neighbor.mass * kernelValue;
                }
            }

            particles[i] = particle; // Update the particle in the list
        }
    }
    void CalculateViscosityForcesAndUpdateVelocities()
    {
        float deltaTime = Time.deltaTime;

        for (int i = 0; i < particles.Count; i++)
        {
            Particle particle = particles[i];

            Vector2 viscosityForce = CalculateViscosityForce(particle);
            print(viscosityForce);
            Vector2 externalForce = new Vector2(0, gravity * particle.mass);

            Vector2 tempVelocity = particle.velocity + deltaTime * (viscosityForce + externalForce) / particle.mass;

            particle.velocity = tempVelocity;

            particles[i] = particle; // Update the particle in the list
        }
    }

    Vector2 CalculateViscosityForce(Particle particle)
    {
        Vector2 laplacianVelocity = Vector2.zero;

        for (int j = 0; j < particles.Count; j++)
        {
            if (particles.IndexOf(particle) != j)
            {
                Particle neighbor = particles[j];
                float distance = Vector2.Distance(particle.position, neighbor.position);

                if (distance < smoothingLength)
                {
                    float kernelLaplacian = CalculateViscosityKernelLaplacian(distance, smoothingLength);

                    // SPH formulation for Laplacian of velocity
                    laplacianVelocity += (neighbor.velocity - particle.velocity) * (neighbor.mass / neighbor.density) * kernelLaplacian;
                }
            }
        }

        // Multiply by viscosity coefficient and particle mass
        return particle.mass * viscosity * laplacianVelocity;
    }
    float CalculateViscosityKernelLaplacian(float distance, float smoothingLength)
    {
        if (distance > 0 && distance <= smoothingLength)
        {
            float r = distance;
            float h = smoothingLength;
            float h3 = h * h * h;
            return (15.0f / (2.0f * Mathf.PI * h3)) * (-r * r * r / (2 * h3) + r * r / (h * h) + h / (2 * r) - 1);
        }
        else
        {
            return 0.0f;
        }
    }

    void CalculatePressureForcesAndUpdateVelocities()
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Particle particle = particles[i];
            Vector2 pressureForce = CalculatePressureForce(particle);
            particle.velocity = - pressureForce * Time.deltaTime / particle.mass;
            //particle.velocity += new Vector2(0, gravity * particle.mass);
            particles[i] = particle; // Update the particle in the list
        }
    }
    void CalculatePressures(float restDensity, float stiffnessConstant)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            Particle particle = particles[i];
            particle.pressure = stiffnessConstant * (particle.density - restDensity);
            particles[i] = particle; // Update the particle in the list
        }
    }

    Vector2 CalculatePressureForce(Particle particle)
    {
        Vector2 pressureForce = Vector2.zero;

        for (int j = 0; j < particles.Count; j++)
        {
            if (particles.IndexOf(particle) != j)
            {
                Particle neighbor = particles[j];
                float distance = Vector2.Distance(particle.position, neighbor.position);

                if (distance < smoothingLength)
                {
                    // Compute the gradient of the Spiky kernel function
                    Vector2 kernelGradient = CalculateSpikyKernelGradient(particle.position, neighbor.position, smoothingLength);

                    // SPH formulation for pressure force
                    pressureForce -= (neighbor.mass * (particle.pressure + neighbor.pressure) / (2.0f * neighbor.density)) * kernelGradient;
                }
            }
        }

        return pressureForce;
    }

    Vector2 CalculateSpikyKernelGradient(Vector2 position_i, Vector2 position_j, float smoothingLength)
    {
        Vector2 r = position_i - position_j;
        float rLength = r.magnitude;
        if (rLength > smoothingLength || rLength == 0.0f)
        {
            return Vector2.zero;
        }

        float q = rLength / smoothingLength;
        float sigma = 15.0f / (Mathf.PI * Mathf.Pow(smoothingLength, 6)); // Normalization factor for 2D
        float gradient = sigma * -3 * Mathf.Pow(smoothingLength - rLength, 2);

        return gradient * (r / rLength);
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

                // Visualize cells with fluid
                if (visualizeFluidCells)
                {
                    if (grid[x, y].hasFluid)
                    {
                        Gizmos.color = Color.blue;
                        Gizmos.DrawCube(new Vector3(centerPosition.x, centerPosition.y, 0), new Vector3(cellWidth * 0.9f, cellWidth * 0.9f, 0));
                        Gizmos.color = Color.white;
                    }
                }

            }
        }
    }


    void HandleMouseClick()
    {
        if (Input.GetMouseButtonDown(0) && visualizeCellsProperties) // Left mouse button clicked
        {
            if(!cellVisualizer.cellInfoText.gameObject.activeSelf)
            {
                cellVisualizer.cellInfoText.gameObject.SetActive(true);
            }
            Vector2 mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            int cols = grid.GetLength(0);
            int rows = grid.GetLength(1);

            for (int y = 0; y < rows; y++)
            {
                for (int x = 0; x < cols; x++)
                {
                    Vector2 cellCenter = grid[x, y].centerPosition;
                    float halfCellWidth = cellWidth / 2;

                    if (mousePosition.x >= cellCenter.x - halfCellWidth && mousePosition.x <= cellCenter.x + halfCellWidth &&
                        mousePosition.y >= cellCenter.y - halfCellWidth && mousePosition.y <= cellCenter.y + halfCellWidth)
                    {
                        // Cell clicked, display its parameters
                        Cell clickedCell = grid[x, y];
                        cellVisualizer.DisplayCellInfo(clickedCell.centerPosition, clickedCell.velocity, clickedCell.hasFluid , clickedCell.distanceToBoundary , clickedCell.isBoundary);
                    }
                }
            }
        }
        else if (!visualizeCellsProperties && cellVisualizer.cellInfoText.gameObject.activeSelf)
        {
            cellVisualizer.cellInfoText.gameObject.SetActive(false);
        }
    }
}
