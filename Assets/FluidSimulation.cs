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
    public int particleCount = 100; // Number of particles

    private List<Particle> particles = new List<Particle>();

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

    void Start()
    {
        // Initialize particles in a grid pattern
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

    void DrawCircle(Vector2 position, float size, UnityEngine.Color color, ref Particle particle)
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
        Gizmos.color = UnityEngine.Color.green;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boundsSize.x, boundsSize.y, 0));
    }

    void SpawnParticlesInGrid()
    {
        int gridSize = Mathf.CeilToInt(Mathf.Sqrt(particleCount)); // Calculate the size of the grid
        float spacing = particleSize; // Set the spacing between particles

        // Calculate the total width and height of the grid
        float gridWidth = gridSize * spacing;
        float gridHeight = gridSize * spacing;

        // Center the grid in the bounds
        Vector2 startPos = new Vector2(
            -gridWidth / 2 + spacing / 2,
            -gridHeight / 2 + spacing / 2
        );

        for (int y = 0; y < gridSize; y++)
        {
            for (int x = 0; x < gridSize; x++)
            {
                if (particles.Count >= particleCount) return; // Stop if the particle count is reached

                Vector2 spawnPos = startPos + new Vector2(x * spacing, y * spacing);
                Particle newParticle = new Particle(spawnPos, Vector2.zero, null);
                DrawCircle(spawnPos, particleSize, UnityEngine.Color.cyan, ref newParticle); // Draw the particle
                particles.Add(newParticle);
            }
        }
    }
}
