using UnityEngine;
using System.Collections.Generic;

public class ParticlePICFLIP : MonoBehaviour
{
    [SerializeField] int resolutionX = 512;
    [SerializeField] int resolutionY = 512;
    [SerializeField] float deltaTime = 0.01f;
    [SerializeField] int numParticles = 10000;
    [SerializeField] float maxInitialVelocity = 1.0f;
    [SerializeField] GameObject particlePrefab;
    [SerializeField] Vector2 boundsSize = new Vector2(10, 10);
    [SerializeField, Range(0.1f, 2.0f)] float particleSize = 0.5f;
    [SerializeField, Range(0.0f, 1.0f)] float dampingFactor = 0.8f;
    [SerializeField] float gravity = -9.81f;
    [SerializeField, Range(0.0f, 1.0f)] float alpha = 0.05f;

    private List<Particle> particles;
    private Vector2[,] velocityField;
    private Vector2[,] prevVelocityField;
    private float[,] pressureField;
    private float[,] divField;
    private bool isInitialized;

    public class Particle
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
        InitializeFields();
        InitializeParticles();
        isInitialized = true;
    }

    void InitializeFields()
    {
        velocityField = new Vector2[resolutionX + 1, resolutionY + 1];  // MAC Grid
        prevVelocityField = new Vector2[resolutionX + 1, resolutionY + 1];
        pressureField = new float[resolutionX + 1, resolutionY + 1];
        divField = new float[resolutionX + 1, resolutionY + 1];
    }

    void InitializeParticles()
    {
        particles = new List<Particle>(numParticles);
        Vector2 halfBoundsSize = boundsSize / 2;

        for (int i = 0; i < numParticles; i++)
        {
            Vector2 randomPosition = new Vector2(
                Random.Range(-halfBoundsSize.x, halfBoundsSize.x),
                Random.Range(-halfBoundsSize.y, halfBoundsSize.y)
            );

            Vector2 randomVelocity = new Vector2(
                Random.Range(-maxInitialVelocity, maxInitialVelocity),
                Random.Range(-maxInitialVelocity, maxInitialVelocity)
            );

            GameObject particleObj = Instantiate(particlePrefab, randomPosition, Quaternion.identity);
            particleObj.transform.localScale = new Vector3(particleSize, particleSize, particleSize);

            particles.Add(new Particle(randomPosition, randomVelocity, particleObj));
        }
    }

    void Update()
    {
        if (!isInitialized) return;

        UpdateParticleSizes();
        ScatterParticlesToGrid();
        PressureProjectionStep();
        GatherGridToParticles();
        UpdateParticlePositions();
    }

    void UpdateParticleSizes()
    {
        foreach (Particle particle in particles)
        {
            particle.gameObject.transform.localScale = new Vector3(particleSize, particleSize, particleSize);
        }
    }

    void ScatterParticlesToGrid()
    {
        Vector2[,] newVelocityField = new Vector2[resolutionX + 1, resolutionY + 1];
        float[,] weights = new float[resolutionX + 1, resolutionY + 1];

        foreach (Particle particle in particles)
        {
            float gridX = (particle.position.x / boundsSize.x + 0.5f) * resolutionX;
            float gridY = (particle.position.y / boundsSize.y + 0.5f) * resolutionY;

            int x0 = Mathf.FloorToInt(gridX);
            int x1 = Mathf.Clamp(x0 + 1, 0, resolutionX);
            int y0 = Mathf.FloorToInt(gridY);
            int y1 = Mathf.Clamp(y0 + 1, 0, resolutionY);

            float sx = gridX - x0;
            float sy = gridY - y0;

            newVelocityField[x0, y0] += (1 - sx) * (1 - sy) * particle.velocity;
            newVelocityField[x0, y1] += (1 - sx) * sy * particle.velocity;
            newVelocityField[x1, y0] += sx * (1 - sy) * particle.velocity;
            newVelocityField[x1, y1] += sx * sy * particle.velocity;

            weights[x0, y0] += (1 - sx) * (1 - sy);
            weights[x0, y1] += (1 - sx) * sy;
            weights[x1, y0] += sx * (1 - sy);
            weights[x1, y1] += sx * sy;
        }

        for (int y = 0; y < resolutionY + 1; y++)
        {
            for (int x = 0; x < resolutionX + 1; x++)
            {
                if (weights[x, y] > 0)
                {
                    newVelocityField[x, y] /= weights[x, y];
                }
                else
                {
                    newVelocityField[x, y] = Vector2.zero;
                }
                velocityField[x, y] = newVelocityField[x, y];
            }
        }
    }

    void GatherGridToParticles()
    {
        foreach (Particle particle in particles)
        {
            float gridX = (particle.position.x / boundsSize.x + 0.5f) * resolutionX;
            float gridY = (particle.position.y / boundsSize.y + 0.5f) * resolutionY;

            int x0 = Mathf.FloorToInt(gridX);
            int x1 = Mathf.Clamp(x0 + 1, 0, resolutionX);
            int y0 = Mathf.FloorToInt(gridY);
            int y1 = Mathf.Clamp(y0 + 1, 0, resolutionY);

            float sx = gridX - x0;
            float sy = gridY - y0;

            Vector2 v00 = velocityField[x0, y0];
            Vector2 v01 = velocityField[x0, y1];
            Vector2 v10 = velocityField[x1, y0];
            Vector2 v11 = velocityField[x1, y1];

            Vector2 interpolatedVelocity =
                (1 - sx) * (1 - sy) * v00 +
                (1 - sx) * sy * v01 +
                sx * (1 - sy) * v10 +
                sx * sy * v11;

            Vector2 prevV00 = prevVelocityField[x0, y0];
            Vector2 prevV01 = prevVelocityField[x0, y1];
            Vector2 prevV10 = prevVelocityField[x1, y0];
            Vector2 prevV11 = prevVelocityField[x1, y1];

            Vector2 interpolatedPrevVelocity =
                (1 - sx) * (1 - sy) * prevV00 +
                (1 - sx) * sy * prevV01 +
                sx * (1 - sy) * prevV10 +
                sx * sy * prevV11;

            Vector2 picVelocity = interpolatedVelocity;
            Vector2 flipVelocity = particle.velocity + (interpolatedVelocity - interpolatedPrevVelocity);

            particle.velocity = alpha * picVelocity + (1 - alpha) * flipVelocity;
        }
    }

    void UpdateParticlePositions()
    {
        Vector2 halfBoundsSize = boundsSize / 2;
        float maxGridSize = Mathf.Min(boundsSize.x / resolutionX, boundsSize.y / resolutionY);
        float substepTime = deltaTime / 5;

        foreach (Particle particle in particles)
        {
            for (int i = 0; i < 5; i++)
            {
                float cflCondition = Mathf.Min(substepTime, maxGridSize / particle.velocity.magnitude);

                Vector2 velocityAtStart = particle.velocity;
                Vector2 positionAtMidpoint = particle.position + velocityAtStart * (cflCondition / 2);
                Vector2 midpointVelocity = velocityAtStart + new Vector2(0, gravity) * (cflCondition / 2);

                Vector2 velocityAtMidpoint = midpointVelocity;
                particle.position += velocityAtMidpoint * cflCondition;
                particle.velocity += new Vector2(0, gravity) * cflCondition;

                if (particle.position.x < -halfBoundsSize.x || particle.position.x > halfBoundsSize.x)
                {
                    particle.velocity.x *= -dampingFactor;
                    particle.position.x = Mathf.Clamp(particle.position.x, -halfBoundsSize.x, halfBoundsSize.x);
                }

                if (particle.position.y < -halfBoundsSize.y || particle.position.y > halfBoundsSize.y)
                {
                    particle.velocity.y *= -dampingFactor;
                    particle.position.y = Mathf.Clamp(particle.position.y, -halfBoundsSize.y, halfBoundsSize.y);
                }
            }

            particle.gameObject.transform.position = new Vector3(particle.position.x, particle.position.y, 0);
        }
    }

    void PressureProjectionStep()
    {
        float dx = 1.0f / resolutionY;
        float beta = 4.0f / (dx * dx);

        ComputeDivergence();
        SolvePressurePoissonCG(beta);
        SubtractPressureGradient();
        CopyVelocityField();
    }

    void ComputeDivergence()
    {
        for (int y = 1; y < resolutionY; y++)
        {
            for (int x = 1; x < resolutionX; x++)
            {
                divField[x, y] = -0.5f * (velocityField[x + 1, y].x - velocityField[x - 1, y].x +
                                          velocityField[x, y + 1].y - velocityField[x, y - 1].y);
            }
        }
    }

    void SolvePressurePoissonCG(float beta)
    {
        int maxIterations = 100;
        float tolerance = 1e-5f;
        float[,] residual = new float[resolutionX + 1, resolutionY + 1];
        float[,] direction = new float[resolutionX + 1, resolutionY + 1];
        float[,] temp = new float[resolutionX + 1, resolutionY + 1];

        // Initialize pressure field to zero
        for (int y = 0; y < resolutionY + 1; y++)
        {
            for (int x = 0; x < resolutionX + 1; x++)
            {
                pressureField[x, y] = 0;
            }
        }

        // Compute initial residual and direction
        for (int y = 1; y < resolutionY; y++)
        {
            for (int x = 1; x < resolutionX; x++)
            {
                float laplacian = -4.0f * pressureField[x, y] +
                                  pressureField[x + 1, y] + pressureField[x - 1, y] +
                                  pressureField[x, y + 1] + pressureField[x, y - 1];
                residual[x, y] = divField[x, y] - beta * laplacian;
                direction[x, y] = residual[x, y];
            }
        }

        float residualDot = DotProduct(residual, residual);
        if (residualDot < tolerance) return;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            // Apply laplacian to direction
            for (int y = 1; y < resolutionY; y++)
            {
                for (int x = 1; x < resolutionX; x++)
                {
                    temp[x, y] = -4.0f * direction[x, y] +
                                 direction[x + 1, y] + direction[x - 1, y] +
                                 direction[x, y + 1] + direction[x, y - 1];
                }
            }

            float directionDot = DotProduct(direction, temp);
            float alpha = residualDot / directionDot;

            // Update pressure field
            for (int y = 1; y < resolutionY; y++)
            {
                for (int x = 1; x < resolutionX; x++)
                {
                    pressureField[x, y] += alpha * direction[x, y];
                }
            }

            // Update residual
            for (int y = 1; y < resolutionY; y++)
            {
                for (int x = 1; x < resolutionX; x++)
                {
                    residual[x, y] -= alpha * temp[x, y];
                }
            }

            float newResidualDot = DotProduct(residual, residual);
            if (newResidualDot < tolerance) break;

            float betaCoeff = newResidualDot / residualDot;
            residualDot = newResidualDot;

            // Update direction
            for (int y = 1; y < resolutionY; y++)
            {
                for (int x = 1; x < resolutionX; x++)
                {
                    direction[x, y] = residual[x, y] + betaCoeff * direction[x, y];
                }
            }
        }
    }

    float DotProduct(float[,] a, float[,] b)
    {
        float result = 0;
        for (int y = 1; y < resolutionY; y++)
        {
            for (int x = 1; x < resolutionX; x++)
            {
                result += a[x, y] * b[x, y];
            }
        }
        return result;
    }

    void SubtractPressureGradient()
    {
        for (int y = 1; y < resolutionY; y++)
        {
            for (int x = 1; x < resolutionX; x++)
            {
                velocityField[x, y] -= 0.5f * new Vector2(pressureField[x + 1, y] - pressureField[x - 1, y],
                                                          pressureField[x, y + 1] - pressureField[x, y - 1]);
            }
        }
    }

    void CopyVelocityField()
    {
        for (int y = 0; y < resolutionY + 1; y++)
        {
            for (int x = 0; x < resolutionX + 1; x++)
            {
                prevVelocityField[x, y] = velocityField[x, y];
            }
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boundsSize.x, boundsSize.y, 0));
    }
}
