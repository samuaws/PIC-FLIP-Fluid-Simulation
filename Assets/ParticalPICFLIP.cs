using UnityEngine;
using System.Collections.Generic;

public class ParticalPICFLIP : MonoBehaviour
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
        velocityField = new Vector2[resolutionX, resolutionY];
        prevVelocityField = new Vector2[resolutionX, resolutionY];
        pressureField = new float[resolutionX, resolutionY];
        divField = new float[resolutionX, resolutionY];
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
        Vector2[,] newVelocityField = new Vector2[resolutionX, resolutionY];
        float[,] weights = new float[resolutionX, resolutionY];

        foreach (Particle particle in particles)
        {
            float gridX = (particle.position.x / boundsSize.x + 0.5f) * (resolutionX - 1);
            float gridY = (particle.position.y / boundsSize.y + 0.5f) * (resolutionY - 1);

            int x0 = Mathf.FloorToInt(gridX);
            int x1 = Mathf.Clamp(x0 + 1, 0, resolutionX - 1);
            int y0 = Mathf.FloorToInt(gridY);
            int y1 = Mathf.Clamp(y0 + 1, 0, resolutionY - 1);

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

        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                if (weights[x, y] > 0)
                {
                    newVelocityField[x, y] /= weights[x, y];
                    velocityField[x, y] = newVelocityField[x, y];
                }
            }
        }
    }

    void GatherGridToParticles()
    {
        foreach (Particle particle in particles)
        {
            float gridX = (particle.position.x / boundsSize.x + 0.5f) * (resolutionX - 1);
            float gridY = (particle.position.y / boundsSize.y + 0.5f) * (resolutionY - 1);

            int x0 = Mathf.FloorToInt(gridX);
            int x1 = Mathf.Clamp(x0 + 1, 0, resolutionX - 1);
            int y0 = Mathf.FloorToInt(gridY);
            int y1 = Mathf.Clamp(y0 + 1, 0, resolutionY - 1);

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

            particle.velocity = 0.95f * flipVelocity + 0.05f * picVelocity;
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
        float alpha = -dx * dx;
        float beta = 4.0f;

        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int x = 1; x < resolutionX - 1; x++)
            {
                divField[x, y] = -0.5f * (velocityField[x + 1, y].x - velocityField[x - 1, y].x +
                                          velocityField[x, y + 1].y - velocityField[x, y - 1].y);
                pressureField[x, y] = 0;
            }
        }

        ApplyBoundaryConditions(divField);
        ApplyBoundaryConditions(pressureField);

        for (int i = 0; i < 50; i++)
        {
            JacobiScalar(pressureField, divField, alpha, beta);
            ApplyBoundaryConditions(pressureField);
        }

        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int x = 1; x < resolutionX - 1; x++)
            {
                velocityField[x, y] -= 0.5f * new Vector2(pressureField[x + 1, y] - pressureField[x - 1, y],
                                                          pressureField[x, y + 1] - pressureField[x, y - 1]);
            }
        }

        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                prevVelocityField[x, y] = velocityField[x, y];
            }
        }
    }

    void ApplyBoundaryConditions(float[,] field)
    {
        for (int x = 0; x < resolutionX; x++)
        {
            field[x, 0] = field[x, 1];
            field[x, resolutionY - 1] = field[x, resolutionY - 2];
        }

        for (int y = 0; y < resolutionY; y++)
        {
            field[0, y] = field[1, y];
            field[resolutionX - 1, y] = field[resolutionX - 2 , y];
        }
    }

    void JacobiScalar(float[,] x, float[,] b, float alpha, float beta)
    {
        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int xCoord = 1; xCoord < resolutionX - 1; xCoord++)
            {
                pressureField[xCoord, y] = (b[xCoord, y] + alpha * (x[xCoord + 1, y] + x[xCoord - 1, y] +
                                                                    x[xCoord, y + 1] + x[xCoord, y - 1])) / beta;
            }
        }
    }

    Vector2 BilinearSample(Vector2[,] field, float x, float y)
    {
        int x1 = Mathf.FloorToInt(x);
        int x2 = Mathf.Clamp(x1 + 1, 0, resolutionX - 1);
        int y1 = Mathf.FloorToInt(y);
        int y2 = Mathf.Clamp(y1 + 1, 0, resolutionY - 1);

        x1 = Mathf.Clamp(x1, 0, resolutionX - 1);
        y1 = Mathf.Clamp(y1, 0, resolutionY - 1);

        float t1 = x - x1;
        float t2 = y - y1;

        Vector2 v11 = field[x1, y1];
        Vector2 v12 = field[x1, y2];
        Vector2 v21 = field[x2, y1];
        Vector2 v22 = field[x2, y2];

        Vector2 v1 = Vector2.Lerp(v11, v12, t2);
        Vector2 v2 = Vector2.Lerp(v21, v22, t2);

        return Vector2.Lerp(v1, v2, t1);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boundsSize.x, boundsSize.y, 0));
    }
}
