using UnityEngine;
using TMPro;
using System.Collections.Generic;

public class Ice : MonoBehaviour
{
    [SerializeField] int resolutionX = 512;
    [SerializeField] int resolutionY = 512;
    [SerializeField] float deltaTime = 0.01f;
    [SerializeField] int numParticles = 10000;
    [SerializeField] float maxInitialVelocity = 1.0f;
    [SerializeField] Shader shader;
    [SerializeField] Texture2D initialTexture;
    [SerializeField] float force = 300;
    [SerializeField] TextMeshProUGUI velocityText;

    private List<Particle> particles;
    private Vector2[,] velocityField;
    private Vector2[,] prevVelocityField;
    private float[,] pressureField;
    private float[,] divField;

    private Material shaderMaterial;
    private RenderTexture velocityTexture;
    private RenderTexture colorRT1;
    private RenderTexture colorRT2;
    private bool isInitialized;

    public class Particle
    {
        public Vector2 position;
        public Vector2 velocity;

        public Particle(Vector2 pos, Vector2 vel)
        {
            position = pos;
            velocity = vel;
        }
    }

    void Start()
    {
        InitializeFields();
        InitializeParticles();

        shaderMaterial = new Material(shader);

        velocityTexture = AllocateRenderTexture(2, resolutionX, resolutionY);
        colorRT1 = AllocateRenderTexture(4, Screen.width, Screen.height);
        colorRT2 = AllocateRenderTexture(4, Screen.width, Screen.height);

        // Initialize colorRT1 with the initial texture
        if (initialTexture != null)
        {
            Graphics.Blit(initialTexture, colorRT1);
        }

        UpdateVelocityTexture();
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
        for (int i = 0; i < numParticles; i++)
        {
            Vector2 randomPosition = new Vector2(Random.Range(0f, 1f), Random.Range(0f, 1f));
            Vector2 randomVelocity = new Vector2(Random.Range(-maxInitialVelocity, maxInitialVelocity), Random.Range(-maxInitialVelocity, maxInitialVelocity));
            particles.Add(new Particle(randomPosition, randomVelocity));
        }
    }

    void Update()
    {
        if (!isInitialized) return;

        ScatterParticlesToGrid();
        PressureProjectionStep();
        GatherGridToParticles();
        UpdateParticlePositions();
        UpdateVelocityTexture();

        if (Input.GetMouseButtonDown(0))
        {
            DisplayVelocityAtMousePosition();
        }
    }

    void ScatterParticlesToGrid()
    {
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                velocityField[x, y] = Vector2.zero;
            }
        }

        foreach (Particle particle in particles)
        {
            int x = Mathf.FloorToInt(particle.position.x * resolutionX);
            int y = Mathf.FloorToInt(particle.position.y * resolutionY);

            if (x >= 0 && x < resolutionX && y >= 0 && y < resolutionY)
            {
                velocityField[x, y] += particle.velocity;
            }
        }
    }

    void GatherGridToParticles()
    {
        foreach (Particle particle in particles)
        {
            int x = Mathf.FloorToInt(particle.position.x * resolutionX);
            int y = Mathf.FloorToInt(particle.position.y * resolutionY);

            if (x >= 0 && x < resolutionX && y >= 0 && y < resolutionY)
            {
                Vector2 newGridVelocity = BilinearSample(velocityField, particle.position.x * resolutionX, particle.position.y * resolutionY);
                Vector2 picVelocity = newGridVelocity;
                Vector2 flipVelocity = particle.velocity + (newGridVelocity - BilinearSample(prevVelocityField, particle.position.x * resolutionX, particle.position.y * resolutionY));
                particle.velocity = 0.95f * flipVelocity + 0.05f * picVelocity;
            }
        }
    }

    void UpdateParticlePositions()
    {
        foreach (Particle particle in particles)
        {
            particle.position += particle.velocity * deltaTime;

            // Handle boundary conditions
            particle.position.x = Mathf.Clamp(particle.position.x, 0f, 1f);
            particle.position.y = Mathf.Clamp(particle.position.y, 0f, 1f);
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

        for (int i = 0; i < 20; i++)
        {
            JacobiScalar(pressureField, divField, alpha, beta);
        }

        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int x = 1; x < resolutionX - 1; x++)
            {
                velocityField[x, y] -= 0.5f * new Vector2(pressureField[x + 1, y] - pressureField[x - 1, y],
                                                          pressureField[x, y + 1] - pressureField[x, y - 1]);
            }
        }

        // Save the previous velocity field for FLIP calculations
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                prevVelocityField[x, y] = velocityField[x, y];
            }
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

    void UpdateVelocityTexture()
    {
        Texture2D texture = new Texture2D(resolutionX, resolutionY, TextureFormat.RGFloat, false);
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                Vector2 velocity = velocityField[x, y];
                texture.SetPixel(x, y, new Color(velocity.x, velocity.y, 0.0f, 1.0f));
            }
        }
        texture.Apply();
        Graphics.Blit(texture, velocityTexture);
        Destroy(texture);
    }

    RenderTexture AllocateRenderTexture(int componentCount, int width, int height)
    {
        var format = RenderTextureFormat.ARGBFloat;
        if (componentCount == 1) format = RenderTextureFormat.RFloat;
        if (componentCount == 2) format = RenderTextureFormat.RGFloat;

        var rt = new RenderTexture(width, height, 0, format);
        rt.enableRandomWrite = true;
        rt.Create();
        return rt;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        shaderMaterial.SetTexture("_VelocityField", velocityTexture);
        shaderMaterial.SetTexture("_MainTex", colorRT1);

        Graphics.Blit(colorRT1, colorRT2, shaderMaterial, 0);

        // Swap the color buffers
        var temp = colorRT1;
        colorRT1 = colorRT2;
        colorRT2 = temp;

        Graphics.Blit(colorRT1, destination, shaderMaterial, 1);
    }

    void DisplayVelocityAtMousePosition()
    {
        Vector2 mousePosition = Input.mousePosition;
        int x = Mathf.FloorToInt(mousePosition.x / Screen.width * resolutionX);
        int y = Mathf.FloorToInt(mousePosition.y / Screen.height * resolutionY);

        if (x >= 0 && x < resolutionX && y >= 0 && y < resolutionY)
        {
            Vector2 velocity = velocityField[x, y];
            if (velocityText != null)
            {
                velocityText.text = $"Velocity at ({x}, {y}): {velocity}";
            }
        }
    }
}
