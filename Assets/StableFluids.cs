using UnityEngine;

public class StableFluids : MonoBehaviour
{
    public int gridSize = 128;
    public float timeStep = 0.1f;
    public float diffusion = 0.0001f;
    public float viscosity = 0.0001f;

    private float[,] velocityX, velocityY, velocityX0, velocityY0;
    private float[,] density, density0;

    void Start()
    {
        velocityX = new float[gridSize, gridSize];
        velocityY = new float[gridSize, gridSize];
        velocityX0 = new float[gridSize, gridSize];
        velocityY0 = new float[gridSize, gridSize];
        density = new float[gridSize, gridSize];
        density0 = new float[gridSize, gridSize];
    }

    void Update()
    {
        HandleInput();

        Step();

        RenderDensity();
    }

    void HandleInput()
    {
        if (Input.GetMouseButton(0))
        {
            Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            int x = Mathf.Clamp((int)mousePos.x, 1, gridSize - 2);
            int y = Mathf.Clamp((int)mousePos.y, 1, gridSize - 2);

            density[x, y] += 100.0f;
            velocityX[x, y] += 10.0f;
            velocityY[x, y] += 10.0f;
        }
    }

    void Step()
    {
        Diffuse(1, velocityX0, velocityX, viscosity, timeStep);
        Diffuse(2, velocityY0, velocityY, viscosity, timeStep);

        Project(velocityX0, velocityY0, velocityX, velocityY);

        Advect(1, velocityX, velocityX0, velocityX0, velocityY0, timeStep);
        Advect(2, velocityY, velocityY0, velocityX0, velocityY0, timeStep);

        Project(velocityX, velocityY, velocityX0, velocityY0);

        Diffuse(0, density0, density, diffusion, timeStep);
        Advect(0, density, density0, velocityX, velocityY, timeStep);
    }

    void Diffuse(int b, float[,] x, float[,] x0, float diff, float dt)
    {
        float a = dt * diff * (gridSize - 2) * (gridSize - 2);
        LinearSolve(b, x, x0, a, 1 + 4 * a);
    }

    void LinearSolve(int b, float[,] x, float[,] x0, float a, float c)
    {
        for (int k = 0; k < 20; k++)
        {
            for (int i = 1; i < gridSize - 1; i++)
            {
                for (int j = 1; j < gridSize - 1; j++)
                {
                    x[i, j] = (x0[i, j] + a * (x[i + 1, j] + x[i - 1, j] + x[i, j + 1] + x[i, j - 1])) / c;
                }
            }
            SetBoundary(b, x);
        }
    }

    void Advect(int b, float[,] d, float[,] d0, float[,] u, float[,] v, float dt)
    {
        float dt0 = dt * (gridSize - 2);
        for (int i = 1; i < gridSize - 1; i++)
        {
            for (int j = 1; j < gridSize - 1; j++)
            {
                float x = i - dt0 * u[i, j];
                float y = j - dt0 * v[i, j];

                if (x < 0.5f) x = 0.5f;
                if (x > gridSize - 2 + 0.5f) x = gridSize - 2 + 0.5f;
                int i0 = (int)x;
                int i1 = i0 + 1;

                if (y < 0.5f) y = 0.5f;
                if (y > gridSize - 2 + 0.5f) y = gridSize - 2 + 0.5f;
                int j0 = (int)y;
                int j1 = j0 + 1;

                float s1 = x - i0;
                float s0 = 1 - s1;
                float t1 = y - j0;
                float t0 = 1 - t1;

                d[i, j] = s0 * (t0 * d0[i0, j0] + t1 * d0[i0, j1]) + s1 * (t0 * d0[i1, j0] + t1 * d0[i1, j1]);
            }
        }
        SetBoundary(b, d);
    }

    void Project(float[,] u, float[,] v, float[,] p, float[,] div)
    {
        for (int i = 1; i < gridSize - 1; i++)
        {
            for (int j = 1; j < gridSize - 1; j++)
            {
                div[i, j] = -0.5f * (u[i + 1, j] - u[i - 1, j] + v[i, j + 1] - v[i, j - 1]) / gridSize;
                p[i, j] = 0;
            }
        }
        SetBoundary(0, div);
        SetBoundary(0, p);

        LinearSolve(0, p, div, 1, 4);

        for (int i = 1; i < gridSize - 1; i++)
        {
            for (int j = 1; j < gridSize - 1; j++)
            {
                u[i, j] -= 0.5f * gridSize * (p[i + 1, j] - p[i - 1, j]);
                v[i, j] -= 0.5f * gridSize * (p[i, j + 1] - p[i, j - 1]);
            }
        }
        SetBoundary(1, u);
        SetBoundary(2, v);
    }

    void SetBoundary(int b, float[,] x)
    {
        for (int i = 1; i < gridSize - 1; i++)
        {
            x[i, 0] = b == 2 ? -x[i, 1] : x[i, 1];
            x[i, gridSize - 1] = b == 2 ? -x[i, gridSize - 2] : x[i, gridSize - 2];
            x[0, i] = b == 1 ? -x[1, i] : x[1, i];
            x[gridSize - 1, i] = b == 1 ? -x[gridSize - 2, i] : x[gridSize - 2, i];
        }
        x[0, 0] = 0.5f * (x[1, 0] + x[0, 1]);
        x[0, gridSize - 1] = 0.5f * (x[1, gridSize - 1] + x[0, gridSize - 2]);
        x[gridSize - 1, 0] = 0.5f * (x[gridSize - 2, 0] + x[gridSize - 1, 1]);
        x[gridSize - 1, gridSize - 1] = 0.5f * (x[gridSize - 2, gridSize - 1] + x[gridSize - 1, gridSize - 2]);
    }

    void RenderDensity()
    {
        Texture2D texture = new Texture2D(gridSize, gridSize);
        for (int i = 0; i < gridSize; i++)
        {
            for (int j = 0; j < gridSize; j++)
            {
                float d = density[i, j];
                texture.SetPixel(i, j, new Color(d, d, d));
            }
        }
        texture.Apply();

        // Apply the texture to a quad or a plane to visualize the density
        GetComponent<Renderer>().material.mainTexture = texture;
    }
}
