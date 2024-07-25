using UnityEngine;

public class StableFluids : MonoBehaviour
{
    [SerializeField] int resolutionX = 16;
    [SerializeField] int resolutionY = 9;
    [SerializeField] float deltaTime = 0.01f;
    [SerializeField] float cellSize = 1.0f;
    [SerializeField] bool visualizeFluidCells = true;
    [SerializeField] bool randomizeVelocity = true;
    [SerializeField] float maxInitialVelocity = 1.0f;
    [SerializeField] float viscosity = 0.1f;

    private Vector2[,] velocityField;
    private Vector2[,] tempField;
    private float[,] pressureField;
    private float[,] divField;
    private Color[,] colorField;

    void Start()
    {
        InitializeFields();
        if (randomizeVelocity)
        {
            ApplyRandomInitialConditions();
        }
        else
        {
            ApplyInitialConditions();
        }
    }

    void Update()
    {
        AdvectionStep();
        DiffusionStep();
        PressureProjectionStep();
        UpdateColorField();
    }

    void InitializeFields()
    {
        velocityField = new Vector2[resolutionX, resolutionY];
        tempField = new Vector2[resolutionX, resolutionY];
        pressureField = new float[resolutionX, resolutionY];
        divField = new float[resolutionX, resolutionY];
        colorField = new Color[resolutionX, resolutionY];
    }

    void ApplyInitialConditions()
    {
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                velocityField[x, y] = Vector2.zero;
            }
        }
    }

    void ApplyRandomInitialConditions()
    {
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                float randomX = Random.Range(-maxInitialVelocity, maxInitialVelocity);
                float randomY = Random.Range(-maxInitialVelocity, maxInitialVelocity);
                velocityField[x, y] = new Vector2(randomX, randomY);
            }
        }
    }

    void AdvectionStep()
    {
        float inverseResolutionX = 1.0f / resolutionX;
        float inverseResolutionY = 1.0f / resolutionY;
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                Vector2 pos = new Vector2(x * inverseResolutionX, y * inverseResolutionY);
                Vector2 velocity = velocityField[x, y];
                Vector2 prevPos = pos - new Vector2(velocity.x * inverseResolutionX, velocity.y * inverseResolutionY) * deltaTime;

                prevPos.x = Mathf.Clamp(prevPos.x, 0, 1);
                prevPos.y = Mathf.Clamp(prevPos.y, 0, 1);

                Vector2 sampledVelocity = BilinearSample(velocityField, prevPos.x * resolutionX, prevPos.y * resolutionY);
                tempField[x, y] = sampledVelocity;
            }
        }

        // Swap the fields
        var swapTemp = velocityField;
        velocityField = tempField;
        tempField = swapTemp;
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

    void DiffusionStep()
    {
        float alpha = (cellSize * cellSize) / (viscosity * deltaTime);
        float beta = 4.0f + alpha;

        for (int i = 0; i < 20; i++) // Perform multiple Jacobi iterations
        {
            JacobiVector(velocityField, tempField, alpha, beta);
            Swap(ref velocityField, ref tempField);
        }
    }

    void PressureProjectionStep()
    {
        float alpha = -(cellSize * cellSize);
        float beta = 4.0f;

        // Compute divergence of velocity field
        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int x = 1; x < resolutionX - 1; x++)
            {
                divField[x, y] = -0.5f * (velocityField[x + 1, y].x - velocityField[x - 1, y].x +
                                          velocityField[x, y + 1].y - velocityField[x, y - 1].y) / cellSize;
                pressureField[x, y] = 0;
            }
        }

        for (int i = 0; i < 20; i++) // Perform multiple Jacobi iterations
        {
            JacobiScalar(pressureField, divField, alpha, beta);
        }

        // Subtract pressure gradient from velocity field
        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int x = 1; x < resolutionX - 1; x++)
            {
                velocityField[x, y] -= 0.5f * new Vector2(pressureField[x + 1, y] - pressureField[x - 1, y],
                                                          pressureField[x, y + 1] - pressureField[x, y - 1]) / cellSize;
            }
        }
    }

    void JacobiVector(Vector2[,] x, Vector2[,] b, float alpha, float beta)
    {
        for (int y = 1; y < resolutionY - 1; y++)
        {
            for (int xCoord = 1; xCoord < resolutionX - 1; xCoord++)
            {
                tempField[xCoord, y] = (b[xCoord, y] + alpha * (x[xCoord + 1, y] + x[xCoord - 1, y] +
                                                                x[xCoord, y + 1] + x[xCoord, y - 1])) / beta;
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

    void Swap(ref Vector2[,] a, ref Vector2[,] b)
    {
        var temp = a;
        a = b;
        b = temp;
    }

    void UpdateColorField()
    {
        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                Vector2 velocity = velocityField[x, y];
                colorField[x, y] = new Color(velocity.x, velocity.y, 0.0f, 1.0f);
            }
        }
    }

    void OnDrawGizmos()
    {
        if (velocityField == null) return;

        Gizmos.color = Color.white;
        float inverseResolutionX = 1.0f / resolutionX;
        float inverseResolutionY = 1.0f / resolutionY;
        Vector3 bottomLeft = transform.position - new Vector3(resolutionX * cellSize * 0.5f, resolutionY * cellSize * 0.5f, 0);

        for (int y = 0; y < resolutionY; y++)
        {
            for (int x = 0; x < resolutionX; x++)
            {
                Vector2 velocity = velocityField[x, y];
                Vector3 pos = bottomLeft + new Vector3(x * cellSize, y * cellSize, 0);
                Gizmos.color = Color.white;
                Gizmos.DrawWireCube(pos, new Vector3(cellSize, cellSize, 0));

                if (visualizeFluidCells)
                {
                    Gizmos.color = colorField[x, y];
                    Gizmos.DrawCube(pos, new Vector3(cellSize * 0.9f, cellSize * 0.9f, 0));
                }
            }
        }
    }
}
