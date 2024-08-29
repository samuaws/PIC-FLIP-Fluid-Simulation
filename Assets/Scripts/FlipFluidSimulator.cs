using UnityEngine;

public class FlipFluidSimulator : MonoBehaviour
{
    public float gravity = -9.81f;
    public float dt = 1.0f / 120.0f;
    [Range(0,1)]
    public float picFlipRatio = 0.9f;
    public int numPressureIters = 100;
    public int numParticleIters = 2;
    public int frameNr = 0;
    public float overRelaxation = 1.9f;
    public bool compensateDrift = true;
    public bool separateParticles = true;
    public bool paused = true;
    public bool showParticles = true;
    public bool showGrid = false;
    public FlipFluid fluid;

    [Header("Particle Settings")]
    public int numParticles = 1000; // Control the number of particles from the Inspector

    private void Start()
    {
        SetupScene();
        InvokeRepeating("UpdateSimulation", 0f, dt);
    }

    private void Update()
    {
        HandleInput();
    }

    private void OnDrawGizmos()
    {
        if (fluid != null)
        {
            // Draw particles
            if (showParticles)
            {
                Gizmos.color = Color.cyan;
                for (int i = 0; i < fluid.numParticles; i++)
                {
                    Vector3 pos = new Vector3(fluid.particlePos[2 * i], fluid.particlePos[2 * i + 1], 0);
                    Gizmos.DrawSphere(pos, fluid.particleRadius);
                }
            }

            // Draw grid
            if (showGrid)
            {
                Gizmos.color = Color.gray;
                for (int i = 0; i < fluid.fNumX; i++)
                {
                    for (int j = 0; j < fluid.fNumY; j++)
                    {
                        if (fluid.cellType[i * fluid.fNumY + j] == 0)
                        {
                            Vector3 center = new Vector3((i + 0.5f) * fluid.h, (j + 0.5f) * fluid.h, 0);
                            Gizmos.DrawWireCube(center, new Vector3(fluid.h, fluid.h, 0));
                        }
                    }
                }
            }
        }
    }

    private void SetupScene()
    {
        dt = 1.0f / 60.0f;
        numPressureIters = 50;
        numParticleIters = 2;

        int res = 100;

        float tankHeight = 1.0f * Camera.main.orthographicSize * 2.0f;
        float tankWidth = tankHeight * Camera.main.aspect;
        float h = tankHeight / res;
        float density = 1000.0f;

        float relWaterHeight = 0.8f;
        float relWaterWidth = 0.6f;

        float r = 0.3f * h; // particle radius w.r.t. cell size
        float dx = 2.0f * r;
        float dy = Mathf.Sqrt(3.0f) / 2.0f * dx;

        int numX = Mathf.FloorToInt((relWaterWidth * tankWidth - 2.0f * h - 2.0f * r) / dx);
        int numY = Mathf.FloorToInt((relWaterHeight * tankHeight - 2.0f * h - 2.0f * r) / dy);
        int maxParticles = numX * numY;

        fluid = new FlipFluid(density, tankWidth, tankHeight, h, r, numParticles > maxParticles ? maxParticles : numParticles);

        fluid.numParticles = Mathf.Min(numParticles, maxParticles); // Set based on the inspector value
        int p = 0;
        for (int i = 0; i < numX && p < fluid.numParticles; i++)
        {
            for (int j = 0; j < numY && p < fluid.numParticles; j++)
            {
                fluid.particlePos[p++] = h + r + dx * i + (j % 2 == 0 ? 0.0f : r);
                fluid.particlePos[p++] = h + r + dy * j;
            }
        }

        int n = fluid.fNumY;

        for (int i = 0; i < fluid.fNumX; i++)
        {
            for (int j = 0; j < fluid.fNumY; j++)
            {
                float s_val = 1.0f; // fluid
                if (i == 0 || i == fluid.fNumX - 1 || j == 0)
                    s_val = 0.0f; // solid
                fluid.s[i * n + j] = s_val;
            }
        }
    }

    private void UpdateSimulation()
    {
        if (!paused)
        {
            fluid.Simulate(
                dt, gravity, picFlipRatio, numPressureIters, numParticleIters,
                overRelaxation, compensateDrift, separateParticles);
            frameNr++;
        }
    }

    private void HandleInput()
    {
        if (Input.GetMouseButton(0))
        {
            Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            print(mousePos);
        }
    }
}

public class FlipFluid
{
    public float density;
    public int fNumX, fNumY, fNumCells;
    public float h, fInvSpacing;
    public float[] u, v, du, dv, prevU, prevV, p, s;
    public int[] cellType;

    public int maxParticles;
    public float[] particlePos, particleVel, particleDensity;
    public float particleRadius, pInvSpacing, particleRestDensity;
    public int pNumX, pNumY, pNumCells;
    public int[] numCellParticles, firstCellParticle, cellParticleIds;

    public int numParticles;

    public FlipFluid(float density, float width, float height, float spacing, float particleRadius, int maxParticles)
    {
        this.density = density;
        fNumX = Mathf.FloorToInt(width / spacing) + 1;
        fNumY = Mathf.FloorToInt(height / spacing) + 1;
        h = Mathf.Max(width / fNumX, height / fNumY);
        fInvSpacing = 1.0f / h;
        fNumCells = fNumX * fNumY;

        u = new float[fNumCells];
        v = new float[fNumCells];
        du = new float[fNumCells];
        dv = new float[fNumCells];
        prevU = new float[fNumCells];
        prevV = new float[fNumCells];
        p = new float[fNumCells];
        s = new float[fNumCells];
        cellType = new int[fNumCells];

        this.maxParticles = maxParticles;
        particlePos = new float[2 * maxParticles];
        particleVel = new float[2 * maxParticles];
        particleDensity = new float[fNumCells];
        particleRestDensity = 0.0f;

        this.particleRadius = particleRadius;
        pInvSpacing = 1.0f / (2.2f * particleRadius);
        pNumX = Mathf.FloorToInt(width * pInvSpacing) + 1;
        pNumY = Mathf.FloorToInt(height * pInvSpacing) + 1;
        pNumCells = pNumX * pNumY;

        numCellParticles = new int[pNumCells];
        firstCellParticle = new int[pNumCells + 1];
        cellParticleIds = new int[maxParticles];

        numParticles = 0;
    }

    public void Simulate(float dt, float gravity, float picFlipRatio, int numPressureIters, int numParticleIters, float overRelaxation, bool compensateDrift, bool separateParticles)
    {
        int numSubSteps = 1;
        float sdt = dt / numSubSteps;

        for (int step = 0; step < numSubSteps; step++)
        {
            IntegrateParticles(sdt, gravity);
            if (separateParticles)
                PushParticlesApart(numParticleIters);
            HandleParticleCollisions();
            TransferVelocities(true, picFlipRatio);
            UpdateParticleDensity();
            SolveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
            TransferVelocities(false, picFlipRatio);
        }
    }

    public void IntegrateParticles(float dt, float gravity)
    {
        for (int i = 0; i < numParticles; i++)
        {
            particleVel[2 * i + 1] += dt * gravity;
            particlePos[2 * i] += particleVel[2 * i] * dt;
            particlePos[2 * i + 1] += particleVel[2 * i + 1] * dt;
        }
    }

    public void PushParticlesApart(int numIters)
    {
        numCellParticles = new int[pNumCells];
        firstCellParticle = new int[pNumCells + 1];
        cellParticleIds = new int[maxParticles];

        for (int i = 0; i < numParticles; i++)
        {
            float x = particlePos[2 * i];
            float y = particlePos[2 * i + 1];

            int xi = Mathf.Clamp(Mathf.FloorToInt(x * pInvSpacing), 0, pNumX - 1);
            int yi = Mathf.Clamp(Mathf.FloorToInt(y * pInvSpacing), 0, pNumY - 1);
            int cellNr = xi * pNumY + yi;
            numCellParticles[cellNr]++;
        }

        int firstIndex = 0;

        for (int i = 0; i < pNumCells; i++)
        {
            firstIndex += numCellParticles[i];
            firstCellParticle[i] = firstIndex;
        }
        firstCellParticle[pNumCells] = firstIndex;

        for (int i = 0; i < numParticles; i++)
        {
            float x = particlePos[2 * i];
            float y = particlePos[2 * i + 1];

            int xi = Mathf.Clamp(Mathf.FloorToInt(x * pInvSpacing), 0, pNumX - 1);
            int yi = Mathf.Clamp(Mathf.FloorToInt(y * pInvSpacing), 0, pNumY - 1);
            int cellNr = xi * pNumY + yi;
            firstCellParticle[cellNr]--;
            cellParticleIds[firstCellParticle[cellNr]] = i;
        }

        float minDist = 2.0f * particleRadius;
        float minDist2 = minDist * minDist;

        for (int iter = 0; iter < numIters; iter++)
        {
            for (int i = 0; i < numParticles; i++)
            {
                float px = particlePos[2 * i];
                float py = particlePos[2 * i + 1];

                int pxi = Mathf.FloorToInt(px * pInvSpacing);
                int pyi = Mathf.FloorToInt(py * pInvSpacing);
                int x0 = Mathf.Max(pxi - 1, 0);
                int y0 = Mathf.Max(pyi - 1, 0);
                int x1 = Mathf.Min(pxi + 1, pNumX - 1);
                int y1 = Mathf.Min(pyi + 1, pNumY - 1);

                for (int xi = x0; xi <= x1; xi++)
                {
                    for (int yi = y0; yi <= y1; yi++)
                    {
                        int cellNr = xi * pNumY + yi;
                        int firstParticle = firstCellParticle[cellNr];
                        int last = firstCellParticle[cellNr + 1];
                        for (int j = firstParticle; j < last; j++)
                        {
                            int id = cellParticleIds[j];
                            if (id == i)
                                continue;

                            float qx = particlePos[2 * id];
                            float qy = particlePos[2 * id + 1];

                            float dx = qx - px;
                            float dy = qy - py;
                            float d2 = dx * dx + dy * dy;
                            if (d2 > minDist2 || d2 == 0.0f)
                                continue;

                            float d = Mathf.Sqrt(d2);
                            float sFactor = 0.5f * (minDist - d) / d;
                            dx *= sFactor;
                            dy *= sFactor;
                            particlePos[2 * i] -= dx;
                            particlePos[2 * i + 1] -= dy;
                            particlePos[2 * id] += dx;
                            particlePos[2 * id + 1] += dy;
                        }
                    }
                }
            }
        }
    }

    public void HandleParticleCollisions()
    {
        float h = 1.0f / fInvSpacing;
        float r = particleRadius;

        float minX = h + r;
        float maxX = (fNumX - 1) * h - r;
        float minY = h + r;
        float maxY = (fNumY - 1) * h - r;

        for (int i = 0; i < numParticles; i++)
        {
            float x = particlePos[2 * i];
            float y = particlePos[2 * i + 1];

            if (x < minX)
            {
                x = minX;
                particleVel[2 * i] = 0.0f;
            }
            if (x > maxX)
            {
                x = maxX;
                particleVel[2 * i] = 0.0f;
            }
            if (y < minY)
            {
                y = minY;
                particleVel[2 * i + 1] = 0.0f;
            }
            if (y > maxY)
            {
                y = maxY;
                particleVel[2 * i + 1] = 0.0f;
            }
            particlePos[2 * i] = x;
            particlePos[2 * i + 1] = y;
        }
    }

    public void UpdateParticleDensity()
    {
        int n = fNumY;
        float h1 = fInvSpacing;
        float h2 = 0.5f * h;

        float[] d = particleDensity;
        d = new float[fNumCells];

        for (int i = 0; i < numParticles; i++)
        {
            float x = particlePos[2 * i];
            float y = particlePos[2 * i + 1];

            x = Mathf.Clamp(x, h, (fNumX - 1) * h);
            y = Mathf.Clamp(y, h, (fNumY - 1) * h);

            int x0 = Mathf.FloorToInt((x - h2) * h1);
            float tx = ((x - h2) - x0 * h) * h1;
            int x1 = Mathf.Min(x0 + 1, fNumX - 2);

            int y0 = Mathf.FloorToInt((y - h2) * h1);
            float ty = ((y - h2) - y0 * h) * h1;
            int y1 = Mathf.Min(y0 + 1, fNumY - 2);

            float sx = 1.0f - tx;
            float sy = 1.0f - ty;

            if (x0 < fNumX && y0 < fNumY) d[x0 * n + y0] += sx * sy;
            if (x1 < fNumX && y0 < fNumY) d[x1 * n + y0] += tx * sy;
            if (x1 < fNumX && y1 < fNumY) d[x1 * n + y1] += tx * ty;
            if (x0 < fNumX && y1 < fNumY) d[x0 * n + y1] += sx * ty;
        }

        if (particleRestDensity == 0.0f)
        {
            float sum = 0.0f;
            int numFluidCells = 0;

            for (int i = 0; i < fNumCells; i++)
            {
                if (cellType[i] == 0)
                {
                    sum += d[i];
                    numFluidCells++;
                }
            }

            if (numFluidCells > 0)
                particleRestDensity = sum / numFluidCells;
        }
    }

    public void TransferVelocities(bool toGrid, float picFlipRatio)
    {
        int n = fNumY;
        float h = this.h;
        float h1 = this.fInvSpacing;
        float h2 = 0.5f * h;

        if (toGrid)
        {
            prevU = (float[])u.Clone();
            prevV = (float[])v.Clone();

            du = new float[fNumCells];
            dv = new float[fNumCells];
            u = new float[fNumCells];
            v = new float[fNumCells];

            for (int i = 0; i < fNumCells; i++)
                cellType[i] = s[i] == 0.0f ? 2 : 1;

            for (int i = 0; i < numParticles; i++)
            {
                float x = particlePos[2 * i];
                float y = particlePos[2 * i + 1];
                int xi = Mathf.Clamp(Mathf.FloorToInt(x * h1), 0, fNumX - 1);
                int yi = Mathf.Clamp(Mathf.FloorToInt(y * h1), 0, fNumY - 1);
                int cellNr = xi * n + yi;
                if (cellType[cellNr] == 1)
                    cellType[cellNr] = 0;
            }
        }

        for (int component = 0; component < 2; component++)
        {
            float dx = component == 0 ? 0.0f : h2;
            float dy = component == 0 ? h2 : 0.0f;

            float[] f = component == 0 ? u : v;
            float[] prevF = component == 0 ? prevU : prevV;
            float[] dArray = component == 0 ? du : dv;

            for (int i = 0; i < numParticles; i++)
            {
                float x = particlePos[2 * i];
                float y = particlePos[2 * i + 1];

                x = Mathf.Clamp(x, h, (fNumX - 1) * h);
                y = Mathf.Clamp(y, h, (fNumY - 1) * h);

                int x0 = Mathf.Min(Mathf.FloorToInt((x - dx) * h1), fNumX - 2);
                float tx = ((x - dx) - x0 * h) * h1;
                int x1 = Mathf.Min(x0 + 1, fNumX - 2);

                int y0 = Mathf.Min(Mathf.FloorToInt((y - dy) * h1), fNumY - 2);
                float ty = ((y - dy) - y0 * h) * h1;
                int y1 = Mathf.Min(y0 + 1, fNumY - 2);

                float sx = 1.0f - tx;
                float sy = 1.0f - ty;

                float d0 = sx * sy;
                float d1 = tx * sy;
                float d2 = tx * ty;
                float d3 = sx * ty;

                int nr0 = x0 * n + y0;
                int nr1 = x1 * n + y0;
                int nr2 = x1 * n + y1;
                int nr3 = x0 * n + y1;

                if (toGrid)
                {
                    float pv = particleVel[2 * i + component];
                    f[nr0] += pv * d0;
                    dArray[nr0] += d0;
                    f[nr1] += pv * d1;
                    dArray[nr1] += d1;
                    f[nr2] += pv * d2;
                    dArray[nr2] += d2;
                    f[nr3] += pv * d3;
                    dArray[nr3] += d3;
                }
                else
                {
                    int offset = component == 0 ? n : 1;
                    float valid0 = cellType[nr0] != 1 || cellType[nr0 - offset] != 1 ? 1.0f : 0.0f;
                    float valid1 = cellType[nr1] != 1 || cellType[nr1 - offset] != 1 ? 1.0f : 0.0f;
                    float valid2 = cellType[nr2] != 1 || cellType[nr2 - offset] != 1 ? 1.0f : 0.0f;
                    float valid3 = cellType[nr3] != 1 || cellType[nr3 - offset] != 1 ? 1.0f : 0.0f;

                    float vel = particleVel[2 * i + component];
                    float dSum = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;

                    if (dSum > 0.0f)
                    {
                        float picV = (valid0 * d0 * f[nr0] + valid1 * d1 * f[nr1] + valid2 * d2 * f[nr2] + valid3 * d3 * f[nr3]) / dSum;
                        float corr = (valid0 * d0 * (f[nr0] - prevF[nr0]) + valid1 * d1 * (f[nr1] - prevF[nr1])
                            + valid2 * d2 * (f[nr2] - prevF[nr2]) + valid3 * d3 * (f[nr3] - prevF[nr3])) / dSum;
                        float flipV = vel + corr;

                        particleVel[2 * i + component] = (1.0f - picFlipRatio) * picV + picFlipRatio * flipV;
                    }
                }
            }

            if (toGrid)
            {
                for (int i = 0; i < f.Length; i++)
                {
                    if (dArray[i] > 0.0f)
                        f[i] /= dArray[i];
                }

                for (int i = 0; i < fNumX; i++)
                {
                    for (int j = 0; j < fNumY; j++)
                    {
                        bool solid = cellType[i * n + j] == 2;
                        if (solid || (i > 0 && cellType[(i - 1) * n + j] == 2))
                            u[i * n + j] = prevU[i * n + j];
                        if (solid || (j > 0 && cellType[i * n + j - 1] == 2))
                            v[i * n + j] = prevV[i * n + j];
                    }
                }
            }
        }
    }

    public void SolveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift)
    {
        p = new float[fNumCells];
        prevU = (float[])u.Clone();
        prevV = (float[])v.Clone();

        int n = fNumY;
        float cp = density * h / dt;

        for (int iter = 0; iter < numIters; iter++)
        {
            for (int i = 1; i < fNumX - 1; i++)
            {
                for (int j = 1; j < fNumY - 1; j++)
                {
                    if (cellType[i * n + j] != 0)
                        continue;

                    int center = i * n + j;
                    int left = (i - 1) * n + j;
                    int right = (i + 1) * n + j;
                    int bottom = i * n + j - 1;
                    int top = i * n + j + 1;

                    float s_val = this.s[center];
                    float sx0 = this.s[left];
                    float sx1 = this.s[right];
                    float sy0 = this.s[bottom];
                    float sy1 = this.s[top];
                    float sSum = sx0 + sx1 + sy0 + sy1;
                    if (sSum == 0.0f)
                        continue;

                    float div = u[right] - u[center] +
                        v[top] - v[center];

                    if (particleRestDensity > 0.0f && compensateDrift)
                    {
                        float k = 1.0f;
                        float compression = particleDensity[i * n + j] - particleRestDensity;
                        if (compression > 0.0f)
                            div = div - k * compression;
                    }

                    float pVal = -div / sSum;
                    pVal *= overRelaxation;
                    this.p[center] += cp * pVal;

                    u[center] -= sx0 * pVal;
                    u[right] += sx1 * pVal;
                    v[center] -= sy0 * pVal;
                    v[top] += sy1 * pVal;
                }
            }
        }
    }
}
