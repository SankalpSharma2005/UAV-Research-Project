using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

[System.Serializable]
public class TileData
{
    public GameObject tileObject;
    public int x, z;
    public bool isStart, isEnd, isObstacle;

    public void SetColor(Color color)
    {
        tileObject.GetComponent<Renderer>().material.color = color;
    }
}

public class PathManager : MonoBehaviour
{
    [Header("Scene References")]
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public Text legendText;
    public Transform drone; // reserved if you want a moving drone

    [Header("Grid Settings")]
    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 400;

    [Header("APF Settings")]
    public float kAttractive = 1.0f;
    public float kRepulsive = 200.0f;
    public float obstacleInfluenceDist = 5.0f;
    public int maxSteps = 5000;

    private TileData[,] grid;
    private TileData startTile, goalTile;
    private List<TileData> lastPath;
    private Coroutine currentAnimationCoroutine;

    // Metrics
    private double lastReplanningTimeMs = 0;
    private double lastCPUTimeMs = 0;
    private float lastMemoryUsedMB = 0;

    void Start()
    {
        StartCoroutine(GenerateGridAndRun());
    }

    IEnumerator GenerateGridAndRun()
    {
        yield return StartCoroutine(GenerateGrid());
        yield return StartCoroutine(PlaceFixedObstacles());
        RunAPF();
    }

    IEnumerator GenerateGrid()
    {
        grid = new TileData[gridWidth, gridHeight];

        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                Vector3 pos = new Vector3(x, 0, z);
                GameObject tileObj = Instantiate(tilePrefab, pos, Quaternion.identity);

                Renderer renderer = tileObj.GetComponent<Renderer>();
                renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                renderer.receiveShadows = false;

                TileData tile = new TileData
                {
                    tileObject = tileObj,
                    x = x,
                    z = z,
                    isStart = false,
                    isEnd = false,
                    isObstacle = false
                };

                tile.SetColor(Color.white);
                grid[x, z] = tile;

                if ((x * gridHeight + z) % 200 == 0)
                    yield return null;
            }
        }
    }

    IEnumerator PlaceFixedObstacles()
    {
        int placed = 0;
        System.Random rand = new System.Random(300);

        while (placed < obstacleCount)
        {
            int ox = rand.Next(0, gridWidth);
            int oz = rand.Next(0, gridHeight);

            if (IsStartOrEnd(ox, oz)) continue;

            TileData tile = grid[ox, oz];
            if (tile.isObstacle) continue;

            tile.isObstacle = true;
            tile.SetColor(Color.black);
            tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
            tile.tileObject.transform.position += new Vector3(0, 0.5f, 0);

            placed++;

            if (placed % 50 == 0) yield return null;
        }
    }

    bool IsStartOrEnd(int x, int z)
    {
        Vector3 sp = startPoint.transform.position;
        Vector3 ep = endPoint.transform.position;
        return (x == Mathf.RoundToInt(sp.x) && z == Mathf.RoundToInt(sp.z)) ||
               (x == Mathf.RoundToInt(ep.x) && z == Mathf.RoundToInt(ep.z));
    }

    void RunAPF()
    {
        Stopwatch sw = Stopwatch.StartNew();
        long beforeMem = System.GC.GetTotalMemory(false);

        int sx = Mathf.RoundToInt(startPoint.transform.position.x);
        int sz = Mathf.RoundToInt(startPoint.transform.position.z);
        int gx = Mathf.RoundToInt(endPoint.transform.position.x);
        int gz = Mathf.RoundToInt(endPoint.transform.position.z);

        startTile = grid[sx, sz];
        goalTile = grid[gx, gz];
        startTile.isStart = true;
        goalTile.isEnd = true;

        List<TileData> initialPath = ComputeAPFPath(startTile, goalTile);

        sw.Stop();
        lastCPUTimeMs = sw.Elapsed.TotalMilliseconds;
        long totalMem = System.GC.GetTotalMemory(false);
        lastMemoryUsedMB = (totalMem) / (1024f * 1024f);

        if (currentAnimationCoroutine != null)
            StopCoroutine(currentAnimationCoroutine);
        currentAnimationCoroutine = StartCoroutine(AnimateInitialPathAndReplan(initialPath));

        UpdateLegendText(initialPath.Count, lastCPUTimeMs, 0, lastMemoryUsedMB);
    }

    List<TileData> ComputeAPFPath(TileData start, TileData goal)
    {
        List<TileData> path = new List<TileData>();
        TileData current = start;
        HashSet<TileData> visited = new HashSet<TileData>();

        int steps = 0;
        while (current != goal && steps < maxSteps)
        {
            path.Add(current);
            visited.Add(current);

            Vector2 force = ComputeNetForce(current, goal);

            // pick best neighbor in direction of force
            TileData next = GetBestNeighbor(current, force);

            if (next == null || visited.Contains(next))
            {
                // escape step
                next = GetRandomNeighbor(current);
                if (next == null) break;
            }

            current = next;
            steps++;
        }

        path.Add(goal);
        lastPath = path;
        return path;
    }

    Vector2 ComputeNetForce(TileData current, TileData goal)
    {
        Vector2 pos = new Vector2(current.x, current.z);
        Vector2 goalPos = new Vector2(goal.x, goal.z);

        // Attractive
        Vector2 force = kAttractive * (goalPos - pos);

        // Repulsive
        foreach (var obs in GetObstaclesInRange(current, obstacleInfluenceDist))
        {
            Vector2 obsPos = new Vector2(obs.x, obs.z);
            Vector2 diff = pos - obsPos;
            float dist = diff.magnitude;
            if (dist < 0.001f) continue;
            if (dist < obstacleInfluenceDist)
            {
                float rep = kRepulsive * (1.0f / dist - 1.0f / obstacleInfluenceDist) / (dist * dist);
                force += rep * (diff.normalized);
            }
        }

        return force;
    }

    TileData GetBestNeighbor(TileData current, Vector2 force)
    {
        TileData best = null;
        float bestDot = float.NegativeInfinity;

        foreach (TileData n in GetNeighbors(current))
        {
            Vector2 dir = new Vector2(n.x - current.x, n.z - current.z).normalized;
            float dot = Vector2.Dot(force.normalized, dir);
            if (dot > bestDot)
            {
                bestDot = dot;
                best = n;
            }
        }
        return best;
    }

    TileData GetRandomNeighbor(TileData current)
    {
        List<TileData> neighbors = GetNeighbors(current);
        if (neighbors.Count == 0) return null;
        return neighbors[Random.Range(0, neighbors.Count)];
    }

    List<TileData> GetObstaclesInRange(TileData t, float range)
    {
        List<TileData> obs = new List<TileData>();
        int r = Mathf.CeilToInt(range);
        for (int dx = -r; dx <= r; dx++)
        {
            for (int dz = -r; dz <= r; dz++)
            {
                int nx = t.x + dx;
                int nz = t.z + dz;
                if (nx >= 0 && nx < gridWidth && nz >= 0 && nz < gridHeight)
                {
                    TileData neighbor = grid[nx, nz];
                    if (neighbor.isObstacle)
                        obs.Add(neighbor);
                }
            }
        }
        return obs;
    }

    IEnumerator AnimateInitialPathAndReplan(List<TileData> path)
    {
        yield return StartCoroutine(AnimateTraversalTiles(path, Color.cyan));

        yield return new WaitForSeconds(0.3f);

        // Insert new obstacles
        yield return StartCoroutine(InsertObstacleAndReplan());

        List<TileData> newPath = ComputeAPFPath(startTile, goalTile);
        yield return StartCoroutine(AnimateTraversalTiles(newPath, Color.yellow));
    }

    IEnumerator AnimateTraversalTiles(List<TileData> pathTiles, Color color)
    {
        foreach (TileData t in pathTiles)
        {
            if (!t.isStart && !t.isEnd)
                t.SetColor(color);
            yield return new WaitForSeconds(0.05f);
        }
    }

    IEnumerator InsertObstacleAndReplan()
    {
        if (lastPath == null || lastPath.Count < 5)
            yield break;

        long beforeMem = System.GC.GetTotalMemory(false);
        Stopwatch sw = Stopwatch.StartNew();

        int numObstacles = Mathf.Min(4, lastPath.Count / 3);
        for (int i = 1; i <= numObstacles; i++)
        {
            int idx = (i * lastPath.Count) / (numObstacles + 1);
            TileData tile = lastPath[idx];
            if (!tile.isStart && !tile.isEnd && !tile.isObstacle)
            {
                tile.isObstacle = true;
                tile.SetColor(Color.magenta);
                tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
                tile.tileObject.transform.position += new Vector3(0, 0.5f, 0);
            }
        }

        List<TileData> newPath = ComputeAPFPath(startTile, goalTile);

        sw.Stop();
        long totalMem = System.GC.GetTotalMemory(false);

        lastReplanningTimeMs = sw.Elapsed.TotalMilliseconds;
        lastMemoryUsedMB = (totalMem) / (1024f * 1024f);

        UpdateLegendText(newPath.Count, lastCPUTimeMs, lastReplanningTimeMs, lastMemoryUsedMB);
        yield return null;
    }

    List<TileData> GetNeighbors(TileData t)
    {
        List<TileData> neighbors = new List<TileData>();
        int[,] dirs = { { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 } };

        for (int i = 0; i < 4; i++)
        {
            int nx = t.x + dirs[i, 0];
            int nz = t.z + dirs[i, 1];
            if (nx >= 0 && nx < gridWidth && nz >= 0 && nz < gridHeight)
            {
                TileData neighbor = grid[nx, nz];
                if (!neighbor.isObstacle)
                    neighbors.Add(neighbor);
            }
        }
        return neighbors;
    }

    void UpdateLegendText(int cost, double cpuTimeMs, double replanTimeMs, float memoryMB)
    {
        if (legendText != null)
        {
            legendText.text =
                "Legend:\n" +
                "Start - Green\n" +
                "End - Red\n" +
                "Walkable - White\n" +
                "Fixed Obstacles - Black\n" +
                "Inserted Obstacles - Magenta\n" +
                "Current Path (initial) - Cyan\n" +
                "Replanned Path - Yellow\n\n" +
                "Path Cost: " + cost + "\n" +
                "Algorithm CPU Time: " + cpuTimeMs.ToString("F3") + " ms\n" +
                "Replanning Time: " + replanTimeMs.ToString("F3") + " ms\n" +
                "Managed Memory Δ: " + memoryMB.ToString("F3") + " MB\n" +
                "GPU: Use Unity Profiler (Frame Debugger) to inspect GPU timings\n";
        }
    }
}
