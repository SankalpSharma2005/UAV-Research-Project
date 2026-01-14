using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine.Profiling;

using Debug = UnityEngine.Debug;

[System.Serializable]
public class TileData
{
    public GameObject tileObject;
    public int x, z;
    public float gCost = float.MaxValue;
    public float rhs = float.MaxValue;
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        var renderer = tileObject.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = color;
        }
    }
}

public class PathManager : MonoBehaviour
{
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public Text legendText;
    public Transform drone;

    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 400;

    private TileData[,] grid;
    private TileData startTile, goalTile;
    private SortedSet<(float, float, TileData)> openSet = new(new KeyComparer());
    private List<TileData> lastPath;
    private Coroutine currentAnimationCoroutine;

    private double lastReplanningTimeMs = 0;
    private float lastMemoryUsedMB = 0;

    float GetAlgorithmMemoryMB()
{
    // Force a garbage collection to clean up unused objects
    System.GC.Collect();
    System.GC.WaitForPendingFinalizers();

    long memBefore = System.GC.GetTotalMemory(true); // managed memory only
    // Compute rough memory used by your algorithm's core structures
    long memUsed = memBefore;

    // Convert to MB
    return memUsed / (1024f * 1024f);
}


    void Start()
    {
        StartCoroutine(GenerateGridAndRun());
    }

    IEnumerator GenerateGridAndRun()
    {
        yield return StartCoroutine(GenerateGrid());
        yield return StartCoroutine(PlaceFixedObstacles());
        MarkStartAndGoalClear();
        RunLPAStar();
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
                var renderer = tileObj.GetComponent<Renderer>();
                if (renderer != null)
                {
                    renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    renderer.receiveShadows = false;
                }

                TileData tile = new TileData
                {
                    tileObject = tileObj,
                    x = x,
                    z = z,
                    isStart = false,
                    isEnd = false,
                    isObstacle = false,
                    parent = null,
                    gCost = float.MaxValue,
                    rhs = float.MaxValue
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

    void MarkStartAndGoalClear()
    {
        int sx = Mathf.RoundToInt(startPoint.transform.position.x);
        int sz = Mathf.RoundToInt(startPoint.transform.position.z);
        int gx = Mathf.RoundToInt(endPoint.transform.position.x);
        int gz = Mathf.RoundToInt(endPoint.transform.position.z);

        sx = Mathf.Clamp(sx, 0, gridWidth - 1);
        sz = Mathf.Clamp(sz, 0, gridHeight - 1);
        gx = Mathf.Clamp(gx, 0, gridWidth - 1);
        gz = Mathf.Clamp(gz, 0, gridHeight - 1);

        startTile = grid[sx, sz];
        goalTile = grid[gx, gz];

        if (startTile.isObstacle)
        {
            startTile.isObstacle = false;
            startTile.SetColor(Color.green);
            startTile.tileObject.transform.localScale = Vector3.one;
            startTile.tileObject.transform.position = new Vector3(startTile.x, 0, startTile.z);
        }
        startTile.isStart = true;

        if (goalTile.isObstacle)
        {
            goalTile.isObstacle = false;
            goalTile.SetColor(Color.red);
            goalTile.tileObject.transform.localScale = Vector3.one;
            goalTile.tileObject.transform.position = new Vector3(goalTile.x, 0, goalTile.z);
        }
        goalTile.isEnd = true;
    }
    float GetTotalMemoryMB()
{
    // Allocated memory in MB
    return Profiler.GetTotalAllocatedMemoryLong() / (1024f * 1024f);
}


    void RunLPAStar()
    {
        Stopwatch sw = Stopwatch.StartNew();

        // Reset tiles
        foreach (var tile in grid)
        {
            tile.gCost = float.MaxValue;
            tile.rhs = float.MaxValue;
            tile.parent = null;
        }
        openSet.Clear();

        // LPA* begins with the start vertex's rhs = 0, but path to goal!
        startTile.rhs = 0;
        InsertIntoOpen(startTile);

        ComputeShortestPath_LPA();

        List<Vector3> initialPath = TracePath(goalTile);

        if (initialPath == null || initialPath.Count < 2)
        {
            Debug.LogWarning("No valid path found on initial search.");
            return;
        }
        if (currentAnimationCoroutine != null)
            StopCoroutine(currentAnimationCoroutine);
        currentAnimationCoroutine = StartCoroutine(AnimateInitialPathAndReplan(initialPath));

        sw.Stop();
UpdateLegendText(CalculateCost(), sw.Elapsed.TotalMilliseconds, initialPath != null);

// Measure algorithm memory
float algoMemMB = GetAlgorithmMemoryMB();
Debug.Log($"Initial LPA* finished. Path Cost: {CalculateCost()}, " +
          $"Time: {sw.Elapsed.TotalMilliseconds:F3} ms, " +
          $"Algorithm Memory: {algoMemMB:F2} MB");


    }

    IEnumerator AnimateInitialPathAndReplan(List<Vector3> path)
    {
        yield return StartCoroutine(AnimateTraversalTiles(path, path.Count - 1, Color.cyan));
        yield return new WaitForSeconds(0.2f);
        yield return StartCoroutine(InsertObstacleAndReplan());

        List<Vector3> replannedPath = TracePath(goalTile);
        if (replannedPath != null && replannedPath.Count > 1)
            yield return StartCoroutine(AnimateTraversalTiles(replannedPath, replannedPath.Count - 1, Color.yellow));
    }

    IEnumerator AnimateTraversalTiles(List<Vector3> pathPositions, int upToIndex, Color color)
    {
        for (int i = 0; i <= upToIndex && i < pathPositions.Count; i++)
        {
            Vector3Int gridPos = Vector3Int.RoundToInt(new Vector3(pathPositions[i].x, 0, pathPositions[i].z));
            TileData tile = grid[gridPos.x, gridPos.z];
            if (!tile.isStart && !tile.isEnd)
                tile.SetColor(color);
            yield return new WaitForSeconds(0.07f);
        }
    }

    IEnumerator InsertObstacleAndReplan()
    {
        yield return new WaitForSeconds(0.3f);

        if (lastPath == null || lastPath.Count < 2)
        {
            Debug.LogWarning("Current path too short or missing; skipping obstacle insertion.");
            yield break;
        }
        long beforeMem = System.GC.GetTotalMemory(false);

        List<int> indices = new();
        if (lastPath.Count < 5)
        {
            indices.Add(lastPath.Count / 2);
        }
        else
        {
            int numObstacles = Mathf.Min(4, lastPath.Count / 3);
            for (int i = 1; i <= numObstacles; i++)
                indices.Add((i * lastPath.Count) / (numObstacles + 1));
        }

        foreach (int idx in indices)
        {
            if (idx <= 0 || idx >= lastPath.Count - 1) continue;
            TileData tile = lastPath[idx];
            if (!tile.isStart && !tile.isEnd && !tile.isObstacle)
            {
                tile.isObstacle = true;
                tile.SetColor(Color.magenta);
                tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
                tile.tileObject.transform.position += new Vector3(0, 0.5f, 0);

                tile.gCost = float.MaxValue;
                tile.rhs = float.MaxValue;
                UpdateVertex_LPA(tile);
                foreach (var n in GetNeighbors(tile))
                    UpdateVertex_LPA(n);
            }
        }
        Stopwatch replanningStopwatch = Stopwatch.StartNew();
        ComputeShortestPath_LPA();
        replanningStopwatch.Stop();

        long afterMem = System.GC.GetTotalMemory(false);

        lastReplanningTimeMs = replanningStopwatch.Elapsed.TotalMilliseconds;
        lastMemoryUsedMB = (afterMem - beforeMem) / (1024f * 1024f);

        Debug.Log($"Replanning time: {lastReplanningTimeMs:F3} ms");
        Debug.Log($"Memory usage change during replanning: {lastMemoryUsedMB:F3} MB");

        TracePath(goalTile);
        yield return null;
        float algoMemAfterReplanMB = GetAlgorithmMemoryMB();
Debug.Log($"Replanning done. Time: {lastReplanningTimeMs:F3} ms, " +
          $"Memory change during replanning: {lastMemoryUsedMB:F3} MB, " +
          $"Algorithm total memory now: {algoMemAfterReplanMB:F2} MB");

    }

    void ComputeShortestPath_LPA()
    {
        int stepLimit = 200000;
        int steps = 0;

        while (openSet.Count > 0 &&
               (LessThan(openSet.Min, CalculateKey_LPA(goalTile)) || goalTile.rhs != goalTile.gCost))
        {
            if (steps++ > stepLimit)
            {
                Debug.LogWarning("LPA* search exceeded step limit. Aborting.");
                break;
            }

            var (k1, k2, u) = openSet.Min;
            openSet.Remove(openSet.Min);

            if (u.gCost > u.rhs)
            {
                u.gCost = u.rhs;
                foreach (var s in GetNeighbors(u))
                    UpdateVertex_LPA(s);
            }
            else
            {
                u.gCost = float.MaxValue;
                UpdateVertex_LPA(u);
                foreach (var s in GetNeighbors(u))
                    UpdateVertex_LPA(s);
            }
        }
    }

   void UpdateVertex_LPA(TileData u)
{
    if (u != startTile)
    {
        float minRhs = float.MaxValue;
        TileData minParent = null;
        foreach (var s in GetNeighbors(u))
        {
            float tentative = s.gCost + MovementCost(s, u);
            if (tentative < minRhs)
            {
                minRhs = tentative;
                minParent = s;
            }
        }
        u.rhs = minRhs;
        u.parent = minParent;
    }

    RemoveFromOpen(u);
    if (u.gCost != u.rhs)
        InsertIntoOpen(u);
}


    List<Vector3> TracePath(TileData to)
    {
        // Recolor walkables as white
        foreach (var t in grid)
        {
            if (!t.isStart && !t.isEnd && !t.isObstacle)
                t.SetColor(Color.white);
        }
        List<TileData> pathTiles = new List<TileData>();
        TileData curr = to;
        while (curr != null && !curr.isStart)
        {
            pathTiles.Add(curr);
            curr = curr.parent;
        }
        if (curr == null)
        {
            lastPath = null;
            return null;
        }
        pathTiles.Add(startTile);
        startTile.SetColor(Color.green);
        goalTile.SetColor(Color.red);

        pathTiles.Reverse();
        lastPath = pathTiles;

        List<Vector3> pathPositions = new List<Vector3>();
        foreach (var t in pathTiles)
            pathPositions.Add(t.tileObject.transform.position + Vector3.up * 1.2f);

        return pathPositions;
    }

    float CalculateCost()
{
    float cost = 0f;
    TileData curr = goalTile;
    while (curr != null && !curr.isStart)
    {
        TileData parent = curr.parent;
        if (parent == null) return 0f; // no valid path
        cost += MovementCost(curr, parent);
        curr = parent;
    }
    return cost;
}

    float Heuristic(TileData a, TileData b)
{
    float dx = Mathf.Abs(a.x - b.x);
    float dz = Mathf.Abs(a.z - b.z);
    float diag = Mathf.Min(dx, dz);
    float straight = Mathf.Abs(dx - dz);
    return diag * Mathf.Sqrt(2f) + straight * 1f;
}

    void InsertIntoOpen(TileData t)
    {
        var key = CalculateKey_LPA(t);
        // Remove existing in open set, as SortedSet won't update order on value change
        RemoveFromOpen(t);
        openSet.Add((key.Item1, key.Item2, t));
    }

    void RemoveFromOpen(TileData t)
    {
        openSet.RemoveWhere(item => item.Item3 == t);
    }

    (float, float) CalculateKey_LPA(TileData s)
    {
        float minCost = Mathf.Min(s.gCost, s.rhs);
        // LPA* proceeds from start towards goal
        return (minCost + Heuristic(s, goalTile), minCost);
    }

    bool LessThan((float, float, TileData) a, (float, float) b)
    {
        if (a.Item1 != b.Item1) return a.Item1 < b.Item1;
        return a.Item2 < b.Item2;
    }

    List<TileData> GetNeighbors(TileData t)
{
    List<TileData> neighbors = new List<TileData>();

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dz = -1; dz <= 1; dz++)
        {
            if (dx == 0 && dz == 0) continue;

            int nx = t.x + dx;
            int nz = t.z + dz;

            if (nx < 0 || nx >= gridWidth || nz < 0 || nz >= gridHeight) continue;

            TileData neighbor = grid[nx, nz];
            if (neighbor.isObstacle) continue;

            // If diagonal, optionally prevent corner-cutting:
            if (Mathf.Abs(dx) == 1 && Mathf.Abs(dz) == 1)
            {
                // neighbors on the orthogonal sides
                TileData side1 = grid[t.x + dx, t.z];
                TileData side2 = grid[t.x, t.z + dz];
                // if either orthogonal adjacent tile is an obstacle, block diagonal move
                if (side1.isObstacle || side2.isObstacle) continue;
            }

            neighbors.Add(neighbor);
        }
    }
    return neighbors;
}
float MovementCost(TileData a, TileData b)
{
    int dx = Mathf.Abs(a.x - b.x);
    int dz = Mathf.Abs(a.z - b.z);
    // orthogonal move
    if (dx + dz == 1) return 1f;
    // diagonal move
    if (dx == 1 && dz == 1) return Mathf.Sqrt(2f);
    // non-adjacent (shouldn't happen), return large
    return float.MaxValue;
}

    bool IsStartOrEnd(int x, int z)
    {
        Vector3 sp = startPoint.transform.position;
        Vector3 ep = endPoint.transform.position;
        return (x == Mathf.RoundToInt(sp.x) && z == Mathf.RoundToInt(sp.z)) ||
               (x == Mathf.RoundToInt(ep.x) && z == Mathf.RoundToInt(ep.z));
    }

    void UpdateLegendText(float cost, double timeMs, bool pathFound)
{
    if (legendText != null)
    {
        string costText = pathFound ? cost.ToString("F3") : "No path";
        legendText.text =
            "Legend:\n" +
            "Start - Green\n" +
            "End - Red\n" +
            "Walkable - White\n" +
            "Obstacle - Black\n" +
            "Current Path (initial) - Cyan\n" +
            "Replanned Path - Yellow\n" +
            "Inserted Obstacles - Magenta\n\n" +
            "Path Cost: " + costText + "\n" +
            "Execution Time: " + timeMs.ToString("F3") + " ms\n";
    }
}

    class KeyComparer : IComparer<(float, float, TileData)>
    {
        public int Compare((float, float, TileData) a, (float, float, TileData) b)
        {
            if (a.Item1 != b.Item1)
                return a.Item1.CompareTo(b.Item1);
            if (a.Item2 != b.Item2)
                return a.Item2.CompareTo(b.Item2);
            return a.Item3.GetHashCode().CompareTo(b.Item3.GetHashCode());
        }
    }
}
