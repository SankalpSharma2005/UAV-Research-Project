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
    public float gCost = float.MaxValue;
    public float rhs = float.MaxValue;
    public float hCost = 0;
    public float fCost => Mathf.Min(gCost, rhs) + hCost;
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        tileObject.GetComponent<Renderer>().material.color = color;
    }
}

public class PathManager : MonoBehaviour
{
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public Text legendText;
    public Transform drone;  // drone object kept but not moved along path
    private Stopwatch totalStopwatch;

    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 400;

    private TileData[,] grid;
    private TileData startTile, goalTile;
    private SortedSet<(float, float, TileData)> openSet = new(new KeyComparer());
    private float km = 0;
    private List<TileData> lastPath;
    private Coroutine currentAnimationCoroutine = null;

    // Fields for performance measurement display (kept for printing to console)
    private double lastReplanningTimeMs = 0;
    private float lastMemoryUsedMB = 0;

    void Start()
{
    
    StartCoroutine(GenerateGridAndRun());
}


    IEnumerator GenerateGridAndRun()
    {
        yield return StartCoroutine(GenerateGrid());
        yield return StartCoroutine(PlaceFixedObstacles());
        RunDStarLite();
        long totalMemBytes = System.GC.GetTotalMemory(false);
    float totalMemMB = totalMemBytes / (1024f * 1024f);
    Debug.Log($"Total memory used by entire process: {totalMemMB:F3} MB");
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

    void RunDStarLite()
    {
        Stopwatch totalStopwatch = Stopwatch.StartNew();

        int sx = Mathf.RoundToInt(startPoint.transform.position.x);
        int sz = Mathf.RoundToInt(startPoint.transform.position.z);
        int gx = Mathf.RoundToInt(endPoint.transform.position.x);
        int gz = Mathf.RoundToInt(endPoint.transform.position.z);

        startTile = grid[sx, sz];
        goalTile = grid[gx, gz];

        foreach (var t in grid)
        {
            t.gCost = float.MaxValue;
            t.rhs = float.MaxValue;
            t.parent = null;
        }

        goalTile.rhs = 0;
        InsertIntoOpen(goalTile);

        km = 0;
        ComputeShortestPath();
        totalStopwatch.Stop();
Debug.Log($"Total time taken by process: {totalStopwatch.Elapsed.TotalMilliseconds:F3} ms");
        List<Vector3> initialPath = TracePath(startTile);

        if (currentAnimationCoroutine != null)
            StopCoroutine(currentAnimationCoroutine);
        currentAnimationCoroutine = StartCoroutine(AnimateInitialPathAndReplan(initialPath));

        long totalMemoryBytes = System.GC.GetTotalMemory(false);
float totalMemoryMB = totalMemoryBytes / (1024f * 1024f);
Debug.Log($"Total memory used after initial path computation: {totalMemoryMB:F3} MB");


    }

    IEnumerator AnimateInitialPathAndReplan(List<Vector3> path)
    {
        // Animate the initial path in cyan
        yield return StartCoroutine(AnimateTraversalTiles(path, path.Count - 1, Color.cyan));

        yield return new WaitForSeconds(0.2f); // Reduced pause for faster pacing

        // Insert dynamic obstacles and replan
        yield return StartCoroutine(InsertObstacleAndReplan());

        // Get new path after replanning
        List<Vector3> replannedPath = TracePath(startTile);

        // Animate the replanned path in yellow
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

            // Faster animation (~0.07 seconds delay)
            yield return new WaitForSeconds(0.07f);
        }
    }

    IEnumerator InsertObstacleAndReplan()
    {
        // Short delay before inserting obstacles
        yield return new WaitForSeconds(0.3f);

        if (lastPath == null || lastPath.Count < 5)
        {
            Debug.LogWarning("Path is too short or not set. Cannot insert multiple obstacles.");
            yield break;
        }

        long beforeMem = System.GC.GetTotalMemory(false);

        int numObstacles = Mathf.Min(4, lastPath.Count / 3);
        List<int> indices = new();

        for (int i = 1; i <= numObstacles; i++)
            indices.Add((i * lastPath.Count) / (numObstacles + 1));

        foreach (int idx in indices)
        {
            TileData tile = lastPath[idx];
            if (!tile.isStart && !tile.isEnd && !tile.isObstacle)
            {
                tile.isObstacle = true;
                tile.SetColor(Color.magenta);
                tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
                tile.tileObject.transform.position += new Vector3(0, 0.5f, 0);

                tile.gCost = float.MaxValue;
                tile.rhs = float.MaxValue;
                UpdateVertex(tile);

                foreach (var neighbor in GetNeighbors(tile))
                    UpdateVertex(neighbor);
            }
        }

        km += Heuristic(startTile, goalTile);

        Stopwatch replanningStopwatch = Stopwatch.StartNew();
        ComputeShortestPath();
        replanningStopwatch.Stop();

        long afterMem = System.GC.GetTotalMemory(false);

        lastReplanningTimeMs = replanningStopwatch.Elapsed.TotalMilliseconds;
        lastMemoryUsedMB = (afterMem - beforeMem) / (1024f * 1024f);

        // Print metrics to Unity console (instead of legend)
        Debug.Log($"Replanning time: {lastReplanningTimeMs:F3} ms");
        Debug.Log($"Memory usage change during replanning: {lastMemoryUsedMB:F3} MB");
        // CPU/GPU load comment omitted since CPU load isn't directly measured

        TracePath(startTile);

        yield return null;
    }

    void ComputeShortestPath()
    {
        while (openSet.Count > 0 &&
               (LessThan(openSet.Min, CalculateKey(startTile)) || startTile.rhs != startTile.gCost))
        {
            var (k1, k2, u) = openSet.Min;
            openSet.Remove(openSet.Min);

            if (u.gCost > u.rhs)
            {
                u.gCost = u.rhs;
                foreach (var s in GetNeighbors(u))
                    UpdateVertex(s);
            }
            else
            {
                u.gCost = float.MaxValue;
                UpdateVertex(u);
                foreach (var s in GetNeighbors(u))
                    UpdateVertex(s);
            }
        }
    }

    void UpdateVertex(TileData u)
    {
        if (u != goalTile)
        {
            float minRhs = float.MaxValue;
            TileData minParent = null;

            foreach (var s in GetNeighbors(u))
            {
                float tentative = s.gCost + 1;
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

    List<Vector3> TracePath(TileData from)
    {
        foreach (var t in grid)
        {
            if (!t.isStart && !t.isEnd && !t.isObstacle)
                t.SetColor(Color.white);
        }

        List<TileData> pathTiles = new();
        TileData curr = from;
        while (curr != null && !curr.isEnd)
        {
            pathTiles.Add(curr);
            curr = curr.parent;
        }

        pathTiles.Add(goalTile);

        from.SetColor(Color.green);
        goalTile.SetColor(Color.red);

        pathTiles.Reverse();

        if (pathTiles.Count > 0)
            lastPath = pathTiles;

        List<Vector3> pathPositions = new();

        foreach (var t in pathTiles)
            pathPositions.Add(t.tileObject.transform.position + Vector3.up * 1.2f);

        return pathPositions;
    }

    int CalculateCost()
    {
        int cost = 0;
        TileData curr = startTile;
        while (curr != null && !curr.isEnd)
        {
            cost++;
            curr = curr.parent;
        }
        return cost;
    }

    float Heuristic(TileData a, TileData b)
    {
        return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.z - b.z);
    }

    void InsertIntoOpen(TileData t)
    {
        var key = CalculateKey(t);
        openSet.Add((key.Item1, key.Item2, t));
    }

    void RemoveFromOpen(TileData t)
    {
        openSet.RemoveWhere(item => item.Item3 == t);
    }

    (float, float) CalculateKey(TileData s)
    {
        float min = Mathf.Min(s.gCost, s.rhs);
        return (min + Heuristic(startTile, s) + km, min);
    }

    bool LessThan((float, float, TileData) a, (float, float) b)
    {
        return a.Item1 < b.Item1 || (a.Item1 == b.Item1 && a.Item2 < b.Item2);
    }

    List<TileData> GetNeighbors(TileData t)
    {
        List<TileData> neighbors = new();
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

    void UpdateLegendText(int cost, double timeMs, bool pathFound)
    {
        if (legendText != null)
        {
            // Removed metrics info per your request
            legendText.text =
                "Legend:\n" +
                "Start - Green\n" +
                "End - Red\n" +
                "Walkable - White\n" +
                "Obstacle - Black\n" +
                "Current Path (initial) - Cyan\n" +
                "Replanned Path - Yellow\n" +
                "Inserted Obstacles - Magenta\n\n" +
                "Path Cost: " + (pathFound ? cost.ToString() : "No path") + "\n" +
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
