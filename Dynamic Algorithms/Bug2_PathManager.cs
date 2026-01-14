// PathManager.cs
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System;

[System.Serializable]
public class TileData
{
    public GameObject tileObject;
    public int x, z;
    public bool isStart, isEnd, isObstacle;

    public void SetColor(Color color)
    {
        if (tileObject != null)
        {
            var renderer = tileObject.GetComponent<Renderer>();
            if (renderer != null)
                renderer.material.color = color;
        }
    }
}

public class PathManager : MonoBehaviour
{
    [Header("References")]
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public Text legendText;
    public Transform drone;

    [Header("Grid / Obstacles")]
    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 300;

    [Header("Simulation")]
    public float simulationStepDelay = 0.02f;
    public float dynamicObstacleInjectDelay = 3f;
    public int dynamicObstaclesToInject = 5;

    [Header("Bug2 Tuning")]
    public int obstacleLookahead = 5;

    private TileData[,] grid;
    private TileData startTile, goalTile;

    private List<TileData> mline;
    private TileData currentTile;
    private TileData hitTile;
    private Vector2Int heading;
    private List<TileData> pathTaken = new List<TileData>();
    private bool dynamicInserted = false;

    private HashSet<string> visitedBoundarySet = new HashSet<string>();

    private static readonly Vector2Int[] ORTHO_DIRS = new Vector2Int[]
    {
        new Vector2Int(1,0),
        new Vector2Int(0,1),
        new Vector2Int(-1,0),
        new Vector2Int(0,-1)
    };

    long GetMemoryUsageBytes()
    {
        return GC.GetTotalMemory(false);
    }

    void Start()
    {
        StartCoroutine(GenerateGridAndRun());
    }

    IEnumerator GenerateGridAndRun()
    {
        yield return StartCoroutine(GenerateGrid());

        MarkStartAndGoalClear();
        BuildMLine();
        yield return StartCoroutine(PlaceFixedObstacles());

        foreach (var t in mline)
        {
            if (!t.isStart && !t.isEnd && !t.isObstacle)
                t.SetColor(Color.cyan);
        }

        StartCoroutine(ExecuteBug2());

        yield return new WaitForSeconds(dynamicObstacleInjectDelay);
        StartCoroutine(InjectDynamicObstacles());
    }

    IEnumerator GenerateGrid()
    {
        grid = new TileData[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                Vector3 pos = new Vector3(x, 0, z);
                GameObject tileObj = Instantiate(tilePrefab, pos, Quaternion.identity, this.transform);

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
                    isObstacle = false,
                    isStart = false,
                    isEnd = false
                };
                tile.SetColor(Color.white);
                grid[x, z] = tile;

                if ((x * gridHeight + z) % 200 == 0) yield return null;
            }
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

        if (startTile.isObstacle) ClearTile(startTile, Color.green);
        startTile.isStart = true;
        startTile.SetColor(Color.green);

        if (goalTile.isObstacle) ClearTile(goalTile, Color.red);
        goalTile.isEnd = true;
        goalTile.SetColor(Color.red);
    }

    IEnumerator PlaceFixedObstacles()
    {
        foreach (var tile in grid)
        {
            if (tile.isObstacle && !tile.isStart && !tile.isEnd)
                ClearTile(tile, Color.white);
        }

        int[,] fixedObstacles = new int[,]
        {
            {20, 27}, {21, 27}, {22, 27}, {23, 27}, {24, 27},
            {26, 30}, {26, 31}, {26, 32}, {26, 33},
            {30, 40}, {31, 40}, {32, 40}, {33, 40}
        };

        for (int i = 0; i < fixedObstacles.GetLength(0); i++)
        {
            int ox = fixedObstacles[i, 0];
            int oz = fixedObstacles[i, 1];

            if (IsStartOrEnd(ox, oz)) continue;
            if (ox < 0 || ox >= gridWidth || oz < 0 || oz >= gridHeight) continue;

            TileData tile = grid[ox, oz];
            tile.isObstacle = true;
            tile.SetColor(Color.black);
            tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
            tile.tileObject.transform.position += new Vector3(0, 0.5f, 0);
        }

        yield return null;
    }

    IEnumerator InjectDynamicObstacles()
    {
        int injected = 0;
        int skipCount = Mathf.Min(10, mline.Count / 5);

        System.Random rand = new System.Random();
        List<int> validIndices = new List<int>();
        for (int i = skipCount; i < mline.Count - 1; i++)
        {
            if (!mline[i].isStart && !mline[i].isEnd && !mline[i].isObstacle)
                validIndices.Add(i);
        }

        for (int i = validIndices.Count - 1; i > 0; i--)
        {
            int j = rand.Next(i + 1);
            int temp = validIndices[i];
            validIndices[i] = validIndices[j];
            validIndices[j] = temp;
        }

        foreach (var idx in validIndices)
        {
            if (injected >= dynamicObstaclesToInject) break;

            TileData t = mline[idx];
            if (t == startTile || t == goalTile) continue;

            t.isObstacle = true;
            t.SetColor(Color.magenta);
            t.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
            t.tileObject.transform.position += new Vector3(0, 0.5f, 0);
            injected++;
            yield return new WaitForSeconds(0.25f);
        }

        dynamicInserted = true;
        Debug.Log($"Injected {injected} dynamic obstacles on M-line beyond start region.");

        StopCoroutine(nameof(ExecuteBug2));
        StartCoroutine(ExecuteBug2());
    }

    IEnumerator ExecuteBug2()
    {
        long startMem = GetMemoryUsageBytes();
        Stopwatch sw = Stopwatch.StartNew();

        currentTile = startTile;
        pathTaken.Clear();
        pathTaken.Add(currentTile);
        visitedBoundarySet.Clear();
        heading = Vector2Int.zero;
        hitTile = null;

        if (drone != null) drone.position = currentTile.tileObject.transform.position + Vector3.up * 1.2f;

        int stepLimit = gridWidth * gridHeight * 10;
        int steps = 0;
        bool reached = false;

        while (steps++ < stepLimit)
        {
            if (currentTile == goalTile)
            {
                reached = true;
                break;
            }

            int currIdx = mline.IndexOf(currentTile);
            if (currIdx < 0)
            {
                currIdx = FindNearestMLineIndex(currentTile, mline);
                if (currIdx < 0)
                {
                    Debug.LogWarning("Current tile not on or near M-line; aborting.");
                    break;
                }
            }

            if (currIdx + 1 >= mline.Count)
                break;

            TileData nextOnM = mline[currIdx + 1];

            TileData obstacleAheadTile = null;
            if (dynamicInserted)
            {
                int lookahead = Mathf.Max(1, obstacleLookahead);
                int limit = Mathf.Min(mline.Count - 1, currIdx + lookahead);
                for (int k = currIdx + 1; k <= limit; k++)
                {
                    if (mline[k].isObstacle)
                    {
                        obstacleAheadTile = mline[k];
                        break;
                    }
                }
            }

            if (!nextOnM.isObstacle && obstacleAheadTile == null)
            {
                MoveToTile(nextOnM);
                yield return new WaitForSeconds(simulationStepDelay);
                continue;
            }
            else
            {
                TileData detour = FindNearestFreeNeighbor(currentTile);
                if (detour != null)
                {
                    MoveToTile(detour);
                    yield return new WaitForSeconds(simulationStepDelay);
                    continue;
                }

                hitTile = currentTile;
                float hitDist = GetDistance(hitTile, goalTile);

                TileData obstacleToUse = nextOnM.isObstacle ? nextOnM : obstacleAheadTile ?? nextOnM;

                TileData leavePoint = LeftHandWallFollow(hitTile, obstacleToUse, hitDist, out List<TileData> circPath);

                if (leavePoint != null)
                {
                    foreach (var segTile in circPath)
                    {
                        if (segTile == hitTile) continue;
                        MoveToTile(segTile);
                        yield return new WaitForSeconds(simulationStepDelay);
                    }

                    currentTile = leavePoint;
                    if (drone != null) drone.position = currentTile.tileObject.transform.position + Vector3.up * 1.2f;
                    continue;
                }
                else
                {
                    Debug.LogWarning("Bug2: Unable to find leave point during circumnavigation. Aborting.");
                    break;
                }
            }
        }

        sw.Stop();
        long endMem = GetMemoryUsageBytes();
        float deltaKB = (endMem - startMem) / 1024f;
        float totalKB = endMem / 1024f;

        Debug.Log($"Pure Bug2 runtime: {sw.Elapsed.TotalMilliseconds:F3} ms");
        Debug.Log($"Memory Usage: {deltaKB:F2} KB (delta), Total: {totalKB:F2} KB");

        bool pathFound = currentTile == goalTile;
        UpdateLegendText(pathTaken.Count, sw.Elapsed.TotalMilliseconds, pathFound, totalKB, deltaKB);

        if (pathFound)
        {
            UnityEngine.Debug.Log($"Goal reached in {steps} steps. Time: {sw.Elapsed.TotalMilliseconds:F3} ms");
        }
        else
        {
            UnityEngine.Debug.Log("Goal could not be reached.");
        }
    }

    // ---------------- HELPERS ----------------
    TileData FindNearestFreeNeighbor(TileData from)
    {
        foreach (var d in ORTHO_DIRS)
        {
            TileData n = GetTileInDirection(from, d);
            if (n != null && !n.isObstacle) return n;
        }
        return null;
    }

    TileData LeftHandWallFollow(TileData hitPoint, TileData obstacleOnM, float hitDistToGoal, out List<TileData> circPath)
    {
        circPath = new List<TileData>();

        TileData prev = hitPoint;
        TileData cur = hitPoint;
        Vector2Int curHeading = Vector2Int.zero;

        Vector2Int towardGoal = new Vector2Int(Mathf.Clamp(goalTile.x - hitPoint.x, -1, 1),
                                               Mathf.Clamp(goalTile.z - hitPoint.z, -1, 1));
        curHeading = (towardGoal != Vector2Int.zero) ? towardGoal : new Vector2Int(1, 0);

        visitedBoundarySet.Clear();
        circPath.Add(hitPoint);

        int steps = 0;
        int maxSteps = gridWidth * gridHeight * 8;

        while (steps++ < maxSteps)
        {
            if (IsOnMLine(cur))
            {
                float dCur = GetDistance(cur, goalTile);
                if (dCur <= hitDistToGoal + 0.1f)
                {
                    return cur;
                }
            }

            Vector2Int left = RotateLeft(curHeading);
            Vector2Int forward = curHeading;
            Vector2Int right = RotateRight(curHeading);
            Vector2Int back = -curHeading;

            Vector2Int[] tries = new Vector2Int[] { left, forward, right, back };

            bool moved = false;
            TileData next = null;
            foreach (var tryDir in tries)
            {
                next = GetTileInDirection(cur, tryDir);
                if (next != null && !next.isObstacle)
                {
                    moved = true;
                    curHeading = tryDir;
                    break;
                }
            }

            if (!moved) return null;

            prev = cur;
            cur = next;
            circPath.Add(cur);

            if (!cur.isStart && !cur.isEnd)
            {
                cur.SetColor(dynamicInserted ? Color.yellow : Color.cyan);
            }

            string sig = BoundarySignature(cur, curHeading);
            if (visitedBoundarySet.Contains(sig))
            {
                int nearestIdx = FindNearestMLineIndex(cur, mline);
                if (nearestIdx >= 0 && !mline[nearestIdx].isObstacle)
                    return mline[nearestIdx];
                return null;
            }

            visitedBoundarySet.Add(sig);

            if (cur == goalTile) return cur;
        }

        return null;
    }

    void MoveToTile(TileData t)
    {
        currentTile = t;
        pathTaken.Add(currentTile);

        if (!currentTile.isStart && !currentTile.isEnd)
            currentTile.SetColor(dynamicInserted ? Color.yellow : Color.cyan);

        if (drone != null)
            drone.position = currentTile.tileObject.transform.position + Vector3.up * 1.2f;
    }

    void BuildMLine()
    {
        mline = new List<TileData>();
        var points = BresenhamLine(startTile.x, startTile.z, goalTile.x, goalTile.z);
        foreach (var p in points)
        {
            if (p.x >= 0 && p.x < gridWidth && p.z >= 0 && p.z < gridHeight)
                mline.Add(grid[p.x, p.z]);
        }
    }

    List<(int x, int z)> BresenhamLine(int x0, int z0, int x1, int z1)
    {
        List<(int x, int z)> points = new List<(int x, int z)>();
        int dx = Mathf.Abs(x1 - x0);
        int dz = Mathf.Abs(z1 - z0);
        int sx = x0 < x1 ? 1 : -1;
        int sz = z0 < z1 ? 1 : -1;
        int err = dx - dz;
        int x = x0, z = z0;
        while (true)
        {
            points.Add((x, z));
            if (x == x1 && z == z1) break;
            int e2 = 2 * err;
            if (e2 > -dz) { err -= dz; x += sx; }
            if (e2 < dx) { err += dx; z += sz; }
        }
        return points;
    }

    int FindNearestMLineIndex(TileData t, List<TileData> m)
    {
        int best = -1;
        float bd = float.MaxValue;
        for (int i = 0; i < m.Count; i++)
        {
            float d = Vector2.Distance(new Vector2(t.x, t.z), new Vector2(m[i].x, m[i].z));
            if (d < bd)
            {
                bd = d;
                best = i;
            }
        }
        return best;
    }

    TileData GetTileInDirection(TileData from, Vector2Int dir)
    {
        int nx = from.x + dir.x;
        int nz = from.z + dir.y;
        if (nx >= 0 && nx < gridWidth && nz >= 0 && nz < gridHeight)
            return grid[nx, nz];
        return null;
    }

    float GetDistance(TileData a, TileData b)
    {
        return Vector2.Distance(new Vector2(a.x, a.z), new Vector2(b.x, b.z));
    }

    bool IsOnMLine(int x, int z)
    {
        int vx1 = goalTile.x - startTile.x;
        int vz1 = goalTile.z - startTile.z;
        int vx2 = x - startTile.x;
        int vz2 = z - startTile.z;

        if (vx1 == 0 && vz1 == 0)
            return x == startTile.x && z == startTile.z;

        float cross = Mathf.Abs(vx1 * vz2 - vz1 * vx2);
        float len = Mathf.Sqrt(vx1 * vx1 + vz1 * vz1);
        float perpDist = cross / (len + 1e-6f);

        return perpDist <= 0.5f;
    }
    bool IsOnMLine(TileData t) => IsOnMLine(t.x, t.z);

    bool IsStartOrEnd(int x, int z)
    {
        Vector3 sp = startPoint.transform.position;
        Vector3 ep = endPoint.transform.position;
        return (x == Mathf.RoundToInt(sp.x) && z == Mathf.RoundToInt(sp.z)) ||
               (x == Mathf.RoundToInt(ep.x) && z == Mathf.RoundToInt(ep.z));
    }

    void ClearTile(TileData tile, Color c)
    {
        tile.isObstacle = false;
        tile.tileObject.transform.localScale = Vector3.one;
        tile.tileObject.transform.position = new Vector3(tile.x, 0, tile.z);
        tile.SetColor(c);
    }

    void UpdateLegendText(int cost, double timeMs, bool pathFound, float totalKB, float deltaKB)
    {
        if (legendText != null)
        {
            legendText.text =
                "Algorithm: Bug2\n" +
                "Legend:\n" +
                "Start - Green\n" +
                "End - Red\n" +
                "Walkable - White\n" +
                "Fixed Obstacle - Black\n" +
                "Dynamic Obstacle - Magenta\n" +
                "M-line - Cyan\n" +
                "Final / Replanned Path - Yellow\n\n" +
                "Path Cost (steps): " + (pathFound ? cost.ToString() : "Trapped") + "\n" +
                "Execution Time: " + timeMs.ToString("F3") + " ms\n" +
                "Memory Used: " + totalKB.ToString("F2") + " KB\n" +
                "Memory Delta: " + deltaKB.ToString("F2") + " KB\n";
        }
    }

    Vector2Int RotateLeft(Vector2Int dir) => new Vector2Int(-dir.y, dir.x);
    Vector2Int RotateRight(Vector2Int dir) => new Vector2Int(dir.y, -dir.x);

    string BoundarySignature(TileData t, Vector2Int heading)
    {
        return t.x + "_" + t.z + ":" + heading.x + "_" + heading.y;
    }
}
