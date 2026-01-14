using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Diagnostics;
using System;

/// <summary>
/// Holds all the data for a single tile in the grid.
/// </summary>
[System.Serializable]
public class TileData
{
    public GameObject tileObject;
    public int x, z;
    public float gCost; // Only gCost is needed for BFS path cost calculation
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        tileObject.GetComponent<Renderer>().material.color = color;
    }
}

/// <summary>
/// Manages grid generation and runs the BFS pathfinding algorithm.
/// </summary>
public class PathManager : MonoBehaviour
{
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public Text legendText;

    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 300;

    private TileData[,] grid;
    private static readonly float Sqrt2 = Mathf.Sqrt(2);

    #region Grid Generation and Setup
    void Start()
    {
        StartCoroutine(GenerateGridAndRun());
    }

    System.Collections.IEnumerator GenerateGridAndRun()
    {
        yield return StartCoroutine(GenerateGrid());
        RunBFS();
    }

    System.Collections.IEnumerator GenerateGrid()
    {
        grid = new TileData[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                Vector3 pos = new Vector3(x, 0, z);
                GameObject tileObj = Instantiate(tilePrefab, pos, Quaternion.identity);
                tileObj.transform.parent = this.transform;
                TileData tile = new TileData { tileObject = tileObj, x = x, z = z };
                grid[x, z] = tile;
                if ((x * gridHeight + z) % 200 == 0) yield return null;
            }
        }
        System.Random rand = new System.Random(300);
        int placed = 0;
        while (placed < obstacleCount)
        {
            int ox = rand.Next(0, gridWidth);
            int oz = rand.Next(0, gridHeight);
            if (grid[ox, oz].isObstacle || IsStartOrEnd(ox, oz)) continue;
            grid[ox, oz].isObstacle = true;
            grid[ox, oz].SetColor(Color.black);
            placed++;
            if (placed % 50 == 0) yield return null;
        }
    }

    bool IsStartOrEnd(int x, int z)
    {
        Vector3 sp = startPoint.transform.position;
        Vector3 ep = endPoint.transform.position;
        return (x == (int)sp.x && z == (int)sp.z) || (x == (int)ep.x && z == (int)ep.z);
    }
    #endregion

    void RunBFS()
    {
        int sx = Mathf.RoundToInt(startPoint.transform.position.x);
        int sz = Mathf.RoundToInt(startPoint.transform.position.z);
        int ex = Mathf.RoundToInt(endPoint.transform.position.x);
        int ez = Mathf.RoundToInt(endPoint.transform.position.z);

        TileData startTile = grid[sx, sz];
        TileData endTile = grid[ex, ez];

        startTile.isStart = true;
        endTile.isEnd = true;
        startTile.SetColor(Color.green);
        endTile.SetColor(Color.red);

        FindPathBFS(startTile, endTile);
    }

    // --- Corrected BFS Algorithm ---
    void FindPathBFS(TileData start, TileData end)
    {
        Stopwatch sw = Stopwatch.StartNew();

        Queue<TileData> queue = new Queue<TileData>();
        HashSet<TileData> visited = new HashSet<TileData>();

        // Reset costs and parents
        foreach (var tile in grid)
        {
            tile.gCost = float.MaxValue;
            tile.parent = null;
        }

        start.gCost = 0;
        queue.Enqueue(start);
        visited.Add(start);

        while (queue.Count > 0)
        {
            TileData current = queue.Dequeue();

            if (!current.isStart) current.SetColor(Color.yellow); // Visualize explored

            if (current == end)
            {
                sw.Stop();
                TracePath(end, sw.Elapsed.TotalMilliseconds);
                return;
            }

            foreach (TileData neighbor in GetNeighbors(current))
            {
                if (neighbor.isObstacle || visited.Contains(neighbor))
                    continue;

                // Prevent cutting corners diagonally
                if (Mathf.Abs(neighbor.x - current.x) == 1 && Mathf.Abs(neighbor.z - current.z) == 1)
                {
                    if (grid[current.x, neighbor.z].isObstacle || grid[neighbor.x, current.z].isObstacle)
                    {
                        continue;
                    }
                }
                
                visited.Add(neighbor);
                neighbor.parent = current;

                // Calculate the true gCost for the neighbor
                float moveCost = (current.x == neighbor.x || current.z == neighbor.z) ? 1.0f : Sqrt2;
                neighbor.gCost = current.gCost + moveCost;

                queue.Enqueue(neighbor);
            }
        }

        sw.Stop();
        UnityEngine.Debug.Log("No path found by BFS.");
    }

    List<TileData> GetNeighbors(TileData tile)
    {
        List<TileData> neighbors = new List<TileData>();
        // Check all 8 directions
        for (int x = -1; x <= 1; x++)
        {
            for (int z = -1; z <= 1; z++)
            {
                if (x == 0 && z == 0) continue;

                int checkX = tile.x + x;
                int checkZ = tile.z + z;

                if (IsInBounds(checkX, checkZ))
                {
                    neighbors.Add(grid[checkX, checkZ]);
                }
            }
        }
        return neighbors;
    }

    #region Helper and UI Functions
    void TracePath(TileData endTile, double timeMs)
    {
        long memoryBytes = System.GC.GetTotalMemory(false);
        double memoryMB = memoryBytes / (1024.0 * 1024.0);

        List<TileData> path = new List<TileData>();
        TileData current = endTile;
        while (current != null)
        {
            path.Add(current);
            current = current.parent;
        }
        path.Reverse();

        foreach (var tile in path)
        {
            if (!tile.isStart && !tile.isEnd)
                tile.SetColor(Color.cyan);
        }

        float finalCost = endTile.gCost; // Use the calculated gCost for an accurate path cost
        UnityEngine.Debug.Log($"BFS path found! Cost: {finalCost:F2}, Time: {timeMs:F3} ms, Memory: {memoryMB:F3} MB");
        UpdateLegendText(finalCost, timeMs, true, memoryMB);
    }

    bool IsInBounds(int x, int z)
    {
        return x >= 0 && x < gridWidth && z >= 0 && z < gridHeight;
    }

    void UpdateLegendText(float cost, double timeMs, bool pathFound, double memoryMB)
    {
        if (legendText != null)
        {
            legendText.text = $"Algorithm: BFS\n" +
                              $"Path Cost: {(pathFound ? cost.ToString("F2") : "N/A")}\n" +
                              $"Time: {timeMs:F3} ms\n" +
                              $"Memory: {memoryMB:F3} MB";
        }
    }
    #endregion
}
