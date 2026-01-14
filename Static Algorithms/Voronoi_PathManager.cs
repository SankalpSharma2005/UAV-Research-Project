using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Diagnostics;
using System; // Needed for IComparable

/// <summary>
/// A simple and efficient min-heap Priority Queue implementation.
/// </summary>
public class PriorityQueue<T> where T : IComparable<T>
{
    private List<T> data;
    public int Count => data.Count;

    public PriorityQueue()
    {
        this.data = new List<T>();
    }

    public void Enqueue(T item)
    {
        data.Add(item);
        int childIndex = data.Count - 1;
        while (childIndex > 0)
        {
            int parentIndex = (childIndex - 1) / 2;
            if (data[childIndex].CompareTo(data[parentIndex]) >= 0)
                break;
            Swap(childIndex, parentIndex);
            childIndex = parentIndex;
        }
    }

    public T Dequeue()
    {
        int lastIndex = data.Count - 1;
        T frontItem = data[0];
        data[0] = data[lastIndex];
        data.RemoveAt(lastIndex);

        lastIndex--;
        int parentIndex = 0;
        while (true)
        {
            int childIndex1 = parentIndex * 2 + 1;
            if (childIndex1 > lastIndex) break;
            int childIndex2 = childIndex1 + 1;
            if (childIndex2 <= lastIndex && data[childIndex2].CompareTo(data[childIndex1]) < 0)
                childIndex1 = childIndex2;
            
            if (data[parentIndex].CompareTo(data[childIndex1]) <= 0) break;
            Swap(parentIndex, childIndex1);
            parentIndex = childIndex1;
        }
        return frontItem;
    }

    private void Swap(int i, int j)
    {
        T temp = data[i];
        data[i] = data[j];
        data[j] = temp;
    }
}

/// <summary>
/// Holds all the data for a single tile in the grid.
/// </summary>
[System.Serializable]
public class TileData : IComparable<TileData>
{
    public GameObject tileObject;
    public int x, z;
    public float gCost;
    public float priority; // Used by the priority queue for this algorithm
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        tileObject.GetComponent<Renderer>().material.color = color;
    }

    // Allows the Priority Queue to sort tiles by our custom priority
    public int CompareTo(TileData other)
    {
        return priority.CompareTo(other.priority);
    }
}

/// <summary>
/// Manages grid generation and runs the "Safest Path" algorithm.
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
    [Tooltip("Higher values make the path avoid obstacles more strongly.")]
    public float clearanceWeight = 5.0f;


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
        RunSafestPath();
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

    void RunSafestPath()
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

        FindPathSafest(startTile, endTile);
    }
    
    void FindPathSafest(TileData start, TileData end)
    {
        Stopwatch sw = Stopwatch.StartNew();

        // This part is slow and could be optimized, but is correct for one-off calculation
        int[,] clearanceMap = ComputeClearanceMap(); 

        PriorityQueue<TileData> openSet = new PriorityQueue<TileData>();
        Dictionary<TileData, float> gCostMap = new Dictionary<TileData, float>();

        foreach (TileData tile in grid)
        {
            gCostMap[tile] = float.MaxValue;
            tile.parent = null;
        }

        gCostMap[start] = 0;
        start.priority = 0;
        openSet.Enqueue(start);

        while (openSet.Count > 0)
        {
            TileData current = openSet.Dequeue();

            if (current == end)
            {
                sw.Stop();
                TracePath(end, sw.Elapsed.TotalMilliseconds, gCostMap[end]);
                return;
            }
            
            if (!current.isStart) current.SetColor(Color.yellow); // Visualize explored

            foreach (TileData neighbor in GetNeighbors(current))
            {
                if (neighbor.isObstacle) continue;
                
                // Prevent cutting corners diagonally
                if (Mathf.Abs(neighbor.x - current.x) == 1 && Mathf.Abs(neighbor.z - current.z) == 1)
                {
                    if (grid[current.x, neighbor.z].isObstacle || grid[neighbor.x, current.z].isObstacle) continue;
                }

                float moveCost = (current.x == neighbor.x || current.z == neighbor.z) ? 1.0f : Sqrt2;
                float newGCost = gCostMap[current] + moveCost;

                if (newGCost < gCostMap[neighbor])
                {
                    gCostMap[neighbor] = newGCost;
                    
                    // Priority is the actual cost PLUS a penalty for being close to obstacles
                    // The clearance map gives distance TO an obstacle, so we want to MAXIMIZE it.
                    // We invert it to a penalty: lower clearance = higher penalty.
                    float clearancePenalty = (clearanceMap.GetLength(0) - clearanceMap[neighbor.x, neighbor.z]) * clearanceWeight;
                    neighbor.priority = newGCost + clearancePenalty;

                    neighbor.parent = current;
                    openSet.Enqueue(neighbor);
                }
            }
        }

        sw.Stop();
        UnityEngine.Debug.Log("Safest path not found.");
    }
    
    // This computes a "distance transform" or clearance map from obstacles
    int[,] ComputeClearanceMap()
    {
        int[,] clearance = new int[gridWidth, gridHeight];
        Queue<(int x, int z, int dist)> queue = new Queue<(int, int, int)>();

        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                if (grid[x, z].isObstacle)
                {
                    clearance[x, z] = 0;
                    queue.Enqueue((x, z, 0));
                }
                else
                {
                    clearance[x, z] = int.MaxValue;
                }
            }
        }

        int[] dx = { 0, 0, 1, -1 };
        int[] dz = { 1, -1, 0, 0 };

        while (queue.Count > 0)
        {
            var (x, z, dist) = queue.Dequeue();

            for (int i = 0; i < 4; i++)
            {
                int nx = x + dx[i];
                int nz = z + dz[i];

                if (IsInBounds(nx, nz) && clearance[nx, nz] > dist + 1)
                {
                    clearance[nx, nz] = dist + 1;
                    queue.Enqueue((nx, nz, dist + 1));
                }
            }
        }
        return clearance;
    }

    List<TileData> GetNeighbors(TileData tile)
    {
        List<TileData> neighbors = new List<TileData>();
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
    void TracePath(TileData endTile, double timeMs, float finalCost)
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

        UnityEngine.Debug.Log($"Safest path found! Cost: {finalCost:F2}, Time: {timeMs:F3} ms, Memory: {memoryMB:F3} MB");
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
            legendText.text = $"Algorithm: Safest Path\n" +
                              $"Path Cost: {(pathFound ? cost.ToString("F2") : "N/A")}\n" +
                              $"Time: {timeMs:F3} ms\n" +
                              $"Memory: {memoryMB:F3} MB";
        }
    }
    #endregion
}
