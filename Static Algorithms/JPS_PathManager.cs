using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Diagnostics;
using System; // Needed for IComparable

// A simple and efficient min-heap Priority Queue implementation
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


[System.Serializable]
public class TileData : IComparable<TileData>
{
    public GameObject tileObject;
    public int x, z;
    public float gCost, hCost;
    public float fCost => gCost + hCost;
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        tileObject.GetComponent<Renderer>().material.color = color;
    }

    // Allows the Priority Queue to sort tiles by fCost, then hCost
    public int CompareTo(TileData other)
    {
        int compare = fCost.CompareTo(other.fCost);
        if (compare == 0)
        {
            compare = hCost.CompareTo(other.hCost);
        }
        return compare;
    }
}

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
    private TileData startNode, endNode;

    // Grid generation and setup
    #region Grid Generation
    void Start()
    {
        StartCoroutine(GenerateGridAndRun());
    }

    System.Collections.IEnumerator GenerateGridAndRun()
    {
        yield return StartCoroutine(GenerateGrid());
        RunJPS();
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

    void RunJPS()
    {
        int sx = Mathf.RoundToInt(startPoint.transform.position.x);
        int sz = Mathf.RoundToInt(startPoint.transform.position.z);
        int ex = Mathf.RoundToInt(endPoint.transform.position.x);
        int ez = Mathf.RoundToInt(endPoint.transform.position.z);

        startNode = grid[sx, sz];
        endNode = grid[ex, ez];

        startNode.isStart = true;
        endNode.isEnd = true;
        startNode.SetColor(Color.green);
        endNode.SetColor(Color.red);

        FindPathJPS(startNode, endNode);
    }
    
    void FindPathJPS(TileData start, TileData end)
    {
        Stopwatch sw = Stopwatch.StartNew();
        
        PriorityQueue<TileData> openSet = new PriorityQueue<TileData>();
        HashSet<TileData> closedSet = new HashSet<TileData>();

        // Initialize grid costs
        foreach (var tile in grid)
        {
            tile.gCost = float.MaxValue;
        }

        start.gCost = 0;
        start.hCost = Heuristic(start, end);
        openSet.Enqueue(start);

        while (openSet.Count > 0)
        {
            TileData current = openSet.Dequeue();
            closedSet.Add(current);

            // Visualize explored jump points
            if (!current.isStart && !current.isEnd) current.SetColor(Color.yellow);

            if (current == end)
            {
                sw.Stop();
                TracePath(end, sw.Elapsed.TotalMilliseconds);
                return;
            }

            IdentifySuccessors(current, end, openSet, closedSet);
        }
        
        sw.Stop();
        UnityEngine.Debug.Log("No path found by JPS.");
        // If no path is found, you might want to log memory usage here as well
        long memoryBytes = System.GC.GetTotalMemory(false);
        double memoryMB = memoryBytes / (1024.0 * 1024.0);
        UpdateLegendText(0, sw.Elapsed.TotalMilliseconds, false, memoryMB);
    }
    
    void IdentifySuccessors(TileData node, TileData end, PriorityQueue<TileData> openSet, HashSet<TileData> closedSet)
    {
        foreach (var neighbor in GetPrunedNeighbors(node))
        {
            TileData jumpPoint = Jump(node, neighbor.x, neighbor.z, end);

            if (jumpPoint != null && !closedSet.Contains(jumpPoint))
            {
                float newGCost = node.gCost + Vector2.Distance(new Vector2(node.x, node.z), new Vector2(jumpPoint.x, jumpPoint.z));
                
                if (newGCost < jumpPoint.gCost)
                {
                    jumpPoint.gCost = newGCost;
                    jumpPoint.hCost = Heuristic(jumpPoint, end);
                    jumpPoint.parent = node;
                    openSet.Enqueue(jumpPoint);
                }
            }
        }
    }

    TileData Jump(TileData parent, int currentX, int currentZ, TileData end)
    {
        int dx = currentX - parent.x;
        int dz = currentZ - parent.z;
        
        if (!IsWalkable(currentX, currentZ)) return null;

        TileData currentNode = grid[currentX, currentZ];
        if (currentNode == end) return currentNode;
        
        // --- Diagonal Case ---
        if (dx != 0 && dz != 0)
        {
            if ((IsWalkable(currentX - dx, currentZ + dz) && !IsWalkable(currentX - dx, currentZ)) ||
                (IsWalkable(currentX + dx, currentZ - dz) && !IsWalkable(currentX, currentZ - dz)))
            {
                return currentNode;
            }
            if (Jump(currentNode, currentX + dx, currentZ, end) != null || Jump(currentNode, currentX, currentZ + dz, end) != null)
            {
                return currentNode;
            }
        }
        else // --- Straight Case ---
        {
            if (dx != 0) // Horizontal
            {
                if ((IsWalkable(currentX + dx, currentZ + 1) && !IsWalkable(currentX, currentZ + 1)) ||
                    (IsWalkable(currentX + dx, currentZ - 1) && !IsWalkable(currentX, currentZ - 1)))
                {
                    return currentNode;
                }
            }
            else // Vertical
            {
                if ((IsWalkable(currentX + 1, currentZ + dz) && !IsWalkable(currentX + 1, currentZ)) ||
                    (IsWalkable(currentX - 1, currentZ + dz) && !IsWalkable(currentX - 1, currentZ)))
                {
                    return currentNode;
                }
            }
        }
        
        // Continue jumping in the same direction
        return Jump(currentNode, currentX + dx, currentZ + dz, end);
    }
    
    List<TileData> GetPrunedNeighbors(TileData node)
    {
        List<TileData> neighbors = new List<TileData>();
        if (node.parent == null) // Start node
        {
            for (int x = -1; x <= 1; x++)
            {
                for (int z = -1; z <= 1; z++)
                {
                    if (x == 0 && z == 0) continue;
                    if (IsWalkable(node.x + x, node.z + z))
                        neighbors.Add(grid[node.x + x, node.z + z]);
                }
            }
            return neighbors;
        }

        int dx = Math.Sign(node.x - node.parent.x);
        int dz = Math.Sign(node.z - node.parent.z);
        
        if (dx != 0 && dz != 0) // Diagonal
        {
            if (IsWalkable(node.x, node.z + dz)) neighbors.Add(grid[node.x, node.z + dz]);
            if (IsWalkable(node.x + dx, node.z)) neighbors.Add(grid[node.x + dx, node.z]);
            if (IsWalkable(node.x + dx, node.z + dz)) neighbors.Add(grid[node.x + dx, node.z + dz]);
        }
        else // Straight
        {
            if (dx == 0) // Vertical
            {
                if (IsWalkable(node.x, node.z + dz)) neighbors.Add(grid[node.x, node.z + dz]);
                if (!IsWalkable(node.x + 1, node.z)) neighbors.Add(grid[node.x + 1, node.z + dz]);
                if (!IsWalkable(node.x - 1, node.z)) neighbors.Add(grid[node.x - 1, node.z + dz]);
            }
            else // Horizontal
            {
                if (IsWalkable(node.x + dx, node.z)) neighbors.Add(grid[node.x + dx, node.z]);
                if (!IsWalkable(node.x, node.z + 1)) neighbors.Add(grid[node.x + dx, node.z + 1]);
                if (!IsWalkable(node.x, node.z - 1)) neighbors.Add(grid[node.x + dx, node.z - 1]);
            }
        }
        return neighbors;
    }

    bool IsWalkable(int x, int z)
    {
        return IsInBounds(x, z) && !grid[x, z].isObstacle;
    }

    bool IsInBounds(int x, int z)
    {
        return x >= 0 && x < gridWidth && z >= 0 && z < gridHeight;
    }

    // Octile distance heuristic for grids allowing diagonal movement
    float Heuristic(TileData a, TileData b)
    {
        int dx = Math.Abs(a.x - b.x);
        int dz = Math.Abs(a.z - b.z);
        float diag = Mathf.Sqrt(2);
        return diag * Math.Min(dx, dz) + (Math.Max(dx, dz) - Math.Min(dx, dz));
    }
    
    void TracePath(TileData endTile, double timeMs)
    {
        // --- MEMORY CALCULATION ---
        long memoryBytes = System.GC.GetTotalMemory(false);
        double memoryMB = memoryBytes / (1024.0 * 1024.0);

        TileData current = endTile;
        while (current != null)
        {
            if (current.parent != null)
            {
                Vector3 startPos = new Vector3(current.x, 0.5f, current.z);
                Vector3 endPos = new Vector3(current.parent.x, 0.5f, current.parent.z);
                UnityEngine.Debug.DrawLine(startPos, endPos, Color.magenta, 60f);
            }
            current = current.parent;
        }

        float finalCost = endTile.gCost; // Correct path cost
        
        // --- CONSOLE LOG WITH MEMORY ---
        UnityEngine.Debug.Log($"JPS path found! Cost: {finalCost:F2}, Time: {timeMs:F3} ms, Memory: {memoryMB:F3} MB");
        
        UpdateLegendText(finalCost, timeMs, true, memoryMB);
    }

    void UpdateLegendText(float cost, double timeMs, bool pathFound, double memoryMB)
    {
        if (legendText != null)
        {
            legendText.text = $"Algorithm: JPS\n" +
                              $"Path Cost: {(pathFound ? cost.ToString("F2") : "N/A")}\n" +
                              $"Time: {timeMs:F3} ms\n" +
                              $"Memory: {memoryMB:F3} MB";
        }
    }
}
