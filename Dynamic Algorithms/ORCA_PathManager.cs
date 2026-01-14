// PathManager.cs (A* + ORCA Baseline, Final Corrected Version)
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

[System.Serializable]
public class TileData
{
    public GameObject tileObject;
    public int x, z;
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        if (tileObject != null && tileObject.GetComponent<Renderer>() != null)
        {
            tileObject.GetComponent<Renderer>().material.color = color;
        }
    }
}

public class AgentData
{
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 preferredVelocity;
    public float radius = 0.5f;
    public TileData currentTile;
    public List<Vector3> pathHistory = new List<Vector3>();

    // For stuck detection
    public Vector3 lastPosition;
    public float stuckTimer = 0f;
    public float stuckCheckInterval = 0.5f;

    // For metrics
    public Vector3 lastVelocity;
    public float sumSquaredAccel = 0f;
    public int smoothSamples = 0;
}

public struct OrcaConstraint
{
    public Vector3 point;
    public Vector3 normal;

    public OrcaConstraint(Vector3 p, Vector3 n)
    {
        point = p;
        normal = n.normalized;
    }
}

public class PathManager : MonoBehaviour
{
    // Scene references
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public Text legendText;

    // Grid properties
    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 300; // Adjusted from 400
    private TileData[,] grid;
    private TileData startTile, goalTile;

    // Agent properties
    private AgentData agent;

    // Simulation parameters
    private float maxSpeed = 3f;
    private float timeHorizon = 3f;
    private float dt = 0.07f;

    // Path properties
    private List<TileData> initialPath = new List<TileData>();
    private List<Vector3> waypoints = new List<Vector3>();
    private int currentWaypointIndex = 0;
    private float waypointTolerance = 0.35f;

    // Metric variables
    private Stopwatch missionStopwatch;
    private int collisionFlag = 0;
    private float minClearanceAlongFlight = float.MaxValue;
    private float sumClearanceAlongFlight = 0f;
    private int clearanceSamples = 0;
    private float sumDeviation = 0f;
    private int deviationSamples = 0;
    private int[,] clearanceMap;
    private string csvPath;
    private long totalReplanMs = 0;
    private float lastPathCost = 0f;

    void Start()
    {
        if (tilePrefab == null || startPoint == null || endPoint == null)
        {
            Debug.LogError("FATAL ERROR: A public variable (TilePrefab, StartPoint, or EndPoint) is not assigned in the Inspector.");
            return; // Stop execution
        }
        StartCoroutine(SetupAndRun());
    }

    IEnumerator SetupAndRun()
    {
        // 1. Setup the environment
        yield return StartCoroutine(GenerateGrid());
        yield return StartCoroutine(PlaceFixedObstacles());
        clearanceMap = ComputeClearanceMap();

        // 2. Define Start and Goal
        Vector3 sp = startPoint.transform.position;
        Vector3 ep = endPoint.transform.position;
        int sx = Mathf.Clamp(Mathf.RoundToInt(sp.x), 0, gridWidth - 1);
        int sz = Mathf.Clamp(Mathf.RoundToInt(sp.z), 0, gridHeight - 1);
        int gx = Mathf.Clamp(Mathf.RoundToInt(ep.x), 0, gridWidth - 1);
        int gz = Mathf.Clamp(Mathf.RoundToInt(ep.z), 0, gridHeight - 1);

        startTile = grid[sx, sz];
        goalTile = grid[gx, gz];
        startTile.isStart = true;
        goalTile.isEnd = true;
        startTile.SetColor(Color.green);
        goalTile.SetColor(Color.red);

        // 3. Find the initial path
        initialPath = AStar(startTile, goalTile);
        if (initialPath == null || initialPath.Count == 0)
        {
            Debug.LogError("CRITICAL PROBLEM: Initial A* could not find a path. Simulation aborted. Please ensure Start and End points are in reachable locations.");
            yield break; // Stop the coroutine completely
        }

        lastPathCost = ComputePathCost(initialPath);
        ColorPathTiles(initialPath, Color.cyan);
        yield return new WaitForSeconds(1f);

        // 4. Create dynamic event and replan
        InsertDynamicObstaclesOnPath();
        InitAgent();
        ReplanForAgentAndSetWaypoints();

        // 5. Initialize agent and start simulation
        
        missionStopwatch = Stopwatch.StartNew();
        yield return StartCoroutine(RunOrcaSimulation());
    }

    IEnumerator RunOrcaSimulation()
    {
        while (true)
        {
            // Update preferred velocity towards next waypoint
            if (waypoints != null && currentWaypointIndex < waypoints.Count)
            {
                Vector3 target = waypoints[currentWaypointIndex];
                if (Vector3.Distance(agent.position, target) < waypointTolerance)
                {
                    currentWaypointIndex++;
                }

                if (currentWaypointIndex < waypoints.Count)
                {
                    agent.preferredVelocity = (waypoints[currentWaypointIndex] - agent.position).normalized * maxSpeed;
                }
                else
                {
                    agent.preferredVelocity = (goalTile.tileObject.transform.position - agent.position).normalized * maxSpeed;
                }
            }
            else
            {
                agent.preferredVelocity = (goalTile.tileObject.transform.position - agent.position).normalized * maxSpeed;
            }

            // Get nearby obstacles and compute ORCA velocity
            List<AgentData> obstacleAgents = GetObstacleAgentsAround(agent.position, 3f);
            Vector3 newVelocity = ComputeOrcaVelocity(agent, obstacleAgents);
            agent.velocity = Vector3.ClampMagnitude(newVelocity, maxSpeed);

            // Store last position and move agent
            Vector3 lastPos = agent.position;
            agent.position += agent.velocity * dt;

            // --- METRICS CALCULATION ---
            UpdateMetrics(lastPos);

            // Check for goal
            if (Vector3.Distance(agent.position, goalTile.tileObject.transform.position) < 0.5f)
            {
                missionStopwatch.Stop();
                WriteSummaryCSV();
                Debug.Log("Agent reached goal! Metrics saved to CSV.");
                if (legendText != null) legendText.text = "Goal Reached!";
                yield break; // End the simulation
            }

            UpdateLegendText();
            yield return new WaitForSeconds(dt);
        }
    }

    void UpdateMetrics(Vector3 lastPos)
    {
        // 1. Smoothness (Mean Squared Acceleration)
        if (dt > 0.001f)
        {
            Vector3 acceleration = (agent.velocity - agent.lastVelocity) / dt;
            agent.sumSquaredAccel += acceleration.sqrMagnitude;
            agent.smoothSamples++;
        }
        agent.lastVelocity = agent.velocity;

        // 2. Collision & Sliding
        if (IsPositionInObstacle(agent.position))
        {
            collisionFlag = 1;
            agent.position = lastPos; // Stop agent at point of collision
            agent.velocity = Vector3.zero;
        }

        // 3. Clearance
        int agentX = Mathf.Clamp(Mathf.RoundToInt(agent.position.x), 0, gridWidth - 1);
        int agentZ = Mathf.Clamp(Mathf.RoundToInt(agent.position.z), 0, gridHeight - 1);
        float currentClearance = clearanceMap[agentX, agentZ];
        sumClearanceAlongFlight += currentClearance;
        minClearanceAlongFlight = Mathf.Min(minClearanceAlongFlight, currentClearance);
        clearanceSamples++;

        // 4. Deviation from Replanned Path
        if (waypoints != null && currentWaypointIndex < waypoints.Count)
        {
            Vector3 targetWaypoint = waypoints[currentWaypointIndex];
            Vector3 prevWaypoint = (currentWaypointIndex == 0) ? startTile.tileObject.transform.position : waypoints[currentWaypointIndex - 1];
            float currentDeviation = FindDistanceToLineSegment(agent.position, prevWaypoint, targetWaypoint);
            sumDeviation += currentDeviation;
            deviationSamples++;
        }
    }

    void UpdateLegendText()
    {
        if (legendText != null)
        {
            legendText.text = $"Path Cost: {lastPathCost:F2}\n" +
                              $"Replan Time: {totalReplanMs} ms\n" +
                              $"Mission Time: {missionStopwatch.ElapsedMilliseconds} ms\n" +
                              $"Speed: {agent.velocity.magnitude:F2} m/s\n" +
                              $"Collision: {(collisionFlag == 1 ? "Yes" : "No")}";
        }
    }

   void WriteSummaryCSV()
{
    csvPath = Path.Combine(Application.persistentDataPath, $"AStar_ORCA_results_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
    try
    {
        using (var sw = new StreamWriter(csvPath))
        {
            sw.WriteLine("Algorithm,PathCost,PlanTimeMs,CollisionFlag,AvgClearance,MinClearance,AvgDeviation,MeanSquaredAccel,MissionTimeMs");

            float avgClearance = (clearanceSamples > 0) ? sumClearanceAlongFlight / clearanceSamples : 0f;
            float avgDeviation = (deviationSamples > 0) ? sumDeviation / deviationSamples : 0f;
            float meanSqAccel = (agent.smoothSamples > 0) ? agent.sumSquaredAccel / agent.smoothSamples : 0f;
            long missionTime = missionStopwatch.ElapsedMilliseconds;

            sw.WriteLine($"AStar+ORCA,{lastPathCost:F2},{totalReplanMs},{collisionFlag},{avgClearance:F2},{minClearanceAlongFlight:F2},{avgDeviation:F2},{meanSqAccel:F4},{missionTime}");

            // ✅ Console log
            Debug.Log(
                $"==== Simulation Metrics ====\n" +
                $"Path Cost     : {lastPathCost:F2}\n" +
                $"Replan Time   : {totalReplanMs} ms\n" +
                $"Mission Time  : {missionTime} ms\n" +
                $"CollisionFlag : {collisionFlag}\n" +
                $"AvgClearance  : {avgClearance:F2}\n" +
                $"MinClearance  : {minClearanceAlongFlight:F2}\n" +
                $"AvgDeviation  : {avgDeviation:F2}\n" +
                $"MeanSqAccel   : {meanSqAccel:F4}\n" +
                $"============================"
            );
        }
        Debug.Log($"CSV saved to {csvPath}");
    }
    catch (Exception ex)
    {
        Debug.LogError("Error writing CSV: " + ex.Message);
    }
}


    #region Helper Functions

    void InitAgent()
    {
        agent = new AgentData
        {
            position = startTile.tileObject.transform.position + Vector3.up * 0.5f,
            velocity = Vector3.zero,
            radius = 0.5f,
            currentTile = startTile
        };
        agent.lastPosition = agent.position;
        agent.lastVelocity = Vector3.zero;
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
                tileObj.transform.SetParent(this.transform);

                Renderer renderer = tileObj.GetComponent<Renderer>();
                if (renderer)
                {
                    renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    renderer.receiveShadows = false;
                }

                TileData tile = new TileData { tileObject = tileObj, x = x, z = z };
                tile.SetColor(Color.white);
                grid[x, z] = tile;
            }
        }
        yield return null;
    }

    IEnumerator PlaceFixedObstacles()
    {
        int placed = 0;
        System.Random rand = new System.Random(300);
        while (placed < obstacleCount)
        {
            int ox = rand.Next(0, gridWidth);
            int oz = rand.Next(0, gridHeight);
            if ((startPoint != null && ox == Mathf.RoundToInt(startPoint.transform.position.x) && oz == Mathf.RoundToInt(startPoint.transform.position.z)) ||
                (endPoint != null && ox == Mathf.RoundToInt(endPoint.transform.position.x) && oz == Mathf.RoundToInt(endPoint.transform.position.z)))
            {
                continue;
            }

            TileData tile = grid[ox, oz];
            if (tile.isObstacle) continue;

            tile.isObstacle = true;
            tile.SetColor(Color.black);

            tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
            Vector3 p = tile.tileObject.transform.position;
            p.y = 0.5f;
            tile.tileObject.transform.position = p;

            placed++;
        }
        yield return null;
    }

    void InsertDynamicObstaclesOnPath()
    {
        if (initialPath == null || initialPath.Count < 5) return;

        int numObstacles = Mathf.Min(4, Mathf.Max(1, initialPath.Count / 3));
        for (int i = 1; i <= numObstacles; i++)
        {
            int idx = (i * initialPath.Count) / (numObstacles + 1);
            if (idx > 0 && idx < initialPath.Count - 1)
            {
                TileData tile = initialPath[idx];
                if (!tile.isStart && !tile.isEnd && !tile.isObstacle)
                {
                    tile.isObstacle = true;
                    tile.SetColor(Color.magenta);
                    tile.tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
                    Vector3 pos = tile.tileObject.transform.position;
                    pos.y = 0.5f;
                    tile.tileObject.transform.position = pos;
                }
            }
        }
    }

    List<TileData> AStar(TileData start, TileData goal)
    {
        if (start == null || goal == null) return null;
        var open = new List<TileData>();
        var gScore = new Dictionary<TileData, float>();
        var cameFrom = new Dictionary<TileData, TileData>();
        var closed = new HashSet<TileData>();

        open.Add(start);
        gScore[start] = 0f;

        while (open.Count > 0)
        {
            int bestIdx = 0;
            for (int i = 1; i < open.Count; i++)
            {
                if ((gScore[open[i]] + Heuristic(open[i], goal)) < (gScore[open[bestIdx]] + Heuristic(open[bestIdx], goal)))
                {
                    bestIdx = i;
                }
            }
            TileData current = open[bestIdx];
            open.RemoveAt(bestIdx);

            if (current == goal)
            {
                List<TileData> path = new List<TileData>();
                TileData node = goal;
                while (node != null)
                {
                    path.Add(node);
                    node = cameFrom.ContainsKey(node) ? cameFrom[node] : null;
                }
                path.Reverse();
                return path;
            }
            closed.Add(current);

            foreach (var neighbor in GetNeighbors(current))
            {
                if (neighbor.isObstacle || closed.Contains(neighbor)) continue;
                float tentativeG = gScore.ContainsKey(current) ? gScore[current] + CostBetween(current, neighbor) : float.MaxValue;
                if (!gScore.ContainsKey(neighbor) || tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    if (!open.Contains(neighbor)) open.Add(neighbor);
                }
            }
        }
        return null; // no path
    }

    List<TileData> GetNeighbors(TileData tile)
    {
        List<TileData> neighbors = new List<TileData>();
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dz == 0) continue;
                int nx = tile.x + dx;
                int nz = tile.z + dz;
                if (nx < 0 || nx >= gridWidth || nz < 0 || nz >= gridHeight) continue;
                if (Mathf.Abs(dx) + Mathf.Abs(dz) == 2)
                {
                    if (grid[tile.x + dx, tile.z].isObstacle || grid[tile.x, tile.z + dz].isObstacle) continue;
                }
                neighbors.Add(grid[nx, nz]);
            }
        }
        return neighbors;
    }

    float Heuristic(TileData a, TileData b)
    {
        float dx = Mathf.Abs(a.x - b.x);
        float dz = Mathf.Abs(a.z - b.z);
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    float CostBetween(TileData a, TileData b)
    {
        return (a.x == b.x || a.z == b.z) ? 1f : 1.41421f;
    }

    float ComputePathCost(List<TileData> path)
    {
        if (path == null || path.Count < 2) return 0f;
        float cost = 0f;
        for (int i = 1; i < path.Count; i++) cost += CostBetween(path[i - 1], path[i]);
        return cost;
    }

    void ColorPathTiles(List<TileData> path, Color color)
    {
        if (path == null) return;
        foreach (var t in path)
        {
            if (!t.isStart && !t.isEnd && !t.isObstacle) t.SetColor(color);
        }
    }

    void ReplanForAgentAndSetWaypoints()
    {
        var sw = Stopwatch.StartNew();
        List<TileData> newPath = AStar(agent.currentTile, goalTile);
        sw.Stop();
        totalReplanMs += sw.ElapsedMilliseconds;

        if (newPath == null || newPath.Count == 0)
        {
            Debug.LogWarning("Replan failed: no path available.");
            waypoints.Clear();
            return;
        }
        lastPathCost = ComputePathCost(newPath);
        ColorPathTiles(newPath, Color.yellow);
        waypoints.Clear();
        foreach (var tile in newPath) waypoints.Add(tile.tileObject.transform.position + Vector3.up * 0.5f);
        currentWaypointIndex = 0;
    }

    List<AgentData> GetObstacleAgentsAround(Vector3 position, float searchRadius)
    {
        List<AgentData> agents = new List<AgentData>();
        float searchRadiusSq = searchRadius * searchRadius;
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                if (grid[x, z].isObstacle)
                {
                    Vector3 pos = grid[x, z].tileObject.transform.position + Vector3.up * 0.5f;
                    if ((pos - position).sqrMagnitude < searchRadiusSq)
                    {
                        agents.Add(new AgentData { position = pos, velocity = Vector3.zero, radius = 0.5f });
                    }
                }
            }
        }
        return agents;
    }

    Vector3 ComputeOrcaVelocity(AgentData agent, List<AgentData> obstacleAgents)
    {
        List<OrcaConstraint> constraints = new List<OrcaConstraint>();
        float invTimeHorizon = 1f / timeHorizon;

        foreach (var other in obstacleAgents)
        {
            Vector3 relativePosition = other.position - agent.position;
            Vector3 relativeVelocity = agent.velocity - other.velocity;
            float distSq = relativePosition.sqrMagnitude;
            float combinedRadius = agent.radius + other.radius;
            float combinedRadiusSq = combinedRadius * combinedRadius;

            Vector3 u;
            if (distSq > combinedRadiusSq)
            {
                Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;
                float wLengthSq = w.sqrMagnitude;
                float dotProduct = Vector3.Dot(w, relativePosition);
                if (dotProduct < 0f && dotProduct * dotProduct > combinedRadiusSq * wLengthSq)
                {
                    float wLength = Mathf.Sqrt(wLengthSq);
                    Vector3 unitW = w / wLength;
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    constraints.Add(new OrcaConstraint(agent.velocity + 0.5f * u, unitW));
                    continue;
                }
            }

            float invTimeStep = 1f / dt;
            Vector3 w2 = relativeVelocity - invTimeStep * relativePosition;
            float w2Length = w2.magnitude;
            Vector3 unitW2 = w2 / w2Length;
            u = (combinedRadius * invTimeStep - w2Length) * unitW2;
            constraints.Add(new OrcaConstraint(agent.velocity + 0.5f * u, unitW2));
        }
        
        return LinearProgram(agent.preferredVelocity, constraints, maxSpeed);
    }

    Vector3 LinearProgram(Vector3 preferredVelocity, List<OrcaConstraint> constraints, float maxSpeed)
    {
        Vector3 newVelocity = preferredVelocity;
        for (int i = 0; i < constraints.Count; ++i)
        {
            float dot = Vector3.Dot(newVelocity - constraints[i].point, constraints[i].normal);
            if (dot < 0.0f)
            {
                newVelocity -= dot * constraints[i].normal;
            }
        }
        if (newVelocity.sqrMagnitude > maxSpeed * maxSpeed)
        {
            newVelocity = Vector3.Normalize(newVelocity) * maxSpeed;
        }
        return newVelocity;
    }

    private float FindDistanceToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        point.y = 0; lineStart.y = 0; lineEnd.y = 0;
        if ((lineStart - lineEnd).sqrMagnitude < 0.001f) return Vector3.Distance(point, lineStart);
        float t = Mathf.Clamp01(Vector3.Dot(point - lineStart, lineEnd - lineStart) / (lineEnd - lineStart).sqrMagnitude);
        return Vector3.Distance(point, lineStart + t * (lineEnd - lineStart));
    }

    bool IsPositionInObstacle(Vector3 pos)
    {
        int x = Mathf.RoundToInt(pos.x);
        int z = Mathf.RoundToInt(pos.z);
        if (x < 0 || x >= gridWidth || z < 0 || z >= gridHeight) return true;
        return grid[x, z].isObstacle;
    }

    TileData GetTileFromPosition(Vector3 pos)
    {
        int x = Mathf.RoundToInt(pos.x);
        int z = Mathf.RoundToInt(pos.z);
        if (x < 0 || x >= gridWidth || z < 0 || z >= gridHeight) return null;
        return grid[x, z];
    }

    int[,] ComputeClearanceMap()
    {
        int[,] clearance = new int[gridWidth, gridHeight];
        Queue<(int, int, int)> queue = new Queue<(int, int, int)>();
        for (int x = 0; x < gridWidth; x++) for (int z = 0; z < gridHeight; z++)
        {
            if (grid[x,z].isObstacle) { clearance[x,z] = 0; queue.Enqueue((x,z,0)); }
            else { clearance[x,z] = int.MaxValue; }
        }
        int[] dx = {0,0,1,-1}, dz = {1,-1,0,0};
        while(queue.Count > 0)
        {
            var (x,z,dist) = queue.Dequeue();
            for(int i=0; i<4; i++)
            {
                int nx=x+dx[i], nz=z+dz[i];
                if(nx>=0 && nx<gridWidth && nz>=0 && nz<gridHeight && clearance[nx,nz] > dist+1)
                {
                    clearance[nx,nz] = dist+1;
                    queue.Enqueue((nx,nz,dist+1));
                }
            }
        }
        return clearance;
    }
    
    #endregion
}
