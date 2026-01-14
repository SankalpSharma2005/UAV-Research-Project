# An Efficient Hybrid Path Planning Approach for UAV Delivery

This repository contains the implementation and experimental evaluation of multiple UAV path planning algorithms developed as part of the research paper:

**“An Efficient Hybrid Path Planning Approach for UAV Delivery”**  
*Accepted at the 8th International Conference on Recent Trends in Advanced Computing (ICRTAC’25)*

The project focuses on analyzing classical and modern path planning techniques and proposes a novel hybrid approach to achieve safer and faster UAV navigation.

---

## 📌 Research Overview

Unmanned Aerial Vehicle (UAV) delivery systems require efficient path planning algorithms that balance **optimality, safety, and computational efficiency**.  
This research presents a comparative study of existing algorithms and introduces a **new hybrid Voronoi + ORCA-based approach** for improved UAV navigation in dynamic environments.

---

## ⚙️ Algorithms Implemented

### Static Path Planning Algorithms
- Dijkstra’s Algorithm  
- A* Algorithm  
- Jump Point Search (JPS) 
- Voronoi Diagram 
- Depth-First Search (DFS)  

### Dynamic Path Planning Algorithms
- D* Lite  
- Lifelong Planning A* (LPA)  
- Artificial Potential Field (APF) 
- ORCA (Optimal Reciprocal Collision Avoidance)  
- Bug2

### Hybrid Algorithms
- Existing commonly used hybrid algorithm  
- **Proposed Voronoi + ORCA Hybrid Algorithm (Novel Contribution)**

---

## 🚀 Proposed Hybrid Solution

The **Voronoi + ORCA hybrid algorithm** combines:
- **Voronoi diagrams** for global path safety and obstacle clearance  
- **ORCA** for real-time local collision avoidance  

### Key Advantages:
- Faster path computation  
- Improved obstacle avoidance  
- Safer navigation in dynamic environments  
- Reduced collision risk compared to standalone methods  

---

## 🧪 Experimental Setup

- **Simulation Environment:** Unity Hub  
- **Environment Type:** Grid-based UAV navigation with static and dynamic obstacles  
- **Evaluation Metrics:**
  -**For Static:**
    - Path Cost
    - Computational time (in ms)
    - Memory (in MB)
  -**For Dynamic:**
    - Path Cost
    - Total Execution Time(in ms)
    - Replanning Time(in ms)
    - Memory Usage change during Replanning(in MB)
    - Total Memory Usage(in MB)

All algorithms were implemented and evaluated under identical conditions for fair comparison.

---

## 📊 Results & Analysis

- Static algorithms performed well in obstacle-free environments but lacked adaptability.
- Dynamic algorithms handled moving obstacles effectively but incurred higher computation costs.
- The proposed **Voronoi + ORCA hybrid algorithm** outperformed existing approaches by achieving:
  - Faster convergence
  - Safer paths
  - Better real-time adaptability

Detailed quantitative and qualitative results are discussed in the research paper.

---

## 🛠️ Tools & Technologies Used

- **Simulation Engine:** Unity Hub  
- **Programming Language:** C#  
- **Visualization:** Real-time path rendering and obstacle interaction  
- **Research Methodology:** Comparative algorithm analysis and hybrid optimization  

---

## 📄 Publication Status

- **Paper Status:** Accepted  
- **Conference:** 8th International Conference on Recent Trends in Advanced Computing (ICRTAC’25)  
- **Publication:** To be published

---

## 📬 Contact

**Sankalp Sharma**  
📧 Email: sankalp.sharma2005@gmail.com  
🔗 GitHub: https://github.com/SankalpSharma2005  
🔗 LinkedIn: https://www.linkedin.com/in/sankalp-sharma-9283b0312  



