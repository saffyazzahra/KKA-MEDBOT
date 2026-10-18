# HOSPITAL MEDBOT DELIVERY

## GROUP MEMBERS
|Student Name|NRP|
|---|---|
|Agile Octa Agrakha Handrian|5025251010|
|Nisrina Zahralilla|5025251013|
|Padhang Abiyu Fikri|5025251014|
|Safiya Nadira Az-zahra|5025251268|

## 🗺️ Fixed Map Layout & Simulation Overview

> **Notice:** This is the **fixed map** layout used for the simulation runs.

---

## 📷 Map Layout
<img width="1090" height="492" alt="image" src="https://github.com/user-attachments/assets/0a31e039-8511-40e0-833e-630454001e1d" />

---

## ⚙️ Algorithms Used
* **BFS (Breadth-First Search):** Unweighted hop-count pathfinding.
* **UCS (Uniform Cost Search):** Optimal physical distance/edge-weight pathfinding.
* **A\* (A-Star Search):** Heuristic-guided optimal pathfinding.

---

## 📊 Metrics Being Measured
* **On-Time Deliveries:** Count and percentage of tasks completed before their deadlines.
*   * *What it tells us:* Evaluates the binary success rate of the mission under strict scheduling constraints, showing how reliably each algorithm meets the established deadlines.
* **Total Lateness:** Cumulative minutes overdue across all missed deliveries.
*   * *What it tells us:* Measures the cumulative magnitude of deadline failures, indicating how severely overdue missed deliveries drifted past their targets across the run.
* **Lateness Locations:** Specific nodes where delays occurred (`bfs nodes || ucs nodes || a* nodes`).
*   * *What it tells us:* Highlights the specific rooms where delays occurred, allowing to track cascading bottlenecks and diagnose where an algorithm fell behind.
* **Total Delivery Cost:** Cumulative walking distance/time to complete all delivery.
*   * *What it tells us:* Quantifies the physical walking effort/time required to complete all delivery, revealing whether an algorithm chooses efficient paths.
* **Return Path Cost:** Walking distance/time from the final delivery back to the base (`pharmacy`).
*   * *What it tells us:* Isolates the distance and cost of the final trip back from the last delivery station to the base (`pharmacy`).
* **Total Trip Cost:** Combined sum of the delivery cost and return cost.
*   * *What it tells us:* Provides the aggregate cost of the entire mission (delivery + return trip combined) for overall resource expenditure.
* **Nodes Expanded:** Total count of graph nodes explored by each search algorithm.
*   * *What it tells us:* Measures algorithmic search efficiency and computational overhead by showing how many graph nodes each search had to explore to find its paths.
