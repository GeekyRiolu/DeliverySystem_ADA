# 📦 Optimized Delivery System Using Fractional Knapsack and Dijkstra's Algorithm

> 🚴‍♂️ A smart delivery optimization project using classical algorithms — maximizing delivery value and minimizing travel time & carbon emissions.

---

## 🧩 Problem Statement

A delivery partner must carry multiple packages across a city. Each package has:
- A **value** (based on tip or delivery priority),
- A **weight** (in kg), and
- A **destination node** (on a city map graph).

The city is modeled as a graph:
- **Nodes** = Intersections,
- **Edges** = Roads with travel times (or distances).

The partner’s bag has a maximum weight limit.

---

## 🎯 Objectives

1. **Package Selection – Fractional Knapsack Algorithm**
   - Maximize the total value of carried packages without exceeding the bag's weight limit.
   - Fractional items allowed.

2. **Routing – Dijkstra's Algorithm**
   - Calculate the shortest/fastest path from the hub to all selected delivery nodes.

3. **Eco Impact – Carbon Emission Awareness**
   - Calculate total CO₂ emission and suggest lower-emission routes.

---

## 🔄 Workflow

```plaintext
Input Packages → Apply Fractional Knapsack → Select Top Value/kg Packages
→ Build City Graph → Apply Dijkstra's Algorithm
→ Calculate Route, Travel Time, and Emission
→ Display Final Output
