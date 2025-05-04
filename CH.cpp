#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <chrono>
#include <algorithm>
using namespace std;
using namespace std::chrono;

const int INF = numeric_limits<int>::max();

// Original graph: each node maps to a vector of (neighbor, weight)
unordered_map<int, vector<pair<int, int>>> graph = {
    {0, {{1, 4}, {2, 1}}},
    {1, {{3, 1}}},
    {2, {{1, 2}, {3, 5}}},
    {3, {}}
};

// CH data structures: upward and downward graphs and nodeRank.
// upGraph: edges go from lower (less important) to higher (more important) nodes.
// downGraph: the reverse of these edges.
unordered_map<int, vector<pair<int, int>>> upGraph, downGraph;
unordered_map<int, int> nodeRank;

// Utility to add or update an edge from u to v in graph 'g' with weight w.
void addEdge(unordered_map<int, vector<pair<int,int>>>& g, int u, int v, int w) {
    bool updated = false;
    for (auto &edge : g[u]) {
        if (edge.first == v) {
            if (w < edge.second)
                edge.second = w; // update if the new weight is better
            updated = true;
            break;
        }
    }
    if (!updated)
        g[u].push_back({v, w});
}

// Preprocess the graph to build contraction hierarchies.
// This builds the initial upward/downward graphs then contracts nodes (adding shortcuts).
void preprocessCH() {
    auto prepStart = high_resolution_clock::now();

    // Assign node ranks: lower rank = less important (contracted earlier).
    // Here we use natural order: 0, 1, 2, 3.
    nodeRank = { {0, 0}, {1, 1}, {2, 2}, {3, 3} };

    // Clear any old data.
    upGraph.clear();
    downGraph.clear();

    // Step 1: Build the initial upward and downward graphs using the original edges.
    for (auto &p : graph) {
        int u = p.first;
        for (auto &edge : p.second) {
            int v = edge.first, w = edge.second;
            // If u's rank is lower than v's, the edge goes u -> v (upward).
            if (nodeRank[u] < nodeRank[v]) {
                addEdge(upGraph, u, v, w);
                addEdge(downGraph, v, u, w);
            } else {
                addEdge(upGraph, v, u, w);
                addEdge(downGraph, u, v, w);
            }
        }
    }

    // Step 2: Contract nodes in increasing rank order.
    // Gather nodes sorted by rank.
    vector<pair<int,int>> nodes; // pair: (node, rank)
    for (auto &p : nodeRank) {
        nodes.push_back({p.first, p.second});
    }
    sort(nodes.begin(), nodes.end(), [](auto &a, auto &b) { return a.second < b.second; });

    // For each node v (to be contracted), add a shortcut from every incoming neighbor u to
    // every outgoing neighbor w if the ordering holds and if it improves the route.
    for (auto &p : nodes) {
        int v = p.first;
        // For every edge coming into v (u -> v):
        for (auto &inEdge : downGraph[v]) {
            int u = inEdge.first;
            int weight_uv = inEdge.second;
            // For every edge going out of v (v -> w):
            for (auto &outEdge : upGraph[v]) {
                int w = outEdge.first;
                int weight_vw = outEdge.second;
                // Only add shortcut if nodeRank[u] < nodeRank[v] < nodeRank[w].
                if (nodeRank[u] < nodeRank[v] && nodeRank[v] < nodeRank[w]) {
                    int shortcutWeight = weight_uv + weight_vw;
                    addEdge(upGraph, u, w, shortcutWeight);
                    addEdge(downGraph, w, u, shortcutWeight);
                }
            }
        }
    }

    auto prepEnd = high_resolution_clock::now();
    cout << "Preprocessing time: "
         << duration_cast<microseconds>(prepEnd - prepStart).count()
         << " μs\n";
}

// Bidirectional CH Query using the upward graph (from start) and the downward graph (from goal).
int CHQuery(int start, int goal) {
    if (start == goal)
        return 0;

    unordered_map<int,int> distUp, distDown;
    for (auto &p : graph) {
        distUp[p.first] = INF;
        distDown[p.first] = INF;
    }

    // Priority queues store (distance, node).
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pqUp, pqDown;

    distUp[start] = 0;
    pqUp.push({0, start});
    distDown[goal] = 0;
    pqDown.push({0, goal});

    int best = INF;

    while (!pqUp.empty() || !pqDown.empty()) {
        if (!pqUp.empty()) {
            auto [d, u] = pqUp.top();
            pqUp.pop();
            if (d > distUp[u])
                continue;
            // Relax outgoing upward edges.
            for (auto &edge : upGraph[u]) {
                int v = edge.first, weight = edge.second;
                if (distUp[v] > distUp[u] + weight) {
                    distUp[v] = distUp[u] + weight;
                    pqUp.push({distUp[v], v});
                }
            }
            if (distDown[u] != INF)
                best = min(best, distUp[u] + distDown[u]);
        }

        if (!pqDown.empty()) {
            auto [d, u] = pqDown.top();
            pqDown.pop();
            if (d > distDown[u])
                continue;
            // Relax outgoing downward edges.
            for (auto &edge : downGraph[u]) {
                int v = edge.first, weight = edge.second;
                if (distDown[v] > distDown[u] + weight) {
                    distDown[v] = distDown[u] + weight;
                    pqDown.push({distDown[v], v});
                }
            }
            if (distUp[u] != INF)
                best = min(best, distUp[u] + distDown[u]);
        }
    }

    return best == INF ? -1 : best;
}

int main() {
    preprocessCH();

    cout << "\nUpward graph:\n";
    for (auto &p : upGraph) {
        int u = p.first;
        cout << u << ": ";
        for (auto &edge : p.second) {
            cout << "(" << edge.first << "," << edge.second << ") ";
        }
        cout << "\n";
    }

    auto queryStart = high_resolution_clock::now();
    int dist = CHQuery(0, 3);
    auto queryEnd = high_resolution_clock::now();

    cout << "\nCH distance: " << dist << "\n";
    cout << "Time to perform CH query: "
         << duration_cast<microseconds>(queryEnd - queryStart).count()
         << " μs\n";

    return 0;
}
