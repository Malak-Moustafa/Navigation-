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

unordered_map<int, vector<pair<int, int>>> graph = {
    {0, {{1, 4}, {2, 1}}}, {1, {{3, 1}}}, {2, {{1, 2}, {3, 5}}}, {3, {}}
};

unordered_map<int, unordered_map<int, int>> landmarkDist;

void precomputeLandmarks(vector<int>& landmarks) {
    for (int L : landmarks) {
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        unordered_map<int, int> dist;
        for (auto& node : graph) dist[node.first] = INF;
        dist[L] = 0; pq.push({0, L});

        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop();
            for (auto [v, w] : graph[u]) {
                if (dist[v] > d + w) {
                    dist[v] = d + w;
                    pq.push({dist[v], v});
                }
            }
        }
        landmarkDist[L] = dist;
    }
}

int ALT(int start, int goal, vector<int>& landmarks) {
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    unordered_map<int, int> gScore;
    for (auto& node : graph) gScore[node.first] = INF;
    gScore[start] = 0; pq.push({0, start});

    while (!pq.empty()) {
        auto [f, u] = pq.top(); pq.pop();
        if (u == goal) return gScore[u];

        for (auto [v, w] : graph[u]) {
            int new_g = gScore[u] + w;
            if (new_g < gScore[v]) {
                gScore[v] = new_g;
                // Heuristic: max(|dist(L,u) - dist(L,goal)|)
                int h = 0;
                for (int L : landmarks)
                    h = max(h, abs(landmarkDist[L][u] - landmarkDist[L][goal]));
                pq.push({new_g + h, v});
            }
        }
    }
    return INF;
}

int main() {
    vector<int> landmarks = {0, 3};

    auto preStart = high_resolution_clock::now();
    precomputeLandmarks(landmarks);
    auto preEnd = high_resolution_clock::now();

    auto queryStart = high_resolution_clock::now();
    int dist = ALT(0, 3, landmarks);
    auto queryEnd = high_resolution_clock::now();

    cout << "Preprocessing time: "
         << duration_cast<microseconds>(preEnd - preStart).count() << " μs\n";

    cout << "ALT distance: " << dist << "\nTime to perform ALT query: "
         << duration_cast<microseconds>(queryEnd - queryStart).count() << " μs\n";

    return 0;
}
