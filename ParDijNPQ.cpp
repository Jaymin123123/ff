#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <utility>
#include <map>
#include <chrono>
#include <omp.h>

typedef std::pair<int, int> Edge; // Edge(destination, weight)
typedef std::vector<std::vector<Edge>> Graph; // Graph as an adjacency list

Graph readGraph(const std::string& filename, std::map<std::string, int>& nodeMap) {
    std::ifstream file(filename);
    Graph graph;
    std::string line;
    int nodeIndex = 0;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string node1, node2, weight;

        if (!getline(ss, node1, ',') || !getline(ss, node2, ',') || !getline(ss, weight)) {
            std::cerr << "Warning: Malformed line skipped: " << line << std::endl;
            continue;
        }

        if (nodeMap.find(node1) == nodeMap.end()) nodeMap[node1] = nodeIndex++;
        if (nodeMap.find(node2) == nodeMap.end()) nodeMap[node2] = nodeIndex++;
        int n1 = nodeMap[node1];
        int n2 = nodeMap[node2];
        int w = std::stoi(weight);

        if (graph.size() <= std::max(n1, n2)) {
            graph.resize(std::max(n1, n2) + 1);
        }

        graph[n1].push_back({n2, w});
    }
    return graph;
}

void dijkstra(const Graph& graph, int src) {
    int V = graph.size();
    std::vector<int> dist(V, std::numeric_limits<int>::max());
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        // Parallel region to update distances
        #pragma omp parallel for
        for (const auto& neighbor : graph[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;
            int new_dist = dist[u] + weight;

            // Protect access to shared resources
            #pragma omp critical
            {
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    // Direct priority queue access should be guarded by a critical section
                    pq.push({new_dist, v});
                }
            }
        }
    }

    // Output the shortest distances
    std::cout << "Vertex\tDistance from Source\n";
    for (int i = 0; i < V; i++) {
        std::cout << i << "\t" << dist[i] << "\n";
    }
}

int main() {
    std::string filename = "fully_connected_graphw1000.csv";
    std::map<std::string, int> nodeMap;
    Graph graph = readGraph(filename, nodeMap);
    int source = 0;

    auto start = std::chrono::high_resolution_clock::now();
    dijkstra(graph, source);
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = end - start;  
    std::cout << "Execution time: " << elapsed.count() << " seconds.\n";

    return 0;
}
