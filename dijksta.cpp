#include <iostream>
#include<list>
using namespace std;
#define INF 10000000;
class Graph {
private:
	int V;
	list<pair<int, float>>* adj;
public:
// Constructor
	Graph(int v);
	void addEdge(int v1, int v2, float weight);
	void shortestPath(int s);
};

Graph::Graph(int v) {
	// Num of Vertices
	V = v;
	
	// Adjacency List declaration
	adj = new list<pair<int, float>>[v];
}

void Graph::addEdge(int v1, int v2, float weight) {
	adj[v1].push_back(make_pair(v2, weight));
}

void Graph::shortestPath(int s) {
	// Declaration of set to store vertices
	set<pair<int, int>> extract_set;

	// Vector for distances 
	vector<float> distances(V, INF);

	// Entry for set with dist = 0
	extract_set.insert(make__pair(0, s));
	distances[s] = 0;

	while (!extract_set.empty()) {
		// Extract min distance
		pair<int, int> tmp = *(extract_set.begin());
		extract_set.erase(extract_set.begin());

		// Retrieve vertex index
		int u = tmp.second;

		// Loop adjacency list
		for (auto i = adj[u].begin(); i != adj[u].end(); i++) {
			// Vertex and weight
			int v = (*i).first;
			float weight = (*i).second;

			// Check for smaller dist
			if (distances[v] > distances[u] + weight) {
				// Remove current dist
				if (distances[v] != INF) {
					extract_set.erase(extract_set.find(make_pair(distances[v], v)));
				}

				// Update dist
				distances[v] = distances[u] + weight;
				extract_set.insert(make_pair(distances[v], v));
			}
		}
	}
	// Output

}

int main() {
	Graph mesh(10);
	mesh.addEdge(0, 2, 3.21);
}