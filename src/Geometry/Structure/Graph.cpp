#include "Graph.h"
#include "Geometry/Structure/KDTree.h"
#include "IO/PLYManager.h"
#include <algorithm>
#include <queue>
#include <set>

namespace dragon
{
namespace geometry
{

    void Graph::ConstructEdgeThreshold(double threshold)
    {
        for (size_t i = 0; i < vertices.size(); i++) 
            for (size_t j = i + 1; j < vertices.size(); j++) 
                if (geometry::Distance(vertices[i], vertices[j]) <= threshold) 
                {
                    MakeNeighbor(i, j);
                    MakeNeighbor(j, i);
                }
    }
    void Graph::ConstructEdgeRadius(const std::vector<double> & radius)
    {
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(vertices);
        std::vector<size_t> indices;
        std::vector<float> dists;
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            kdtree.RadiusSearch(vertices[i], indices, dists, 9 * radius[i] * radius[i]);
            
            if(indices.size() <= 1)
            {
                std::cout<<YELLOW<<"[WARNING]::[GRAPH]::Single vertex in graph."<<RESET<<std::endl;
                continue;
            }
            // std::cout<<radius[i]<<" "<<dists.back()<<std::endl;
            geometry::Vec3List directions;
            for (size_t j = 1; j < indices.size(); ++j)
            {

                geometry::Vector3 tmp_d = (vertices[indices[j]] -  vertices[i]).normalized();
                bool new_direction = true;
                for(auto &d: directions)
                {
                    if(tmp_d.dot(d) > 0)
                    {
                        new_direction = false;
                        break;
                    }
                }
                // std::cout<<dists[j]<<std::endl;
                if(new_direction)
                {
                    MakeNeighbor(i, indices[j]);
                    MakeNeighbor(indices[j], i);
                    directions.push_back(tmp_d);
                }
                // else break;
            }
            // std::cout<<"-----------------------------"<<std::endl;
        }
    }
    void Graph::ConstructEdgeKNN(int k)
    {
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(vertices);
        std::vector<size_t> indices;
        std::vector<float> dists;
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            kdtree.KnnSearch(vertices[i], indices, dists, k);
            for (size_t j = 1; j < indices.size(); ++j)
            {
                MakeNeighbor(i, indices[j]);
                MakeNeighbor(indices[j], i);
            }
        }
    }
    void Graph::ConstructEdgeAdaptive(double factor, int max_k, double min_degree)
    {
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(vertices);
        std::vector<size_t> indices;
        std::vector<float> dists;
        double cos_degree = cos(min_degree / 180.0 * M_PI);
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            kdtree.KnnSearch(vertices[i], indices, dists, max_k);
            if(indices.size() <= 1)
            {
                std::cout<<YELLOW<<"[WARNING]::[GRAPH]::Single vertex in graph."<<RESET<<std::endl;
                continue;
            }
            MakeNeighbor(i, indices[1]);
            MakeNeighbor(indices[1], i);
            geometry::Vec3List directions;
            directions.push_back((vertices[indices[1]] -  vertices[i]).normalized());
            for (size_t j = 2; j < indices.size(); ++j)
            {
                if(dists[j] / dists[1] > factor * factor)
                break;
                geometry::Vector3 tmp_d = (vertices[indices[j]] -  vertices[i]).normalized();
                bool new_direction = true;
                for(auto &d: directions)
                {
                    if(tmp_d.dot(d) > cos_degree)
                    {
                        new_direction = false;
                        break;
                    }
                }
                // std::cout<<dists[j]<<std::endl;
                if(new_direction)
                {
                    MakeNeighbor(i, indices[j]);
                    MakeNeighbor(indices[j], i);
                    directions.push_back(tmp_d);
                }
                // else break;
            }
            // std::cout<<"-----------------------------"<<std::endl;
        }
    }
    void Graph::MakeNeighbor(size_t u, size_t v) 
    {
        if(std::find(neighbors[u].begin(), neighbors[u].end(), v) == neighbors[u].end())
        {
            neighbors[u].push_back(v);
        }
    }
    void Graph::ComputeEdges()
    {
        edges.clear();
        for(size_t i = 0; i != neighbors.size(); ++i)
        {
            for(size_t j = 0; j != neighbors[i].size(); ++j )
                if(neighbors[i][j] > i) edges.push_back(Edge(i, neighbors[i][j]));
        }
    }
    void Graph::ComputeNeighbors()
    {
        neighbors.clear();
        neighbors = std::vector<std::vector<size_t>>(vertices.size(), std::vector<size_t>());
        for(size_t i = 0; i != edges.size(); ++i)
        {
            neighbors[edges[i].start].push_back(edges[i].end);
            neighbors[edges[i].end].push_back(edges[i].start);
        }
    }
    bool Graph::WriteToPLY(const std::string &filename) const
    {
        // through neighbors
        return io::WritePLY(filename,vertices, colors, neighbors);
    }
    // generate min spanning tree
    // if the graph is a connected graph, then there will be only one spanning tree
    // some parts of this function were generated by ChatGPT
    std::vector<std::vector<Edge>> Graph::GenerateMinSpanningTrees()
    {
        // use prim algorithm
        if(!HasEdges()) ComputeEdges();
        // compute edge weight
        for(size_t i = 0; i != edges.size(); ++i)
        edges[i].weight = geometry::Distance(vertices[edges[i].start], vertices[edges[i].end]);
        // std::cout<<"number of edges: "<<edges.size()<<std::endl;
        int n = vertices.size();
        std::vector<std::vector<Edge>> msts; 
        std::vector<bool> visited(n, false);
        std::priority_queue<Edge, std::vector<Edge>, CompareEdge> min_heap; //
        size_t start_id = 0;
        bool all_visited;
        do
        {
            // std::cout<<"start id: "<<start_id<<std::endl;
            msts.push_back(std::vector<Edge>());
            auto & mst = msts.back();
            visited[start_id] = true;

            for (const Edge& edge : edges) {
                if (edge.start == start_id || edge.end == start_id) min_heap.push(edge);
            }
            while (!min_heap.empty()) 
            {
                Edge current_edge = min_heap.top();
                min_heap.pop();

                size_t next_vertex = current_edge.end;
                if (visited[next_vertex]) next_vertex = current_edge.start;
                if (visited[next_vertex]) continue;

                mst.push_back(current_edge);
                visited[next_vertex] = true;

                for (const Edge& edge : edges) 
                {
                    if (!visited[edge.end] && edge.start == next_vertex) min_heap.push(edge);
                    if (!visited[edge.start] && edge.end == next_vertex) min_heap.push(edge);
                }
            }
            all_visited = true;
            for(size_t i = 0; i != visited.size(); ++i)
                if(!visited[i])
                {
                    start_id = i;
                    all_visited = false;
                }
        }while(!all_visited);

        return msts;    
    }
}
}