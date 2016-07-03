#include "ReebGraph.h"

using namespace std;
using namespace argos;

int ReebGraph::addNode() {
    int node = nodes.size();
    nodes.push_back(node);
    return node;
}
size_t ReebGraph::addEdge(CVector2 cellBeginning, int startNode) {
    edges.emplace_back(cellBeginning, startNode);
    return edges.size()-1;
}

list<reference_wrapper<TaskCell>> ReebGraph::getCells() {
    list<reference_wrapper<TaskCell>> cells;
    for (auto& edge : edges)
        cells.push_back(edge.getCell());
    return cells;
}

list<reference_wrapper<const TaskCell>> ReebGraph::getCells() const {
    list<reference_wrapper<const TaskCell>> cells;
    for (auto& edge : edges)
        cells.push_back(edge.getCell());
    return cells;
}

list<size_t> ReebGraph::getEdgesIdsFromNode(int node) const {
    list<size_t> edgesFromNode;
    for (size_t i = 0; i < edges.size(); i++)
        if (edges.at(i).getBeginning() == node)
            edgesFromNode.push_back(i);
    return edgesFromNode;
}

ostream& operator<<(ostream& o, const ReebGraph& graph) {
    o << graph.getNodesSize() << " nodes, "
        << graph.getEdgesSize() << " edges\n";
    const auto& edges = graph.getEdges();
    for (size_t i = 0; i < edges.size(); i++)
        o << "(" << edges.at(i).getBeginning() << ")--"
            << i << "--[" << edges.at(i).getEnd() << "]\n";
}
