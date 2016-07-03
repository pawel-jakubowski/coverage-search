#pragma once

#include "ReebEdge.h"
#include <functional>
#include <list>

class ReebGraph {
public:
    ReebGraph() = default;
    int addNode();
    std::size_t addEdge(argos::CVector2 cellBeginning, int startNode);

    std::size_t getNodesSize() const { return nodes.size(); }
    std::size_t getEdgesSize() const { return edges.size(); }
    std::size_t getCellsSize() const { return getEdgesSize(); }

    ReebEdge& getEdge(std::size_t id) { return edges.at(id); }
    const ReebEdge& getEdge(std::size_t id) const { return edges.at(id); }
    TaskCell& getCell(std::size_t id) { return edges.at(id).getCell(); }

    std::vector<int>& getNodes() { return nodes; }
    const std::vector<int>& getNodes() const { return nodes; }
    std::vector<ReebEdge>& getEdges() { return edges; }
    const std::vector<ReebEdge>& getEdges() const { return edges; }
    std::list<std::reference_wrapper<TaskCell>> getCells();
    std::list<std::reference_wrapper<const TaskCell>> getCells() const;

    std::list<std::size_t> getEdgesIdsFromNode(int node) const;
private:
    std::vector<ReebEdge> edges;
    std::vector<int> nodes;
};

std::ostream& operator<<(std::ostream& o, const ReebGraph& graph);


