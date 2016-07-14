#include "ReebEdge.h"

using namespace argos;

ReebEdge::ReebEdge(CVector2 cellBeginning, int beginNode)
    : beginning(beginNode)
    , end(NO_NODE)
    , cell(cellBeginning)
{}

