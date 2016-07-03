#include "ReebEdge.h"

using namespace argos;

ReebEdge::ReebEdge(CVector2 cellBeginning, int beginNode)
    : beginning(beginNode)
    , end(-1)
    , cell(cellBeginning)
{}

