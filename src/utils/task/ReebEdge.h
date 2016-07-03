#pragma once

#include "TaskCell.h"
#include <argos3/core/utility/math/vector2.h>

class ReebEdge {
public:
    ReebEdge(argos::CVector2 cellBeginning, int beginNode);
    int getEnd() const { return end; }
    void setEnd(int endNode) { end = endNode; }
    int getBeginning() const { return beginning; }
    TaskCell& getCell() { return cell; }
    const TaskCell& getCell() const { return cell; }
private:
    int beginning;
    int end;
    TaskCell cell;
};



