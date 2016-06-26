#pragma once

#include <argos3/core/utility/math/range.h>
#include "TaskHandler.h"

class TaskCell {
public:
    TaskCell(std::array<argos::CVector2, 2> corners);
    void update();
    void addLeftExplorer(TaskHandler& e);
    void addRightExplorer(TaskHandler& e);
    bool isReady() const;
    const auto getLimits() const { return limits; }
private:
    enum Explorer { Left = 0, Right = 1 };

    argos::CRange<argos::CVector2> limits;
    std::array<TaskHandler*, 2> explorers;
    std::vector<std::reference_wrapper<TaskHandler>> sweepers;
    bool finished = false;

    bool isNear(TaskHandler& handler, const argos::CVector2& point) const;
    bool isExplorerNearBeginning(Explorer index) const;
    bool areExplorersAtBeginning() const;
    void proceedExplorer(Explorer index);
    void proceedExplorers();
    void moveExplorerToBegin(Explorer index);
    void moveExplorersToBegin();
    void stopExplorer(Explorer index);

    void updateCellLimits(TaskHandler* explorer);
};
