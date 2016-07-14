#pragma once

#include <argos3/core/utility/math/range.h>
#include <list>
#include "TaskHandler.h"

class TaskCell {
public:
    TaskCell(argos::CVector2 beginning);
    void update();
    void finish(argos::CVector2 end);
    void addLeftExplorer(TaskHandler& e);
    void addRightExplorer(TaskHandler& e);
    bool isReady() const;
    bool isFinished() const;
    bool isForwardConvex() const;
    bool isReverseConvex() const;

    std::list<Task> getExplorersTasks() const;
    const argos::CVector2& getBeginning() const { return beginning; }
    const argos::CVector2& getEnd() const { return end; }
    const argos::CRange<argos::CVector2>& getLimits() const { return limits; }

private:
    enum Explorer { Left = 0, Right = 1 };

    argos::CVector2 beginning;
    argos::CVector2 end;
    argos::CRange<argos::CVector2> limits;
    std::array<TaskHandler*, 2> explorers;
    std::vector<std::reference_wrapper<TaskHandler>> sweepers;
    bool started = false;
    bool prepared = false;
    bool finished = false;
    bool forwardConvexCP = false;
    bool reverseConvexCP = false;

    bool isNear(TaskHandler& handler, const argos::CVector2& point) const;
    bool isExplorerNearBeginning(Explorer index) const;
    bool areExplorersAtBeginning() const;
    bool areExplorersReadyToProceed() const;
    void updateExplorers(Task::Status status);
    void updateExplorerStatus(Explorer index, Task::Status status);
    void setRevertTaskForExplorers();
    void setRevertTaskForExplorer(Explorer index);
    void freeExplorers();
    void freeExplorer(Explorer index);

    void updateCellLimits();
    void finishCell();

    void moveExplorersToBeginning();
    void prepareExplorers();
    void proceedExplorers();
};
