#include "Mbfo.h"

#include <assert.h>

using namespace std;

namespace argos {

/* Footbot dimensions */
static const Real INTERWHEEL_DISTANCE        = 0.14f;
static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
static const Real HALF_INTERWHEEL_DISTANCE_IN_CM = HALF_INTERWHEEL_DISTANCE * 100;

/* Simulator parameters */
static const Real TICKS_PER_SEC = 10;
static const unsigned long CHEMOTAXIS_LENGTH = 10;

Mbfo::Mbfo()
    : wheelsEngine(nullptr)
    , proximitySensor(nullptr)
    , stopped(false)
    , velocity(5.0f)
    , rotationSpeed(0)
    , minDistanceFromObstacle(0.1f)
    , minAngleFromObstacle(CDegrees(-45.0f), CDegrees(45.0f))
    , loopFnc(dynamic_cast<MbfoLoopFunction&>(CSimulator::GetInstance().GetLoopFunctions()))
    , step(0)
{}

void Mbfo::Init(TConfigurationNode& configuration) {
    currentCellId = GetId();
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    lightSensor = GetSensor<CCI_LightSensor>("light");
    rabRx = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

    GetNodeAttributeOrDefault(configuration, "velocity", velocity, velocity);
    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
            minDistanceFromObstacle);

    coverage = &loopFnc.getCoverageGrid();

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
    assert(positioningSensor != nullptr);
    assert(lightSensor != nullptr);
    assert(rabRx != nullptr);
    assert(coverage != nullptr);
}

void Mbfo::Reset() {
    stopped = false;
}

void Mbfo::ControlStep() {
    if (!stopped) {
        CDegrees robotsOrientation = getOrientationOnXY();
        if (step % CHEMOTAXIS_LENGTH == 0)
            determineNewDirection();
        if ((robotsOrientation - desiredDirection).GetAbsoluteValue() > angleEpsilon)
            tumble(robotsOrientation);
        else
            swim();
    }
    else {
        wheelsEngine->SetLinearVelocity(0,0);
    }
    detectTarget();
    step++;
}

void Mbfo::detectTarget() const {
    /*auto lightReadings = lightSensor->GetReadings();
    double averageReading = 0;
    for (auto& reading : lightReadings) {
        averageReading += reading;
    }
    averageReading /= lightReadings.size();*/

    auto packets = rabRx->GetReadings();
    for(auto& packet : packets) {
        int id;
        Real posX, posY, posZ;
        packet.Data >> id >> posX >> posY >> posZ;
        if (id == 0) continue; // Skip default message send by mbfo robots
//        LOG << "[" << GetId() << "] Found target " << id << " ("
//        << posX << ", " << posY << ", " << posZ << ")" << endl;
        loopFnc.addTargetPosition(id, CVector3(posX, posY, posZ));
    }
}

CDegrees Mbfo::getOrientationOnXY() {
    CRadians angleX, angleY, angleZ;
    positioningSensor->GetReading().Orientation.ToEulerAngles(angleX, angleY, angleZ);
    CDegrees robotsOrientation = ToDegrees(angleX);
    return robotsOrientation;
}

void Mbfo::determineNewDirection() {
    const auto& cell = getVoronoiCell(currentCellId);

    if (isCellDone(cell)) {
        auto otherCells = loopFnc.getNeighbouringVoronoiCells(GetId());
        const auto robotsPosition = positioningSensor->GetReading().Position;
        auto closestCellComparator = [&, robotsPosition](const VoronoiCell* a, const VoronoiCell* b)
            { return calculateDistance(a->seed.position, robotsPosition) < calculateDistance(b->seed.position,
                                                                                             robotsPosition); };
        std::sort(otherCells.begin(), otherCells.end(), closestCellComparator);

        const VoronoiCell* nextCellPtr = nullptr;
        for(auto cellPtr : otherCells)
            if (!isCellDone(*cellPtr)) {
                nextCellPtr = cellPtr;
                break;
            }

        if (nextCellPtr != nullptr) {
            currentCellId = nextCellPtr->seed.id;
//            LOG << "[" << GetId() << "] Go now to cell " << currentCellId << endl;
        }
        else {
            if (!stopped)
                LOG << "[" << GetId() << "] No cells to cover! Stop" << endl;
            stopped = true;
        }
    }

    if (!stopped) {
        auto nextBestDirections = findNextBestDirectionsInVoronoiCell(cell);

        // Determine new direction
        if (nextBestDirections.size() != 0)
            chooseBestDirectionFromVector(nextBestDirections);

        CVector3 robotsInteractionForce = calculateRobotsInteractionForce();
        CDegrees robotsInteractionAngle = ToDegrees(robotsInteractionForce.GetZAngle());

        const auto a = 0.025;
        const auto b = (1 - a);
        desiredDirection *= b;
        desiredDirection += a*robotsInteractionAngle;
        desiredDirection.SignedNormalize();
    }
}

CVector3 Mbfo::calculateRobotsInteractionForce() const {
    const auto robotsPosition = positioningSensor->GetReading().Position;
    auto robotsPositions = loopFnc.getRobotsPositions();
    CVector3 robotsInteractionForce;
    for (auto& position : robotsPositions) {
            if (position.first == GetId())
                continue;
            auto diff = (robotsPosition - position.second);
            robotsInteractionForce += diff / diff.SquareLength();
        }
    return robotsInteractionForce;
}

bool Mbfo::isCellDone(const VoronoiDiagram::Cell& cell) const {
    int maxCellConcentration = 0;
    for (auto cellIndex : cell.coverageCells) {
        int concentration = coverage->getGrid().at(cellIndex.x).at(cellIndex.y).concentration;
        if (concentration > maxCellConcentration)
            maxCellConcentration = concentration;
    }
    return maxCellConcentration == 0;
}

void Mbfo::chooseBestDirectionFromVector(vector<Mbfo::NextDirection>& nextBestDirections) {
    auto angleComparator = [&](NextDirection d) {
        return (d.angle - this->desiredDirection).GetAbsoluteValue() < this->angleEpsilon;
    };
    auto sameDirection = std::find_if(nextBestDirections.begin(), nextBestDirections.end(),
                                      angleComparator);
    if (sameDirection == nextBestDirections.end()) {
        auto randomIndex = rand() % nextBestDirections.size();
        this->desiredDirection = nextBestDirections.at(randomIndex).angle;
    }
}

void Mbfo::tumble(const CDegrees &robotsOrientation) {
    auto angleDiff = (robotsOrientation - desiredDirection).SignedNormalize();
    auto rotationDirection = getRotationDirection(angleDiff);
    rotationSpeed = ToRadians(angleDiff).GetAbsoluteValue()
        * HALF_INTERWHEEL_DISTANCE_IN_CM
        * TICKS_PER_SEC;
    rotate(rotationDirection);
}

void Mbfo::swim() const {
    wheelsEngine->SetLinearVelocity(this->velocity, this->velocity);
}

CDegrees Mbfo::getAngleBetweenPoints(const CVector3 &a, const CVector3 &b) const {
    return ToDegrees(ATan2(b.GetY() - a.GetY(), b.GetX() - a.GetX()));
}

std::vector<Mbfo::NextDirection> Mbfo::findNextBestDirectionsInVoronoiCell(const VoronoiDiagram::Cell &cell) const {
    const CVector3 realPosition = positioningSensor->GetReading().Position;
    auto index = loopFnc.getCoverageGrid().getCellIndex(realPosition);
    const CVector2 positionCellIndex(index.first, index.second);

    // Complete search
    std::vector<Mbfo::NextDirection> nextDirections;
    Real bestValue = 0;
    for (auto& i : cell.coverageCells) {
        CVector2 v(i.x, i.y);
        auto& gridCell = getCoverageCell(v);

        // Take only cells that are better or same as current bestValue
        if (bestValue <= gridCell.concentration) {
            bestValue = gridCell.concentration;
            Real distance = calculateDistance(v, positionCellIndex);
            CDegrees angle = getAngleBetweenPoints(realPosition, gridCell.center);
            nextDirections.push_back(NextDirection{gridCell.concentration, distance, angle, v});
        }
    }

    // remove elements lower than max
    auto elementsLowerThanBestValue = std::remove_if(nextDirections.begin(), nextDirections.end(),
        [bestValue](NextDirection d){ return d.value < bestValue; });
    nextDirections.erase(elementsLowerThanBestValue, nextDirections.end());

    // From vector with best values find closest distance
    Real closestDistance = std::numeric_limits<Real>::max();
    for (auto& d : nextDirections)
        closestDistance = (closestDistance > d.distance) ? d.distance : closestDistance;

    // remove elements with greater distance than closest one
    auto elementsWithSameDistance = std::remove_if(nextDirections.begin(), nextDirections.end(),
        [closestDistance](NextDirection d){ return d.distance > closestDistance; });
    nextDirections.erase(elementsWithSameDistance, nextDirections.end());

    return nextDirections;
}

const CoverageGrid::Cell& Mbfo::getCoverageCell(CVector2 index) const {
    assert(coverage != nullptr);
    return coverage->getGrid().at(index.GetX()).at(index.GetY());
}

const VoronoiDiagram::Cell& Mbfo::getVoronoiCell(string cellId) {
    const auto cell = loopFnc.getVoronoiCell(cellId);
    assert(cell != nullptr);
    return *cell;
}

CVector2 Mbfo::getWeightedProximityReading() {
    const CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();
    CVector2 accumulator;
    for (size_t i = 0; i < proximityReadings.size(); ++i) {
        accumulator += CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
    }
    accumulator /= proximityReadings.size();
    return accumulator;
}

bool Mbfo::isRoadClear(const CVector2& obstacleProximity) {
    CDegrees obstacleAngle(ToDegrees(obstacleProximity.Angle()));
    CDegrees safeAngle(150.0f);
    return (minAngleFromObstacle.WithinMinBoundIncludedMaxBoundIncluded(obstacleAngle)
            && obstacleProximity.Length() < minDistanceFromObstacle) ||
            (obstacleAngle < -safeAngle || obstacleAngle > safeAngle);
}

void Mbfo::avoidObstacle(const CVector2& obstacleProximity) {
    auto obstacleAngle = ToDegrees(obstacleProximity.Angle());
    auto rotationDirection = getRotationDirection(obstacleAngle);
    rotate(rotationDirection);
}

Mbfo::Direction Mbfo::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::right : Direction::left;
}

void Mbfo::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::right)
        wheelsEngine->SetLinearVelocity(rotationSpeed, -rotationSpeed);
    else if (rotationDirection == Direction::left)
        wheelsEngine->SetLinearVelocity(-rotationSpeed, rotationSpeed);
}


}

REGISTER_CONTROLLER(Mbfo, "mbfo_controller")
