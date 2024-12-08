#ifndef LAPTRACKER_H
#define LAPTRACKER_H

#include <GeographicLib/Geodesic.hpp>
#include <chrono>
#include <VehicleDataTypes.hpp>

struct DataPoint {
    double latitude;
    double longitude;
    double orientation;
};

class LapTracker {
public:
    LapTracker();
    LapTracker(int lapNumber);
    void processData(const DataPoint& point);
    bool isRaceOver() const;

private:
    bool started;
    int lapCount;
    int lapNumber;
    double startLineLat1, startLineLon1, startLineLat2, startLineLon2;
    double finishLineLat1, finishLineLon1, finishLineLat2, finishLineLon2;
    double totalDistance;
    std::chrono::steady_clock::time_point startTime;

    DataPoint previousPoint;
    bool hasPreviousPoint;

    void calculateStartLine(const DataPoint& point);
    void calculateFinishLine(const DataPoint& point);
    bool checkLapCompletion(const DataPoint& previousPoint, const DataPoint& currentPoint);
    bool checkFinishLineCrossing(const DataPoint& previousPoint, const DataPoint& currentPoint);
    double calculateDistance(const DataPoint& point1, const DataPoint& point2);
    bool checkLineCrossing(const DataPoint& previousPoint, const DataPoint& currentPoint, double lat1, double lon1, double lat2, double lon2);
    DataPoint process_race_data(const core::VehicleState &current_state);
};

#endif
