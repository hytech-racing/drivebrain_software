#include <LapTracker.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>

using namespace GeographicLib;

LapTracker::LapTracker(int lapNumber) : started(false), lapCount(0), totalDistance(0.0), hasPreviousPoint(false), lapNumber(lapNumber) {}

int lapCountTarget = 0;

DataPoint LapTracker::process_race_data(const core::VehicleState& current_state) {
    if (lapCountTarget == -1) return {};

    DataPoint point = {current_state.current_gps_position.latitude, 
                       current_state.current_gps_position.longitude, 
                       current_state.current_ypr_rad.yaw};

    processData(point);

    if (lapCount < lapCountTarget) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        DataPoint returned = {elapsed, 
                              totalDistance, 
                              lapCount};

        return returned;
    } 
}


void LapTracker::calculateStartLine(const DataPoint& point) {
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());

    double azi = point.orientation;
    double length = 2.5;

    geod.Direct(point.latitude, point.longitude, azi + 90, length, startLineLat1, startLineLon1);
    geod.Direct(point.latitude, point.longitude, azi - 90, length, startLineLat2, startLineLon2);

    started = true;
    startTime = std::chrono::steady_clock::now();
}

void LapTracker::calculateFinishLine(const DataPoint& point) {
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());

    double azi = point.orientation; 
    double forwardLength = 75.0;
    double lineWidth = 2.5;    

    double finishCenterLat, finishCenterLon;
    geod.Direct(point.latitude, point.longitude, azi, forwardLength, finishCenterLat, finishCenterLon);

    geod.Direct(finishCenterLat, finishCenterLon, azi + 90, lineWidth, finishLineLat1, finishLineLon1);
    geod.Direct(finishCenterLat, finishCenterLon, azi - 90, lineWidth, finishLineLat2, finishLineLon2);
}

void LapTracker::processData(const DataPoint& point) {
    static std::vector<std::string> lapCompletionMessages;
    static bool firstLap = true;

    if (!started) {
        if (lapCountTarget > 0) {
            calculateStartLine(point);
            std::cout << "Race started with lap target: " << lapCountTarget << std::endl;
        } else {
            calculateFinishLine(point); 
            std::cout << "Acceleration event started!" << std::endl;
        }
        started = true;
    } else {
        if (hasPreviousPoint) {
            totalDistance += calculateDistance(previousPoint, point);

            if (lapCountTarget > 0 && checkLapCompletion(previousPoint, point)) {
                lapCount++;
            }

            if (lapCountTarget == 0 && checkFinishLineCrossing(previousPoint, point)) {
                lapCount++;
            }
        }

        previousPoint = point;
        hasPreviousPoint = true;
    }
}

double LapTracker::calculateDistance(const DataPoint& point1, const DataPoint& point2) {
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    double s12;
    geod.Inverse(point1.latitude, point1.longitude, point2.latitude, point2.longitude, s12);
    return s12;
}

bool LapTracker::checkLapCompletion(const DataPoint& previousPoint, const DataPoint& currentPoint) {
    return checkLineCrossing(previousPoint, currentPoint, startLineLat1, startLineLon1, startLineLat2, startLineLon2);
}

bool LapTracker::checkFinishLineCrossing(const DataPoint& previousPoint, const DataPoint& currentPoint) {
    return checkLineCrossing(previousPoint, currentPoint, finishLineLat1, finishLineLon1, finishLineLat2, finishLineLon2);
}

bool LapTracker::checkLineCrossing(const DataPoint& previousPoint, const DataPoint& currentPoint, double lat1, double lon1, double lat2, double lon2) {
    auto det = [](double x1, double y1, double x2, double y2) {
        return x1 * y2 - y1 * x2;
    };

    double x1 = lon1, y1 = lat1;
    double x2 = lon2, y2 = lat2;
    double x3 = previousPoint.longitude, y3 = previousPoint.latitude;
    double x4 = currentPoint.longitude, y4 = currentPoint.latitude;

    double d1 = det(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
    double d2 = det(x2 - x1, y2 - y1, x4 - x1, y4 - y1);
    double d3 = det(x4 - x3, y4 - y3, x1 - x3, y1 - y3);
    double d4 = det(x4 - x3, y4 - y3, x2 - x3, y2 - y3);

    return (d1 * d2 < 0) && (d3 * d4 < 0);
}
