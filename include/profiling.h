#ifndef _PROFILINGH_
#define _PROFILINGH_

#include <vector>

struct Point {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double heading;
};

struct UltraPose {
    double x;
    double y;
    double heading;
    double curvature;
};

struct Line {
    double slope;
    double yIntercept;
};

struct MPPoint {
    double x;
    double y;
    double heading;
    double linVel;
    double angVel;
    double t;
    double timeAtPoint;
};

struct Zone {
    double startT; // start value of t
    double endT; // end value of t
    Line zoneLine; // line that represents the zone on the graph
};

struct HexicPolyData {
    double P6;
    double P5;
    double P4;
    double P3;
    double P2;
    double P1;
    double P0;
};

struct CubicPolyData {
    double a;
    double b;
    double c;
    double d;
};

struct QuadraticPolyData {
    double a;
    double b;
    double c;
};

enum Direction {
    LEFT = -1,
    STRAIGHT = 0,
    RIGHT = 1,
};

class CubicHermiteSpline {
    public:
        CubicHermiteSpline(Point startPos, Point startV, Point endPos, Point endV);
        Point findPoint(double t);
        Pose findPose(double t, double step);
        UltraPose findUltraPose(double t, double step);
        std::vector<UltraPose> entirePath(double numPoints);
        void findFunction(void);
        void findDerivative(void);
        void findSecondDerivative(void);
        double calculateCurvature(double t);
        double findNearestPointOnSpline(Point givenPoint, double excludeBelow);
        double calculateCurveSpeed(double t);
        double findNextT(double currentT, double distanceToMove);

        std::vector<CubicPolyData> functions;
        std::vector<QuadraticPolyData> derivative;
        std::vector<HexicPolyData> secondDerivative;
        std::vector<UltraPose> fullSampleSpline;


    private:
        double evaluateP0(double t);
        double evaluateV0(double t);
        double evaluateP1(double t);
        double evaluateV1(double t);

        Point startPos;
        Point startV;
        Point endPos;
        Point endV;
};

class MotionProfile {
    public:
        // constructors (one with custom zoning, one without)
        MotionProfile(CubicHermiteSpline* path, double maxSpeed, std::vector<std::vector<Point>> zonePoints = {});
        MotionProfile(std::vector<MPPoint>* pregeneratedProfile, double maxSpeed);

        // instance variables (data about profile)
        std::vector<MPPoint> profile;
        std::vector<Zone> zones;
        double maxSpeed;

        // public methods (operations on points)
        MPPoint findNearestPoint(double givenT);

    private:
        // private methods (profile generation)
        void generateVelocities(void);
        void constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints);

        // private instance variables (used in profile generation)
        CubicHermiteSpline* path;


};

class VelocityController {
    public:
        double linVel;
        double angVel;
        std::vector<double> FINDME;
        VelocityController(std::vector<std::function<void(void)>> actions = {}, std::vector<double> actionTs = {});
        void addAction(std::function<void(void)> action, double time);
        void clearActions(void);
        void startProfile(MotionProfile* profile, bool reverse = false, bool RAMSETE = true);


    private:
        std::vector<double> calculateOutputOfSides(double linearVelocityMPS, double angularVelocityRADPS, double profileMaxIPS);
        double calculateSingleDegree(double wheelDiameter);
        void followProfile(MotionProfile* profile, bool reverse, bool RAMSETE);
        
        double timeToRun;
        std::vector<double> actionTs;
        std::vector<std::function<void(void)>> actions;
        std::vector<bool> actionCompleteds;
};



#endif