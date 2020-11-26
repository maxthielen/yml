//
// Created by rolf on 26-09-20.
//

#include <cmath>
#include <include/roboteam_ai/control/positionControl/BBTrajectories/BBTrajectory2D.h>
#include <include/roboteam_ai/utilities/Constants.h>
namespace rtt::BB {

    BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                                   double maxVel, double maxAcc, double alpha)  {
        generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
    }

    BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                                   double maxVel, double maxAcc)  {
        generateSyncedTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc);
    }
    void BBTrajectory2D::generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
                                            const Vector2 &finalPos,
                                            double maxVel, double maxAcc, double alpha)  {
        x = BBTrajectory1D(initialPos.x, initialVel.x, finalPos.x, maxVel*cos(alpha), maxAcc*cos(alpha));
        y = BBTrajectory1D(initialPos.y, initialVel.y, finalPos.y, maxVel*sin(alpha), maxAcc*sin(alpha));
    }

    void BBTrajectory2D::generateSyncedTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
                                                  const Vector2 &finalPos, double maxVel, double maxAcc)  {
        //The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
        double inc = M_PI_4*0.5;
        double alpha = M_PI_4;
        //TODO: tune convergence numbers
        constexpr double iterationLimit = 1e-7;
        constexpr double timeDiffLimit = 0.001;
        while (inc > iterationLimit) {
            generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
            double diff = abs(x.getTotalTime() - y.getTotalTime());
            //If the trajectories match enough we stop earlier
            if (diff < timeDiffLimit) {
                return;
            }
            if (x.getTotalTime() > y.getTotalTime()) {
                alpha -= inc;
            }
            else {
                alpha += inc;
            }
            inc *= 0.5;
        }
    }

    Vector2 BBTrajectory2D::getPosition(double t) const {
        return Vector2(x.getPosition(t),y.getPosition(t));
    }

    Vector2 BBTrajectory2D::getVelocity(double t) const {
        return Vector2(x.getVelocity(t),y.getVelocity(t));
    }

    Vector2 BBTrajectory2D::getAcceleration(double t) const {
        return Vector2(x.getAcceleration(t),y.getAcceleration(t));
    }

    std::vector<Vector2> BBTrajectory2D::getStraightLines(unsigned int N) const {
        std::vector<Vector2> points;
        double timeStep=fmax(x.getTotalTime(),y.getTotalTime())/N;
        for (int i = 0; i <= N; ++ i) {
            points.push_back(getPosition(timeStep*i));
        }
        return points;
    }

    std::vector<Vector2> BBTrajectory2D::getPathApproach(double timeStep) const {
        std::vector<Vector2> points;
        auto totalTime = fmax(x.getTotalTime(),y.getTotalTime());
        //auto radius = rtt::ai::Constants::ROBOT_RADIUS();
        //auto vMax = rtt::ai::Constants::MAX_VEL();
        //auto aMax = rtt::ai::Constants::MAX_ACC_UPPER();

        //double minTimeStep = 2*radius/vMax; // 2 times the robotradius divided by maximum velocity
        //double maxTimeStep = 2*sqrt(radius/aMax); // sqrt(2* radius / (0.5* maximum acceleration) )
        double time = 0;

        while(time<totalTime){
            //timeStep = std::clamp(2*radius/getVelocity(time).length(),minTimeStep,maxTimeStep);
            time += timeStep;
            points.push_back(getPosition(time));
        }
        return points;
    }

    double BBTrajectory2D::getTotalTime() const {
        return std::max(x.getTotalTime(),y.getTotalTime());
    }

    Vector2 BBTrajectory2D::getBrakePos(Vector2 pos, Vector2 vel, double accMax) const {
        return Vector2(x.fullBrakePos(pos.x,vel.x,accMax),y.fullBrakePos(pos.y,vel.y,accMax));
    }
}