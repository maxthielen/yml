
#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <QColor>
#include <ros/node_handle.h>
#include "math.h"


namespace rtt {
namespace ai {

class Constants {
public:
    static void init();
    static bool GRSIM();

// Show timing for different functions
    static bool SHOW_LONGEST_TICK()             { return true;};
    static bool SHOW_TICK_TIME_TAKEN()          { return true;};
    static bool SHOW_NUMTREE_TIME_TAKEN()       { return false;};

// Show debug information for different functions
    static bool SHOW_NUMTREE_DEBUG_INFO()       { return false; };

// Max values we can send through robothub
    static double MAX_VEL_CMD()                 { return 8.191; };
    static int GENEVA_LEFT()                    { return 0; } ;     //TODO: Might be reversed, please check
    static int GENEVA_RIGHT()                   { return 5; };
    static int MAX_ID_CMD()                     { return 15; };
    static double MAX_ANGULAR_VEL_CMD()         { return 16*M_PI; };
    static double MIN_ANGLE()                   { return -M_PI; };
    static double MAX_ANGLE()                   { return M_PI; };

// Limits as defined in AI itself
    static double MAX_VEL()                     { return 8.0; };
    static double MAX_VEL_BALLPLACEMENT()       { return 3.0; };

// Other/multiple usage
    static int DEFAULT_ROBOT_ID()               { return 1; };

    static double MAX_ANGULAR_VELOCITY()        { return 6.0; };    // Rad per second
    static double ROBOT_RADIUS()                { return 0.089;  }; // TODO: Need to test if world_state agrees with this definition of the centre of the robot
    static double ROBOT_RADIUS_MAX()            { return 0.091; };
    static double FRONT_LENGTH()                { return 0.118; };  // Length of the front (flat) part of the robot
    static double DRIBBLER_ANGLE_OFFSET()       { return asin(FRONT_LENGTH()/2/ROBOT_RADIUS()); };  // If the angle 0 is the centre of the robot, then -DRIBBLER_ANGLE_OFFSET() points to the left and DRIBBLER_ANGLE_OFFSET() to the right.

  static double CENTRE_TO_FRONT()             { return sin(DRIBBLER_ANGLE_OFFSET())*ROBOT_RADIUS(); };
    static double BALL_RADIUS()                 { return 0.0215; };
    static int TICK_RATE()                       { return 60 ; };// Rate at which we tick our behavior Trees

    //skills
    static double DEFAULT_KICK_POWER()          { return  5.0; }; // max kick power() { return  100
    static double MAX_KICK_POWER()              { return  8.0; }; //TODO: CHECK

    static double MAX_POWER_KICK_DISTANCE()     { return 9.0; };
    static int MAX_KICK_CYCLES()                { return 20; };
    static int MAX_GENEVA_CYCLES()              { return 20; };
    static int DEFAULT_GENEVA_STATE()           { return 0; };


// Dribble
    static double MAX_BALL_BOUNCE_RANGE()       { return 0.15; };
    static double DRIBBLE_POSDIF()              { return 0.05; };
    static float  DRIBBLE_SPEED()               { return 0.8; };


// HasBall
    static double MAX_BALL_RANGE()              { return 0.15; };   // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
    static double HAS_BALL_ANGLE()              { return 0.2; }

// GetBall
    static double COLLISION_RADIUS()            { return 0.18; };
    static double ANGLE_SENS()                  { return 0.05*M_PI; };
    static double MAX_GETBALL_RANGE()           { return 0.7; };
    static int POSSES_BALL_CYCLES()             { return 25; };
    static double GETBALL_SPEED()               { return 0.5; };
    static double GETBALL_OVERSHOOT()           { return 0.02; };    // Meters

// PositionControl
    static double MAX_CALCULATION_TIME()        { return 5.0; };   // Max time in ms
    static double NUMTREE_ERROR_MARGIN()        { return 0.25; };
    static double MIN_DISTANCE_FOR_FORCE()      { return 0.8; };

// Keeper
    static double KEEPER_POST_MARGIN()          { return 0.08; };   // Meters
    static double KEEPER_CENTREGOAL_MARGIN()    { return 0.3; };    // Meters
    static double KEEPER_POSDIF()               { return 0.04; };

// BallkickedtoGoal
    static double BALL_TO_GOAL_MARGIN()         { return BALL_RADIUS(); };  //Margin at which a ball is still detected as 'kicked at goal' next to the goalie ends, so goalie tries to save the ball.
    static double BALL_TO_GOAL_TIME()           { return 1.5; };    // Seconds

// GoAroundPos
    static double MAX_GOAROUND_STOP_TIME()      { return 2.0; };    // Seconds
    static double GOAROUND_MAX_DIST_DEVIATION() { return 0.2; };    // Meters
    static double GOAROUND_SPEED()              { return 6.0; };    // Rad/second
    static double GOAROUND_BALL_DIST()          { return 0.11; };   // Meters
    static double GOAROUND_POS_MARGIN()         { return 0.05; };   // Meters
    static double GOAROUND_ANGLE_MARGIN()       { return 0.03*M_PI; };  //Rad
    static double GOAROUND_MIN_SPEED()          { return 0.25; };   // Meters/second

// Intercept
    static double MAX_INTERCEPT_TIME()          { return 2.0; };    // Seconds. Intercept terminates  after this time.
    static double BALL_DEFLECTION_ANGLE()       { return 30.0/180.0*M_PI; };    // Angle at which a ball is considered 'deflected'
    static double INTERCEPT_POSDIF()            { return 0.04; };   // Meters acceptable deviation
    static double DEFAULT_MAX_VEL()             { return 2.0; };

// BallInDefenseAreaAndStill
    static double BALL_STILL_VEL()              { return 0.1; };    // If the ball has velocity lower than this in defense area, keeper starts getting it

    static double INTERCEPT_P()                 { return 5.7;};
    static double INTERCEPT_I()                 { return 1.7;};
    static double INTERCEPT_D()                 { return 0.0;};

    static double DRIBBLE_ROTATE_WAIT_TIME()    { return 0.2; };    // Seconds
    static double DRIBBLE_ROTATE_MAX_SPEED()    { return 0.5; };    // Rad/second


// Interface
    static int ROBOT_DRAWING_SIZE()             { return 8; };
    static int BALL_DRAWING_SIZE()              { return 5; };
    static int TACTIC_COLOR_DRAWING_SIZE()      { return 10; };
    static int WINDOW_FIELD_MARGIN()            { return 5; };

    static int KEEPER_HELP_DRAW_SIZE()          { return 7; };
    static int INTERCEPT_DRAW_VECTOR_SIZE()     { return 5; };

    static double BP_MOVE_BACK_DIST()           { return 0.4; };
    static double BP_MOVE_TOWARDS_DIST()        { return 0.15; };

// Avoid ball
    static double robotWeight()                 { return 0.09; };
    static double minRobotDistanceForForce()    { return 0.7; };
    static double ballWeight()                  { return 0.15; };
    static double minBallDistanceForForce()     { return 0.7; };
    static double wallWeight()                  { return 0.05; };
    static double minWallDistanceForForce()     { return 0.4; };

// Settings
    static bool STD_SHOW_ROLES()                { return true; };
    static bool STD_SHOW_TACTICS()              { return false; };
    static bool STD_SHOW_TACTICS_COLORS()       { return true; };
    static bool STD_SHOW_VELOCITIES()           { return true; };
    static bool STD_SHOW_ANGLES()               { return true; };
    static bool STD_SHOW_VORONOI()              { return false; };
    static bool STD_SHOW_PATHS_ALL()            { return false; };
    static bool STD_SHOW_PATHS_CURRENT()        { return true; };
    static bool STD_SHOW_BALL_PLACEMENT_MARKER(){ return true; };
    static bool STD_SHOW_DEBUG_VALUES()         { return true; };
    static bool STD_USE_REFEREE()               { return false; };

    static QColor FIELD_COLOR()                 { return GRSIM() ? QColor(30 , 30 , 30 , 255) :
                                                                   QColor(50 , 0  , 0  , 255); };
    static QColor FIELD_LINE_COLOR()            { return Qt::white; };
    static QColor ROBOT_COLOR_BLUE()            { return {150, 150, 255, 255}; }; // Blue
    static QColor ROBOT_COLOR_YELLOW()          { return {255, 255, 0  , 255}; }; // Yellow
    static QColor BALL_COLOR()                  { return {255, 120, 50 , 255}; }; // Orange
    static QColor TEXT_COLOR()                  { return Qt::white; };
    static QColor SELECTED_ROBOT_COLOR()        { return Qt::magenta; };

    static std::vector<QColor> TACTIC_COLORS()  { return { {255, 0  , 255, 255},
                                                           {255, 0  , 255, 255},
                                                           {255, 255, 0  , 255},
                                                           {255, 120, 180, 255},
                                                           {255, 100, 255, 255} }; };

// Default PID values for the interface
    static double standardNumTreePosP()         { return GRSIM() ? 2.5 : 2.8; };
    static double standardNumTreePosI()         { return GRSIM() ? 0.0 : 0.6; };
    static double standardNumTreePosD()         { return GRSIM() ? 2.0 : 2.3; };

    static double standardNumTreeVelP()         { return GRSIM() ? 2.0 : 2.8; };
    static double standardNumTreeVelI()         { return GRSIM() ? 0.5 : 0.6; };
    static double standardNumTreeVelD()         { return GRSIM() ? 1.8 : 2.3; };

private:
    static bool isInitialized;
    static bool robotOutputTargetGrSim; // Don't use this value. use GRSIM() instead.
};

} // ai
} // rtt


enum class RefGameState {
// Ref states as dictated by RoboCup SSL
    HALT = 0,
    STOP = 1,
    NORMAL_START = 2,
    FORCED_START = 3,
    PREPARE_KICKOFF_US = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US = 8,
    DIRECT_FREE_THEM = 9,
    INDIRECT_FREE_US = 10,
    INDIRECT_FREE_THEM = 11,
    TIMEOUT_US = 12,
    TIMEOUT_THEM = 13,
    GOAL_US = 14,
    GOAL_THEM = 15,
    BALL_PLACEMENT_US = 16,
    BALL_PLACEMENT_THEM = 17,

// Custom extended refstates
// These numbers will never be called from the referee immediately, they can only be used as follow-up commands
    DO_KICKOFF = 18,
    DEFEND_KICKOFF = 19,
    DO_PENALTY = 20,
    DEFEND_PENALTY = 21,
    UNDEFINED = -1
};

#endif //ROBOTEAM_AI_CONSTANTS_H
