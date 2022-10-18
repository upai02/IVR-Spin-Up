#include "robot.h"
#include <array>
#include <vector>
#include "misc/PositionTracker.h"

void followPath(std::vector<std::vector<double>> path, double lookForwardRadius) {

    double firstX = path[0][0];
    double firstY = path[0][1];
    double currentIndex = 0;

    // once the currentIndex is = to the last point on the path you exit the loop (because you're done)
    while (currentIndex < path.size() - 1) {
        updatePosition(imu.get_heading());

    }

    /*
    The plan:
     - get all the info you need from constructor
     - each cycle find points on path that connect to points on circle
     - determine which of these points are the best to go towards by...
        - seeing which is in front of / behind the robot (behind is bad)
        - Seeing what path segments the points belong to
        - if no intersections then go to last point you were going towards
          (which by default is the start of the path)
     - drive and turn towards the point (doing a bunch of math you've done before)
       and can do again

     - do special stuff once you get close to the end

    */
}