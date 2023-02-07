#ifndef MOCKTRACK_H
#define MOCKTRACK_H

#include "LineFollower.cpp" //Copy LineFollower.cpp and LineFollower.h into this directory for testing
#include <iostream>
#define MAX_SIMS 100

class Track
{
    private:
        direction turns[6] = {right, right, left, left, left, right};
        int currentIndex = 0; 
        float leftMotor = 0;
        float rightMotor = 0;
    public:
        direction readTrack();
        void simulate();
        LineFollower controller = LineFollower(leftMotor, rightMotor);
};

direction Track::readTrack()
{
    return turns[currentIndex];
}

void Track::simulate()
{   
    int branchCount = 0;
    int pathfindRes;
    direction dir;

    for (int i = 0; i < MAX_SIMS; i++)
    {
        dir = readTrack();
        pathfindRes = controller.pathfind(dir, branchCount);

        std::cout << "Direction: " << dir << "\n" << "Pathfind result: " << pathfindRes << "\n" << "Branch Count: " << branchCount << "\n\n" << std::endl;

        if(currentIndex == 5) currentIndex = 0;
        else currentIndex++;
    }
}
#endif