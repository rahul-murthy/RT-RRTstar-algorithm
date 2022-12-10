#pragma once
#include "nodeStruct.h"
#include "ofMain.h"
#include "obstacle.h"
#include<set>

class SMP
{
public:
	SMP();
	static void addNode(Nodes n, std::list<Nodes>& nodes);
	static Nodes* nearestNode(Nodes n, std::list<Nodes>& nodes);
	static Nodes* nearestNode(Nodes n, std::list<Nodes*>& nodes);
	static bool checkCollision(Nodes n1, Nodes n2, list<obstacles*> obst);
	static bool checkSample(Nodes n, list<obstacles*> obst);
	static Nodes sampler();
	static bool goalFound;
	static bool goal1Found;
	static bool goal2Found;
	static bool sampledInGoalRegion;
	static bool moveNow;
	static ofVec2f start; 
	static ofVec2f goal;
	static ofVec2f goal1;
	static ofVec2f goal2;
	static Nodes* root;
	static Nodes* target;
	static Nodes* target1;
	static Nodes* target2;
	static Nodes* nextTarget;
	static float movingStartTime;
    static float GoalReachedTime;
};
