#pragma once
#include"ofMain.h"

class CollisionCheck {
public:
	CollisionCheck() {};
	static ofVec2f rotatedPoint(ofVec2f coordinate, float degree, ofVec2f origin = { 0,0 });
	static bool IsCollision_RecToRec(ofVec2f rec1LF, ofVec2f rec1RF, ofVec2f rec1LR, ofVec2f rec1RR,
		ofVec2f rec2LF, ofVec2f rec2RF, ofVec2f rec2LR, ofVec2f rec2RR);
	static bool IsCollision_RecToCircle(ofVec2f recLF, ofVec2f recRF, ofVec2f recLR, ofVec2f recRR,
		ofVec2f circleCenter, float circleRadius);
	static bool IsCollision_CircleToCircle(ofVec2f circle1Center, float circle1Radius, ofVec2f circle2Center, float circle2Radius);
};
