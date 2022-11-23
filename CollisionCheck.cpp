#include "CollisionCheck.h"

ofVec2f CollisionCheck::rotatedPoint(ofVec2f coordinate, float degree, ofVec2f origin)
{
	float oldX = coordinate.x;
	float oldY = coordinate.y;
	float s = sin(degree);
	float c = cos(degree);
	ofVec2f p = { 0,0 };

	// translate point back to origin:
	p.x -= origin.x;
	p.y -= origin.y;

	// rotate point
	float xnew = p.x * c - p.y * s;
	float ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + origin.x;
	p.y = ynew + origin.y;

	return p;
}

bool CollisionCheck::IsCollision_RecToRec(ofVec2f rec1LF, ofVec2f rec1RF, ofVec2f rec1LR, ofVec2f rec1RR,
	ofVec2f rec2LF, ofVec2f rec2RF, ofVec2f rec2LR, ofVec2f rec2RR) {
	assert(false);		// need implementation
	return false;
}
bool CollisionCheck::IsCollision_RecToCircle(ofVec2f recLF, ofVec2f recRF, ofVec2f recLR, ofVec2f recRR,
	ofVec2f circleCenter, float circleRadius) {
	assert(false);		// need implementation
	return false;
}
bool CollisionCheck::IsCollision_CircleToCircle(ofVec2f circle1Center, float circle1Radius, ofVec2f circle2Center, float circle2Radius) {
	// get euclidean distance
	 float dist = std::sqrt(std::pow((circle1Center.x - circle2Center.x), 2) + std::pow((circle1Center.y - circle2Center.y), 2));
	 return (dist < circle1Radius + circle2Radius);
}