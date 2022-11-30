#pragma once
#include"ofMain.h"

enum VertexType {
	VertexStart = 0,
	//  [new ordering ]
	//  LR(-2, -1)	LF (2, -1)
	//  RR(-2, 1)		RF (2, 1)	
	LR = VertexStart,
	RR,
	RF,
	LF,
	VertexCount,
};

struct collisionRect {
	ofVec2f center;
	ofVec2f Vertex[VertexCount];
	float width;
	float height;
	float sizeVal;
	//Robot path area (Rect) to obstacle collision check.
	collisionRect(ofVec2f _center, ofVec2f _LR, ofVec2f _RR, ofVec2f _RF, ofVec2f _LF){
		center = _center;
		Vertex[LR] = _LR;
		Vertex[RR] = _RR;
		Vertex[RF] = _RF;
		Vertex[LF] = _LF;
		sizeVal = 0;
		width = std::sqrt(std::pow((_LF.x - _RF.x), 2) + std::pow((_LF.y - _RF.y), 2));
		float width2 = std::sqrt(std::pow((_LR.x - _RR.x), 2) + std::pow((_LR.y - _RR.y), 2));
		//assert(width == width2);	// two sides should be the same length
		height = std::sqrt(std::pow((_LF.x - _LR.x), 2) + std::pow((_LF.y - _LR.y), 2));
		float height2 = std::sqrt(std::pow((_RF.x - _RR.x), 2) + std::pow((_RF.y - _RR.y), 2));
		//assert(height == height2);	// two sides should be the same length
	}
	// Robot Rect to obstacle collision check
	collisionRect(ofVec2f _center, ofVec2f _LR, ofVec2f _RR, ofVec2f _RF, ofVec2f _LF, float _RobotSizeVal) : sizeVal(_RobotSizeVal) {
		// [Nov23]
		// current robot shape: || x || == 2*r,  || y || = 4*r rectangle.
		center = _center;
		Vertex[LR] = _LR;
		Vertex[RR] = _RR;
		Vertex[RF] = _RF;
		Vertex[LF] = _LF;
		width = 2 * _RobotSizeVal;
		height = 4 * _RobotSizeVal;
	}
	//Robot path area (Rect) to obstacle collision check.
	collisionRect(ofVec2f _center, ofVec2f _LR, ofVec2f _LF, ofVec2f _RF, ofVec2f _RR, float _w, float _h) : width(_w), height(_h) {
		center = _center;
		Vertex[LR] = _LR;
		Vertex[RR] = _RR;
		Vertex[RF] = _RF;
		Vertex[LF] = _LF;
		sizeVal = 0;
	}
};

struct collisionCircle {
	ofVec2f center;
	float radius;
	collisionCircle(ofVec2f location, float _radius)
	{
		center.x = location.x;
		center.y = location.y;
		radius = _radius;
	}
	collisionCircle(float x, float y, float _radius)
	{
		center.x = x;
		center.y = y;
		radius = _radius;
	}
};

class CollisionCheck {
private:
	static ofVec2f _getNormalizedVector(ofVec2f &curPt, ofVec2f &nextPt);
	static ofVec2f _getNormPerpendicularAxis(ofVec2f &curVertex, ofVec2f &nextVertex);
	static void _computeProjections(collisionRect &rect1, collisionRect &rect2, ofVec2f &normalizedAxis, vector <float> &projection1, vector <float> &projection2);
	static void _computeProjections(collisionRect &rect, collisionCircle &circle, ofVec2f &normalizedAxis, vector <float> &projection1, vector <float> &projection2);
	static bool _IsOverlapping(vector <float> &projection1, vector <float> &projection2);
public:
	CollisionCheck() {};
	static float getEucDist(ofVec2f a, ofVec2f b);
	static ofVec2f rotatedPoint(ofVec2f coordinate, float degree, ofVec2f origin = { 0,0 });
	static bool IsCollision_RectToRect(collisionRect rect1, collisionRect rect2);
	static bool IsCollision_RecToCircle(collisionRect rect, collisionCircle circle);
	static bool IsCollision_CircleToCircle(collisionCircle circle1, collisionCircle circle2);
};