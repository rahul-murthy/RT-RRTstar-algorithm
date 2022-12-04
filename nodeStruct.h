#pragma once
#include"ofMain.h"
struct Nodes
{
	ofVec2f location;
    ofVec2f LR, LF, RF, RR;
    ofVec2f velocity;
	ofVec2f orientation;
	float thetaXaxis;  // (-180, 180]
	double time;	// expected time for the robot to reach the Node

	Nodes *parent, *prevParent=NULL;

	bool alive = true;
	ofColor color = { 10, 12, 160 };
	float costToStart;
	std::list<Nodes*> children;
	Nodes()
	{
		parent = nullptr;
		prevParent = nullptr;
	}
	Nodes(float x_, float y_, float costToStart_, float _sizeVal, Nodes* p_ = NULL)
	{
		location.x = x_;
		location.y = y_;
		costToStart = costToStart_;
		parent = p_;
		float r = _sizeVal;
		LR.set(x_ -r, y_ -r * 2);
		LF.set(x_ -r, y_+r * 2);
		RF.set(x_+r, y_+r * 2);
		RR.set(x_+r, y_ -r * 2);
		velocity = { 0,0 };
		time = 0;
		orientation = { 1, 0 };
		thetaXaxis = 0;
	}
	Nodes(float x_, float y_, float costToStart_, Nodes* p_ = NULL)
	{
		location.x = x_;
		location.y = y_;
		costToStart = costToStart_;
		parent = p_;
		LR = { 0,0 };
		LF = { 0,0 };
		RF = { 0,0 };
		RR = { 0,0 };
		velocity = { 0,0 };
		time = 0;
		orientation = { 1, 0 };
		thetaXaxis = 0;
	}

};
