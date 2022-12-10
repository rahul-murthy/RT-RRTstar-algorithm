#pragma once
#include"simulationParam.h"
#include"nodeStruct.h"
#include"obstacle.h"
#include<list>
#include"SMP.h"
#include"Robot.h"
#include"RRTstar.h"
#include"InformedRRTstar.h"
#include"RT-RRTstar.h"
#include"ofxGui.h"
//#include <windows.h>
#include <iostream>
//--------------------------------------------------------------class
class Enviroment
{
public:
	//--------------------------------------------------------------Function
	// Default constructor  
	Enviroment() { setup(); };
	Enviroment(ofVec2f _start) { setup(_start); };
	// Default destructor  
	~Enviroment() {};
	// Setup method
	void setup();
	void setup(ofVec2f _start);
	void update(Robot * car, list<obstacles*> obst);
	void targetSet(ofVec2f loc1);
	void targetSet1(ofVec2f loc2);
	// Update method
	void update(Robot *car);
	// Render method draw nodes in enviroment.
	void render();
	float numofnode() { return nodes.size(); };
	void renderGrid();
	//--------------------------------------------------------------Variables
	bool grid = false;
	bool goal1in = false;
	bool goal2in = false;
	ofxFloatSlider guiRad,guiEpsilon;
	ofxPanel gui;
private:
	//--------------------------------------------------------------Variables
protected:
	//--------------------------------------------------------------Variables
	std::list<Nodes> nodes;
	//std::list<obstacles> obst;
	std::list<Nodes*> path;
	RRTstar rrtstar;
	InformedRRTstar irrtstar;
	RTRRTstar rtrrtstar;
	bool rrtFlag = true;
	bool planner = true;

	ofVec2f goal1;
	ofVec2f goal2;
	ofVec2f home;
	//Robot *car;

	// A list or an array of Obstacles should come here

};

inline void Enviroment::setup()
{
	home.set(startx, starty);
	Nodes start(startx, starty, 0);
	this->nodes.push_back(start);
	SMP::start.set(startx, starty);
	SMP::goalFound = false;
	SMP::goal1Found = false;
	SMP::goal2Found = false;
}


inline void Enviroment::setup(ofVec2f _start)
{
	gui.setup();
	gui.add(guiRad.setup("Radius", rrtstarradius, 10, 200));
	gui.add(guiEpsilon.setup("Epsilon",epsilon , 5, 150));
	home = _start;

	Nodes start(home.x, home.y, 0, robotSizeValue);
	this->nodes.push_back(start);

	SMP::root = &(this->nodes.front());
	goal1.set(goalx, goaly);
	goal2.set(goalx, goaly);
	SMP::start.set(startx, starty);
	SMP::goalFound = false;
	SMP::goal1Found = false;
	SMP::goal2Found = false;
}

inline void Enviroment::update(Robot *car, list<obstacles*> obst)
{
	//RRTstar - 
	//rrtstar.nextIter(nodes, obst);

	//Informed RRT*-
	//irrtstar.nextIter(nodes, obst);
	//InformedRRTstar::usingInformedRRTstar = true;

	//RTRRTstar-
	ofColor redColor = { 145,50,80 };
	ofColor greenColor = { 50,145,80 };
	bool isInside = false;

    if (car->getLocation().distance(SMP::goal1) < converge) {
        planner = false;
        if (SMP::GoalReachedTime == 0) {
            SMP::GoalReachedTime = ofGetElapsedTimef();
        }
    }

    if (car->getLocation().distance(SMP::goal2) < converge) {
        planner = false;
        if (SMP::GoalReachedTime == 0) {
            SMP::GoalReachedTime = ofGetElapsedTimef();
        }
    }

	if (planner)
	{
		car->fillEnviroment(obst, nodes);
		car->controller(SMP::root->location);
		car->update();
	}

	rtrrtstar.nextIter(nodes, obst, car);

	if (planner && SMP::target != NULL)
	{
		path = rtrrtstar.currPath;
		rtrrtstar.currPath.clear();
	}
	for (auto it : obst)
	{
#ifdef rectangleRobot
		collisionRect rect = car->getRenderedRectangle();
		if (it->isInside(rect))
#else
		if (it->isInside(car->getLocation()))
#endif
		{
			// [Nov12]	Rather than using an infinite loop,
			//				Should pass the "collision occured" information to the Open Framework and inform the user.
			while (true);
//          exit( 3 );
#if 0
            AllocConsole();
            if (MessageBox(FindWindowA("ConsoleWindowClass", NULL), L"Hi, Do you want to exit?", L"Bye!!", MB_HELP| MB_CANCELTRYCONTINUE | MB_ICONHAND | MB_DEFBUTTON2 | MB_SYSTEMMODAL) == IDCANCEL)
            {
                exit( 3 );
            }
#else
			if (car->getColor() != redColor)
			{
				car->setColor(redColor);
			}
			isInside = true;
#endif
		}

		// no collision at this moment
		if (isInside == false)
		{
			if (car->getColor() != greenColor)
			{
				car->setColor(greenColor);
			}
		}
	}
	
	//Following is Informed RRT-star stuff
	/*
	if (SMP::sampledInGoalRegion)
		SMP::sampledInGoalRegion = false;

	if (SMP::target != NULL && !SMP::moveNow && InformedRRTstar::usingInformedRRTstar)
	{
		path.clear();
		Nodes *pathNode = SMP::target;
		do
		{
			path.push_back(pathNode);
			pathNode = pathNode->parent;
		} while (pathNode->parent != NULL);
		//planner = !planner;
		path.reverse();
	}
	if (SMP::moveNow && InformedRRTstar::usingInformedRRTstar) {
		std::list<Nodes*>::iterator pathIt = path.begin();

		while(pathIt !=path.end()){
			if ((*pathIt)->alive) {
				car->controller((*pathIt)->location);
				float dist= (*pathIt)->location.distance(car->getLocation());
				if (dist < 2.0) (*pathIt)->alive = false;
				break;
			}
			pathIt++;
		}
		if(pathIt!=path.end())
		car->update();

	}*/
}

inline void Enviroment::targetSet(ofVec2f loc1)
{
	goal1 = loc1;
	SMP::goal1 = goal1;
	RTRRTstar::goalDefined = true;
	
	planner = true;
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		if ((*it).location.distance(loc1) < converge)
		{
			SMP::target = &(*it);
			return;
		}
		it++;
	}
	SMP::goalFound = false;
	SMP::goal1Found = false;
	SMP::target = NULL;
	path.clear();
	goal1in = true;
}

inline void Enviroment::targetSet1(ofVec2f loc1)
{
	goal2 = loc1;
	SMP::goal2 = goal2;
	RTRRTstar::goalDefined = true;

	planner = true;
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		if ((*it).location.distance(loc1) < converge)
		{
			SMP::target = &(*it);
			return;
		}
		it++;
	}
	SMP::goalFound = false;
	SMP::goal2Found = false;
	SMP::target = NULL;
	path.clear();
	goal2in = true;
}

inline void Enviroment::render()
{
	//gui.draw();
	ofEnableAlphaBlending();

	ofSetColor({150, 0, 255});
	if (goal1in) {
		ofFill();
		ofDrawCircle(goal1.x, goal1.y, NODE_RADIUS+2);
		ofNoFill();
		ofSetLineWidth(2);
		ofDrawCircle(goal1.x, goal1.y, converge);
	}

	ofSetColor({ 0, 170, 255 });
	if (goal2in) {
		ofFill();
		ofDrawCircle(goal2.x, goal2.y, NODE_RADIUS + 2);
		ofNoFill();
		ofSetLineWidth(2);
		ofDrawCircle(goal2.x, goal2.y, converge);
	}

	for (auto i : this->nodes)
	{
		ofSetColor({ 10,10,10 },150);

		if (i.costToStart == inf) ofSetColor({ 200,0,0 },120);
		if (i.alive == false) ofSetColor({ 200,0,0 }, 120);
		
		ofSetLineWidth(2);
		if (i.parent != NULL) {
			ofPoint pt;
			ofPolyline line;
			pt.set(i.location.x, i.location.y);line.addVertex(pt);
			pt.set(i.parent->location.x, i.parent->location.y);line.addVertex(pt);
			line.draw();
		}
		//ofSetColor({ 10,10,250 },80);
		ofSetLineWidth(1);
		//if (i.prevParent != NULL) {
		//	
		//	ofPoint pt; ofPolyline line;
		//	pt.set(i.location.x, i.location.y); line.addVertex(pt);
		//	pt.set(i.prevParent->location.x, i.prevParent->location.y); line.addVertex(pt);
		//	line.draw();
		//}
		//int hue = i.alive ? 130 : 80;
		//ofSetColor(i.color, hue);
		//ofDrawCircle(i.location.x, i.location.y, NODE_RADIUS);
	}
	if (!path.empty())
	{
		ofSetColor({ 10,250,10 });
		ofSetLineWidth(5);
		for (auto i : path) {
			if (i->parent != NULL) {
				ofPoint pt; ofPolyline line;
				pt.set(i->location.x, i->location.y); line.addVertex(pt);
				pt.set(i->parent->location.x, i->parent->location.y); line.addVertex(pt);
				line.draw();
			}
		}
		ofSetLineWidth(1);
	}
	ofDisableAlphaBlending();
}

inline void Enviroment::renderGrid()
{
	ofEnableAlphaBlending();
	ofSetColor(20, 130, 0, 50);
	for (int i = 0; i < ofGetWindowWidth(); i += 5)
	{
		ofDrawLine(i, 0, i, ofGetWindowHeight());
	}
	for (int j = 0; j < ofGetWindowHeight(); j += 5)
	{
		ofDrawLine(0, j, ofGetWindowWidth(), j);
	}
	ofDisableAlphaBlending();
}
