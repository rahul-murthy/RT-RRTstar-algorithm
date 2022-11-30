#pragma once

#pragma once
#include"simulationParam.h"
#include"nodeStruct.h"
#include"obstacle.h"
#include "CollisionCheck.h"
#define robotSizeValue 20

class Robot
{
public:
	// Rendered Points - for Debugging
	glm::vec4 render_LR;
	glm::vec4 render_RR;
	glm::vec4 render_RF;
	glm::vec4 render_LF;
	glm::vec4 render_Center;
	bool bIsStartedMoving;
	//--------------------------------------------------------------Function
	// Default constructor  
	Robot() { setup(); }
	Robot(ofVec2f loc) { setup(loc);}
	// Default destructor  
	~Robot() {};
	void setup();
	void setup(ofVec2f loc);
	// Update method
	void update();
	//Render method
	void render();
	//Compute force addition
	void addForce(ofVec2f force);
	//Controller genrate force toward target
	void controller(ofVec2f target);
	bool controller(ofVec2f target, ofVec2f targetVel);
	//Find Path from assign node
	//void fly(Nodes *&nodes);
	// Return state of Robot
	bool isAlive() { return alive; }
	// Return X cordinate
	float x() { return location.x; }
	// Return Y cordinate
	float y() { return location.y; }
	// Return scanning accuracy of Robot
	float accu() { return accuracy; }
	// Return scanning radius of Robot
	float getScanRadius() { return scanRadius; }

	// Return Location of Robot (point)
	ofVec2f getLocation() { return location; }
#ifdef rectangleRobot
	// calculate the coordinates of the four vertices
	void updateVertices();
	ofVec2f getVertex(VertexType eVertexType);
	// Return Location of Robot (Rectangle)
	collisionRect getRectangle() {
		//  [new ordering ]
		//  LR(-2, -1)	LF (2, -1)
		//  RR(-2, 1)		RF (2, 1)	
		collisionRect rec = collisionRect(location, LR, RR, RF, LF);
		return rec;
	}
	collisionRect getRenderedRectangle() {
		//  [new ordering ]
		//  LR(-2, -1)	LF (2, -1)
		//  RR(-2, 1)		RF (2, 1)	
		ofVec2f ren_LR = { render_LR.x, render_LR.y };
		ofVec2f ren_RR = { render_RR.x, render_RR.y };
		ofVec2f ren_RF = { render_RF.x, render_RF.y };
		ofVec2f ren_LF = { render_LF.x, render_LF.y };
		collisionRect rec = collisionRect(location, ren_LR, ren_RR, ren_RF, ren_LF);
		return rec;
	}
#endif
	// Return Color of Robot
	ofColor getColor() { return color; }
	void setColor(ofColor &newColor) { color = newColor; }
	ofVec2f getMaxVelocity() { return maxVelocity; }
	ofVec2f getVelocity() { return velocity; }
	void fillEnviroment(const list<obstacles*> obst,list<Nodes> &node);
	void updateEnviroment(list<Nodes> &node, obstacles *obst);
	bool isStartedMoving();
	//--------------------------------------------------------------Variables
private:
	bool alive;
	float scanRadius, mass, accuracy;
    float radius;

	ofColor color;
	ofVec2f HOME, location, velocity, accelaration, maxVelocity, maxForce;
	ofVec2f LR, RR, RF, LF;
	ofPolyline line;
	ofPoint pt;
};

