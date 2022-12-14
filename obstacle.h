#pragma once
#include"simulationParam.h"
#include "CollisionCheck.h"
#include "nodeStruct.h"

class obstacles
{
public:
	obstacles();
	obstacles(ofVec2f loc);
	obstacles(ofVec2f loc, float _rad);
	~obstacles();
#ifdef automatic
	virtual void move(std::list<obstacles*> obst);
	virtual void applyForce(ofVec2f force);
	virtual void update();
	virtual ofVec2f repulsive(obstacles *obst);
#endif // automatic
	virtual void render();
	virtual ofVec2f loc(){ return location; }
	virtual float rad() { return radius; }
	float getX() { return location.x;}
	float getY() { return location.y;}
	virtual  bool isCircle() { return true; }
	virtual bool isCollide(ofVec2f, ofVec2f);
#ifdef rectangleRobot
	// check if a path between node n and its parent is in collision with this obstacle
	virtual bool isCollide(Nodes &n);
	// robot(rectangle) collides with this static obstacle(circle)
	virtual bool isInside(collisionRect &rec);
	virtual bool isMovingObst() { return false; };
#ifdef predictMovement
	virtual bool isInside(collisionRect &rec, float time);
	virtual bool closeEnough(ofVec2f targetLocation, float nTargetRadius) { return false; };	// only for moving obstacles
#endif
#else
	// robot(point) collides with this static obstacle(circle)
	virtual bool isInside(ofVec2f);
#endif
	float mass;
private:
	ofVec2f location,velocity,accelaration;
	float radius;
	ofColor color;
};

class movingObst : public obstacles
{
public:
	movingObst();
	movingObst(ofVec2f loc);
#ifdef automatic
	movingObst(ofVec2f loc, ofVec2f vel);
	movingObst(ofVec2f loc, ofVec2f vel, float _rad);
#endif // automatic
	~movingObst();
	void render();
#ifdef manual
	void move(char key);
#endif // manual
#ifdef automatic
	void move(std::list<obstacles*> obst);
#endif // automatic
	ofVec2f loc() { return this->location; }
	float rad() { return this->radius; }
	bool isCircle() { return true; }
	bool isCollide(ofVec2f, ofVec2f);
#ifdef rectangleRobot
	// check if a path between node n and its parent is in collision with this obstacle
    bool isCollide(Nodes &n);
	// robot(rectangle) collides with this moving obstacle(circle)
	bool isInside(collisionRect &rec);
	bool isMovingObst() { return true; };
#ifdef predictMovement
	bool isInside(collisionRect &rec, float time);
	bool closeEnough(ofVec2f targetLocation, float nTargetRadius);
#endif
#else
	// robot(point) collides with this moving obstacle(circle)
	bool isInside(ofVec2f n);
#endif
	void applyForce(ofVec2f force);
	void update();
    double getNextChangeTime(double time);
	ofVec2f repulsive(obstacles *obst);
private:
    double curr_time;
	ofVec2f location, accelaration;
	float radius;
	ofColor color;
	float maxVal;
#ifdef automatic
	ofVec2f velocity;
#endif // automatic
};

class maze:public obstacles
{
public:
	maze(ofVec2f loc);
	maze(ofVec2f loc, float width, float height);
	~maze();
	void render();
#ifdef automatic
	void move(std::list<obstacles*> obst);
#endif 
	ofVec2f loc();
	bool isCircle() { return false; }
	bool isCollide(ofVec2f p1, ofVec2f p2);
#ifdef rectangleRobot
	// check if a path between node n and its parent is in collision with this obstacle
	bool isCollide(Nodes &n);
	// robot(rectangle) collides with this maze(rectangle)
	bool isInside(collisionRect &collRec);
	bool isMovingObst() { return false; };
#ifdef predictMovement
	bool isInside(collisionRect &rec, float time);
	bool closeEnough(ofVec2f targetLocation, float nTargetRadius) { return false; };	// only for moving obstacles
#endif
#else
	// robot(point) collides with this moving obstacle(rectangle)
	bool isInside(ofVec2f p);
#endif
private:
	ofColor color;
	ofVec2f location;
	float width, height;
	ofRectangle rect;
};
