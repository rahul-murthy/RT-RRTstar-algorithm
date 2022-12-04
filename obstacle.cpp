#include "obstacle.h"
#include "SMP.h"

obstacles::obstacles()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	location.set(x, y);
	radius = ofRandom(10, 20);
	color = { 200,50,10 };
	mass = 3.14*radius*radius;
}

obstacles::obstacles(ofVec2f loc)
{
	location = loc;
	radius = ofRandom(10, 20);
	color = { 200,50,10 };
	mass = 3.14*radius*radius;
}

obstacles::obstacles(ofVec2f loc, float _rad)
{
	location = loc;
	radius = _rad;
	color = { 200,50,10 };
	mass = 3.14*radius*radius;
}

obstacles::~obstacles()
{
}
#ifdef automatic
void obstacles::move(std::list<obstacles*> obst)
{
	for (auto i : obst)
	{
		this->applyForce(repulsive(i));
	}
	this->update();
}
#endif
void obstacles::render()
{
	//move();
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	ofDrawCircle(location.x, location.y, radius-2);
	ofNoFill();
	ofDisableAlphaBlending();
}

bool obstacles::isCollide(ofVec2f n1, ofVec2f n2)
{
	float x1 = n1.x;
	float x2 = n2.x;
	float y1 = n1.y;
	float y2 = n2.y;

	float xo = location.x;
	float yo = location.y;
	float lambda = std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2);
	float t = (std::pow(x1, 2) + x2 * xo - x1 * (x2 + xo) - (yo - y1)*(y1 - y2)) / lambda;
	float shortest_dist;
	if (t >= 0 && t <= 1) // If the perpendicular distance lies on the line 'segment' connecting point_1 and point_2
		shortest_dist = std::sqrt(std::pow((x2 * (y1 - yo) + x1 * (yo - y2) + xo * (y2 - y1)), 2) / lambda);
	else  // If not then only check for the end-points of the segment for the collision
	{
		float d1 = std::sqrt(std::pow((x1 - xo), 2) + std::pow((y1 - yo), 2));
		float d2 = std::sqrt(std::pow((x2 - xo), 2) + std::pow((y2 - yo), 2));
		shortest_dist = std::min(d1, d2);
	}

	if (shortest_dist < radius + RobotRadius) 	return true;
	return false;
}

#ifdef rectangleRobot
bool obstacles::isCollide(Nodes &n)
{
#ifdef PathCollisionCheck
	Nodes* parent = n.parent;
	collisionRect *rec;
	collisionCircle *ObstacleCircle = new collisionCircle(this->loc(), this->rad());

	// Step1. Check if the rectangle shaped area that the robot covers when it follows this path. 
	//          (FROM parent node TO node n)
	if (parent != nullptr)
	{
		collisionRect *rotatedParentRect = new collisionRect(parent->LR, parent->LF /*LF*/, parent->RF /*RF*/, parent->RR /*RR*/);
		CollisionCheck::rotateRectToTarget(*rotatedParentRect, n.orientation);
		rec = new collisionRect(rotatedParentRect->Vertex[VertexType::LR] /*LR*/, n.LF /*LF*/, n.RF /*RF*/, rotatedParentRect->Vertex[VertexType::LR] /*RR*/);
		delete rotatedParentRect;
	}
	else
	{
		rec = new collisionRect(n.location, n.LR, n.LF, n.RF, n.RR);
	}

	/// need prediction if this is a moving obstacle.
	if (CollisionCheck::IsCollision_RecToCircle(*rec, *ObstacleCircle)) return true;
	delete rec;

	// Step2. Check if the circle shaped area that the robot covers when it rotates. 
	//          (before it starts moving toward the Node n)
	float rad = (n.LF.distance(n.RR) / 2);

	if (parent != nullptr)
	{
		collisionCircle *parentRotationCircle = new collisionCircle(parent->location.x, parent->location.y, rad);
		if (CollisionCheck::IsCollision_CircleToCircle(*parentRotationCircle, *ObstacleCircle)) return true;
		delete parentRotationCircle;
	}

	collisionCircle *rotationCircle = new collisionCircle(n.location.x, n.location.y, rad);
	if (CollisionCheck::IsCollision_CircleToCircle(*rotationCircle, *ObstacleCircle)) return true;
	delete rotationCircle;
	delete ObstacleCircle;

	return false;
#else
	assert(false); /// use this function only when the robot's shape is rectangle
	return false;
#endif
}
#endif

#ifdef rectangleRobot
bool obstacles::isInside(collisionRect &rec)
{
	collisionCircle circle = collisionCircle(this->loc(), this->rad());
	return CollisionCheck::IsCollision_RecToCircle(rec, circle);
}
#ifdef predictMovement
bool obstacles::isInside(collisionRect &rec, float time)
{
	return isInside(rec);
}
#endif
#else	// when robot is a point with no area.
bool obstacles::isInside(ofVec2f n)
{
	return (n.distance(location) <= radius + RobotRadius);
}
#endif

#ifdef automatic
void obstacles::applyForce(ofVec2f force)
{
	accelaration += force/mass;
}

void obstacles::update()
{
	velocity += accelaration;
	location += velocity;
	accelaration.set(0, 0);
}

ofVec2f obstacles::repulsive(obstacles *obst)
{
	ofVec2f force = location - obst->loc();
	float distance = force.length();
	if (distance < 5.0 || distance > 25.0) {
		if (distance < 5.0) distance = 5.0;
		else distance = 25.0;
	}
	force.normalized();
	float strength = (66.7428e-11 * mass * obst->mass) / (distance * distance);
	force.rescale(strength);
	return force;
}
#endif
movingObst::movingObst()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	location.set(x, y);
	maxVal = obstMaxVelocity;
#ifdef automatic
	velocity.set(maxVal*ofRandom(-1,1), maxVal*ofRandom(-1, 1));
#endif // automatic
	radius = 25;
	mass = 3.14*radius*radius;
	color = { 200,100,20 };
}

movingObst::movingObst(ofVec2f loc)
{
	location = loc;
	maxVal = obstMaxVelocity;
#ifdef automatic
	velocity.set(maxVal*ofRandom(-1, 1), maxVal*ofRandom(-1, 1));
#endif // automatic
	radius = 25;
	mass = 3.14*radius*radius;
	color = { 200,100,20 };
}

movingObst::movingObst(ofVec2f loc, ofVec2f vel)
{
	location = loc;
	maxVal = obstMaxVelocity;
#ifdef automatic
	velocity = vel;
#endif // automatic
	radius = 25;
	mass = 3.14*radius*radius;
	color = { 200,100,20 };
}

movingObst::movingObst(ofVec2f loc, ofVec2f vel, float _rad)
{
	location = loc;
	maxVal = obstMaxVelocity;
#ifdef automatic
	velocity = vel;
#endif // automatic
	radius = _rad;
	mass = 3.14*radius*radius;
	color = { 200,100,20 };
}

movingObst::~movingObst()
{
}

void movingObst::render()
{
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	ofDrawCircle(location.x, location.y, radius-2);
	ofNoFill();
	ofDisableAlphaBlending();
}
#ifdef manual
void movingObst::move(char key)
{
	if (key == 'w')
	{
		location.y -= maxVal;
	}
	else if (key == 's')
	{
		location.y += maxVal;
	}
	if (key == 'a')
	{
		location.x -= maxVal;
	}
	else if (key == 'd')
	{
		location.x += maxVal;
	}
}
#endif // 
#ifdef automatic
void movingObst::move(std::list<obstacles*> obst)
{
	if (SMP::goalFound)
	{
#if 0
		ofVec2f temp, maxForce, maxVelocity;
		for (auto i : obst) {
			ofVec2f dir = location - i->loc();
			float accel = 1 / (dir.length() *dir.length());
			temp += accel * dir.normalized();
		}
		maxForce.set(mForce, mForce);
		temp = (temp.length() <= maxForce.length()) ? temp : (temp.normalized() * 10 * mForce);
		velocity = velocity + temp;
		maxVelocity.set(maxVal, maxVal);
		velocity = (velocity.length() <= maxForce.length()) ? velocity : (velocity.normalized() *maxVal);
#endif
		if (location.y + radius >= ofGetHeight() || location.y - radius <= 0) {
			velocity.y = velocity.y*-1;
		}
		if (location.x - radius <= 0 || location.x + radius >= ofGetWidth()) {
			velocity.x = velocity.x*-1;
		}

		location += velocity;
	}
}
#endif // automatic
bool movingObst::isCollide(ofVec2f n1, ofVec2f n2)
{
	float x1 = n1.x;
	float x2 = n2.x;
	float y1 = n1.y;
	float y2 = n2.y;

	float xo = location.x;
	float yo = location.y;

	
	float lambda = std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2);
	float t = (std::pow(x1, 2) + x2 * xo - x1*(x2 + xo) - (yo - y1)*(y1 - y2)) / lambda;
	float shortest_dist;
	if (t >= 0 && t <= 1) // If the perpendicular distance lies on the line 'segment' connecting point_1 and point_2
		shortest_dist = std::sqrt(std::pow((x2 * (y1 - yo) + x1 * (yo - y2) + xo * (y2 - y1)), 2) / lambda);
	else  // If not then only check for the end-points of the segment for the collision
	{
		float d1 = std::sqrt(std::pow((x1 - xo), 2) + std::pow((y1 - yo), 2));
		float d2 = std::sqrt(std::pow((x2 - xo), 2) + std::pow((y2 - yo), 2));
		shortest_dist = std::min(d1, d2);
	}

	if (shortest_dist < radius + RobotRadius) 	return true;
	return false;
}

#ifdef rectangleRobot
bool movingObst::isCollide(Nodes &n)
{
#ifdef PathCollisionCheck
	Nodes* parent = n.parent;
	collisionRect *rec;
	float timeDiff = n.time;
	/// need prediction if this is a moving obstacle.
	collisionCircle *ObstacleCircle = new collisionCircle(this->loc() + (velocity * timeDiff), this->rad());


	// Step1. Check if the rectangle shaped area that the robot covers when it follows this path. 
	//          (FROM parent node TO node n)
	if (parent != nullptr)
	{
		collisionRect *rotatedParentRect = new collisionRect(parent->LR, parent->LF /*LF*/, parent->RF /*RF*/, parent->RR /*RR*/);
		CollisionCheck::rotateRectToTarget(*rotatedParentRect, n.orientation);
		rec = new collisionRect(rotatedParentRect->Vertex[VertexType::LR] /*LR*/, n.LF /*LF*/, n.RF /*RF*/, rotatedParentRect->Vertex[VertexType::LR] /*RR*/);
		delete rotatedParentRect;
	}
	else
	{
		rec = new collisionRect(n.location, n.LR, n.LF, n.RF, n.RR);
	}

	/// need prediction if this is a moving obstacle.
	if (CollisionCheck::IsCollision_RecToCircle(*rec, *ObstacleCircle)) return true;
	delete rec;

	// Step2. Check if the circle shaped area that the robot covers when it rotates. 
	//          (before it starts moving toward the Node n)
	float rad = (n.LF.distance(n.RR) / 2);
	if (parent != nullptr)
	{
		collisionCircle *parentRotationCircle = new collisionCircle(parent->location.x, parent->location.y, rad);
		if (CollisionCheck::IsCollision_CircleToCircle(*parentRotationCircle, *ObstacleCircle)) return true;
		delete parentRotationCircle;
	}

	collisionCircle *rotationCircle = new collisionCircle(n.location.x, n.location.y, rad);
	
	if (CollisionCheck::IsCollision_CircleToCircle(*rotationCircle, *ObstacleCircle)) return true;
	delete rotationCircle;
	delete ObstacleCircle;

	return false;
#else
	assert(false); /// use this function only when the robot's shape is rectangle
	return false;
#endif
}
#endif

#ifdef rectangleRobot
bool movingObst::isInside(collisionRect &rec)
{
	collisionCircle circle = collisionCircle(this->loc(), this->rad());
	return CollisionCheck::IsCollision_RecToCircle(rec, circle);
}
#ifdef predictMovement
bool movingObst::isInside(collisionRect &rec, float time)
{
	// update location
	ofVec2f predictedLocation = {this->loc().x + (velocity.x * time), this->loc().y + (velocity.y * time)};
	collisionCircle circle = collisionCircle(predictedLocation, this->rad());

	return CollisionCheck::IsCollision_RecToCircle(rec, circle);
}
#endif
#else	// when robot is a point with no area.
bool movingObst::isInside(ofVec2f n)
{
	return (n.distance(location) <= radius + RobotRadius);
}
#endif

#ifdef automatic
void movingObst::applyForce(ofVec2f force)
{
	accelaration += force / mass;
}
void movingObst::update()
{
	velocity += accelaration;
	location += velocity;
	accelaration.set(0, 0);
}

ofVec2f movingObst::repulsive(obstacles *obst)
{
	ofVec2f force = location - obst->loc();
	float distance = force.length();
	if (distance < 5.0 || distance > 25.0) {
		if (distance < 5.0) distance = 5.0;
		else distance = 25.0;
	}
	force.normalized();
	float strength = (66.7428e-11 * mass * obst->mass) / (distance * distance);
	force.rescale(strength);
	return force;
}
#endif
maze::maze(ofVec2f loc)
{
	location = loc;
	color = { 10,10,50 };
	rect.height = 0.40*ofGetHeight();
	rect.width = 20;
	rect.x = loc.x;
	rect.y = loc.y;
	mass = 1000;
}

maze::maze(ofVec2f loc, float width, float height)
{
	location = loc;
	color = { 10,10,50 };
	rect.height = height;
	rect.width = width;
	rect.x = loc.x;
	rect.y = loc.y;
	mass = 1000;
}

maze::~maze()
{
}

void maze::render()
{
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	//ofRect(location.x, location.y, width, height);
	ofDrawRectangle(rect);
	ofNoFill();
	ofDisableAlphaBlending();
}
#ifdef automatic
void maze::move(std::list<obstacles*> obst)
{

}
#endif
ofVec2f maze::loc()
{
	ofVec2f temp;
	temp.set(rect.width, rect.height);
	return location + temp;
}

bool maze::isCollide(ofVec2f p1, ofVec2f p2)
{
	// Need Rectangle(Robot) to Rectangle(Maze wall) isCollide function
	return rect.intersects(p1,p2);
}

#ifdef rectangleRobot
bool maze::isCollide(Nodes &n)
{
#ifdef PathCollisionCheck
	Nodes* parent = n.parent;
	collisionRect *rec;
	ofVec2f _LR = { rect.getMinX(), rect.getMinY() };
	ofVec2f _RR = { rect.getMinX(), rect.getMaxY() };
	ofVec2f _RF = { rect.getMaxX(), rect.getMaxY() };
	ofVec2f _LF = { rect.getMaxX(), rect.getMinY()};

	collisionRect *ObstacleRect = new collisionRect(this->loc(), _LR, _RR, _RF, _LF);

	// Step1. Check if the rectangle shaped area that the robot covers when it follows this path. 
	//          (FROM parent node TO node n)
	if (parent != nullptr)
	{
		collisionRect *rotatedParentRect = new collisionRect(parent->LR, parent->LF /*LF*/, parent->RF /*RF*/, parent->RR /*RR*/);
		CollisionCheck::rotateRectToTarget(*rotatedParentRect, n.orientation);
		rec = new collisionRect(rotatedParentRect->Vertex[VertexType::LR] /*LR*/, n.LF /*LF*/, n.RF /*RF*/, rotatedParentRect->Vertex[VertexType::LR] /*RR*/);
		delete rotatedParentRect;
	}
	else
	{
		rec = new collisionRect(n.location, n.LR, n.LF, n.RF, n.RR);
	}

	/// need prediction if this is a moving obstacle.
	if (CollisionCheck::IsCollision_RectToRect(*rec, *ObstacleRect)) return true;
	delete rec;

	// Step2. Check if the circle shaped area that the robot covers when it rotates. 
	//          (before it starts moving toward the Node n)
	float rad = (n.LF.distance(n.RR) / 2);
	if (parent != nullptr)
	{
		// If there is a parent node, check collision with a circle centered at the parent node.
		collisionCircle *parentRotationCircle = new collisionCircle(parent->location.x, parent->location.y, rad);
		if (CollisionCheck::IsCollision_RecToCircle(*ObstacleRect, *parentRotationCircle)) return true;
		delete parentRotationCircle;
	}

	// Check collision with a circle centered at the node.
	collisionCircle *rotationCircle = new collisionCircle(n.location.x, n.location.y, rad);
	if (CollisionCheck::IsCollision_RecToCircle(*ObstacleRect, *rotationCircle)) return true;
	delete ObstacleRect;

	return false;
#else
	assert(false); /// use this function only when the robot's shape is rectangle
	return false;
#endif
}
#endif

#ifdef rectangleRobot
bool maze::isInside(collisionRect &collRec)
{
	//  _LR, ofVec2f _LF, ofVec2f _RF, ofVec2f _RR
	ofVec3f tmpTL = rect.getTopLeft();
	ofVec3f tmpBL = rect.getBottomLeft();
	ofVec3f tmpBR = rect.getBottomRight();
	ofVec3f tmpTR = rect.getTopRight();
	ofVec3f tmpCenter = rect.getCenter();
	ofVec2f LR = { tmpTL.x, tmpTL.y };
	ofVec2f LF = { tmpBL.x, tmpBL.y };
	ofVec2f RF = { tmpBR.x, tmpBR.y };
	ofVec2f RR = { tmpTR.x, tmpTR.y };
	ofVec2f mazeCenter = { tmpCenter.x, tmpCenter.y};

	collisionRect mazeRec = collisionRect(mazeCenter, LR, LF, RF, RR);
	return CollisionCheck::IsCollision_RectToRect(collRec, mazeRec);
}
#ifdef predictMovement
bool maze::isInside(collisionRect &rec, float time)
{
	return isInside(rec);
}
#endif
#else	// when robot is a point with no area.
bool maze::isInside(ofVec2f p)
{
	// Need Rectangle(Robot) to Rectangle(Maze wall) isInside function
	return rect.inside(p);
}
#endif