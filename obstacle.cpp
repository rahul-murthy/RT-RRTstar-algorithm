#include "obstacle.h"

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

bool obstacles::isInside(ofVec2f n)
{
	return (n.distance(location) <= radius + RobotRadius);
}
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
	ofVec2f temp, maxForce,maxVelocity;
	for (auto i : obst) {
		ofVec2f dir = location - i->loc();
		float accel = 1/ (dir.length() *dir.length());
		temp += accel* dir.normalized();
	}
	maxForce.set(mForce, mForce);
	temp = (temp.length() <= maxForce.length()) ? temp : (temp.normalized() *10*mForce);
	velocity = velocity + temp;
	maxVelocity.set(maxVal, maxVal);
	velocity = (velocity.length() <= maxForce.length()) ? velocity : (velocity.normalized() *maxVal);

	if (location.y+radius >= ofGetHeight() || location.y- radius <= 0) {
		velocity.y = velocity.y*-1;
	}
	if (location.x- radius <= 0 || location.x+ radius >= ofGetWidth()) {
		velocity.x = velocity.x*-1;
	}

	location += velocity;
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
bool movingObst::isInside(ofVec2f n)
{
	return (n.distance(location) <= radius + RobotRadius);
}
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

bool maze::isInside(ofVec2f p)
{
	// Need Rectangle(Robot) to Rectangle(Maze wall) isInside function
	return rect.inside(p);
}
#if 0
bool isCollide(ofVec2f){
	//prepare the vectors
	var v : Vector2d;
	var current_box_corner : Point;
	var center_box : Point = box1.getDot(0);

	var max : Number = Number.NEGATIVE_INFINITY;
	var box2circle : Vector2d = new Vector2d(c.x - center_box.x, c.y - center_box.y)
		var box2circle_normalised : Vector2d = box2circle.unitVector

		//get the maximum
		for (var i : int = 1; i < 5; i++)
		{
			current_box_corner = box1.getDot(i)
				v = new Vector2d(
					current_box_corner.x - center_box.x,
					current_box_corner.y - center_box.y);
			var current_proj : Number = v.dotProduct(box2circle_normalised)

				if (max < current_proj) max = current_proj;
		}
	if (box2circle.magnitude - max - c.radius > 0 && box2circle.magnitude > 0) t.text = "No Collision"
	else t.text = "Collision"
}
#endif