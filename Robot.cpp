#include "Robot.h"
#include "SMP.h"

void Robot::setup()
{
//        alive = true; mass = 5.0; scanRadius = sensorRadius; accuracy = accur;
        alive = true; mass = 5.0; scanRadius = robotSizeValue; accuracy = accur;
        //battery = 100;
        //float x = ofRandom(0, ofGetWindowWidth()); float y = ofRandom(0, ofGetWindowHeight());`
        location.set(0.0,0.0);
//        location.set(0.0,0.0,0.0,0.0);
        HOME = location;
        velocity.set(0.0, 0.0);
        accelaration.set(0.0, 0.0);
        maxVelocity.set(mVal, mVal);
        maxForce.set(mForce, mForce);
        //color = { ofRandom(0,255),ofRandom(0,255) ,ofRandom(0,255) };
        color = {50,145,80};
		bIsStartedMoving = false;
#ifdef rectangleRobot
		// Get 4 corners around the center
		float r = robotSizeValue;
		//  [new ordering ]
		//  LR(-2, -1)	LF (2, -1)
		//  RR(-2, 1)		RF (2, 1)	
		LR.set(-2*r + location.x, (-r + location.y));
		RR.set(-2*r + location.x, (r + location.y));
		RF.set(2*r + location.x, (r + location.y));
		LF.set(2*r + location.x, (-r + location.y));
#endif
}

void Robot::setup(ofVec2f loc)
{
    alive = true; mass = 5.0; scanRadius = 20; accuracy = accur;
    location = loc;
    HOME = location;
    velocity.set(0.0, 0.0);
    accelaration.set(0.0, 0.0);
    maxVelocity.set(mVal, mVal);
    maxForce.set(mForce, mForce);
    color = { 50,145,80 };
	bIsStartedMoving = false;

#ifdef rectangleRobot
	// Get 4 corners around the center
	float r = robotSizeValue;
	//  [new ordering ]
	//  LR(-2, -1)	LF (2, -1)
	//  RR(-2, 1)		RF (2, 1)	
	LR.set(-2 * r + location.x, (-r + location.y));
	RR.set(-2 * r + location.x, (r + location.y));
	RF.set(2 * r + location.x, (r + location.y));
	LF.set(2 * r + location.x, (-r + location.y));
#endif
}

void Robot::update()
{
    velocity += accelaration;
    velocity = (velocity.length() <= maxVelocity.length()) ? velocity : (velocity.normalized() *mVal);

	location += velocity;
    accelaration *= 0.0;
    pt.set(location.x, location.y);
    line.addVertex(pt);

#ifdef rectangleRobot
	if (velocity.length() != 0)
	{
		updateVertices();
	}
#endif
}

void Robot::render()
{
    float r = robotSizeValue;
    ofEnableAlphaBlending();
    ofFill();
    ofSetColor(color);
    ofSetLineWidth(3);

    this->line.draw();

    ofSetLineWidth(1);
    ofNoFill();
    ofSetColor(color);
    ofDrawCircle(location.x,location.y,scanRadius);
    ofPushMatrix();
    ofTranslate(location.x,location.y);
    ofRotate(ofRadToDeg(atan2(velocity.y, velocity.x)));
    ofNoFill();
    
    ofBeginShape();
#ifdef rectangleRobot
    // Rectangle Shape
	// Y axis and X axis are reversed in render environment.
	// -----------------------> (y)
	//     | 
	//     |
	//     |
	// (x) v 
	glm::vec4 _LR = {-2 * r, -r, 0, 1};
	glm::vec4 _RR = { -2 * r, r, 0, 1 };
	glm::vec4 _RF = { 2 * r, r, 0, 1 };
	glm::vec4 _LF = { 2 * r, -r, 0, 1 };
	glm::vec4 _O = { 0, 0, 0, 1 };

	auto modelMatrix = glm::inverse(ofGetCurrentViewMatrix()) * ofGetCurrentMatrix(OF_MATRIX_MODELVIEW);
	_LR = modelMatrix *  _LR;
	_RR =  modelMatrix * _RR;
	_RF = modelMatrix * _RF;
	_LF =  modelMatrix * _LF;
	_O = modelMatrix * _O;

	render_LR = _LR;
	render_RR = _RR;
	render_RF = _RF;
	render_LF = _LF;
	render_Center = _O;

	//  [new ordering ]
	//  LR(-2, -1)	LF (2, -1)
	//  RR(-2, 1)		RF (2, 1)	
    ofVertex(-2 * r, -r);		// LR
	ofVertex(-2 * r, r);		// RR
	ofVertex(2 * r, r);		// RF
	// ofVertex(2 * r, -r);		// LF
#else
	// Triangle Shape (only rendering. The actual robot shape is a point)
	// ofVertex(0, -r * 2);
	// ofVertex(-r, r * 2);
	// ofVertex(r, r * 2);
#endif
    ofEndShape(true);
    
    ofPopMatrix();
    ofSetColor(color, 80);
    ofDrawCircle(location.x, location.y, ofGetFrameNum() % int(scanRadius));
    ofNoFill();
    ofDisableAlphaBlending();
}

void Robot::addForce(ofVec2f force)
{
    accelaration += (force / mass);
}

void Robot::controller(ofVec2f target)
{
    ofVec2f error = (target - location);
    //error.normalize();
    //error *= 1.5;
    //accelaration = error;
    float m;
    if (error.length() < converge) {
        m = ofMap(error.length(), 0, converge, 0, mVal);
    }
    else {
        m = mVal;
    }

    ofVec2f temp = error.normalized()*m;
    ofVec2f steer = (temp - velocity);
    steer = (steer.length() <= maxForce.length()) ? steer : (steer.normalized() *mForce);
    addForce(steer);
}

bool Robot::isStartedMoving(void)
{
	if (bIsStartedMoving == false && SMP::goalFound)
	{
		bIsStartedMoving = true;
		return true;
	}
	return false;
}

bool Robot::controller(ofVec2f target, ofVec2f targetVel)
{
	ofVec2f error = (target - location);
	//error.normalize();
	//error *= 1.5;
	//accelaration = error;
	float m;
	if (error.length() < 0.1) {
		this->velocity = { 0, 0 };
		return true;
	}
	else if (error.length() < converge) {
		m = ofMap(error.length(), 0, converge, 0, mVal);
	}
	else {
		this->velocity = targetVel;
		return false;
	}

	ofVec2f temp = error.normalized()*m;
	ofVec2f steer = (temp - velocity);
	steer = (steer.length() <= maxForce.length()) ? steer : (steer.normalized() *mForce);
	addForce(steer);
	return false;
}

void Robot::fillEnviroment(const list<obstacles*> obst, list<Nodes>& node)
{
    //check for enviroment
    for (auto index : obst) {
        float dist = this->location.distance(index->loc());
        if (dist <= this->scanRadius + index->rad()) {
            updateEnviroment(node, index);
        }
    }
}

void Robot::updateEnviroment(list<Nodes>& node,obstacles *obst)
{
    std::list<Nodes>::iterator it = node.begin();
    while (it != node.end())
    {
        float dist = it->location.distance(obst->loc());
        if (dist <= obst->rad()) {
            it->costToStart = inf;
            it->alive = false;
        }
        it++;
    }
	// updateVertices();
}

#ifdef rectangleRobot
void Robot::updateVertices()
{
#if 0
    ofVec2f _LR, _RR, _RF, _LF;
#else
	float r = robotSizeValue;
	glm::vec4 _LR = { -2 * r, -r, 0, 1 };
	glm::vec4 _RR = { -2 * r, r, 0, 1 };
	glm::vec4 _RF = { 2 * r, r, 0, 1 };
	glm::vec4 _LF = { 2 * r, -r, 0, 1 };
#endif
	ofVec2f axisX = { 1,0 };
	ofVec2f orientation = velocity.normalized();
    // float theta = ofRadToDeg(atan2(velocity.y, velocity.x));

	// theta diff between the closet node and the new node.
	float cos_theta = axisX.dot(orientation);
	if (cos_theta > 1) { cos_theta = 1; }
	else if (cos_theta < -1) { cos_theta = -1; }
	float theta = ofRadToDeg(glm::acos(cos_theta));  // glm::acos() returns [0:PI]. 

	// tell if the rotation should be done clockwise or counter clockwise
	float crossProduct = (axisX.x * orientation.y) - (axisX.y * orientation.x);

	if (crossProduct < 0) {
		// clockwise rotation. (-)
		theta *= -1;
	}

#if 0
	ofVec2f tmp1, tmp2;
	tmp1.set(cos(theta), -sin(theta));
	tmp2.set(sin(theta), cos(theta));

    // Set origin
    // Get 4 corners
    // Do the rotation metric on 4 points
    // Shift back to original spot by adding original x and y
    // Have the current location of the 4 corners

	// Get 4 corners around origin
	//  [new ordering ]
	//  LR(-2, -1)	LF (2, -1)
	//  RR(-2, 1)		RF (2, 1)	
	_LR.set(-2 * r, -r);
	_RR.set(-2 * r, r);
	_RF.set(2 * r, r);
	_LF.set(2 * r, -r);

	// Do the rotation metric
	_LR.set((tmp1.x*_LR.x + tmp1.y*_LR.y), (tmp2.x*_LR.x + tmp2.y*_LR.y));
	_RR.set((tmp1.x*_RR.x + tmp1.y*_RR.y), (tmp2.x*_RR.x + tmp2.y*_RR.y));
	_RF.set((tmp1.x*_RF.x + tmp1.y*_RF.y), (tmp2.x*_RF.x + tmp2.y*_RF.y));
	_LF.set((tmp1.x*_LF.x + tmp1.y*_LF.y), (tmp2.x*_LF.x + tmp2.y*_LF.y));

	// Shift back
	_LR.set(_LR.x + location.x, _LR.y + location.y);
	_RR.set(_RR.x + location.x, _RR.y + location.y);
	_RF.set(_RF.x + location.x, _RF.y + location.y);
	_LF.set(_LF.x + location.x, _LF.y + location.y);

	this->LR = _LR;
	this->RR = _RR;
	this->RF = _RF;
	this->LF = _LF;
#else
	glRotatef(theta, 0, 0, 1);

	_LR = {-2 * r, -r, 0, 1};
	_RR = {-2 * r, r, 0, 1};
	_RF = {2 * r, r, 0, 1};
	_LF = {2 * r, -r, 0, 1};

	auto modelMatrix = glm::inverse(ofGetCurrentViewMatrix()) * ofGetCurrentMatrix(OF_MATRIX_MODELVIEW);
	_LR = modelMatrix * _LR;
	_RR = modelMatrix * _RR;
	_RF = modelMatrix * _RF;
	_LF = modelMatrix * _LF;

	_LR = {_LR.x + location.x, _LR.y + location.y, 0, 1};
	_RR = {_RR.x + location.x, _RR.y + location.y, 0, 1};
	_RF = {_RF.x + location.x, _RF.y + location.y, 0, 1};
	_LF = {_LF.x + location.x, _LF.y + location.y, 0, 1};

	this->LR = { _LR.x, _LR.y };
	this->RR = { _RR.x, _RR.y };
	this->RF = { _RF.x, _RF.y };
	this->LF = { _LF.x, _LF.y };
#endif
}

ofVec2f Robot::getVertex(VertexType eVertexType)
{
	ofVec2f* Vertex = nullptr;
	switch (eVertexType)
	{
		//  [new ordering ]
		//  LR(-2, -1)	LF (2, -1)
		//  RR(-2, 1)		RF (2, 1)	
		case VertexType::LR:
			Vertex = &LR;
			break;
		case VertexType::RR:
			Vertex = &RR;
			break;
		case VertexType::RF:
			Vertex = &RF;
			break;
		case VertexType::LF:
			Vertex = &LF;
			break;
		default:
			break;
	}
	return *Vertex;
}
#endif

//
//inline void quadCopter::fly(Nodes *& nodes)
//{
//    if (!wTraj.empty())
//    {
//        int index = 0;
//        bool prio = false;
//        Vector2f temp('inf', 'inf');
//        for (auto i : wTraj)
//        {
//            if (nodes->waypoint[i.index].alive)
//            {
//                if (!nodes->waypoint[i.index].stress) nodes->waypoint[i.index].color = color;
//                else nodes->waypoint[i.index].color = { 255,0,0 };
//                Vector2f min = this->location - nodes->waypoint[i.index].location;
//                if (min.norm()<temp.norm() && nodes->waypoint[i.index].priority)
//                {
//                    temp = min;
//                    index = i.index;
//                    prio = true;
//                }
//            }
//        }
//        if (prio) {
//            controller(nodes->waypoint[index].location);
//        }
//        //else if (!prio) {
//        //    int index = 0;
//        //    Vector2f temp('inf', 'inf');
//        //    for (auto i : wTraj)
//        //    {
//        //        if (nodes->waypoint[i.index].alive)
//        //        {
//        //            if (!nodes->waypoint[i.index].stress) nodes->waypoint[i.index].color = color;
//        //            else nodes->waypoint[i.index].color = { 255,0,0 };
//        //            Vector2f min = this->location - nodes->waypoint[i.index].location;
//        //            if (min.norm()<temp.norm() && nodes->waypoint[i.index].priority)
//        //            {
//        //                temp = min;
//        //                index = i.index;
//        //                prio = true;
//        //            }
//        //        }
//        //    }
//        //    controller(nodes->waypoint[index].location);
//        //}
//
//        //list<waypoints>::iterator it;
//        //waypoints target = findAlive(wTraj,nodes);
//        //controller(target.location);
//
//    }
//    else
//    {
//        //Vector2f mouse(ofGetMouseX(), ofGetMouseY());
//        //controller(mouse);
//        Vector2f HOME(0, 0);
//        controller(HOME);
//    }
//}

//
//inline waypoints quadCopter::findAlive(list<waypoints> &TRAJ, node *nodes)
//{
//    list<waypoints>::iterator it;
//    it = TRAJ.begin();
//    if (TRAJ.size() <= 1)
//    {
//        return *it;
//    }
//    if (nodes->waypoint[it->index].alive)
//    {
//        return *it;
//    }
//    else
//    {
//        TRAJ.pop_front();
//        return findAlive(TRAJ, nodes);
//    }
//}
