#include "SMP.h"
#include "RRTstar.h"
#include "InformedRRTstar.h"
#include "simulationParam.h"
#include "RT-RRTstar.h"

// Set current time as moving start time.
float SMP::movingStartTime = 0;
bool SMP::goalFound = false;
bool SMP::goal1Found = false;
bool SMP::goal2Found = false;
bool SMP::sampledInGoalRegion = false;
bool SMP::moveNow = false;
bool InformedRRTstar::usingInformedRRTstar = false;
bool RTRRTstar::goalDefined = false;

ofVec2f SMP::goal = { -1,-1 };
ofVec2f SMP::goal1;
ofVec2f SMP::goal2;
ofVec2f SMP::start;
Nodes* SMP::target = NULL;
Nodes* SMP::target1 = NULL;
Nodes* SMP::target2 = NULL;
Nodes* SMP::nextTarget = NULL;
Nodes* SMP::root = NULL;
std::set<Nodes*, nodes_compare> RTRRTstar::visited_set;

SMP::SMP()
{

}

void SMP::addNode(Nodes n, std::list<Nodes>& nodes)
{
	nodes.push_back(n);
	if (n.location.distance(goal1) < converge)
	{
		goal1Found = true;
		sampledInGoalRegion = true;
		target = &(nodes.back());
	}
	else if (n.location.distance(goal2) < converge)
	{
		goal2Found = true;
		sampledInGoalRegion = true;
		target = &(nodes.back());
	}
}

Nodes* SMP::nearestNode(Nodes n, std::list<Nodes>& nodes)
{
	double min_dist = n.location.squareDistance(nodes.front().location);
	Nodes* near_node = &(nodes.front());
	if (!nodes.empty())
	{
		std::list<Nodes>::iterator it = nodes.begin();
		while (it != nodes.end())
		{
			if (n.location.squareDistance((*it).location) < min_dist)
			{
				min_dist = n.location.squareDistance((*it).location);
				near_node = &(*it);
			}
			it++;
		}
		return near_node;
	}
	else
		return NULL;
}

Nodes* SMP::nearestNode(Nodes n, std::list<Nodes*>& nodes)
{
	double min_dist = n.location.squareDistance(nodes.front()->location);
	Nodes* near_node = nodes.front();
	if (!nodes.empty())
	{
		std::list<Nodes*>::iterator it = nodes.begin();
		while (it != nodes.end())
		{
			if (n.location.squareDistance((*it)->location) < min_dist)
			{
				min_dist = n.location.squareDistance((*it)->location);
				near_node = *it;
			}
			it++;
		}
		return near_node;
	}
	else
		return NULL;
}


Nodes SMP::sampler()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	Nodes new_node;
	new_node.location.x = x;
	new_node.location.y = y;
	return new_node;
}

bool SMP::checkCollision(Nodes n1, Nodes n2, list<obstacles*> obst)
{
	for (auto i : obst) {
#ifdef PathCollisionCheck
			if (i->isCollide(n1)) 	return false;
#else
			if (i->isCollide(n1.location, n2.location)) 	return false;
#endif
	}
	return true;
}

bool SMP::checkSample(Nodes n,  list<obstacles*> obst)
{
#ifdef rectangleRobot
	collisionRect *rec;
	for (auto i : obst) {
		rec = new collisionRect(n.location, n.LR, n.LF, n.RF, n.RR);

#ifdef predictMovement
		if (i->isInside(*rec, n.time)) return false;
		if (i->isMovingObst())
		{
			float timeDiffWindow = 5; 
			// should be proportionate to location diff of the robot and obstacle's predicted location at time t.
			// if it's too close, check more precisely
			//if (i->isInside(*rec, n.time + 2.5)) return false;
			//if (i->isInside(*rec, n.time + 5)) return false;
			//if ((n.time >= 2.5) && (i->isInside(*rec, n.time - 2.5))) return false;
			//if ((n.time >= 5) && (i->isInside(*rec, n.time - 5))) return false;
		}
#else
		if (i->isInside(*rec)) return false;
#endif
		delete rec;
	}
#else
	for (auto i : obst) {
			if (i->isInside(n.location)) return false;		
	}
#endif
	return true;
}


void RRTstar::nextIter(std::list<Nodes>& nodes,list<obstacles*> obst, Nodes* u_)
{
#if 0
	Nodes u;
	if (u_ == NULL)
		u = SMP::sampler();
	else
		u = *u_;

	Nodes* v = SMP::nearestNode(u, nodes);
	double dist = u.location.distance((*v).location);

	if (dist > epsilon)
	{
		float x_n = v->location.x + (u.location.x - v->location.x)  * epsilon / dist;
		float y_n = v->location.y + (u.location.y - v->location.y)  * epsilon / dist;
		u.location.x = x_n;
		u.location.y = y_n;
	}
	if (!SMP::checkSample(u, obst)) return;

	std::list<Nodes*> closestNeighbours;
	closestNeighbours = RRTstar::findClosestNeighbours(u, nodes);

	if (closestNeighbours.empty()) return;

	std::list<Nodes*>::iterator it = closestNeighbours.begin();
	std::list<Nodes*> safeNeighbours;
	while (it != closestNeighbours.end())
	{
		if (SMP::checkCollision(u, *(*it), obst))
			safeNeighbours.push_back(*it);
		it++;
	}
	if (safeNeighbours.empty()) return;

	it = safeNeighbours.begin();

	float minDist = (*it)->costToStart + u.location.distance((*it)->location);
	auto index = it;

	while (it != safeNeighbours.end())
	{
		float dist = (*it)->costToStart + u.location.distance((*it)->location);
		if (dist < minDist) {
			minDist = dist;
			index = it;
		}
		it++;
	}
	u.parent = *index;
	u.costToStart = minDist;

	SMP::addNode(u, nodes);

	safeNeighbours.remove(*index);

	// Bring the iterator to initial position again
	it = safeNeighbours.begin();
	while (it != safeNeighbours.end())
	{
		float dist = u.costToStart + u.location.distance((*it)->location);
		if ((*it)->costToStart > dist)
		{
			(*it)->prevParent = (*it)->parent;
			(*it)->parent = &(nodes.back());	/// must update the time value as well
			(*it)->costToStart = dist;
		}
		it++;
	}
#endif
}

std::list<Nodes*> RRTstar::findClosestNeighbours(Nodes u, std::list<Nodes>& nodes)
{
	std::list<Nodes*> closestNeighbours;
#if 0
	std::list<Nodes>::iterator it = nodes.begin();

	/*float rrtstarradius = std::sqrt((ofGetWindowWidth() * ofGetWindowHeight() * maxNeighbours) / (3.146 * nodes.size()));
	if (rrtstarradius < minDistClosestNode)
		rrtstarradius = minDistClosestNode;*/

	while (it != nodes.end())
	{
		float f = u.location.distance(it->location);
		if (f < rrtstarradius && f > 0.0001)
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
#endif
	return closestNeighbours;
}

void InformedRRTstar::nextIter(std::list<Nodes> &nodes, std::list<obstacles*> obst)
{
	if (sol_nodes.empty())
	{
		RRTstar::nextIter(nodes, obst);
	}
	else
	{
		float min_cost = sol_nodes.front()->costToStart;
		std::list<Nodes*>::iterator it = sol_nodes.begin();
		while (it != sol_nodes.end())
		{
			if ((*it)->costToStart < min_cost)
			{
				min_cost = (*it)->costToStart;
				SMP::target = *it;
			}
			it++;
		}
        Nodes tmp = InformedRRTstar::sample(min_cost);
		RRTstar::nextIter(nodes, obst, &tmp);
	}
	if (SMP::sampledInGoalRegion)
		sol_nodes.push_back(&nodes.back());
}

Nodes InformedRRTstar::sample(float c_max)
{
	float c_min = SMP::goal.distance(SMP::start);

	if (goal.x == -1 && goal.y == -1) { assert(false); }

	if (std::abs(c_max - c_min) < 100 && usingInformedRRTstar) //Putting a dummy value for now - Robot might not move for some configurations with this value
		SMP::moveNow = true; //TODO: The flag will be associated with time. Should turn on when the spcified time lapses

	ofVec2f x_centre = (SMP::start + SMP::goal) / 2;
	ofVec2f dir = SMP::goal - SMP::start;
	dir = dir.getNormalized();
	float angle = std::atan2(-dir.y, dir.x); //Frame is with y pointing downwards
	float r1 = c_max / 2;
	float r2 = std::sqrt(std::pow(c_max, 2) - std::pow(c_min, 2)) / 2;

	float x = ofRandom(-1, 1);
	float y = ofRandom(-1, 1);

	float x2 = x * r1 * std::cos(angle) + y * r2 * std::sin(angle);
	float y2 = -x * r1 * std::sin(angle) + y * r2 * std::cos(angle);

	ofVec2f rot_sample, rot_trans_sample;
	rot_sample.set(x2, y2);
	rot_trans_sample = rot_sample + x_centre;

	Nodes n;
	n.location.x = rot_trans_sample.x;
	n.location.y = rot_trans_sample.y;

	return n;
}

void RTRRTstar::nextIter(std::list<Nodes> &nodes, const std::list<obstacles*>& obst, Robot* agent)
{
	bool bIsPathObstructed = false;
	timeKeeper = ofGetElapsedTimef();
	expandAndRewire(nodes, obst);
	if (SMP::goalFound)
		bIsPathObstructed = updateNextBestPath();
	if (currPath.size() > 1 && agent->getLocation().distance(SMP::root->location) < 0.1)
	{
		//We have to pre-increment rather than post-increment
		Nodes* nextPoint = *((++currPath.begin())); //Change the DS for path to vector?
		changeRoot(nextPoint, nodes);

		RTRRTstar::visited_set.clear();
		pushedToRewireRoot.clear();
		rewireRoot.clear();
	}

	if (bIsPathObstructed)
	{
		SMP::goalFound = false;
	}
	closestNeighbours.clear();
}

void RTRRTstar::changeRoot(Nodes* nextPoint, std::list<Nodes>& nodes)
{
	nextPoint->children.push_back(SMP::root);
	nextPoint->parent = NULL;
	nextPoint->prevParent = NULL;
	nextPoint->costToStart = 0;

	SMP::root->parent = nextPoint;
	SMP::root->costToStart = SMP::root->location.distance(nextPoint->location);
	SMP::root = nextPoint;
}

void RTRRTstar::expandAndRewire(std::list<Nodes>& nodes, const std::list<obstacles*>& obst)
{
	Nodes* pu = new Nodes();
	Nodes u = sample();
	pu = &u;
	Nodes* v = RTRRTstar::getClosestNeighbour(*pu, nodes);
	double dist = (*pu).location.distance((*v).location);

	if (dist > epsilon)
	{
		float x_n = v->location.x + ((*pu).location.x - v->location.x)  * epsilon / dist;
		float y_n = v->location.y + ((*pu).location.y - v->location.y)  * epsilon / dist;
		(*pu).location.x = x_n;
		(*pu).location.y = y_n;
	}

#ifdef rectangleRobot
	// Set variables of Node u other than it's center location, using parent node's information.
	if (nodes.size() != 0) {
		InitNode((*pu), *v);
	}
#endif

	if (!SMP::checkSample((*pu), obst)) return;
	if (SMP::checkCollision((*pu), *v, obst))
	{
		if (this->closestNeighbours.size() < maxNeighbours)//u.location.distance(v->location) > minDistClosestNode)
		{
			this->addNode((*pu), v, nodes, obst);
		}
		else
		{
			this->rewireRand.push_front(v);
		}
		// rewireRandomNode(obst, nodes);
	}
	 rewireFromRoot(obst, nodes);
}

bool RTRRTstar::updateNextBestPath()
{
	bool bIsPathObstructed = false;
	std::list<Nodes*> updatedPath;
	Nodes *pathNode = target;
	if (SMP::goalFound) {
		std::list<float> tmpDeltaTimeList;
		do
		{
			float delta_time;
			// check if one of the nodes in the path got obstructed
			if (pathNode->alive == false) { bIsPathObstructed = true; }
			if (pathNode->parent != NULL)
			{
				float dist = pathNode->location.distance(pathNode->parent->location);
				delta_time = dist / mVal;
			}
			else
			{
				// if the pathNode is root, set time as 0.
				delta_time = 0;
			}
			tmpDeltaTimeList.push_back(delta_time);

			currPath.push_back(pathNode);
			pathNode = pathNode->parent;
		} while (pathNode != NULL);

		// reverse path
		currPath.reverse();

		// update the time values
		tmpDeltaTimeList.reverse();

		float accumulatedTime = 0;
		std::list<Nodes*>::iterator iterNode = currPath.begin();
		std::list<float>::iterator iterTime = tmpDeltaTimeList.begin();
		for (int i = 0; i < tmpDeltaTimeList.size(); i++)
		{
			accumulatedTime += (*iterTime);
			(*iterNode)->time = accumulatedTime;

			iterNode++;
			iterTime++;
		}

		return bIsPathObstructed;
	}
	else {
		if (!goalDefined)
			return bIsPathObstructed;
		Nodes* curr_node = SMP::root;
		while (!curr_node->children.empty())
		{
			std::list<Nodes*>::iterator it = curr_node->children.begin();
			Nodes* tempNode = curr_node->children.front();
			float cost_ = cost(tempNode);
			float minCost = cost_ + getHeuristic(*it);
			while (it != curr_node->children.end()) {
				cost_ = cost(*it);
				float cost_new = cost_ + getHeuristic(*it);
				if (cost_new < minCost) {
					minCost = cost_new;
					tempNode = *it;
				}
				it++;
			}
			updatedPath.push_back(tempNode);
			if (tempNode->children.empty() || cost(tempNode) == inf)
			{
				visited_set.insert(tempNode);
				break;
			}
			curr_node = tempNode;
		}
		if (currPath.size() == 0)
			currPath.push_back(SMP::root);

		if (updatedPath.back()->location.distance(SMP::goal1) < currPath.back()->location.distance(SMP::goal1))
			currPath = updatedPath;
	}

	return bIsPathObstructed;
}

Nodes RTRRTstar::sample()
{
	float rand_num = ofRandom(0, 1);

	if (rand_num > 1 - alpha && SMP::target != NULL)
	{
		float x = ofRandom(SMP::root->location.x, SMP::target->location.x);
		float y = ofRandom(SMP::root->location.y, SMP::target->location.y);
		Nodes new_node;
		new_node.location.x = x;
		new_node.location.y = y;
		return new_node;
	}
	else if (rand_num >= (1 - alpha) / beta && SMP::goalFound)
	{
		return InformedRRTstar::sample(cost(SMP::target));
	}
	else
	{
		return SMP::sampler();
	}
}

#ifdef rectangleRobot
void RTRRTstar::InitNode(Nodes &newNode, Nodes &closestNode)
{
	ofVec2f closestCenter = closestNode.location;
	ofVec2f newNodeCenter = newNode.location;
	ofVec2f velocity = { newNodeCenter.x - closestCenter.x, newNodeCenter.y - closestCenter.y };
	ofVec2f orientation = { newNodeCenter.x - closestCenter.x, newNodeCenter.y - closestCenter.y };
	newNode.parent = &closestNode;

	// openframework rotation
	float r = robotSizeValue;
	glm::vec4 _LR = { -2 * r, -r, 0, 1 };
	glm::vec4 _RR = { -2 * r, r, 0, 1 };
	glm::vec4 _RF = { 2 * r, r, 0, 1 };
	glm::vec4 _LF = { 2 * r, -r, 0, 1 };

	orientation = orientation.normalized();
	if (orientation.x == 0 && orientation.y == 0)
	{
		orientation = closestNode.orientation;
	}
	velocity = (velocity.normalized() *mVal);

	newNode.velocity = velocity;
	newNode.orientation = orientation;

	ofVec2f xAxis = { 1, 0 };
	// theta between the xAxis and orientation.
	float cos_theta = xAxis.dot(newNode.orientation);
	if (cos_theta > 1) { cos_theta = 1; }
	else if (cos_theta < -1) { cos_theta = -1; }
	float theta = ofRadToDeg(glm::acos(cos_theta));  // glm::acos() returns [0:PI]. 

	newNode.thetaXaxis = ofRadToDeg(atan2(orientation.y, orientation.x));

	// tell if the rotation should be done clockwise or counter clockwise
	float crossProduct = (xAxis.x * newNode.orientation.y) - (xAxis.y * newNode.orientation.x);

	if (crossProduct > 0) {
		// counter clockwise rotation. (+)
	}
	else if (crossProduct < 0) {
		// clockwise rotation (-)
		theta *= -1;
	}
	else {
		// assert(((theta >= 0) && (theta < 0.05)) || 
		//	((theta > 179.95) && (theta <= 180)));
	}

	glRotatef(theta, 0, 0, 1);

	_LR = { -2 * r, -r, 0, 1 };
	_RR = { -2 * r, r, 0, 1 };
	_RF = { 2 * r, r, 0, 1 };
	_LF = { 2 * r, -r, 0, 1 };

	auto modelMatrix = glm::inverse(ofGetCurrentViewMatrix()) * ofGetCurrentMatrix(OF_MATRIX_MODELVIEW);
	_LR = modelMatrix * _LR;
	_RR = modelMatrix * _RR;
	_RF = modelMatrix * _RF;
	_LF = modelMatrix * _LF;

	// translate to center location
	_LR = { _LR.x + newNodeCenter.x, _LR.y + newNodeCenter.y, 0, 1 };
	_RR = { _RR.x + newNodeCenter.x, _RR.y + newNodeCenter.y, 0, 1 };
	_RF = { _RF.x + newNodeCenter.x, _RF.y + newNodeCenter.y, 0, 1 };
	_LF = { _LF.x + newNodeCenter.x, _LF.y + newNodeCenter.y, 0, 1 };

	newNode.LR = { _LR.x, _LR.y };
	newNode.RR = { _RR.x, _RR.y };
	newNode.RF = { _RF.x, _RF.y };
	newNode.LF = { _LF.x, _LF.y };

	// d = 1/2 * v * t
	double dist = closestCenter.distance(newNodeCenter);
	double delta_t = dist / mVal;

	newNode.time = closestNode.time + delta_t;
}
#endif

Nodes* RTRRTstar::getClosestNeighbour(Nodes u, std::list<Nodes>& nodes) //Using all the nodes for the time being
{
	double min_dist = u.location.squareDistance(nodes.front().location);
	Nodes* near_node = &(nodes.front());
	std::list<Nodes>::iterator it = nodes.begin();

	/*float rrtstarradius = std::sqrt((ofGetWindowWidth() * ofGetWindowHeight() * maxNeighbours) / (3.146 * nodes.size()));
	if (rrtstarradius < minDistClosestNode)
		rrtstarradius = minDistClosestNode;*/

	while (it != nodes.end())
	{
		float dist = u.location.squareDistance((*it).location);
		if (dist < min_dist)
		{
			min_dist = dist;
			near_node = &(*it);
		}
		if (u.location.distance(it->location) < rrtstarradius) 
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
	return near_node;
}

void RTRRTstar::addNode(Nodes n, Nodes* closest, std::list<Nodes>& nodes, const std::list<obstacles*>& obst)
{
	Nodes* parent = closest;
	float c_min = cost(closest) + n.location.distance(closest->location);
	std::list<Nodes*>::iterator it = (this->closestNeighbours).begin();
	float c_new;
	while (it != closestNeighbours.end())
	{
		c_new = cost(*it) + n.location.distance((*it)->location);
		if (c_new < c_min && SMP::checkCollision(n, *(*it), obst))
		{
			c_min = c_new;
			parent = *it;
			n.costToStart = c_min;
		}
		it++;
	}
	n.parent = parent;
	nodes.push_back(n);
	parent->children.push_back(&(nodes.back()));

	//if (n.location.distance(SMP::goal1 < converge || (SMP::goal2 < converge))
	if (n.location.distance(SMP::goal1) < converge)
	{
		if (SMP::target1 == NULL || (SMP::target1 != NULL && SMP::target1->costToStart > n.costToStart))
		{
			SMP::target1 = &(nodes.back());
		}
		SMP::goal1Found = true;
		if (SMP::goal2Found == true) 
		{
			SMP::goalFound = true;
		}
	}
	else if (n.location.distance(SMP::goal2) < converge)
	{
		if (SMP::target2 == NULL || (SMP::target2 != NULL && SMP::target2->costToStart > n.costToStart))
		{
			SMP::target2 = &(nodes.back());
		}
		SMP::goal2Found = true;
		if (SMP::goal1Found == true) 
		{
			SMP::goalFound = true;
			// Set current time as moving start time.
		}
	}

	// select one of the two targets and assign that to the SMP::target
	if (SMP::goalFound == true)
	{
		std::list<Nodes*> target1Path;
		Nodes* pathNode = target1;
		double target1TotalCost = 0;
		// do while loop on both targets and get total distance of two paths.
		do
		{
			target1Path.push_back(pathNode);
			// get cost (educlidean dist)
			if (pathNode->parent != NULL)
			{
				target1TotalCost += pathNode->location.distance(pathNode->parent->location);
			}
			pathNode = pathNode->parent;
		} while (pathNode != NULL);

		std::list<Nodes*> target2Path;
		double target2TotalCost = 0;
		pathNode = target2;

		do
		{
			target2Path.push_back(pathNode);
			// get cost (educlidean dist)
			if (pathNode->parent != NULL)
			{
				target2TotalCost += pathNode->location.distance(pathNode->parent->location);
			}
			pathNode = pathNode->parent;
		} while (pathNode != NULL);
		
		// select one of the two targets and assign that to the SMP::target

		if (target1TotalCost < target2TotalCost)
		{
			goal = goal1;
			target = target1;
		}
		else
		{
			goal = goal2;
			target = target2;
		}

		// Set current time as moving start time.
		SMP::movingStartTime = ofGetElapsedTimef();
	}
	
	//TODO: Add the node to the Grid based/KD-Tree Data structure

	this->rewireRand.push_front(&(nodes.back()));
}

float RTRRTstar::cost(Nodes* node)
{
	bool badNode = false;
	float cost_ = 0;
	Nodes* curr = node;
	while (curr->parent != NULL)			// This is where the code gets trapped in an infinite loop!!!
	{
		// if (curr->parent == curr) { break; }
		if (curr->parent->costToStart == inf)
		{
			node->costToStart = inf;
			badNode = true;
			break;
		}
		cost_ += curr->location.distance(curr->parent->location);
		curr = curr->parent;
	}
	if (badNode)
		return inf;
	else
	{
		node->costToStart = cost_;
		return cost_;
	}
}

void RTRRTstar::rewireRandomNode(const list<obstacles*> &obst, std::list<Nodes> &nodes)
{
	while (!rewireRand.empty() && (ofGetElapsedTimef() - timeKeeper) < 0.5 * allowedTimeRewiring)
	{
		Nodes* Xr = rewireRand.front();
		rewireRand.pop_front();

		std::list<Nodes*> nearNodes = RRTstar::findClosestNeighbours(*Xr, nodes);
		std::list<Nodes*>::iterator it = nearNodes.begin();
		std::list<Nodes*> safeNeighbours;
		while (it != nearNodes.end())
		{
			if (SMP::checkCollision(*Xr, *(*it), obst))
				safeNeighbours.push_back(*it);
			it++;
		}
		if (safeNeighbours.empty()) continue;

		it = safeNeighbours.begin();
		float cost_ = cost(Xr);
		while (it != safeNeighbours.end())
		{
			// update the node based on parent node
			bool IsCollision = false;
			Nodes tmp = *(*it);
#if 01
			InitNode(tmp, *Xr);
			if (!SMP::checkSample(tmp, obst)) { IsCollision = true; }
			if (!IsCollision && SMP::checkCollision(tmp, *Xr, obst)) { IsCollision = true; }
#endif
			float oldCost = cost(*it);
			float newCost = cost_ + Xr->location.distance((*it)->location);
			if (!IsCollision && (newCost < oldCost))
			{
#if 01
				// update the node based on parent node
				InitNode(*(*it), *Xr);
#endif
				(*it)->prevParent = (*it)->parent;
				(*it)->parent->children.remove(*it);
				(*it)->parent = Xr;
#if 1
				float distToNewParent = (*it)->location.distance(Xr->location);
				(*it)->time = Xr->time + (distToNewParent / mVal);
#endif
				(*it)->costToStart = newCost;
				Xr->children.push_back(*it);
				rewireRand.push_back(*it);
			}
			it++;
		}
	}
}

void RTRRTstar::rewireFromRoot(const list<obstacles*> &obst, std::list<Nodes> &nodes) {

	if (rewireRoot.empty()) {
		rewireRoot.push_back(SMP::root);
	}

	while (!rewireRoot.empty() && (ofGetElapsedTimef() - timeKeeper) < allowedTimeRewiring) 
	{
		Nodes* Xs = rewireRoot.front();
		rewireRoot.pop_front();
		std::list<Nodes*> nearNeighbours;
		nearNeighbours = RRTstar::findClosestNeighbours(*Xs, nodes);

		std::list<Nodes*>::iterator it = nearNeighbours.begin();
		std::list<Nodes*> safeNeighbours;
		while (it != nearNeighbours.end())
		{
			if (SMP::checkCollision(*Xs, *(*it), obst))
				safeNeighbours.push_back(*it);
			it++;
		}
		if (safeNeighbours.empty()) continue;

		safeNeighbours.remove(Xs->parent);
		it = safeNeighbours.begin();
		while (it != safeNeighbours.end()) {

			// update the node based on parent node
			bool IsCollision = false;
#if 01
			Nodes tmp = *(*it);
			InitNode(tmp, *Xs);
			if (!SMP::checkSample(tmp, obst)) { IsCollision = true; }
			if (!IsCollision && SMP::checkCollision(tmp, *Xs, obst)) { IsCollision = true; }
#endif
			float oldCost = cost(*it);
			float newCost = cost(Xs) + Xs->location.distance((*it)->location);
			if (!IsCollision && (newCost < oldCost)) {
#if 01
				InitNode(*(*it), *Xs);
#endif
				(*it)->prevParent = (*it)->parent;
				(*it)->parent->children.remove(*it);
				(*it)->parent = Xs;
				(*it)->costToStart = newCost;
				Xs->children.push_back(*it);
			}
			//TODO: take care of restarting the queue part
			bool found = std::find(pushedToRewireRoot.begin(), pushedToRewireRoot.end(), (*it)) != pushedToRewireRoot.end();
			if (!found) {
				rewireRoot.push_back((*it));
				pushedToRewireRoot.push_back((*it));
			}
			it++;
		}
	}
}

float RTRRTstar::getHeuristic(Nodes* u) {
	if (visited_set.find(u) != visited_set.end())
		return inf;
	else
		if (goal.x == -1 && goal.y == -1) { assert(false); }
		return u->location.distance(SMP::goal);
}

//method not used
bool RTRRTstar::isPathToGoalAvailable()
{
	if (!SMP::goalFound)
		return false;

	std::list<Nodes*> tempPath = currPath;
	tempPath.reverse();
	Nodes* curr = tempPath.front();
	while (curr->parent != NULL)
	{
		if (curr->parent->costToStart == inf)
		{
			return false;
		}
		curr = curr->parent;
	}
	return true;
}

