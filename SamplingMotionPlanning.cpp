#include "SMP.h"
#include "RRTstar.h"
#include "InformedRRTstar.h"
#include "simulationParam.h"
#include "RT-RRTstar.h"

bool SMP::goalFound = false;
bool SMP::sampledInGoalRegion = false;
bool SMP::moveNow = false;
bool InformedRRTstar::usingInformedRRTstar = false;
bool RTRRTstar::goalDefined = false;

ofVec2f SMP::goal;
ofVec2f SMP::start;
Nodes* SMP::target = NULL;
Nodes* SMP::nextTarget = NULL;
Nodes* SMP::root = NULL;
std::set<Nodes*, nodes_compare> RTRRTstar::visited_set;

SMP::SMP()
{

}

void SMP::addNode(Nodes n, std::list<Nodes>& nodes)
{
	nodes.push_back(n);
	if (n.location.distance(goal) < converge)
	{
		goalFound = true;
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
			if (i->isCollide(n1.location, n2.location)) 	return false;
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
			(*it)->parent = &(nodes.back());
			(*it)->costToStart = dist;
		}
		it++;
	}

}

std::list<Nodes*> RRTstar::findClosestNeighbours(Nodes u, std::list<Nodes>& nodes)
{
	std::list<Nodes*> closestNeighbours;
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
	timeKeeper = ofGetElapsedTimef();
	expandAndRewire(nodes, obst);
	if (SMP::goalFound)
		updateNextBestPath();
	if (currPath.size() > 1 && agent->getLocation().distance(SMP::root->location) < 0.1)
	{
		//We have to pre-increment rather than post-increment
		Nodes* nextPoint = *((++currPath.begin())); //Change the DS for path to vector?
		changeRoot(nextPoint, nodes);

		RTRRTstar::visited_set.clear();
		pushedToRewireRoot.clear();
		rewireRoot.clear();
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
	// rewireFromRoot(obst, nodes);
}

void RTRRTstar::updateNextBestPath()
{
	std::list<Nodes*> updatedPath;
	Nodes *pathNode = target;
	if (SMP::goalFound) {
		do
		{
			currPath.push_back(pathNode);
			pathNode = pathNode->parent;
		} while (pathNode != NULL);
		currPath.reverse();
		return;
	}
	else {
		if (!goalDefined)
			return;
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

		if (updatedPath.back()->location.distance(SMP::goal) < currPath.back()->location.distance(SMP::goal))
			currPath = updatedPath;
	}
	
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
#if 0
	// manual rotation
#else
	// openframework rotation
	float r = robotSizeValue;
	glm::vec4 _LR = { -2 * r, -r, 0, 1 };
	glm::vec4 _RR = { -2 * r, r, 0, 1 };
	glm::vec4 _RF = { 2 * r, r, 0, 1 };
	glm::vec4 _LF = { 2 * r, -r, 0, 1 };
#endif
	orientation = orientation.normalized();
	if (orientation.x == 0 && orientation.y == 0)
	{
		orientation = closestNode.orientation;
	}
	velocity = (velocity.normalized() *mVal);

	//assert((orientation.x != 0) || (orientation.y != 0));

	newNode.velocity = velocity;
	newNode.orientation = orientation;

	// float theta = ofRadToDeg(atan2(velocity.x, velocity.y));
	// float r = robotSizeValue;
#if 0
	// manual rotation
	// theta diff between the closet node and the new node.
	float cos_theta = closestNode.orientation.dot(newNode.orientation);
	if (cos_theta > 1) { cos_theta = 1; }
	else if (cos_theta < -1) { cos_theta = -1; }
	float theta = ofRadToDeg(glm::acos(cos_theta));  // glm::acos() returns [0:PI]. 

	newNode.thetaXaxis = ofRadToDeg(atan2(orientation.y, orientation.x));

	// tell if the rotation should be done clockwise or counter clockwise
	float crossProduct = (closestNode.orientation.x * newNode.orientation.y) - (closestNode.orientation.y * newNode.orientation.x);
#else
	ofVec2f xAxis = { 1, 0 };
	// theta between the xAxis and orientation.
	float cos_theta = xAxis.dot(newNode.orientation);
	if (cos_theta > 1) { cos_theta = 1; }
	else if (cos_theta < -1) { cos_theta = -1; }
	float theta = ofRadToDeg(glm::acos(cos_theta));  // glm::acos() returns [0:PI]. 

	newNode.thetaXaxis = ofRadToDeg(atan2(orientation.y, orientation.x));

	// tell if the rotation should be done clockwise or counter clockwise
	float crossProduct = (xAxis.x * newNode.orientation.y) - (xAxis.y * newNode.orientation.x);
#endif

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
#if 0
	// calculate four vertices based on the closest node info
	////////////////////////////////////////////////
	newNode.LR = closestNode.LR - closestCenter;
	newNode.LF = closestNode.LF - closestCenter; 
	newNode.RF = closestNode.RF - closestCenter;
	newNode.RR = closestNode.RR - closestCenter;

	newNode.LR.rotate(theta);
	newNode.LF.rotate(theta);
	newNode.RF.rotate(theta);
	newNode.RR.rotate(theta);

	newNode.LR += newNodeCenter;
	newNode.LF += newNodeCenter;
	newNode.RF += newNodeCenter;
	newNode.RR += newNodeCenter;
#else
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

	_LR = { _LR.x + newNodeCenter.x, _LR.y + newNodeCenter.y, 0, 1 };
	_RR = { _RR.x + newNodeCenter.x, _RR.y + newNodeCenter.y, 0, 1 };
	_RF = { _RF.x + newNodeCenter.x, _RF.y + newNodeCenter.y, 0, 1 };
	_LF = { _LF.x + newNodeCenter.x, _LF.y + newNodeCenter.y, 0, 1 };

	newNode.LR = { _LR.x, _LR.y };
	newNode.RR = { _RR.x, _RR.y };
	newNode.RF = { _RF.x, _RF.y };
	newNode.LF = { _LF.x, _LF.y };
#endif

	// d = 1/2 * v * t
	float dist = closestCenter.distance(newNodeCenter);
	float delta_t = 2 * dist / mVal;

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

	if (n.location.distance(SMP::goal) < converge)
	{
		if (SMP::target == NULL || (SMP::target != NULL && SMP::target->costToStart > n.costToStart))
		{
			SMP::target = &(nodes.back());
		}
		SMP::goalFound = true;
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
		if (curr->parent == curr) { break; }
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
#if 0
			InitNode(tmp, *Xr);
			if (!SMP::checkSample(tmp, obst)) { IsCollision = true; }
			if (!IsCollision && SMP::checkCollision(tmp, *Xr, obst)) { IsCollision = true; }
#endif
			float oldCost = cost(*it);
			float newCost = cost_ + Xr->location.distance((*it)->location);
			if (!IsCollision && (newCost < oldCost))
			{
#if 0
				// update the node based on parent node
				InitNode(*(*it), *Xr);
#endif
				(*it)->prevParent = (*it)->parent;
				(*it)->parent->children.remove(*it);
				(*it)->parent = Xr;
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
#if 0
				Nodes tmp = *(*it);
				InitNode(tmp, *Xs);
				if (!SMP::checkSample(tmp, obst)) { IsCollision = true; }
				if (!IsCollision && SMP::checkCollision(tmp, *Xs, obst)) { IsCollision = true; }
#endif
				float oldCost = cost(*it);
				float newCost = cost(Xs) + Xs->location.distance((*it)->location);
				if (!IsCollision && (newCost < oldCost)) {
#if 0
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

