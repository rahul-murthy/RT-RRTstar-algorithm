#include "CollisionCheck.h"

// private functions
ofVec2f CollisionCheck::_getNormPerpendicularAxis(ofVec2f &curVertex, ofVec2f &nextVertex)
{
	float axis_x = -(nextVertex.y - curVertex.y);
	float axis_y = (nextVertex.x - curVertex.x);
	float magnitude = hypot(axis_x, axis_y);

	ofVec2f normalizedAxis = { (axis_x / magnitude), (axis_y / magnitude) };

	return normalizedAxis;
}

ofVec2f CollisionCheck::_getNormalizedVector(ofVec2f &curPt, ofVec2f &nextPt)
{
	float axis_x = (nextPt.x - curPt.x);
	float axis_y = (nextPt.y - curPt.y);
	float magnitude = hypot(axis_x, axis_y);

	ofVec2f normalizedAxis = { (axis_x / magnitude), (axis_y / magnitude) };

	return normalizedAxis;
}

void CollisionCheck::_computeProjections(collisionRect &rect1, collisionRect &rect2, ofVec2f &normalizedAxis, vector <float> &projection1, vector <float> &projection2)
{
	projection1.clear();
	projection2.clear();

	for (int i = (int)VertexStart; i < (int)VertexCount; i++)
	{
		ofVec2f rect1Vertex = rect1.Vertex[i];
		ofVec2f rect2Vertex = rect2.Vertex[i];

		float nDotProduct1 = normalizedAxis.dot(rect1Vertex);
		float nDotProduct2 = normalizedAxis.dot(rect2Vertex);
	
		ofVec2f proj1Vertex = nDotProduct1 * normalizedAxis;
		ofVec2f proj2Vertex = nDotProduct2 * normalizedAxis;

		float pt_projection1 = normalizedAxis.dot(proj1Vertex);
		float pt_projection2 = normalizedAxis.dot(proj2Vertex);

		projection1.push_back(pt_projection1);	// four vertices of rect1 projected on the normalized axis
		projection2.push_back(pt_projection2);	// four vertices of rect2 projected on the normalized axis
	}
}

void CollisionCheck::_computeProjections(collisionRect &rect, collisionCircle &circle, ofVec2f &normalizedAxis, vector <float> &projection1, vector <float> &projection2)
{
	projection1.clear();
	projection2.clear();

	// project the vertices of the rectangle
	for (int i = (int)VertexStart; i < (int)VertexCount; i++)
	{
		ofVec2f rectVertex = rect.Vertex[i];
		float nDotProduct1 = normalizedAxis.dot(rectVertex);
		ofVec2f proj1Vertex = nDotProduct1 * normalizedAxis;
		float pt_projection1 = normalizedAxis.dot(proj1Vertex);

		projection1.push_back(pt_projection1);	// four vertices of rect1 projected on the normalized axis
	}

	// project the circle's two points
	ofVec2f CirclePt1 = circle.center + (circle.radius * normalizedAxis);
	ofVec2f CirclePt2 = circle.center - (circle.radius * normalizedAxis);

	float nDotProduct1 = normalizedAxis.dot(CirclePt1);
	float nDotProduct2 = normalizedAxis.dot(CirclePt2);

	ofVec2f projPt1 = nDotProduct1 * normalizedAxis;
	ofVec2f projPt2 = nDotProduct2 * normalizedAxis;

	float Pt1_projection = normalizedAxis.dot(projPt1);
	float Pt2_projection = normalizedAxis.dot(projPt2);

	projection2.push_back(Pt1_projection);	
	projection2.push_back(Pt2_projection);	// two points of the circle projected on the normalized axis
}

bool CollisionCheck::_IsOverlapping(vector <float> &projection1, vector <float> &projection2)
{
	const float max_projection_1 = *std::max_element(projection1.begin(), projection1.end());
	const float min_projection_1 = *std::min_element(projection1.begin(), projection1.end());
	const float max_projection_2 = *std::max_element(projection2.begin(), projection2.end());
	const float min_projection_2 = *std::min_element(projection2.begin(), projection2.end());

	// True if projection overlaps but does not necessarily mean the polygons are intersecting yet
	return !((max_projection_1 < min_projection_2) || (max_projection_2 < min_projection_1));
}

// public functions
float CollisionCheck::getEucDist(ofVec2f a, ofVec2f b)
{
	return std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));
}

void CollisionCheck::rotateRectToTarget(collisionRect &rect, ofVec2f &targetOrientation) 
{
	// openframework rotation
	float r = rect.sizeVal;
	assert(r == 20);

	glm::vec4 _LR = { -2 * r, -r, 0, 1 };
	glm::vec4 _RR = { -2 * r, r, 0, 1 };
	glm::vec4 _RF = { 2 * r, r, 0, 1 };
	glm::vec4 _LF = { 2 * r, -r, 0, 1 };

	ofVec2f orientation = targetOrientation.normalized();
	if (orientation.x == 0 && orientation.y == 0)
	{
		assert(false);
		orientation = { 1,0 };
	}

	ofVec2f xAxis = { 1, 0 };
	// theta between the xAxis and orientation.
	float cos_theta = xAxis.dot(orientation);
	if (cos_theta > 1) { cos_theta = 1; }
	else if (cos_theta < -1) { cos_theta = -1; }
	float theta = ofRadToDeg(glm::acos(cos_theta));  // glm::acos() returns [0:PI]. 

	// tell if the rotation should be done clockwise or counter clockwise
	float crossProduct = (xAxis.x * orientation.y) - (xAxis.y * orientation.x);
	
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

	auto modelMatrix = glm::inverse(ofGetCurrentViewMatrix()) * ofGetCurrentMatrix(OF_MATRIX_MODELVIEW);
	_LR = modelMatrix * _LR;
	_RR = modelMatrix * _RR;
	_RF = modelMatrix * _RF;
	_LF = modelMatrix * _LF;

	// translate to center location
	_LR = { _LR.x + rect.center.x, _LR.y + rect.center.y, 0, 1 };
	_RR = { _RR.x + rect.center.x, _RR.y + rect.center.y, 0, 1 };
	_RF = { _RF.x + rect.center.x, _RF.y + rect.center.y, 0, 1 };
	_LF = { _LF.x + rect.center.x, _LF.y + rect.center.y, 0, 1 };

	// update rectangle
	rect.Vertex[LR] = { _LR.x, _LR.y };
	rect.Vertex[RR] = { _RR.x, _RR.y };
	rect.Vertex[RF] = { _RF.x, _RF.y };
	rect.Vertex[LF] = { _LF.x, _LF.y };
}


ofVec2f CollisionCheck::rotatedPoint(ofVec2f coordinate, float degree, ofVec2f origin)
{
	float oldX = coordinate.x;
	float oldY = coordinate.y;
	float s = sin(degree);
	float c = cos(degree);
	ofVec2f p = { 0,0 };

	// translate point back to origin:
	p.x -= origin.x;
	p.y -= origin.y;

	// rotate point
	float xnew = p.x * c - p.y * s;
	float ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + origin.x;
	p.y = ynew + origin.y;

	return p;
}

bool CollisionCheck::IsCollision_RectToRect(collisionRect rect1, collisionRect rect2) {
	// <Separating axis theorem>
	// https://github.com/winstxnhdw/2d-separating-axis-theorem/blob/master/separating_axis.hpp

	vector<float> projection1;
	vector<float> projection2;
	projection1.reserve(VertexCount); // four vertices
	projection2.reserve(VertexCount); // four vertices

	for (int i = (int)VertexStart; i < (int)(VertexCount / 2); i++)	// since we are checking collision of rectangles, other 2 axis are parallel to the first two axises 
	{
		ofVec2f curVertex = rect1.Vertex[i];
		ofVec2f nextVertex = rect1.Vertex[(i + 1) % VertexCount];
		ofVec2f normalizedAxis = _getNormPerpendicularAxis(curVertex, nextVertex);
		_computeProjections(rect1, rect2, normalizedAxis, projection1, projection2);

		if (_IsOverlapping(projection1, projection2) == false) { return false; }
	}

	for (int i = (int)VertexStart; i < (int)(VertexCount / 2); i++)	// since we are checking collision of rectangles, other 2 axis are parallel to the first two axises 
	{
		ofVec2f curVertex = rect2.Vertex[i];
		ofVec2f nextVertex = rect2.Vertex[(i + 1) % VertexCount];
		ofVec2f normalizedAxis = _getNormPerpendicularAxis(curVertex, nextVertex);
		_computeProjections(rect1, rect2, normalizedAxis, projection1, projection2);
		
		if (_IsOverlapping(projection1, projection2) == false) { return false; }
	}
	return true;	// if all projection results werer "true", the two rectangles are in collision.
}

bool CollisionCheck::IsCollision_RecToCircle(collisionRect rect, collisionCircle circle) {
	// using separation axis theorem on rectangel to circle.
	// https://www.youtube.com/watch?v=vWs33LVrs74 (2:00)

	vector<float> projection1;
	vector<float> projection2;
	projection1.reserve(VertexCount); // four vertices
	projection2.reserve(VertexCount); // four vertices

	for (int i = (int)VertexStart; i < (int)(VertexCount / 2); i++)	// since we are checking collision of rectangles, other 2 axis are parallel to the first two axises 
	{
		ofVec2f curVertex = rect.Vertex[i];
		ofVec2f nextVertex = rect.Vertex[(i + 1) % VertexCount];
		ofVec2f normalizedAxis = _getNormPerpendicularAxis(curVertex, nextVertex);
		_computeProjections(rect, circle, normalizedAxis, projection1, projection2);

		if (_IsOverlapping(projection1, projection2) == false) { return false; }
	}

	// one more projection from circle center to closest vertex of the rectangle!!!!!!!!!!!!
	ofVec2f closestVertex;
	float minDist = FLT_MAX;
	// get closestVertex
	for (int i = (int)VertexStart; i < (int)VertexCount; i++) 
	{
		float curDist = rect.Vertex[i].distance(circle.center);
		if (curDist < minDist) {
			minDist = curDist; 
			closestVertex = rect.Vertex[i];
		}
	}
	ofVec2f normalizedVector = _getNormalizedVector(circle.center, closestVertex);
	_computeProjections(rect, circle, normalizedVector, projection1, projection2);
	if (_IsOverlapping(projection1, projection2) == false) { return false; }

	return true;
}
bool CollisionCheck::IsCollision_CircleToCircle(collisionCircle circle1, collisionCircle circle2) {
	// get euclidean distance
	float dist = getEucDist(circle1.center, circle2.center);
	 return (dist < circle1.radius + circle2.radius);
}