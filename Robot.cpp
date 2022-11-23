#include "Robot.h"

void Robot::setup()
{
//        alive = true; mass = 5.0; scanRadius = sensorRadius; accuracy = accur;
        alive = true; mass = 5.0; scanRadius = 20; accuracy = accur;
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
}

void Robot::update()
{
    velocity += accelaration;
    velocity = (velocity.length() <= maxVelocity.length()) ? velocity : (velocity.normalized() *mVal);
    location += velocity;
    accelaration *= 0.0;
    pt.set(location.x, location.y);
    line.addVertex(pt);
}

void Robot::render()
{

    int r = sizeValue;
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
    ofRotate(ofRadToDeg(atan2(velocity.y, velocity.x) + PI / 2));
    ofFill();
    
    ofBeginShape();
    // Triangle Shape
//    ofVertex(0, -r * 2);
//    ofVertex(-r, r * 2);
//    ofVertex(r, r * 2);
    // Rectangle Shape
    ofVertex(-r, -r * 2);  // LR
    ofVertex(-r, r * 2);  // LF
    ofVertex(r, r * 2);  // RF
    ofVertex(r, -r * 2);  //RR
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
    ofVec2f LR, LF, RF, RR = getPoints();
}

ofVec2f Robot::getPoints()
{
    ofVec2f origin, LR, LF, RF, RR;
    int theta = ofRadToDeg(atan2(velocity.y, velocity.x) + PI / 2);
    
    ofVec2f tmp1, tmp2;
    tmp1.set(cos(theta), -sin(theta));
    tmp2.set(sin(theta), cos(theta));
    
    int r = sizeValue;
    // Set origin
    // Get 4 corners
    // Do the rotation metric on 4 points
    // Shift back to original spot by adding original x and y
    // Have the current location of the 4 corners
    
    // Get 4 corners around origin
//    origin.set(0.0, 0.0);
    LR.set(-r, -r*2);
    LF.set(-r, r*2);
    RF.set(r, r*2);
    RR.set(r, -r*2);
    
    // Do the rotation metric
    LR.set( (tmp1.x*LR.x+tmp1.y*LR.y),  (tmp2.x*LR.x+tmp2.y*LR.y) );
    LF.set( (tmp1.x*LF.x+tmp1.y*LF.y),  (tmp2.x*LF.x+tmp2.y*LF.y) );
    RF.set( (tmp1.x*RF.x+tmp1.y*RF.y),  (tmp2.x*RF.x+tmp2.y*RF.y) );
    RR.set( (tmp1.x*RR.x+tmp1.y*RR.y),  (tmp2.x*RR.x+tmp2.y*RR.y) );
    
    // Shift back
    LR.set(LR.x+location.x, LR.y+location.y);
    LF.set(LF.x+location.x, LF.y+location.y);
    RF.set(RF.x+location.x, RF.y+location.y);
    RR.set(RR.x+location.x, RR.y+location.y);
    
    return LR, LF, RF, RR;
}

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
