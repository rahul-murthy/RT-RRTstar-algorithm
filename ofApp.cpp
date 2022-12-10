#include "ofApp.h"
#include "CollisionCheck.h"

//--------------------------------------------------------------
void ofApp::setup() {
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
	InitialTime = start;
#endif // DEBUG
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(200,200,200,200);
	myfont.loadFont(OF_TTF_SANS, 10);
	//map = new Enviroment();
	//car.setup();


	float arrMazeWallX[numberOfmazewall] = {
		(float)0.6 * ofGetWidth(),		// wall #0
        (float)0.4 * ofGetWidth(),		// wall #1
        (float)0.2 * ofGetWidth(),		// wall #2
        (float)0.8 * ofGetWidth(),		// wall #3
        (float)0.2 * ofGetWidth(),		// wall #4
        (float)0.86 * ofGetWidth(),		// wall #5
        (float)0.72 * ofGetWidth(),		// wall #6
	};
	float arrMazeWallY[numberOfmazewall] = { 
		0,								// wall #0
        (float)0.45 * ofGetHeight(),	// wall #1
        (float)0.45 * ofGetHeight(),		// wall #2
        (float)0.4 * ofGetHeight(),		// wall #3
        (float)0.2 * ofGetHeight(),		// wall #4
        (float)0.1 * ofGetHeight(),		// wall #5
        (float)0.9 * ofGetHeight(),		// wall #6
	};

	float arrMazeWallWidth[numberOfmazewall] = {
		20,		// wall #0
		600,	// wall #1
		20,		// wall #2
		200,	// wall #3
		60,		// wall #4
		40,		// wall #5
		60,		// wall #6
	};

	float arrMazeWallHeight[numberOfmazewall] = {
		160,	// wall #0
		40,		// wall #1
		300,	// wall #2
		180,	// wall #3
		40,		// wall #4
		60,		// wall #5
		40,		// wall #6
	};

	obstacles *ob;
	for (int i = 0; i < 3/*numberOfmazewall*/; i++)
	{
		ofVec2f w;
		w.set(arrMazeWallX[i], arrMazeWallY[i]);
		wall = new maze(w, arrMazeWallWidth[i], arrMazeWallHeight[i]);
		ob = wall;
		obst.push_back(ob);
	}

#if (numberOfstaticObst != 0)
	// obstacles' info
	ofVec2f arrObsLocation[numberOfstaticObst] = { {(float)0.5 * ofGetWidth(), (float)0.7 * ofGetHeight()} };
	float arrObsRadius[numberOfstaticObst] = { 50 };

	for (unsigned int i = 0; i < numberOfstaticObst; i++)
	{
		// static obstacles
		ob = new obstacles(arrObsLocation[i], arrObsRadius[i]);
		obst.push_back(ob);
	}
#endif

#if (numberOfmovObst != 0)
	// moving obstacles' info
	ofVec2f arrMovObsLocation[numberOfmovObst] = { 
		{(float)400/*0.4 * ofGetWidth()*/, (float)260/*0.35 * ofGetHeight()*/},
		{(float)400/*0.4 * ofGetWidth()*/, (float)260/*0.35 * ofGetHeight()*/},
//        {(float)500/*0.4 * ofGetWidth()*/, (float)160/*0.35 * ofGetHeight()*/},
//        {(float)700/*0.4 * ofGetWidth()*/, (float)460/*0.35 * ofGetHeight()*/},
        
//        {(float)600/*0.4 * ofGetWidth()*/, (float)160/*0.35 * ofGetHeight()*/},
		{(float)0.9 * ofGetWidth(), (float)260/*0.35 * ofGetHeight()*/},
	};
    ofVec2f arrMovObsVector[numberOfmovObst] = { {-1, 0}, {0, -1}, {-1, 0} };
	
	for (unsigned int i = 0; i < numberOfmovObst; i++)
	{
		// moving obstacles
		OBST = new movingObst(arrMovObsLocation[i], arrMovObsVector[i], 50);
		obstacles *ob = OBST;
		obst.push_back(ob);
	}
#endif

	cout << "Obst size: " << obst.size() << endl;

#ifdef randomSeed
	std::cout << "RandomSeed:" << randomSeed << endl;
#endif

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	std::cout << std::endl << "Setup:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::update(){
	if (!updateFlag) return;
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG

#ifdef automatic
	for (auto i : obst) {
		i->move(obst);
		//cout << "location: " << i->loc() << "Radius: " << i->rad() << endl;
		//cout << i.getX() << "  " << i.getY() << endl;
	}
#endif // automatic

	if (map!= NULL) map->update(car,obst);
#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	/*std::cout << std::endl << "Update:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;*/
	updateTime = std::chrono::duration<double, std::milli>(end - start).count();
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::draw(){
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG

	for (auto i : obst) {
		i->render();
	}
	if (map != NULL) map->render();
	if (car != NULL) {
		if (car->isStartedMoving()) {
			InitialTime = std::chrono::steady_clock::now();
		}
		car->render();
	}

	char fpsStr[255]; // an array of chars
	ofSetColor({ 255,0,0 });
	sprintf(fpsStr, "Frame rate: %d", int(ofGetFrameRate()));
	myfont.drawString(fpsStr, ofGetWindowWidth() - 100, ofGetWindowHeight() - 25);
	if (map != NULL) {
		char numNode[255];
		sprintf(numNode, "Number of nodes: %d", int(map->numofnode()));
		myfont.drawString(numNode, ofGetWindowWidth() - 140, ofGetWindowHeight() - 10);
	}

#ifdef CLK
	auto end = std::chrono::steady_clock::now();
	/*std::cout << std::endl << "Draw:" << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;*/
	drawTime = std::chrono::duration<double, std::milli>(end - start).count();

	char time[255];
	//sprintf(time, "Update rate: %f", updateTime);
	//myfont.drawString(time, ofGetWindowWidth() - 140, ofGetWindowHeight() - 755);
	//sprintf(time, "Draw rate: %f", drawTime);
	//myfont.drawString(time, ofGetWindowWidth() - 140, ofGetWindowHeight() - 740);

	if (car != NULL)
	{
		// Robot Information
		//sprintf(time, "LR: (%f, %f)", car->getVertex(VertexType::LR).x, car->getVertex(VertexType::LR).y);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 725);
		//sprintf(time, "RR: (%f, %f)", car->getVertex(VertexType::RR).x, car->getVertex(VertexType::RR).y);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 710);
		//sprintf(time, "RF: (%f, %f)", car->getVertex(VertexType::RF).x, car->getVertex(VertexType::RF).y);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 695);
		//sprintf(time, "LF: (%f, %f)", car->getVertex(VertexType::LF).x, car->getVertex(VertexType::LF).y);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 680);
		//sprintf(time, "Center: (%f, %f)", car->getLocation().x, car->getLocation().y);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 665);
		//ofVec2f curOrientation = {
		//	(car->getVertex(VertexType::LF).x - car->getVertex(VertexType::LR).x),
		//	(car->getVertex(VertexType::LF).y - car->getVertex(VertexType::LR).y)
		//};
		//curOrientation = curOrientation.normalized();
		//sprintf(time, "Current Ori: <%f, %f>", curOrientation.x, curOrientation.y);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 650);

		//float theta = ofRadToDeg(atan2(curOrientation.y, curOrientation.x));
		//sprintf(time, "Theta axisX: %f", theta);
		//myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 635);


		//// Rendered position
		//sprintf(time, "Rendered Position");
		//myfont.drawString(time, 10, ofGetWindowHeight() - 740);
		//sprintf(time, "LR: (%f, %f)", car->render_LR.x, car->render_LR.y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 725);
		//sprintf(time, "RR: (%f, %f)", car->render_RR.x, car->render_RR.y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 710);
		//sprintf(time, "RF: (%f, %f)", car->render_RF.x, car->render_RF.y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 695);
		//sprintf(time, "LF: (%f, %f)", car->render_LF.x, car->render_LF.y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 680);
		//sprintf(time, "Center: (%f, %f)", car->render_Center.x, car->render_Center.y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 665);
		//sprintf(time, "Desired Ori(vel): <%f, %f>", car->getVelocity().x, car->getVelocity().y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 650);

		//curOrientation = {
		//	(car->render_LF.x - car->render_LR.x),
		//	(car->render_LF.y - car->render_LR.y)
		//};
		//curOrientation = curOrientation.normalized();
		//sprintf(time, "Desired Ori(pos): <%f, %f>", curOrientation.x, curOrientation.y);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 635);
		//theta = ofRadToDeg(atan2(curOrientation.y, curOrientation.x));
		//sprintf(time, "Theta axisX: %f", theta);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 620);

		//double passedTime = std::chrono::duration<double, std::milli>(end - InitialTime).count();
		//sprintf(time, "Current Time: %f", passedTime);
		//myfont.drawString(time, 10, ofGetWindowHeight() - 605);
        if (SMP::movingStartTime > 0) {
            sprintf(time, "Moving Start Time: %f", SMP::movingStartTime);
            myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 725);
            // Pause
        }
        if (SMP::GoalReachedTime > 0) {
            sprintf(time, "Goal Reached Time: %f", SMP::GoalReachedTime - SMP::movingStartTime);
            myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 700);
            // Pause
        }
        if (SMP::GoalReachedTime > 0) {
            sprintf(time, "Execution Time: %f", SMP::GoalReachedTime);
            myfont.drawString(time, ofGetWindowWidth() - 200, ofGetWindowHeight() - 675);
            // Pause
        }
        
        
//        myfont.drawString(time, 10, ofGetWindowHeight() - 590);
	}
#endif // DEBUG
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'p')
	{
		updateFlag = !updateFlag;
	}
	else if(key=='g')
	{
		map->grid = !map->grid;
	}
	else if (key == 'x') {
		ofImage img;
		img.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
		img.save("screenshot.png");
	}
#ifdef manual
	OBST->move(key);
#endif // manual

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

	//ofVec2f loc2;
	//loc2.set(x, y);
	//if (button == 1) {
	//		map->targetSet1(loc2);
	//}

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	ofVec2f loc1;
	ofVec2f loc2;
	loc1.set(x, y);
	loc2.set(x, y);
	if (button == 0) {
		if (car != NULL) {
			map->targetSet(loc1);
		}
	}
	else if (button == 2) {
		car = new Robot(loc1);
		map = new Enviroment(car->getLocation());
	}
	else if (button == 1) {
		if (car != NULL) {
			map->targetSet1(loc2);
		}

	else
	{

	}

	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
