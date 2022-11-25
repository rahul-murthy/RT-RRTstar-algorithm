#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
#ifdef randomSeed
	ofSeedRandom(randomSeed);
#endif // randomSeed
#ifdef CLK
	auto start = std::chrono::steady_clock::now();
#endif // DEBUG
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofSetWindowTitle("Dynamic-obstacles");
	ofBackground(200,200,200,200);
	myfont.loadFont(OF_TTF_SANS, 10);
	//map = new Enviroment();
	//car.setup();


	float arrMazeWallWidth[numberOfmazewall] = {
		(float)0.6 * ofGetWidth(),		// wall #0
        (float)0.1 * ofGetWidth(),		// wall #1
        (float)0.2 * ofGetWidth(),		// wall #2
        (float)0.8 * ofGetWidth(),		// wall #3
        (float)0.2 * ofGetWidth(),		// wall #4
        (float)0.86 * ofGetWidth(),		// wall #5
        (float)0.72 * ofGetWidth(),		// wall #6
	};
	float arrMazeWallHeight[numberOfmazewall] = { 
		0,								// wall #0
        (float)0.45 * ofGetHeight(),	// wall #1
        (float)0.8 * ofGetHeight(),		// wall #2
        (float)0.4 * ofGetHeight(),		// wall #3
        (float)0.2 * ofGetHeight(),		// wall #4
        (float)0.1 * ofGetHeight(),		// wall #5
        (float)0.9 * ofGetHeight(),		// wall #6
	};

	float arrMazeWallX[numberOfmazewall] = {
		20,		// wall #0
		480,	// wall #1
		20,		// wall #2
		200,	// wall #3
		60,		// wall #4
		40,		// wall #5
		60,		// wall #6
	};

	float arrMazeWallY[numberOfmazewall] = {
		160,	// wall #0
		40,		// wall #1
		160,	// wall #2
		180,	// wall #3
		40,		// wall #4
		60,		// wall #5
		40,		// wall #6
	};

	obstacles *ob;
	for (int i = 0; i < numberOfmazewall; i++)
	{
		ofVec2f w;
		w.set(arrMazeWallWidth[i], arrMazeWallHeight[i]);
		wall = new maze(w, arrMazeWallX[i], arrMazeWallY[i]);
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
	ofVec2f arrMovObsLocation[numberOfmovObst] = { {90, 30} };
	ofVec2f arrMovObsVector[numberOfmovObst] = { {-1, 0} };
	
	for (unsigned int i = 0; i < numberOfmovObst; i++)
	{
		// moving obstacles
		OBST = new movingObst(arrMovObsLocation[i], arrMovObsVector[i]);
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
	if (car!= NULL) car->render();

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
	sprintf(time, "Update rate: %f", updateTime);
	myfont.drawString(time, ofGetWindowWidth() - 140, ofGetWindowHeight() - 755);
	sprintf(time, "Draw rate: %f", drawTime);
	myfont.drawString(time, ofGetWindowWidth() - 140, ofGetWindowHeight() - 740);

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

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	ofVec2f loc;
	loc.set(x, y);
	if (button == 0) {
		if (car != NULL) {
			map->targetSet(loc);
		}
	}
	else if (button == 2) {
		car = new Robot(loc);
		map = new Enviroment(car->getLocation());
	}
	else
	{

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
