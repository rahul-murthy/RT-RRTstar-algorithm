#pragma once

#include "ofMain.h"
#include "Enviroment.h"
//#include "Robot.h"

#define _CRT_SECURE_NO_WARNINGS
#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
	private:
		ofTrueTypeFont myfont;
		bool updateFlag = true;
		Enviroment *map;
		Robot *car;
		std::list<obstacles*> obst;
		movingObst *OBST;
		maze *wall;
		std::chrono::time_point<std::chrono::steady_clock> InitialTime;

		double updateTime = 0, drawTime = 0;
};
