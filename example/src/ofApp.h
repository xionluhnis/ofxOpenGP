#pragma once

#include "ofMain.h"
#include "ofxOpenGP.h"

using namespace opengp;

class ofApp : public ofBaseApp{
  public:
    void setup();
    void update();
    void draw();

    void keyPressed(int key);
    void mousePressed(int x, int y, int button);

  private:
    int dispMode;
    ofMesh mesh;
    ofEasyCam cam;
};
