#pragma once

#include "ofMain.h"
#include "ofxOpenGP.h"

using namespace opengp;

typedef unsigned int Flag;

class ofApp : public ofBaseApp{
  public:

    // Display mode
    static const Flag DISP_MODE_POINTS = 1 << 0;
    static const Flag DISP_MODE_WIRE   = 1 << 1;
    static const Flag DISP_MODE_FILL   = 1 << 2;
    // Switches
    static const Flag DISP_NORMALS     = 1 << 3;
    static const Flag DISP_TRANSPARENT = 1 << 4;
    static const Flag DISP_HELP        = 1 << 5;

    void setup();
    void update();
    void draw();

    void keyPressed(int key);
    void mousePressed(int x, int y, int button);

  private:
    Flag dispState;
    ofVboMesh mesh;
    ofEasyCam cam;
    ofLight light;
    string path;
    bool pathChanged;

  private:
    inline void toggle(Flag flag){
      dispState ^= flag;
    }
    inline bool has(Flag flag){
      return dispState & flag;
    }
    inline void enable(Flag flag){
      dispState |= flag;
    }
    inline void disable(Flag flag){
      dispState &= ~flag;
    }
};
