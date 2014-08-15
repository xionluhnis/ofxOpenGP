#pragma once

#include "ofMain.h"
#include "ofxOpenGP.h"

using namespace opengp;

enum ogpMeshProperty {
  OGP_NONE = 0,
  OGP_UNIFORM_MEAN_CURVATURE,
  OGP_MEAN_CURVATURE,
  OGP_GAUSS_CURVATURE,
  OGP_K1,
  OGP_K2,
  OGP_VORONOI_AREA
};

class ofApp : public ofBaseApp{
  public:

    void setup();
    void update();
    void draw();

    void keyPressed(int key);
    void mousePressed(int x, int y, int button);

  private:
    bool help;
    ogpMeshProperty prop;
    bool propChanged;
    Surface_mesh mesh;
    ofVboMesh dispMesh;
    ofEasyCam cam;
    ofLight light;
    string path;
    bool pathChanged;
};
