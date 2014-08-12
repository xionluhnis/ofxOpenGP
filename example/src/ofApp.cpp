#include "ofApp.h"
#include <iostream>

//--------------------------------------------------------------
void ofApp::setup(){
  ofSetFrameRate(24);
  dispMode = 0;
  // ofEnableAntiAliasing();
  ofEnableBlendMode(OF_BLENDMODE_ALPHA);
  // ofEnableSmoothing();
  // voronoi.setMinDist(std::epsilon<float>());
  Surface_mesh surf;
  surf.read("data/bunny.obj");

  // convert using ofxOpenGP
  bool ok = ofxOpenGP::convert(surf, mesh);
  std::cout << "Loaded: " << (ok ? "OK" : "Failed!") << "\n";
}

//--------------------------------------------------------------
void ofApp::update(){
  
}

//--------------------------------------------------------------
void ofApp::draw(){
  // draw background
  ofColor centerColor = ofColor(85, 78, 68);
  ofColor edgeColor(0, 0, 0);
  ofBackgroundGradient(centerColor, edgeColor, OF_GRADIENT_CIRCULAR);

  cam.begin();
  switch(dispMode){
    case '1':
      glPointSize(5.0f);
      mesh.drawVertices();
      // mesh.draw(OF_MESH_POINTS);
      break;
    case '2':
      mesh.drawWireframe();
      // mesh.draw(OF_MESH_WIREFRAME);
      break;
    default:
      //mesh.draw(OF_MESH_FILL);
      mesh.drawFaces();
  }
  cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
  switch(key){
    case 'f':
      ofToggleFullscreen();
      break;
    case '1':
    case '2':
    case '3':
      dispMode = key;
      break;
  }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
}

