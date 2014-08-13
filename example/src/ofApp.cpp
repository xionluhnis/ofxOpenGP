#include "ofApp.h"
#include <limits>
#include <iostream>

//--------------------------------------------------------------
void ofApp::setup(){
  ofSetFrameRate(24);
  dispMode = 0;
  showNormals = false;
  transparent = false;
  // ofEnableAntiAliasing();
  ofEnableDepthTest(); //make sure we test depth for 3d
  ofSetVerticalSync(true);
  ofEnableLighting();
  // ofEnableAlphaBlending();
  // ofEnableBlendMode(OF_BLENDMODE_ALPHA);
  ofEnableSmoothing();
  path = "data/bunny.obj";
  pathChanged = true;

  // light the scene to show off why normals are important
  light.enable();
  light.setPointLight();
  light.setPosition(0, 0, 300);
}

//--------------------------------------------------------------
void ofApp::update(){
  if(pathChanged){
    pathChanged = false;
    // mesh
    Surface_mesh surf;
    if(!surf.read(path)){
      std::cerr << "Couldn't read " << path << "\n";
      return;
    }
    surf.update_vertex_normals();
    surf.property_stats();

    // pre-processing
    ofxOpenGP::normalize(surf);

    // convert using ofxOpenGP
    float scale = 0.5f * std::min(ofGetWidth(), ofGetHeight());
    bool ok = ofxOpenGP::convert(surf, mesh, OFX_AUTO_MESH, scale);
    std::cout << "Loaded " << path << ": " << (ok ? "OK" : "Failed!") << "\n";
  }
}

//--------------------------------------------------------------
void ofApp::draw(){
  ofEnableLighting();
  // draw background
  ofColor centerColor = ofColor(85, 78, 68);
  ofColor edgeColor(0, 0, 0);
  ofBackgroundGradient(centerColor, edgeColor, OF_GRADIENT_CIRCULAR);

  cam.begin();
  ofSetColor(255, 255, 255, transparent ? 70 : 255);
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
      // mesh.drawFaces();
      mesh.draw();
  }
  // draw our normals, and show that they are perpendicular to the vector from the center to the vertex
  if(showNormals){
    vector<ofVec3f> n = mesh.getNormals();
    vector<ofVec3f> v = mesh.getVertices();
    float normalLength = 10.;
    ofDisableLighting();
    ofSetColor(0,0,0,70);
    for(unsigned int i=0; i < n.size() ;i++){
      ofLine(v[i].x,v[i].y,v[i].z,
          v[i].x+n[i].x*normalLength,v[i].y+n[i].y*normalLength,v[i].z+n[i].z*normalLength);
      ofLine(.98*v[i].x,.98*v[i].y,.98*v[i].z,
          .98*v[i].x+n[i].x*normalLength*.2,.98*v[i].y+n[i].y*normalLength*.2,.98*v[i].z+n[i].z*normalLength*.2);
      ofLine(.98*v[i].x+n[i].x*normalLength*.2,.98*v[i].y+n[i].y*normalLength*.2,.98*v[i].z+n[i].z*normalLength*.2,
          v[i].x+n[i].x*normalLength*.2,v[i].y+n[i].y*normalLength*.2,v[i].z+n[i].z*normalLength*.2);
    }
  }
  cam.end();

  // text
  ofSetColor(255,255,255);
  int ypos = 50;
  ofDrawBitmapString("<f> Toggle fullscreen", 50, ypos); ypos += 20;
  ofDrawBitmapString("<n> Toggle normals", 50, ypos); ypos += 20;
  ofDrawBitmapString("<t> Toggle transparence", 50, ypos); ypos += 20;
  ofDrawBitmapString("<1-3> Display mode", 50, ypos); ypos += 20;
  ofDrawBitmapString("---", 50, ypos); ypos += 20;
  ofDrawBitmapString(ofFilePath::getBaseName(path), 50, ypos); ypos += 20;
  // ofDrawBitmapString("light", cam.worldToScreen(light.getGlobalPosition()) + ofPoint(10,0));
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
    case 'n':
      showNormals = !showNormals;
      break;
    case 't':
      transparent = !transparent;
      break;
    case ' ':
      {
        ofFile file(ofToDataPath(path));
        ofFileDialogResult res = ofSystemLoadDialog("Chose mesh file (.obj, .off, .ply)", false, file.getEnclosingDirectory());
        path = res.getPath();
        pathChanged = true;
      }
      break;
  }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
}

