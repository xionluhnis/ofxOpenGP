#include "ofApp.h"
#include <limits>
#include <iostream>

//--------------------------------------------------------------
void ofApp::setup(){
  ofSetFrameRate(24);
  help = true;
  prop = OGP_NONE;
  propChanged = true;
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
  // load new mesh
  if(pathChanged){
    pathChanged = false;
    propChanged = true;
    // mesh
    mesh.clear();
    if(!mesh.read(path)){
      std::cerr << "Couldn't read " << path << "\n";
      return;
    }
    // triangulate mesh if not triangle-based
    if(!mesh.is_triangle_mesh()){
      mesh.triangulate();
    }
    // update vertices
    mesh.update_vertex_normals();

    // pre-processing
    ofxOpenGP::normalize(mesh);
    std::cout << "Loaded " << path << "\n";
  }

  // compute new property mapping
  if(propChanged){
    propChanged = false;
    // conversion settings
    // - always with triangle meshes
    ofxOpenGP::Settings settings = OFX_TRIANGLE_MESH;

    Surface_mesh::Vertex_property<float> vdata;
    Surface_mesh::Face_property<float> fdata;
    switch(prop){
      case OGP_UNIFORM_MEAN_CURVATURE:
        vdata = ofxOpenGP::uniform_mean_curvature(mesh);
        break;
      case OGP_MEAN_CURVATURE:
        vdata = ofxOpenGP::mean_curvature(mesh);
        break;
      case OGP_GAUSS_CURVATURE:
        vdata = ofxOpenGP::gauss_curvature(mesh);
        break;
      case OGP_VORONOI_AREA:
        vdata = ofxOpenGP::voronoi_area(mesh);
        break;
      case OGP_TRIANGLE_QUALITY:
        fdata = ofxOpenGP::circum_radius_to_min_edge(mesh);
        break;
      default:
        settings << OFX_NO_COLORS;
        break;
    }

    // map value to color between blue (min), green (mean) and red (max).
    Surface_mesh::Face_property<Color> fc = mesh.get_face_property<Color>("f:color");
    Surface_mesh::Vertex_property<Color> vc = mesh.get_vertex_property<Color>("v:color");
    if(vdata){
      ofxOpenGP::property_to_color(mesh, vdata);
      settings << OFX_VERTEX_COLORS;
    } else if(fdata){
      ofxOpenGP::property_to_color(mesh, fdata);
      settings << OFX_FACE_COLORS;
    } else {
      // no color (white)
      settings << OFX_NO_COLORS;
    }

    // convert using ofxOpenGP
    float scale = 0.5f * std::min(ofGetWidth(), ofGetHeight());
    ofxOpenGP::convert(mesh, dispMesh, settings << scale);
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
  ofSetColor(255, 255, 255, 255);
  // display mesh
  //mesh.draw(OF_MESH_FILL);
  // mesh.drawFaces();
  dispMesh.draw();

  // display edges?
  ofSetColor(100, 100, 100, 70);
  dispMesh.disableColors();
  dispMesh.drawWireframe();
  dispMesh.enableColors();
  // mesh.draw(OF_MESH_WIREFRAME);
  cam.end();

  // text
  if(help){
    ofSetColor(255,255,255);
    int ypos = 50;
    ofDrawBitmapString("<f> Toggle fullscreen", 50, ypos); ypos += 20;
    ofDrawBitmapString("<h> Toggle this help", 50, ypos); ypos += 20;
    ofDrawBitmapString("<1-7> Property to show", 50, ypos); ypos += 20;
    ofDrawBitmapString("---", 50, ypos); ypos += 20;
    ofDrawBitmapString(ofFilePath::getBaseName(path), 50, ypos); ypos += 20;
    switch(prop){
      case OGP_UNIFORM_MEAN_CURVATURE:
        ofDrawBitmapString("UNIFORM_MEAN_CURVATURE", 50, ypos); ypos += 20;
        break;
      case OGP_MEAN_CURVATURE:
        ofDrawBitmapString("MEAN_CURVATURE", 50, ypos); ypos += 20;
        break;
      case OGP_GAUSS_CURVATURE:
        ofDrawBitmapString("GAUSS_CURVATURE", 50, ypos); ypos += 20;
        break;
      case OGP_K1:
        ofDrawBitmapString("K1", 50, ypos); ypos += 20;
        break;
      case OGP_K2:
        ofDrawBitmapString("K2", 50, ypos); ypos += 20;
        break;
      case OGP_VORONOI_AREA:
        ofDrawBitmapString("VORONOI_AREA", 50, ypos); ypos += 20;
        break;
      case OGP_TRIANGLE_QUALITY:
        ofDrawBitmapString("TRIANGLE_QUAL", 50, ypos); ypos += 20;
        break;
    }
  }
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
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
      {
        ogpMeshProperty newProp = ogpMeshProperty(key - '1');
        if(newProp != prop){
          prop = newProp;
          propChanged = true;
        }
      }
      break;
    case 'h':
      help = !help;
      break;
    case ' ':
      {
        ofFile file(ofToDataPath(path));
        ofFileDialogResult res = ofSystemLoadDialog("Chose mesh file (.obj, .off, .ply)", false, file.getEnclosingDirectory());
        if(!res.bSuccess) return;
        if(!ofFile(res.getPath()).canRead()) return;
        path = res.getPath();
        pathChanged = true;
      }
      break;
  }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
}

