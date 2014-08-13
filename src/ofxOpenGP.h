/*
 * OpenFramework wrapper for OpenGP (Open Geometry Processing)
 * See http://opengp.github.io
 *
 * Released under the MIT License (MIT)
 *
 * Copyright (c) 2014 Alexandre Kaspar <alexandre.kaspar@alumni.epfl.ch>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <iostream>
#include "ofMain.h"
#include "OpenGP/Surface_mesh.h"

enum ofxMeshType {
  OFX_AUTO_MESH=0,
  OFX_TRIANGLE_MESH,
  OFX_QUAD_MESH,
  OFX_INVALID_MESH,
  OFX_UNKNOWN_MESH
};

using namespace opengp;

class ofxOpenGP {
  public:

    // conversion
    static bool convert(Surface_mesh &mesh, ofMesh &newMesh, ofxMeshType meshType = OFX_AUTO_MESH, float scale = 1e0f){
      if(meshType == OFX_AUTO_MESH){
        if(mesh.is_triangle_mesh()){
          meshType = OFX_TRIANGLE_MESH;
        } else if(mesh.is_quad_mesh()){
          meshType = OFX_QUAD_MESH;
        } else {
          // we triangulate it
          mesh.triangulate();
          meshType = OFX_TRIANGLE_MESH;
        }
      }
      int triIter;
      switch(meshType){
        case OFX_TRIANGLE_MESH:
          triIter = 1;
          break;
        case OFX_QUAD_MESH:
          triIter = 2;
          break;
        default:
          // WARNING: not supported!
          return false;
      }
      newMesh.setMode(OF_PRIMITIVE_TRIANGLES);

      // start from scratch
      newMesh.clear();

      // get (pre-defined) property storing vertex positions
      Surface_mesh::Vertex_property<Vec3> points = mesh.get_vertex_property<Vec3>("v:point");
      Surface_mesh::Vertex_property<Normal> normals = mesh.get_vertex_property<Normal>("v:normal");
      Surface_mesh::Vertex_property<Color> colors = mesh.get_vertex_property<Color>("v:color");

      // copy all vertices
      Surface_mesh::Vertex_iterator vit, vend = mesh.vertices_end();
      for (vit = mesh.vertices_begin(); vit != vend; ++vit) {
        // access point property like an array
        const Vec3 &v = points[*vit] * scale;
        newMesh.addVertex(ofVec3f(v.x, v.y, v.z));

        // TODO add texture coordinates
        // and other properties (mapping?)
        // normals
        if(normals){
          const Normal &n = normals[*vit];
          newMesh.addNormal(ofVec3f(n.x, n.y, n.z));
        }
        // colors
        if(colors){
          const Color &c = colors[*vit];
          newMesh.addColor(ofColor(c.x, c.y, c.z));
        }
      }

      // add all faces
      Surface_mesh::Face_iterator fit, fend = mesh.faces_end();
      for(fit = mesh.faces_begin(); fit != fend; ++fit) {
        // access vertices of the face
        Surface_mesh::Vertex_around_face_circulator vc = mesh.vertices(*fit);
        // triangle registration
        int tri = triIter;
        do {
          newMesh.addIndex((*vc).idx()); ++vc;
          newMesh.addIndex((*vc).idx()); ++vc;
          newMesh.addIndex((*vc).idx()); // not the last one
        } while (--tri > 0);
      }
      return true;
    }
};

