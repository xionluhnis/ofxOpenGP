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
#include <limits>
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

    static void translate(Surface_mesh &mesh, const Vec3 &t) {
      Surface_mesh::Vertex_property<Vec3> points = mesh.get_vertex_property<Vec3>("v:point");
      Surface_mesh::Vertex_iterator vit, vend = mesh.vertices_end();
      for (vit = mesh.vertices_begin(); vit != vend; ++vit) {
        points[*vit] += t;
      }
    }

    static Vec3 centroid(Surface_mesh &mesh) {
      Surface_mesh::Vertex_property<Vec3> points = mesh.get_vertex_property<Vec3>("v:point");

      // compute centroid
      Vec3 centroid(0.0f, 0.0f, 0.0f);

      // copy all vertices
      Surface_mesh::Vertex_iterator vit, vend = mesh.vertices_end();
      for (vit = mesh.vertices_begin(); vit != vend; ++vit) {
        centroid += points[*vit];
      }
      centroid *= 1.0f / mesh.n_vertices();
      return centroid;
    }

    static void bbox(Surface_mesh &mesh, Vec3 &min, Vec3 &max){
      Surface_mesh::Vertex_property<Vec3> points = mesh.get_vertex_property<Vec3>("v:point");

      // compute bounding box
      Surface_mesh::Vertex_iterator vit = mesh.vertices_begin(), vend = mesh.vertices_end();
      min = max = points[*vit]; // set first vertex as min and maximum
      for (; vit != vend; ++vit) {
        Vec3 v = points[*vit];
        min.x = std::min(min.x, v.x); max.x = std::max(max.x, v.x);
        min.y = std::min(min.y, v.y); max.y = std::max(max.y, v.y);
        min.z = std::min(min.z, v.z); max.z = std::max(max.z, v.z);
      }
    }

    static void rescale(Surface_mesh &mesh, float scaleFactor){
      Surface_mesh::Vertex_property<Vec3> points = mesh.get_vertex_property<Vec3>("v:point");

      // rescale all vertices
      Surface_mesh::Vertex_iterator vit, vend = mesh.vertices_end();
      for (vit = mesh.vertices_begin(); vit != vend; ++vit) {
        points[*vit] *= scaleFactor;
      }
    }

    static void normalize(Surface_mesh &mesh){
      Vec3 a, b;
      bbox(mesh, a, b);

      // center in bbox
      translate(mesh, -(a + b) * 0.5f);

      // rescale to have bbox of max width 1
      Vec3 diag = b - a;
      float maxWidth = std::max(diag.x, std::max(diag.y, diag.z));
      rescale(mesh, 1.0f / maxWidth);
    }
};

