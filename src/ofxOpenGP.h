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

#include <cmath>
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

    /**
     * \brief Convert an OpenGP mesh into an ofMesh
     */
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

    ///////////////////////////////////////////////////////////////////////////
    ///// Mesh Normalization //////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
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

    ///////////////////////////////////////////////////////////////////////////
    ///// Properties //////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // property names
    static const std::string VORONOI_AREA = "v:voronoi_area";
    static const std::string COT_PAIR = "e:cot_pair";
    static const std::string UNIFORM_MEAN_CURVATURE = "v:uniform_mean_curvature";
    static const std::string MEAN_CURVATURE = "v:mean_curvature";
    static const std::string GAUSS_CURVATURE = "v:gauss_curvature";

    // property type
    typedef Surface_mesh::Vertex_property<float> VertexFloat;
    typedef Surface_mesh::Edge_property<float> EdgeFloat;

    /**
     * Voronoi area of each vertex
     */
    static VertexFloat voronoi_area(Surface_mesh &mesh, bool halfInv = true, const std::string &propName = VORONOI_AREA) {
      VertexFloat voro_area = mesh.add_vertex_property<float>(propName);

      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it){
        float area = 0.0f;
        Surface_mesh::Face_around_vertex_circulator fv_it, fv_end;
        fv_it = fv_end = mesh.faces(*v_it);
        do {
          Surface_mesh::Vertex_around_face_circulator fv_it = mesh.vertices(*vf_it);
          const Point& P = mesh.point(*fv_it);
          ++fv_it;
          const Point& Q = mesh.point(*fv_it);
          ++fv_it;
          const Point& R = mesh.point(*fv_it);
          area += cross(Q - P, R - P).norm() * 0.5f * 0.3333f;
        } while(++fv_it != fv_end);
        voro_area[*v_it] = halfInv ? 1.0 / (2.0 * area) : area;
      }
      return voro_area;
    }

    static EdgeFloat cot_pair(Surface_mesh &mesh, const std::string &propName = COT_PAIR) {
      typedef Surface_mesh::Vertex Vertex;
      typedef Surface_mesh::Halfedge Halfedge;
      EdgeFloat cot_pair = mesh.add_edge_property<float>(propName);

      Surface_mesh::Edge_iterator e_it, e_end = mesh.edges_end();
      for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
        float w = 0.0f;
        Halfedge h0 = mesh.halfedge(*e_it, 0);
        Vertex v0 = mesh.to_vertex(h0);
        Point p0 = mesh.point(v0);

        Halfedge h1 = mesh.halfedge(*e_it, 1);
        Vertex v1 = mesh.to_vertex(h1);
        Point p1 = mesh.point(v1);

        Halfedge h2 = mesh.next_halfedge(h0);
        Point p2 = mesh.point(mesh.to_vertex(h2));

        Point d0 = (p0 - p2).normalize();
        Point d1 = (p1 - p2).normalize();

        w += 1.0 / std::tan(std::acos(std::min(0.99f, std::max(-0.99f, d0.dot(d1) ))));

        h2 = mesh.next_halfedge(h1);
        p2 = mesh.point(mesh.to_vertex(h2));

        d0 = (p0 - p2).normalize();
        d1 = (p1 - p2).normalize();

        w += 1.0 / std::tan(std::acos(std::min(0.99f, std::max(-0.99f, d0.dot(d1) ))));
        w = std::max(0.0f, w);

        cot_pair[*e_it] = w;
      }
      return cot_pair;
    }

    /**
     * Uniform mean curvature H at each vertex
     */
    static VertexFloat uniform_mean_curvature(Surface_mesh &mesh, const std::string &propName = UNIFORM_MEAN_CURVATURE) {
      VertexFloat uniform_mean_curv = mesh.add_vertex_property<float>(propName);

      return uniform_mean_curv;
    }

    /**
     * Mean curvature H at each vertex
     */
    static VertexFloat mean_curvature(Surface_mesh &mesh, const std::string &propName = MEAN_CURVATURE) {
      VertexFloat mean_curv = mesh.add_vertex_property<float>(propName);

      return mean_curv;
    }

    /**
     * Gaussian curvature K at each vertex
     *
     * K = 1/A sum_i 2pi - phi_i
     */
    static VertexFloat gauss_curvature(Surface_mesh &mesh, bool halfInvArea = true, const std::string &propName = GAUSS_CURVATURE, const std::string &areaName = VORONOI_AREA) {
      VertexFloat gauss_curv = mesh.add_vertex_property<float>(propName);
      VertexFloat voro_area = mesh.get_vertex_property<float>(areaName);
      if(!voro_area) {
        voro_area = voronoi_area(mesh, halfInvArea, areaName);
      }

      // compute for all non-boundary vertices
      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
        if (!mesh.is_boundary(*v_it)) {
          float angle_sum = 0.0f;

          Surface_mesh::Vertex_around_vertex_circulator vv_it, vv_it2, vv_end;
          vv_it = vv_end = mesh.vertices(*v_it);
          do {
            vv_it2 = vv_it; // copy to get next vertex
            ++vv_it2;
            Point d0 = (mesh.point(*vv_it) - mesh.point(*v_it)).normalize();
            Point d1 = (mesh.point(*vv_it2) - mesh.point(*v_it)).normalize();
            float cos_angle = std::max(lb, std::min(ub, d0.dot(d1) ));
            angles += std::acos(cos_angle);
          } while(++vv_it != vv_end);
          float diff_sum = 2 * M_PI - angles;
          gauss_curv[*v_it] = halfInvArea ? diff_sum * 2.0f * voro_area[*v_it] : diff_sum / voro_area[*v_it];
        }
      }
      return gauss_curv;
    }

};
