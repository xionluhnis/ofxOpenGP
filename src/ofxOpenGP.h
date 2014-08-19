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

#include <algorithm>
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

enum ofxColorType {
  OFX_AUTO_COLORS=0,
  OFX_VERTEX_COLORS,
  OFX_FACE_COLORS,
  OFX_NO_COLORS
};

class ofxOpenGP {
  public:

    typedef opengp::Surface_mesh Surface_mesh;
    typedef opengp::Vec3 Vec3;
    typedef opengp::Point Point;
    typedef opengp::Normal Normal;
    typedef opengp::Color Color;

    /**
     * Setting container
     */
    struct Settings {
      ofxMeshType meshType;
      ofxColorType colorType;
      float meshScaling;

      Settings() : meshType(OFX_AUTO_MESH), colorType(OFX_AUTO_COLORS), meshScaling(1.0f) {}
      // implicit conversions
      Settings(ofxMeshType type) : meshType(type), colorType(OFX_AUTO_COLORS), meshScaling(1.0f) {}
      Settings(ofxColorType type) : meshType(OFX_AUTO_MESH), colorType(type), meshScaling(1.0f) {}
      Settings(float scale) : meshType(OFX_AUTO_MESH), colorType(OFX_AUTO_COLORS), meshScaling(scale) {}

      Settings& operator <<(ofxMeshType type){
        meshType = type;
        return *this;
      }
      Settings& operator <<(ofxColorType type){
        colorType = type;
        return *this;
      }
      Settings& operator <<(float scale){
        meshScaling = scale;
        return *this;
      }
    };

    /**
     * \brief Convert an OpenGP mesh into an ofMesh
     */
    static bool convert(Surface_mesh &mesh, ofMesh &newMesh, Settings settings = Settings()){
      if(settings.meshType == OFX_AUTO_MESH){
        if(mesh.is_triangle_mesh()){
          settings.meshType = OFX_TRIANGLE_MESH;
        } else if(mesh.is_quad_mesh()){
          settings.meshType = OFX_QUAD_MESH;
        } else {
          // we triangulate it
          mesh.triangulate();
          settings.meshType = OFX_TRIANGLE_MESH;
        }
      }
      int triIter;
      switch(settings.meshType){
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
      Surface_mesh::Face_property<Color> face_colors = mesh.get_face_property<Color>("f:color");

      // resolve color type
      if(settings.colorType == OFX_AUTO_COLORS){
        if(colors) settings.colorType = OFX_VERTEX_COLORS;
        else if(face_colors) settings.colorType = OFX_FACE_COLORS;
        else settings.colorType = OFX_NO_COLORS;
      }

      // check that it's valid
      switch(settings.colorType){
        case OFX_VERTEX_COLORS:
          if(!colors){
            // WARNING!
            return false;
          }
          break;
        case OFX_FACE_COLORS:
          if(!face_colors){
            // WARNING!
            return false;
          }
          break;
        default:
          // ok
          break;
      }

      // different construction depending on color type
      if(settings.colorType == OFX_FACE_COLORS){
        // we treat colors as per-face
        // => work with faces, not vertices!
        Surface_mesh::Face_iterator f_it, f_end = mesh.faces_end();
        for(f_it = mesh.faces_begin(); f_it != f_end; ++f_it){
          // uniform face color
          const Color &c = face_colors[*f_it];
          // circulate over vertices
          Surface_mesh::Vertex_around_face_circulator fv_it = mesh.vertices(*f_it);
          int tri = triIter;
          do {
            for(unsigned int i = 0; i < 3; ++i){
              // add vertex
              const Vec3 &v = points[*fv_it] * settings.meshScaling;
              newMesh.addVertex(ofVec3f(v.x, v.y, v.z));
              // same color at each vertex
              newMesh.addColor(ofColor(c.x, c.y, c.z));
              // normal?
              if(normals){
                const Normal &n = normals[*fv_it];
                newMesh.addNormal(ofVec3f(n.x, n.y, n.z));
              }
              if(i < 2) ++fv_it;
            }
          } while(--tri > 0);
        }
      } else {
        // we treat colors as per-vertex
        // copy all vertices
        Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
        for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
          // access point property like an array
          const Vec3 &v = points[*v_it] * settings.meshScaling;
          newMesh.addVertex(ofVec3f(v.x, v.y, v.z));

          // TODO add texture coordinates
          // and other properties (mapping?)
          // normals
          if(normals){
            const Normal &n = normals[*v_it];
            newMesh.addNormal(ofVec3f(n.x, n.y, n.z));
          }
          // colors
          if(colors && settings.colorType == OFX_VERTEX_COLORS){
            const Color &c = colors[*v_it];
            newMesh.addColor(ofColor(c.x, c.y, c.z));
          }
        }

        // add all faces
        Surface_mesh::Face_iterator f_it, f_end = mesh.faces_end();
        for(f_it = mesh.faces_begin(); f_it != f_end; ++f_it) {
          // access vertices of the face
          Surface_mesh::Vertex_around_face_circulator vc = mesh.vertices(*f_it);
          // triangle registration
          int tri = triIter;
          do {
            newMesh.addIndex((*vc).idx()); ++vc;
            newMesh.addIndex((*vc).idx()); ++vc;
            newMesh.addIndex((*vc).idx()); // not the last one
          } while (--tri > 0);
        }
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
    static const std::string VORONOI_AREA;
    static const std::string COT_PAIR;
    static const std::string UNIFORM_MEAN_CURVATURE;
    static const std::string MEAN_CURVATURE;
    static const std::string GAUSS_CURVATURE;
    static const std::string CIRCUM_RADIUS_TO_MIN_EDGE;

    // property type
    typedef Surface_mesh::Vertex_property<float> VertexFloat;
    typedef Surface_mesh::Edge_property<float> EdgeFloat;
    typedef Surface_mesh::Face_property<float> FaceFloat;

    /**
     * Voronoi area of each vertex
     */
    static VertexFloat voronoi_area(Surface_mesh &mesh, bool halfInv = true, const std::string &propName = VORONOI_AREA) {
      VertexFloat voro_area = mesh.vertex_property<float>(propName);

      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it){
        float area = 0.0f;
        Surface_mesh::Face_around_vertex_circulator vf_it, vf_end;
        vf_it = vf_end = mesh.faces(*v_it);
        do {
          Surface_mesh::Vertex_around_face_circulator fv_it = mesh.vertices(*vf_it);
          const Point& P = mesh.position(*fv_it);
          ++fv_it;
          const Point& Q = mesh.position(*fv_it);
          ++fv_it;
          const Point& R = mesh.position(*fv_it);
          area += cross(Q - P, R - P).norm() * 0.5f * 0.3333f;
        } while(++vf_it != vf_end);
        voro_area[*v_it] = halfInv ? 1.0 / (2.0 * area) : area;
      }
      return voro_area;
    }

    static EdgeFloat cot_pair(Surface_mesh &mesh, const std::string &propName = COT_PAIR) {
      typedef Surface_mesh::Vertex Vertex;
      typedef Surface_mesh::Halfedge Halfedge;
      EdgeFloat cot_pair = mesh.edge_property<float>(propName);

      Surface_mesh::Edge_iterator e_it, e_end = mesh.edges_end();
      for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
        float w = 0.0f;
        Halfedge h0 = mesh.halfedge(*e_it, 0);
        Vertex v0 = mesh.to_vertex(h0);
        Point p0 = mesh.position(v0);

        Halfedge h1 = mesh.halfedge(*e_it, 1);
        Vertex v1 = mesh.to_vertex(h1);
        Point p1 = mesh.position(v1);

        Halfedge h2 = mesh.next_halfedge(h0);
        Point p2 = mesh.position(mesh.to_vertex(h2));

        Point d0 = (p0 - p2).normalize();
        Point d1 = (p1 - p2).normalize();

        w += 1.0 / std::tan(std::acos(std::min(0.99f, std::max(-0.99f, d0.dot(d1) ))));

        h2 = mesh.next_halfedge(h1);
        p2 = mesh.position(mesh.to_vertex(h2));

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
    static VertexFloat uniform_mean_curvature(Surface_mesh &mesh, const std::string &propName = UNIFORM_MEAN_CURVATURE,
        const std::string &areaName = VORONOI_AREA, bool halfInvArea = true) {
      VertexFloat uniform_mean_curv = mesh.vertex_property<float>(propName);
      VertexFloat voro_area = mesh.get_vertex_property<float>(areaName);
      if(!voro_area) {
        voro_area = voronoi_area(mesh, halfInvArea, areaName);
      }

      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
        uniform_mean_curv[*v_it] = 0.0f;
        Point laplace(0, 0, 0);
        if (!mesh.is_boundary(*v_it)) {
          Surface_mesh::Vertex_around_vertex_circulator vv_it, vv_end;
          vv_it = vv_end = mesh.vertices(*v_it);
          do{
            laplace += (mesh.position(*vv_it) - mesh.position(*v_it));
          } while (++vv_it != vv_end);
          laplace *= halfInvArea ? voro_area[*v_it] : 0.5f / voro_area[*v_it];
          uniform_mean_curv[*v_it] = laplace.norm();
        }
      }
      return uniform_mean_curv;
    }

    /**
     * Mean curvature H at each vertex
     */
    static VertexFloat mean_curvature(Surface_mesh &mesh, const std::string &propName = MEAN_CURVATURE,
        const std::string &cotName = COT_PAIR,
        const std::string &areaName = VORONOI_AREA, bool halfInvArea = true) {
      VertexFloat mean_curv = mesh.vertex_property<float>(propName);
      EdgeFloat cot = mesh.get_edge_property<float>(cotName);
      if(!cot) {
        cot = cot_pair(mesh, cotName);
      }
      VertexFloat voro_area = mesh.get_vertex_property<float>(areaName);
      if(!voro_area) {
        voro_area = voronoi_area(mesh, halfInvArea, areaName);
      }

      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
        mean_curv[*v_it] = 0.0f;
        Point laplace(0, 0, 0);
        if (!mesh.is_boundary(*v_it)) {
          Surface_mesh::Halfedge_around_vertex_circulator vh_it, vh_end;
          vh_it = vh_end = mesh.halfedges(*v_it);
          do {
            Surface_mesh::Halfedge h = *vh_it;
            laplace += cot[mesh.edge(h)] * (mesh.position(mesh.to_vertex(h)) - mesh.position(*v_it));
          } while (++vh_it != vh_end);
          laplace *= halfInvArea ? voro_area[*v_it] : 0.5f / voro_area[*v_it];
          mean_curv[*v_it] = laplace.norm();
        }
      }
      return mean_curv;
    }

    /**
     * Gaussian curvature K at each vertex
     *
     * K = 1/A sum_i 2pi - phi_i
     */
    static VertexFloat gauss_curvature(Surface_mesh &mesh, const std::string &propName = GAUSS_CURVATURE,
        const std::string &areaName = VORONOI_AREA, bool halfInvArea = true) {
      VertexFloat gauss_curv = mesh.vertex_property<float>(propName);
      VertexFloat voro_area = mesh.get_vertex_property<float>(areaName);
      if(!voro_area) {
        voro_area = voronoi_area(mesh, halfInvArea, areaName);
      }

      // compute for all non-boundary vertices
      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
        gauss_curv[*v_it] = 0.0f;
        if (!mesh.is_boundary(*v_it)) {
          float angle_sum = 0.0f;

          Surface_mesh::Vertex_around_vertex_circulator vv_it, vv_it2, vv_end;
          vv_it = vv_end = mesh.vertices(*v_it);
          do {
            vv_it2 = vv_it; // copy to get next vertex
            ++vv_it2;
            Point d0 = (mesh.position(*vv_it) - mesh.position(*v_it)).normalize();
            Point d1 = (mesh.position(*vv_it2) - mesh.position(*v_it)).normalize();
            float cos_angle = std::max(-1.0f, std::min(1.0f, d0.dot(d1) ));
            angle_sum += std::acos(cos_angle);
          } while(++vv_it != vv_end);
          float diff_sum = 2 * M_PI - angle_sum;
          gauss_curv[*v_it] = halfInvArea ? diff_sum * 2.0f * voro_area[*v_it] : diff_sum / voro_area[*v_it];
        }
      }
      return gauss_curv;
    }

    /**
     * Triangle quality using the circum-radius of faces to their minimum edge length
     */
    static FaceFloat circum_radius_to_min_edge(Surface_mesh &mesh, const std::string &propName = CIRCUM_RADIUS_TO_MIN_EDGE) {
      FaceFloat triqual = mesh.face_property<float>(propName);

      Surface_mesh::Face_iterator f_it, f_end = mesh.faces_end();
      for (f_it=mesh.faces_begin(); f_it != f_end; ++f_it){
        Surface_mesh::Vertex_around_face_circulator fv_it = mesh.vertices(*f_it);
        // getting the vertices
        Point v0 = mesh.position(*fv_it); ++fv_it;
        Point v1 = mesh.position(*fv_it); ++fv_it;
        Point v2 = mesh.position(*fv_it);
        // getting the edges
        Point d0 = v1 - v0;
        Point d1 = v2 - v0;
        Point d2 = v2 - v1;
        // circumradius computation
        float denom = 4.0 * d0.cross(d1).sqrnorm();
        float circum_radius_sq;
        if(denom < 1e-4){
          // too small for us, we just assign a large value
          // for numerical stability
          circum_radius_sq = 1e12;
        }else{
          circum_radius_sq = d0.sqrnorm() * d1.sqrnorm() * d2.sqrnorm() / denom;
        }
        float min_length_sq = std::min(std::min(d0.sqrnorm(), d1.sqrnorm()), d2.sqrnorm());
        triqual[*f_it] = std::sqrt(circum_radius_sq / min_length_sq);
      }
      return triqual;
    }

    /**
     * Convert a property into a color mapping
     */
    static void property_to_color(Surface_mesh &mesh, const VertexFloat& prop) {
      Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
      std::vector<float> values(mesh.n_vertices());
      for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it){
        float value = prop[*v_it];
        values.push_back(value);
      }

      //discard upper and lower 5%
      unsigned int N = values.size() - 1;
      unsigned int i = N / 20;
      std::sort(values.begin(), values.end());
      float min = values[i];
      float max = values[N - 1 - i];

      // assign colors
      Surface_mesh::Vertex_property<Color> colors = mesh.vertex_property<Color>("v:color");
      for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it){
        colors[*v_it] = value_to_color(prop[*v_it], min, max);
      }
    }

    /**
     * Convert a property into a color mapping
     */
    static void property_to_color(Surface_mesh &mesh, const FaceFloat& prop) {
      Surface_mesh::Face_iterator f_it, f_end = mesh.faces_end();
      std::vector<float> values(mesh.n_faces());
      for(f_it = mesh.faces_begin(); f_it != f_end; ++f_it){
        float value = prop[*f_it];
        values.push_back(value);
      }

      //discard upper and lower 5%
      unsigned int N = values.size() - 1;
      unsigned int i = N / 20;
      std::sort(values.begin(), values.end());
      float min = values[i];
      float max = values[N - 1 - i];

      // assign colors
      Surface_mesh::Face_property<Color> colors = mesh.face_property<Color>("f:color");
      for(f_it = mesh.faces_begin(); f_it != f_end; ++f_it){
        colors[*f_it] = value_to_color(prop[*f_it], min, max);
      }
    }

    /**
     * Convert a value to a color
     *
     * @param value the value
     * @param min the expected minimum value
     * @param max the expected maximum value
     * @return the corresponding color
     */
    static Color value_to_color(float value, float min, float max) {
      float v0, v1, v2, v3, v4;
      v0 = min + 0.0/4.0 * (max - min);
      v1 = min + 1.0/4.0 * (max - min);
      v2 = min + 2.0/4.0 * (max - min);
      v3 = min + 3.0/4.0 * (max - min);
      v4 = min + 4.0/4.0 * (max - min);

      Color col(255,255,255);
      unsigned char u;

      if (value < v0) col = Color(0, 0, 255);
      else if (value > v4) col = Color(255, 0, 0);
      else if (value <= v2){
        if (value <= v1) // [v0, v1]
        {
          u = (unsigned char) (255.0 * (value - v0) / (v1 - v0));
          col = Color(0, u, 255);
        } else // ]v1, v2]
        {
          u = (unsigned char) (255.0 * (value - v1) / (v2 - v1));
          col = Color(0, 255, 255-u);
        }
      } else {
        if (value <= v3) // ]v2, v3]
        {
          u = (unsigned char) (255.0 * (value - v2) / (v3 - v2));
          col = Color(u, 255, 0);
        }
        else // ]v3, v4]
        {
          u = (unsigned char) (255.0 * (value - v3) / (v4 - v3));
          col = Color(255, 255-u, 0);
        }
      }
      return col;
    }

    ///////////////////////////////////////////////////////////////////////////
    ///// Mesh operations /////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    static void laplace_uniform_smooth(Surface_mesh &mesh, bool tangential = false, unsigned int rounds = 5, float eta = 0.5){
      Surface_mesh::Vertex_property<Vec3> laplace = mesh.vertex_property<Vec3>("v:tmp:vec3f");
      Surface_mesh::Vertex_property<Normal> normals = mesh.vertex_property<Normal>("v:normal");
      if(tangential){
        mesh.update_vertex_normals();
      }
      for(unsigned int r = 0; r < rounds; ++r){
        Surface_mesh::Vertex_iterator v_it, v_end = mesh.vertices_end();
        for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it){
          Vec3 barycenter(0, 0, 0);
          int val = 0;
          Surface_mesh::Vertex_around_vertex_circulator vv_it, vv_end;
          vv_it = vv_end = mesh.vertices(*v_it);
          do {
            barycenter += mesh.position(*vv_it);
            ++val;
          } while (++vv_it != vv_end);
          barycenter *= 1.0f / val;
          Vec3 u = barycenter - mesh.position(*v_it);
          if(tangential){
            Normal n = normals[*v_it];
            float n_norm = n.norm();
            if(n_norm == 0 || n_norm != n_norm){
              u = Normal(0, 0, 0);
            } else {
              // tangential component of u
              u = u - n * u.dot(n) / n_norm;
            }
          }
          laplace[*v_it] = u;
        }
        // apply shift
        for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it){
          if(mesh.is_boundary(*v_it)) continue;
          mesh.position(*v_it) += laplace[*v_it] * eta;
        }
      }
    }

};
