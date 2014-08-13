ofxOpenGP
=========

OpenFrameworks addon to use [OpenGP](http://opengp.github.io) (Open Geometry Processing).

![image](ofxaddons_thumbnail.png)

Usage
-----
```cpp
#include "ofxOpenGP.h"

// 1. Create your Surface_mesh instance
opengp::Surface_mesh mesh;
mesh.load("data/bunny.obj");

// 2. Play with your instance using OpenGP
...

// 3. Convert to ofMesh / ofVboMesh
ofMesh newMesh;
ofxOpenGP::convert(mesh, newMesh);
```

For a more comprehensive case, look at the project in `example/`.

FAQ
---

**Q. My mesh looks tiny with my `ofEasyCam`, what should I do?**
A. Either you scale your mesh, change your camera, or automatically scale it using the 4th parameter of `ofxOpenGP::convert`.

**Q. My mesh is a polygonal mesh with fancy face valences, what can I do?**
A. OpenGP automatically triangulate meshes in the conversion using `Surface_mesh::triangulate` if needed.
While quad meshes are easy and mostly safe, this may not be the case for general polygons.
You may want to take care of the topology yourself beforehand.

**Q. Can you tell us more about `ofxOpenGP::convert`?**
A. Here it is:

```cpp
bool ofxOpenGP::convert(opengp::Surface_mesh &mesh, ofMesh &newMesh, ofxMeshType meshType, float scaling = 1.0f);
```

  - Param `mesh`: your surface mesh from OpenGP (not `const` as we use triangulate when we need to)
  - Param `newMesh`: your ofMesh for an easy display within OpenFrameworks
  - Param `meshType`: one of `OFX_AUTO_MESH`, `OFX_TRIANGLE_MESH` and `OFX_QUAD_MESH`
  - Param `scaling`: a factor by which to scale the vertex positions (because of `ofEasyCam`
  - Returns: `true` if it worked without error, `false` in case an error occurred

License
-------
Released under the [MIT License](http://www.opensource.org/licenses/MIT).
