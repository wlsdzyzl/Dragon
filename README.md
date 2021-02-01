# Dragon
Dragon is a library for geometry processing. 

## What can dragon do?
### DragonView2D
- Voronoi diagram and Delaunay triangulation
![](./image/dragon_voronoi.gif)
- 7 kinds of Curves
![](./image/dragon_curve.gif)

### DragonView3D
#### Mesh
- Mesh Curvature
- Mesh Smoothing
- Mesh Parameterization for model with border
- Mesh simplification
![](./image/dragon_mesh.gif)
#### Point Cloud
- Point Cloud Processing
- Octree
- Simplified RBF reconstruction and Poisson reconstruction
![](./image/dragon_pcd.gif)
**Reconstructed model using RBF and Poisson**
![](./image/arma_comp.jpg)
![](./image/dragon_comp.jpg)
![](./image/kitten_comp.jpg)

**Reconstructed model with different depth (3-7)**
![](./image/arma.jpg)
![](./image/dragon.jpg)
![](./image/kitten.jpg)
### Other  
- SpiderMan (The curve is cubic spline):
![](./image/dragon_spiderman.gif)

## Get Started
Prerequisites:
- OpenGL, GLUT, GLEW

Other 3rd libraries are packaged into 3rdparty (imgui)
```
mkdir build
cd build
cmake .. && make -j
```
Refer to [https://github.com/MyEvolution/GAMES102](https://github.com/MyEvolution/GAMES102) for more information.

## Acknowledgement

Thanks to [GAMES 102](http://staff.ustc.edu.cn/~lgliu/Courses/GAMES102_2020/default.html), a large part of this library comes from the assignments of this course.

## TODO
Remeshing
## License 
Under the MIT License, see http://opensource.org/licenses/MIT.