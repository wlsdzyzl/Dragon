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

- Loop subdivision (updated in 2021/3/10)

![](./image/dragon_loopsubdivision.gif)
- Mesh to SDF (updated in 2021/7/2)

![](./image/dragon_mesh2sdf.gif)

We can estimate the quality of mesh2sdf algorithm by applying marching-cube algorithm on generated sdf to extract a new mesh, here are some results:

**Bunny_head.obj, voxel resolution: 0.01**

![](./image/generated_bunny_head.png)

**happy_vrip.ply, voxel resolution: 0.002**

![](./image/generated_happy.png)
#### Point Cloud
- Point Cloud Processing
- Octree
- Simplified RBF reconstruction and Poisson reconstruction

![](./image/dragon_pcd.gif)

##### Reconstructed model using RBF and Poisson
![](./image/arma_comp.jpg)

![](./image/dragon_comp.jpg)

![](./image/kitten_comp.jpg)

##### Reconstructed model with different depth (3-7)
![](./image/arma.jpg)

![](./image/dragon.jpg)

![](./image/kitten.jpg)

#### Tubular Shape Reconstruction from Skeleton Representation
To better understand this work, please refer to [A Geometric Algorithm for Tubular Shape Reconstruction from Skeletal Representation](https://arxiv.org/pdf/2402.12797v2).

![](./image/tubu_shape_recon.png)

#### Extract Tree Graph and Key-node Graph from Skeleton Representation
**Skeleton Representation**
![](./image/skeleton_points.png)
**Tree Graph from Skeleton**
![](./image/tree_graph.png)
**Key-node Graph from Skeleton (End points and bifurcation points are considered as key-nodes)**
![](./image/key_graph.png)
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
## ChangeLog
- 2021/3/10: Loop subdivision
- 2021/3/14: Apply real Phong model on visualization as following(the old shader is from elastic fusion and flashfusion, and the color on each vertex will not change as the moving object):
![](./image/dragon_phong_model.gif)
- 2021/7/2: Generate sdf from triangle mesh (`Mesh2SDF` in `Mesh2SDF.h`). 
- 2023/9/25: Generate sdf from skeleton representation (`CenterLine2SDF` in `Mesh2SDF.h`)
- 2023/12/7: Generate tree graph from skeleton representation (Refer to `app/Centerline2SDF.cpp`)
## License 
Under the MIT License, see http://opensource.org/licenses/MIT.