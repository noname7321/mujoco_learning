# 可视化及渲染
## 可视化配置
**mjtVisFlag对应mjvOption.flags中的设置，在simulate中对应如下配置**
![](../../MJCF/asset/mjtvisflag.png)
可以使用opt.flags[mjtVisFlag::mjVIS_XXX]=true开启功能
```C++
typedef enum mjtVisFlag_ {        // flags enabling model element visualization
  mjVIS_CONVEXHULL    = 0,        // mesh convex hull
  mjVIS_TEXTURE,                  // textures
  mjVIS_JOINT,                    // joints
  mjVIS_CAMERA,                   // cameras
  mjVIS_ACTUATOR,                 // actuators
  mjVIS_ACTIVATION,               // activations
  mjVIS_LIGHT,                    // lights
  mjVIS_TENDON,                   // tendons
  mjVIS_RANGEFINDER,              // rangefinder sensors
  mjVIS_CONSTRAINT,               // point constraints
  mjVIS_INERTIA,                  // equivalent inertia boxes
  mjVIS_SCLINERTIA,               // scale equivalent inertia boxes with mass
  mjVIS_PERTFORCE,                // perturbation force
  mjVIS_PERTOBJ,                  // perturbation object
  mjVIS_CONTACTPOINT,             // contact points
  mjVIS_ISLAND,                   // constraint islands
  mjVIS_CONTACTFORCE,             // contact force
  mjVIS_CONTACTSPLIT,             // split contact force into normal and tangent
  mjVIS_TRANSPARENT,              // make dynamic geoms more transparent
  mjVIS_AUTOCONNECT,              // auto connect joints and body coms
  mjVIS_COM,                      // center of mass
  mjVIS_SELECT,                   // selection point
  mjVIS_STATIC,                   // static bodies
  mjVIS_SKIN,                     // skin
  mjVIS_FLEXVERT,                 // flex vertices
  mjVIS_FLEXEDGE,                 // flex edges
  mjVIS_FLEXFACE,                 // flex element faces
  mjVIS_FLEXSKIN,                 // flex smooth skin (disables the rest)
  mjVIS_BODYBVH,                  // body bounding volume hierarchy
  mjVIS_FLEXBVH,                  // flex bounding volume hierarchy
  mjVIS_MESHBVH,                  // mesh bounding volume hierarchy
  mjVIS_SDFITER,                  // iterations of SDF gradient descent

  mjNVISFLAG                      // number of visualization flags
} mjtVisFlag;
```
**mjvOption.label使用mjtLabel显示对应标签**
**mjvOption.frame使用mjtFrame显示对应坐标系**
**mjvOption中还可以设置分组的可视化和bvh树深度**
完整定义：
```C++
struct mjvOption_ {                  // abstract visualization options
  int      label;                    // what objects to label (mjtLabel)
  int      frame;                    // which frame to show (mjtFrame)
  mjtByte  geomgroup[mjNGROUP];      // geom visualization by group
  mjtByte  sitegroup[mjNGROUP];      // site visualization by group
  mjtByte  jointgroup[mjNGROUP];     // joint visualization by group
  mjtByte  tendongroup[mjNGROUP];    // tendon visualization by group
  mjtByte  actuatorgroup[mjNGROUP];  // actuator visualization by group
  mjtByte  flexgroup[mjNGROUP];      // flex visualization by group
  mjtByte  skingroup[mjNGROUP];      // skin visualization by group
  mjtByte  flags[mjNVISFLAG];        // visualization flags (indexed by mjtVisFlag)
  int      bvh_depth;                // depth of the bounding volume hierarchy to be visualized
  int      flex_layer;               // element layer to be visualized for 3D flex
};
typedef struct mjvOption_ mjvOption;
```

## 场景渲染
**mjtRndFlag作用于mjvScene.flags，对应simulate中如下配置**
![](../../MJCF/asset/mjtRndFlag.png)
```C++
typedef enum mjtRndFlag_ {        // flags enabling rendering effects
  mjRND_SHADOW        = 0,        // shadows
  mjRND_WIREFRAME,                // wireframe
  mjRND_REFLECTION,               // reflections
  mjRND_ADDITIVE,                 // additive transparency
  mjRND_SKYBOX,                   // skybox
  mjRND_FOG,                      // fog
  mjRND_HAZE,                     // haze
  mjRND_SEGMENT,                  // segmentation with random color
  mjRND_IDCOLOR,                  // segmentation with segid+1 color
  mjRND_CULL_FACE,                // cull backward faces

  mjNRNDFLAG                      // number of rendering flags
} mjtRndFlag;
```
### 图像分割
**mjRND_SEGMENT and mjRND_IDCOLOR**
mjtRndFlag中mjRND_SEGMENT是随机颜色分割物体，
mjRND_IDCOLOR是通过设置mjvGeom.segid固定物体分割颜色
随机分割效果：
![](../../MJCF/asset/seg1.png)
![](../../MJCF/asset/seg2.png)
mujoco源码中segid映射到rgba见src/render/render_gl3.c如下
![](../../MJCF/asset/segid.png)
使用segid分割：
![](../../MJCF/asset/set_segid1.png)
![](../../MJCF/asset/set_segid2.png)
## 单/双目渲染
**mjtStereo作用于mjvScene.stereo**
分别是单目，四缓冲,并排，可以直接使用mjSTEREO_SIDEBYSIDE显示双目，mjSTEREO_QUADBUFFERED则需要更好一些的GPU
```C++
typedef enum mjtStereo_ {         // type of stereo rendering
  mjSTEREO_NONE       = 0,        // no stereo; use left eye only
  mjSTEREO_QUADBUFFERED,          // quad buffered; revert to side-by-side if no hardware support
  mjSTEREO_SIDEBYSIDE             // side-by-side
} mjtStereo;
```
演示：
![](../../MJCF/asset/Binocular.png)

