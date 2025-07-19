# flex
**name=""**        
&emsp;&emsp;复合体固定位置          
**dim=""**        
&emsp;&emsp;维度，可以写1,2,3。在 1D 中，元素是胶囊，在 2D 中，元素是 具有半径的三角形，在 3D 中，元素是具有（可选）半径的四面体。      
**radius=""**        
&emsp;&emsp;所有 flex 元素的半径。它在 3D 中可以为零，但在 1D 和 2D 中必须为正。半径会影响两者 碰撞检测和渲染。在 1D 和 2D 中，需要使单元具有体积。         
**body="string"**        
&emsp;&emsp;定点元素名
**material=""**        
&emsp;&emsp;材质        
**rgba=""**        
&emsp;&emsp;颜色

## flex/edge
边缘弯曲的阻尼和刚度
**stiffness="0"**       
**damping="0"**   

## flex/elasticity
有限元参数
**young="0"**        
&emsp;&emsp;杨式模量        
**poisson="0"**        
&emsp;&emsp;柏松比        
**damping="0"**        
&emsp;&emsp;该量缩放由 Young 模量定义的刚度，以生成阻尼矩阵。           
**thickness="-1"**        
&emsp;&emsp;壳厚，长度单位;仅适用于二手 2D 弯曲。用于缩放拉伸刚度。 此厚度可以设置为等于半径的 2 倍，以匹配几何体， 但会单独显示，因为半径可能受到与碰撞检测相关的注意事项的限制。  
**elastic2d="[none, bend, stretch, both]"**
&emsp;&emsp;该参数在3.3.3及以后版本使用，对 2D 弯曲的被动力的弹性贡献。“none”： 无， “bend”： 仅弯曲， “stretch”： 仅拉伸， “both”： 弯曲和拉伸。

## flex/contact
**internal="false"**        
&emsp;&emsp;启用或禁用内部碰撞        
**selfcollide="auto" [none, narrow, bvh, sap, auto]**        
&emsp;&emsp;启用或禁用内部碰撞

# flexcomp
**name=""**        
&emsp;&emsp;生成命名          
**dim=""**        
&emsp;&emsp;维度，可以写1,2,3。在 1D 中，元素是胶囊，在 2D 中，元素是 具有半径的三角形，在 3D 中，元素是具有（可选）半径的四面体。  
**dof="full" [full, radial, trilinear]**        
&emsp;&emsp;弯曲自由度 （dof） 的参数化。       
**type="grid" [grid, box, cylinder, ellipsoid, disc, circle, mesh, gmsh, direct]**        
&emsp;&emsp;有限元子对象类型
**count=""**        
&emsp;&emsp;每个维度中自动生成的点的数量。this 和 next 属性仅适用于类型网格， box、cylinder、ellipsoid。         
**spacing=""**        
&emsp;&emsp;每个维度中自动生成的点之间的间距。间距应足够大 与半径相比，以避免永久接触。          
**mass=""**        
&emsp;&emsp;总质量，每个自动生成的实体的质量等于该值除以点数。           
**inertiabox=""**        
&emsp;&emsp;每个点的旋转惯性，此处为对角线。         
**file=""**        
&emsp;&emsp;从中加载曲面（三角形）或体积（四面体）网格的文件的名称。可以是stl,obj,gmsh格式      
**rigid="false"**        
&emsp;&emsp;如果为 true，则所有点都对应于父形体中的顶点，并且不会创建新形体。这是 等效于固定所有点。请注意，如果所有点都确实被固定，模型编译器将检测到 Flex 是刚性的（在碰撞检测中表现为非凸面网格）。           
**pos=""**        
&emsp;&emsp;位置          
**quat，axisangle, xyaxes, zaxis, euler=""**        
&emsp;&emsp;旋转             
**scale=""**        
&emsp;&emsp;点坐标缩放，宏观就是尺寸缩放       

## flexcomp/edge
边缘弯曲的阻尼和刚度
**stiffness="0"**       
**damping="0"**  
**equality="false"  [true, false]**  
&emsp;&emsp;约束边缘    

## flexcomp/pin
固定body/grid
**id=""**        
&emsp;&emsp;位置        
**range="int(2 * n)"**        
&emsp;&emsp;位置        
**grid="int(dim * n)"**        
&emsp;&emsp;位置        
**gridrange="int(2 * dim * n)"**        
&emsp;&emsp;位置        