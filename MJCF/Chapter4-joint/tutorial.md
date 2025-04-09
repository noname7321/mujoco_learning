# joint
![](../asset/joint.png)
&emsp;&emsp;joint将body之间连接在一起，使其可以进行活动。这么说吧，body中的所有geom为一个整体然后joint是连接这些整体的。就是body和body靠joint活动，body中只能有一个joint用来连接当前body和上一层body。再根本一点就是joint对于上一层body是相对静止的，当前body与joint是在运动。
* name
* tpye="[free/ball/slide/hinge]" 自由关节，一般不用；球形关节，绕球旋转；滑轨；旋转关节
* pos="0 0 0"关节在body的位置
* axis="0 0 1"x,y,z活动轴，只有slide和hinge有用
* stiffness="0"弹簧，数值正让关节具有弹性
**`(0-pos)*stiffness`**
* range="0 0"关节限制，当球形时只有二参有效，一参设置为0，但是要在compiler指定autolimits
* limited="auto"此属性指定关节是否有限制
* damping="0"阻尼   
**`(0-v)*damping`**
* frictionloss="0"关节摩擦损失
* armature="0" 电枢 转子转动惯量*减速比^2
* ref 角度偏置