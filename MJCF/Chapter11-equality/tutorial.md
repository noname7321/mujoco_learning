# 闭链
&emsp;&emsp;在机器人设计中有时候会出现并联连杆结构，但是常规的建模方式没办法形成闭环，在urdf中我们没办法实现并联连杆，但是mujoco中提供了一种连接方式。
![](../asset/equality.png)
&emsp;&emsp;mujoco节点下的equality节点可以实现这一功能，它可以将两个物体连接到一起。
&emsp;&emsp;connect和weld中body1和body2是要连接的两个body
* anchor是在body1的坐标系中的两个body连接位置。
* site 作为连接点
connect和weld在很多情况上区别不大，比如连杆两端都是固定位置。connect是点对点连接，自由度和球形关节一样，连接只限制了物体之间的距离。weld是刚性连接，自由度是锁死的，可以通过torquescale来调节连接强度（阻尼）。
