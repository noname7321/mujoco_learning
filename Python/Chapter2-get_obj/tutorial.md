# get obj
## 获取名字
数量
![](../../MJCF/asset/entity.png)
**先看mjModel结构体开头部分，这里的命名方式都是nXXX，这代表各个元素的数量**
![](../../MJCF/asset/names.png)
## 获取id
`MJAPI int mj_name2id(const mjModel* m, int type, const char* name);`
**通过name获取实体的id          
m :mjModel
type:   mjmodel.h文件中的mjtObj中定义，这个是要获取id的实体类型一下是部分type类型枚举，在mjtObj中找到       
name: name
**     
![](../../MJCF/asset/enum_mjtobj.png)
## 获取位置
![](../../MJCF/asset/xpos.png)
**可以通过 xpos和xxx_xpos获取各个对象的位置**
## 获取姿态
**通过xquat可以获取body的姿态**

我们可以和C++接口一样通过m.names中寻找各个实体的名字nXXX得到实体数量，name_xxxadr来寻找实体名字在names中的索引。
在names中名字字符串之间通过”\x00”分割，name_xxxadr定位到的是该实体的第一个字符的位置，可以使用python的数组截取功能实现读取字符串，在寻找末尾的0来截取实体的实际名字。

```python
name= m.names[m.name_bodyadr[1]:]
for i in range(len(name)):
if name[i] == 0:
name = name[:i]
break
print(name)
```
