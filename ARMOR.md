# ARMOR

------

### 文件结构目录：

> **上海交通大学**
>
> https://github.com/sycskr/SJ-code-Reading

```
.
└── armor                       // 存放自瞄主要算法代码
    ├── include                 // 自瞄头文件
    |     ├──classifier.h
    |     |
    |     └──armor_finder.h
    |
    └── src                     // 自瞄源码
          ├──anti_top.cpp
          |
          ├──armor_box.cpp
          |
          ├──classifier.cpp
          |
          ├──find_armor_box.cpp
          |
          ├──find_light_blobs.cpp
          |
          ├──searching_state.cpp
          |
          ├──send_target.cpp
          |
          ├──standby_state.cpp
          |
          ├──tracking_state.cpp
          |
          ├──armor_finder.cpp
          |
          └──show_images.cpp
```

### include:

------

#### 1、classifier.h



#### 2、armor_finder.h



### src：

------

#### 1、anti_top

> **反小陀螺**

```cpp
#include <additions.h>
#include <log.h>  //？？？？什么库
```

```c++
/*
 * name @ mean
 * param @ 输入陀螺周期循环队列
 * func @ 遍历整个环形队列，求环形队列内部总共的和，得出环形队列平均值
 * return @ 环形队列的平均值（double）
*/
```



```c++
/**
 * name @ getFrontTime
 * params @ time_seq一个周期内的时间采样点
 *          angle_seq一个周期内的角度采样点
 * func @ 通过线性拟合计算出角度为0时对应的时间点
 * return @ 角度为0时对应的时间点
*/
```



```c++
/**
 * name @ ArmorFinder::antiTop
 * params @
 * func @ 判断是否发生装甲目标切换。
 *      @ 记录切换前一段时间目标装甲的角度和时间
 *      @ 通过线性拟合计算出角度为0时对应的时间点
 *      @ 通过两次装甲角度为零的时间差计算陀螺旋转周期
 *      @ 根据旋转周期计算下一次装甲出现在角度为零的时间点
 * return @ void
*/
```



#### 2、armor_box



#### 3、classifier



#### 4、find

##### 4.1find_armor_box



##### 4.2find_light_blobs

#### 5、searching_state



#### 6、send_target



#### 7、standby_state



#### 8、tracking_state

##### 8.1tracking_state



##### 8.2armor_finder



#### 9、show_images

