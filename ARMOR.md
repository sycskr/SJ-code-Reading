# ARMOR

------

### 文件结构目录：

> **上海交通大学**
>
> 代码注释见：https://github.com/sycskr/SJ-code-Reading

```
.
└── armor                       // 存放自瞄主要算法代码
    ├── include                 // 自瞄头文件
    |     ├──classifier.h
    |     |
    |     └──armor_finder.h
    |
    └── src                     // 自瞄源码
          ├──anti_top.cpp       //反小陀螺
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

​    当敌我双方没有发送相对移动的时候，则可以计算转动角速度。
![pic4](https://xinyang-go.github.io//static/img/anti-top1.png)

通过简单的几何计算可以发现β∝α，再假设敌方匀速运动，即α∝t，所以β∝t，而β和t都是可以采集到的数据，所以我们可以通过对正面这个装甲板从消失到出现的角度和时间进行线性拟合，得出β=0时对应的时间，函数如下

```c++
/**
 * name @ getFrontTime
 * params @ time_seq一个周期内的时间采样点
 *          angle_seq一个周期内的角度采样点
 * func @ 通过线性拟合计算出角度为0时对应的时间点
 * return @ 角度为0时对应的时间点
*/
```



**反陀螺主函数：**

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

​      由于通常敌方车辆前后装甲板和左右装甲板距离旋转中心的半径不同，当我方视野中心不是正对敌方旋转中心时，连续两次β=0的时间差将不再是90°。如图

![pic5](https://xinyang-go.github.io//static/img/anti-top2.png)

使用前后装甲板，和左右装甲板分别计算β=0的时间差，这样时间差正好为敌方转过180°的时间。

#### 2、armor_box



#### 3、classifier



#### 4、find

#####        4.1find_armor_box

> 其中RotateRect的角度计算
>
> https://www.cnblogs.com/panxiaochun/p/5478555.html

```c++
/**
 * name @ angelJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的角度差(小于20度)
 * return @ static bool
*/
```



```c++
/**
 * name @ heightJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的高度差（小于30）
 * return @ static bool
*/
```



```c++
/**
 * name @ lengthJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的间距(在装甲板长度的0.5到10倍之间)
 * return @ static bool
*/
```



```c++
/**
 * name @ lengthRatioJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的长度比(长度比在0.4到2.5之间)
 * return @ static bool
*/
```



```c++
/**
 * name @ CuoWeiDuJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的错位度，不知道英文是什么！！！
 * return @ static bool
*/
```



```c++
/**
 * name @ boxAngleJudge
 * params @ 两个LightBlob
 * func @ 判断装甲板方向
 * return @ static bool
*/
```



```c++
/**
 * name @ isCoupleLight
 * params @ 两个LightBlob,unsigned char敌方颜色
 * func @ 判断两个灯条是否可以匹配
 * return @ static bool
*/
```



#####        4.2find_light_blobs

#### 5、searching_state



#### 6、send_target



#### 7、standby_state



#### 8、tracking_state

#####         8.1tracking_state

追踪模式主函数:

```c++
/**
 * @name ArmorFinder::stateTrackingTarget
 * @param cv::Mat &src 当前图像
 * @func 根据上一帧目标的位置得出这一帧的目标
 * @return bool 追踪失败，返回false。
*/
```

​       问题：

- 由于KCF算法是根据上一帧更新下一帧，所以当追踪时间长了，可能会忘记一开始的目标追到其他地方去。

- 同时由于KCFTracker在追踪的时候，只改变追踪框的位置，而不改变长宽。但在后续数据发送时需要长宽信息进行距离的估算。

  解决方式：

- 需要使用分类器判断当前是否追踪错误。
- 搜寻roi中亮点并与装甲板亮点数进行对比判断是否跟丢
- 将追踪区域扩大一倍重新搜寻一次目标，由于搜寻区域小，速度较快。

#####         8.2armor_finder



#### 9、show_images

