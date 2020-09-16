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
static bool angelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    return abs(angle_i - angle_j) < 20;
}
```



```c++
/**
 * name @ heightJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的高度差（小于30）
 * return @ static bool
*/
static bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    return abs(centers.y) < 30;
}
```



```c++
/**
 * name @ lengthJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的间距(在装甲板长度的0.5到10倍之间)
 * return @ static bool
*/
static bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    return (side_length / light_blob_i.length < 10 && side_length / light_blob_i.length > 0.5);
}
```



```c++
/**
 * name @ lengthRatioJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的长度比(长度比在0.4到2.5之间)
 * return @ static bool
*/
static bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return (light_blob_i.length / light_blob_j.length < 2.5
            && light_blob_i.length / light_blob_j.length > 0.4);
}
```



```c++
/**
 * name @ CuoWeiDuJudge
 * params @ 两个LightBlob
 * func @ 判断两个灯条的错位度，不知道英文是什么！！！
 * return @ static bool
*/
static bool CuoWeiDuJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
    if (abs(angle_i - angle_j) > 90) {
        angle += 3.14159265459 / 2;
    }
    Vector2f orientation(cos(angle), sin(angle));
    Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
                 light_blob_j.rect.center.y - light_blob_i.rect.center.y);
    return abs(orientation.dot(p2p)) < 25;
}
```



```c++
/**
 * name @ boxAngleJudge
 * params @ 两个LightBlob
 * func @ 判断装甲板方向
 * return @ static bool
*/
static bool boxAngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0;
    if (abs(angle_i - angle_j) > 90) {
        angle += 90.0;
    }
    return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}
```



```c++
/**
 * name @ isCoupleLight
 * params @ 两个LightBlob,unsigned char敌方颜色
 * func @ 判断两个灯条是否可以匹配
 * return @ static bool
*/
static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, uint8_t enemy_color) {
    return light_blob_i.blob_color == enemy_color &&
           light_blob_j.blob_color == enemy_color &&
           lengthRatioJudge(light_blob_i, light_blob_j) &&
           lengthJudge(light_blob_i, light_blob_j) &&
           //           heightJudge(light_blob_i, light_blob_j) &&
           angelJudge(light_blob_i, light_blob_j) &&
           boxAngleJudge(light_blob_i, light_blob_j) &&
           CuoWeiDuJudge(light_blob_i, light_blob_j);

}
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

- 没法应对全遮挡，追踪失败也较难的得到反馈。

- 全部遮挡后很难恢复

  解决方式：

- 需要使用分类器判断当前是否追踪错误。

- 搜寻roi中亮点并与装甲板亮点数进行对比判断是否跟丢

- 将追踪区域扩大一倍重新搜寻一次目标，由于搜寻区域小，速度较快。

```c++
bool ArmorFinder::stateTrackingTarget(cv::Mat &src) {
     auto pos = target_box.rect;
    // 使用KCFTracker进行追踪
    //1.找不到追踪目标：
     if(!tracker->update(src, pos)){ 
         target_box = ArmorBox();//找不到追踪目标，把识别到的装甲板box给target，重新开始追踪。
         LOGW("Track fail!");
         return false;//追踪失败
     }
     //Rect2d(0, 0, 640, 480) x,y,width,height
    //2.追踪目标超出范围
     if((pos & cv::Rect2d(0, 0, 640, 480)) != pos){
         target_box = ArmorBox();
         LOGW("Track out range!");
         return false;//超出范围
     }

     // 扩大范围，获取相较于追踪区域两倍长款的区域，用于重新搜索，获取灯条信息
 @@ -30,11 +37,16 @@ bool ArmorFinder::stateTrackingTarget(cv::Mat &src) {
     cv::Mat roi = src(bigger_rect).clone();

     ArmorBox box;
     // 在区域内重新搜索。

     //由于KCFTracker在追踪的时候，只改变追踪框的位置，而不改变长宽。但在后续数据发送时需要长宽信息进行距离的估算。
     //并且由于KCF算法是根据上一帧更新下一帧，所以当追踪时间长了，可能会忘记一开始的目标追到其他地方去。
     //所以在区域内重新搜索。
     if(findArmorBox(roi, box)) { // 如果成功获取目标，则利用搜索区域重新更新追踪器
         target_box = box;
         target_box.rect.x += bigger_rect.x; //　添加roi偏移量
         //target_box　添加roi偏移量
         target_box.rect.x += bigger_rect.x; 
         target_box.rect.y += bigger_rect.y;
         //blob　添加roi偏移量
         for(auto &blob : target_box.light_blobs){
             blob.rect.center.x += bigger_rect.x;
             blob.rect.center.y += bigger_rect.y;
 @@ -46,17 +58,18 @@ bool ArmorFinder::stateTrackingTarget(cv::Mat &src) {
         if(classifier){ // 分类器可用，使用分类器判断。
             cv::resize(roi, roi, cv::Size(48, 36));
             if(classifier(roi) == 0){
                 target_box = ArmorBox();
                 LOGW("Track classify fail range!");
                 return false;
             }
         }else{ //　分类器不可用，使用常规方法判断
             cv::Mat roi_gray;
             cv::cvtColor(roi, roi_gray, CV_RGB2GRAY);
             cv::threshold(roi_gray, roi_gray, 180, 255, cv::THRESH_BINARY);
             //cv::countNonZero返回灰度值不为0的像素数,可用来判断图像是否全黑,此处寻找亮点数
             contour_area = cv::countNonZero(roi_gray);
             if(abs(cv::countNonZero(roi_gray) - contour_area) > contour_area * 0.3){
                 target_box = ArmorBox();
                 return false;
             }
         }
```



------

------

**追踪算法：**

如果需要更高的准确率，并且可以容忍延迟的话，使用CSRT

如果需要更快的FPS，并且可以容许稍低一点的准确率的话，使用KCF

如果纯粹的需要速度的话，用MOSSE



**Boost 追踪器**

以Haar特征级联的人脸检测器内部使用。此分类器需要在运行时以正负样本来训练。

其初始框由用户指定，作为追踪的正样本，而在框范围之外许多其他patch都作为背景。

在新的一帧图像中，分类器在前一帧框的周围的每个像素上分类，并给出得分。

目标的新位置即得分最高的这样一来有新的正样本来重新训练分类器。

**MIL追踪**

算法与Boost很像，唯一的区别是，它会考虑当前标定框周围小部分框同时作为正样本，你可能认为这个想法比较烂，因为大部分的这些正样本其实目标并不在中心。

这就是MIL(Multiple Instance Learning)的独特之处，在MIL中你不需要指定正负样本，而是正负样包(bags)。在正样本包中的并不全是正样本，而是仅需要一个样本是正样本即可。当前示例中，正样本包里面的样本包含的是处于中心位置的框，以及中心位置周围的像素所形成的框。即便当前位置的跟踪目标不准确，从以当前位置为中心在周围像素抽取的样本框所构成的正样本包中，仍然有很大概率命中一个恰好处于中心位置的框。

**KCF 追踪**

KCF即Kernelized Correlation Filters,思路借鉴了前面两个。注意到MIL所使用的多个正样本之间存在交大的重叠区域。这些重叠数据可以引出一些较好的数学特性，这些特性同时可以用来构造更快更准确的分类器。

它的速度和精度都比MIL要高。



#####         8.2armor_finder



#### 9、show_images

