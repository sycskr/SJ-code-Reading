//
// Created by xinyang on 19-3-27.
//
#define LOG_LEVEL LOG_NONE
#include <log.h>
#include <options.h>
#include <armor_finder/armor_finder.h>
#include <show_images/show_images.h>

/**
 * @name ArmorFinder::stateTrackingTarget
 * @param cv::Mat &src 当前图像
 * @func 根据上一帧目标的位置得出这一帧的目标
 * @return bool 追踪失败，返回false。
*/
bool ArmorFinder::stateTrackingTarget(cv::Mat &src) {
    auto pos = target_box.rect;
    if(!tracker->update(src, pos)){ // 使用KCFTracker进行追踪
        target_box = ArmorBox();//?????????????????????????????????
        LOGW("Track fail!");
        return false;//追踪失败
    }
    //Rect2d(0, 0, 640, 480) x,y,width,height
    if((pos & cv::Rect2d(0, 0, 640, 480)) != pos){
        target_box = ArmorBox();//?????????????????????????????????
        LOGW("Track out range!");
        return false;//超出范围
    }

    // 获取相较于追踪区域两倍长款的区域，用于重新搜索，获取灯条信息
    cv::Rect2d bigger_rect;
    bigger_rect.x = pos.x - pos.width / 2.0;
    bigger_rect.y = pos.y - pos.height / 2.0;
    bigger_rect.height = pos.height * 2;
    bigger_rect.width  = pos.width * 2;
    bigger_rect &= cv::Rect2d(0, 0, 640, 480);
    cv::Mat roi = src(bigger_rect).clone();

    ArmorBox box;

    //由于KCFTracker在追踪的时候，只改变追踪框的位置，而不改变长宽。但在后续数据发送时需要长宽信息进行距离的估算。
    //并且由于KCF算法是根据上一帧更新下一帧，所以当追踪时间长了，可能会忘记一开始的目标追到其他地方去。
    //所以在区域内重新搜索。
    if(findArmorBox(roi, box)) { // 如果成功获取目标，则利用搜索区域重新更新追踪器
        target_box = box;
        //target_box　添加roi偏移量
        target_box.rect.x += bigger_rect.x; 
        target_box.rect.y += bigger_rect.y;
        //blob　添加roi偏移量
        for(auto &blob : target_box.light_blobs){
            blob.rect.center.x += bigger_rect.x;
            blob.rect.center.y += bigger_rect.y;
        }
        tracker = TrackerToUse::create();
        tracker->init(src, target_box.rect);
    }else{    // 如果没有成功搜索目标，则使用判断是否跟丢。
        roi = src(pos).clone();
        if(classifier){ // 分类器可用，使用分类器判断。
            cv::resize(roi, roi, cv::Size(48, 36));
            if(classifier(roi) == 0){
                target_box = ArmorBox();//?????????????????????????????????
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
                target_box = ArmorBox();//?????????????????????????????????
                return false;
            }
        }
        target_box.rect = pos;
        target_box.light_blobs.clear();
    }
    return true;
}
