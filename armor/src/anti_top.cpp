//
// Created by xinyang on 19-7-15.
//

#include <armor_finder/armor_finder.h>
#include <additions.h>
#include <log.h>

/*
 * name @ mean
 * param @ 输入陀螺周期循环队列
 * func @ 遍历整个环形队列，求环形队列内部总共的和，得出环形队列平均值
 * return @ 环形队列的平均值（double）
*/
template<int length>
static double mean(RoundQueue<double, length> &vec) {
    double sum = 0;
    //遍历整个环形队列
    for (int i = 0; i < vec.size(); i++) {
        //求环形队列内部总共的和
        sum += vec[i];
    }
    //得出环形队列平均值
    return sum / length;
}

/**
 * name @ getFrontTime
 * params @ time_seq一个周期内的时间采样点
 *          angle_seq一个周期内的角度采样点
 * func @ 通过线性拟合计算出角度为0时对应的时间点
 * return @ 角度为0时对应的时间点
*/
static systime getFrontTime(const vector<systime> time_seq, const vector<float> angle_seq) {
    double A = 0, B = 0, C = 0, D = 0;
    int len = time_seq.size();
    for (int i = 0; i < len; i++) {
        A += angle_seq[i] * angle_seq[i];
        B += angle_seq[i];
        C += angle_seq[i] * time_seq[i];
        D += time_seq[i];
        cout << "(" << angle_seq[i] << ", " << time_seq[i] << ") ";
    }
    double b = (A * D - B * C) / (len * A - B * B);
    cout << b << endl;
    return b;
}

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
void ArmorFinder::antiTop() {
    if (target_box.rect == cv::Rect2d()) return;
    // 判断是否发生装甲目标切换。
    // 记录切换前一段时间目标装甲的角度和时间  
    if (getPointLength(last_box.getCenter() - target_box.getCenter()) > last_box.rect.height * 1.5) {
        // 通过线性拟合计算出角度为0时对应的时间点
        auto front_time = getFrontTime(time_seq, angle_seq);
        // 通过两次装甲角度为零的时间差计算陀螺旋转周期
        auto once_periodms = getTimeIntervalms(front_time, last_front_time);
//        if (abs(once_periodms - top_periodms[-1]) > 50) {
//            sendBoxPosition(0);
//            return;
//        }
        LOGM(STR_CTR(WORD_GREEN, "Top period: %.1lf"), once_periodms);//生成矩阵？？
        top_periodms.push(once_periodms);
        auto periodms = mean(top_periodms);
        systime curr_time;
        getsystime(curr_time);
        // 根据旋转周期计算下一次装甲出现在角度为零的时间点
        uint16_t shoot_delay = front_time + periodms * 2 - curr_time;//射击延迟时间
        //anti_top_cnt采样次数小于4
        if (anti_top_cnt < 4) {
            sendBoxPosition(0);// 和主控板通讯 发送当前时间的方框位置
        } else if (abs(once_periodms - top_periodms[-1]) > 50) {
            sendBoxPosition(0);
        } else {
            sendBoxPosition(shoot_delay);// 和主控板通讯 发送下次预测时间的方框位置
        }
        time_seq.clear();
        angle_seq.clear();
        last_front_time = front_time;
    } else {
        time_seq.emplace_back(frame_time);
        //dx 目标框中心与图像中心的水平间距
        double dx = target_box.rect.x + target_box.rect.width / 2 - IMAGE_CENTER_X;
        //yaw 旋转的弧度
        double yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
        angle_seq.emplace_back(yaw);
        sendBoxPosition(0);
    }
    anti_top_cnt++;
}

