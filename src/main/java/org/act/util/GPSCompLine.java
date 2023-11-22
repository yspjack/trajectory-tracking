package org.act.util;

// 实际上记录了两条线
public class GPSCompLine {
    int index = 1;                // 线段的编号，从1开始。
    int points = 0;                // 包含的原始轨迹点数量，含(inactive). For Debug
    int actPoints = 0;            // 包含的active点数量.

    // R/R'的起点
    GPSPoint startPoint = null;

    //////////////////////////////////////////////////////////
    // 压缩过程中保存最新的点。一个线段压缩结束的时候保存的是最终需要输出的R(历史原因，继承其他算法的输出方式)
    GPSPoint endPointR = null;
    double lengthR = 0;            // 矢量的长度
    double angleR = 0;            // 矢量R的角度。

    //////////////////////////////////////////////////////////
    // 最后一个有效点 或 最长R的端点；输出的时候需要保存到R(历史原因，继承其他算法的输出方式)
    GPSPoint endPointRL = null;
    double lengthRL = 0;            // 矢量的长度
    double angleRL = 0;            // 矢量R'的角度

    //////////////////////////////////////////////////////////
    // R'的端点
    GPSPoint endPointRF = null;
    double lengthRF = 0;            // 矢量的长度
    double angleRF = 0;            // 矢量R'的角度

    ///////////////////////////////////////////////////////////
    double max_p_dis = 0; // 子轨迹所有点的 最大距离（最大正的距离）, to R'
    double max_n_dis = 0; // 子轨迹所有点的 最小距离（最大负的距离）, to R'

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public GPSPoint getStartPoint() {
        return startPoint;
    }

    public void setStartPoint(GPSPoint startPoint) {
        this.startPoint = startPoint;
    }

    public int getPoints() {
        return points;
    }

    public void setPoints(int points) {
        this.points = points;
    }

}