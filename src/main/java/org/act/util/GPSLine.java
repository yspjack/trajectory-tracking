package org.act.util;

import java.util.Objects;

// 实际上记录了两条线
public class GPSLine {
    public int index = 1;                // 线段的编号，从1开始。

    public double length = 0;            // 矢量/线段 的长度
    public double angle = 0;            // 矢量/线段 的角度。

    public GPSPoint startPoint = null;
    public GPSPoint endPoint = null;

    ///////////////////////////////////////////////////////

    public int startIndex() {
        return this.getStartPoint().index;
    }

    public double startLatitude() {
        return this.getStartPoint().latitude;
    }

    public double startLongitude() {
        return this.getStartPoint().longitude;
    }

    public long startTime() {
        return this.getStartPoint().time;
    }

    public int endIndex() {
        return this.getEndPoint().index;
    }

    public double endLatitude() {
        return this.getEndPoint().latitude;
    }

    public double endLongitude() {
        return this.getEndPoint().longitude;
    }

    long endTime() {
        return this.getEndPoint().time;
    }

    @Override
    public String toString() {
        return "start: " + this.getStartPoint() + " \n end: " + this.getEndPoint() + "\n";
    }

    public GPSPoint getStartPoint() {
        return startPoint;
    }

    public void setStartPoint(GPSPoint startPoint) {
        this.startPoint = startPoint;
    }

    public GPSPoint getEndPoint() {
        return endPoint;
    }

    public void setEndPoint(GPSPoint endPoint) {
        this.endPoint = endPoint;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    @Override
    public boolean equals(Object arg0) {
        if (arg0 == this) {
            return true;
        }
        if (arg0 instanceof GPSLine) {
            GPSLine line = (GPSLine) arg0;
            boolean flag = true;
            flag = Objects.equals(this.startPoint, line.startPoint);
            flag = flag && Objects.equals(this.endPoint, line.endPoint);
            return flag;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return (startPoint.hashCode() + endPoint.hashCode());
    }

}