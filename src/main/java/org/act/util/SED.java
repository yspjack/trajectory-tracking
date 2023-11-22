package org.act.util;

public class SED extends DisCalculator {
    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "SED";
    }

    @Override
    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        // TODO Auto-generated method stub
        GPSPoint tmp = predict(lineStart, lineEnd, loc); // 计算同步点
        double dis = (getDistanceOfP2P(tmp.latitude, tmp.longitude, loc.latitude, loc.longitude));
        return Math.abs(dis);
    }

    @Override
    public double getDistanceOfP2Line(GPSLine line, GPSPoint loc) {
        // TODO Auto-generated method stub
        GPSPoint lineStart = line.getStartPoint();
        GPSPoint lineEnd = line.getEndPoint();
        return getDistanceOfP2Line(lineStart, lineEnd, loc);
    }


}
