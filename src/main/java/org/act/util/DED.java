package org.act.util;

public class DED extends DisCalculator {
    private DisCalculator dc = new PED();
    static double r;
    static double vDis;

    // static boolean isVertical = false;

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "DED";
    }

    @Override
    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        // TODO Auto-generated method stub
        // TODO Auto-generated method stub

        // GPSPoint tmp = predict(lineStart, lineEnd, loc);

        // double verticalDis = getVerticalDis(lineStart, lineEnd, loc);

        // double dis = 0;
// 1.计算loc到line原点的距离。绝对值。
        // isVertical = true;
        r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
        if (r == 0)
            return 0;

        double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
                lineEnd.longitude);
        // 2.计算line原点与loc构成直线的角度[0,2pi)
        double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

        // 3.计算距离
        double verticalDis = Math.abs(r * Math.sin(sita - lineAngle));
        // double verticalDis = (r*Math.sin(sita - lineAngle));
        // verticalDis = verticalDis*verticalDis;

        double dis = r * Math.cos(sita - lineAngle);
        dis = Math.abs(dis);

        GPSPoint tmp = predict(lineStart, lineEnd, loc);
        double distmp = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, tmp.latitude, tmp.longitude);
        distmp = Math.abs(distmp);

        // double HorizontalDis = dis*dis + distmp*distmp - 2*dis*distmp;

        double HorizontalDis = Math.abs(dis - distmp);
        // double HorizontalDis = getHorizontalDis(lineStart, lineEnd, loc);

        // ratio = threshold2 / threshold;
        double newVDis = HorizontalDis / ratio;
        if (newVDis > verticalDis) {
            verticalDis = newVDis;

        }
        // else {
        // HorizontalDis = verticalDis * ratio;
        // }

        return Math.abs(verticalDis);
        // return getDistanceOfP2P(loc.latitude, loc.longitude, tmp.latitude,
        // tmp.longitude);

    }

    @Override
    public double getDistanceOfP2Line(GPSLine line, GPSPoint loc) {
        // TODO Auto-generated method stub
        GPSPoint lineStart = line.getStartPoint();
        GPSPoint lineEnd = line.getEndPoint();
        return getDistanceOfP2Line(lineStart, lineEnd, loc);

    }

    public double getRadialDis(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
        if (r == 0)
            return 0;

        double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
                lineEnd.longitude);
        // 2.计算line原点与loc构成直线的角度[0,2pi)
        double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

        // 3.计算距离

        double dis = r * Math.cos(sita - lineAngle);
        dis = Math.abs(dis);

        GPSPoint tmp = predict(lineStart, lineEnd, loc);
        double distmp = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, tmp.latitude, tmp.longitude);
        distmp = Math.abs(distmp);

        // double HorizontalDis = dis*dis + distmp*distmp - 2*dis*distmp;

        double HorizontalDis = Math.abs(dis - distmp);

        return HorizontalDis;
    }

    public double getRadialDis(GPSLine line, GPSPoint loc) {
        GPSPoint lineStart = line.getStartPoint();
        GPSPoint lineEnd = line.getEndPoint();
        return getRadialDis(lineStart, lineEnd, loc);

    }

    public double getPerpendicularDis(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
        if (r == 0)
            return 0;

        double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
                lineEnd.longitude);
        // 2.计算line原点与loc构成直线的角度[0,2pi)
        double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

        // 3.计算距离
        double verticalDis = Math.abs(r * Math.sin(sita - lineAngle));

        return verticalDis;
    }

    public double getPerpendicularDis(GPSLine line, GPSPoint loc) {
        GPSPoint lineStart = line.getStartPoint();
        GPSPoint lineEnd = line.getEndPoint();
        return getPerpendicularDis(lineStart, lineEnd, loc);

    }


}
