package org.act.util;


public class PED extends DisCalculator {

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "PED";
    }


    @Override
    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        // TODO Auto-generated method stub
        // TODO Auto-generated method stub
        double dis = 0;

        // 1.计算loc到line原点的距离。绝对值。
        double r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
        if (r == 0)
            return 0;

        double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
                lineEnd.longitude);
        // 2.计算line原点与loc构成直线的角度[0,2pi)
        double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

        // 3.计算距离
        dis = r * Math.sin(sita - lineAngle);

        return Math.abs(dis);
    }

    public double getDistanceOfP2Segment(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {

        double dis = 0;

        double r1 = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
        double r2 = getDistanceOfP2P(lineEnd.latitude, lineEnd.longitude, loc.latitude, loc.longitude);

        double PI2 = Math.PI / 2;
        if (r1 == 0 || r2 == 0)
            return 0;

        double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
                lineEnd.longitude);

        double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

        double sitaEnd = getAngleOfVector(lineEnd.latitude, lineEnd.longitude, loc.latitude, loc.longitude);


        double delta1 = Math.abs(lineAngle - sita);
        if ((2 * Math.PI - delta1) < delta1) {
            delta1 = 2 * Math.PI - delta1;
        }

        double delta2 = Math.abs(lineAngle - sitaEnd);
        if ((2 * Math.PI - delta2) < delta2) {
            delta2 = 2 * Math.PI - delta2;
        }


        if (delta1 < PI2 && delta2 < PI2) {
            return Math.abs(r2);
        }


        if (delta1 > PI2) {
            return Math.abs(r1);
        }

        dis = r1 * Math.sin(sita - lineAngle);

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
