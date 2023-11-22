package org.act.util;


/**
 * direction-aware distance.
 * @author jiangjh
 *
 */

public class DAD extends DisCalculator {

    @Override
    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc, GPSPoint suc) {

        // TODO Auto-generated method stub

        double alpha = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);

        double beta = DisCalculator.getAngleOfVector(loc.latitude, loc.longitude, suc.latitude, suc.longitude);
        double theta = Math.abs(alpha - beta);
        if ((2 * Math.PI - theta) < theta) {
            theta = 2 * Math.PI - theta;
        }

        theta = DisCalculator.degree(theta);
        return theta;
    }


    public double getDistanceOfP2Line(double alpha, GPSPoint loc, GPSPoint suc) {
        // TODO Auto-generated method stub
        double beta = DisCalculator.getAngleOfVector(loc.latitude, loc.longitude, suc.latitude, suc.longitude);
        double theta = Math.abs(alpha - beta);
        if ((2 * Math.PI - theta) < theta) {
            theta = 2 * Math.PI - theta;
        }
        theta = DisCalculator.degree(theta);

        return theta;
    }


    public double getDistanceOfP2Line(GPSLine line, GPSPoint loc, GPSPoint suc) {
        // TODO Auto-generated method stub
        GPSPoint lineStart = line.startPoint;
        GPSPoint lineEnd = line.endPoint;
        return getDistanceOfP2Line(lineStart, lineEnd, loc, suc);

    }


    @Override
    public double getDistanceOfP2Line(GPSLine line, GPSPoint loc) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public String toString() {
        return "DAD";
    }

}
