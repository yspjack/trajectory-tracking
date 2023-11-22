package org.act.util;

public class ErrorZone extends DisCalculator {

    @Override
    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        double angleLine = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude,
                lineEnd.latitude, lineEnd.longitude);

        double angleStartLoc = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);


        double dist = 0;

        angleStartLoc = DisCalculator.getIncludedAngle(angleLine, angleStartLoc);

        angleLine = DisCalculator.getAngleOfVector(lineEnd.latitude, lineEnd.longitude,
                lineStart.latitude, lineStart.longitude);
        double angleEndLoc = DisCalculator.getAngleOfVector(lineEnd.latitude, lineEnd.longitude, loc.latitude, loc.longitude);
        angleEndLoc = DisCalculator.getIncludedAngle(angleLine, angleEndLoc);


        if (Math.abs(angleStartLoc) > Math.PI / 2) {
            dist = DisCalculator.getDistanceOfP2P(lineStart, loc);
        } else if (Math.abs(angleEndLoc) > Math.PI / 2) {
            dist = DisCalculator.getDistanceOfP2P(lineEnd, loc);
        } else {
            DisCalculator ped = new PED();
            dist = ped.getDistanceOfP2Line(lineStart, lineEnd, loc);
        }

        return dist;

    }


    @Override
    public double getDistanceOfP2Line(GPSLine line, GPSPoint loc) {
        GPSPoint lineStart = line.getStartPoint();
        GPSPoint lineEnd = line.getEndPoint();
        return getDistanceOfP2Line(lineStart, lineEnd, loc);
    }

    @Override
    public String toString() {
        return "ERZONE";
    }

}
