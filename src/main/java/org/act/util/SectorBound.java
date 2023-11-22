package org.act.util;

import org.act.traj.TrajectoryCompressor;

import java.util.Optional;

public class SectorBound {
    double angleU;
    double angleL;

    public SectorBound(double angleL, double angleU) {
        super();
        this.angleU = angleU;
        this.angleL = angleL;
    }

    public SectorBound(double angleL, double angleU, boolean deg) {
        if (deg) {
            angleL = TrajectoryCompressor.rad(angleL);
            angleU = TrajectoryCompressor.rad(angleU);
        }

        this.angleU = angleU;
        this.angleL = angleL;
    }

    public SectorBound() {
        angleU = 0;
        angleL = 0;

    }

    public double getAngleU() {
        return angleU;
    }

    public void setAngleU(double angleU) {
        this.angleU = angleU;
    }

    public double getAngleL() {
        return angleL;
    }

    public void setAngleL(double angleL) {
        this.angleL = angleL;
    }

    public Optional<SectorBound> intersects(Optional<SectorBound> s) {

        Optional<SectorBound> result;

        result = s.map(v -> {

            SectorBound reSectorBound = new SectorBound(this.angleL, this.angleU);

            if (this.angleU < 0) {
                reSectorBound = new SectorBound(v.angleL, v.angleU);
                return reSectorBound;
            }


            if (v.getAngleL() < 0) {
                reSectorBound = new SectorBound(angleL, angleU);
                return reSectorBound;
            }


            double delta = 0;


            delta = angleU - v.angleU;
            if ((delta > 0.5 * Math.PI && delta < 1.5 * Math.PI)
                    || (delta < -0.5 * Math.PI && delta > -1.5 * Math.PI)) {
                return null;

            }
            if (delta <= -1 * Math.PI || (delta > 0 && delta <= 1 * Math.PI)) {
                reSectorBound.setAngleU(v.angleU);
            }

            delta = angleL - v.angleL;
            if ((delta > 0.5 * Math.PI && delta < 1.5 * Math.PI)
                    || (delta < -0.5 * Math.PI && delta > -1.5 * Math.PI)) {
                return null;
            }
            if (delta > 1 * Math.PI || (delta <= 0 && delta > -1 * Math.PI)) {
                reSectorBound.setAngleL(v.angleL);
            }


            delta = reSectorBound.getAngleU() - reSectorBound.getAngleL();

            if ((delta < 0 && delta > -Math.PI) || (delta > Math.PI)) {
                return null;
            }


            return reSectorBound;
        });
        return result;

    }

    public boolean contains(double angle) {

        // full plane , always true
        if (angleL < 0) {
            return true;
        }

        if (angleU > angleL) {
            if (angle < angleU && angle > angleL) {
                return true;
            } else {
                return false;
            }
        } else {
            if (angle < angleU || angle > angleL) {
                return true;
            } else {
                return false;
            }
        }

    }

    public boolean contains(GPSPoint start, GPSPoint end) {

        double angle = TrajectoryCompressor.getAngleOfVector(start.latitude, start.longitude, end.latitude, end.longitude); // [0,

        return this.contains(angle);

    }

    public static SectorBound getSector(GPSPoint start, GPSPoint end, double threshold) {
        double len = TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude, end.latitude, end.longitude);

        if (len < threshold / 2) {
            return new SectorBound(-1, -1);
        }

        double angle_i = TrajectoryCompressor.getAngleOfVector(start.latitude, start.longitude, end.latitude, end.longitude); // [0,
        double deltaAngle = Math.asin(0.5 * threshold / len);
        double angleU_i = (angle_i + deltaAngle) % (2 * Math.PI); // (0, 2pi);
        double angleL_i = (angle_i - deltaAngle + 2 * Math.PI) % (2 * Math.PI); // (0, 2pi);

        return new SectorBound(angleL_i, angleU_i);

    }

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        StringBuilder sBuilder = new StringBuilder();
        sBuilder.append("lower : ").append(DisCalculator.degree(angleL));
        sBuilder.append("  upper : ").append(DisCalculator.degree(angleU));
        return sBuilder.toString();
    }


    public static void main(String[] args) {
        SectorBound sb1 = new SectorBound(30, 60, true);

        SectorBound sb2 = new SectorBound(100, 200, true);

        Optional<SectorBound> res1;
        Optional<SectorBound> res2;

        res1 = sb1.intersects(Optional.ofNullable(sb2));
        res2 = sb2.intersects(Optional.ofNullable(sb1));

        sb1 = new SectorBound(30, 60, true);
        sb2 = new SectorBound(300, 320, true);

        res1 = sb1.intersects(Optional.ofNullable(sb2));
        res2 = sb2.intersects(Optional.ofNullable(sb1));

    }

}