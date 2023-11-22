package org.act.util;

public class GPSPoint {
    public int index = 0;
    public int lineId = 0;

    public double longitude;
    public double latitude;
    public long time;
    public double altitude;


    public GPSPoint() {
        super();
    }


    public GPSPoint(double longitude, double latitude, long time) {

        super();
        this.longitude = longitude;
        this.latitude = latitude;
        this.time = time;
    }


    public GPSPoint(double longitude, double latitude) {
        super();
        this.longitude = longitude;
        this.latitude = latitude;
        this.time = 0;
    }


    @Override
    public String toString() {
        return "lat: " + this.latitude + " long: " + this.longitude + " time: " + this.time;
    }

    @Override
    public boolean equals(Object arg0) {
        // TODO Auto-generated method stub
        if (arg0 == this) {
            return true;
        }
        if (arg0 instanceof GPSPoint) {
            GPSPoint point = (GPSPoint) arg0;
            boolean flag = Math.abs(this.latitude - point.latitude) < 1e-15;
            flag = flag && (Math.abs(this.longitude - point.longitude) < 1e-15);
            flag = flag && (time == point.time);
            return flag;
        }
        return super.equals(arg0);
    }

    @Override
    public int hashCode() {
        return Double.hashCode(latitude) + Double.hashCode(longitude) + Long.hashCode(time);
    }

}
