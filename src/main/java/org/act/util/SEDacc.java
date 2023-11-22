package org.act.util;

import net.sf.geographiclib.Geodesic;
import net.sf.geographiclib.GeodesicData;
import net.sf.geographiclib.GeodesicLine;
import net.sf.geographiclib.GeodesicMask;

public class SEDacc extends DisCalculator {

    public static void main(String[] args) {
        // testGeoLine();
        geoSED();
    }

    private static void testGeoLine() {
        // Print waypoints between JFK and SIN
        Geodesic geod = Geodesic.WGS84;
        double lat1 = 40.640, lon1 = -73.779, // JFK
                lat2 = 1.359, lon2 = 103.989; // SIN
        GeodesicLine line = geod.InverseLine(lat1, lon1, lat2, lon2,
                GeodesicMask.DISTANCE_IN | GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);
        double ds0 = 500e3; // Nominal distance between points = 500 km
        // The number of intervals
        int num = (int) (Math.ceil(line.Distance() / ds0));
        {
            // Use intervals of equal length
            double ds = line.Distance() / num;
            for (int i = 0; i <= num; ++i) {
                GeodesicData g = line.Position(i * ds, GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);
                System.out.println(i + " " + g.lat2 + " " + g.lon2);
            }
        }
        {
            // Slightly faster, use intervals of equal arc length
            double da = line.Arc() / num;
            for (int i = 0; i <= num; ++i) {
                GeodesicData g = line.ArcPosition(i * da, GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);
                System.out.println(i + " " + g.lat2 + " " + g.lon2);
            }
        }
    }

    private static void geoSED() {
        GPSPoint start = new GPSPoint();
        start.latitude = 51.484291;
        start.longitude = 0.001526;
        start.time = 1397382137127L;

        GPSPoint end = new GPSPoint();
        end.latitude = 51.481431;
        end.longitude = -0.007209;
        end.time = 1397382313126L;

        GPSPoint loc = new GPSPoint();
        loc.latitude = 51.481686;
        loc.longitude = -0.006361;
        loc.time = 1397382295120L;

        loc.latitude = 51.482436;
        loc.longitude = -0.004119;
        loc.time = 1397382251141L;

        Geodesic geod = Geodesic.WGS84;
        GeodesicLine line = geod.InverseLine(start.latitude, start.longitude, end.latitude, end.longitude,
                GeodesicMask.DISTANCE_IN | GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);
        double dist = 500e3; // Nominal distance between points = 500 km

        double k = 0;
        k = (double) (loc.time - start.time) / (end.time - start.time);
        System.out.println("propotion: " + k);

        dist = DisCalculator.getDistanceOfP2P(start, end);
        System.out.println("dist from start to end; " + dist);
        dist *= k;

        System.out.println("dist from start to sync; " + dist);

        GeodesicData g = line.Position(dist, GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);
        System.out.println("sync: " + " " + g.lat2 + " " + g.lon2);

        loc.latitude = g.lat2;
        loc.longitude = g.lon2;

        dist = DisCalculator.getDistanceOfP2P(start, loc);
        System.out.println("dist from start to sync; " + dist);

    }

    @Override
    public double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        GPSPoint sync = SEDacc.predict(lineStart, lineEnd, loc);
        double dist = DisCalculator.getDistanceOfP2P(loc, sync);
        return dist;
    }

    public static GPSPoint predict(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        double dist = 500e3; // Nominal distance between points = 500 km

        double k = 0;
        k = (double) (loc.time - lineStart.time) / (lineEnd.time - lineStart.time);

        if (k < 1e-9) {
            dist = DisCalculator.getDistanceOfP2P(lineStart, loc);
            return lineStart;
        } else if (1.0 - k < 1e-9) {
            dist = DisCalculator.getDistanceOfP2P(lineEnd, loc);
            return lineEnd;
        }


        dist = DisCalculator.getDistanceOfP2P(lineStart, lineEnd);
        dist *= k;

        Geodesic geod = Geodesic.WGS84;
        GeodesicLine line = geod.InverseLine(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
                lineEnd.longitude, GeodesicMask.DISTANCE_IN | GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);

        GeodesicData g = line.Position(dist, GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE);

        GPSPoint sync = new GPSPoint(g.lon2, g.lat2);
        sync.index = loc.index;
        sync.time = loc.time;

        return sync;
    }


    @Override
    public double getDistanceOfP2Line(GPSLine line, GPSPoint loc) {
        // TODO Auto-generated method stub
        GPSPoint lineStart = line.startPoint;
        GPSPoint lineEnd = line.endPoint;
        return getDistanceOfP2Line(lineStart, lineEnd, loc);
    }

}
