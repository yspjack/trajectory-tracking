package org.act.util.polygon;

import net.sf.geographiclib.Geodesic;
import net.sf.geographiclib.GeodesicData;
import org.act.traj.TrajectoryCompressor;
import org.act.util.DisCalculator;
import org.act.util.GPSPoint;

import static java.lang.Math.*;

public class Point {
    public double x, y, z;

    public Point() {

    }

    /**
     * the x-y plane coordinates of the gpspoint p
     *
     * @param p
     * @param start
     */
    public Point(GPSPoint p, GPSPoint start) {

        // double threshold = 1e18;

        double latS = start.latitude;
        double latE = p.latitude;
        double lngE = p.longitude;
        double lngS = start.longitude;
        double angle = 0;

        double high = TrajectoryCompressor.getDistanceOfP2P(latE, lngE, latS, lngE);
        double xLength = TrajectoryCompressor.getDistanceOfP2P(latS, lngS, latS, lngE);

        if (xLength == 0 && high == 0) {
            angle = 0;
            x = 0;
            y = 0;
            return;
        }

        if (latS == latE && lngS == lngE) {
            angle = 0;
            x = 0;
            y = 0;
            return;
        } else if (latS == latE && lngS < lngE) {
            // return 0;
            angle = 0;
            y = 0;
            x = Math.abs(xLength);
            return;
        } else if (latS == latE && lngS > lngE) {
            // return Math.PI;
            angle = Math.PI;
            y = 0;
            x = -Math.abs(xLength);
            return;
        } else if (latS < latE && lngS == lngE) {
            angle = Math.PI / 2;
            x = 0;
            y = Math.abs(high);
            return;
        } else if (latS > latE && lngS == lngE) {
            angle = 3.0 * Math.PI / 2;
            x = 0;
            y = -Math.abs(high);
            return;
        }

        if (latE > latS && lngE > lngS) {
            x = Math.abs(xLength);
            y = Math.abs(high);
            return;
        } else if (latE > latS && lngE < lngS) {
            x = -Math.abs(xLength);
            y = Math.abs(high);
            return;
        } else if (latE < latS && lngE > lngS) {
            x = Math.abs(xLength);
            y = -Math.abs(high);
            return;
        } else if (latE < latS && lngE < lngS) {
            x = -Math.abs(xLength);
            y = -Math.abs(high);
            return;
        }

        // double dist = TrajectoryCompressor.getDistanceOfP2P(p.latitude, p.longitude,
        // start.latitude, start.longitude);
        // dist = Math.abs(dist);
        // double theta = TrajectoryCompressor.getAngleOfVector(p.latitude, p.longitude,
        // start.latitude, start.longitude);
        // x = dist*Math.cos(theta);
        // y = dist*Math.sin(theta);

    }

    /**
     * ����ת����
     *
     * @param p
     * @param start
     * @return
     */
    public boolean transform(GPSPoint p, GPSPoint start, double threshold) {

        double latS = start.latitude;
        double latE = p.latitude;
        double lngE = p.longitude;
        double lngS = start.longitude;
        double angle = 0;

        // double dist = TrajectoryCompressor.getDistanceOfP2P(p.latitude, p.longitude,
        // start.latitude, start.longitude);
        // dist = Math.abs(dist);
        // double theta = TrajectoryCompressor.getAngleOfVector(p.latitude, p.longitude,
        // start.latitude, start.longitude);
        // x = dist*Math.cos(theta);
        // y = dist*Math.sin(theta);
        // double high = Math.abs(y);
        // double xLength = Math.abs(x);
        // double length = TrajectoryCompressor.getDistanceOfP2P(latS, lngS, latE,
        // lngE);

        double high = TrajectoryCompressor.getDistanceOfP2P(latE, lngE, latS, lngE);
        double xLength = TrajectoryCompressor.getDistanceOfP2P(latS, lngS, latS, lngE);

        // ������볬����ֵ����ֹѹ��
        if ((high * high + xLength * xLength) > threshold * threshold) {
            x = 0;
            y = 0;
            return false;
        }

        if (xLength == 0 && high == 0) {
            angle = 0;
            x = 0;
            y = 0;
            return true;
        }

        if (latS == latE && lngS == lngE) {
            angle = 0;
            x = 0;
            y = 0;
            return true;
        } else if (latS == latE && lngS < lngE) {
            // return 0;
            angle = 0;
            y = 0;
            x = Math.abs(xLength);
            return true;
        } else if (latS == latE && lngS > lngE) {
            // return Math.PI;
            angle = Math.PI;
            y = 0;
            x = -Math.abs(xLength);
            return true;
        } else if (latS < latE && lngS == lngE) {
            angle = Math.PI / 2;
            x = 0;
            y = Math.abs(high);
            return true;
        } else if (latS > latE && lngS == lngE) {
            angle = 3.0 * Math.PI / 2;
            x = 0;
            y = -Math.abs(high);
            return true;
        }

        if (latE > latS && lngE > lngS) {
            x = Math.abs(xLength);
            y = Math.abs(high);
            return true;
        } else if (latE > latS && lngE < lngS) {
            x = -Math.abs(xLength);
            y = Math.abs(high);
            return true;
        } else if (latE < latS && lngE > lngS) {
            x = Math.abs(xLength);
            y = -Math.abs(high);
            return true;
        } else if (latE < latS && lngE < lngS) {
            x = -Math.abs(xLength);
            y = -Math.abs(high);
            return true;
        }
        return true;

    }

    public boolean transformGeo(GPSPoint p, GPSPoint start, double threshold) {

        GeodesicData g = Geodesic.WGS84.Inverse(start.latitude, start.longitude, p.latitude, p.longitude);
        double angle = g.azi1;
        double dist = g.s12;

        x = dist * Math.cos(angle);
        y = dist * Math.sin(angle);

        return true;

    }

    enum Mode {
        NORTH_POLAR, SOUTH_POLAR, EQUATORIAL, OBLIQUE
    }

    public boolean transformNormalized(GPSPoint p, GPSPoint start, double threshold) {
        double lambda, phi;
        lambda = toRadians(p.longitude);
        phi = toRadians(p.latitude);
        double sinphi = Math.sin(phi);
        double cosphi = Math.cos(phi);
        double coslam = Math.cos(lambda);

        double sinph0 = Math.sin(toRadians(start.latitude));
        double cosph0 = Math.cos(toRadians(start.latitude));
        double latitudeOfOrigin = toRadians(start.latitude);
        double HALF_PI = Math.PI / 2;
        double EPS10 = 1.e-10;
        double TOL = 1.e-14;

        final Mode mode;

        if (Math.abs(latitudeOfOrigin - HALF_PI) < EPS10) {
            mode = Mode.NORTH_POLAR;
            sinph0 = 1;
            cosph0 = 0;
        } else if (abs(latitudeOfOrigin + HALF_PI) < EPS10) {
            mode = Mode.SOUTH_POLAR;
            sinph0 = -1;
            cosph0 = 0;
        } else if (abs(latitudeOfOrigin) < EPS10) {
            mode = Mode.EQUATORIAL;
            sinph0 = 0;
            cosph0 = 1;
        } else {
            mode = Mode.OBLIQUE;
            sinph0 = Math.sin(latitudeOfOrigin);
            cosph0 = Math.cos(latitudeOfOrigin);
        }

        switch (mode) {
            case EQUATORIAL:
            case OBLIQUE:
                if (mode == Mode.EQUATORIAL) {
                    y = cosphi * coslam;
                } else { // Oblique
                    y = sinph0 * sinphi + cosph0 * cosphi * coslam;
                }
                if (abs(abs(y) - 1) < TOL) {
                    if (y < 0) {
                    } else {
                        x = 0;
                        y = 0;
                    }
                } else {
                    y = Math.acos(y);
                    y /= Math.sin(y);
                    x = y * cosphi * Math.sin(lambda);
                    y *= (mode == Mode.EQUATORIAL) ? sinphi : (cosph0 * sinphi - sinph0 * cosphi * coslam);
                }
                break;
            case NORTH_POLAR:
                phi = -phi;
                coslam = -coslam;
            case SOUTH_POLAR:
                if (Math.abs(phi - HALF_PI) < EPS10) {
                }
                y = HALF_PI + phi;
                x = y * sin(lambda);
                y *= coslam;
                break;
        }
        return true;
    }

    public boolean transform2ENU(GPSPoint p, GPSPoint start, double threshold) {
        double lat = p.latitude;
        double lon = p.longitude;
        double lat0 = start.latitude;
        double lon0 = start.longitude;
        double dis = DisCalculator.getDistanceOfP2P(start, p);

        if (dis > threshold) {
            return false;
        }

        EarthEllipsoid ell = new EarthEllipsoid();
        Point res = geodetic2enu(lat, lon, 0, lat0, lon0, 0, ell, true);
        x = res.x;
        y = res.y;
        return true;
    }

    private Point geodetic2enu(double lat, double lon, double h, double lat0, double lon0, double h0,
                               EarthEllipsoid ell, boolean deg) {
        Point point1 = geodetic2ecef(lat, lon, h, ell, deg);
        Point point2 = geodetic2ecef(lat0, lon0, h0, ell, deg);
        double dx = point1.x - point2.x;
        double dy = point1.y - point2.y;
        double dz = point1.z - point2.z;
        return uvw2enu(dx, dy, dz, lat0, lon0, deg);
    }

    private double get_radius_normal(double lat_radians, EarthEllipsoid ell) {
        double a = ell.a;
        double b = ell.b;
        return a * a / sqrt(
                a * a * (cos(lat_radians)) * (cos(lat_radians)) + b * b * (sin(lat_radians)) * (sin(lat_radians)));

    }

    private Point geodetic2ecef(double lat, double lon, double alt, EarthEllipsoid ell, boolean deg) {
        if (deg) {
            lat = toRadians(lat);
            lon = toRadians(lon);
        }
        // # radius of curvature of the prime vertical section
        double N = get_radius_normal(lat, ell);
        // # Compute cartesian (geocentric) coordinates given (curvilinear) geodetic
        // # coordinates.
        double x = (N + alt) * cos(lat) * cos(lon);
        double y = (N + alt) * cos(lat) * sin(lon);
        double z = (N * (ell.b / ell.a) * (ell.b / ell.a) + alt) * sin(lat);
        Point res = new Point(x, y);
        res.z = z;
        return res;
    }

    private Point uvw2enu(double u, double v, double w, double lat0, double lon0, boolean deg) {
        if (deg) {
            lat0 = toRadians(lat0);
            lon0 = toRadians(lon0);
        }
        double t = cos(lon0) * u + sin(lon0) * v;
        double East = -sin(lon0) * u + cos(lon0) * v;
        double Up = cos(lat0) * t + sin(lat0) * w;
        double North = -sin(lat0) * t + cos(lat0) * w;
        Point res = new Point(East, North);
        res.z = Up;
        return res;
    }

    private static class EarthEllipsoid {
        double a = 6378137; // semi-major axis [m]
        double f = 1 / 298.2572235630; //
        double b = a * (1 - f); // semi-minor axis

        private EarthEllipsoid() {

        }

    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /*
     * The signed area of the triangle det. by a,b,c; pos. if ccw, neg. if cw
     */
    public double Area2(Point a, Point b, Point c) {
        double area = ((c.x - b.x) * (a.y - b.y)) - ((a.x - b.x) * (c.y - b.y));
        return area;
    }

    /**
     * ba x ca的方向
     * @param a
     * @param b
     * @param c
     * @return
     */
    public int AreaSign(Point a, Point b, Point c) {
        double area2;

        area2 = (b.x - a.x) * (double) (c.y - a.y) - (c.x - a.x) * (double) (b.y - a.y);

        return Double.compare(area2, 0);

        /* The area should be an integer. */
        // if ( area2 > 0 ) return 1;
        // else if ( area2 < 0 ) return -1;
        // else return 0;
    }

    /*---------------------------------------------------------------------
     *Returns true iff c is strictly to the left of the directed
     *line through a to b.
     */
    public boolean Left(Point a, Point b, Point c) {
        return AreaSign(a, b, c) > 0;
    }

    public boolean LeftOn(Point a, Point b, Point c) {
        return AreaSign(a, b, c) >= 0;
    }

    public boolean Collinear(Point a, Point b, Point c) {
        return AreaSign(a, b, c) == 0;
    }

    /*---------------------------------------------------------------------
      SegSegInt: Finds the point of intersection p between two closed
      segments ab and cd.  Returns p and a char with the following meaning:
      'e': The segments collinearly overlap, sharing a point.
      'v': An endpoint (vertex) of one segment is on the other segment,
      but 'e' doesn't hold.
      '1': The segments intersect properly (i.e., they share a point and
      neither 'v' nor 'e' holds).
      '0': The segments do not intersect (i.e., they share no points).
      Note that two collinear segments that share just one point, an endpoint
      of each, returns 'e' rather than 'v' as one might expect.
      ---------------------------------------------------------------------*/
    public char SegSegInt(Point a, Point b, Point c, Point d, Point p, Point q) {
        double s, t; /* The two parameters of the parametric eqns. */
        double num, denom; /* Numerator and denoninator of equations. */
        char code = '?'; /* Return char characterizing intersection. */
        p.x = p.y = 100.0; /* For testing purposes only... */

        denom = a.x * (double) (d.y - c.y) + b.x * (double) (c.y - d.y) + d.x * (double) (b.y - a.y)
                + c.x * (double) (a.y - b.y);

        /* If denom is zero, then segments are parallel: handle separately. */
        if (denom == 0.0)
            return ParallelInt(a, b, c, d, p, q);

        num = a.x * (double) (d.y - c.y) + c.x * (double) (a.y - d.y) + d.x * (double) (c.y - a.y);
        if ((num == 0.0) || (num == denom))
            code = 'v';
        s = num / denom;
        // System.out.println("SegSegInt: num=" + num + ",denom=" + denom + ",s="+s);

        num = -(a.x * (double) (c.y - b.y) + b.x * (double) (a.y - c.y) + c.x * (double) (b.y - a.y));
        if ((num == 0.0) || (num == denom))
            code = 'v';
        t = num / denom;
        // System.out.println("SegSegInt: num=" +num + ",denom=" + denom + ",t=" + t);

        if ((0.0 < s) && (s < 1.0) && (0.0 < t) && (t < 1.0))
            code = '1';
        else if ((0.0 > s) || (s > 1.0) || (0.0 > t) || (t > 1.0))
            code = '0';

        p.x = a.x + s * (b.x - a.x);
        p.y = a.y + s * (b.y - a.y);

        return code;
    }

    /**
     * 求一条直线(斜率为k，截距为b)与两点(a,b)构成的线段的交点
     * @param a
     * @param c
     * @param k
     * @param b
     * @return
     */
    public static Point lineIntersects(Point a, Point c, long k, double b) {
        double k_old = (c.y - a.y) / (c.x - a.x);
        double b_old = (a.y - k_old * a.x);
        double x = (b - b_old) / (k_old - k);
        double y = k * x + b;
        Point res = new Point(x, y);
        return res;
    }

    public char ParallelInt(Point a, Point b, Point c, Point d, Point p, Point q) {
        if (!a.Collinear(a, b, c))
            return '0';

        if (Between1(a, b, c) && Between1(a, b, d)) {
            Assigndi(p, c);
            Assigndi(q, d);
            return 'e';
        }
        if (Between1(c, d, a) && Between1(c, d, b)) {
            Assigndi(p, a);
            Assigndi(q, b);
            return 'e';
        }
        if (Between1(a, b, c) && Between1(c, d, b)) {
            Assigndi(p, c);
            Assigndi(q, b);
            return 'e';
        }
        if (Between1(a, b, c) && Between1(c, d, a)) {
            Assigndi(p, c);
            Assigndi(q, a);
            return 'e';
        }
        if (Between1(a, b, d) && Between1(c, d, b)) {
            Assigndi(p, d);
            Assigndi(q, b);
            return 'e';
        }
        if (Between1(a, b, d) && Between1(c, d, a)) {
            Assigndi(p, d);
            Assigndi(q, a);
            return 'e';
        }
        return '0';
        /*
         * if ( Between1( a, b, c ) ) { Assigndi( p, c ); return 'e'; } if ( Between1(
         * a, b, d ) ) { Assigndi( p, d ); return 'e'; } if ( Between1( c, d, a ) ) {
         * Assigndi( p, a ); return 'e'; } if ( Between1( c, d, b ) ) { Assigndi( p, b
         * ); return 'e'; } return '0';
         */
    }

    public void Assigndi(Point p, Point a) {
        p.x = a.x;
        p.y = a.y;
    }

    /*---------------------------------------------------------------------
      Returns TRUE iff point c lies on the closed segement ab.
      Assumes it is already known that abc are collinear.
      (This is the only difference with Between().)
      ---------------------------------------------------------------------*/
    public boolean Between1(Point a, Point b, Point c) {
        Point ba, ca;

        /* If ab not vertical, check betweenness on x; else on y. */
        if (a.x != b.x)
            return ((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x));
        else
            return ((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y));
    }

    public static double getAngle(Point a) {
        double theta = (Math.atan2(a.y, a.x) + Math.PI * 2) % (Math.PI * 2);
        return theta;
    }

    public void PrintPoint() {
        System.out.println(" (" + x + "," + y + ")");
    }

    @Override
    public String toString() {
        // TODO Auto-generated method stub

        return (" (" + x + "," + y + ")");
    }

}
