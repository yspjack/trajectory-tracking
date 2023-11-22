package org.act.traj;

import org.act.util.*;
import org.act.util.polygon.Point;
import org.act.util.polygon.RegularPolygon;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class CITT extends TrajectoryCompressor {

    public static double timeInsects = 0, timeGetPolygon = 0;
    public static boolean testIntersectTime = false;

    int isDebug = 0;
    int notin = 0;
    int m = 0;

    public double[] deltX = new double[m];
    public double[] deltY = new double[m];
    private static int sumPoint = 0;
    private static int sumVec = 0;

    public CITT(int m) {
        this.setDc(new SED());
        this.m = m;
        deltX = new double[m];
        deltY = new double[m];
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }
    }

    public CITT() {
        // TODO Auto-generated constructor stub
        this.m = 10;
        this.setDc(new SED());
        deltX = new double[m];
        deltY = new double[m];
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }

    }

    @Override
    public GPSLine[] compress() {
        return compressContinuous();
    }

    private GPSLine[] compressContinuous() {
        sumPoint = 0;
        sumVec = 0;

        int index = 1;
        int iStart = 0;
        int iEnd = 1;
        int lineId = 1;

        boolean isIntial = true;

        int flag = 0;
        int size = 0;

        RegularPolygon poly_R = new RegularPolygon(m);
        // RegularPolygon poly_k = new RegularPolygon(m);
        // RegularPolygon polygon_intersects = new RegularPolygon(m);

        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

        poly_R = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), getThreshold(), poly_R);

        index = 2;

        GPSPoint s1 = iTrajectory.get(iStart);
        GPSPoint s2 = iTrajectory.get(iStart + 1);
        sumPoint++;
        sumVec++;

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);
            p.lineId = lineId;
            // System.out.println(index + "!!!");
            // System.out.println(index + "?123?");

            /*
             * if (poly_R.size()==0) { poly_R = getPolyGon(iTrajectory.get(index),
             * iTrajectory.get(iStart+1), getThreshold() , poly_R); }
             */

            double t = (double) (p.time - start.time) / (iTrajectory.get(iStart + 1).time - start.time);
            if (1 == 2) {// t<1.00001||t>100000) {
                putLine(v, iStart, index - 1, lineId);

                lineId++;

                iStart = index - 1;
                start = (GPSPoint) this.iTrajectory.get(iStart);

                poly_R.clean();
                s1 = iTrajectory.get(iStart);
                s2 = iTrajectory.get(iStart + 1);
                iEnd = index - 1;
                poly_R = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), getThreshold(), poly_R);
                sumPoint++;
                sumVec++;
            } else {

                if ((poly_R.size() == 0) || (poly_R.size() != 0 && check(start, iTrajectory.get(iStart), p, poly_R))) {
                    putLine(v, iStart, index - 1, lineId);

                    lineId++;

                    iStart = index - 1;
                    start = (GPSPoint) this.iTrajectory.get(iStart);

                    poly_R.clean();
                    s1 = iTrajectory.get(iStart);
                    s2 = iTrajectory.get(iStart + 1);
                    iEnd = index - 1;
                    sumPoint++;
                    sumVec++;

                } else if (checkv(s1, s2, p)) {
                    s2 = p;
                    sumVec++;
                }
                // System.out.println(index + "?321?");
                // System.out.println(index + "!??!" + " " + poly_R.size());
                if (poly_R.size() == 0) {
                    poly_R = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), getThreshold(), poly_R);
                } else {
                    RegularPolygon poly_T = new RegularPolygon(m);
                    poly_T = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), getThreshold(), poly_T);
                    // System.out.println(index + "???");
                    // System.out.println(poly_R.size() + " " + poly_T.size());
                    // if (poly_R.size()>=poly_T.size())
                    // poly_res.clean();

                    // RegularPolygon poly_res = new RegularPolygon(m);
                    // int test = RegularPolygon.intersects_agg(poly_T, poly_R, poly_T);
                    // poly_R = poly_T;


                    RegularPolygon res = new RegularPolygon(m);
//				res = res.intersects(poly_T, poly_R);
                    res = res.intersects(poly_T, poly_R);

                    poly_R = res;

                    // else
                    // poly_R = poly_R.intersects(poly_T, poly_R);
                    // System.out.println(index + "!!!");
                    // int t = poly_R.intersects_agg(poly_R, poly_T, poly_R);
                }
            }
            iEnd = index;
            index++;
        }
        // System.out.println(iStart + " !!!!" + iEnd + "    " + iTrajectory.size());

        putLine(v, iStart, index - 1, lineId);
        sumPoint++;

        GPSLine[] lines = new GPSLine[v.size()];

        v.toArray(lines);

        return lines;

    }

    // not pass return true
    // else return false
    private boolean check(GPSPoint s, GPSPoint splus1, GPSPoint e, RegularPolygon r) {
        // GPSPoint tmp = DisCalculator.predict(s, e, (double)splus1.time-s.time);
        GPSPoint tmp = DisCalculator.predict(s, e, (double) splus1.time - s.time);
        Point point = new Point();
        // Point.x = tmp.;
        return !r.contains(s, e);
        // return true;
    }

    private boolean checkv(GPSPoint vs, GPSPoint ve, GPSPoint p) {
        GPSPoint tmp = DisCalculator.predict(vs, ve, p);
        double dis = DisCalculator.getDistanceOfP2P(tmp, p);
        return (dis >= getThreshold());
    }

    private void putLine(List<GPSLine> v, int iStart, int iEnd, int lineId) {
        GPSLine line = new GPSLine();
        line.index = lineId;
        GPSPoint p_s = (GPSPoint) this.iTrajectory.get(iStart);
        p_s.index = iStart;
        line.startPoint = p_s;

        GPSPoint p_e = (GPSPoint) this.iTrajectory.get(iEnd);
        p_e.index = iEnd;
        line.endPoint = p_e;

        v.add(line);
    }

    private RegularPolygon getPolyGon(GPSPoint p, GPSPoint start, double radius, RegularPolygon res) {
        long t0 = p.time - start.time;
        double t = 1000.0;
        double k = (t0) / (t);

        radius = radius / (k);

        Point point = new Point();

        // boolean flag = point.transform(p, start, 930); //mopsi 930 else 1e5
        // boolean flag = point.transform(p, start, 450); //geolife 1e3
        boolean flag = point.transform(p, start, 5000);

        if (!flag) {
            res.clean();
            return res;
        }
        point.x /= k;
        point.y /= k;

        res.getPolygon(this.deltX, this.deltY, point, radius);

        return res;
    }

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "CITT";

    }

    public static void main(String[] args) throws IOException {
        long timeUsed = 0;
        DisCalculator ded = new DED();
        DisCalculator ped = new PED();
        DisCalculator sed = new SED();
        int errorPoints;
        // double ave = 0;
        int npoints = 0;
        int nlines = 0;

        String time = "CITT-time ";
        String CR = "CITT-CR ";
        String CITTave = "CITT-ave ";
        // double avesed=0;
        // double aveped=0;


//		java.io.BufferedReader reader=new java.io.BufferedReader(new java.io.FileReader("/Users/apple/Documents/geo_list_final.txt"));
//		java.util.List list=new java.util.ArrayList();
//		String line;
//		while((line=reader.readLine())!=null)
//		{
//			list.add(line);
//		}
//		reader.close();

        for (int zeta = 10; zeta <= 200; zeta += 10) {

            // if (zeta >= 50 && zeta <= 110)
            //	zeta += 10;
            // if (zeta >= 130 && zeta <= 170)
            //	zeta += 30;

            npoints = 0;
            nlines = 0;
            timeUsed = 0;
            // ave = 0;
            // double timeUsed=0;
            double avesed = 0;
            double maxsed = 0;
            double aveped = 0;
            double maxped = 0;
            int nsumPoint = 0;
            int nsumVec = 0;

            CITT tc = new CITT(16);
            CITT.timeInsects = 0;
            CITT.timeGetPolygon = 0;

            tc.setThreshold(zeta);
            tc.radialThreshold = 1.0 * zeta;
            // tc.deviation = 13;
            // System.out.println("CITT  mopsi.");

            // System.out.println("Threshold : " + tc.getThreshold() + " m.");
            GPSLine[] lines = null;
            for (int i = 1; i <= 182; i++) {
                if (i == 7 || i == 85 || i == 87 || i == 131 || i == 144 || i == 146 || i == 157 || i == 167 || i == 176)
                    continue;
//			for(java.util.Iterator i=list.iterator();i.hasNext();) {	


//				tc.strFileName = "/Users/apple/Documents/traj/geolife/CleanData/"+i.next();

                // tc.strFileName = "/Users/apple/Documents/traj/SerCar/CleanData/" + i + ".txt";//28
                // tc.strFileName = "D:\\HYC\\traj\\traj\\SerCar\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\geolife\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\mopsi\\join\\" + i+".txt";
                tc.strFileName = "/Users/apple/Documents/geolife/traj/CleanData/" + i + ".txt";


                tc.loadTrajectory();
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress();
                double etime = System.currentTimeMillis();
                avesed += tc.getAveErrorSED(tc.iTrajectory, lines);
                aveped += tc.getAveErrorPED(tc.iTrajectory, lines);
                nsumPoint += sumPoint;
                nsumVec += sumVec;

                timeUsed += (etime - stime);

                double newsed = tc.printToConsoleSED(tc.iTrajectory, lines);
                // double newped = tc.printToConsolePED(tc.iTrajectory, lines);

                // System.out.println(tc.getThreshold());
                // System.out.println("!!!!!!!!!! " + newsed);
                nlines = nlines + lines.length;
                // System.out.println(i+" "+tc.getThreshold()+","+newsed+","+newped+","+(1.0 * nlines) / (1.0 * npoints) * 100 + "%");
                // tc.export(lines);
                if (newsed > tc.getThreshold()) {
//					System.out.println("!!!!!!!!!!!!!!!!!"+tc.strFileName+"  "+tc.getThreshold()+" "+newsed);
                }

            }
            time += timeUsed + ",";
            CR += (1.0 * nlines) / (1.0 * npoints) * 100 + ",";
            // CITTave += (1.0 * ave) / (1.0 * npoints) + ",";
            double avesed_p = (1.0 * avesed) / (1.0 * npoints);
            double aveped_p = (1.0 * aveped) / (1.0 * npoints);

            System.out.println(tc.getThreshold() + "," + nsumPoint + "," + nsumVec + "," + npoints + "," + ((1.0 * nlines) / (1.0 * npoints) * 100) + "," + timeUsed + "," + avesed_p + "," + aveped_p);

            // System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");

            // System.out.println("Time Used = " + timeUsed + "ms");
            // System.out.println("AveSEDError = " + (1.0 * avesed) / (1.0 * npoints) +
            // "m.");
            // System.out.println(time);
            // System.out.println(CR);
            // System.out.println(CITTave);

        }
    }
}
