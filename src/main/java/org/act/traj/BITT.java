package org.act.traj;

import org.act.util.*;
import org.act.util.polygon.Point;
import org.act.util.polygon.RegularPolygon;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

public class BITT extends TrajectoryCompressor {

    public static double timeInsects = 0, timeGetPolygon = 0;
    public static boolean testIntersectTime = false;
    private static int sumPoint = 0;
    private static int sumVec = 0;

    int isDebug = 0;
    int notin = 0;
    int m = 0;

    public double[] deltX = new double[m];
    public double[] deltY = new double[m];

    public BITT(int m) {
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

    public BITT() {
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

        // System.out.println("sed: " + this.SEDerr + " ped: "+this.PEDerr);

        // long timeInsects = 0,timeContains = 0,timeGetPolygon = 0;
        int index = 1;
        int iStart = 0;
        int iEnd = 1;
        int lineId = 1;
        sumPoint = 0;
        sumVec = 0;


        boolean isIntial = true;

        int flag = 0;
        int size = 0;

        RegularPolygon poly_R = new RegularPolygon(m);
        // RegularPolygon poly_k = new RegularPolygon(m);
        // RegularPolygon polygon_intersects = new RegularPolygon(m);

        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

        poly_R = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), this.SEDerr, poly_R);

        index = 2;

        GPSPoint s1 = iTrajectory.get(iStart);
        GPSPoint s2 = iTrajectory.get(iStart + 1);
        GPSPoint ttt = iTrajectory.get(iStart + 1);

        SectorBound sec = new SectorBound(-1, -1);
        sec = SectorBound.getSector(start, iTrajectory.get(iStart + 1), 2 * this.PEDerr);
        Optional<SectorBound> inters;
        inters = Optional.of(sec);

        double lm = TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude,
                iTrajectory.get(iStart + 1).latitude, iTrajectory.get(iStart + 1).longitude);

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);
            p.lineId = lineId;


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
                sec = null;
                lm = 0;
                sumPoint++;
                sumVec++;
            } else {

                boolean contains = inters.get().contains(start, p);

                if ((poly_R.size() == 0) || (poly_R.size() != 0 && check(start, iTrajectory.get(iStart), p, poly_R))
                        || ((!inters.isPresent() || sec == null) || (TrajectoryCompressor.getDistanceOfP2P(start.latitude,
                        start.longitude, p.latitude, p.longitude) < lm && TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude, p.latitude, p.longitude) < Math.sqrt(lm * lm - this.PEDerr * this.PEDerr))
                        || (!contains))) {
                    putLine(v, iStart, index - 1, lineId);

                    lineId++;

                    iStart = index - 1;
                    start = (GPSPoint) this.iTrajectory.get(iStart);

                    poly_R.clean();
                    s1 = iTrajectory.get(iStart);
                    s2 = iTrajectory.get(iStart + 1);
                    iEnd = index - 1;
                    sec = null;
                    lm = 0;
                    sumPoint++;
                    sumVec++;
                } else if (checkv(s1, s2, p) || checkv2(s1, s2, p)) {
                    s2 = p;
                    sumVec++;
                }

                if (sec == null || !inters.isPresent()) {
                    sec = SectorBound.getSector(start, p, 2 * this.PEDerr);
                    inters = Optional.of(sec);
                    poly_R = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), this.SEDerr, poly_R);
                } else {

                    Optional<SectorBound> res;
                    Optional.of(sec);
                    SectorBound temp = new SectorBound();
                    temp = SectorBound.getSector(start, p, 2 * this.PEDerr);

                    Optional<SectorBound> t1;
                    t1 = Optional.of(temp);

                    res = sec.intersects(t1);

                    if (res.isPresent()) {
                        sec = res.get();
                        inters = Optional.of(sec);
                    } else {
                        sec = null;
                    }

                    RegularPolygon poly_T = new RegularPolygon(m);
                    poly_T = getPolyGon(iTrajectory.get(index), iTrajectory.get(iStart), this.SEDerr, poly_T);

                    RegularPolygon resR = new RegularPolygon(m);
                    resR = resR.intersects(poly_T, poly_R);
                    poly_R = resR;
                    // double dis =
                    // TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude);
                    // if (dis>lm) lm = dis;

                }
            }
            double dis = TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude, p.latitude,
                    p.longitude);
            if (dis > lm) {
                lm = dis;
                ttt = p;
            }
            iEnd = index;
            index++;
        }
        // System.out.println(iStart + " !!!!" + iEnd + " " + iTrajectory.size());

        putLine(v, iStart, index - 1, lineId);

        GPSLine[] lines = new GPSLine[v.size()];
        v.toArray(lines);
        sumPoint++;

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
        return (dis > this.SEDerr);
    }

    private boolean checkv2(GPSPoint vs, GPSPoint ve, GPSPoint p) {
        double dis = TrajectoryCompressor.getDistance(vs, ve, p);

        return (dis > this.PEDerr);
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

        // boolean flag = point.transform(p, start, 1e5);
        boolean flag = point.transform(p, start, 5000); // mopsi 930 else 1e5

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
        return "BITT";

    }

    public static void main(String[] args) throws IOException {
        long timeUsed = 0;
        DisCalculator ded = new DED();
        DisCalculator ped = new PED();
        DisCalculator sed = new SED();
        int errorPoints;
        double ave = 0;
        int npoints = 0;
        int nlines = 0;

        String time = "BITT-time ";
        String CR = "BITT-CR ";
        String BITTave = "BITT-ave ";
		
		
		/*java.io.BufferedReader reader=new java.io.BufferedReader(new java.io.FileReader("/Users/apple/Documents/geo_list_final.txt"));
		java.util.List list=new java.util.ArrayList();
		String line;
		while((line=reader.readLine())!=null)
		{
			list.add(line);
		}
		reader.close();*/

        for (int pederr = 10; pederr <= 200; pederr += 10) {
            for (int sederr = 10; sederr <= 200; sederr += 10) {

                // if (zeta >= 50 && zeta <= 110)
                // zeta += 10;
                // if (zeta >= 130 && zeta <= 170)
                // zeta += 30;

                npoints = 0;
                nlines = 0;
                timeUsed = 0;
                ave = 0;
                // double timeUsed=0;
                double avesed = 0;
                double maxsed = 0;
                double aveped = 0;
                double maxped = 0;

                int nsumPoint = 0;
                int nsumVec = 0;

                BITT tc = new BITT(16);
                BITT.timeInsects = 0;
                BITT.timeGetPolygon = 0;
                tc.setPEDandSED(pederr, sederr);

                // tc.setThreshold(zeta);
                // tc.radialThreshold = 1.0 * zeta;
                // tc.deviation = 13;
                // System.out.println("BITT mopsi.");

                // System.out.println("Threshold : " + tc.getThreshold() + " m.");
                GPSLine[] lines = null;
                for (int i = 1; i <= 182; i++) {
                    if (i == 7 || i == 85 || i == 87 || i == 131 || i == 144 || i == 146 || i == 157 || i == 167 || i == 176)
                        continue;
                    // tc.strFileName = "/Users/apple/Documents/" + i + ".txt";
                    //	tc.strFileName = "/Users/apple/Documents/mopsi/" + i + ".txt";
                    // for(java.util.Iterator i=list.iterator();i.hasNext();) {
                    //	tc.strFileName = "/Users/apple/Documents/traj/geolife/CleanData/"+i.next();
                    // tc.strFileName = "/Users/apple/Documents/traj/SerCar/CleanData/" + i + ".txt";
//					tc.strFileName = "D:\\HYC\\traj\\traj\\SerCar\\CleanData\\" + i+".txt";
//					tc.strFileName = "D:\\HYC\\traj\\traj\\geolife\\CleanData\\" + i+".txt";
                    // tc.strFileName = "D:\\HYC\\traj\\traj\\mopsi\\join\\" + i+".txt";
                    tc.strFileName = "/Users/apple/Documents/geolife/traj/CleanData/" + i + ".txt";

                    tc.loadTrajectory();
                    npoints = npoints + tc.iTrajectory.size();

                    double stime = System.currentTimeMillis();
                    lines = tc.compress();
                    double etime = System.currentTimeMillis();
                    ave += tc.getAveErrorSED(tc.iTrajectory, lines);

                    timeUsed += (etime - stime);
                    nsumPoint += sumPoint;
                    nsumVec += sumVec;

                    double newsed = tc.printToConsoleSED(tc.iTrajectory, lines);
                    double newped = tc.printToConsolePED(tc.iTrajectory, lines);

                    avesed += tc.getAveErrorSED(tc.iTrajectory, lines);
                    aveped += tc.getAveErrorPED(tc.iTrajectory, lines);

                    // System.out.println(tc.getThreshold());
                    // System.out.println("sed !!!!!!!!!! " + newsed);
                    // System.out.println("ped !!!!!!!!!! " + newped);

                    nlines = nlines + lines.length;
                    // tc.export(lines);
                    // System.out.println(sederr + "," + pederr + "," + newsed + "," + newped + ","
                    //		+ (1.0 * nlines) / (1.0 * npoints) * 100 + "%");
                    if (newped > pederr) {
                        // System.out.println("ped!!!!!!!!!!!!!!!!!"+tc.strFileName+"  "+pederr+" "+newped);
                    }
                    if (newsed > sederr) {
                        // System.out.println("sed!!!!!!!!!!!!!!!!!"+tc.strFileName+"  "+sederr+" "+newsed);
                    }


                }
                time += timeUsed + ",";
                CR += (1.0 * nlines) / (1.0 * npoints) * 100 + ",";
                BITTave += (1.0 * ave) / (1.0 * npoints) + ",";


                double avesed_p = (1.0 * avesed) / (1.0 * npoints);
                double aveped_p = (1.0 * aveped) / (1.0 * npoints);
                // System.out.println(pederr+","+sederr+","+nsumPoint+","+nsumVec+","+((1.0 * nlines) / (1.0 * npoints) * 100)+"%,"+timeUsed+","+avesed_p+","+aveped_p);
                System.out.println(pederr + "," + sederr + "," + nsumPoint + "," + nsumVec + "," + npoints + "," + ((1.0 * nlines) / (1.0 * npoints) * 100) + "," + timeUsed + "," + avesed_p + "," + aveped_p);

                // System.out.println();
                // System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) *
                // 100 + "%.");
                // System.out.println("Time Used = " + timeUsed + "ms");
                // System.out.println("AveSEDError = " + (1.0 * avesed) / (1.0 * npoints) +
                // "m.");
                // System.out.println(time);
                // System.out.println(CR);
                // System.out.println(BITTave);

            }
        }
    }
}
