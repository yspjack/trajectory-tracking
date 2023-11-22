package org.act.traj;

import org.act.util.GPSLine;
import org.act.util.GPSPoint;
import org.act.util.SED;
import org.act.util.polygon.RegularPolygon;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

public class OptimalSED extends CISED_RPI {

    int m = 10;

    public double disThreshold = 1e4;

    public double[] deltX = new double[m];
    public double[] deltY = new double[m];

    public OptimalSED(int m) {
        super();
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

    public OptimalSED() {
        this.m = 16;
        this.setDc(new SED());
        deltX = new double[m];
        deltY = new double[m];
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }
    }

    int isDebug = 0;

    @Override
    public GPSLine[] compress() {
        return this.compressByopt(this.iTrajectory, 0, this.iTrajectory.size() - 1);
    }

    public GPSLine[] compressByopt(List<GPSPoint> iTraj, int iStart, int iEnd) {


        int[] sd = new int[iEnd - iStart + 1];
        RegularPolygon[] inscribedPolygons = new RegularPolygon[iEnd - iStart + 1];
        RegularPolygon[] circumscribedPolygons = new RegularPolygon[iEnd - iStart + 1];
        int[] pred = new int[iEnd - iStart + 1];
        // int n = iTrajectory.size();
        int n = iEnd + 1;

        sd[0] = 0;
        pred[0] = -1;
        GPSPoint pstart, pend;
        pstart = iTraj.get(iStart);
        pend = iTraj.get(iStart + 1);
        RegularPolygon cuRegularPolygon = new RegularPolygon(m);
        RegularPolygon inter = new RegularPolygon(m);

        double timeNewPolygon = 0, timePolygonInter = 0, timeGetPath = 0, timeGetPolygon = 0;
        double timeContains = 0, timeCycle = 0, timeIf = 0, timeElse = 0, timeGetPoint = 0;
        double stime, etime, cycStime, cycEtime, ifStime, ifEtime;
        int nPolygons = 0;


        for (int j = iStart + 1; j < n; j++) {
            sd[j - iStart] = Integer.MAX_VALUE;
//			start point ranges from 0 to j-1
            pend = iTraj.get(j);

            for (int k = iStart; k < j; k++) {
                // System.out.println(k);
//				D[k][j]
//				F[j-1][j-1]
                if (k == j - 1) {
//					F[j-1][j] = D[j-1][j] = D[k][j]
//					polygons[k] stores the intersection of projection polygons
//					with start point at p[k] and end point at p[j]

                    pstart = iTraj.get(k);

                    inscribedPolygons[k - iStart] = new RegularPolygon(m);
                    circumscribedPolygons[k - iStart] = new RegularPolygon(m);

                    inscribedPolygons[k - iStart] = getInscribedPolyGon(pend, pstart, getThreshold(), inscribedPolygons[k - iStart]);
                    circumscribedPolygons[k - iStart] = getCircumscribedPolyGon(pend, pstart, getThreshold(), circumscribedPolygons[k - iStart]);

//					P[j] is always in D[j-1][j]
                    if (sd[j - iStart] > sd[k - iStart] + 1) {
                        sd[j - iStart] = sd[k - iStart] + 1;
                        pred[j - iStart] = k - iStart;
                    }

                    nPolygons++;
                } else {

                    //				F[k][j] = F[k][j-1] & D[k][j]

                    if (circumscribedPolygons[k - iStart].n == 0) {
                        continue;
                    }

                    pstart = iTraj.get(k);

                    cuRegularPolygon = getInscribedPolyGon(pend, pstart, getThreshold(), cuRegularPolygon);
                    nPolygons++;
                    int flag = RegularPolygon.intersects_agg(cuRegularPolygon, inscribedPolygons[k - iStart], inter);
                    if (flag == RegularPolygon.PCONTAINSQ) {
                    } else if (flag == RegularPolygon.QCONTAINSP) {
                        RegularPolygon tmPolygon = cuRegularPolygon;
                        cuRegularPolygon = inscribedPolygons[k - iStart];
                        inscribedPolygons[k - iStart] = tmPolygon;
                    } else {
                        RegularPolygon tmPolygon = inter;
                        inter = inscribedPolygons[k - iStart];
                        inscribedPolygons[k - iStart] = tmPolygon;
                    }
                    boolean contains = false;
                    if (inscribedPolygons[k - iStart].size() > 0) {
                        contains = inscribedPolygons[k - iStart].contains(pstart, pend);
                    }
                    if (contains) {
                        if (sd[j - iStart] > sd[k - iStart] + 1) {
                            sd[j - iStart] = sd[k - iStart] + 1;
                            pred[j - iStart] = k - iStart;
                        }
                    } else {
                        cuRegularPolygon = getCircumscribedPolyGon(pend, pstart, getThreshold(), cuRegularPolygon);
                        nPolygons++;
                        flag = RegularPolygon.intersects_agg(cuRegularPolygon, circumscribedPolygons[k - iStart], inter);
                        if (flag == RegularPolygon.PCONTAINSQ) {
                        } else if (flag == RegularPolygon.QCONTAINSP) {
                            RegularPolygon tmPolygon = cuRegularPolygon;
                            cuRegularPolygon = circumscribedPolygons[k - iStart];
                            circumscribedPolygons[k - iStart] = tmPolygon;
                        } else {
                            RegularPolygon tmPolygon = inter;
                            inter = circumscribedPolygons[k - iStart];
                            circumscribedPolygons[k - iStart] = tmPolygon;
                        }
                        contains = false;
                        if (circumscribedPolygons[k - iStart].size() > 0) {
                            contains = circumscribedPolygons[k - iStart].contains(pstart, pend);
                        }
                        if (contains) {
                            if (sd[j - iStart] > sd[k - iStart] + 1) {
                                sd[j - iStart] = sd[k - iStart] + 1;
                                pred[j - iStart] = k - iStart;
                            }
                        }

                    }

                }

            }
        }


//		backtrace with pred pointers to generate the shortest path.
        Deque<Integer> stack = new ArrayDeque<>(sd[n - iStart - 1] + 1);
        int w = n - iStart - 1;
        while (w != -1) {
            stack.push(w);
            w = pred[w];
        }

        GPSLine gpsLine;
        int start, end;
        GPSLine[] result = new GPSLine[stack.size() - 1];
        start = stack.pop();
        for (int i = 0; i < result.length; i++) {
            end = stack.pop();
            gpsLine = new GPSLine();
            gpsLine.setStartPoint(iTraj.get(start + iStart));
            gpsLine.setEndPoint(iTraj.get(end + iStart));
            result[i] = gpsLine;
            start = end;
        }

//		System.out.println("timeNewPolygon : "+ timeNewPolygon);
//		System.out.println("timePolygonInter: "+ timePolygonInter);
//		System.out.println("timeGetPath: "+ timeGetPath);
//		System.out.println("timeGetPoint: "+ timeGetPoint/1e6);
//		System.out.println("timeContains: "+ timeContains);
//		System.out.println("timeGetPolygon: "+ timeGetPolygon);
//		System.out.println("timeIf: "+ timeIf);
//		System.out.println("timeCycle: "+ timeCycle);
//		System.out.println("nPolygons: " +nPolygons);
//		System.out.println("timeIfElse: "+timeElse / 1e6);

        return result;
    }


    @Override
    public String toString() {
        return "OptSED";
    }

    /**
     * @param p
     * @param start
     * @return
     */

    public static void main(String[] args) {

        int npoints = 0;
        int nlines = 0;
        for (int zeta = 40; zeta <= 40; zeta += 10) { // 锟斤拷锟斤拷锟街�
            npoints = 0;
            nlines = 0;
            double timeUsed = 0;
            double avesed = 0;

            OptimalSED tc = new OptimalSED();
            tc.setThreshold(zeta);
//			System.out.println("Threshold : " + tc.getThreshold() + " m.");
            GPSLine[] lines = null;
            for (int i = 448; i <= 1000; i++) {

//				tc.strFileName = "/data/ACT/data/traj/taxi/test.txt";
                // tc.strFileName = "/data/ACT/data/traj/taxi/7.txt";
//				tc.strFileName = "/data/ACT/data/traj/taxi/test2.txt";
//				tc.strFileName = "/data/ACT/data/traj/taxi/test3.txt";
                tc.strFileName = "/Users/apple/Documents/SerCar/CleanData/" + i + ".txt";
                // tc.strFileName = "/data/ACT/data/trajData/SerCar/CleanData/7.txt";

                // tc.strFileName = "/data/ACT/data/trajData/geolife/CleanData/176.txt";

//				tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\mopsi\\OneThousand\\"+i+".txt" ;

                tc.loadTrajectory();
//				System.out.print("" + i);
//				System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                Instant start = Instant.now();
                lines = tc.compress();
                Instant end = Instant.now();
                double etime = System.currentTimeMillis();
                Duration tDuration = Duration.between(start, end);
                timeUsed += etime - stime;
//				System.out.println("Time:" + (etime - stime));
//				System.out.println("Time:" + (tDuration));
//				System.out.println("Compress By "+tc.toString()+"  to : (" + lines.length + ") lines.");
//
//				tc.printToConsole(tc.iTrajectory, lines);

                nlines = nlines + lines.length;
                double temp = tc.getAveErrorSED(tc.iTrajectory, lines);
                temp = (double) Math.round(temp * 100) / 100;
                avesed += temp;
                tc.export(lines);
//				temp=tc.printToConsoleSED(tc.iTrajectory, lines);
//				double temp1=tc.printToConsolePED(tc.iTrajectory, lines);
//				tc.printToConsole(tc.iTrajectory, lines);
//				System.out.println(i);
            }

//			System.out.println();
//			System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
            System.out.println((1.0 * nlines) / (1.0 * npoints) * 100 + "%," + timeUsed + "ms," +
                    (1.0 * avesed) / (1.0 * npoints));
        }

    }

}


