package org.act.traj;

import org.act.util.*;
import org.act.util.polygon.IndexedVertex;
import org.act.util.polygon.Point;
import org.act.util.polygon.RegularPolygon;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;

public class CISED_RPI extends TrajectoryCompressor {

    public static double timeInsects = 0, timeGetPolygon = 0;
    public static boolean testIntersectTime = false;

    int isDebug = 0;
    int notin = 0;
    public int m = 10;

    // radius of the circle which the Circumscribed polygon lies in.
    double CircumscribedRadius = 0;

    public double disThreshold = 1e4;

    public double[] deltX = new double[m];
    public double[] deltY = new double[m];
    public double[] deltaCircumX = new double[m];
    public double[] deltaCircumY = new double[m];

    public CISED_RPI(int m) {
        this.setDc(new SED());
//		this.dc = new SEDacc();

        double angle0 = Math.PI / m / 2;

        CircumscribedRadius = 1.0 / Math.cos(angle0);

        this.m = m;
        deltX = new double[m];
        deltY = new double[m];
        deltaCircumX = new double[m];
        deltaCircumY = new double[m];

        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
            deltaCircumX[i] = Math.cos(angle + angle0);
            deltaCircumY[i] = Math.sin(angle + angle0);
        }


    }

    public CISED_RPI() {
        this.m = 16;
        this.setDc(new SED());

        double angle0 = Math.PI / m / 2;
        CircumscribedRadius = 1.0 / Math.cos(angle0);

        deltX = new double[m];
        deltY = new double[m];
        deltaCircumX = new double[m];
        deltaCircumY = new double[m];

        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
            deltaCircumX[i] = Math.cos(angle + angle0);
            deltaCircumY[i] = Math.sin(angle + angle0);
        }

    }

    public CISED_RPI(DisCalculator dc) {
        this();
        this.setDc(dc);
    }

    public CISED_RPI(int m, DisCalculator dc) {
        this(m);
        this.setDc(dc);
    }

    @Override
    public GPSLine[] compress() {
        return compressContinuous();
    }

    private GPSLine[] compressContinuous() {
        // long timeInsects = 0,timeContains = 0,timeGetPolygon = 0;
        int index = 1;
        int iStart = 0;
        int iEnd = 1;
        int lineId = 1;

        boolean isIntial = true;

        int flag = 0;
        int size = 0;
        RegularPolygon poly_i = new RegularPolygon(m);
        RegularPolygon poly_k = new RegularPolygon(m);
        RegularPolygon polygon_intersects = new RegularPolygon(m);

        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

//		long tc = 0;

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);
            p.lineId = lineId;

            if (isIntial) {
//				tc = p.time - start.time;
                if (testIntersectTime) {
                    long sTime = System.currentTimeMillis();
                    poly_k = getPolyGon(p, start, getThreshold() / 2, poly_k);
//					poly_k = getPolyGon(p, start, getThreshold() / 2, tc,poly_k);
                    timeInsects += (System.currentTimeMillis() - sTime);

                } else {
                    poly_k = getPolyGon(p, start, getThreshold() / 2, poly_k);
//					poly_k = getPolyGon(p, start, getThreshold() / 2, tc,poly_k);
                }

                size = poly_k.size();
                isIntial = false;
            } else {

                if (testIntersectTime) {
                    long sTime = System.currentTimeMillis();
                    poly_i = getPolyGon(p, start, getThreshold() / 2, poly_i);
//					poly_i = getPolyGon(p, start, getThreshold() / 2, tc,poly_i);
                    timeInsects += (System.currentTimeMillis() - sTime);

                } else {
                    poly_i = getPolyGon(p, start, getThreshold() / 2, poly_i);
//					poly_i = getPolyGon(p, start, getThreshold() / 2, tc,poly_i);
                }

                if (index - iStart > 2) {
                    RegularPolygon tmPolygon = poly_k;
                    poly_k = polygon_intersects;
                    polygon_intersects = tmPolygon;
                }

                if (testIntersectTime) {
                    long sTime = System.currentTimeMillis();
                    flag = RegularPolygon.intersects_agg(poly_i, poly_k, polygon_intersects);
                    timeInsects += (System.currentTimeMillis() - sTime);

                } else {
                    flag = RegularPolygon.intersects_agg(poly_i, poly_k, polygon_intersects);
                }

                // poly_i contains poly_k, result is poly_k
                // exchanges poly_k and polygon_intersects
                if (flag == RegularPolygon.PCONTAINSQ) {
                    RegularPolygon tmPolygon = poly_k;
                    poly_k = polygon_intersects;
                    polygon_intersects = tmPolygon;
                }
                // poly_k contains poly_i, result is poly_i
                // exchanges poly_i and polygon_intersects
                // then exchanges poly_i and poly_k
                else if (flag == RegularPolygon.QCONTAINSP) {
                    RegularPolygon tmPolygon = poly_i;
                    poly_i = polygon_intersects;
                    polygon_intersects = tmPolygon;

                    tmPolygon = poly_i;
                    poly_i = poly_k;
                    poly_k = tmPolygon;
                }
                size = polygon_intersects.size();
            }

            if (size == 0) {
                putLine(v, iStart, iEnd, lineId);
                lineId++;
                iStart = iEnd++;
                start = (GPSPoint) this.iTrajectory.get(iStart);
                index = iEnd;
                isIntial = true;

            } else {
                iEnd = index;
                index++;
            }

        }

        if (iEnd < iTrajectory.size()) {
            putLine(v, iStart, iEnd, lineId);
        }

        GPSLine[] lines = new GPSLine[v.size()];
        v.toArray(lines);

        return lines;

    }


    private GPSLine[] compressContinuous(int isDebug) {
        // long timeInsects = 0,timeContains = 0,timeGetPolygon = 0;
        int testStart = 452618;
        int testEnd = 452621;

        int index = 1;
        int iStart = 0;
        int iEnd = 1;
        int lineId = 1;

        boolean isIntial = true;

        int flag = 0;

        int size = 0;
        RegularPolygon poly_i = new RegularPolygon(m);
        RegularPolygon poly_k = new RegularPolygon(m);
        RegularPolygon polygon_intersects = new RegularPolygon(m);

        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

//		for debug 
        List<RegularPolygon> exPolygons = new LinkedList<>();
        List<Integer> exIndices = new LinkedList<>();
        List<GPSPoint> exTraj = new LinkedList<>();
        List<GPSPoint> exTrajSync = new LinkedList<>();
        List<Double> exDist = new LinkedList<>();
        List<Double> exDistSync = new LinkedList<>();
        long tc = 0;

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);
            p.lineId = lineId;
            if (isDebug == 1 && iStart == testStart) {
                exTraj.add(p);
//				GPSPoint sync = SEDacc.predict(start, iTrajectory.get(testEnd), p);
                GPSPoint sync = DisCalculator.predict(start, iTrajectory.get(testEnd), p);
                exTrajSync.add(sync);
                double dist = DisCalculator.getDistanceOfP2P(start, p);
                exDist.add(dist);
                dist = DisCalculator.getDistanceOfP2P(sync, p);
                exDistSync.add(dist);
            }

            if (isIntial) {
                tc = p.time - start.time;
                if (testIntersectTime) {
                    long sTime = System.currentTimeMillis();
//					poly_k = getPolyGon(p, start, getThreshold() / 2, poly_k);
                    poly_k = getInscribedPolyGon(p, start, getThreshold() / 2, poly_k);
                    timeInsects += (System.currentTimeMillis() - sTime);

                } else {
//					poly_k = getPolyGon(p, start, getThreshold() / 2, poly_k);
//					poly_k = getPolyGon(p, start, getThreshold() / 2, tc,poly_k);
                    poly_k = getInscribedPolyGon(p, start, getThreshold() / 2, poly_k);
                }

                size = poly_k.size();
                isIntial = false;
            } else {

                if (testIntersectTime) {
                    long sTime = System.currentTimeMillis();
//					poly_i = getPolyGon(p, start, getThreshold() / 2, poly_i);
//					poly_i = getPolyGon(p, start, getThreshold() / 2, tc,poly_i);
                    poly_i = getInscribedPolyGon(p, start, getThreshold() / 2, poly_i);
                    timeInsects += (System.currentTimeMillis() - sTime);

                } else {
//					poly_i = getPolyGon(p, start, getThreshold() / 2, poly_i);
                    poly_i = getPolyGon(p, start, getThreshold() / 2, tc, poly_i);
                }

                if (index - iStart > 2) {
                    RegularPolygon tmPolygon = poly_k;
                    poly_k = polygon_intersects;
                    polygon_intersects = tmPolygon;
                }

                if (index == 112225) {
                    System.out.println();
                }
                if (isDebug == 1 && iStart == testStart) {
                    System.out.println("size of poly_k: " + poly_k.size());
                    poly_k.PrintVertices();
                    System.out.println("size of poly_i: " + poly_i.size());
                    poly_i.PrintVertices();
                    exIndices.add(index);
                    exPolygons.add(poly_k.copy());
                    exPolygons.add(poly_i.copy());
                }


                if (testIntersectTime) {
                    long sTime = System.currentTimeMillis();
                    flag = RegularPolygon.intersects_agg(poly_i, poly_k, polygon_intersects);
                    timeInsects += (System.currentTimeMillis() - sTime);

                } else {
                    flag = RegularPolygon.intersects_agg(poly_i, poly_k, polygon_intersects);
                }

                // poly_i contains poly_k, result is poly_k
                // exchanges poly_k and polygon_intersects
                if (flag == RegularPolygon.PCONTAINSQ) {
                    RegularPolygon tmPolygon = poly_k;
                    poly_k = polygon_intersects;
                    polygon_intersects = tmPolygon;
                }
                // poly_k contains poly_i, result is poly_i
                // exchanges poly_i and polygon_intersects
                // then exchanges poly_i and poly_k
                else if (flag == RegularPolygon.QCONTAINSP) {
                    RegularPolygon tmPolygon = poly_i;
                    poly_i = polygon_intersects;
                    polygon_intersects = tmPolygon;

                    tmPolygon = poly_i;
                    poly_i = poly_k;
                    poly_k = tmPolygon;
                }
                size = polygon_intersects.size();
            }

            if (size == 0) {
                putLine(v, iStart, iEnd, lineId);

                if (isDebug == 1 && iEnd == testStart) {
                    exTraj.add(iTrajectory.get(iEnd));
                    exDist.add(0.0);
                    exTrajSync.add(iTrajectory.get(iEnd));
                    exDistSync.add(0.0);
                }


//				if(isDebug == 1) {
//					System.out.println("start :"+iStart);
//					System.out.println("end :"+iEnd);
//				}
                lineId++;
                iStart = iEnd++;
                start = (GPSPoint) this.iTrajectory.get(iStart);
                index = iEnd;
                isIntial = true;

            } else {
                iEnd = index;
                index++;
            }

        }

        if (isDebug == 1) {
            exportPolygons(exIndices, exPolygons);
            exportTraj(exTraj, exDist, "traj.txt");
            exportTraj(exTrajSync, exDistSync, "sync.txt");
        }

        if (iEnd < iTrajectory.size()) {
            putLine(v, iStart, iEnd, lineId);
        }


        GPSLine[] lines = new GPSLine[v.size()];
        v.toArray(lines);

        return lines;

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

    /**
     *
     *
     * @param p
     * @param start
     * @return
     */
    protected final RegularPolygon getInscribedPolyGon(GPSPoint p, GPSPoint start, double radius, RegularPolygon res) {
        long t0 = p.time - start.time;
        double t = 1000.0;
        double k = (t0) / (t);

        radius = radius / (k);

        Point point = new Point();

        boolean flag = point.transform(p, start, disThreshold);

        if (!flag) {
            res.clean();
            return res;
        }

        point.x /= k;
        point.y /= k;

        res.getPolygon(this.deltX, this.deltY, point, radius);

        return res;
    }

    protected final RegularPolygon getCircumscribedPolyGon(GPSPoint p, GPSPoint start, double radius, RegularPolygon res) {
        long t0 = p.time - start.time;
        double t = 1000.0;
        double k = (t0) / (t);

        radius = radius / (k);

        radius *= CircumscribedRadius;

        Point point = new Point();

        boolean flag = point.transform(p, start, disThreshold);

        if (!flag) {
            res.clean();
            return res;
        }

        point.x /= k;
        point.y /= k;

        res.getPolygon(this.deltaCircumX, this.deltaCircumY, point, radius);

        return res;
    }

    protected final RegularPolygon getPolyGon(GPSPoint p, GPSPoint start, double radius, RegularPolygon res) {
        long t0 = p.time - start.time;
        double t = 1000.0;
        double k = (t0) / (t);

        radius = radius / (k);

        Point point = new Point();

        boolean flag = point.transform(p, start, disThreshold);
//		boolean flag = point.transformGeo(p, start, disThreshold);
//		boolean flag = point.transformNormalized(p, start, disThreshold);
//		boolean flag = point.transform2ENU(p, start, disThreshold);

        if (!flag) {
            res.clean();
            return res;
        }

        point.x /= k;
        point.y /= k;

        res.getPolygon(this.deltX, this.deltY, point, radius);

        return res;
    }


    private RegularPolygon getPolyGon(GPSPoint p, GPSPoint start, double radius, long tc, RegularPolygon res) {

        double t0 = p.time - start.time;
        double k = (t0) / (tc);

        radius = radius / (k);

        Point point = new Point();

        boolean flag = point.transform(p, start, disThreshold);
//		boolean flag = point.transformGeo(p, start, disThreshold);
//		boolean flag = point.transformNormalized(p, start, disThreshold);
//		boolean flag = point.transform2ENU(p, start, disThreshold);

        if (!flag) {
            res.clean();
            return res;
        }

        point.x /= k;
        point.y /= k;

        res.getPolygon(this.deltX, this.deltY, point, radius);

        return res;
    }


    private String exportPolyGon(String index, RegularPolygon polygon) {
        StringBuilder sb = new StringBuilder();
        StringBuilder sbx = new StringBuilder();
        StringBuilder sby = new StringBuilder();
        sb.append(index).append("\n");
        if (polygon.size() == 0) {
            sbx.append(0).append("\n");
            sby.append(0).append("\n");
            sb.append(sbx);
            sb.append(sby);
            return sb.toString();
        }

        IndexedVertex b = polygon.vertices[polygon.head];
        double x, y;
        do {
            x = b.x;
            sbx.append(x).append(",");
            y = b.y;
            sby.append(y).append(",");
            b = polygon.vertices[b.next];
        } while (b.index != polygon.head);

        b = polygon.vertices[polygon.head];
        sbx.append(b.x).append("\n");
        sby.append(b.y).append("\n");
        sb.append(sbx);
        sb.append(sby);

        return sb.toString();
    }

    private void exportPolygons(List<Integer> indices, List<RegularPolygon> polygons) {

        StringBuilder sb = new StringBuilder();
        String sbx;
        int index;
        RegularPolygon polygon;
        System.out.println(indices.size());
        System.out.println(polygons.size());

        for (int i = 0; i < indices.size(); i++) {
            System.out.println(i);
            index = indices.get(i);
            System.out.println(index);
            polygon = polygons.get(2 * i);
            sbx = exportPolyGon(index + "-inters", polygon);
            sb.append(sbx);
            polygon = polygons.get(2 * i + 1);
            sbx = exportPolyGon(String.valueOf(index), polygon);
            sb.append(sbx);
        }

        System.out.println(sb);
        Path fname = Paths.get("/data/ACT/codes/evaluation/plot/polygons.txt");
        printBuilder(sb, fname);

    }

    private void exportTraj(List<GPSPoint> exTraj, List<Double> exDist, String fname) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < exTraj.size(); i++) {
            GPSPoint point = exTraj.get(i);
            double dist = exDist.get(i);
            sb.append(point.index).append(",");
            sb.append(point.latitude).append(",");
            sb.append(point.longitude).append(",");
            sb.append(point.time).append(",");
            sb.append(dist).append("\n");
        }

        fname = "/data/ACT/codes/evaluation/plot/" + fname;
        Path path = Paths.get(fname);
        printBuilder(sb, path);
    }

    public static void printBuilder(StringBuilder sb, Path fname) {
        try {
            if (!Files.exists(fname)) {
                Files.createFile(fname);
            }
            Files.write(fname, sb.toString().getBytes());
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }


    }

    @Override
    public String toString() {
        return "CISED-RPI";

    }

    public String abbrev() {
        return "CISED-RPI";
    }

    public static void main(String[] args) {
        int npoints = 0;
        int nlines = 0;

        long timeUsed = 0;
        DisCalculator ded = new DED();
        DisCalculator ped = new PED();
        DisCalculator sed = new SEDacc();
        int errorPoints;
        for (int zeta = 60; zeta <= 60; zeta += 45) { // 误差阈值
            npoints = 0;
            nlines = 0;

            CISED_RPI tc = new CISED_RPI(6, sed);
            CISED_RPI.timeInsects = 0;
            CISED_RPI.timeGetPolygon = 0;

            tc.setThreshold(zeta); // 阈值，单位为米。
            tc.radialThreshold = 1.0 * zeta;
            // tc.deviation = 13; // 判断掉头的阈值，单位为米。

            System.out.println("Threshold : " + tc.getThreshold() + " m.");

            System.out.println("radialThreshold : " + tc.radialThreshold + " m.");
            GPSLine[] lines = null;
            double[] aboutError = new double[3];

            System.out.println(CISED_RPI.testIntersectTime);
            for (int i = 1; i <= 0; i++) {
                tc.strFileName = "traj/geolife/new/" + i + ".txt";// 轨迹文件
                tc.strFileName = "/data/ACT/data/traj/taxi/7.txt";
                tc.strFileName = "/data/ACT/data/trajData/SerCar/CleanData/7.txt";
                tc.strFileName = "/data/ACT/data/trajData/geolife/CleanData/176.txt";
//				tc.strFileName = "/data/ACT/data/trajData/zh/CleanData/"+i+".txt";
//				tc.strFileName = "/data/ACT/data/trajData/mopsi/clean/7/1397379610249";
                tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + i + ".txt";

                tc.loadTrajectory();
                System.out.print("" + i);
                System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress(); // 算法：n
                double etime = System.currentTimeMillis();

                System.out.println("Time:" + (etime - stime));
                timeUsed += (etime - stime);
                System.out.println("Compress By Sleeve to : (" + lines.length + ") lines.");
//				tc.getError(tc.iTrajectory, lines, aboutError);
//				System.out.println("max error: "+aboutError[2]);
//				System.out.println("error points : "+aboutError[1]);
//				tc.printToConsole(tc.iTrajectory, lines);
//				tc.printToConsoleB(tc.iTrajectory, lines);

                nlines = nlines + lines.length;

            }

            System.out.println();
            System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
            System.out.println("Time Used = " + timeUsed + "ms");
            System.out.println("timeInsects :" + CISED_RPI.timeInsects / 1000000);
            System.out.println("timeGetPolygon :" + CISED_RPI.timeGetPolygon / 1000000);
        }
    }
}
