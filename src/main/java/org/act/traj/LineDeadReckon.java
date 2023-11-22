package org.act.traj;

import org.act.util.*;

import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class LineDeadReckon extends TrajectoryCompressor {

    int count = 0;

    public LineDeadReckon(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByLDR(this.iTrajectory, 0, this.iTrajectory.size() - 1);
        // return this.compressByLDR(this.iTrajectory, 0,100);

    }

    private GPSLine[] compressByLDR(List<GPSPoint> iTraj, int iStart, int iEnd) {


        GPSPoint lineStart = iTraj.get(iStart);
        GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd];

        // double dis1 = DisCalculator.getDistanceOfP2P(lineStart.latitude, lineStart.longitude,
        //        iTraj.get(iStart + 1).latitude, iTraj.get(iStart + 1).longitude);
        // double speed = dis1 / (iTraj.get(iStart + 1).time - iTraj.get(iStart).time);
        // double alpha = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude,iTraj.get(iStart + 1).latitude, iTraj.get(iStart + 1).longitude);
        int j = 0;
        int k = 0;
        GPSPoint loc = new GPSPoint();


        for (int i = iStart + 1; i <= iEnd; i++) {

            // System.out.println(iStart);
            // System.out.println(iEnd);

            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), iTraj.get(i).time - iTraj.get(j).time);
            loc.time = iTraj.get(i).time;
            System.out.println(i);
            System.out.println(loc);
            System.out.println(iTraj.get(i));
            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            System.out.println(disOfP2P);

            if (disOfP2P > getThreshold()) {

                GPSPoint tmp = new GPSPoint();
                tmp = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), iTraj.get(i - 1).time - iTraj.get(j).time);
                tmp.time = iTraj.get(i - 1).time;


                GPSLine line = new GPSLine();
                // GPSLine[] lines = new GPSLine[];
                line.setStartPoint((GPSPoint) iTraj.get(j));
                line.setEndPoint((GPSPoint) tmp);
                lines[k++] = line;
                System.out.println("�߶����" + j);
                System.out.println("�߶��յ�" + (i - 1));

                j = i - 1;
                // System.out.println("�߶α��"+k);
            }
            // System.out.println("hahaha"+k);
            // System.out.println(lines);
        }

        GPSLine line = new GPSLine();
        line.setStartPoint((GPSPoint) iTraj.get(j));
        line.setEndPoint((GPSPoint) iTraj.get(iEnd));
        lines[k++] = line;


        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
        }


        // System.out.println("hahaha"+k);
        return liness;

    }

    @Override
    public String toString() {
        return "DP" + getDc();
    }

    public static void main(String[] args) {
        int npoints = 0;
        int nlines = 0;

        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();

        TrajectoryCompressor tc = new LineDeadReckon(dad);

//		tc.setThreshold(DisCalculator.rad(30));
        tc.setThreshold(10);

        tc.radialThreshold = 2 * tc.getThreshold();

        System.out.println("Threshold : " + tc.getThreshold() + " m.");

        for (int i = 105; i <= 105; i++) {

            // tc.strFileName = "/data/ACT/data/trajData/SerCar/gps_routes_long/selected/2.txt";
            // tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.7.8"  + ".txt";
            // tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.29.4"  + ".txt";
            // tc.strFileName = "/data/ACT/data/trajData/SerCar/CleanData/1.txt";
//			tc.strFileName = "d:/ACT/trajData/geolife/CleanData/182.txt";
            // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/172.txt";
            // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/180.txt";

            tc.strFileName = "D:\\EXP\\data\\geolife\\" + i + ".txt";
            System.out.println(tc.strFileName);

            tc.loadTrajectory();
//			tc.iTrajectory = tc.iTrajectory.subList(140, 144);
            npoints = npoints + tc.iTrajectory.size();
            System.out.print("" + i);
            System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());

            double stime = System.currentTimeMillis();
            GPSLine[] lines = tc.compress();
            double etime = System.currentTimeMillis();


            nlines = nlines + lines.length;


            System.out.println("	Compress By LDR to : (" + lines.length + ") lines");
            System.out.println("	Using Time :" + (etime - stime) + "mS");
            // tc.printToConsole(tc.iTrajectory, lines);
            // tc.export(lines);
            System.out.println(lines.length);
            System.out.println(tc.iTrajectory.size());
        }
        System.out.println();
        System.out.println(nlines);
        System.out.println(npoints);
        System.out.println("Compress Ratio = " + ((1.0 * nlines) / (1.0 * npoints)) * 100 + "%.");
    }
}
