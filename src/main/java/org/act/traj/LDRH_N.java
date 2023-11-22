package org.act.traj;

import org.act.util.*;

import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class LDRH_N extends TrajectoryCompressor {

    int count = 0;

    public LDRH_N(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByLDRH_N(this.iTrajectory, 0, this.iTrajectory.size() - 1);
        // return this.compressByLDRH(this.iTrajectory, 0,1000);

    }

    private GPSLine[] compressByLDRH_N(List<GPSPoint> iTraj, int iStart, int iEnd) {


        // GPSPoint lineStart = iTraj.get(iStart);
        // GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd];
        int j = 0;
        int k = 0;
        double HalfThreshold = 0.5 * getThreshold();
        GPSPoint loc = new GPSPoint();


        for (int i = iStart + 1; i <= iEnd; i++) {

            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), iTraj.get(i).time - iTraj.get(j).time);
            // loc.time = iTraj.get(i).time;
            // loc = DisCalculator.predict(iTraj.get(j), iTraj.get(j+1), iTraj.get(i));

            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            if (disOfP2P > HalfThreshold) {

                GPSLine line = new GPSLine();
                line.setStartPoint((GPSPoint) iTraj.get(j));
                line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                lines[k++] = line;

                j = i - 1;

            }
        }

        GPSLine line = new GPSLine();
        line.setStartPoint((GPSPoint) lines[k - 1].endPoint);
        line.setEndPoint((GPSPoint) iTraj.get(iEnd));
        lines[k++] = line;


        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
        }

        return liness;

    }

    @Override
    public String toString() {
        return "LDRH_N" + getDc();
    }

    public static void main(String[] args) {
        int npoints = 0;
        int nlines = 0;
        long timeUsed = 0;

        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();

        TrajectoryCompressor tc = new LDRH_N(sed);

//		tc.setThreshold(DisCalculator.rad(30));
        tc.setThreshold(40);

        tc.radialThreshold = 2 * tc.getThreshold();
//        Format f1 = new DecimalFormat("000");
//		String num="0";


        System.out.println("Threshold : " + tc.getThreshold() + " m.");

        for (int i = 1; i <= 10; i++) {

            // tc.strFileName = "/data/ACT/data/trajData/SerCar/gps_routes_long/selected/2.txt";
            // tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.7.8"  + ".txt";
            // tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.29.4"  + ".txt";
            // tc.strFileName = "/data/ACT/data/trajData/SerCar/CleanData/1.txt";
//			tc.strFileName = "d:/ACT/trajData/geolife/CleanData/182.txt";
            // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/172.txt";
            // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/180.txt";
            // num=f1.format(i);
            // System.out.println(num);

            tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\OneThousand\\" + i + ".txt";


            System.out.println(tc.strFileName);

            tc.loadTrajectory();
            // tc.iTrajectory = tc.iTrajectory.subList(1, 144);
            npoints = npoints + tc.iTrajectory.size();
            System.out.print("" + i);
            System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());

            double stime = System.currentTimeMillis();
            GPSLine[] lines = tc.compress();
            double etime = System.currentTimeMillis();
            timeUsed += (etime - stime);

            nlines = nlines + lines.length;


            System.out.println("	Compress By LDR to : (" + lines.length + ") lines");
            System.out.println("	Using Time :" + (etime - stime) + "mS");
            tc.printToConsole(tc.iTrajectory, lines);
            // tc.export(lines);
            System.out.println(lines.length);
            System.out.println(tc.iTrajectory.size());
        }
        System.out.println();
        System.out.println(nlines);
        System.out.println(npoints);
        System.out.println("	Using Time :" + timeUsed + "mS");
        System.out.println("Compress Ratio = " + ((1.0 * nlines) / (1.0 * npoints)) * 100 + "%.");
    }
}
