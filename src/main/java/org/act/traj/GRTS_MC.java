package org.act.traj;

import org.act.util.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class GRTS_MC extends TrajectoryCompressor {

    int count = 0;

    public GRTS_MC(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByGRTS_MC(this.iTrajectory, 0, this.iTrajectory.size() - 1);
    }

    private GPSLine[] subcompress(int iStart, int iEnd, double subThreshold) {
        DisCalculator sed = new SED();
        // CDR sub = new CDR(sed);
        DouglasPeucker sub = new DouglasPeucker(sed);
        sub.setThreshold(subThreshold);
        // sub.iTrajectory = sub.iTrajectory.subList(iStart,iEnd+1);
        GPSLine[] linesU = sub.compressByDP(this.iTrajectory, iStart, iEnd);
        // System.out.println(iStart);
        // System.out.println(iEnd);
        return linesU;
    }

    private int VdeleteU(ArrayList<GPSLine> BufferU, ArrayList<GPSLine> BufferV) {
        if (BufferV.isEmpty() || BufferV == null) return 0;
        for (int j = 0; j < BufferV.size(); j++) {
            if (BufferV.get(j) != BufferU.get(j))
                return j;
        }
        return 0;
    }


    private GPSLine[] compressByGRTS_MC(List<GPSPoint> iTraj, int iStart, int iEnd) {


        // GPSPoint lineStart = iTraj.get(iStart);
        // GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd];

        ArrayList<GPSLine> BufferU = new ArrayList<>();
        ArrayList<GPSLine> BufferV = new ArrayList<>();
        int j = 0;
        int k = 0;
        int Sstart = 0;
        int lastCompressed = 0;
        double bThreshold = 0;
        int C1 = 0, C = 2;
        int Sa = 0, Sb = 0;

        double Bufferm = getBuffer();
        double Threshold = getThreshold();
        GPSPoint loc = new GPSPoint();


        for (int i = iStart + 1; i <= iEnd; i++) {
            // System.out.println("这次缓冲区满了");
            if (i - lastCompressed + C1 == Bufferm - C) {
                System.out.println("这次缓冲区满了");
                double subThreshold = Threshold - bThreshold;
                GPSLine[] linesU = subcompress(Sstart, i, subThreshold);
                if (linesU[0].endPoint.index != lastCompressed) {
                    Sa = Sstart;
                    Sb = linesU[0].endPoint.index;
                } else {
                    Sa = linesU[0].endPoint.index;
                    Sb = linesU[1].endPoint.index;
                }
                double SiThreshold = bThreshold;
                bThreshold = 0;
                for (int n = Sa + 1; n < Sb; n++) {
                    if (n < lastCompressed) {
                        continue;
                    } else if (n == lastCompressed) {
                        GPSPoint predictOfSi = DisCalculator.getEndPoint(iTraj.get(Sa), iTraj.get(Sb), iTraj.get(n).time - iTraj.get(Sa).time);
                        double distanceOfSi = DisCalculator.getDistanceOfP2P(iTraj.get(n), predictOfSi);
                        bThreshold = Math.max(bThreshold, distanceOfSi + SiThreshold);
                        C1--;
                    } else {
                        GPSPoint predictOfSi = DisCalculator.getEndPoint(iTraj.get(Sa), iTraj.get(Sb), iTraj.get(n).time - iTraj.get(Sa).time);
                        double distanceOfSi = DisCalculator.getDistanceOfP2P(iTraj.get(n), predictOfSi);
                        bThreshold = Math.max(bThreshold, distanceOfSi);
                    }
                }
                lastCompressed = Sb;
                C1++;
                if (C1 > C) {
                    BufferU.add(linesU[0]);
                    Sstart = linesU[0].endPoint.index;
                    C1--;
                }


            }


            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), iTraj.get(i).time - iTraj.get(j).time);
            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            if (disOfP2P > Threshold) {
                GPSLine[] linesU = subcompress(Sstart, i - 1, Threshold);


                int sizeOfU = BufferU.size();
//            	System.out.println(sizeOfU);
//            	System.out.println(linesU.size());
                for (int n = 0; n < linesU.length; n++) {
                    BufferU.add(linesU[n]);
                }
                // System.out.println(BufferU.size());
                int VdeleteU = VdeleteU(BufferU, BufferV);
                // System.out.println(VdeleteU);
                for (int n = 0; n < sizeOfU + VdeleteU + 1; n++) {
                    lines[k++] = BufferU.get(n);
                }
                // System.out.println(k);
                // System.out.println(lines[k-1]);
                Sstart = lines[k - 1].endPoint.index;
                lastCompressed = Sstart;
                // System.out.println("zhegedian"+iTraj.get(i-1).index);
                // System.out.println(Sstart);
                BufferV = new ArrayList<>(Arrays.asList(linesU));
                // BufferV.remove(0);
                for (int n = 0; n < VdeleteU + 1; n++) {
                    BufferV.remove(n);
                }
                BufferU.clear();

                j = i - 1;

            }
        }


        GPSLine[] linesU = subcompress(Sstart, iEnd, Threshold);
        for (int n = 0; n < linesU.length; n++) {
            lines[k++] = linesU[n];
        }


        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
        }

        return liness;

    }

    @Override
    public String toString() {
        return "GRTS_MC" + getDc();
    }

    public static void main(String[] args) throws IOException {
        int npoints = 0;
        int nlines = 0;
        long timeUsed = 0;

        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();

        TrajectoryCompressor tc = new GRTS_MC(sed);

//		tc.setThreshold(DisCalculator.rad(30));
        tc.setThreshold(40);
        tc.setBuffer(100);

        tc.radialThreshold = 2 * tc.getThreshold();
//        Format f1 = new DecimalFormat("000");
//		String num="0";


        System.out.println("Threshold : " + tc.getThreshold() + " m.");

        for (int i = 1; i <= 1; i++) {

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
            index = 0;
            tc.loadTrajectory();
//			tc.iTrajectory = tc.iTrajectory.subList(140, 144);
            npoints = npoints + tc.iTrajectory.size();
            System.out.print("" + i);
            System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());

            double stime = System.currentTimeMillis();
            GPSLine[] lines = tc.compress();
            double etime = System.currentTimeMillis();
            timeUsed += (etime - stime);

            nlines = nlines + lines.length;


            System.out.println("	Compress By GRTS_MC to : (" + lines.length + ") lines");
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
