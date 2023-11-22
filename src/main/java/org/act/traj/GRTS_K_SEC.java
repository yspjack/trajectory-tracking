package org.act.traj;

import org.act.util.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class GRTS_K_SEC extends TrajectoryCompressor {

    int count = 0;

    public GRTS_K_SEC(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByGRTS_K_SEC(this.iTrajectory, 0, this.iTrajectory.size() - 1);
    }

    private GPSLine[] subcompress(int iStart, int iEnd) {
        DisCalculator sed = new SED();
        // CDR sub = new CDR(sed);
        DouglasPeucker sub = new DouglasPeucker(sed);
        sub.setThreshold(this.getThreshold());
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


    private GPSLine[] compressByGRTS_K_SEC(List<GPSPoint> iTraj, int iStart, int iEnd) {


        // GPSPoint lineStart = iTraj.get(iStart);
        // GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd];

        ArrayList<GPSLine> BufferU = new ArrayList<>();
        // ArrayList<GPSLine> BufferV=new ArrayList<GPSLine>();
        GPSLine lineV = new GPSLine();
        int j = 0;
        int k = 0;
        int Sstart = 0;
        // double Bufferm=getBuffer();
        double Threshold = getThreshold();
        GPSPoint loc = new GPSPoint();


        for (int i = iStart + 1; i <= iEnd; i++) {
            System.out.println("���ѭ���Ĺ�����" + Sstart + "he" + i);
            for (int n = Sstart + 1; n < i; n++) {
                // System.out.println("���ǵ���");
                GPSPoint temp = DisCalculator.getEndPoint(iTraj.get(Sstart), iTraj.get(i), iTraj.get(n).time - iTraj.get(Sstart).time);
                double disOfSi = DisCalculator.getDistanceOfP2P(iTraj.get(n), temp);
                if (disOfSi > Threshold) {
//                	System.out.println("���ǵ���");
//                	System.out.println(i);
                    System.out.println("����diyici " + i);
                    GPSLine line = new GPSLine();
                    line.setStartPoint((GPSPoint) iTraj.get(Sstart));
                    line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                    BufferU.add(line);
                    Sstart = i - 1;
                    break;
                }
            }

            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), iTraj.get(i).time - iTraj.get(j).time);
            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            if (disOfP2P > Threshold) {
                System.out.println("���ǵ���" + i);
                if (true) {
                    GPSLine line = new GPSLine();
                    line.setStartPoint((GPSPoint) iTraj.get(Sstart));
                    line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                    BufferU.add(line);
                }
                if (BufferU.size() > 1) {
                    for (int n = 0; n < BufferU.size() - 1; n++) {
                        lines[k++] = BufferU.get(n);
                    }
                }

                lineV = BufferU.get(BufferU.size() - 1);
                BufferU.clear();
                j = i - 1;

            }
        }

        for (int n = 0; n < BufferU.size(); n++) {
            lines[k++] = BufferU.get(n);
        }

        GPSLine line = new GPSLine();
        line.setStartPoint((GPSPoint) iTraj.get(Sstart));
        line.setEndPoint((GPSPoint) iTraj.get(iEnd));
        lines[k++] = line;

        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
            System.out.println(lines[i]);
        }

        return liness;

    }

    @Override
    public String toString() {
        return "GRTS_K_SEC" + getDc();
    }

    public static void main(String[] args) throws IOException {
        int npoints = 0;
        int nlines = 0;
        long timeUsed = 0;

        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();

        TrajectoryCompressor tc = new GRTS_K_SEC(sed);

//		tc.setThreshold(DisCalculator.rad(30));
        tc.setThreshold(10);
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

            // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\OneThousand\\"+i+".txt" ;
            tc.strFileName = "/Users/apple/Documents/" + i + ".txt";


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


            System.out.println("	Compress By GRTS_K_SEC to : (" + lines.length + ") lines");
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
