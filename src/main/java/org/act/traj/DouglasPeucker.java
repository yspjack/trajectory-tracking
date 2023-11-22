package org.act.traj;

import org.act.util.*;

import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class DouglasPeucker extends TrajectoryCompressor {

    int count = 0;

    boolean PED_flag = true;
    boolean DAD_flag = true;
    boolean SED_flag = true;

    private double SEDThreshold = 0;
    private double PEDThreshold = 0;
    private double DADThreshold = 0;
    DAD dadCalc = new DAD();
    SED sed = new SED();
    PED ped = new PED();
    List<GPSPoint> itraj;

    public DouglasPeucker() {
    }

    public DouglasPeucker(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByDP(this.iTrajectory, 0, this.iTrajectory.size() - 1);

    }

    public GPSLine[] compressDPMUL(List<GPSPoint> iTraj, int iStart, int iEnd, double SEDThreshold, double PEDThreshold, double DADThreshold) {
        this.SEDThreshold = SEDThreshold;
        this.PEDThreshold = PEDThreshold;
        this.DADThreshold = DADThreshold;
        SED_flag = (SEDThreshold != 0);
        PED_flag = (PEDThreshold != 0);
        DAD_flag = (DADThreshold != 0);
        itraj = iTraj;
        // itraj.addAll(iTraj);
        return this.compressByMulDP(iStart, iEnd);

    }


    protected GPSLine[] compressByDP(List<GPSPoint> iTraj, int iStart, int iEnd) {

        double maxDist = 0;
        int index = 0;

        if (getDc() instanceof DAD) {
            DAD dadCalc = (DAD) getDc();
            GPSPoint lineStart = iTraj.get(iStart);
            GPSPoint lineEnd = iTraj.get(iEnd);
            double alpha = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);

            for (int i = iStart; i < iEnd; i++) {
                double dis = 0;
                dis = dadCalc.getDistanceOfP2Line(alpha, (GPSPoint) iTraj.get(i), iTraj.get(i + 1));

                if (dis > maxDist || dis < -maxDist) {
                    maxDist = Math.abs(dis);
                    index = i;
                }
            }
            if (index == iStart) {
                index = iStart + 1;
            }

        } else {

            for (int i = iStart + 1; i < iEnd; i++) {
                double dis = getDc().getDistanceOfP2Line((GPSPoint) iTraj.get(iStart), (GPSPoint) iTraj.get(iEnd),
                        (GPSPoint) iTraj.get(i));
                if (dis > maxDist || dis < -maxDist) {
                    maxDist = Math.abs(dis);
                    index = i;
                }

            }
        }

        if (maxDist < getThreshold()) {

            GPSLine line = new GPSLine();
            GPSLine[] lines = new GPSLine[1];
            lines[0] = line;

            line.setStartPoint((GPSPoint) iTraj.get(iStart));
            line.setEndPoint((GPSPoint) iTraj.get(iEnd));

            line.setAngle(TrajectoryCompressor.getAngleOfVector(line.getStartPoint().latitude,
                    line.getStartPoint().longitude, line.getEndPoint().latitude, line.getEndPoint().longitude));
            line.setLength(TrajectoryCompressor.getDistanceOfP2P(line.getEndPoint().latitude,
                    line.getEndPoint().longitude, line.getStartPoint().latitude, line.getStartPoint().longitude));

            return lines;

        } else {
            GPSLine[] lines1, lines2;
//			System.out.println(iStart+","+index+","+iEnd);
            if (index - iStart == 1) {
                lines1 = new GPSLine[1];
                GPSLine line = new GPSLine();
                lines1[0] = line;

                line.setStartPoint((GPSPoint) iTraj.get(iStart));
                line.setEndPoint((GPSPoint) iTraj.get(index));

                line.setAngle(TrajectoryCompressor.getAngleOfVector(line.getStartPoint().latitude,
                        line.getStartPoint().longitude, line.getEndPoint().latitude, line.getEndPoint().longitude));
                line.setLength(TrajectoryCompressor.getDistanceOfP2P(line.getEndPoint().latitude,
                        line.getEndPoint().longitude, line.getStartPoint().latitude, line.getStartPoint().longitude));

            } else {
                lines1 = compressByDP(iTraj, iStart, index);
            }

            if (iEnd - index == 1) {
                lines2 = new GPSLine[1];
                GPSLine line = new GPSLine();
                lines2[0] = line;

                line.setStartPoint((GPSPoint) iTraj.get(index));
                line.setEndPoint((GPSPoint) iTraj.get(iEnd));

                line.setAngle(TrajectoryCompressor.getAngleOfVector(line.getStartPoint().latitude,
                        line.getStartPoint().longitude, line.getEndPoint().latitude, line.getEndPoint().longitude));
                line.setLength(TrajectoryCompressor.getDistanceOfP2P(line.getEndPoint().latitude,
                        line.getEndPoint().longitude, line.getStartPoint().latitude, line.getStartPoint().longitude));

            } else {
                lines2 = compressByDP(iTraj, index, iEnd);
            }
            GPSLine[] lines = new GPSLine[lines1.length + lines2.length];

            for (int i = 0; i < lines1.length; i++) {
                lines[i] = lines1[i];
            }
            for (int i = 0; i < lines2.length; i++) {
                lines[lines1.length + i] = lines2[i];
            }
            return lines;
        }
    }

    // compressByMulDP
    protected GPSLine[] compressByMulDP(int iStart, int iEnd) {
        // double maxDist = 0;
        int DADindex = 0;
        int PEDindex = 0;
        int SEDindex = 0;
        // SEDThreshold,PEDThreshold,DADThreshold

        double maxDAD = 0;
        double maxPED = 0;
        double maxSED = 0;
		
		/*System.out.println(iStart);
		System.out.println(iEnd);
		System.out.println("!!!!!!");*/


        if (DAD_flag) {

            GPSPoint lineStart = itraj.get(iStart);
            GPSPoint lineEnd = itraj.get(iEnd);
            double alpha = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);

            for (int i = iStart; i < iEnd; i++) {
                double dis = 0;
                dis = dadCalc.getDistanceOfP2Line(alpha, (GPSPoint) itraj.get(i), itraj.get(i + 1));

                if (dis > maxDAD || dis < -maxDAD) {
                    maxDAD = Math.abs(dis);
                    DADindex = i;
                }
            }
            if (DADindex == iStart) {
                DADindex = iStart + 1;
            }

        }
        if (SED_flag) {

            for (int i = iStart; i <= iEnd; i++) {

                double dis = sed.getDistanceOfP2Line((GPSPoint) itraj.get(iStart), (GPSPoint) itraj.get(iEnd),
                        (GPSPoint) itraj.get(i));
                if (dis > maxSED || dis < -maxSED) {
                    maxSED = Math.abs(dis);
                    SEDindex = i;
                }

            }
        }
        if (PED_flag) {

            for (int i = iStart; i <= iEnd; i++) {

                double dis = ped.getDistanceOfP2Line((GPSPoint) itraj.get(iStart), (GPSPoint) itraj.get(iEnd),
                        (GPSPoint) itraj.get(i));
                if (dis > maxPED || dis < -maxPED) {
                    maxPED = Math.abs(dis);
                    PEDindex = i;
                }

            }
        }

        boolean flag = false;
        int index = iStart;
        boolean p = false;


        if ((DAD_flag && maxDAD > DADThreshold) || (SED_flag && maxSED > SEDThreshold) || (PED_flag && maxPED > PEDThreshold)) {
            flag = true;
            if (DAD_flag && maxDAD > DADThreshold) {
                index = DADindex;
                p = true;
            }
            int mid = (iStart + iEnd) / 2;
            if (SED_flag && maxSED > SEDThreshold) {
                if (p == false) {
                    index = SEDindex;
                    p = true;
                }
                if (p && Math.abs(mid - index) < Math.abs(mid - SEDindex)) {
                    index = SEDindex;
                    p = true;
                }
            }
            if (PED_flag && maxPED > PEDThreshold) {
                if (p == false) {
                    index = PEDindex;
                    p = true;
                }
                if (p && Math.abs(mid - index) < Math.abs(mid - PEDindex)) {
                    index = PEDindex;
                }
            }
        }

        if (flag == false) {

            GPSLine line = new GPSLine();
            GPSLine[] lines = new GPSLine[1];
            lines[0] = line;

            line.setStartPoint((GPSPoint) itraj.get(iStart));
            line.setEndPoint((GPSPoint) itraj.get(iEnd));

            line.setAngle(TrajectoryCompressor.getAngleOfVector(line.getStartPoint().latitude,
                    line.getStartPoint().longitude, line.getEndPoint().latitude, line.getEndPoint().longitude));
            line.setLength(TrajectoryCompressor.getDistanceOfP2P(line.getEndPoint().latitude,
                    line.getEndPoint().longitude, line.getStartPoint().latitude, line.getStartPoint().longitude));

            return lines;

        } else {
            GPSLine[] lines1 = compressByMulDP(iStart, index);
            GPSLine[] lines2 = compressByMulDP(index, iEnd);
            GPSLine[] lines = new GPSLine[lines1.length + lines2.length];

            for (int i = 0; i < lines1.length; i++) {
                lines[i] = lines1[i];
            }
            for (int i = 0; i < lines2.length; i++) {
                lines[lines1.length + i] = lines2[i];
            }
            return lines;
        }
    }


    @Override
    public String toString() {
        return "DP" + getDc();
    }

    public static void main(String[] args) throws Exception {


        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();

        TrajectoryCompressor tc = new DouglasPeucker();
        // System.out.println(" THIS is DPSED.mopsi");

        for (int zeta = 15; zeta <= 90; zeta += 15) {
//			if (zeta==50) zeta+=10;
//			if (zeta==70) zeta+=10;
//			if (zeta==90) zeta+=10;

            int npoints = 0;
            int nlines = 0;
            double timeUsed = 0;
            double avesed = 0;
            double maxsed = 0;
            double aveped = 0;
            double maxped = 0;
            double avedad = 0;
            double maxtime = 0;
            double avetime = 0;
            tc.setThreshold(zeta);

            tc.radialThreshold = 2 * tc.getThreshold();
            // System.out.println(" THIS is DPPED.mopsi");

            // System.out.println("Threshold : " + tc.getThreshold() + " m.");

            for (int i = 1; i <= 1000; i++) {
                // if (i==7 || i==29) continue;
//			if (i==7 || i==85 || i==87 || i==131 || i==144 || i==146 || i==157 || i==167 || i==176) continue;
//			tc.strFileName = "/data/ACT/data/trajData/SerCar/gps_routes_long/selected/2.txt";
//			tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.7.8"  + ".txt";
//			tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.29.4"  + ".txt";
//			tc.strFileName = "/data/ACT/data/trajData/SerCar/CleanData/1.txt";
//			tc.strFileName = "d:/ACT/trajData/geolife/CleanData/182.txt";
                // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/172.txt";
                // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/180.txt";


                // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\SerCar\\CleanData\\"+ i + ".txt";
                // tc.strFileName = "C:\\Users\\fewda\\Desktop\\"+i+".txt";
                tc.strFileName = "/Users/apple/Documents/SerCar/CleanData/" + i + ".txt";
//			tc.strFileName = "/Users/apple/Documents/traj 2/CleanData/" + i+".txt";

                tc.loadTrajectory();
//			tc.iTrajectory = tc.iTrajectory.subList(140, 144);
                npoints = npoints + tc.iTrajectory.size();
                // System.out.print("" + i);
                // System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                // System.out.println(tc.strFileName);
                double stime = System.currentTimeMillis();
                GPSLine[] lines = tc.compress();
                double etime = System.currentTimeMillis();
                timeUsed += etime - stime;

                nlines = nlines + lines.length;


                double temp = tc.getAveErrorSED(tc.iTrajectory, lines);

                temp = (double) Math.round(temp * 100) / 100;

                avesed += temp;

                double newsed = tc.printToConsoleSED(tc.iTrajectory, lines);
                if (newsed > maxsed) maxsed = newsed;

                // tc.export(lines);

                // System.out.println(i+","+(double)100.0*lines.length/tc.iTrajectory.size()+","+(etime-stime)+","+temp/tc.iTrajectory.size());
                double mtime = tc.getMaxErrorTime(tc.iTrajectory, lines, tc.strFileName);
                if (mtime > maxtime) maxtime = mtime;

                avetime += tc.getAveErrorTime(tc.iTrajectory, lines, tc.strFileName);

//			System.out.println(i+","+(1.0 * avesed) / (1.0 * tc.iTrajectory.size())+","+(1.0 * avetime) / (1.0 * tc.iTrajectory.size()));

                // tc.export(lines);
            }


            // System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
            // System.out.println("Time Used = " + timeUsed + "ms");
            // System.out.println("MAXSED = " + maxsed + "m");
            // System.out.println("MAXPED = " + maxped + "m");
            // System.out.println("AveSEDError = " + (1.0 * avesed) / (1.0 * npoints) + "m.");
            // System.out.println("AvePEDError = " + (1.0 * aveped) / (1.0 * npoints) + "m.");
            // System.out.println(tc.getThreshold()+ ","+ (1.0 * nlines) / (1.0 * npoints) * 100 + "%,"+ timeUsed+"ms,"+maxped+"m,"+maxsed+"m,"+
            //(1.0 * aveped) / (1.0 * npoints)+"m,"+ (1.0 * avesed) / (1.0 * npoints)+"m");

            System.out.println(tc.getThreshold() + "," + (1.0 * nlines) / (1.0 * npoints) * 100 + "%," + timeUsed + "ms,"
                    //+ (1.0 * avesed) / (1.0 * npoints)
                    + "," + maxtime + "," + (1.0 * avetime) / (1.0 * npoints));
        }


    }

}
