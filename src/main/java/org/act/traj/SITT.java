package org.act.traj;

import org.act.util.*;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

public class SITT extends TrajectoryCompressor {

    public static double timeInsects = 0, timeGetPolygon = 0;
    public static boolean testIntersectTime = false;

    int isDebug = 0;
    int notin = 0;
    int m = 0;

    public double[] deltX = new double[m];
    public double[] deltY = new double[m];
    private static int sumPoint = 0;
    private static int sumVec = 0;

    public SITT(int m) {
        this.setDc(new PED());
        this.m = m;
        deltX = new double[m];
        deltY = new double[m];
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }
    }

    public SITT() {
        // TODO Auto-generated constructor stub
        this.m = 10;
        this.setDc(new PED());
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


        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

        index = 2;


        SectorBound sec = new SectorBound(-1, -1);
        sec = SectorBound.getSector(start, iTrajectory.get(iStart + 1), 2 * this.getThreshold());
        Optional<SectorBound> inters;
        inters = Optional.of(sec);
        GPSPoint ttt = iTrajectory.get(iStart + 1);
        double lm = TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude, iTrajectory.get(iStart + 1).latitude, iTrajectory.get(iStart + 1).longitude);

        GPSPoint s1 = iTrajectory.get(iStart);
        GPSPoint s2 = iTrajectory.get(iStart + 1);
        sumPoint++;
        sumVec++;

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);
            p.lineId = lineId;
            // System.out.println(p.index);

            double t = (double) (p.time - start.time) / (iTrajectory.get(iStart + 1).time - start.time);
            if (1 == 2) {// t<1.00001||t>100000) {
                lineId++;
                putLine(v, iStart, index - 1, lineId);
                iStart = index - 1;
                start = (GPSPoint) this.iTrajectory.get(iStart);

                iEnd = index - 1;

                sec = null;
                lm = 0;

                s1 = iTrajectory.get(iStart);
                s2 = iTrajectory.get(iStart + 1);
                sumPoint++;
                sumVec++;
            } else {

                boolean contains = inters.get().contains(start, p);

                // if ((!inters.isPresent()||sec==null)
                //			||(!contains)) {
                // if ((!inters.isPresent()||sec==null)||(TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)<lm&&TrajectoryCompressor.getDistanceOfP2P(p.latitude,p.longitude,ttt.latitude,ttt.longitude)>this.getThreshold()*this.getThreshold()/2/lm)||
                //			 (!contains)) {

                // final

                double sqr;
                if (lm >= this.getThreshold()) {
                    sqr = Math.sqrt(lm * lm - this.getThreshold() * this.getThreshold());
                } else {
                    sqr = 0;
                }
                double disS_P = TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude, p.latitude, p.longitude);
                // if ((!inters.isPresent()||sec==null)||(TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)==lm)||(TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)<=lm&&TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)<sqr)||
                //		 (!contains)) {

                if ((!inters.isPresent() || sec == null) || (disS_P < lm && disS_P < sqr) ||
                        (!contains)) {
                    // if ((!inters.isPresent()||sec==null)||(TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)<=lm)||
                    //(!contains)) {
                    // if (TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)==lm) {
                    //	System.out.println(TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)+"   "+sqr);
                    //}
                    lineId++;
                    putLine(v, iStart, index - 1, lineId);
                    iStart = index - 1;
                    start = (GPSPoint) this.iTrajectory.get(iStart);

                    iEnd = index - 1;

                    sec = null;
                    sec = SectorBound.getSector(start, p, 2 * this.getThreshold());
                    inters = Optional.of(sec);
                    lm = 0;

                    s1 = iTrajectory.get(iStart);
                    s2 = iTrajectory.get(iStart + 1);
                    sumPoint++;
                    sumVec++;

                } else if (checkv(s1, s2, p)) {
                    s2 = p;
                    sumVec++;
                }
                if (sec == null || !inters.isPresent()) {
				/*if (TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)<this.getThreshold()) {
					sec = SectorBound.getSector(start, p, 100000*this.getThreshold());
					sec.setAngleL(0.001);
					sec.setAngleU(2*Math.PI-0.001);
					
					inters = Optional.of(sec);
				}
				else {*/
                    sec = SectorBound.getSector(start, p, 2 * this.getThreshold());
                    inters = Optional.of(sec);
                    //}
                } else {

                    Optional<SectorBound> res;
                    Optional.of(sec);
                    SectorBound temp = new SectorBound();
				
				/*if (TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude)<this.getThreshold()) {
					temp.setAngleL(0.001);
					temp.setAngleU(2*Math.PI-0.001);
				}
				else {
					temp = SectorBound.getSector(start, p, 2*this.getThreshold());
				}*/

                    temp = SectorBound.getSector(start, p, 2 * this.getThreshold());
                    Optional<SectorBound> t1;
                    t1 = Optional.of(temp);

                    res = sec.intersects(t1);

                    if (res.isPresent()) {
                        sec = res.get();
                        inters = Optional.of(sec);
                    } else {
                        sec = null;
                    }
                    // double dis = TrajectoryCompressor.getDistanceOfP2P(start.latitude,start.longitude,p.latitude,p.longitude);
                    // if (dis>lm) {lm = dis; ttt=p;}
                }
            }
            double dis = TrajectoryCompressor.getDistanceOfP2P(start.latitude, start.longitude, p.latitude, p.longitude);
            if (dis > lm) {
                lm = dis;
                ttt = p;
            }

            iEnd = index;
            index++;
        }
        // System.out.println(iStart + " !!!!"+iEnd + "    " + iTrajectory.size());

        putLine(v, iStart, index - 1, lineId);

        GPSLine[] lines = new GPSLine[v.size()];
        v.toArray(lines);
        sumPoint++;

        return lines;

    }

    private boolean checkv(GPSPoint vs, GPSPoint ve, GPSPoint p) {
        double dis = TrajectoryCompressor.getDistance(vs, ve, p);

        return dis > getThreshold();
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

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "SITT";

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

        String time = "SITT-time ";
        String CR = "SITT-CR ";
        String SITTave = "SITT-ave ";
		
		
		/*java.io.BufferedReader reader=new java.io.BufferedReader(new java.io.FileReader("/Users/apple/Documents/geo_list_final.txt"));
		java.util.List list=new java.util.ArrayList();
		String line;
		while((line=reader.readLine())!=null)
		{
			list.add(line);
		}
		reader.close();*/
        // for(java.util.Iterator i=list.iterator();i.hasNext();)
        // System.out.println(i.next());


        for (int zeta = 20; zeta <= 20; zeta += 10) {
            //		if(zeta>=50&&zeta<=110) zeta+=10;
            //		if(zeta>=130&&zeta<=170) zeta+=30;

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

            SITT tc = new SITT(16);
            SITT.timeInsects = 0;
            SITT.timeGetPolygon = 0;

            tc.setThreshold(zeta);
            tc.radialThreshold = 1.0 * zeta;
            // tc.deviation = 13;
            // System.out.println("SITT  mopsi.");

            // System.out.println("Threshold : " + tc.getThreshold() + " m.");
            GPSLine[] lines = null;
            for (int i = 1; i <= 1; i++) {
                // for(java.util.Iterator i=list.iterator();i.hasNext();) {
//				if (i==7 || i==85 || i==87 || i==131 || i==144 || i==146 || i==157 || i==167 || i==176) continue;
                // tc.strFileName = "/Users/apple/Documents/mopsi/"+i+".txt";//23
                // tc.strFileName = "/Users/apple/Documents/"+i+".txt";
                // tc.strFileName = "/Users/apple/Documents/geolife/"+i+".txt";  //23
                // tc.strFileName = "/Users/apple/Documents/traj/geolife/CleanData/"+i.next();
//				tc.strFileName = "/Users/apple/Documents/traj/SerCar/CleanData/" + i + ".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\SerCar\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\geolife\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\mopsi\\join\\" + i+".txt";
//				tc.strFileName = "/Users/apple/Documents/geolife/traj/CleanData/" + i +".txt";
                tc.strFileName = "/Users/apple/Documents/geolife1000.txt";

                tc.loadTrajectory();
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress();
                double etime = System.currentTimeMillis();
                // ave+=tc.getAveErrorSED(tc.iTrajectory, lines);

                avesed += tc.getAveErrorSED(tc.iTrajectory, lines);
                aveped += tc.getAveErrorPED(tc.iTrajectory, lines);

                timeUsed += (etime - stime);

                nsumPoint += sumPoint;
                nsumVec += sumVec;


                double newped = tc.printToConsolePED(tc.iTrajectory, lines);
                // System.out.println("maxPED error : " + newped);
                nlines = nlines + lines.length;
                tc.export(lines);
                // System.out.println(tc.getThreshold()+","+newped+","+1.0*lines.length/tc.iTrajectory.size()*100+"%");
                if (newped > tc.getThreshold()) {
                    System.out.println("!!!!!!!!!!!!!!!!!" + tc.strFileName + "  " + tc.getThreshold() + " " + newped);
                }
            }
            // System.out.println("????????");

            time += timeUsed + ",";
            CR += (1.0 * nlines) / (1.0 * npoints) * 100 + ",";
            SITTave += (1.0 * ave) / (1.0 * npoints) + ",";

            double avesed_p = (1.0 * avesed) / (1.0 * npoints);
            double aveped_p = (1.0 * aveped) / (1.0 * npoints);

//			System.out.println(tc.getThreshold()+","+nsumPoint+","+nsumVec+","+npoints+","+timeUsed+","+avesed_p+","+aveped_p);
            System.out.println(tc.getThreshold() + "," + nsumPoint + "," + nsumVec + "," + npoints + "," + ((1.0 * nlines) / (1.0 * npoints) * 100) + "," + timeUsed + "," + avesed_p + "," + aveped_p);


            // System.out.println();

            // System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");

            // System.out.println("Time Used = " + timeUsed + "ms");
            // System.out.println("AveSEDError = " + (1.0 * avesed) / (1.0 * npoints) + "m.");
            // System.out.println(time);
            // System.out.println(CR);
            // System.out.println(SITTave);

        }
    }
}
