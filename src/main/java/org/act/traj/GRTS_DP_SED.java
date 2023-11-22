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
public class GRTS_DP_SED extends TrajectoryCompressor {

    int count = 0;

    // private double SEDThreshold=0;


    public GPSLine[] compress() {
        return this.compressByGRTS(this.iTrajectory, 0, this.iTrajectory.size() - 1);
    }

    private GPSLine[] subcompressbyDP(int iStart, int iEnd) {
        DisCalculator sed = new SED();
        DouglasPeucker sub = new DouglasPeucker(sed);
        sub.setThreshold(this.getThreshold());
        GPSLine[] linesU = sub.compressByDP(this.iTrajectory, iStart, iEnd);

        return linesU;
    }

    private GPSLine[] compressByGRTS(List<GPSPoint> iTraj, int iStart, int iEnd) {


        GPSLine[] lines = new GPSLine[iEnd];

        ArrayList<GPSLine> BufferU = new ArrayList<>();
        int j = 0;
        int k = 0;
        int Sstart = 0;

        GPSPoint loc = new GPSPoint();
        BufferU.clear();

        int cntt = 0;
        int testcnt = 0;
        int cntfortest = 0;
        int cntfortest1 = 0;
        for (int i = iStart + 1; i <= iEnd; i++) {

            double t = (double) (iTraj.get(i).time - iTraj.get(j).time) / (iTraj.get(j + 1).time - iTraj.get(j).time);

            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), t);

            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            double alpha = DisCalculator.getAngleOfVector(iTraj.get(j).latitude, iTraj.get(j).longitude, iTraj.get(j + 1).latitude, iTraj.get(j + 1).longitude);
            // System.out.println(DADis);
            // if (disOfP2P >Threshold) {
            // if (t<1.0001||t>10000||disOfP2P>this.SEDThreshold||(( (DADis > this.DADThreshold) || (DADis < -this.DADThreshold) ) && this.DAD_flag) ||(PEDis > this.PEDThreshold && this.PED_flag)) {
            if (disOfP2P > this.threshold) {
                cntt++;
                GPSLine[] linesU = subcompressbyDP(Sstart, i);

                for (int n = 0; n < linesU.length; n++) {
                    BufferU.add(linesU[n]);
                }

                testcnt += linesU.length;

                int cnt = 0;
                if (BufferU.size() > 1)
                    for (int idu = 0; idu < BufferU.size(); idu++) {
                        lines[k++] = BufferU.get(idu);
                    }

                // System.out.println("xxxxxxx  +  "+cnt);
                if (BufferU.size() > 1) {
                    // Sstart=BufferU.get(BufferU.size()-1).endPoint.index;
                    Sstart = i;
                    // System.out.println(BufferU.get(BufferU.size()-1).endPoint.index);
                    // System.out.println(i);

                }

                j = i - 1;
                BufferU.clear();

            }
        }

        // System.out.println(Sstart+ "  + " + iEnd + "   !");
        GPSLine[] linesU = subcompressbyDP(Sstart, iEnd);


        for (int n = 0; n < linesU.length; n++) {
            lines[k++] = linesU[n];
        }

        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
        }
        
        /*for (int i=0;i<k-1;i++)
        {
        	if (liness[i].startPoint.index==liness[i+1].startPoint.index)
        	System.out.println(liness[i].startPoint.index+ "    +    "+liness[i].endPoint.index);
        }*/
        /*System.out.println("????");
        System.out.println(Sstart);
        System.out.println(j);
        System.out.println(iEnd);*/
        // System.out.println(linesU.length);
      
        /*for (int xx=linesU.length-10;xx<linesU.length;xx++)
        {
        	System.out.println(linesU[xx].startPoint.index+"   +   "+linesU[xx].endPoint.index);
        }
        */
        
        
        
       /* System.out.println("cntt !!!!!!!!!!!!!!!"+cntt);
        System.out.println("sum of BufferU length!!!!!!!!!!    "+testcnt);*/
        // System.out.println("cntfortest!!!!!!!!!!    "+cntfortest);
        
        /*for(int i=1;i<k;i++) {
        	if (liness[i].startPoint.index!=liness[i-1].endPoint.index)
        	{
        		System.out.println("asdasd");
        	}
        }*/

        return liness;

    }

    @Override
    public String toString() {
        return "GRTS" + getDc();
    }

    public static void main(String[] args) throws IOException {
        int npoints = 0;
        int nlines = 0;
        long timeUsed = 0;

        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();
        DisCalculator tdsed = new ThreeSED();


        double SEDThreshold, PEDThreshold, DADThreshold;
        SEDThreshold = 50;
        PEDThreshold = 0;
        DADThreshold = 0;


        // System.out.println("Threshold : " + tc.getThreshold() + " m.");
        TrajectoryCompressor tc = new GRTS_DP_SED();
        for (SEDThreshold = 10; SEDThreshold <= 40; SEDThreshold += 10) {
            {
                // System.out.println("SEDThreshold : " + SEDThreshold);
                // System.out.println("PEDThreshold : " + PEDThreshold);
                // System.out.println("DADThreshold : " + DADThreshold);
                double totyasuo = 0;

                for (int i = 1; i <= 1; i++) {

                    // if (i==7) continue;

//		        	tc.strFileName = "/Users/apple/Documents/"+i +".txt";
                    tc.strFileName = "/Users/apple/Documents/mopsi/" + i + ".txt";


                    System.out.println(tc.strFileName);
                    index = 0;
                    tc.loadTrajectory();
                    tc.setDc(new SED());
//		            tc.setDc(new ThreeSED());

                    //			tc.iTrajectory = tc.iTrajectory.subList(140, 144);
                    npoints = npoints + tc.iTrajectory.size();
                    // System.out.print("" + i);
                    // System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                    tc.setThreshold(SEDThreshold);
                    double stime = System.currentTimeMillis();
                    GPSLine[] lines = tc.compress();

                    // System.out.print("COMPRESS DONE!");

                    double etime = System.currentTimeMillis();
                    timeUsed += (etime - stime);

                    nlines = nlines + lines.length;

                    double ErrorSED = tc.printToConsoleSED(tc.iTrajectory, lines);
                    // System.out.println("ErrorSED :  " + ErrorSED );
                    // System.out.println("	Compress By GRTS to : (" + lines.length + ") lines");
                    double xxx = tc.printToConsoledad(tc.iTrajectory, lines);
                    if (DADThreshold != 0 && DADThreshold < xxx) {
                        System.out.println("error!!!!!!!!!!!!!!!!!!!!!");
                        break;
                    }
                    // System.out.println(DADThreshold + " dad and error "+xxx);


                    totyasuo += (1.0 * lines.length / tc.iTrajectory.size());
                    double ErrorPED = tc.printToConsolePED(tc.iTrajectory, lines);

                    // System.out.println("ErrorSED :  " + ErrorSED );

                    if (ErrorSED > SEDThreshold) {
                        System.out.println("!!!???????????????????!!!");
                    }

                    System.out.println(SEDThreshold + "," + ErrorSED + "," + (1.0 * nlines) / (1.0 * npoints) * 100 + "%");


                    // System.out.println("ErrorPED :  " + ErrorPED );
                    // System.out.println("	Using Time :" + (etime - stime) + "mS");
                    // tc.printToConsole(tc.iTrajectory, lines);
                    // tc.export(lines);
                    // System.out.println((1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
                    // System.out.println(tc.iTrajectory.size());
                }
                // totyasuo=totyasuo/9;
                // System.out.println("ratio : " + totyasuo);
            }
        }
       /* System.out.println(System.currentTimeMillis());
        System.out.println(nlines);
        System.out.println(npoints);
	    System.out.println("	Using Time :" +  timeUsed+ "mS");
        System.out.println("Compress Ratio = " +((1.0 * nlines) / (1.0 * npoints) )* 100 + "%.");*/
    }
}
