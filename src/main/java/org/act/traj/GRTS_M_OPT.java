package org.act.traj;

import org.act.util.*;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class GRTS_M_OPT extends TrajectoryCompressor {
    private static int sumPoint = 0;
    private static int sumVec = 0;
    private static int sumint = 0;
    int count = 0;

    public GRTS_M_OPT(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByGRTS_M_OPT(this.iTrajectory, 0, this.iTrajectory.size() - 1);
    }

    /*private GPSLine[] subcompress(int iStart, int iEnd) {
    	DisCalculator sed = new SED();
		//CDR sub = new CDR(sed);
		DouglasPeucker sub=new DouglasPeucker(sed);
		sub.setThreshold(this.getThreshold());
		//sub.iTrajectory = sub.iTrajectory.subList(iStart,iEnd+1);
		GPSLine[] linesU = sub.compressByDP( this.iTrajectory, iStart,  iEnd);
		//System.out.println(iStart);
		//System.out.println(iEnd);
		return linesU;
    }*/
    private int VdeleteU(ArrayList<GPSLine> BufferU, ArrayList<GPSLine> BufferV) {
        if (BufferV.isEmpty() || BufferV == null) return 0;
        for (int j = 0; j < BufferV.size(); j++) {
            if (BufferV.get(j) != BufferU.get(j))
                return j;
        }
        return 0;
    }


    private GPSLine[] compressByGRTS_M_OPT(List<GPSPoint> iTraj, int iStart, int iEnd) {

        // GPSPoint lineStart = iTraj.get(iStart);
        // GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd];

        ArrayList<GPSLine> BufferU = new ArrayList<>();
        // ArrayList<GPSLine> BufferV=new ArrayList<GPSLine>();
        ArrayList<Integer> removeS = new ArrayList<>();
        GPSLine lineV = new GPSLine();
        int j = 0;
        int k = 0;
        int Sstart = 0;
        double Bufferm = getBuffer();
        double Threshold = getThreshold();
        GPSPoint loc = new GPSPoint();
        sumPoint = 0;
        sumVec = 0;
        // int sumPoint=0, sumVec=0, sumint=0;
        for (int i = iStart + 1; i <= iEnd; i++) {
            int iremove = 0;
            // System.out.println("杩欐寰幆鐨勮繃绋嬫槸"+Sstart+"he"+i);
            for (int n = Sstart + 1; n < i; n++) {
                // if(n<10) System.out.println("removeS"+removeS);
                if (((removeS != null) && (iremove < removeS.size())) && n == removeS.get(iremove)) {
                    // System.out.println("杩欐绉婚櫎鏁版嵁浜�"+n);
                    iremove++;
                    continue;
                }
                // if(n<30) System.out.println("杩欐椂鍊欑偣鏄�"+n);

                double time = (double) (iTraj.get(n).time - iTraj.get(Sstart).time) / (iTraj.get(i).time - iTraj.get(Sstart).time);
                GPSPoint temp1 = DisCalculator.getEndPoint(iTraj.get(Sstart), iTraj.get(i), time);
                double disOfSi = DisCalculator.getDistanceOfP2P(iTraj.get(n), temp1);
                if ((disOfSi > Threshold) || (i - Sstart == Bufferm)) {
//                	System.out.println("杩欐槸鍒拌揪");
//                	System.out.println(i);
                    // System.out.println("杩欐槸diyici "+i);
                    GPSLine line = new GPSLine();
                    line.setStartPoint((GPSPoint) iTraj.get(Sstart));
                    line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                    
                    /*if (iTraj.get(Sstart).index>173869) {
                    	System.out.println("?????");
                    }
                    if (iTraj.get(i-1).index>173869) {
                    	System.out.println("?!!!!?");
                    }*/

                    BufferU.add(line);
                    Sstart = i - 1;
                    removeS.clear();
                    break;
                }
            }
            // removeS.clear();
            iremove = 0;
            for (int n = Sstart + 1; n < i; n++) {
                if (((removeS != null) && (iremove < removeS.size())) && n == removeS.get(iremove)) {
                    // System.out.println("杩欐绉婚櫎鏁版嵁浜�"+n);
                    iremove++;
                    continue;
                }
                for (int q = n + 1; q <= i; q++) {
                    // 绉婚櫎婊¤冻鏉′欢鐨勭偣
                    double time2 = (double) (iTraj.get(n).time - iTraj.get(Sstart).time) / (iTraj.get(q).time - iTraj.get(Sstart).time);
                    GPSPoint temp = DisCalculator.getEndPoint(iTraj.get(Sstart), iTraj.get(q), time2);
                    double disOfSiSr = DisCalculator.getDistanceOfP2P(iTraj.get(n), temp);
                    double ratio = (iTraj.get(n).time - iTraj.get(Sstart).time) / (double) (iTraj.get(q).time - iTraj.get(Sstart).time);
                    double radius = Threshold * ratio;
//                    if(n<=16) {
//                		System.out.println(iTraj.get(n).time);
//                		System.out.println("ratio"+ratio+"banjing"+radius);
//                	}
                    if ((radius + disOfSiSr) < Threshold) {
//                    	if(n==16) {
//                    		System.out.println(q);
//                    		System.out.println("zhegesi"+disOfSiSr+"banjing"+radius);
//                    	}
                        removeS.add(n);
                        break;
                    }
                }

            }

            double time3 = (double) (iTraj.get(i).time - iTraj.get(j).time) / (iTraj.get(j + 2).time - iTraj.get(j).time);
            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 2), time3);
            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            if (disOfP2P > Threshold) {
                // System.out.println("杩欐槸鍒拌揪"+i);

                GPSLine line = new GPSLine();
                line.setStartPoint((GPSPoint) iTraj.get(Sstart));
                line.setEndPoint((GPSPoint) iTraj.get(i - 1));

//                if (iTraj.get(Sstart).index>173869) {
//                	System.out.println("????? 1111");
//                }
//                if (iTraj.get(i-1).index>173869) {
//                	System.out.println("?!!!!? 1111");
//                }

                BufferU.add(line);
                sumPoint += BufferU.size();
                if (lineV == BufferU.get(0)) {
                    sumPoint--;
                }
                if (BufferU.size() > 1) {
                    for (int n = 0; n < BufferU.size() - 1; n++) {
                        lines[k++] = BufferU.get(n);
                        // System.out.println("鍘嬬缉杞ㄨ抗"+BufferU.get(n));
                    }
                }

                lineV = BufferU.get(BufferU.size() - 1);
                BufferU.clear();
                sumVec++;
                j = i - 1;

            }
        }
        sumPoint += BufferU.size();
        for (int n = 0; n < BufferU.size(); n++) {
            lines[k++] = BufferU.get(n);
        }

        GPSLine line = new GPSLine();
        line.setStartPoint((GPSPoint) iTraj.get(Sstart));
        line.setEndPoint((GPSPoint) iTraj.get(iEnd));
        if (k != 0) {
            if (lines[k - 1].endPoint.index == iTraj.get(Sstart).index) {
                lines[k++] = line;
                sumPoint++;
            } else {
                lines[k - 1] = line;
            }
        } else {
            lines[k++] = line;
            sumPoint++;
        }


        // System.out.println("sumPoinr"+sumPoint+"sumVec"+sumVec);


        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
            // System.out.println(lines[i]);
        }

        return liness;

    }

    @Override
    public String toString() {
        return "GRTS_M_OPT" + getDc();
    }

    public static void main(String[] args) throws Exception {
        int npoints = 0;
        int nlines = 0;
        int nsumPoint = 0;
        int nsumVec = 0;
        int nsumload = 0;
        double ave = 0;

        long timeUsed = 0;
        DisCalculator ded = new DED();
        DisCalculator ped = new PED();
        DisCalculator sed = new SED();
        int errorPoints;
        String time = "CISED-LDR-time ";
        String CR = "CISED-LDR-CR ";
        String CISEDpoints = "CISED-LDR-points ";
        String CISEDvecs = "CISED-LDR-vecs ";
        String CISEDtrans = "CISED-LDR-trans ";
        String CISEDave = "CISED-LDR-ave ";
		
		/*java.io.BufferedReader reader=new java.io.BufferedReader(new java.io.FileReader("/Users/apple/Documents/geo_list_final.txt"));
		java.util.List list=new java.util.ArrayList();
		String line;
		while((line=reader.readLine())!=null)
		{
			list.add(line);
		}
		reader.close();*/

        for (int zeta = 10; zeta <= 40; zeta += 10) {
            npoints = 0;
            nlines = 0;
            ave = 0;
            nsumPoint = 0;
            nsumVec = 0;
            nsumload = 0;

            double avesed = 0;
            double maxsed = 0;
            double aveped = 0;
            double maxped = 0;
            timeUsed = 0;
            // if(zeta>=50&&zeta<=110) zeta+=10;
            // if(zeta>=130&&zeta<=170) zeta+=30;
            TrajectoryCompressor tc = new GRTS_M_OPT(sed);
            tc.setBuffer(500);
            tc.setThreshold(zeta);
            tc.radialThreshold = 1.0 * zeta;
            GPSLine[] lines = null;
            for (int i = 1; i <= 1; i++) {
                // if (i==7 || i==85 || i==87 || i==131 || i==144 || i==146 || i==157 || i==167 || i==176) continue;
                //	tc.strFileName = "/Users/apple/Documents/mopsi/" + i + ".txt";
                // tc.strFileName = "/Users/apple/Documents/" + i + ".txt";
                // for(java.util.Iterator i=list.iterator();i.hasNext();) {
                ///Users/apple/Documents/traj/geolife/CleanData/182.txt
                // tc.strFileName = "/Users/apple/Documents/traj/geolife/CleanData/"+i.next();
                // tc.strFileName="/Users/apple/Documents/traj/geolife/CleanData/182.txt";

//				tc.strFileName = "/Users/apple/Documents/traj/SerCar/CleanData/" + i + ".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\SerCar\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\geolife\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\mopsi\\join\\" + i+".txt";
//				tc.strFileName = "/Users/apple/Documents/geolife/traj/CleanData/" + i +".txt";

                tc.strFileName = "/Users/apple/Documents/" + i + ".txt";

                tc.loadTrajectory();
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress();
                double etime = System.currentTimeMillis();
                nsumPoint += sumPoint;
                nsumVec += sumVec;
                timeUsed += (etime - stime);
                // tc.printToConsoleSED(tc.iTrajectory, lines);
                nlines = nlines + lines.length;
                // if (lines.length<3) {
                //	System.out.println("!!!!! "+lines.length+ "    "+tc.strFileName);
                //}
                // ave+=tc.getAveErrorSED(tc.iTrajectory, lines);
                avesed += tc.getAveErrorSED(tc.iTrajectory, lines);
                aveped += tc.getAveErrorPED(tc.iTrajectory, lines);
                // tc.export(lines);
                double newsed = tc.printToConsoleSED(tc.iTrajectory, lines);
                if (newsed > tc.getThreshold()) {
                    System.out.println("!!!!!!!!!!!!!!!!!" + tc.strFileName + "  " + tc.getThreshold() + " " + newsed);
                }
                // System.out.println(tc.getThreshold()+","+newsed+","+1.0*lines.length/tc.iTrajectory.size()*100+"%");
            }

            time += timeUsed + ",";
            CR += (1.0 * nlines) / (1.0 * npoints) * 100 + ",";
            CISEDpoints += nsumPoint + ",";
            CISEDvecs += nsumVec + ",";
            CISEDtrans += (nsumPoint * 24 + nsumVec * 16) + ",";
            CISEDave += (1.0 * ave) / (1.0 * npoints) + ",";

            double avesed_p = (1.0 * avesed) / (1.0 * npoints);
            double aveped_p = (1.0 * aveped) / (1.0 * npoints);

            System.out.println(tc.getThreshold() + "," + nsumPoint + "," + nsumVec + "," + npoints + "," + ((1.0 * nlines) / (1.0 * npoints) * 100) + "," + timeUsed + "," + avesed_p + "," + aveped_p);
			

			
			/*System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
			System.out.println("nlines = "+nlines);
			System.out.println("Time Used = " + timeUsed + "ms");
			System.out.println("AveError = " + (1.0 * ave) / (1.0 * npoints) + "m.");
			System.out.println("nsumPoint = " + nsumPoint );
			System.out.println("nsumVec= "+nsumVec);
			System.out.println("nsumload = " + (nsumPoint*24 +nsumVec*16));
			System.out.println();
			System.out.println(time);
			System.out.println(CR);
			System.out.println(CISEDpoints);
			System.out.println(CISEDvecs);
			System.out.println(CISEDtrans);
			System.out.println(CISEDave);
			System.out.println("mopsi");*/
        }


//		double newsed;
//		
//		int zeta=10;
//		
//		sed = new SED();
//		GRTS_M_OPT tc = new GRTS_M_OPT(sed);
//		tc.setBuffer(500);
//		tc.setThreshold(zeta); 
//		tc.radialThreshold = 1.0 * zeta;
//		GPSLine[] lines = null;	
//		tc.strFileName = "/Users/apple/Documents/1.txt";
//		tc.loadTrajectory();
//		lines = tc.compress();
//		newsed=tc.printToConsoleSED(tc.iTrajectory, lines);
//		//System.out.println(tc.getThreshold()+","+newsed+","+1.0*lines.length/tc.iTrajectory.size()*100+"%");
//		System.out.println(tc.getThreshold()+","+newsed+","+1.0*lines.length/tc.iTrajectory.size()*100+"%"+"   "+lines.length+" "+tc.iTrajectory.size());
//		
//		zeta=20;
//		sed = new SED();
//		tc = new GRTS_M_OPT(sed);
//		tc.setBuffer(500);
//		tc.setThreshold(zeta); 
//		tc.radialThreshold = 1.0 * zeta;
//		lines = null;	
//		tc.strFileName = "/Users/apple/Documents/1.txt";
//		tc.loadTrajectory();
//		lines = tc.compress();
//		newsed=tc.printToConsoleSED(tc.iTrajectory, lines);
//		System.out.println(tc.getThreshold()+","+newsed+","+1.0*lines.length/tc.iTrajectory.size()*100+"%"+"   "+lines.length+" "+tc.iTrajectory.size());
    }


}
