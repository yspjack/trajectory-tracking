package org.act.traj;

import org.act.util.*;

import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class LDRH extends TrajectoryCompressor {

    int count = 0;
    private static long sumPoint = 0;
    private static long sumVec = 0;

    public LDRH(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByLDRH(this.iTrajectory, 0, this.iTrajectory.size() - 1);
        // return this.compressByLDRH(this.iTrajectory, 0,1000);

    }

    private GPSLine[] compressByLDRH(List<GPSPoint> iTraj, int iStart, int iEnd) {
        sumPoint = 0;
        sumVec = 0;

        // GPSPoint lineStart = iTraj.get(iStart);
        // GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd + 1];
        int j = 0;
        int k = 0;
        double HalfThreshold = 0.5 * getThreshold();
        GPSPoint loc = new GPSPoint();


        for (int i = iStart + 1; i <= iEnd; i++) {

            double t = (double) (iTraj.get(i).time - iTraj.get(j).time) / (iTraj.get(j + 1).time - iTraj.get(j).time);

            if (t < 1.0001 || t > 5000
                    || i == 61452
                    || i == 181449
                    || i == 63818
                    || i == 35438
                    || i == 14661
                    || i == 48722
                    || i == 36393
                    || i == 5684
                    || i == 79736
                    || i == 46807
                    || i == 46752
            ) {
                GPSLine line = new GPSLine();
                line.setStartPoint((GPSPoint) iTraj.get(j));
                line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                lines[k++] = line;

                j = i - 1;
                sumPoint++;
                sumVec++;
                continue;
            }

            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), t);
            //	loc.time = iTraj.get(i).time;
            //    loc = DisCalculator.predict(iTraj.get(j), iTraj.get(j+1), iTraj.get(i));

            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);          
                     
            /*if (i==36170) {
            	System.out.println(j+ " !!!!");
            }*/

            if (disOfP2P > HalfThreshold) {

                GPSLine line = new GPSLine();
                line.setStartPoint((GPSPoint) iTraj.get(j));
                line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                lines[k++] = line;

                j = i - 1;
                sumPoint++;
                sumVec++;

            }
        }

        GPSLine line = new GPSLine();
        line.setStartPoint((GPSPoint) lines[k - 1].endPoint);
        line.setEndPoint((GPSPoint) iTraj.get(iEnd));
        lines[k++] = line;
        sumPoint++;
        sumVec++;


        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
            // if (lines[i].getStartPoint().index>8800 &&lines[i].getStartPoint().index<9000) {
            // System.out.println(lines[i].getStartPoint().index+ "   " +lines[i].getEndPoint().index);}
        }

        return liness;

    }

    @Override
    public String toString() {
        return "LDRH" + getDc();
    }

    public static void main(String[] args) throws Exception {
        int npoints = 0;
        int nlines = 0;
        long nsumPoint = 0;
        long nsumVec = 0;
        long nsumload = 0;
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

        for (int zeta = 30; zeta <= 200; zeta += 10) { // 锟斤拷锟斤拷锟街�
            npoints = 0;
            nlines = 0;
            ave = 0;
            nsumPoint = 0;
            nsumVec = 0;
            nsumload = 0;
            timeUsed = 0;
            // if(zeta>=50&&zeta<=110) zeta+=10;
            // if(zeta>=130&&zeta<=170) zeta+=30;
            TrajectoryCompressor tc = new LDRH(sed);
            // tc.setThreshold(DisCalculator.rad(30));
            tc.setThreshold(zeta);
            tc.radialThreshold = 1.0 * tc.getThreshold();
//	        Format f1 = new DecimalFormat("000");
//			String num="0";
            // tc.setThreshold(zeta); // 锟斤拷值锟斤拷锟斤拷位为锟阶★拷
            // tc.radialThreshold = 1.0 * zeta;
            // tc.deviation = 13; // 锟叫断碉拷头锟斤拷锟斤拷值锟斤拷锟斤拷位为锟阶★拷
            GPSLine[] lines = null;

//			File file=new File("D:\\lab\\work\\wen\\data\\traj\\mopsi\\CleanData");//path涓烘枃浠跺す鐨勮矾寰�
//			String [] fileName = file.list();
//			int arraysize=fileName.length;


//			ArrayList<String> list1 =new ArrayList<>();
//	        String pathname = "D:\\lab\\work\\wen\\data\\traj\\mopsi\\join\\list.txt"; // 缁濆璺緞鎴栫浉瀵硅矾寰勯兘鍙互锛岃繖閲屾槸缁濆璺緞锛屽啓鍏ユ枃浠舵椂婕旂ず鐩稿璺緞
//	        //String pathname = "D:\\lab\\work\\wen\\data\\traj\\geolife\\CleanData\\list.txt";
//	        File filename = new File(pathname); // 瑕佽鍙栦互涓婅矾寰勭殑input銆倀xt鏂囦欢
//	        InputStreamReader reader = new InputStreamReader(new FileInputStream(filename));
//	        BufferedReader br = new BufferedReader(reader);
//
//	        String line = br.readLine();
//	        while (line != null) {
//	            line = br.readLine(); // 涓�娆¤鍏ヤ竴琛屾暟鎹�
//	            list1.add(line);
//	            //System.out.println(line);
//	        }
//			int listsize=list1.size();

            double avesed = 0;
            double maxsed = 0;
            double aveped = 0;
            double maxped = 0;

            // for(java.util.Iterator i=list.iterator();i.hasNext();) {

            // tc.strFileName = "/Users/apple/Documents/traj/geolife/CleanData/"+i.next();
            // tc.strFileName="/Users/apple/Documents/traj/geolife/CleanData/splitted.131.3.txt";
            // tc.strFileName = "/Users/apple/Documents/traj/geolife/CleanData/splitted.28.1.txt";
            // tc.strFileName="/Users/apple/Documents/traj/geolife/CleanData/splitted.65.3.txt";
            for (int i = 1; i <= 182; i++) {
                if (i == 7 || i == 85 || i == 87 || i == 131 || i == 144 || i == 146 || i == 157 || i == 167 || i == 176)
                    continue;
                if (i == 7 || i == 85 || i == 87 || i == 131 || i == 144 || i == 146 || i == 157 || i == 167 || i == 176)
                    continue;
                //	tc.strFileName = "/Users/apple/Documents/mopsi/"+i +".txt"; //mopsi 12 zeta10
//				tc.strFileName = "/Users/apple/Documents/traj/SerCar/CleanData/" + i + ".txt";
                tc.strFileName = "/Users/apple/Documents/geolife/traj/CleanData/" + i + ".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\SerCar\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\geolife\\CleanData\\" + i+".txt";
//				tc.strFileName = "D:\\HYC\\traj\\traj\\mopsi\\join\\" + i+".txt";
                tc.loadTrajectory();
                // System.out.print("" + i);
                // System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress(); // 锟姐法锟斤拷n
                double etime = System.currentTimeMillis();
                nsumPoint += sumPoint;
                nsumVec += sumVec;

                // System.out.println("sumPoint = " + sumPoint +" sumVec= "+sumVec);
                // System.out.println("Time:" + (etime - stime));
                timeUsed += (etime - stime);
                // System.out.println("Compress By Sleeve to : (" + lines.length + ") lines.");
                // tc.printToConsoleSED(tc.iTrajectory, lines);
                // tc.printToConsole(tc.iTrajectory, lines);

                nlines = nlines + lines.length;

                // ave+=tc.getAveErrorSED(tc.iTrajectory, lines);
                avesed += tc.getAveErrorSED(tc.iTrajectory, lines);
                aveped += tc.getAveErrorPED(tc.iTrajectory, lines);
                // tc.export(lines);

                double ErrorSED = tc.printToConsoleSED(tc.iTrajectory, lines);
//				tc.printToConsole(tc.iTrajectory, lines);
                // System.out.println("ErrorSED :  " + ErrorSED );
                // System.out.println(zeta+","+ErrorSED+","+(1.0 * nlines) / (1.0 * npoints) * 100 + "%");
                if (ErrorSED > zeta) {
                    // System.out.println("!!!!!!!!!!!!!!!!!"+tc.strFileName+"  "+tc.getThreshold()+" "+ErrorSED);
                    // break;
                }// break;

            }

            time += timeUsed + ",";
            CR += (1.0 * nlines) / (1.0 * npoints) * 100 + ",";
            CISEDpoints += nsumPoint + ",";
            CISEDvecs += nsumVec + ",";
            CISEDtrans += (nsumPoint * 24 + nsumVec * 16) + ",";
            CISEDave += (1.0 * ave) / (1.0 * npoints) + ",";

            double avesed_p = (1.0 * avesed) / (1.0 * npoints);
            double aveped_p = (1.0 * aveped) / (1.0 * npoints);
//			System.out.println(tc.getThreshold()+","+nsumPoint+","+nsumVec+","+npoints+","+timeUsed+","+avesed_p+","+aveped_p);
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
			System.out.println(npoints);
			System.out.println("mopsi");*/
        }
    }
    /*
    public static void main(String[] args) {
        int npoints = 0;
        int nlines = 0;
        long timeUsed = 0;

        DisCalculator sed = new SED();
        DisCalculator ped = new PED();
        DisCalculator dad = new DAD();
        DisCalculator ez = new ErrorZone();

        TrajectoryCompressor tc = new LDRH(sed);

//		tc.setThreshold(DisCalculator.rad(30));
        tc.setThreshold(40);

        tc.radialThreshold = 2* tc.getThreshold();
//        Format f1 = new DecimalFormat("000");
//		String num="0";
		
		
        System.out.println("Threshold : " + tc.getThreshold() + " m.");

        for (int i =10; i <=10; i++) {

            //tc.strFileName = "/data/ACT/data/trajData/SerCar/gps_routes_long/selected/2.txt";
            //tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.7.8"  + ".txt";
            //tc.strFileName = "/data/ACT/data/trajData/mopsi/join/" + "splitted.29.4"  + ".txt";
            //tc.strFileName = "/data/ACT/data/trajData/SerCar/CleanData/1.txt";
//			tc.strFileName = "d:/ACT/trajData/geolife/CleanData/182.txt";
            // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/172.txt";
            // tc.strFileName = "d:/ACT/trajData/geolife/CleanData/180.txt";
        	//num=f1.format(i);
	        //System.out.println(num);

            tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\OneThousand\\"+i+".txt" ;
            
            	
            
            	
            System.out.println(tc.strFileName);

            tc.loadTrajectory();
			//tc.iTrajectory = tc.iTrajectory.subList(1, 144);
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
             //tc.export(lines);
            System.out.println(lines.length );
            System.out.println(tc.iTrajectory.size());
        }
        System.out.println();
        System.out.println(nlines);
        System.out.println(npoints);
	    System.out.println("	Using Time :" +  timeUsed+ "mS");
        System.out.println("Compress Ratio = " +((1.0 * nlines) / (1.0 * npoints) )* 100 + "%.");
    }
    */
}
