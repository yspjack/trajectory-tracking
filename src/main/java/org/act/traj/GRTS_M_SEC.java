package org.act.traj;

import org.act.util.*;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Lin Douglas-Peucker, TopDown
 *
 */
public class GRTS_M_SEC extends TrajectoryCompressor {

    int count = 0;
    private static int sumPoint = 0;
    private static int sumVec = 0;

    public GRTS_M_SEC(DisCalculator dc) {
        super();
        this.setDc(dc);
    }

    public GPSLine[] compress() {
        return this.compressByGRTS_M_SEC(this.iTrajectory, 0, this.iTrajectory.size() - 1);
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


    private GPSLine[] compressByGRTS_M_SEC(List<GPSPoint> iTraj, int iStart, int iEnd) {


        // GPSPoint lineStart = iTraj.get(iStart);
        // GPSPoint lineEnd = iTraj.get(iEnd);
        GPSLine[] lines = new GPSLine[iEnd];

        ArrayList<GPSLine> BufferU = new ArrayList<>();
        // ArrayList<GPSLine> BufferV=new ArrayList<GPSLine>();
        GPSLine lineV = new GPSLine();
        int j = 0;
        int k = 0;
        double payLoadSum = 0;
        int Sstart = 0;
        double Bufferm = getBuffer();
        double Threshold = getThreshold();
        GPSPoint loc = new GPSPoint();
        sumPoint = 0;
        sumVec = 0;


        for (int i = iStart + 1; i <= iEnd; i++) {

            for (int n = Sstart + 1; n < i; n++) {
                // System.out.println("这是到达");
                double time = (double) (iTraj.get(n).time - iTraj.get(Sstart).time) / (iTraj.get(i).time - iTraj.get(Sstart).time);
                GPSPoint temp = DisCalculator.getEndPoint(iTraj.get(Sstart), iTraj.get(i), time);
                double disOfSi = DisCalculator.getDistanceOfP2P(iTraj.get(n), temp);
                if ((disOfSi > Threshold) || (i - Sstart == Bufferm)) {
//                	System.out.println("这是到达");
//                	System.out.println(i);
                    GPSLine line = new GPSLine();
                    line.setStartPoint((GPSPoint) iTraj.get(Sstart));
                    line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                    BufferU.add(line);
                    Sstart = i - 1;
                    break;
                }
            }
            double time1 = (double) (iTraj.get(i).time - iTraj.get(j).time) / (iTraj.get(j + 1).time - iTraj.get(j).time);
            loc = DisCalculator.getEndPoint(iTraj.get(j), iTraj.get(j + 1), time1);
            double disOfP2P = DisCalculator.getDistanceOfP2P(iTraj.get(i), loc);

            if (disOfP2P > Threshold) {

                GPSLine line = new GPSLine();
                line.setStartPoint((GPSPoint) iTraj.get(Sstart));
                line.setEndPoint((GPSPoint) iTraj.get(i - 1));
                BufferU.add(line);

                if (BufferU.size() > 1) {
                    for (int n = 0; n < BufferU.size() - 1; n++) {
                        lines[k++] = BufferU.get(n);
                    }
                }
                payLoadSum += (BufferU.size() - 1) * 24 + 40;
                sumPoint += BufferU.size() - 1;
                sumPoint++;
                sumVec++;

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
        payLoadSum += 24;
        sumPoint++;
        // System.out.println("这次传输信息量为："+payLoadSum);

        GPSLine[] liness = new GPSLine[k];
        for (int i = 0; i < k; i++) {
            liness[i] = lines[i];
        }

        return liness;

    }

    @Override
    public String toString() {
        return "GRTS_M_SEC" + getDc();
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
        for (int zeta = 10; zeta <= 100; zeta += 10) { // �����ֵ
            npoints = 0;
            nlines = 0;
            ave = 0;
            nsumPoint = 0;
            nsumVec = 0;
            nsumload = 0;
            timeUsed = 0;
            if (zeta >= 50 && zeta <= 110) zeta += 10;
            if (zeta >= 130 && zeta <= 170) zeta += 30;


            TrajectoryCompressor tc = new GRTS_M_SEC(sed);
            tc.setBuffer(500);
            tc.setThreshold(zeta); // ��ֵ����λΪ�ס�
            tc.radialThreshold = 1.0 * zeta;
            // tc.deviation = 13; // �жϵ�ͷ����ֵ����λΪ�ס�

            System.out.println("Threshold : " + tc.getThreshold() + " m.");
            // System.out.println("radialThreshold : " + tc.radialThreshold + " m.");
            GPSLine[] lines = null;

//			File file=new File("D:\\lab\\work\\wen\\data\\traj\\mopsi\\CleanData");//path为文件夹的路径
//			String [] fileName = file.list();
//			int arraysize=fileName.length;


//			ArrayList<String> list1 =new ArrayList<>();
//	        String pathname = "D:\\lab\\work\\wen\\data\\traj\\mopsi\\join\\list.txt"; // 绝对路径或相对路径都可以，这里是绝对路径，写入文件时演示相对路径
//	        //String pathname = "D:\\lab\\work\\wen\\data\\traj\\geolife\\CleanData\\list.txt";
//	        File filename = new File(pathname); // 要读取以上路径的input。txt文件
//	        InputStreamReader reader = new InputStreamReader(new FileInputStream(filename));
//	        BufferedReader br = new BufferedReader(reader);
//
//	        String line = br.readLine();
//	        while (line != null) {
//	            line = br.readLine(); // 一次读入一行数据
//	            list1.add(line);
//	            //System.out.println(line);
//	        }
//			int listsize=list1.size();


            for (int i = 100; i <= 300; i++) {

                // Format f1 = new DecimalFormat("000");

                // tc.strFileName = ".\\data\\test.txt";//�켣�ļ�
                // tc.strFileName = ".\\data\\geolife\\0" + i + ".txt";// �켣�ļ�
                // tc.strFileName = ".\\data\\taxi\\" + i + ".txt";//�켣�ļ�
                // tc.strFileName = ".\\data\\truck\\" + i + ".txt";//�켣�ļ�
                // tc.strFileName = "traj/taxi/new/" + i + ".txt";// �켣�ļ�
                // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\OneThousand\\"+ i + ".txt";
//				tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\mopsi\\join\\"+ list1.get(i);
                // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\mopsi\\join\\"+ list1.get(i);
                // System.out.println(list1.get(i));
                // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\OneThousand\\"+i+".txt" ;
                // tc.strFileName = "D:\\lab\\test\\result\\mopsi112\\mopsi.txt" ;
                // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\CleanData\\"+ i + ".txt";
                tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\SerCar\\CleanData\\" + i + ".txt";
                // tc.strFileName = "D:\\lab\\work\\wen\\data\\traj\\geolife\\CleanData\\"+ i + ".txt";

                // tc.strFileName = "traj/truck/short/" + i + ".txt";// �켣�ļ�

                // tc.strFileName = "traj/geolife/new/increase2.txt";
                // String fileName = "traj/geolife/new/increase.txt";
                // tc.strFileName = "traj/geolife/new/" + i + ".txt";// �켣�ļ�
                // tc.strFileName = "d:/ACT/trajData/SerCar/CleanData/" + i + ".txt";
                // tc.strFileName = "d:/ACT/trajData/truck/CleanData/" + i + ".txt";
                // tc.strFileName = "d:/ACT/trajData/zh/CleanData/clean/" + i + ".txt";
                // tc.strFileName = "traj/taxi/new/short.txt";// �켣�ļ�

                tc.loadTrajectory();
                // System.out.print("" + i);
                // System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress(); // �㷨��n
                double etime = System.currentTimeMillis();
                nsumPoint += sumPoint;
                nsumVec += sumVec;

                // System.out.println("sumPoint = " + sumPoint +" sumVec= "+sumVec);
                // System.out.println("Time:" + (etime - stime));
                timeUsed += (etime - stime);
                // System.out.println("Compress By Sleeve to : (" + lines.length + ") lines.");
                tc.printToConsoleSED(tc.iTrajectory, lines);

                nlines = nlines + lines.length;

                ave += tc.getAveErrorSED(tc.iTrajectory, lines);
                // tc.export(lines);

            }

            time += timeUsed + ",";
            CR += (1.0 * nlines) / (1.0 * npoints) * 100 + ",";
            CISEDpoints += nsumPoint + ",";
            CISEDvecs += nsumVec + ",";
            CISEDtrans += (nsumPoint * 24 + nsumVec * 16) + ",";
            CISEDave += (1.0 * ave) / (1.0 * npoints) + ",";


            System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
            System.out.println("nlines = " + nlines);
            System.out.println("Time Used = " + timeUsed + "ms");
            System.out.println("AveError = " + (1.0 * ave) / (1.0 * npoints) + "m.");
            System.out.println("nsumPoint = " + nsumPoint);
            System.out.println("nsumVec= " + nsumVec);
            System.out.println("nsumload = " + (nsumPoint * 24 + nsumVec * 16));
            System.out.println();
            System.out.println(time);
            System.out.println(CR);
            System.out.println(CISEDpoints);
            System.out.println(CISEDvecs);
            System.out.println(CISEDtrans);
            System.out.println(CISEDave);
            System.out.println("SerCar");
        }
    }


}
