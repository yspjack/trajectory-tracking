package org.act.traj;

import org.act.util.*;
import org.act.util.polygon.Point;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


@SuppressWarnings("Javadoc")
public abstract class TrajectoryCompressor {
    protected final static double EARTH_RADIUS = 6378137; // 单位为米
    protected double threshold = 20; // 阈值，单位为米。
    protected double Buffer;  // CDR_B
    protected double radialThreshold = 30;
    private DisCalculator dc; // DisCalculator

    public double SEDerr, PEDerr;

    protected List<GPSPoint> iTrajectory; // 车辆原始轨迹（离散点）

    public String strFileName = ".\\data\\Taxi\\32.txt";// 轨迹文件

    public abstract GPSLine[] compress(); // 压缩算法主函数

    static int index = 0;// 计数器，轨迹点的序号

    // 加载轨迹点，按时间排序
    public void loadTrajectory() {
        var file = new File(strFileName);
        if (!file.isFile() || !file.exists()) {
            System.out.println(strFileName + "bad input file.");
            return;
        } else {
        }


        var isGeoFile = 0;
        if (strFileName.endsWith(".plt")) isGeoFile = 1;
        try {
            var read = new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8);
            var reader = new BufferedReader(read);
            String line;

            // 循环，每次读一行，解析后存入vector
            iTrajectory = new ArrayList<>();
            while ((line = reader.readLine()) != null) {
                if (isGeoFile == 1) {
                    parseGeolife(line);
                } else {
                    parseLocation(line);
                }
            }

            reader.close();
            read.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    /*
     * Structure: devicesn,时间戳,经度，纬度，速度，方向 Sample:
     * "967790028725";1440691884;39.8222766666667;116.422643333333;48.59;290
     */
    void parseLocation(String str) {
        GPSPoint loc = null;

        var strs = str.split(";");
        if (strs == null) return;
        if (strs.length >= 3) {
            loc = new GPSPoint();

            loc.time = Long.parseLong(strs[1]);
            loc.latitude = Double.parseDouble(strs[2]);
            loc.longitude = Double.parseDouble(strs[3]);
            // System.out.println("new BMap.Point("+loc.longitude+","+loc.latitude+"),");
            if (loc.latitude < 0 || loc.latitude > 90 || loc.longitude < 0 || loc.latitude > 180) {
                // System.out.println(" Special Point : " + loc.latitude + "," +
                // loc.longitude);
            }
            loc.index = index++;
            this.iTrajectory.add(loc);
        }
    }

    /*
     * Structure: 纬度， 经度，?, 海拔，天数，日期,时间 Sample:
     * 39.994622,116.326757,0,492,39748.496400463,2008-10-27,11:54:49
     */
    static long tcounter = 0;

    void parseGeolife(String str) {

        GPSPoint loc = null;

        if (str.length() < 10) return;
        var strs = str.split(",");
        if (strs == null) return;
        if (strs.length >= 3) {
            loc = new GPSPoint();

            loc.time = ++tcounter;
            loc.latitude = Double.parseDouble(strs[0]);
            loc.longitude = Double.parseDouble(strs[1]);
            loc.index = index++;
            this.iTrajectory.add(loc);
        }
    }

    public static double rad(double d) {
        return 1.0 * d * Math.PI / 180.0;
    }

    /*
     * 计算两个点之间的距离，绝对值 1. Lat1 Lung1 表示A点经纬度，Lat2 Lung2 表示B点经纬度； 2. a=Lat1 – Lat2
     * 为两点纬度之差 b=Lung1 -Lung2 为两点经度之差； 3. 6378.137为地球半径，单位为千米； 计算出来的结果单位为千米。
     */
    public static double getDistanceOfP2P(double lat1, double lng1, double lat2, double lng2) {

        var radLat1 = rad(lat1);
        var radLat2 = rad(lat2);

        var dw = radLat1 - radLat2; // 纬度差，弧度
        var dj = rad(lng1) - rad(lng2); // 经度差，弧度

        // 两个点相对于球心的夹角
        var s = 2 * Math.asin(Math.sqrt(Math.pow(Math.sin(dw / 2.0), 2.0) + Math.cos(radLat1) * Math.cos(radLat2) * Math.pow(Math.sin(dj / 2.0), 2.0)));

        // 距离
        s = s * EARTH_RADIUS;
        // s = Math.round(s * 10000) / 10000;

        return Math.abs(s);
    }

    /////////////////////////////////////////////////////////////////////////////////
    // 以下代码都应该转化为球面坐标下相应计算
    // 直线 -> 相对球心的弧线
    // 直线的角度 -> 弧线所在大圆的角度
    // 直线的夹角 -> 弧线所在大圆的夹角
    // 点到直线的距离 -> 点到弧线的距离
    //////////////////////////////////////////////////////////////////////////////////

    /**
     * 返回 从矢量1 到 矢量2 之间的 相对夹角，介于 (-Pi, Pi)。逆时针 为 正； 顺 时针 为 负
     *
     * @param angle1：矢量1的角度,
     *            矢量1为出发矢量
     * @param angle2：矢量2的角度
     * @return [-Pi, Pi]
     */
    static double getIncludedAngle(double angle1, double angle2) {
        var theta = angle2 - angle1;
        if (theta > Math.PI) theta = -2 * Math.PI + theta;
        if (theta < -Math.PI) theta = 2 * Math.PI + theta;
        return theta;
    }

    /*
     * 计算点到直线的最短距离
     */
    static double getDistance(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {

        double dis = 0;

        var r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
        if (r == 0) return 0;

        var lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        var sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

        dis = r * Math.sin(sita - lineAngle);

        return dis;
        // test by yihaofu
		/*
		double d=0;
		double lels=getDistanceOfP2P(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
		double lls=getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
		double lle=getDistanceOfP2P(lineEnd.latitude, lineEnd.longitude, loc.latitude, loc.longitude);
		double p=lels+lls+lle;
		p=p/2;
		double m=p*(p-lls)
				*(p-lels)
				*(p-lle);
		if (lels*lls==0)
		{
			d=lls;
			
			return d;
		}
		double cosa=(lls*lls)+(lels*lels)-(lle*lle);
		if (cosa<=0)
		{
			d=lls;
			
			return d;
		}
		if (lels*lle==0)
		{
			d=lle;
			
			return d;
		}
		double cosb=(lle*lle)+(lels*lels)-(lls*lls);
		if (cosb<=0)
		{
			d=lle;
			
			return d;
		}
		if (m<=0)
		{
			d=0;
		} else {
			
			double dis = 0;

			double r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
			if (r == 0)
				return 0;

			double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
					lineEnd.longitude);
			double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

			dis = r * Math.sin(sita - lineAngle);
			return dis;
			
		}
		return d;*/
        //=============
		/*  !!!!!!!!!! CITT  BITT SITT by yihaofu
		double lsle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
				lineEnd.longitude);
		double lels = getAngleOfVector(lineEnd.latitude, lineEnd.longitude, lineStart.latitude,
				lineStart.longitude);
		double lsl = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude,
				loc.longitude);
		double lel = getAngleOfVector(lineEnd.latitude, lineEnd.longitude, loc.latitude,
				loc.longitude);
		double dis;
		if (Math.abs(lsle-lsl)>=Math.PI/2 && Math.abs(lsle-lsl)<=Math.PI*3/2)
		{
			dis=getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
			return dis;
		}
		if (lels==lel)
		{
			dis=0;
			return dis;
		}
		if (Math.abs(lels-lel)>Math.PI/2 && Math.abs(lels-lel)<Math.PI*3/2)
		{
			dis=getDistanceOfP2P(lineEnd.latitude, lineEnd.longitude, loc.latitude, loc.longitude);
			if (dis>30) {
				//System.out.println(lineStart.latitude+" "+lineStart.longitude+" "+lineEnd.latitude+" "+lineEnd.longitude+" "+loc.latitude+" "+loc.longitude);
				//System.out.println("???   "+lels+ "   "+lel+" "+dis+" "+getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude));
			}
			return dis;
		}
		
		double r = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);
		if (r == 0)
			return 0;

		double lineAngle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude,
				lineEnd.longitude);
		double sita = getAngleOfVector(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude);

		dis = r * Math.sin(sita - lineAngle);
		return dis;*/
    }

    /*
     * 计算点到向量的最短距离，有正负值
     */
    static double getDistanceOfP2R(GPSLine line, GPSPoint loc) {
        double dis = 0;

        // 1.计算loc到line原点的距离。绝对值。
        var r = getDistanceOfP2P(line.startLatitude(), line.startLongitude(), loc.latitude, loc.longitude);
        if (r == 0) return 0;

        // 2.计算line原点与loc构成直线的角度[0,2pi)
        var sita = getAngleOfVector(line.startLatitude(), line.startLongitude(), loc.latitude, loc.longitude);

        // 3.计算距离
        dis = r * Math.sin(sita - line.angle);

        return dis;
    }

    ///////////////////////////////////////////////////////////////////////////////
    //////////// 以下所有和角度有关的代码都是近似计算,把曲面坐标投影到了平面坐标/////////////
    //////////// @todo...这些代码都应该彻底改造成球面坐标///////////////////////////////

    /**
     * @给定矢量的起点和端点（经纬度），该代码返回矢量相对x坐标的夹角。
     *
     *                                    注意，这个角度计算法是特异的，并没有完全按球面坐标计算，而是直接投影到了平面坐标
     *                                   以后也需要按照这种方法映射回去（比如已知起点，距离和角度，求端点的时候）。
     *
     *                                                                @param latS
     * @param lngS
     * @param latE
     * @param lngE
     * @return [0, 2pi)
     */
    public static double getAngleOfVector(double latS, double lngS, double latE, double lngE) {

        if (latS == latE && lngS == lngE) return 0;
        else if (latS == latE && lngS < lngE) return 0;
        else if (latS == latE && lngS > lngE) return Math.PI;
        else if (latS < latE && lngS == lngE) return Math.PI / 2;
        else if (latS > latE && lngS == lngE) return 3.0 * Math.PI / 2;

        var high = getDistanceOfP2P(latE, lngE, latS, lngE);
        var length = getDistanceOfP2P(latS, lngS, latE, lngE);
        if (length == 0) return 0;

        var angle = Math.asin(high / length);// 返回值[-pi/2, pi/2]
        // System.out.println("Debug: angle = " + angle );
        // 修正角度值
        // if (latS<=latE && lngS<=lngE) angle =angle; //第1区间
        if (latS < latE && lngS > lngE) angle = Math.PI - angle; // 第2区间
        if (latS > latE && lngS > lngE) angle = Math.PI + angle; // 第3区间
        if (latS > latE && lngS < lngE) angle = 2 * Math.PI - angle; // 第4区间

        // System.out.println("Debug: get angle = " + angle );
        return angle;
    }

    public void printToConsole(List<GPSPoint> trajectory, GPSLine[] lines) {
        // System.out.println("---Compress By My Algorithm to : " + lines.length
        // + " lines");
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            //

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }

            }

            if (n == 8) {
                // System.out.println();
            }
//			if(n >= 1330 && n <= 1339) {
//				System.out.println(n);
//			}
            // double dis = getDistanceOfP2R(lines[l_index], loc);


            if (this.getDc() instanceof DAD) {
                // System.out.println("DAD");
                if (n == trajectory.size() - 1) {
                    break;
                }
                var suc = (GPSPoint) trajectory.get(n + 1);
                var dad = ((DAD) getDc()).getDistanceOfP2Line(lines[l_index], loc, suc);

                if (threshold < Math.abs(dad)) {
                    System.out.println("Point " + n + " breaks up dad : " + dad); // For
                    System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);
                    System.out.println("lineId: " + l_index);
                    errorPoints++;
                }
            }
            if (this.getDc() instanceof DED) {
                // System.out.println("DED");
                var radialDis = ((DED) getDc()).getRadialDis(lines[l_index], loc);
                var perDis = ((DED) getDc()).getPerpendicularDis(lines[l_index], loc);
                if (getThreshold() < Math.abs(perDis)) {
                    System.out.println("Point " + n + " breaks up bound : " + perDis); // For
                    System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);// Debug
                    System.out.println("lineId: " + l_index);
                    errorPoints++;
                }
                if (radialThreshold < Math.abs(radialDis)) {
                    System.out.println("Point " + n + " breaks up radialBound : " + radialDis); // For
                    System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);
                    System.out.println("lineId: " + l_index);
                    errorPoints++;
                }
            } else {
                // System.out.println("else");
                var dis = this.getDc().getDistanceOfP2Line(lines[l_index].startPoint, lines[l_index].endPoint, loc);
//				double dis = this.dc.getDistanceOfP2Line(lines[l_index], loc);
                de = de + dis;
                ae = ae + Math.abs(dis);
                // System.out.println("hahahha");

                if (me < Math.abs(dis)) {
                    me = Math.abs(dis);
                    me_pindex = n;
                    me_lindex = l_index;
                }

                if (getThreshold() < Math.abs(dis)) {
                    // if(lines[l_index].startPoint.index >= lines[l_index].endPoint.index -2);
                    // System.out.println("Point " + n + " breaks up bound : " + dis); // For
                    // System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point "
                    //			+ lines[l_index].getEndPoint().index); // Debug
                    //	System.out.println("lineId: " + l_index);

                    errorPoints++;
                }

            }


        }

        // System.out.println(" Average Absolute Error : " +
        // ae/trajectory.size());
        // System.out.println(" Average Error : " + de/trajectory.size());
        System.out.println("    Max Error : " + me + "; Point No." + me_pindex);
        System.out.println("ErrorPoints: " + errorPoints);
        // System.out.print(" Isvalid : " + trajectory.get(me_pindex).isValid);
        // System.out.print(" Distance to R' : " +
        // trajectory.get(me_pindex).dis);
        // System.out.println(" Number: "+ numberOfBigErrorPoints + " Ratio : "
        // + 1.0* numberOfBigErrorPoints /trajectory.size() );

    }

    public double printToConsoledad(List<GPSPoint> trajectory, GPSLine[] lines) {
        // System.out.println("---Compress By My Algorithm to : " + lines.length
        //+ " lines");

        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }

            }


            if (n == trajectory.size() - 1) {
                break;
            }
            var suc = (GPSPoint) trajectory.get(n + 1);
            // double dad = ((DAD) getDc()).getDistanceOfP2Line(lines[l_index], loc,suc);
            var dadd = new DAD();

            var dad = dadd.getDistanceOfP2Line(lines[l_index], loc, suc);
            if (threshold < Math.abs(dad)) {
					/*System.out.println("Point " + n + " breaks up dad : " + dad); // For
					System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point "
							+ lines[l_index].getEndPoint().index);
					System.out.println("lineId: " + l_index);*/
                errorPoints++;
            }
            if (dad > me) {
                me = dad;
                me_pindex = n;
            }


        }

        // System.out.println(" Average Absolute Error : " +
        // ae/trajectory.size());
        // System.out.println(" Average Error : " + de/trajectory.size());
        //!!!!!!!!!!
        // System.out.println("    Max Error : " + me + "; Point No." + me_pindex);
        //!!!!!!!!!!
        // System.out.println("ErrorPoints: " + errorPoints);
        // System.out.print(" Isvalid : " + trajectory.get(me_pindex).isValid);
        // System.out.print(" Distance to R' : " +
        // trajectory.get(me_pindex).dis);
        // System.out.println(" Number: "+ numberOfBigErrorPoints + " Ratio : "
        // + 1.0* numberOfBigErrorPoints /trajectory.size() );
        return me;
    }

    public void printToConsoleB(List<GPSPoint> trajectory, GPSLine[] lines) {
        // System.out.println("---Compress By My Algorithm to : " + lines.length
        // + " lines");
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

//			if (n == 4) {
//				System.out.println(n);
//			}
            //
            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }

            }

            // double dis = getDistanceOfP2R(lines[l_index], loc);

            var pe = new Point(loc, lines[l_index].getStartPoint());
            var ps = this.getDc().predict(lines[l_index].getStartPoint(), lines[l_index].getEndPoint(), loc);
            var psxy = new Point(ps, lines[l_index].getStartPoint());
            var dx = pe.x - psxy.x;
            var dy = pe.y - psxy.y;

            var dis = Math.sqrt(dx * dx + dy * dy);
            de = de + dis;
            ae = ae + Math.abs(dis);

            if (me < Math.abs(dis)) {
                me = Math.abs(dis);
                me_pindex = n;
                me_lindex = l_index;
            }

            if (getThreshold() < Math.abs(dis)) {
                // if(lines[l_index].startPoint.index >= lines[l_index].endPoint.index -2);
                System.out.println("Point " + n + " breaks up bound : " + dis); // For
                System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index); // Debug
                System.out.println("lineId: " + l_index);

                errorPoints++;
            }

        }

        // System.out.println(" Average Absolute Error : " +
        // ae/trajectory.size());
        // System.out.println(" Average Error : " + de/trajectory.size());
        System.out.println("    Max Error : " + me + "; Point No." + me_pindex);
        System.out.println("ErrorPoints: " + errorPoints);
        // System.out.print(" Isvalid : " + trajectory.get(me_pindex).isValid);
        // System.out.print(" Distance to R' : " +
        // trajectory.get(me_pindex).dis);
        // System.out.println(" Number: "+ numberOfBigErrorPoints + " Ratio : "
        // + 1.0* numberOfBigErrorPoints /trajectory.size() );

    }

    public int getErrorPoints(List<GPSPoint> trajectory, GPSLine[] lines) {
        // System.out.println("---Compress By My Algorithm to : " + lines.length
        // + " lines");

        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            //
            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }

            }

            // double dis = getDistanceOfP2R(lines[l_index], loc);

            if (this.getDc() instanceof DED) {
                var radialDis = ((DED) getDc()).getRadialDis(lines[l_index], loc);
                var perDis = ((DED) getDc()).getPerpendicularDis(lines[l_index], loc);
                if (getThreshold() < Math.abs(perDis)) {

                    errorPoints++;
                }
                if (radialThreshold < Math.abs(radialDis)) {

                    errorPoints++;
                }
            } else {
                var dis = this.getDc().getDistanceOfP2Line(lines[l_index], loc);
                de = de + dis;
                ae = ae + Math.abs(dis);

                if (me < Math.abs(dis)) {
                    me = Math.abs(dis);
                    me_pindex = n;
                    me_lindex = l_index;
                }

                if (getThreshold() < Math.abs(dis)) {
                    errorPoints++;
                }

            }

        }

        return errorPoints;
    }

    public int getError(List<GPSPoint> trajectory, GPSLine[] lines, double[] aboutError) {

        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //
        var exceed1Percent = 0;
        var exceed5pPrcent = 0;
        var exceed10Percent = 0;
        var exceed20Percent = 0;
        var exceed50Percent = 0;
        var exceed100Percent = 0;

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }
            }

            if (this.getDc() instanceof DAD) {
                if (n == trajectory.size() - 1) {
                    break;
                }
                var suc = (GPSPoint) trajectory.get(n + 1);
                var dad = ((DAD) getDc()).getDistanceOfP2Line(lines[l_index], loc, suc);

                if (radialThreshold < Math.abs(dad)) {
                    System.out.println("Point " + n + " breaks up dad : " + dad); // For
                    System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);
                    System.out.println("lineId: " + l_index);
                    errorPoints++;
                }
            } else if (this.getDc() instanceof DED) {
                var radialDis = ((DED) getDc()).getRadialDis(lines[l_index], loc);
                var perDis = ((DED) getDc()).getPerpendicularDis(lines[l_index], loc);
                if (getThreshold() < Math.abs(perDis)) {

                    errorPoints++;
                }
                if (radialThreshold < Math.abs(radialDis)) {

                    errorPoints++;
                }
            } else {
                var dis = this.getDc().getDistanceOfP2Line(lines[l_index], loc);
                de = de + dis;
                ae = ae + Math.abs(dis);

                if (me < Math.abs(dis)) {
                    me = Math.abs(dis);
                    me_pindex = n;
                    me_lindex = l_index;
                }

                if (getThreshold() < Math.abs(dis)) {
                    errorPoints++;
                }

            }

        }
        aboutError[0] = ae / iTrajectory.size();
        aboutError[1] = errorPoints;
        aboutError[2] = me;
        return errorPoints;
    }


    public int getdadError(List<GPSPoint> trajectory, GPSLine[] lines, double[] aboutError) {

        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //
        var exceed1Percent = 0;
        var exceed5pPrcent = 0;
        var exceed10Percent = 0;
        var exceed20Percent = 0;
        var exceed50Percent = 0;
        var exceed100Percent = 0;

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }
            }


            if (n == trajectory.size() - 1) {
                break;
            }
            var suc = (GPSPoint) trajectory.get(n + 1);
            var dad = ((DAD) getDc()).getDistanceOfP2Line(lines[l_index], loc, suc);

            if (radialThreshold < Math.abs(dad)) {
                System.out.println("Point " + n + " breaks up dad : " + dad); // For
                System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);
                System.out.println("lineId: " + l_index);
                errorPoints++;
            }
            de = de + dad;
            ae = ae + Math.abs(dad);


        }
        aboutError[0] = ae / iTrajectory.size();
        aboutError[1] = errorPoints;
        aboutError[2] = me;
        return errorPoints;
    }

    public double getdadError(List<GPSPoint> trajectory, GPSLine[] lines) {

        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //
        var exceed1Percent = 0;
        var exceed5pPrcent = 0;
        var exceed10Percent = 0;
        var exceed20Percent = 0;
        var exceed50Percent = 0;
        var exceed100Percent = 0;

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }
            }


            if (n == trajectory.size() - 1) {
                break;
            }
            var suc = (GPSPoint) trajectory.get(n + 1);
            var dad = ((DAD) getDc()).getDistanceOfP2Line(lines[l_index], loc, suc);

            if (radialThreshold < Math.abs(dad)) {
                System.out.println("Point " + n + " breaks up dad : " + dad); // For
                System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);
                System.out.println("lineId: " + l_index);
                errorPoints++;
            }
            de = de + dad;
            ae = ae + Math.abs(dad);


        }

        return ae;
    }


    public Map<String, Number> getError(GPSLine[] lines) {
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //
        var exceed1Percent = 0;
        var exceed5pPrcent = 0;
        var exceed10Percent = 0;
        var exceed20Percent = 0;
        var exceed50Percent = 0;
        var exceed100Percent = 0;
        var less1Percent = 0;

        var errorPoints = 0;
        for (var n = 0; n < iTrajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) iTrajectory.get(n);
            if (l_index < lines.length - 1) {
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }
            }

            if (this.getDc() instanceof DAD) {
                if (n == iTrajectory.size() - 1) {
                    break;
                }
                var suc = (GPSPoint) iTrajectory.get(n + 1);
                var dis = ((DAD) getDc()).getDistanceOfP2Line(lines[l_index], loc, suc);

                de = de + dis;
                ae = ae + Math.abs(dis);
                dis = Math.abs(dis);

                if (me < Math.abs(dis)) {
                    me = Math.abs(dis);
                    me_pindex = n;
                    me_lindex = l_index;
                }
                if (dis >= threshold * 2) {
                    exceed100Percent++;
                } else if (dis >= threshold * 1.5) {
                    exceed50Percent++;
                } else if (dis >= threshold * 1.2) {
                    exceed20Percent++;
                } else if (dis >= threshold * 1.1) {
                    exceed10Percent++;
                } else if (dis >= threshold * 1.05) {
                    exceed5pPrcent++;
                } else if (dis >= threshold * 1.01) {
                    exceed1Percent++;
                } else if (dis >= threshold) {
                    less1Percent++;
                }

                if (getThreshold() < Math.abs(dis)) {
                    errorPoints++;
                }
            } else if (this.getDc() instanceof DED) {
                var radialDis = ((DED) getDc()).getRadialDis(lines[l_index], loc);
                var perDis = ((DED) getDc()).getPerpendicularDis(lines[l_index], loc);
                if (getThreshold() < Math.abs(perDis)) {

                    errorPoints++;
                }
                if (radialThreshold < Math.abs(radialDis)) {

                    errorPoints++;
                }
            } else {
                var dis = this.getDc().getDistanceOfP2Line(lines[l_index].startPoint, lines[l_index].endPoint, loc);
                de = de + dis;
                ae = ae + Math.abs(dis);
                dis = Math.abs(dis);

                if (me < Math.abs(dis)) {
                    me = Math.abs(dis);
                    me_pindex = n;
                    me_lindex = l_index;
                }

                if (dis >= threshold * 2) {
                    exceed100Percent++;
                } else if (dis >= threshold * 1.5) {
                    exceed50Percent++;
                } else if (dis >= threshold * 1.2) {
                    exceed20Percent++;
                } else if (dis >= threshold * 1.1) {
                    exceed10Percent++;
                } else if (dis >= threshold * 1.05) {
                    exceed5pPrcent++;
                } else if (dis >= threshold * 1.01) {
                    exceed1Percent++;
                } else if (dis >= threshold) {
                    less1Percent++;
                }

                if (getThreshold() < Math.abs(dis)) {
                    errorPoints++;
                }

            }

        }
        Map<String, Number> res = new HashMap<>();
        res.put("errorPoints", errorPoints);
        res.put("menError", ae / iTrajectory.size());
        res.put("maxError", me);
        res.put("exceed1Percent", exceed1Percent);
        res.put("exceed5Percent", exceed5pPrcent);
        res.put("exceed10Percent", exceed10Percent);
        res.put("exceed20Percent", exceed20Percent);
        res.put("exceed50Percent", exceed50Percent);
        res.put("exceed100Percent", exceed100Percent);
        res.put("less1Percent", less1Percent);

        return res;
    }

    public double getTime(List<GPSPoint> trajectory) {


        var n = iTrajectory.size();
        var lineStart = iTrajectory.get(0);
        var lineEnd = iTrajectory.get(n - 1);

        // DisCalculator sed = new SED();
        // DisCalculator ped = new PED();
        var dadCalc = (DAD) getDc();

        double stime = System.currentTimeMillis();
        var alpha = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        for (var j = 1; j <= 100; j++) {

            for (var i = 1; i < n - 1; i++) {
                // ����ֹ������֮��������ĵ�
                var p = iTrajectory.get(i);
                var loc = iTrajectory.get(i + 1);
//				double distance = Math.abs(getDistance(startPoint, endPoint, p));
                var distance = dadCalc.getDistanceOfP2Line(alpha, p, loc);

            }
        }
        double etime = System.currentTimeMillis();
        var TimeUsed = etime - stime;
        return TimeUsed;

    }

    // 求SED距离
    public double getAveErrorSED(List<GPSPoint> trajectory, GPSLine[] lines) {
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.time >= lines[l_index + 1].startPoint.time) {
                    l_index++;
                }

            }
            var lineStart = lines[l_index].getStartPoint();
            var lineEnd = lines[l_index].getEndPoint();
            var tmp = DisCalculator.predict(lineStart, lineEnd, loc); // 计算同步点
            var dis = (getDistanceOfP2P(tmp.latitude, tmp.longitude, loc.latitude, loc.longitude));


            ae = ae + Math.abs(dis);


            if (getThreshold() < Math.abs(dis)) {

                errorPoints++;
            }

        }
        return ae;

    }

    // 求PED距离
    public double getAveErrorPED(List<GPSPoint> trajectory, GPSLine[] lines) {
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.time >= lines[l_index + 1].startPoint.time) {
                    l_index++;
                }

            }
            var lineStart = lines[l_index].getStartPoint();
            var lineEnd = lines[l_index].getEndPoint();

            var dis = getDistance(lineStart, lineEnd, loc);


            ae += Math.abs(dis);


            if (getThreshold() < Math.abs(dis)) {

                // errorPoints++;
                // System.out.println("error"+n);
            }

        }
        return ae;

    }

    public double limitVal(double val, double lb, double ub) {
        if (val < lb) val = lb;
        if (val > ub) val = ub;
        return val;
    }

    //		public double getMaxErrorTime(List<GPSPoint> trajectory, GPSLine[] lines) {
//			int l_index = 0; // index to lines
//			double ae = 0;
//			double de = 0; // 累计算术误差
//			double me = 0; // 累计绝对误差
//			int me_pindex = 0; // index of the max err point
//			int me_lindex = 0; //
//
//			int errorPoints = 0;
//			
//			
//			double MERCATOR_LATITUDE_LB = 2.5*2.0-Math.PI/2;
//	        double MERCATOR_LATITUDE_UB = 87.5*2.0-Math.PI/2;
//			
//	        double earthRadius = 6378100.0;
//	        double rx, ry, sf=0;// Longitude/latitude in radian.
//	        double SCALE_FACTOR_PRECISION = 1e-4;
//	        for(int i=0; i<=trajectory.size()-1; ++i)
//	        {
//	        	GPSPoint p = (GPSPoint) iTrajectory.get(i);
//	        	sf += 1.0/ Math.cos( Math.toRadians(p.latitude) );
//	        }
//	        sf /= trajectory.size();
//	        sf = Math.round(sf/SCALE_FACTOR_PRECISION)*SCALE_FACTOR_PRECISION;
//	        double x,y;
//	        Map<Double, Double> mapx = new HashMap<Double,Double>();
//	        Map<Double, Double> mapy = new HashMap<Double,Double>();
//	        for(int i=0; i<=trajectory.size()-1; ++i) {
//	        	GPSPoint p = (GPSPoint) iTrajectory.get(i);
//		        rx = Math.toRadians(p.longitude);
//		        ry = Math.toRadians(limitVal(p.latitude, MERCATOR_LATITUDE_LB, MERCATOR_LATITUDE_UB));
//		        ry = Math.log(Math.abs(Math.tan(ry)+1.0/Math.cos(ry)));
//		        
//		        x=rx*earthRadius/sf;
//		        y=ry*earthRadius/sf;
//		        //System.out.println("raw  "+ x+ " "+y );
//		        mapx.put(p.longitude,x);
//		        mapy.put(p.latitude,y);
//	        }
//			
//			
//			for (int n = 0; n < trajectory.size(); n++) { // 轨迹点
//
//				GPSPoint loc = (GPSPoint) trajectory.get(n);
//				if (l_index < lines.length - 1) {
//					// 针对DP等算法实现，没有给Line编号
//					if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
//						l_index++;
//
//						// 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
//					} else if (loc.time >= lines[l_index + 1].startPoint.time) {
//						l_index++;
//					}
//
//				}
//				GPSPoint lineStart = lines[l_index].getStartPoint();
//				GPSPoint lineEnd = lines[l_index].getEndPoint();
//				
//				double x1=mapx.get(lineStart.longitude);
//				double y1=mapy.get(lineStart.latitude);
//				double x2=mapx.get(lineEnd.longitude);
//				double y2=mapy.get(lineEnd.latitude);
//				double px=mapx.get(loc.longitude);
//				double py=mapy.get(loc.latitude);
//				
//				double xx,yy;
//
//				Point retVal;
//
//				double dx = x1-x2;
//				double dy = y1-y2;
//
//			    if(Math.abs(dx) < 0.00000001 && Math.abs(dy) < 0.00000001 )
//			    {
//			    	xx=x1;
//			    	yy=y2;
//			    }
//			    else {
//			    	double u = (px - x1)*( x1 - x2) + (py - y1)*(y1 - y2);
//			    	u = u/((dx*dx)+(dy*dy));
//			    	xx = x1 + u*dx;
//			    	yy = y1 + u*dy;
//			    }
//			    
//			    double totdis=Math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
//			    double dis1=Math.sqrt((x1-xx)*(x1-xx)+(y1-yy)*(y1-yy));
//			    double temp=lineStart.time + 1.0*dis1/totdis*(lineEnd.time-lineStart.time);
//			    if (totdis==0) {
//			    	temp=(double)lineStart.time;
//		    	}
//			    double t1=Math.abs(lineStart.time-temp);
//			    double t2=Math.abs(lineEnd.time-temp);
//			    double t=Math.min(t1, t2);
//			    if (t>ae) ae=t;
//			}
//			return ae;
//		}
//		
//		public double getAveErrorTime(List<GPSPoint> trajectory, GPSLine[] lines) {
//			int l_index = 0; // index to lines
//			double ae = 0;
//			double de = 0; // 累计算术误差
//			double me = 0; // 累计绝对误差
//			int me_pindex = 0; // index of the max err point
//			int me_lindex = 0; //
//
//			int errorPoints = 0;
//			
//			
//			double MERCATOR_LATITUDE_LB = 2.5*2.0-Math.PI/2;
//	        double MERCATOR_LATITUDE_UB = 87.5*2.0-Math.PI/2;
//			
//	        double earthRadius = 6378100.0;
//	        double rx, ry, sf=0;// Longitude/latitude in radian.
//	        double SCALE_FACTOR_PRECISION = 1e-4;
//	        for(int i=0; i<=trajectory.size()-1; ++i)
//	        {
//	        	GPSPoint p = (GPSPoint) iTrajectory.get(i);
//	        	sf += 1.0/ Math.cos( Math.toRadians(p.latitude) );
//	        }
//	        sf /= trajectory.size();
//	        sf = Math.round(sf/SCALE_FACTOR_PRECISION)*SCALE_FACTOR_PRECISION;
//	        double x,y;
//	        Map<Double, Double> mapx = new HashMap<Double,Double>();
//	        Map<Double, Double> mapy = new HashMap<Double,Double>();
//	        for(int i=0; i<=trajectory.size()-1; ++i) {
//	        	GPSPoint p = (GPSPoint) iTrajectory.get(i);
//		        rx = Math.toRadians(p.longitude);
//		        ry = Math.toRadians(limitVal(p.latitude, MERCATOR_LATITUDE_LB, MERCATOR_LATITUDE_UB));
//		        ry = Math.log(Math.abs(Math.tan(ry)+1.0/Math.cos(ry)));
//		        
//		        x=rx*earthRadius/sf;
//		        y=ry*earthRadius/sf;
//		        //System.out.println("raw  "+ x+ " "+y );
//		        mapx.put(p.longitude,x);
//		        mapy.put(p.latitude,y);
//	        }
//			
//			
//			for (int n = 0; n < trajectory.size(); n++) { // 轨迹点
//
//				GPSPoint loc = (GPSPoint) trajectory.get(n);
//				if (l_index < lines.length - 1) {
//					// 针对DP等算法实现，没有给Line编号
//					if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
//						l_index++;
//
//						// 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
//					} else if (loc.time >= lines[l_index + 1].startPoint.time) {
//						l_index++;
//					}
//
//				}
//				GPSPoint lineStart = lines[l_index].getStartPoint();
//				GPSPoint lineEnd = lines[l_index].getEndPoint();
//				
//				double x1=mapx.get(lineStart.longitude);
//				double y1=mapy.get(lineStart.latitude);
//				double x2=mapx.get(lineEnd.longitude);
//				double y2=mapy.get(lineEnd.latitude);
//				double px=mapx.get(loc.longitude);
//				double py=mapy.get(loc.latitude);
//				
//				double xx,yy;
//				
//				double dx = x1-x2;
//				double dy = y1-y2;
//
//			    if(Math.abs(dx) < 0.00000001 && Math.abs(dy) < 0.00000001 )
//			    {
//			    	xx=x1;
//			    	yy=y2;
//			    }
//			    else {
//			    	double u = (px - x1)*( x1 - x2) + (py - y1)*(y1 - y2);
//			    	u = u/((dx*dx)+(dy*dy));
//			    	xx = x1 + u*dx;
//			    	yy = y1 + u*dy;
//			    }
//			    
//			    double totdis=Math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
//			    double dis1=Math.sqrt((x1-xx)*(x1-xx)+(y1-yy)*(y1-yy));
//			    
//			    double temp=(double)lineStart.time + 1.0*dis1/totdis*(lineEnd.time-lineStart.time);
//			    if (totdis==0) {
//			    	temp=(double)lineStart.time;
//		    	}
//			    
//			    
//			    double t1=Math.abs(lineStart.time-temp);
//			    double t2=Math.abs(lineEnd.time-temp);
//			    double t=Math.min(t1, t2);
//			    
//			    ae+=t;
//			}
//			return ae;
//		}
    public double getMaxErrorTime(List<GPSPoint> trajectory, GPSLine[] lines, String ss) {
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;


        var MERCATOR_LATITUDE_LB = 2.5 * 2.0 - Math.PI / 2;
        var MERCATOR_LATITUDE_UB = 87.5 * 2.0 - Math.PI / 2;

        var earthRadius = 6378100.0;
        double rx, ry, sf = 0;// Longitude/latitude in radian.
        var SCALE_FACTOR_PRECISION = 1e-4;
        for (var i = 0; i <= trajectory.size() - 1; ++i) {
            var p = (GPSPoint) iTrajectory.get(i);
            sf += 1.0 / Math.cos(Math.toRadians(p.latitude));
        }
        sf /= trajectory.size();
        sf = Math.round(sf / SCALE_FACTOR_PRECISION) * SCALE_FACTOR_PRECISION;
        double x, y;
        Map<Double, Double> mapx = new HashMap<>();
        Map<Double, Double> mapy = new HashMap<>();
        for (var i = 0; i <= trajectory.size() - 1; ++i) {
            var p = (GPSPoint) iTrajectory.get(i);
            rx = Math.toRadians(p.longitude);
            ry = Math.toRadians(limitVal(p.latitude, MERCATOR_LATITUDE_LB, MERCATOR_LATITUDE_UB));
            ry = Math.log(Math.abs(Math.tan(ry) + 1.0 / Math.cos(ry)));

            x = rx * earthRadius / sf;
            y = ry * earthRadius / sf;
            // System.out.println("raw  "+ x+ " "+y );
            mapx.put(p.longitude, x);
            mapy.put(p.latitude, y);
        }


        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.time >= lines[l_index + 1].startPoint.time) {
                    l_index++;
                }

            }
            var lineStart = lines[l_index].getStartPoint();
            var lineEnd = lines[l_index].getEndPoint();

            double x1 = mapx.get(lineStart.longitude);
            double y1 = mapy.get(lineStart.latitude);
            double x2 = mapx.get(lineEnd.longitude);
            double y2 = mapy.get(lineEnd.latitude);
            double px = mapx.get(loc.longitude);
            double py = mapy.get(loc.latitude);

            double xx, yy;

            Point retVal;

            var dx = x1 - x2;
            var dy = y1 - y2;

            if (Math.abs(dx) < 0.00000001 && Math.abs(dy) < 0.00000001) {
                xx = x1;
                yy = y2;
            } else {
                var u = (px - x1) * (x1 - x2) + (py - y1) * (y1 - y2);
                u = u / ((dx * dx) + (dy * dy));
                xx = x1 + u * dx;
                yy = y1 + u * dy;
            }

            var totdis = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            var dis1 = Math.sqrt((x1 - xx) * (x1 - xx) + (y1 - yy) * (y1 - yy));

            var dx1 = (x2 - x1);
            var dy1 = (y2 - y1);
            var dx2 = (xx - x1);
            var dy2 = (yy - y1);

            var ratio = dis1 / totdis;
            if (ratio > 1) ratio = 1;
            if (dx1 * dx2 + dy1 * dy2 < 0) ratio = 0;
            var temp = lineStart.time + 1.0 * ratio * (lineEnd.time - lineStart.time);
            if (totdis == 0) {
                temp = (double) lineStart.time;
            }
            // double t1=Math.abs(lineStart.time-temp);
            // double t2=Math.abs(lineEnd.time-temp);
            // double t=Math.min(t1, t2);
            var t = Math.abs(loc.time - temp);

            if (ss.indexOf("mopsi") != -1) {
                t = t / 1000;
            }
            if (t > ae) ae = t;

            if (t > 1000000) {
//					//39.9596138 116.415512083333  CISED_half geolife_53
//					System.out.println((lineEnd.time-lineStart.time)+" "+lineStart.time+ " "+lineEnd.time+ " "+loc.time+ " "+lineStart.latitude+" "+lineStart.longitude
//							+ " "+lineEnd.latitude+" "+lineEnd.longitude+ " "+loc.latitude+ " "+loc.longitude
//							+ " "+dis1+" "+totdis);
//			    	System.out.println((dis1/totdis)+ " "+((double)1.0*(loc.time-lineStart.time)/(lineEnd.time-lineStart.time)));
////					System.out.println(lineEnd.time-lineStart.time+" "+getDistanceOfP2P(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude)
////							+" "
////							+getDistanceOfP2P(lineStart.latitude, lineStart.longitude, loc.latitude, loc.longitude)
////							+" "
////							+getDistanceOfP2P(loc.latitude, loc.longitude, lineEnd.latitude, lineEnd.longitude));

            }
        }
        return ae;
    }

    public double getAveErrorTime(List<GPSPoint> trajectory, GPSLine[] lines, String ss) {
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;


        var MERCATOR_LATITUDE_LB = 2.5 * 2.0 - Math.PI / 2;
        var MERCATOR_LATITUDE_UB = 87.5 * 2.0 - Math.PI / 2;

        var earthRadius = 6378100.0;
        double rx, ry, sf = 0;// Longitude/latitude in radian.
        var SCALE_FACTOR_PRECISION = 1e-4;
        for (var i = 0; i <= trajectory.size() - 1; ++i) {
            var p = (GPSPoint) iTrajectory.get(i);
            sf += 1.0 / Math.cos(Math.toRadians(p.latitude));
        }
        sf /= trajectory.size();
        sf = Math.round(sf / SCALE_FACTOR_PRECISION) * SCALE_FACTOR_PRECISION;
        double x, y;
        Map<Double, Double> mapx = new HashMap<>();
        Map<Double, Double> mapy = new HashMap<>();
        for (var i = 0; i <= trajectory.size() - 1; ++i) {
            var p = (GPSPoint) iTrajectory.get(i);
            rx = Math.toRadians(p.longitude);
            ry = Math.toRadians(limitVal(p.latitude, MERCATOR_LATITUDE_LB, MERCATOR_LATITUDE_UB));
            ry = Math.log(Math.abs(Math.tan(ry) + 1.0 / Math.cos(ry)));

            x = rx * earthRadius / sf;
            y = ry * earthRadius / sf;
            // System.out.println("raw  "+ x+ " "+y );
            mapx.put(p.longitude, x);
            mapy.put(p.latitude, y);
        }


        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.time >= lines[l_index + 1].startPoint.time) {
                    l_index++;
                }

            }
            var lineStart = lines[l_index].getStartPoint();
            var lineEnd = lines[l_index].getEndPoint();

            double x1 = mapx.get(lineStart.longitude);
            double y1 = mapy.get(lineStart.latitude);
            double x2 = mapx.get(lineEnd.longitude);
            double y2 = mapy.get(lineEnd.latitude);
            double px = mapx.get(loc.longitude);
            double py = mapy.get(loc.latitude);

            double xx, yy;

            var dx = x1 - x2;
            var dy = y1 - y2;

            if (Math.abs(dx) < 0.00000001 && Math.abs(dy) < 0.00000001) {
                xx = x1;
                yy = y2;
            } else {
                var u = (px - x1) * (x1 - x2) + (py - y1) * (y1 - y2);
                u = u / ((dx * dx) + (dy * dy));
                xx = x1 + u * dx;
                yy = y1 + u * dy;
            }

            var totdis = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            var dis1 = Math.sqrt((x1 - xx) * (x1 - xx) + (y1 - yy) * (y1 - yy));

            var dx1 = (x2 - x1);
            var dy1 = (y2 - y1);
            var dx2 = (xx - x1);
            var dy2 = (yy - y1);

            var ratio = dis1 / totdis;
            if (ratio > 1) ratio = 1;
            if (dx1 * dx2 + dy1 * dy2 < 0) ratio = 0;
            var temp = lineStart.time + 1.0 * ratio * (lineEnd.time - lineStart.time);

            if (totdis == 0) {
                temp = (double) lineStart.time;
            }


//			    double t1=Math.abs(lineStart.time-temp);
//			    double t2=Math.abs(lineEnd.time-temp);
//			    double t=Math.min(t1, t2);
            var t = Math.abs(loc.time - temp);


            if (ss.indexOf("mopsi") != -1) {
                t = t / 1000;
            }


            ae += t;
        }
        return ae;
    }


    // 求SED最大距离
    public double printToConsoleSED(List<GPSPoint> trajectory, GPSLine[] lines) {
        // System.out.println("---Compress By My Algorithm to : " + lines.length
        // + " lines");
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

//			if (n == 4) {
//				System.out.println(n);
//			}
            //
            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.time >= lines[l_index + 1].startPoint.time) {
                    l_index++;
                }

            }


            var lineStart = lines[l_index].getStartPoint();
            var lineEnd = lines[l_index].getEndPoint();
            var tmp = DisCalculator.predict(lineStart, lineEnd, loc); // 计算同步点
            var dis = getDistanceOfP2P(tmp.latitude, tmp.longitude, loc.latitude, loc.longitude);
            // double dis = getDistanceOfP2P(trajectory.get(n+1).latitude, trajectory.get(n+1).longitude,loc.latitude,loc.longitude);

            // double dis = getDistanceOfP2R(lines[l_index], loc);

//			Point pe = new Point(loc, lines[l_index].getStartPoint());
//			GPSPoint ps = this.getDc().predict(lines[l_index].getStartPoint(), lines[l_index].getEndPoint(), loc);
//			Point psxy = new Point(ps, lines[l_index].getStartPoint());
//			double dx = pe.x - psxy.x;
//			double dy = pe.y - psxy.y;
//
//			double dis = Math.sqrt(dx * dx + dy * dy);
//			de = de + dis;
//			ae = ae + Math.abs(dis);

            // if (Math.abs(dis)>10) {System.out.println(dis);}

            if (me < Math.abs(dis)) {
                me = Math.abs(dis);
                me_pindex = n;
                me_lindex = l_index;
                // System.out.println(" this point is "+n);
                // if (Math.abs(dis)>10) {System.out.println(n+"   "+l_index);}
            }

            if (1000000 < Math.abs(dis)) {

                // System.out.println(" this point is "+n);
                // System.out.println(" this line is "+l_index);
            }

            if (getThreshold() < Math.abs(dis)) {
                if (lines[l_index].startPoint.index >= lines[l_index].endPoint.index - 2) ;
//			System.out.println("Point " + n + " breaks up bound : " + dis); // For
//			System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point "
//					+ lines[l_index].getEndPoint().index); // Debug
//			System.out.println("lineId: " + l_index);

                // errorPoints++;
            }

        }

        // System.out.println(" Average Absolute Error : " +
        // ae/trajectory.size());
        // System.out.println(" Average Error : " + de/trajectory.size());
        // System.out.println("    Max Error : " + me + "; Point No." + me_pindex);
        // System.out.println("ErrorPoints: " + errorPoints);
        // System.out.print(" Isvalid : " + trajectory.get(me_pindex).isValid);
        // System.out.print(" Distance to R' : " +
        // trajectory.get(me_pindex).dis);
        // System.out.println(" Number: "+ numberOfBigErrorPoints + " Ratio : "
        // + 1.0* numberOfBigErrorPoints /trajectory.size() );
        // System.out.println("    Max Error : " + me + "; Point No." + me_pindex);
        return me;

    }

    public double printToConsolePED(List<GPSPoint> trajectory, GPSLine[] lines) {
        // System.out.println("---Compress By My Algorithm to : " + lines.length
        // + " lines");
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

//			if (n == 4) {
//				System.out.println(n);
//			}
            //
            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.time >= lines[l_index + 1].startPoint.time) {
                    l_index++;
                }

            }


            var lineStart = lines[l_index].getStartPoint();
            var lineEnd = lines[l_index].getEndPoint();
            var dis = getDistance(lineStart, lineEnd, loc);

            // double dis = getDistanceOfP2R(lines[l_index], loc);

//			Point pe = new Point(loc, lines[l_index].getStartPoint());
//			GPSPoint ps = this.getDc().predict(lines[l_index].getStartPoint(), lines[l_index].getEndPoint(), loc);
//			Point psxy = new Point(ps, lines[l_index].getStartPoint());
//			double dx = pe.x - psxy.x;
//			double dy = pe.y - psxy.y;
//
//			double dis = Math.sqrt(dx * dx + dy * dy);
//			de = de + dis;
//			ae = ae + Math.abs(dis);

            if (me < Math.abs(dis)) {
                me = Math.abs(dis);
                me_pindex = n;
                me_lindex = l_index;
            }
            if (getThreshold() < Math.abs(dis)) {

                //	System.out.println("this point: " + Math.abs(dis)+"index"+n);
                //	break;
            }
            // yihaofu debug
			/*if (20 < Math.abs(dis)) {
//					System.out.println("lineStart:  "+lineStart.time);
//					System.out.println("lineEnd:    "+lineEnd.time);
					System.out.println("lineStart:  "+lineStart.index);
					System.out.println("lineEnd:    "+lineEnd.index);
					System.out.println("thisP:      "+loc.time);
					
					System.out.println("this point: " + Math.abs(dis)+"index"+n);
					break;
				}*/

            if (getThreshold() < Math.abs(dis)) {
                // if(lines[l_index].startPoint.index >= lines[l_index].endPoint.index -2);
//				System.out.println("Point " + n + " breaks up bound : " + dis); // For
//				System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point "
//						+ lines[l_index].getEndPoint().index); // Debug
//				System.out.println("lineId: " + l_index);
//
//				System.out.println("111  "+  lines[l_index].getStartPoint().time);
//				System.out.println("222  "+  lines[l_index].getEndPoint().time);
//				System.out.println("333  "+  trajectory.get(n).time);
//				
//				
//				errorPoints++;
            }

        }

        // System.out.println(" Average Absolute Error : " +
        // ae/trajectory.size());
        // System.out.println(" Average Error : " + de/trajectory.size());
        // System.out.println("    Max Error : " + me + "; Point No." + me_pindex);
        // System.out.println("ErrorPoints: " + errorPoints);
        // System.out.print(" Isvalid : " + trajectory.get(me_pindex).isValid);
        // System.out.print(" Distance to R' : " +
        // trajectory.get(me_pindex).dis);
        // System.out.println(" Number: "+ numberOfBigErrorPoints + " Ratio : "
        // + 1.0* numberOfBigErrorPoints /trajectory.size() );
        // testyihaofu
        // System.out.println("start point: " + lines[me_lindex].getStartPoint().index + " end point "
        //		+ lines[me_lindex].getEndPoint().index);
        // System.out.println("me_pindex : "+me_pindex);
        return me;

    }


    public double getAveError(List<GPSPoint> trajectory, GPSLine[] lines) {
        var l_index = 0; // index to lines
        double ae = 0;
        double de = 0; // 累计算术误差
        double me = 0; // 累计绝对误差
        var me_pindex = 0; // index of the max err point
        var me_lindex = 0; //

        var errorPoints = 0;
        for (var n = 0; n < trajectory.size(); n++) { // 轨迹点

            var loc = (GPSPoint) trajectory.get(n);
            if (l_index < lines.length - 1) {
                // 针对DP等算法实现，没有给Line编号
                if (loc.lineId == 0 && loc.time >= lines[l_index + 1].startTime()) {
                    l_index++;

                    // 针对OPER的实现，给每个Line顺序编号，并且给每个点也做了标记，记录它所属的line.
                } else if (loc.lineId >= lines[l_index + 1].getIndex() || loc.lineId > lines[l_index].getIndex()) {
                    l_index++;
                }

            }

            if (this.getDc() instanceof DED) {
                var radialDis = ((DED) getDc()).getRadialDis(lines[l_index], loc);
                var perDis = ((DED) getDc()).getPerpendicularDis(lines[l_index], loc);
                if (getThreshold() < Math.abs(perDis)) {
                    System.out.println("Point " + n + " breaks up bound : " + perDis); // For
                    System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);// Debug
                    System.out.println("lineId: " + l_index);
                    errorPoints++;
                }
                if (radialThreshold < Math.abs(radialDis)) {
                    System.out.println("Point " + n + " breaks up radialBound : " + radialDis); // For
                    System.out.println("start point: " + lines[l_index].getStartPoint().index + " end point " + lines[l_index].getEndPoint().index);
                    System.out.println("lineId: " + l_index);
                    errorPoints++;
                }
            } else {
                var dis = this.getDc().getDistanceOfP2Line(lines[l_index], loc);
                de = de + dis;
                ae = ae + Math.abs(dis);

                if (me < Math.abs(dis)) {
                    me = Math.abs(dis);
                    me_pindex = n;
                    me_lindex = l_index;
                }

                if (getThreshold() < Math.abs(dis)) {

                    errorPoints++;
                }

            }

        }

        // System.out.println(" Average Absolute Error : " + ae/trajectory.size());
        // System.out.println(" Average Error : " + de/trajectory.size());
        // System.out.println(" Max Error : " + me + "; Point No." + me_pindex);
        // System.out.println("ErrorPoints: "+errorPoints);

        return ae;
    }

    public void export(GPSLine[] res) {
        var path = Paths.get(this.strFileName);

        var dir = path.getParent();
        var srcName = path.getFileName();

        var splited = srcName.toString().split("\\.");

        var index = srcName.toString().split("\\.")[0];

        var exName = Paths.get(dir.toString(), index + ".compressed_" + this + "_" + this.getDc().toString() + ".txt");

        var sb = new StringBuilder();

        GPSPoint point;

        for (var line : res) {
            point = line.getStartPoint();
            sb.append("0;").append(point.time).append(";").append(point.latitude).append(";").append(point.longitude).append("\n");
        }
        point = res[res.length - 1].getEndPoint();
        sb.append("0;").append(point.time).append(";").append(point.latitude).append(";").append(point.longitude).append("\n");

        try {
            System.out.println("write to: " + exName);
            if (!Files.exists(exName)) {
                Files.createFile(exName);
            }

            Files.write(exName, sb.toString().getBytes());
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public double getThreshold() {
        return threshold;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public void setPEDandSED(double ped, double sed) {
        this.PEDerr = ped;
        this.SEDerr = sed;
    }


    // CDR_B   Buffer
    public void setBuffer(double Buffer) {
        this.Buffer = Buffer;
    }

    public double getBuffer() {
        return Buffer;
    }

    public DisCalculator getDc() {
        return dc;
    }

    public void setDc(DisCalculator dc) {
        this.dc = dc;
    }
}
