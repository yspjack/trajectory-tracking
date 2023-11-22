package org.act.util;

public abstract class DisCalculator {
    protected static final double EARTH_RADIUS = 6378137;
    protected static double threshold;
    protected static double threshold2;
    protected static double ratio;

    public static void getThreshold(double th1, double th2) {
        threshold = th1;
        threshold2 = th2;
        ratio = (threshold2) / (threshold);
    }


    public abstract double getDistanceOfP2Line(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc);

    // public abstract double getDistanceOfP2Line(GPSLine line, GPSPoint loc,GPSPoint suc);
    // public abstract double getDistanceOfP2Line(double alpha, GPSPoint loc,GPSPoint suc);
    public abstract double getDistanceOfP2Line(GPSLine line, GPSPoint loc);

    // public abstract double getDistanceOfP2Segment(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc);

    protected static double rad(double d) {
        return d * Math.PI / 180.0;
    }

    protected static double degree(double d) {
        return d / Math.PI * 180.0;
    }

    /*
     * 计算两个点之间的距离，绝对值
     *  1. Lat1 Lung1 表示A点经纬度，Lat2 Lung2 表示B点经纬度；
     *  2. a=Lat1 – Lat2 为两点纬度之差  b=Lung1 -Lung2 为两点经度之差；
     *  3. 6378.137为地球半径，单位为千米；
     *  计算出来的结果单位为千米。
     */
    public static double getDistanceOfP2P(double lat1, double lng1, double lat2, double lng2) {

        double radLat1 = rad(lat1);
        double radLat2 = rad(lat2);

        double dw = radLat1 - radLat2;          // 纬度差，弧度
        double dj = rad(lng1) - rad(lng2);      // 经度差，弧度

        // 两个点相对于球心的夹角
        double s = 2 * Math.asin(Math.sqrt(Math.pow(Math.sin(dw / 2), 2) + Math.cos(radLat1) * Math.cos(radLat2) * Math.pow(Math.sin(dj / 2), 2)));
        // 距离
        s = s * EARTH_RADIUS;
        return s;
    }

    public static double getDistanceOfP2P(GPSPoint start, GPSPoint end) {

        double lat1 = start.latitude;
        double lng1 = start.longitude;
        double lat2 = end.latitude;
        double lng2 = end.longitude;
        double radLat1 = rad(lat1);
        double radLat2 = rad(lat2);

        double dw = radLat1 - radLat2;          // 纬度差，弧度
        double dj = rad(lng1) - rad(lng2);      // 经度差，弧度

        // 两个点相对于球心的夹角
        double s = 2 * Math.asin(Math.sqrt(Math.pow(Math.sin(dw / 2), 2) + Math.cos(radLat1) * Math.cos(radLat2) * Math.pow(Math.sin(dj / 2), 2)));

        // 距离
        s = s * EARTH_RADIUS;
        return s;
    }

    public static GPSPoint predict(GPSPoint lineStart, GPSPoint lineEnd, double time) {
        GPSPoint tmp = new GPSPoint();

        GPSLine line = new GPSLine();
        double dis = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        double t = (double) (time) / (lineEnd.time - lineStart.time);
        if (t < 1e-9) {
            return lineStart;
            //} else if (1.0 - t < 1e-9) {
            //	return lineEnd;
        }
        dis = t * dis;
        if (dis == 0) {
            return lineStart;
        }

        double angle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        line.setStartPoint(lineStart);
        line.setLength(dis);
        line.setAngle(angle);
        line.setEndPoint(lineStart);
        updateEndPoint(line);
        tmp.latitude = line.endLatitude();
        tmp.longitude = line.endLongitude();
        return tmp;
    }


    public static GPSPoint computerThatLonLat(GPSPoint lineStart, GPSPoint lineEnd, double time) {


        double lon = lineStart.longitude;
        double lat = lineStart.latitude;
        double alpha1 = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        double dis = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        double t = (double) (time) / (lineEnd.time - lineStart.time);
        double dist = t * dis;

        double a = 6378137;
        /** 短半径b=6356752.3142 */
        double b = 6356752.3142;
        /** 扁率f=1/298.2572236 */
        double f = 1 / 298.2572236;
        GPSPoint loc = new GPSPoint();

        // double alpha1 = rad(brng);
        double sinAlpha1 = Math.sin(alpha1);
        double cosAlpha1 = Math.cos(alpha1);

        double tanU1 = (1 - f) * Math.tan(rad(lat));
        double cosU1 = 1 / Math.sqrt((1 + tanU1 * tanU1));
        double sinU1 = tanU1 * cosU1;
        double sigma1 = Math.atan2(tanU1, cosAlpha1);
        double sinAlpha = cosU1 * sinAlpha1;
        double cosSqAlpha = 1 - sinAlpha * sinAlpha;
        double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
        double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
        double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));

        double cos2SigmaM = 0;
        double sinSigma = 0;
        double cosSigma = 0;
        double sigma = dist / (b * A), sigmaP = 2 * Math.PI;
        while (Math.abs(sigma - sigmaP) > 1e-12) {
            cos2SigmaM = Math.cos(2 * sigma1 + sigma);
            sinSigma = Math.sin(sigma);
            cosSigma = Math.cos(sigma);
            double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
            sigmaP = sigma;
            sigma = dist / (b * A) + deltaSigma;
        }

        double tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
        double lat2 = Math.atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1, (1 - f) * Math.sqrt(sinAlpha * sinAlpha + tmp * tmp));
        double lambda = Math.atan2(sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1);
        double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        double L = lambda - (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));

        // double revAz = Math.atan2(sinAlpha, -tmp); // final bearing

        // DecimalFormat df = new DecimalFormat("#.00");


        loc.latitude = lat2 * 180 / Math.PI;
        loc.longitude = L * 180 / Math.PI;
        return loc;

        // System.out.println(revAz);
        // System.out.println(lon+deg(L)+","+deg(lat2));
    }


    public static GPSPoint predict(GPSPoint lineStart, GPSPoint lineEnd, GPSPoint loc) {
        GPSPoint tmp = new GPSPoint();
        //
        // double deltLat = lineStart.latitude - lineEnd.latitude;
        // double deltLng = lineStart.longitude - lineEnd.longitude;
        // double deltTime = lineStart.time - lineEnd.time;
        //
        // tmp.latitude = lineStart.latitude + loc.time*deltLat/deltTime;
        // tmp.longitude = lineStart.longitude + loc.time*deltLng/deltTime;
        // tmp.time = loc.time;
        //

        /*
         * tmp = SLERP(lineStart, lineEnd, loc);
         */

        GPSLine line = new GPSLine();
        double dis = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        double t = (double) (loc.time - lineStart.time) / (lineEnd.time - lineStart.time);
        if (t < 1e-9) {
            return lineStart;
        }
        // else if (1.0 - t < 1e-9) {
        //	return lineEnd;
        //}
        dis = t * dis;
        if (dis == 0) {
            return lineStart;
        }

        double angle = getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        line.setStartPoint(lineStart);
        line.setLength(dis);
        line.setAngle(angle);
        line.setEndPoint(new GPSPoint());
        line.getEndPoint().latitude = lineEnd.latitude;
        line.getEndPoint().longitude = lineEnd.longitude;
        updateEndPoint(line);
        tmp.latitude = line.endLatitude();
        tmp.longitude = line.endLongitude();
        tmp.time = loc.time;
        return tmp;
    }


    /////////////////////////////////////////////////////////////////////////////////
    // 以下代码都应该转化为球面坐标下相应计算
    //	直线  		-> 相对球心的弧线
    //  直线的角度 	-> 弧线所在大圆的角度
    //  直线的夹角     	-> 弧线所在大圆的夹角
    //	点到直线的距离 -> 点到弧线的距离
    //////////////////////////////////////////////////////////////////////////////////

    /**
     * 返回 从矢量1 到 矢量2 之间的 相对夹角，介于 (-Pi, Pi)。逆时针 为 正； 顺 时针 为 负
     * @param angle1：矢量1的角度, 矢量1为出发矢量
     * @param angle2：矢量2的角度
     * @return [-Pi, Pi]
     */
    public static double getIncludedAngle(double angle1, double angle2) {
        double theta = angle2 - angle1;
        if (theta > Math.PI) theta = -2 * Math.PI + theta;
        if (theta < -Math.PI) theta = 2 * Math.PI + theta;
        return theta;
    }


    /*
     * 计算点到直线的最短距离
     */


    ///////////////////////////////////////////////////////////////////////////////
    ////////////以下所有和角度有关的代码都是近似计算,把曲面坐标投影到了平面坐标/////////////
    ////////////@todo...这些代码都应该彻底改造成球面坐标///////////////////////////////

    /**
     * @给定矢量的起点和端点（经纬度），该代码返回矢量相对x坐标的夹角。
     *
     * 注意，这个角度计算法是特异的，并没有完全按球面坐标计算，而是直接投影到了平面坐标
     * 以后也需要按照这种方法映射回去（比如已知起点，距离和角度，求端点的时候）。
     *
     * @param latS
     * @param lngS
     * @param latE
     * @param lngE
     * @return
     *        [0, 2pi)
     */
    public static double getAngleOfVector(double latS, double lngS, double latE, double lngE) {

        if (latS == latE && lngS == lngE) return 0;
        else if (latS == latE && lngS < lngE) return 0;
        else if (latS == latE && lngS > lngE) return Math.PI;
        else if (latS < latE && lngS == lngE) return Math.PI / 2;
        else if (latS > latE && lngS == lngE) return 3.0 * Math.PI / 2;

        double high = getDistanceOfP2P(latE, lngE, latS, lngE);
        double length = getDistanceOfP2P(latS, lngS, latE, lngE);
        if (length == 0) return 0;


        double angle = Math.asin(high / length);// 返回值[-pi/2, pi/2]
        // System.out.println("Debug: angle = " + angle );
        // 修正角度值
        // if (latS<=latE && lngS<=lngE) angle =angle;			//第1区间
        if (latS < latE && lngS > lngE) angle = Math.PI - angle;    // 第2区间
        if (latS > latE && lngS > lngE) angle = Math.PI + angle;    // 第3区间
        if (latS > latE && lngS < lngE) angle = 2 * Math.PI - angle;    // 第4区间

        return angle;
    }

    public static void updateEndPoint(GPSLine line) {
        if (line.getLength() == 0) {
            return;
        }

        // line.endPoint = new GPSPoint();
        // 计算端点的纬度

        double dw = line.getLength() * Math.sin(line.getAngle()) / EARTH_RADIUS; // 平面坐标下的垂直距离，即经度线上的弧线距离，注意有正和负。向北 为正，向南为负


        line.getEndPoint().latitude = line.getStartPoint().latitude + dw / Math.PI * 180; // 转化成度数

        // 计算端点的经度：以下代码为getDistanceofP2P的逆过程
        double dj = Math.pow(Math.sin(line.getLength() / (EARTH_RADIUS * 2)), 2) - Math.pow(Math.sin(dw / 2), 2);
        dj = dj / (Math.cos(line.startLatitude() / 180 * Math.PI) * Math.cos(line.endLatitude() / 180 * Math.PI));
        dj = Math.sqrt(dj);
        dj = Math.asin(dj);
        dj = 2 * dj; // 弧度差

        if (line.startLongitude() > 0 && line.endLongitude() < 0) {
            dj = -dj;
            line.getEndPoint().longitude = line.startLongitude() + dj / Math.PI * 180 + 360;
        } else if (line.startLongitude() < 0 && line.endLongitude() > 0) {
            line.getEndPoint().longitude = (line.startLongitude() + dj / Math.PI * 180) - 360;
        } else if (line.getAngle() > Math.PI / 2 && line.getAngle() < 3.0 * Math.PI / 2) {
            dj = -dj;
            line.getEndPoint().longitude = line.startLongitude() + dj / Math.PI * 180;
        } else {
            line.getEndPoint().longitude = line.startLongitude() + dj / Math.PI * 180;
        }

        // if (line.angle > Math.PI/2 && line.angle< 3.0*Math.PI/2){
        // dj=-dj;
        // }
        // line.endPoint.longitude = line.startLongitude() + dj/Math.PI*180;

        //////////////////////////////// Debug//////////////////////////
        // double len = this.getDistanceOfP2P(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(len - line.lengthRF)>0.00001){
        // System.out.println("Debug, differ of len is : " + (len - line.lengthRF));
        // }
        // double angx = this.getAngleOfVector(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(angx - line.angleRF)>0.00001){
        // System.out.println("Debug: ang = " + angx + "; line.angleRF=" +
        // line.angleRF);
        // }
        //////////////////////////////////////////////////////////////
    }

    public static GPSPoint getEndPoint(GPSPoint start, double dist, double theta, long time, int index) {
        if (dist == 0) {
            return start;
        }

        GPSPoint end = new GPSPoint();
        // 计算端点的纬度
        double dw = dist * Math.sin(theta) / EARTH_RADIUS;    // 平面坐标下的垂直距离，即经度线上的弧线距离，注意有正和负。向北 为正，向南为负
        end.latitude = start.latitude + dw / Math.PI * 180; // 转化成度数

        // 计算端点的经度：以下代码为getDistanceofP2P的逆过程
        double dj = Math.pow(Math.sin(dist / (EARTH_RADIUS * 2)), 2) - Math.pow(Math.sin(dw / 2), 2);
        dj = dj / (Math.cos(start.latitude / 180 * Math.PI) * Math.cos(end.latitude / 180 * Math.PI));
        dj = Math.sqrt(dj);
        dj = Math.asin(dj);
        dj = 2 * dj; // 弧度差

        if (theta > Math.PI / 2 && theta < 3.0 * Math.PI / 2) {
            dj = -dj;
        }
        end.longitude = start.longitude + dj / Math.PI * 180;
        end.time = time;
        end.index = index;

        return end;

        //////////////////////////////// Debug//////////////////////////
        // double len = this.getDistanceOfP2P(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(len - line.lengthRF)>0.00001){
        // System.out.println("Debug, differ of len is : " + (len - line.lengthRF));
        // }
        // double angx = this.getAngleOfVector(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(angx - line.angleRF)>0.00001){
        // System.out.println("Debug: ang = " + angx + "; line.angleRF=" +
        // line.angleRF);
        // }
        //////////////////////////////////////////////////////////////

    }

    public static GPSPoint getEndPoint(GPSPoint lineStart, GPSPoint lineEnd, double t) {

        double theta = DisCalculator.getAngleOfVector(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);

        double dis = getDistanceOfP2P(lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude);
        // double t = (double) (time) / (lineEnd.time - lineStart.time);


        if (Double.isNaN(t) || Double.isInfinite(t)) {
            return lineEnd;
        }
        double dist = t * dis;

        if (dist < 1e-14) {
            return lineStart;
        }

        GPSPoint end = new GPSPoint();
        // 计算端点的纬度
        double dw = dist * Math.sin(theta) / EARTH_RADIUS; // 平面坐标下的垂直距离，即经度线上的弧线距离，注意有正和负。向北 为正，向南为负
        end.latitude = lineStart.latitude + dw / Math.PI * 180; // 转化成度数

        // 计算端点的经度：以下代码为getDistanceofP2P的逆过程
        double dj = Math.pow(Math.sin(dist / (EARTH_RADIUS * 2)), 2) - Math.pow(Math.sin(dw / 2), 2);
        dj = dj / (Math.cos(lineStart.latitude / 180 * Math.PI) * Math.cos(end.latitude / 180 * Math.PI));
        dj = Math.sqrt(dj);
        dj = Math.asin(dj);
        dj = 2 * dj; // 弧度差

        if (theta > Math.PI / 2 && theta < 3.0 * Math.PI / 2) {
            dj = -dj;
        }
        end.longitude = lineStart.longitude + dj / Math.PI * 180;
        // end.time = time;
        // end.index = index;

        return end;

        //////////////////////////////// Debug//////////////////////////
        // double len = this.getDistanceOfP2P(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(len - line.lengthRF)>0.00001){
        // System.out.println("Debug, differ of len is : " + (len - line.lengthRF));
        // }
        // double angx = this.getAngleOfVector(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(angx - line.angleRF)>0.00001){
        // System.out.println("Debug: ang = " + angx + "; line.angleRF=" +
        // line.angleRF);
        // }
        //////////////////////////////////////////////////////////////

    }


    public static GPSPoint getEndPoint(GPSPoint start, double dist, double theta, GPSPoint end) {
        if (dist == 0) {
            return start;
        }

        GPSPoint res = new GPSPoint();
        // 计算端点的纬度
        double dw = dist * Math.sin(theta) / EARTH_RADIUS;    // 平面坐标下的垂直距离，即经度线上的弧线距离，注意有正和负。向北 为正，向南为负
        res.latitude = start.latitude + dw / Math.PI * 180; // 转化成度数

        // 计算端点的经度：以下代码为getDistanceofP2P的逆过程
        double dj = Math.pow(Math.sin(dist / (EARTH_RADIUS * 2)), 2) - Math.pow(Math.sin(dw / 2), 2);
        dj = dj / (Math.cos(start.latitude / 180 * Math.PI) * Math.cos(end.latitude / 180 * Math.PI));
        dj = Math.sqrt(dj);
        dj = Math.asin(dj);
        dj = 2 * dj; // 弧度差

        if (start.longitude > 0 && end.longitude < 0) {
            res.longitude = (start.longitude + dj / Math.PI * 180) - 360;
        } else if (start.longitude < 0 && end.longitude > 0) {
            dj = -dj;
            res.longitude = start.longitude + dj / Math.PI * 180 + 360;
        } else if (theta > Math.PI / 2 && theta < 3.0 * Math.PI / 2) {
            dj = -dj;
            res.longitude = start.longitude + dj / Math.PI * 180;
        } else {
            res.longitude = start.longitude + dj / Math.PI * 180;
        }

        res.time = end.time;
        res.index = end.index;

        return res;

        //////////////////////////////// Debug//////////////////////////
        // double len = this.getDistanceOfP2P(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(len - line.lengthRF)>0.00001){
        // System.out.println("Debug, differ of len is : " + (len - line.lengthRF));
        // }
        // double angx = this.getAngleOfVector(line.startLatitude, line.startLongitude,
        // line.endLatitudeF, line.endLongitudeF);
        // if (Math.abs(angx - line.angleRF)>0.00001){
        // System.out.println("Debug: ang = " + angx + "; line.angleRF=" +
        // line.angleRF);
        // }
        //////////////////////////////////////////////////////////////

    }

}
