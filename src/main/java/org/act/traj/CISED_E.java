package org.act.traj;

import org.act.util.*;
import org.act.util.polygon.Point;
import org.act.util.polygon.Polygon;
import org.act.util.polygon.Vertex;

import java.util.LinkedList;
import java.util.List;

@SuppressWarnings("JavadocReference")
public class CISED_E extends TrajectoryCompressor {

    int isDebug = 0;

    int notin = 0; // 统计不在范围内的估计点

    int m = 0;// 参数，近似多边形的边数

    double[] deltX = new double[m];
    double[] deltY = new double[m];

    public CISED_E(int m) {
        this.setDc(new SED());
        this.m = m;
        deltX = new double[m];
        deltY = new double[m];
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }
    }

    public CISED_E() {
        // TODO Auto-generated constructor stub
        this.m = 10;
        this.setDc(new SED());
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
        // TODO Auto-generated method stub
//		return compressDiscontinuous();
        return compressContinuous();
//
    }


    private GPSLine[] compressContinuous() {


        int index = 1;
        int iStart = 0;
        int iEnd = 1;
        int lineId = 1; // 输出线段的编号 //Debug

        boolean isIntial = true;

        Polygon polygon_intersects = new Polygon();

        Polygon destPolygon = new Polygon();

        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);

            if (index == 40950) {
                System.out.println();
            }
            p.lineId = lineId;
            // 计算当前点对应的多边形
            Polygon poly_i = getPolyGon(p, start, threshold);

            if (isIntial) {
                polygon_intersects = poly_i;
                destPolygon = polygon_intersects;
                isIntial = false;
            } else {
                polygon_intersects = poly_i.intersects(polygon_intersects);

            }


            // 多边形交集为空，压缩终止
            if (polygon_intersects.size() == 0) {
                GPSPoint end = iTrajectory.get(iEnd);
                end = getDest(start, end, destPolygon);

                putLine(v, start, end, lineId);

                lineId++;

                // 初始化
                start = end;
                index = iEnd + 1;
                isIntial = true;
            } else {
                destPolygon = polygon_intersects;
                iEnd = index;
                index++;
            }

        }


        // 最后一条线段
        if (iEnd < iTrajectory.size()) {

            GPSPoint end = iTrajectory.get(iEnd);
            end = getDest(start, end, destPolygon);

            putLine(v, start, end, lineId);


        }


        // 转换成数组

        GPSLine[] lines = new GPSLine[v.size()];
        v.toArray(lines);

        return lines;

    }


    private GPSLine[] compressDiscontinuous() {
        int index = 1;
        int iStart = 0;
        int iEnd = 1;
        int lineId = 1; // 输出线段的编号 //Debug


        boolean isIntial = true;

        Polygon polygon_intersects = new Polygon();


        List<GPSLine> v = new LinkedList<>();

        GPSPoint start = iTrajectory.get(iStart);

        double alpha = 0.9238795325 * threshold;
        double omega = (0.9238795325 + 0.3826834324) * threshold;


        // 每组约束的解集为参数空间中的一个多边形
        Polygon poly_m_b = new Polygon();
        Polygon poly_n_c = new Polygon();
        Polygon poly_p_s = new Polygon();
        Polygon poly_q_t = new Polygon();

        Polygon poly_m_b_inters = new Polygon();
        Polygon poly_n_c_inters = new Polygon();
        Polygon poly_p_s_inters = new Polygon();
        Polygon poly_q_t_inters = new Polygon();

        while (index < iTrajectory.size()) {
            GPSPoint p = iTrajectory.get(index);
            p.lineId = lineId;

            if (index == 515) {
                System.out.println();
            }
            // 以起点为坐标原点，计算当前点在x-y平面的坐标
            Point p_i = new Point(p, start);


            // 初始化多边形
            // 每个点的约束是两条平行直线，初始多边形应该是一个平行四边形
            // 初始的平行四边形总是存在的
            if (isIntial) {
                poly_m_b_inters = getInitial(-alpha, alpha, p_i.x - alpha, p_i.x + alpha, p.time - start.time);
                poly_n_c_inters = getInitial(-alpha, alpha, p_i.y - alpha, p_i.y + alpha, p.time - start.time);
                poly_p_s_inters = getInitial(-omega, omega, p_i.x + p_i.y - omega, p_i.x + p_i.y + omega, p.time - start.time);
                poly_q_t_inters = getInitial(-omega, omega, p_i.y - p_i.x - omega, p_i.y - p_i.x + omega, p.time - start.time);

                poly_m_b = poly_m_b_inters;
                poly_n_c = poly_n_c_inters;
                poly_p_s = poly_p_s_inters;
                poly_q_t = poly_q_t_inters;

                isIntial = false;
            }
            // 添加新约束，求多边形与半平面的交集
            // 半平面约束的表示：直线
            // 用它的斜率和截距表示：斜率均为-t_i,截距护卫相反数

            else {
//				startTime = System.nanoTime();

                // 新加入的约束为两条平行直线
                // 新多边形为原多边形与两条平行直线的交集
                // 有一个多边形为空集，压缩终止。
                // 压缩终止时需要构造线段的起点和终点
                // 记录前一次的交集多边形，用于计算起点和终点
                poly_m_b = poly_m_b_inters;
                poly_n_c = poly_n_c_inters;
                poly_p_s = poly_p_s_inters;
                poly_q_t = poly_q_t_inters;

                // 新的交集多边形
                poly_m_b_inters = poly_m_b_inters.intersects(p.time - start.time, p_i.x - alpha, p_i.x + alpha);
                if (poly_m_b_inters.size() == 0) {
                    putLine(v, poly_m_b, poly_n_c, poly_p_s, poly_q_t, iStart, iEnd, lineId);
                    lineId++;

                    // 初始化
                    iStart = (iEnd + 1);

                    start = (GPSPoint) this.iTrajectory.get(iStart);
                    start.lineId = lineId;
                    index = iEnd + 2;
                    isIntial = true;

                    continue;
                }

                poly_n_c_inters = poly_n_c_inters.intersects(p.time - start.time, p_i.y - alpha, p_i.y + alpha);
                if (poly_n_c_inters.size() == 0) {
                    putLine(v, poly_m_b, poly_n_c, poly_p_s, poly_q_t, iStart, iEnd, lineId);
                    lineId++;

                    // 初始化
                    iStart = (iEnd + 1);
                    start = (GPSPoint) this.iTrajectory.get(iStart);
                    start.lineId = lineId;
                    index = iEnd + 2;
                    isIntial = true;
                    continue;
                }

                poly_p_s_inters = poly_p_s_inters.intersects(p.time - start.time, p_i.x + p_i.y - omega, p_i.x + p_i.y + omega);
                if (poly_p_s_inters.size() == 0) {
                    putLine(v, poly_m_b, poly_n_c, poly_p_s, poly_q_t, iStart, iEnd, lineId);
                    lineId++;

                    // 初始化
                    iStart = (iEnd + 1);
                    start = (GPSPoint) this.iTrajectory.get(iStart);
                    start.lineId = lineId;
                    index = iEnd + 2;
                    isIntial = true;
                    continue;
                }

                poly_q_t_inters = poly_q_t_inters.intersects(p.time - start.time, p_i.y - p_i.x - omega, p_i.y - p_i.x + omega);
                if (poly_q_t_inters.size() == 0) {
                    putLine(v, poly_m_b, poly_n_c, poly_p_s, poly_q_t, iStart, iEnd, lineId);
                    lineId++;

                    // 初始化
                    iStart = (iEnd + 1);
                    start = (GPSPoint) this.iTrajectory.get(iStart);
                    start.lineId = lineId;
                    index = iEnd + 2;
                    isIntial = true;
                    continue;
                }

//				timeOfIntersecs += (System.nanoTime() - startTime);
//				intersectCount++;
            }
            // 更新当前的和终点
            iEnd = index;
            index++;
//			System.out.println(index);
        }


        // 最后一条线段
        if (iEnd < iTrajectory.size()) {

            putLine(v, iStart, iEnd, lineId);

        }

        // 转换成数组


        GPSLine[] lines = new GPSLine[v.size()];
        v.toArray(lines);

        System.out.println("not in points: " + notin);
        return lines;
    }

    private void putLine(List<GPSLine> v, int iStart, int iEnd, int lineId) {
        GPSLine line = new GPSLine();
        line.index = lineId;
        GPSPoint p_s = (GPSPoint) this.iTrajectory.get(iStart);
        p_s.index = iStart;
        line.startPoint = p_s;// 注意，需要取最新时间

        GPSPoint p_e = (GPSPoint) this.iTrajectory.get(iEnd);
        p_e.index = iEnd; // 取当前点的前一个点作为终点
        line.endPoint = p_e;

//		line.angle = this.getAngleOfVector(line.startPoint.latitude, line.startPoint.longitude, line.endPoint.latitude,
//				line.endPoint.longitude);
//		line.length = this.getDistanceOfP2P(line.endPoint.latitude, line.endPoint.longitude, line.startPoint.latitude,
//				line.startPoint.longitude);
        v.add(line);
    }


    /**
     * 生成虚拟的起点和终点作为压缩线段
     * @param v
     * @param Start
     * @param End
     * @param lineId
     */
    private void putLine(List<GPSLine> v, GPSPoint Start, GPSPoint End, int lineId) {
        GPSLine line = new GPSLine();
        line.index = lineId;
        line.startPoint = Start;// 注意，需要取最新时间

        line.endPoint = End;

        Start.lineId = lineId;
        End.lineId = lineId;

//		line.angle = this.getAngleOfVector(line.startPoint.latitude, line.startPoint.longitude, line.endPoint.latitude,
//				line.endPoint.longitude);
//		line.length = this.getDistanceOfP2P(line.endPoint.latitude, line.endPoint.longitude, line.startPoint.latitude,
//				line.startPoint.longitude);
        v.add(line);
    }


    /**
     *
     * 根据最后有交集的四个多边形，计算出起点和终点，并把线段放入list中
     * @param v
     * @param poly_mb
     * @param poly_nc
     * @param poly_ps
     * @param poly_qt
     *
     */
    private void putLine(List<GPSLine> v, Polygon poly_mb, Polygon poly_nc, Polygon poly_ps, Polygon poly_qt, int iStart, int iEnd, int lineId) {
        GPSPoint start = iTrajectory.get(iStart);
        GPSPoint end = iTrajectory.get(iEnd);
        double m = 0;
        double b = 0;
        double n = 0;
        double c = 0;


        Point gravity = new Point();
        // 求poly_mb多边形的重心
        gravity = poly_mb.getGravity();
        m = gravity.x;
        b = gravity.y;


        // 求poly_nc多边形的重心
        gravity = poly_nc.getGravity();
        n = gravity.x;
        c = gravity.y;

        double p = m + n;
        double s = b + c;

        double q = n - m;
        double t = c - b;
        Point ps = new Point(p, s);
        Point qt = new Point(q, t);

        if (isDebug == 1) {
            if (poly_ps.isIn(ps) && poly_qt.isIn(qt)) {
                System.out.println("is in");
            } else {
                System.out.println("not in");
            }
        }

        if (!poly_ps.isIn(ps) || !poly_qt.isIn(qt)) {


            Polygon poly_mb_feasible = new Polygon();
            Polygon poly_qt_inver = poly_qt.getInverse();
            Vertex tmp = poly_ps.PointMaxx;
            Vertex tmp2 = poly_qt_inver.PointMinx;
            do {
                double x = (tmp.v.x + tmp2.v.x) / 2;
                double y = (tmp.v.y + tmp2.v.y) / 2;
                poly_mb_feasible.InsertBeforeHead(new Vertex(x, y));
                tmp2 = tmp2.next;
            } while (tmp2 != poly_qt_inver.PointMaxx);


            tmp = poly_ps.PointMinx;
            do {
                double x = (tmp.v.x + tmp2.v.x) / 2;
                double y = (tmp.v.y + tmp2.v.y) / 2;
                poly_mb_feasible.InsertBeforeHead(new Vertex(x, y));
                tmp2 = tmp2.next;
            } while (tmp2 != poly_qt_inver.PointMinx);


            poly_mb_feasible = poly_mb_feasible.intersects(poly_mb);
            gravity = poly_mb_feasible.getGravity();
            Vertex gravityVer = new Vertex(gravity.x, gravity.y);
            gravityVer.next = poly_mb_feasible.head.next;
            Polygon poly_nc_ps = new Polygon(), poly_nc_qt = new Polygon();
            Polygon poly_nc_inters = new Polygon();

            Point mb = new Point(), nc = new Point();
            for (tmp = gravityVer; tmp != poly_mb_feasible.head; tmp = tmp.next) {
                poly_nc.ListCopy(poly_nc_inters);
                poly_nc_ps = new Polygon();
                poly_nc_qt = new Polygon();

                tmp2 = poly_ps.head;
                do {
                    double x = tmp2.v.x - tmp.v.x;
                    double y = tmp2.v.y - tmp.v.y;
                    poly_nc_ps.InsertBeforeHead(new Vertex(x, y));
                    tmp2 = tmp2.next;
                } while (tmp2 != poly_ps.head);

                tmp2 = poly_qt.head;
                do {
                    double x = tmp.v.x + tmp2.v.x;
                    double y = tmp.v.y + tmp2.v.y;
                    poly_nc_qt.InsertBeforeHead(new Vertex(x, y));
                    tmp2 = tmp2.next;
                } while (tmp2 != poly_qt.head);

                poly_nc_inters = poly_nc_inters.intersects(poly_nc_ps);
                if (poly_nc_inters.size() > 0) {
                    poly_nc_inters = poly_nc_inters.intersects(poly_nc_qt);
                }
                if (poly_nc_inters.size() > 0) {
                    mb = tmp.v;
                    nc = poly_nc_inters.getGravity();
                    break;
                }
            }

            m = mb.x;
            b = mb.y;
            n = nc.x;
            c = nc.y;

            System.out.println("recheck: " + iStart);

            if (!poly_mb.isIn(mb) || !poly_nc.isIn(nc)) {
                System.out.println("id: " + iStart);
                notin++;
//				notin+= (iEnd - iStart);
            }

        }

        double x_start = b;
        double x_end = m * (end.time - start.time) + b;

        double y_start = c;
        double y_end = n * (end.time - start.time) + c;

        double dist = Math.sqrt(x_start * x_start + y_start * y_start);
        double theta = (Math.atan2(y_start, x_start) + Math.PI * 2) % (Math.PI * 2);
        GPSPoint newStart = DisCalculator.getEndPoint(start, dist, theta, start.time, iStart);

        dist = Math.sqrt(x_end * x_end + y_end * y_end);
        theta = (Math.atan2(y_end, x_end) + Math.PI * 2) % (Math.PI * 2);
        GPSPoint newEnd = DisCalculator.getEndPoint(start, dist, theta, end.time, iEnd);


        putLine(v, newStart, newEnd, lineId);
    }


    /**
     * 给定当前的两个点，求其约束对应的初始平行四边形
     * @param k
     * @param t
     * @return
     */
    private Polygon getInitial(double lb_eq, double up_eq, double lb_ieq, double up_ieq, long t) {
        Polygon res = new Polygon();
        Point tmp = new Point();
        List<Point> points = new LinkedList<Point>();


        double x = (lb_ieq - lb_eq) / t;
        double y = lb_eq;
        tmp = new Point(x, y);
        points.add(tmp);

        x = (up_ieq - lb_eq) / t;
        y = lb_eq;
        tmp = new Point(x, y);
        points.add(tmp);

        x = (up_ieq - up_eq) / t;
        y = up_eq;
        tmp = new Point(x, y);
        points.add(tmp);

        x = (lb_ieq - up_eq) / t;
        y = up_eq;
        tmp = new Point(x, y);
        points.add(tmp);

        res = new Polygon(points);
        return res;

    }


    /**
     * 根据起点，终点，标准平面上的多边形交集计算虚拟终点
     * @param start
     * @param end
     * @param inters
     * @return
     */
    private GPSPoint getDest(GPSPoint start, GPSPoint end, Polygon inters) {
        long time = (end.time - start.time);
        long t0 = end.time - start.time;

        double t = 1000.0; // 标准平面在t处
        double k = (t0) / (t);

        Point g = inters.getGravity();
        g.x *= k;
        g.y *= k;

        double dist = Math.sqrt(g.x * g.x + g.y * g.y);
        double theta = (Math.atan2(g.y, g.x) + Math.PI * 2) % (Math.PI * 2);
//		if(Math.abs(end.longitude  - start.longitude) > 180){
//			theta = 2*Math.PI  - theta;
//		}
        GPSPoint res = DisCalculator.getEndPoint(start, dist, theta, end);

        // debug
        dist = DisCalculator.getDistanceOfP2P(start, end);
        theta = DisCalculator.getAngleOfVector(start.latitude, start.longitude, end.latitude, end.longitude);
        res = DisCalculator.getEndPoint(start, dist, theta, end);

        dist = new SED().getDistanceOfP2Line(start, res, end);
//		System.out.println(dist);
        return res;

    }


    /**
     * 根据当前gps点，计算它在x,y,t三维空间中对应的圆的近似多边形
     * @param p
     * @param start
     * @return
     */
    private Polygon getPolyGon(GPSPoint p, GPSPoint start, double radius) {
        if (this.getDc() instanceof SED) {

            long t0 = p.time - start.time;

            double t = 1000.0; // 标准平面在t处
            double k = (t0) / (t);

            List<Point> points = new LinkedList<Point>();

            // 先投射到标准平面

            // 标准平面的半径
            radius = radius / (k);
//			System.out.println("radius: "+radius);

            Point point = new Point(p, start); // 根据当前点，以起点为坐标原点，计算当前点对应的x-y平面坐标

            // 标准平面的圆心
            point.x /= k;
            point.y /= k;


            // 采用八边形近似


            for (int i = 0; i < m; i++) {
                double x = point.x + this.deltX[i] * radius;
                double y = point.y + this.deltY[i] * radius;
                Point tmp = new Point(x, y);
                points.add(tmp);
            }


            Polygon res = new Polygon(points, point);
            return res;
        } else if (this.getDc() instanceof DED) {
            return null;
        }
        return null;
    }

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "CISED-E";

    }

    public static void main(String[] args) {

        System.out.println(Math.atan2(-1, -1));
        int npoints = 0;
        int nlines = 0;

        long timeUsed = 0;
        DisCalculator ded = new DED();
        DisCalculator ped = new PED();
        DisCalculator sed = new SED();
        int errorPoints;
        for (int zeta = 40; zeta <= 40; zeta += 45) { // 误差阈值
            npoints = 0;
            nlines = 0;

            CISED_E tc = new CISED_E(8);
            tc.threshold = zeta; // 阈值，单位为米。
            tc.radialThreshold = 1.0 * zeta;
//			tc.deviation = 13; // 判断掉头的阈值，单位为米。

            System.out.println("Threshold : " + tc.threshold + " m.");
            System.out.println("radialThreshold : " + tc.radialThreshold + " m.");
            GPSLine[] lines = null;
            for (int i = 85; i <= 85; i++) {
                // tc.strFileName = ".\\data\\test.txt";//轨迹文件
                tc.strFileName = ".\\data\\geolife\\0" + i + ".txt";// 轨迹文件
                // tc.strFileName = ".\\data\\taxi\\" + i + ".txt";//轨迹文件
                // tc.strFileName = ".\\data\\truck\\" + i + ".txt";//轨迹文件
                tc.strFileName = "traj/taxi/new/" + i + ".txt";// 轨迹文件

//				tc.strFileName = "traj/truck/short/" + i + ".txt";// 轨迹文件

//				tc.strFileName = "traj/geolife/new/increase2.txt";
//				String fileName = "traj/geolife/new/increase.txt";
                tc.strFileName = "traj/geolife/new/" + i + ".txt";// 轨迹文件
                tc.strFileName = "d:/ACT/trajData/SerCar/CleanData/" + i + ".txt";
//				tc.strFileName = "d:/ACT/trajData/truck/CleanData/"+i+".txt";
                tc.strFileName = "d:/ACT/trajData/geolife/CleanData/" + i + ".txt";
//				 tc.strFileName = "traj/taxi/new/short.txt";// 轨迹文件

                tc.loadTrajectory();
                System.out.print("" + i);
                System.out.println(".	Read lines (GPS Points) :" + tc.iTrajectory.size());
                npoints = npoints + tc.iTrajectory.size();

                double stime = System.currentTimeMillis();
                lines = tc.compress(); // 算法：n
                double etime = System.currentTimeMillis();

                System.out.println("Time:" + (etime - stime));
                timeUsed += (etime - stime);
                System.out.println("Compress By Sleeve to : (" + lines.length + ") lines.");
                tc.printToConsole(tc.iTrajectory, lines);

                nlines = nlines + lines.length;


            }

            System.out.println();
            System.out.println("Compress Ratio = " + (1.0 * nlines) / (1.0 * npoints) * 100 + "%.");
            System.out.println("Time Used = " + timeUsed + "ms");
        }
    }
}


