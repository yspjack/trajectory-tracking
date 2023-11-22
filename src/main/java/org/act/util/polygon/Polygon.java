package org.act.util.polygon;

import java.util.LinkedList;
import java.util.List;

public class Polygon {

    public static double timeCandidates = 0;
    public static double timeIntersects = 0;

    int n; // 0 means empty; 1 means one vertex; etc.
    public Vertex head;

    public double minx, miny, maxx, maxy;
    public double x0, y0;
    public Vertex PointMinx, PointMaxx;
    private static int aa, ba; /* # advances on a & b indices (after 1st inter.) */
    private static final int aAdvance = 0, bAdvance = 1;

    public Polygon() {
        head = null;
        n = 0;
        minx = Double.POSITIVE_INFINITY;
        miny = Double.POSITIVE_INFINITY;
        maxx = Double.NEGATIVE_INFINITY;
        maxy = Double.NEGATIVE_INFINITY;
        PointMaxx = null;
        PointMinx = null;
    }

    /**
     * 用于输出连续线段的正八边形，边界值可以计算得出
     * @param points
     * @param point
     */
    public Polygon(List<Point> points, Point point) {
        this();

        for (Point tmp : points) {
            Vertex vertex = new Vertex(tmp.x, tmp.y);
            this.InsertBeforeHead(vertex);
        }
    }

    /**
     * 用于输出不连续线段的多边形，需要记录左右边界点
     * 使用此构造器的是生产初始平行四边形时
     * @param points
     */
    public Polygon(List<Point> points) {
        this();
        for (Point tmp : points) {
            Vertex vertex = new Vertex(tmp.x, tmp.y);
            this.InsertBeforeHead(vertex);
        }

        PointMinx = head;
        minx = head.v.x;
        maxy = head.v.y;
        //
        PointMaxx = head.next.next;
        maxx = PointMaxx.v.x;
        miny = PointMaxx.v.y;
    }

    public int size() {
        return n;
    }

    public void InitHead(Vertex h) {
        head = new Vertex();
        head = h;
        head.next = head.prev = head;
        n = 1;

        if (minx > head.v.x) {
            PointMinx = head;
        }
        minx = (minx < head.v.x) ? minx : head.v.x;
        miny = (miny < head.v.y) ? miny : head.v.y;
        if (maxx < head.v.x) {
            PointMaxx = head;
        }
        maxx = (maxx > head.v.x) ? maxx : head.v.x;
        maxy = (maxy > head.v.y) ? maxy : head.v.y;
    }

    /*
     * Inserts newV before oldV
     */
    public void InsertBeforeHead(Vertex ver) {
        if (head == null)
            InitHead(ver);
        else {
            InsertBefore(ver, head);
        }
    }

    public void InsertBefore(Vertex newV, Vertex oldV) {
        if (head == null)
            InitHead(newV);
        else {
            oldV.prev.next = newV;
            newV.prev = oldV.prev;
            newV.next = oldV;
            oldV.prev = newV;
            n++;
        }

        if (minx > newV.v.x) {
            PointMinx = newV;
        }
        minx = (minx < newV.v.x) ? minx : newV.v.x;
        miny = (miny < newV.v.y) ? miny : newV.v.y;
        if (maxx < newV.v.x) {
            PointMaxx = newV;
        }
        maxx = (maxx > newV.v.x) ? maxx : newV.v.x;
        maxy = (maxy > newV.v.y) ? maxy : newV.v.y;
    }

    public void SetVertex(int x, int y) {
        Vertex v = new Vertex(x, y);
        InsertBeforeHead(v);
    }

    /*
     * Makes a copy of present list
     */
    public void ListCopy(Polygon list) {
        Vertex temp1 = head, temp2;
        do {
            temp2 = new Vertex(); // Create a new vertex cell
            temp2.v = temp1.v; // Fill it with the same Point as in list
            temp2.mark = temp1.mark;
            temp2.ear = temp1.ear;
            // temp2.duplicate = temp1.duplicate;
            temp2.onhull = temp1.onhull;
            temp2.vnum = temp1.vnum;
            list.InsertBeforeHead(temp2);
            if (temp1 == PointMaxx) {
                list.PointMaxx = temp2;
            }

            if (temp1 == PointMinx) {
                list.PointMinx = temp2;
            }
            temp1 = temp1.next;
        } while (temp1 != head);

    }

    public Polygon getInverse() {
        Polygon list = new Polygon();
        Vertex temp1 = head, temp2;
        do {
            temp2 = new Vertex(); // Create a new vertex cell
            temp2.v = temp1.v; // Fill it with the same Point as in list
            temp2.v.x = -temp2.v.x;
            temp2.v.y = -temp2.v.y;

            temp2.mark = temp1.mark;
            temp2.ear = temp1.ear;
            // temp2.duplicate = temp1.duplicate;
            temp2.onhull = temp1.onhull;
            temp2.vnum = temp1.vnum;
            list.InsertBeforeHead(temp2);
            temp1 = temp1.next;
        } while (temp1 != head);

        return list;

    }

    /**
     * 求参数空间多边形与新加入的两个半平面约束的交集
     * 半平面约束用直线的斜率和截距表示
     * b >= -t*m + alpha
     * b <= -t*m + omega
     * @param k
     * @param b
     * @return
     */
    public Polygon intersects(long t, double alpha, double omega) {
        Vertex lowerRight = new Vertex(), upperRight = new Vertex(), lowerLeft = new Vertex(), upperLeft = new Vertex();

        Vertex lowerRight_in = new Vertex(), upperRight_in = new Vertex(), lowerLeft_in = new Vertex(),
                upperLeft_in = new Vertex();

        boolean notClip = false;

        Polygon res = new Polygon();

        // 先判断不相交的情况：
        // 首先判断右下角点是否落在约束左边  b >= -t*m + alpha
        double x = 0;
        double y = alpha;
        Point a = new Point(x, y);

        x = -1;
        y = t + alpha;
        Point b = new Point(x, y);
        Vertex c = this.PointMaxx; // 右下角点
        double cHB = a.AreaSign(a, b, c.v); // ab x ac的符号

        if (cHB > 0) {
            return res;
        }


        // 再判断左上角点是否落在约束右边 b <= -t*m + omega
        x = 0;
        y = omega;
        a = new Point(x, y);
        x = -1;
        y = t + omega;
        b = new Point(x, y);
        c = this.PointMinx; // 左上角点
        cHB = a.AreaSign(a, b, c.v); // ab x ac的符号
        // 不相交
        if (cHB < 0) {
            return res;
        }

        // 先找下半平面和多边形的交集       			b <= -t*m + omega
        //(a.)先判断多边形的右下角点是否落在半平面内
        // 用向量的叉乘积来判断

        // 如果右下角点落在半平面内，则整个多边形落在半平面内，交集不变
        c = this.PointMaxx;
        cHB = a.AreaSign(a, b, c.v); // ab x ac的符号
        if (cHB >= 0) {
            notClip = true;
            Vertex tmp = new Vertex(this.PointMaxx);
            res.InsertBeforeHead(tmp);
            lowerRight_in = this.PointMaxx.prev;
            upperRight_in = this.PointMaxx.next;
        }
        // 否则，需要逐条边找交点
        if (!notClip) {
            // 从右下角点的上一个点沿下半边开始找，直到找到一个点落在半平面内
            c = this.PointMaxx.prev;
            while (c != this.PointMinx.prev) {
                cHB = a.AreaSign(a, b, c.v); // ab x ac�ķ���
                // �����ڰ�ƽ����
                if (cHB > 0) {
                    // 求交点。
                    // 此交点为新多边形的右下角点
                    lowerRight.v = Point.lineIntersects(c.next.v, c.v, -t, omega);
                    res.InsertBeforeHead(lowerRight);
                    // lowerRight.prev = c;
                    lowerRight_in = c;
                    break;
                }
                c = c.prev;
            }

            // 从右下角点的下一个点沿上半边开始找，直到找到一个点落在半平面内
            c = this.PointMaxx.next;
            while (c != this.PointMinx.next) {
                cHB = a.AreaSign(a, b, c.v); // ab x ac�ķ���
                // �����ڰ�ƽ����
                if (cHB > 0) {
                    // 求交点。
                    // 此交点为新多边形的右上角点，upperRight

                    upperRight.v = Point.lineIntersects(c.prev.v, c.v, -t, omega);
                    res.InsertBeforeHead(upperRight);
                    // upperRight.next = c;
                    upperRight_in = c;

                    break;
                }
                c = c.next;
            }

        }

        notClip = false;
        // 再找上半平面和多边形的交集 				b >= -t*m + alpha
        //(a.)先判断多边形的左上角点是否落在半平面内
        // 用向量的叉乘积来判断
        x = 0;
        y = alpha;
        a = new Point(x, y);
        x = -1;
        y = t + alpha;
        b = new Point(x, y);
        c = this.PointMinx; // 左上角点

        cHB = a.AreaSign(a, b, c.v); // ab x ac的符号

        // 如果左上角点落在半平面内，则整个多边形落在半平面内，交集不变
        if (cHB <= 0) {
            notClip = true;
            for (Vertex tmp = upperRight_in; tmp != lowerRight_in.next; tmp = tmp.next) {
                res.InsertBeforeHead(new Vertex(tmp));
            }
            // res = this;
        }

        // 否则，需要逐条边找交点
        if (!notClip) {

            // 从左上角点的上一个点沿上半边开始找，直到找到一个点落在半平面内
            c = this.PointMinx.prev;
            while (c != this.PointMaxx.prev) {
                cHB = a.AreaSign(a, b, c.v); // ab x ac�ķ���
                // �����ڰ�ƽ����
                if (cHB < 0) {
                    // 求交点。
                    // 此交点为新多边形的左上角点，upperLeft

                    upperLeft.v = Point.lineIntersects(c.next.v, c.v, -t, alpha);
                    if (c == upperRight_in.prev) {
                        res.InsertBeforeHead(upperLeft);
                    } else {
                        for (Vertex tmp = upperRight_in; tmp != c.next; tmp = tmp.next) {

                            res.InsertBeforeHead(new Vertex(tmp));
                        }
                        res.InsertBeforeHead(upperLeft);
                    }
                    break;
                }
                c = c.prev;
            }

            // 从左上角点的下一个点沿上半边开始找，直到找到一个点落在半平面内
            c = this.PointMinx.next;
            while (c != this.PointMaxx.next) {
                cHB = a.AreaSign(a, b, c.v); // ab x ac�ķ���
                // �����ڰ�ƽ����
                if (cHB < 0) {
                    // 求交点。
                    // 此交点为新多边形的左下角点，lowerLeft

                    lowerLeft.v = Point.lineIntersects(c.prev.v, c.v, -t, alpha);
                    res.InsertBeforeHead(lowerLeft);
                    if (c != lowerRight_in.next) {
                        for (Vertex tmp = c; tmp != lowerRight_in.next; tmp = tmp.next) {
                            res.InsertBeforeHead(new Vertex(tmp));
                        }
                    }
                    break;
                }
                c = c.next;
            }

        }


        // 如果多边形整个落在两个半平面内，交集不变


        return res;
    }

    /**
     * 计算多边形的重心
     * @return
     */
    public Point getGravity() {
        Point point = new Point();
        Vertex tmp = this.head;
        double x = 0;
        double y = 0;
        do {
            x += tmp.v.x;
            y += tmp.v.y;
            tmp = tmp.next;
        } while (tmp != this.head);
        x /= this.size();
        y /= this.size();
        point = new Point(x, y);
        return point;
    }

    /**
     * 判断点是否在多边形内部
     * @param a
     * @return
     */
    public boolean isIn(Point a) {

        Vertex b = this.head;
        Vertex b1 = b.prev;
        boolean notQinP = false;

        double aHB = 0;
        do {
            // cycles++;
            aHB = b1.v.AreaSign(b1.v, b.v, a); // bxa
            if (aHB < 0) { // Q�ĵ㲻��P�ڣ���Q����P������
                notQinP = true;
                return false;
            } else {
                b = b.next;
                b1 = b.prev;
            }
        } while (b != this.head);

        return true;
    }

    /**
     * 求两个多边形的交集
     * @param Q
     * @return
     */
    public Polygon intersects(Polygon Q) {
        // Polygon p = new Polygon();
        // this.ListCopy(p);
        // Polygon q = new Polygon();
        // Q.ListCopy(q);
        // int n = p.n;
        // int m = q.n;
        //
        // Polygon res = ConvexIntersect(p, q, n, m);
        Polygon res = ConvexIntersect(this, Q, this.n, Q.n);
        aa = 0;
        ba = 0;
        return res;
    }

    public static Polygon ConvexIntersect(Polygon P, Polygon Q, int n, int m)
        /* P has n vertices, Q has m vertices. */ {
        Polygon inters = new Polygon(); /* intersection of the two polygons */
        if (P.size() == 0 || Q.size() == 0) {
            return inters;
        }

        Vertex a, b; /* indices on P and Q (resp.) */
        Point A, B; /* directed edges on P and Q (resp.) */
        int cross; /* sign of z-component of A x B */
        int bHA, aHB; /* b in H(A); a in H(b). */
        Point Origin; /* (0,0) */
        Point p; /* double point of intersection */
        Point q; /* second point of intersection */
        cInFlag inflag; /* {Pin, Qin, Unknown}: which inside */

        // double minpx = Double.POSITIVE_INFINITY,maxpx =
        // Double.NEGATIVE_INFINITY,minpy = Double.POSITIVE_INFINITY,maxpy =
        // Double.NEGATIVE_INFINITY;
        // double minqx = Double.POSITIVE_INFINITY,maxqx =
        // Double.NEGATIVE_INFINITY,minqy = Double.POSITIVE_INFINITY,maxqy =
        // Double.NEGATIVE_INFINITY;
        boolean FirstPoint; /*
         * Is this the first point? (used to initialize).
         */
        Point p0; /* The first point. */
        int code; /* SegSegInt return code. */
        boolean intersection = true;

        /* Initialize variables. */

        a = P.head;
        b = Q.head;
        aa = ba = 0;
        Origin = new Point(); /* (0,0) */
        inflag = new cInFlag();
        FirstPoint = true;
        Vertex a1, b1;
        A = new Point();
        B = new Point();
        p = new Point();
        q = new Point();
        p0 = new Point();

        // 不相交的情况
        // if(P.minx > Q.maxx || P.miny > Q.maxy || P.maxx < Q.minx || P.maxy <
        // Q.miny){
        // return inters;
        // }

        do {
            /*
             * System.out.println("Before Advances:a="+a.v.x+","+a.v.y+
             * ", b="+b.v.x+","+b.v.y+"; aa="+aa+", ba="+ba+"; inflag="+ inflag.f);
             */
            /* Computations of key variables. */
            a1 = a.prev;
            b1 = b.prev;

            // SubVec(a.v, a1.v, A);
            A.x = a.v.x - a1.v.x;
            A.y = a.v.y - a1.v.y;
            // SubVec(b.v, b1.v, B);
            B.x = b.v.x - b1.v.x;
            B.y = b.v.y - b1.v.y;

            cross = Origin.AreaSign(Origin, A, B);
            aHB = b1.v.AreaSign(b1.v, b.v, a.v);
            bHA = a1.v.AreaSign(a1.v, a.v, b.v);
            // System.out.println("cross=" + cross + ", aHB=" + aHB + ", bHA=" +
            // bHA);

            /* If A & B intersect, update inflag. */
            code = a1.v.SegSegInt(a1.v, a.v, b1.v, b.v, p, q);
            // System.out.println("SegSegInt: code = " + code);

            if (code == '1' || code == 'v') {
                if (inflag.f == inflag.Unknown && FirstPoint) {
                    aa = ba = 0;
                    FirstPoint = false;
                    p0.x = p.x;
                    p0.y = p.y;
                    // InsertInters(p0.x, p0.y, inters);

                }
                inflag = InOut(p, inflag, aHB, bHA, inters);
                // System.out.println("InOut sets inflag=" + inflag.f);
            }

            /*-----Advance rules-----*/

            /* Special case: A & B overlap and oppositely oriented. */
            if ((code == 'e') && (Dot(A, B) < 0)) {
                // InsertSharedSeg(p, q);
                InsertInters(p.x, p.y, inters);

                InsertInters(q.x, q.y, inters);

                return inters;
            }

            /* Special case: A & B parallel and separated. */
            if ((cross == 0) && (aHB < 0) && (bHA < 0)) {

                return inters;
            }

            /* Special case: A & B collinear. */
            else if ((cross == 0) && (aHB == 0) && (bHA == 0)) {
                /* Advance but do not output point. */
                if (inflag.f == inflag.Pin)
                    b = Advance(b, bAdvance, inflag.f == inflag.Qin, b.v, inters);
                else
                    a = Advance(a, aAdvance, inflag.f == inflag.Pin, a.v, inters);
            }

            /* Generic cases. */
            else if (cross >= 0) {
                if (bHA > 0)
                    a = Advance(a, aAdvance, inflag.f == inflag.Pin, a.v, inters);
                else
                    b = Advance(b, bAdvance, inflag.f == inflag.Qin, b.v, inters);
            } else /* if ( cross < 0 ) */ {
                if (aHB > 0)
                    b = Advance(b, bAdvance, inflag.f == inflag.Qin, b.v, inters);
                else
                    a = Advance(a, aAdvance, inflag.f == inflag.Pin, a.v, inters);
            }

            /*
             * Quit when both adv. indices have cycled, or one has cycled twice.
             */
            // cycles++;
        } while (((aa < n) || (ba < m)) && (aa < 2 * n) && (ba < 2 * m));

        /* Deal with special cases: not implemented. */

        // ������ϵ�ж�
        if (inflag.f == inflag.Unknown) {

            // ���ж�Q����P���ڲ������ⲿ

            a = Q.head;
            b = P.head;
            b1 = b.prev;
            boolean notQinP = false;

            do {
                aHB = b1.v.AreaSign(b1.v, b.v, a.v); // bxa
                if (aHB < 0) { // Q的点不在P内，则Q不被P所包含
                    notQinP = true;
                    break;
                    // return inters;
                } else {
                    b = b.next;
                    b1 = b.prev;
                }
            } while (b != P.head);

            // Q����P�ڣ����ж�P�Ƿ���Q��
            if (notQinP) {

                a = P.head;
                b = Q.head;
                b1 = b.prev;
                boolean notPinQ = false;

                do {
                    aHB = b1.v.AreaSign(b1.v, b.v, a.v);
                    if (aHB < 0) { // P的点不在Q内，则P不被Q所包含，不相交
                        notPinQ = true;

                        return inters;
                    } else {
                        b = b.next;
                        b1 = b.prev;
                    }
                } while (b != Q.head);

                return P;
            } else {
                return Q;
            }

        }

        return inters;
    }

    /*---------------------------------------------------------------------
    a - b ==> c.
    ---------------------------------------------------------------------*/
    private static void SubVec(Point a, Point b, Point c) {
        c.x = a.x - b.x;
        c.y = a.y - b.y;
    }

    /*---------------------------------------------------------------------
    Returns the dot product of the two input vectors.
    ---------------------------------------------------------------------*/
    private static double Dot(Point a, Point b) {
        int i;
        double sum = 0.0;

        sum = a.x * b.x + a.y * b.y;

        return sum;
    }

    /*---------------------------------------------------------------------
    Advances and prints out an inside vertex if appropriate.
    ---------------------------------------------------------------------*/
    private static Vertex Advance(Vertex a, int counter, boolean inside, Point v, Polygon inters) {
        if (inside)
            InsertInters(v.x, v.y, inters);
        if (counter == aAdvance)
            aa++;
        else if (counter == bAdvance)
            ba++;
        return a.next;
    }

    /*---------------------------------------------------------------------
    Prints out the double point of intersection, and toggles in/out flag.
    ---------------------------------------------------------------------*/
    static cInFlag InOut(Point p, cInFlag inflag, int aHB, int bHA, Polygon inters) {
        InsertInters(p.x, p.y, inters);

        /* Update inflag. */
        if (aHB > 0) {
            inflag.f = inflag.Pin;
            return inflag;
        } else if (bHA > 0) {
            inflag.f = inflag.Qin;
            return inflag;
        } else /* Keep status quo. */
            return inflag;
    }

    private static void InsertInters(double x, double y, Polygon inters) {
        Vertex v = new Vertex(x, y);
        inters.InsertBeforeHead(v);
    }

    public void Delete(Vertex ver) {
        if (head == head.next)
            head = null;
        else if (ver == head)
            head = head.next;

        ver.prev.next = ver.next;
        ver.next.prev = ver.prev;
        n--;

        // minx = (minx < ver.v.x)?minx:ver.v.x;
        // miny = (miny < ver.v.y)?miny:ver.v.y;
        // maxx = (maxx > ver.v.x)?maxx:ver.v.x;
        // maxy = (maxy > ver.v.y)?maxy:ver.v.y;

    }

    /*
     * Printing to the console:
     */
    public void PrintVertices() {
        Vertex temp = head;
        int i = 1;
        if (head != null) {
            do {
                temp.PrintVertex(i);
                temp = temp.next;
                i++;
            } while (temp != head);
        }
    }

    public static void main(String[] args) {

        testLineIntersectsPolygon();
        // testPolygonIntersects();
        testSpeed();

    }

    public static void testLineIntersectsPolygon() {
        Polygon p = new Polygon();

        Vertex tmp = new Vertex(-1, -1);
        p.InsertBeforeHead(tmp);
        tmp = new Vertex(3, -1);
        p.InsertBeforeHead(tmp);
        tmp = new Vertex(1, 1);
        p.InsertBeforeHead(tmp);
        tmp = new Vertex(-3, 1);
        p.InsertBeforeHead(tmp);
        System.out.println("polygon p:");
        p.PrintVertices();
        long t = 2;
        double a = -2;
        double b = 2;
        Polygon res = p.intersects(t, a, b);

        System.out.println("polygon res:");
        System.out.println(res.size());
        res.PrintVertices();

        System.out.println("jsdklfjsadkf");
        res = res.intersects(t, -1, 1);

        System.out.println("polygon res:");
        System.out.println(res.size());
        res.PrintVertices();
    }

    public static void testSpeed() {

        long timeIntersects = 0;

        Point point = new Point(0, 0); // ���ݵ�ǰ�㣬�����Ϊ����ԭ�㣬���㵱ǰ���Ӧ��x-yƽ������

        int m = 6;
        double[] deltX = new double[m];
        double[] deltY = new double[m];
        double radius = 15.0;
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }

        Polygon p = new Polygon();

        for (int i = 0; i < m; i++) {
            double x = point.x + deltX[i] * radius;
            double y = point.y + deltY[i] * radius;
            p.InsertBeforeHead(new Vertex(x, y));
        }

        point = new Point(3, 3); // ���ݵ�ǰ�㣬�����Ϊ����ԭ�㣬���㵱ǰ���Ӧ��x-yƽ������
        Polygon q = new Polygon();

        for (int i = 0; i < m; i++) {
            double x = point.x + deltX[i] * radius;
            double y = point.y + deltY[i] * radius;
            q.InsertBeforeHead(new Vertex(x, y));
        }

        Polygon res = new Polygon();

        long sTime = System.nanoTime();
        for (int i = 0; i < 1000000; i++) {
            res = p.intersects(q);
        }
        timeIntersects += (System.nanoTime() - sTime);

        System.out.println("timeIntersects: " + timeIntersects / 1000000 + " ms");
    }

    public static void testPolygonIntersects() {
        Polygon p = new Polygon();
        Polygon q = new Polygon();

        // Vertex tmp = new Vertex(-2, 0);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(-1.41, -1.41);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(0, -2);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(1.41, -1.41);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(2, 0);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(1.41, 1.41);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(0, 2);
        // p.InsertBeforeHead(tmp);
        // tmp = new Vertex(-1.41, 1.41);
        // p.InsertBeforeHead(tmp);

        List<Point> pList = new LinkedList<>();
        pList.add(new Point(0, 2));
        pList.add(new Point(0, -1));
        pList.add(new Point(2, -1));
        pList.add(new Point(2, 2));

        p = new Polygon(pList);
        System.out.println("polyGon p:");
        p.PrintVertices();

        List<Point> qList = new LinkedList<>();

        qList.add(new Point(-1, 1));
        qList.add(new Point(-1, 0));
        qList.add(new Point(1, 0));
        qList.add(new Point(1, 1));
        q = new Polygon(qList);

        System.out.println("polyGon q:");
        q.PrintVertices();

        Polygon res = p.intersects(q);

        System.out.println("p intersects q" + res.n);

        res.PrintVertices();
        if (res.n == 0) {

            System.out.println("null");
        }

        res = q.intersects(p);

        System.out.println("q intersects p" + res.n);

        res.PrintVertices();
        if (res.n == 0) {

            System.out.println("null");
        }
        // tmp.PrintVertex();

        // conv.inters.PrintVertices();

    }

}

