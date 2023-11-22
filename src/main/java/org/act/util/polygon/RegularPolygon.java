package org.act.util.polygon;

import org.act.util.GPSPoint;

import java.util.LinkedList;
import java.util.List;

public class RegularPolygon {
    private static final int BADVANCE = 1, AADVANCE = 0;

    public static double timeSegsegint = 0, timeAreasign = 0, timeAdvance = 0, timeInsects = 0;

    public static final int PCONTAINSQ = 1, QCONTAINSP = 2, DISINTERSECT = 3, INTERSECTS = 4, UNKNOWN = 5;

    /* # advances on a & b indices (after 1st inter.) */
    private int aa, ba;

    private static Point Origin = new Point();

    public int n; // 0 means empty; 1 means one vertex; etc.
    int M;

    public IndexedVertex[] vertices;
    public int head;

    public double minx, miny, maxx, maxy;

    public RegularPolygon(int m) {

        M = m;
        head = -1;
        n = 0;
        minx = Double.POSITIVE_INFINITY;
        miny = Double.POSITIVE_INFINITY;
        maxx = Double.NEGATIVE_INFINITY;
        maxy = Double.NEGATIVE_INFINITY;

        vertices = new IndexedVertex[m];

        for (int i = 0; i < m; i++) {
            IndexedVertex tmp = new IndexedVertex();
            vertices[i] = tmp;
        }
    }

    public RegularPolygon(List<Point> points, int m) {
        this(m);
        for (int i = 0; i < m; i++) {
            Point tmp = points.get(i);
            setVertex(tmp.x, tmp.y, i);
        }
    }

    public RegularPolygon copy() {

        RegularPolygon res = new RegularPolygon(M);
        res.head = head;
        if (head != -1) {
            IndexedVertex temp = res.vertices[head];
            IndexedVertex temp2 = vertices[head];
            do {
                res.setVertex(temp2.x, temp2.y, temp2.index);
                temp = res.vertices[temp.next];
                temp2 = vertices[temp2.next];
            } while (temp2.index != head);
        }
        return res;
    }

    /**
     *
     *
     * @param x
     * @param y
     * @param index
     */
    public void setVertex(double x, double y, int index) {

        vertices[index].setIndex(x, y, index);
        // vertices[index].flag = true;

        if (head == -1) {
            head = index;
            vertices[head].prev = index;
            vertices[head].next = index;

        } else {
            IndexedVertex hVertex = vertices[head];
            IndexedVertex newV = vertices[index];
            vertices[hVertex.prev].next = index;
            newV.prev = hVertex.prev;
            newV.next = hVertex.index;
            hVertex.prev = index;
        }

        minx = (minx < x) ? minx : x;
        miny = (miny < y) ? miny : y;

        maxx = (maxx > x) ? maxx : x;
        maxy = (maxy > y) ? maxy : y;
        n++;
    }

    /**
     *
     *
     * @param tc
     * @param p0
     * @param radius
     */

    public void getPolygon(double[] deltaX, double[] deltaY, Point p0, double radius) {
        n = M;
        head = 0;
        minx = Double.POSITIVE_INFINITY;
        miny = Double.POSITIVE_INFINITY;
        maxx = Double.NEGATIVE_INFINITY;
        maxy = Double.NEGATIVE_INFINITY;

        double x = p0.x + deltaX[0] * radius;
        double y = p0.y + deltaY[0] * radius;
        vertices[0].setIndex(x, y, 0);
        vertices[0].next = 1;
        vertices[0].prev = M - 1;
        minx = (minx < x) ? minx : x;
        miny = (miny < y) ? miny : y;

        maxx = (maxx > x) ? maxx : x;
        maxy = (maxy > y) ? maxy : y;

        for (int i = 1; i < M - 1; i++) {
            x = p0.x + deltaX[i] * radius;
            y = p0.y + deltaY[i] * radius;
            vertices[i].setIndex(x, y, i);
            vertices[i].next = i + 1;
            vertices[i].prev = i - 1;
            minx = (minx < x) ? minx : x;
            miny = (miny < y) ? miny : y;

            maxx = (maxx > x) ? maxx : x;
            maxy = (maxy > y) ? maxy : y;
        }

        x = p0.x + deltaX[M - 1] * radius;
        y = p0.y + deltaY[M - 1] * radius;
        vertices[M - 1].setIndex(x, y, M - 1);
        vertices[M - 1].next = 0;
        vertices[M - 1].prev = M - 2;
        minx = (minx < x) ? minx : x;
        miny = (miny < y) ? miny : y;

        maxx = (maxx > x) ? maxx : x;
        maxy = (maxy > y) ? maxy : y;

    }

    /**
     * 閿熺獤鑺傜鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
     */
    public void clean() {

        head = -1;
        n = 0;
        minx = Double.POSITIVE_INFINITY;
        miny = Double.POSITIVE_INFINITY;
        maxx = Double.NEGATIVE_INFINITY;
        maxy = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < M; i++) {
            vertices[i].flag = false;
        }

    }

    /**
     *
     */
    public void reset() {
        head = -1;
        n = 0;
        minx = Double.POSITIVE_INFINITY;
        miny = Double.POSITIVE_INFINITY;
        maxx = Double.NEGATIVE_INFINITY;
        maxy = Double.NEGATIVE_INFINITY;

    }

    public int size() {
        return n;
    }

    /**
     * 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹钄氶敓鏂ゆ嫹閿熸枻鎷烽敓锟�
     *
     * @return
     */
    public Point getGravity() {
        Point point = new Point();
        double x = 0, y = 0;

        for (IndexedVertex tmp : vertices) {
            if (tmp.flag) {
                x += tmp.x;
                y += tmp.y;
            }
        }

        x /= this.size();
        y /= this.size();
        point = new Point(x, y);
        return point;
    }

    /**
     *
     *
     * @param Q
     * @return unknown, disintersect, intersect
     */
    public static int intersects_agg(RegularPolygon P, RegularPolygon Q, RegularPolygon res) {

        // long ssTime = System.nanoTime();
        /* intersection of the two polygons */
        RegularPolygon inters = res;

        res.clean();

        if (P.size() == 0 || Q.size() == 0) {
            return DISINTERSECT;
        }

        IndexedVertex a, b; /* indices on P and Q (resp.) */
        Point A, B; /* directed edges on P and Q (resp.) */

        int cross; /* sign of z-component of A x B */
        int bHA, aHB; /* b in H(A); a in H(b). */
        Point p; /* double point of intersection */
        Point q; /* second point of intersection */
        cInFlag inflag; /* {Pin, Qin, Unknown}: which inside */

        boolean FirstPoint; /*
         * Is this the first point? (used to initialize).
         */

        int code; /* SegSegInt return code. */

        /* Initialize variables. */
        P.aa = P.ba = 0;

        inflag = new cInFlag();
        FirstPoint = true;
        IndexedVertex a1, b1;
        A = new Point();
        B = new Point();
        p = new Point();
        q = new Point();
        a = P.vertices[P.head];
        b = Q.vertices[Q.head];

        // 閿熸枻鎷烽敓娲佷氦閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        if (P.minx > Q.maxx || P.miny > Q.maxy || P.maxx < Q.minx || P.maxy < Q.miny) {
            return DISINTERSECT;
        }

        do {

            a1 = P.vertices[a.prev];
            b1 = Q.vertices[b.prev];

            A.x = a.x - a1.x;
            A.y = a.y - a1.y;

            B.x = b.x - b1.x;
            B.y = b.y - b1.y;

            // long sTime = System.nanoTime();
            cross = Origin.AreaSign(Origin, A, B);
            aHB = b1.AreaSign(b1, b, a);
            bHA = a1.AreaSign(a1, a, b);
            // timeAreasign += (System.nanoTime() - sTime);

            // sTime = System.nanoTime();
            /* If A & B intersect, update inflag. */
            code = a1.SegSegInt(a1, a, b1, b, p, q);

            // timeSegsegint += (System.nanoTime() - sTime);

            // 閿熸磥浜�
            if (code == '1' || code == 'v') {
                if (inflag.f == inflag.Unknown && FirstPoint) {
                    P.aa = P.ba = 0;
                    FirstPoint = false;

                }

                // 閿熸枻鎷峰綍閿熶茎闈╂嫹閿熻妭纰夋嫹閿熸枻鎷烽敓鑺傝鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎻妸鏂ゆ嫹閿熷淇濋敓鏂ゆ嫹閿熻妭鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
                inflag = InOut_agg(a, b, p, inflag, aHB, bHA, inters);

            }

            /*-----Advance rules-----*/

            /* Special case: A & B overlap and oppositely oriented. */
            if ((code == 'e') && (Dot(A, B) < 0)) {
                // InsertSharedSeg(p, q);
                InsertInters(p.x, p.y, inters, a.index);
                InsertInters(q.x, q.y, inters, b.index);
                break;
                // return inters;
            }

            int index = 0;

            /* Special case: A & B parallel and separated. */
            if ((cross == 0) && (aHB < 0) && (bHA < 0)) {
                break;
                // return inters;
            }

            /* Special case: A & B collinear. */
            else if ((cross == 0) && (aHB == 0) && (bHA == 0)) {
                /* Advance but do not output point. */
                if (inflag.f == inflag.Pin) {
                    index = advance_agg(code, BADVANCE, inflag.f == inflag.Qin, inters, P, a, Q, b);

                    // rule 4
                    // a 涔熼敓鏂ゆ嫹鍓嶉敓鏂ゆ嫹涓�閿熸枻鎷�
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Qin && b.next == a.index) {
                        int intial = a.index;
                        a = P.vertices[a.next];
                        P.aa += (a.index + P.M - intial) % P.M;
                    }
                    b = Q.vertices[index];

                } else {
                    index = advance_agg(code, AADVANCE, inflag.f == inflag.Pin, inters, P, a, Q, b);

                    // b 涔熼敓鏂ゆ嫹鍓嶉敓鏂ゆ嫹涓�閿熸枻鎷�
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Pin && b.index == a.next) {
                        int intial = b.index;
                        b = Q.vertices[b.next];
                        P.ba += (b.index + P.M - intial) % P.M;
                    }
                    a = P.vertices[index];
                }
            }

            /* Generic cases. */
            else if (cross >= 0) {
                // sTime = System.nanoTime();
                if (bHA > 0) {
                    // a閿熸枻鎷峰墠閿熺鐨勬唻鎷烽敓锟�
                    index = advance_agg(code, AADVANCE, inflag.f == inflag.Pin, inters, P, a, Q, b);

                    // b 涔熼敓鏂ゆ嫹鍓嶉敓鏂ゆ嫹涓�閿熸枻鎷�
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Pin && b.index == a.next) {
                        int intial = b.index;
                        b = Q.vertices[b.next];
                        P.ba += (b.index + P.M - intial) % P.M;
                    }
                    // a閿熸枻鎷峰墠閿熸枻鎷�
                    a = P.vertices[index];

                } else {
                    index = advance_agg(code, BADVANCE, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Qin && b.next == a.index) {
                        int intial = a.index;
                        a = P.vertices[a.next];
                        P.aa += (a.index + P.M - intial) % P.M;
                    }
                    b = Q.vertices[index];
                }
                // timeAdvance += (System.nanoTime() - sTime);
            } else /* if ( cross < 0 ) */ {
                // sTime = System.nanoTime();
                if (aHB > 0) {
                    index = advance_agg(code, BADVANCE, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Qin && b.next == a.index) {
                        int intial = a.index;
                        a = P.vertices[a.next];
                        P.aa += (a.index + P.M - intial) % P.M;
                    }
                    b = Q.vertices[index];
                } else {
                    index = advance_agg(code, AADVANCE, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    // b 涔熼敓鏂ゆ嫹鍓嶉敓鏂ゆ嫹涓�閿熸枻鎷�
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Pin && b.index == a.next) {
                        int intial = b.index;
                        b = Q.vertices[b.next];
                        P.ba += (b.index + P.M - intial) % P.M;
                    }
                    a = P.vertices[index];
                }
                // timeAdvance += (System.nanoTime() - sTime);
            }

        } while (((P.aa < P.M) || (P.ba < P.M)) && (P.aa < 2 * P.M) && (P.ba < 2 * P.M));

        // can't decide whether P contains Q,Q contains P or they don't intersect.
        if (inflag.f == inflag.Unknown) {
            // the range of polygon P is larger than that of Q
            // the result may be PCONTAINSQ or DISINTERSECT.
            if (P.maxx - P.minx >= Q.maxx - Q.minx && P.maxy - P.miny >= Q.maxy - Q.miny) {
                a = Q.vertices[Q.head];
                return P.contains(a) ? PCONTAINSQ : DISINTERSECT;
            }
            // the range of polygon Q is larger than that of P
            // the result may be QCONTAINSP or DISINTERSECT.
            else if (P.maxx - P.minx <= Q.maxx - Q.minx && P.maxy - P.miny <= Q.maxy - Q.miny) {
                a = P.vertices[P.head];
                return Q.contains(a) ? QCONTAINSP : DISINTERSECT;
            }
            // otherwise, they do not intersect.
            else {
                return DISINTERSECT;
            }
        } else {
            // timeInsects += (System.nanoTime() - ssTime);
            return INTERSECTS;
        }
    }

    public static RegularPolygon intersects(RegularPolygon P, RegularPolygon Q) {
        RegularPolygon res = new RegularPolygon(P.M);
        int flag = intersects_agg(P, Q, res);
        // poly_i contains poly_k, result is poly_k
        // exchanges poly_k and polygon_intersects
        if (flag == RegularPolygon.PCONTAINSQ) {
            return Q.copy();
        }
        // poly_k contains poly_i, result is poly_i
        // exchanges poly_i and polygon_intersects
        // then exchanges poly_i and poly_k
        else if (flag == RegularPolygon.QCONTAINSP) {
            return P.copy();
        }
        return res;
    }

    /**
     *
     *
     * @param P
     * @param Q
     * @param res
     * @return PCONTAINSQ, QCONTAINSP, disintersect
     */
    public static int contains(RegularPolygon P, RegularPolygon Q, RegularPolygon res) {
        /* Deal with special cases: not implemented. */

        IndexedVertex a, b, a1, b1;
        int aHB, bHA;

        if (P.maxx - P.minx >= Q.maxx - Q.minx && P.maxy - P.miny >= Q.maxy - Q.miny) {

            a = Q.vertices[Q.head];
            b = P.vertices[P.head];
            b1 = P.vertices[b.prev];
            boolean notQinP = false;
            do {

                aHB = b1.AreaSign(b1, b, a); // bxa
                if (aHB < 0) { // Q閿熶茎鐐逛笉閿熸枻鎷稰閿熻妭锝忔嫹閿熸枻鎷稱閿熸枻鎷烽敓鏂ゆ嫹P閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
                    notQinP = true;
                    return DISINTERSECT;
                } else {
                    b = P.vertices[b.next];
                    b1 = P.vertices[b.prev];
                }
            } while (b.index != P.head);

            return PCONTAINSQ;
        }

        // Q閿熶茎鍑ゆ嫹鍥撮敓鏂ゆ嫹P閿熸枻鎷稱閿熷彨鍖℃嫹閿熸澃甯嫹閿熸枻鎷稰
        else if (P.maxx - P.minx <= Q.maxx - Q.minx && P.maxy - P.miny <= Q.maxy - Q.miny) {

            a = P.vertices[P.head];
            b = Q.vertices[Q.head];
            b1 = Q.vertices[b.prev];
            boolean notPinQ = false;

            do {
                aHB = b1.AreaSign(b1, b, a);
                if (aHB < 0) { // P閿熶茎鐐逛笉閿熸枻鎷稱閿熻妭锝忔嫹閿熸枻鎷稰閿熸枻鎷烽敓鏂ゆ嫹Q閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓娲佷氦
                    notPinQ = true;
                    return DISINTERSECT;
                } else {
                    b = Q.vertices[b.next];
                    b1 = Q.vertices[b.prev];
                }
            } while (b.index != Q.head);

            return QCONTAINSP;
        }

        // 閿熸枻鎷烽敓娲佷氦
        else {

            return DISINTERSECT;
        }

    }

    public boolean contains(GPSPoint start, GPSPoint end) {
        long t0 = end.time - start.time;
        double t = 1000.0; //
        double k = (t0) / (t);
        Point point = new Point();
        boolean flag = point.transform(end, start, 1e5);
        point.x /= k;
        point.y /= k;
        return flag && this.contains(point);
    }

    public boolean contains(Point point) {
        IndexedVertex a = new IndexedVertex();
        a.setXY(point.x, point.y);
        return this.contains(a);
    }

    public boolean contains(IndexedVertex vertex) {
        // System.out.println(this.vertices.length + " " + this.head);

        IndexedVertex b = this.vertices[this.head];
        IndexedVertex b1 = this.vertices[b.prev];
        int aHB = 0;
        do {
            aHB = b1.AreaSign(b1, b, vertex); // bxa
            if (aHB < 0) {
                return false;
            } else {
                b = this.vertices[b.next];
                b1 = this.vertices[b.prev];
            }
        } while (b.index != this.head);
        return true;
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

    /**
     *
     *
     * @param code
     * @param counter
     * @param inside
     * @param inters
     * @param P
     * @param a
     * @param Q
     * @param b
     * @return
     */
    private static int advance_agg(int code, int counter, boolean inside, RegularPolygon inters, RegularPolygon P,
                                   IndexedVertex a, RegularPolygon Q, IndexedVertex b) {

        if (inside) {
            if (counter == AADVANCE) {
                InsertInters(a.x, a.y, inters, a.index);
            } else {
                InsertInters(b.x, b.y, inters, b.index);
            }
        }

        int index = 0;

        // 閿熸磥浜�
        if (code == '1' || code == 'v') {

            int delta = (b.index + P.M - a.index) % P.M;

            // a瑕侀敓鏂ゆ嫹鍓嶉敓鏂ゆ嫹
            // P閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鍗遍敓鏂ゆ嫹閿熸枻鎷烽敓琛楁唻鎷烽敓鏂ゆ嫹閿熸枻鎷�
            if (counter == AADVANCE) {
                index = (a.index + 2 * delta) % P.M;
                // a = P.vertices[index];
                P.aa += 2 * delta;

            }
            // b瑕侀敓鏂ゆ嫹鍓嶉敓鏂ゆ嫹
            else if (counter == BADVANCE) {

                delta = (a.index + P.M - b.index) % P.M;
                int intial = b.index;
                index = (a.index + 1);
                P.ba += (delta + 1);
                for (int i = index; i != intial; ) {
                    i %= P.M;
                    if (Q.vertices[i].flag) {
                        index = i;
                        break;
                    }
                    i++;
                    P.ba++;
                    i %= P.M;

                }

            }
        }

        // 閿熸枻鎷烽敓娲佷氦閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        else {
            // rule 3
            if (counter == AADVANCE) {
                // inflag.f = inflag.Pin
                // a is inside

                int intial = a.index;
                index = a.next;
                P.aa += (index + P.M - intial) % P.M;

            }

            // rule 4
            else if (counter == BADVANCE) {

                // inflag.f = inflag.Pin
                // a is inside

                int intial = b.index;
                index = b.next;
                P.ba += (index + P.M - intial) % P.M;

            }

        }
        return index;
    }

    /*---------------------------------------------------------------------
    Prints out the double point of intersection, and toggles in/out flag.
    ---------------------------------------------------------------------*/
    static cInFlag InOut_agg(IndexedVertex a, IndexedVertex b, Point p, cInFlag inflag, int aHB, int bHA,
                             RegularPolygon inters) {

        /* Update inflag. */
        if (aHB > 0) {
            inflag.f = inflag.Pin;
            InsertInters(p.x, p.y, inters, b.index);
            // inters.vertices[indexQ] = new Vertex(p.x,p.y);

            return inflag;
        } else if (bHA > 0) {
            inflag.f = inflag.Qin;
            InsertInters(p.x, p.y, inters, a.index);
            return inflag;
        } else /* Keep status quo. */
            return inflag;
    }

    /**
     *
     *
     * @param P
     * @param Q
     * @param res 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
     * @return
     */
    public static int INTERSECTS(RegularPolygon P, RegularPolygon Q, RegularPolygon res) {

        RegularPolygon inters = res; /* intersection of the two polygons */
        res.clean();

        IndexedVertex a, b; /* indices on P and Q (resp.) */
        Point A, B; /* directed edges on P and Q (resp.) */
        int cross; /* sign of z-component of A x B */
        int bHA, aHB; /* b in H(A); a in H(b). */
        Point p; /* double point of intersection */
        Point q; /* second point of intersection */
        cInFlag inflag; /* {Pin, Qin, Unknown}: which inside */

        boolean FirstPoint; /*
         * Is this the first point? (used to initialize).
         */

        int code; /* SegSegInt return code. */
        boolean intersection = true;

        /* Initialize variables. */
        P.aa = P.ba = 0;

        inflag = new cInFlag();
        FirstPoint = true;
        IndexedVertex a1, b1;
        A = new Point();
        B = new Point();
        p = new Point();
        q = new Point();
        a = P.vertices[P.head];
        b = Q.vertices[Q.head];

        // 閿熸枻鎷烽敓娲佷氦閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        if (P.minx > Q.maxx || P.miny > Q.maxy || P.maxx < Q.minx || P.maxy < Q.miny) {
            // return inters;
            return DISINTERSECT;
        }

        do {

            a1 = P.vertices[a.prev];
            b1 = Q.vertices[b.prev];

            A.x = a.x - a1.x;
            A.y = a.y - a1.y;

            B.x = b.x - b1.x;
            B.y = b.y - b1.y;

            cross = Origin.AreaSign(Origin, A, B);
            aHB = b1.AreaSign(b1, b, a);
            bHA = a1.AreaSign(a1, a, b);

            /* If A & B intersect, update inflag. */
            code = a1.SegSegInt(a1, a, b1, b, p, q);

            if (code == '1' || code == 'v') {
                if (inflag.f == inflag.Unknown && FirstPoint) {
                    P.aa = P.ba = 0;
                    FirstPoint = false;

                }
                inflag = InOut(a, b, p, inflag, aHB, bHA, inters);

            }

            /*-----Advance rules-----*/

            /* Special case: A & B overlap and oppositely oriented. */
            if ((code == 'e') && (Dot(A, B) < 0)) {
                // InsertSharedSeg(p, q);
                InsertInters(p.x, p.y, inters, a.index);

                InsertInters(q.x, q.y, inters, b.index);
                break;
                // return inters;
            }

            int index = 0;

            /* Special case: A & B parallel and separated. */
            if ((cross == 0) && (aHB < 0) && (bHA < 0)) {
                break;
                // return inters;
            }

            /* Special case: A & B collinear. */
            else if ((cross == 0) && (aHB == 0) && (bHA == 0)) {
                /* Advance but do not output point. */
                if (inflag.f == inflag.Pin) {
                    index = advance(code, BADVANCE, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    b = Q.vertices[index];
                } else {
                    index = advance(code, AADVANCE, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    a = P.vertices[index];
                }
            }

            /* Generic cases. */
            else if (cross >= 0) {
                if (bHA > 0) {
                    index = advance(code, AADVANCE, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    a = P.vertices[index];
                } else {
                    index = advance(code, BADVANCE, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    b = Q.vertices[index];
                }
            } else /* if ( cross < 0 ) */ {
                if (aHB > 0) {
                    index = advance(code, BADVANCE, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    b = Q.vertices[index];
                } else {
                    index = advance(code, AADVANCE, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    a = P.vertices[index];
                }
            }

        } while (((P.aa < P.n) || (P.ba < Q.n)) && (P.aa < 2 * P.n) && (P.ba < 2 * Q.n));

        if (inflag.f == inflag.Unknown) {
            return UNKNOWN;
        } else {
            return INTERSECTS;
        }
    }

    static cInFlag InOut(IndexedVertex a, IndexedVertex b, Point p, cInFlag inflag, int aHB, int bHA,
                         RegularPolygon inters) {

        /* Update inflag. */
        if (aHB > 0) {
            inflag.f = inflag.Pin;
            InsertInters(p.x, p.y, inters, b.index);
            // inters.vertices[indexQ] = new Vertex(p.x,p.y);

            return inflag;
        } else if (bHA > 0) {
            inflag.f = inflag.Qin;
            InsertInters(p.x, p.y, inters, a.index);
            return inflag;
        } else /* Keep status quo. */
            return inflag;
    }

    private static int advance(int code, int counter, boolean inside, RegularPolygon inters, RegularPolygon P,
                               IndexedVertex a, RegularPolygon Q, IndexedVertex b) {

        if (inside) {
            if (counter == AADVANCE) {
                InsertInters(a.x, a.y, inters, a.index);
            } else {
                InsertInters(b.x, b.y, inters, b.index);
            }
        }

        int index = 0;
        if (counter == AADVANCE) {
            index = a.next;
            P.aa++;
        } else if (counter == BADVANCE) {
            index = b.next;
            P.ba++;
        }
        return index;
    }

    /**
     * 閿熺獤鑺傜鎷烽敓鏂ゆ嫹閿熻妭纰夋嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
     *
     * @param x
     * @param y
     * @param inters
     * @param index
     */
    private static void InsertInters(double x, double y, RegularPolygon inters, int index) {
        inters.setVertex(x, y, index);
    }

    /*
     * Printing to the console:
     */
    public void PrintVertices() {
        IndexedVertex temp = vertices[head];
        int i = 1;
        if (head != -1) {
            do {
                temp.PrintVertex(temp.index);
                temp = vertices[temp.next];
                i++;
            } while (temp.index != head);
        }

    }

    public static void main(String[] args) {

        // testLineIntersectsPolygon();
        // testPolygonIntersects();
        testSpeed();
    }

    public static void testSpeed() {

        long timeIntersects = 0;

        Point point = new Point(0, 0); // 閿熸枻鎷烽敓鎹风鎷峰墠閿熷锛岄敓鏂ゆ嫹閿熸枻鎷烽敓杞款亷鎷烽敓鏂ゆ嫹閿熺殕顓ㄦ嫹鎮栴剨鎷烽敓鏂ゆ嫹鎰曢蓟甯嫹閿熸枻鎷峰簲閿熸枻鎷穢-y骞抽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹

        int m = 6;
        double[] deltX = new double[m];
        double[] deltY = new double[m];
        double radius = 15.0;
        for (int i = 0; i < m; i++) {
            double angle = Math.PI / m * (2 * i + 1);
            deltX[i] = Math.cos(angle);
            deltY[i] = Math.sin(angle);
        }

        RegularPolygon p = new RegularPolygon(m);

        for (int i = 0; i < m; i++) {
            double x = point.x + deltX[i] * radius;
            double y = point.y + deltY[i] * radius;
            p.setVertex(x, y, i);
        }

        point = new Point(3, 3); // 閿熸枻鎷烽敓鎹风鎷峰墠閿熷锛岄敓鏂ゆ嫹閿熸枻鎷烽敓杞款亷鎷烽敓鏂ゆ嫹閿熺殕顓ㄦ嫹鎮栴剨鎷烽敓鏂ゆ嫹鎰曢蓟甯嫹閿熸枻鎷峰簲閿熸枻鎷穢-y骞抽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
        RegularPolygon q = new RegularPolygon(m);

        for (int i = 0; i < m; i++) {
            double x = point.x + deltX[i] * radius;
            double y = point.y + deltY[i] * radius;
            q.setVertex(x, y, i);
        }

        RegularPolygon res = new RegularPolygon(m);

        long sTime = System.nanoTime();
        for (int i = 0; i < 1000000; i++) {
            intersects_agg(p, q, res);
        }
        timeIntersects += (System.nanoTime() - sTime);

        System.out.println("timeIntersects: " + timeIntersects / 1000000);
    }

    public static void testPolygonIntersects() {
        RegularPolygon p = new RegularPolygon(6);
        RegularPolygon q = new RegularPolygon(6);

        List<Point> pList = new LinkedList<>();

        pList.add(new Point(-1.41, 1.41));
        pList.add(new Point(-2, 0));
        pList.add(new Point(-1.41, -1.41));
        pList.add(new Point(1.41, -1.41));
        pList.add(new Point(2, 0));
        pList.add(new Point(1.41, 1.41));

        // pList.add(new Point(-2, 2));
        // pList.add(new Point(-2, 0));
        // pList.add(new Point(0, 0));
        // pList.add(new Point(0, 2));

        p = new RegularPolygon(pList, 6);
        System.out.println("polyGon p:");
        p.PrintVertices();

        List<Point> qList = new LinkedList<>();

        qList.add(new Point(-2.82 + 5, 2.82));
        qList.add(new Point(-4 + 5, 0));
        qList.add(new Point(-2.82 + 5, -2.82));
        qList.add(new Point(1.41 * 2 + 5, -1.41 * 2));
        qList.add(new Point(2 * 2 + 5, 0));
        qList.add(new Point(1.41 * 2 + 5, 1.41 * 2));

        // qList.add(new Point(-1,1));
        // qList.add(new Point(-1,-1));
        // qList.add(new Point(1,-1));
        // qList.add(new Point(1,1));
        q = new RegularPolygon(qList, 6);

        System.out.println("polyGon q:");
        q.PrintVertices();

        RegularPolygon res = new RegularPolygon(6);
        INTERSECTS(p, q, res);

        System.out.println("p INTERSECTS q" + res.n);

        res.PrintVertices();
        if (res.n == 0) {

            System.out.println("null");
        }

        INTERSECTS(q, p, res);

        System.out.println("q INTERSECTS p" + res.n);

        res.PrintVertices();
        if (res.n == 0) {

            System.out.println("null");
        }

        pList = new LinkedList<>();
        pList.add(new Point(-1.41 + 1.5, 1.41 - 1.6));
        pList.add(new Point(-2 + 1.5, 0 - 1.6));
        pList.add(new Point(-1.41 + 1.5, -1.41 - 1.6));
        pList.add(new Point(1.41 + 1.5, -1.41 - 1.6));
        pList.add(new Point(2 + 1.5, 0 - 1.6));
        pList.add(new Point(1.41 + 1.5, 1.41 - 1.6));

        p = new RegularPolygon(pList, 6);

        System.out.println("polyGon p:");
        p.PrintVertices();

        // res = p.INTERSECTS(res);
        INTERSECTS(p, res, q);
        res = q;

        System.out.println("res INTERSECTS p " + res.n);

        res.PrintVertices();
        if (res.n == 0) {
            System.out.println("null");
        }
    }

}
