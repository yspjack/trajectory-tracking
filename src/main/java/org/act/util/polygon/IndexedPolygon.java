package org.act.util.polygon;

import java.util.LinkedList;
import java.util.List;

public class IndexedPolygon {
    private static final int bAdvance = 1, aAdvance = 0;
    public static final int PcontainsQ = 1, QcontainsP = 2, disIntersect = 3, intersects = 4, unKnown = 5;

    public static double timeCandidates = 0;
    public static double timeIntersects = 0;

    int n; // 0 means empty; 1 means one vertex; etc.
    static int M;
    // public Vertex head;
    public IndexedVertex[] vertices;

    public int head;

    public double minx, miny, maxx, maxy;

    private static int aa, ba; /* # advances on a & b indices (after 1st inter.) */
    private static Point Origin = new Point(); /* (0,0) */

    public IndexedPolygon(int m) {

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

    public IndexedPolygon(List<Point> points, int m) {
        this(m);
        for (int i = 0; i < m; i++) {
            Point tmp = points.get(i);
            setVertex(tmp.x, tmp.y, i);
        }
    }

    public void setVertex(double x, double y, int index) {
        vertices[index].setIndex(x, y, index);
        vertices[index].flag = true;

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

    public void copy(IndexedPolygon res) {
        res.head = head;
        res.maxx = maxx;
        res.minx = minx;
        res.maxy = maxy;
        res.miny = miny;

    }

    public int size() {
        return n;
    }

    /**
     * ����������εĽ��� �Ż��㷨
     *
     * @param Q
     * @return
     */
    public static int intersects_agg(IndexedPolygon P, IndexedPolygon Q, IndexedPolygon res) {

        IndexedPolygon inters = res; /* intersection of the two polygons */
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
        aa = ba = 0;

        inflag = new cInFlag();
        FirstPoint = true;
        IndexedVertex a1, b1;
        A = new Point();
        B = new Point();
        p = new Point();
        q = new Point();
        a = P.vertices[P.head];
        b = Q.vertices[Q.head];

        // 不相交的情况
        if (P.minx > Q.maxx || P.miny > Q.maxy || P.maxx < Q.minx || P.maxy < Q.miny) {
            // return inters;
            return disIntersect;
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
                    aa = ba = 0;
                    FirstPoint = false;

                }
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
                    index = advance_agg(code, bAdvance, inflag.f == inflag.Qin, inters, P, a, Q, b);

                    // rule 4
                    // a Ҳ��ǰ��һ��
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Qin && b.next == a.index) {
                        int intial = a.index;
                        a = P.vertices[a.next];
                        aa += (a.index + M - intial) % M;
                    }
                    b = Q.vertices[index];

                } else {
                    index = advance_agg(code, aAdvance, inflag.f == inflag.Pin, inters, P, a, Q, b);

                    // b Ҳ��ǰ��һ��
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Pin && b.index == a.next) {
                        int intial = b.index;
                        b = Q.vertices[b.next];
                        ba += (b.index + M - intial) % M;
                    }
                    a = P.vertices[index];
                }
            }

            /* Generic cases. */
            else if (cross >= 0) {
                if (bHA > 0) {
                    index = advance_agg(code, aAdvance, inflag.f == inflag.Pin, inters, P, a, Q, b);

                    // b Ҳ��ǰ��һ��
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Pin && b.index == a.next) {
                        int intial = b.index;
                        b = Q.vertices[b.next];
                        ba += (b.index + M - intial) % M;
                    }
                    a = P.vertices[index];

                } else {
                    index = advance_agg(code, bAdvance, inflag.f == inflag.Qin, inters, P, a, Q, b);

                    // a Ҳ��ǰ��һ��
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Qin && b.next == a.index) {
                        int intial = a.index;
                        a = P.vertices[a.next];
                        aa += (a.index + M - intial) % M;
                    }
                    b = Q.vertices[index];
                }
            } else /* if ( cross < 0 ) */ {
                if (aHB > 0) {
                    index = advance_agg(code, bAdvance, inflag.f == inflag.Qin, inters, P, a, Q, b);

                    // a Ҳ��ǰ��һ��
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Qin && b.next == a.index) {
                        int intial = a.index;
                        a = P.vertices[a.next];
                        aa += (a.index + M - intial) % M;
                    }
                    b = Q.vertices[index];
                } else {
                    index = advance_agg(code, aAdvance, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    // b Ҳ��ǰ��һ��
                    if (!(code == '1' || code == 'v') && inflag.f == inflag.Pin && b.index == a.next) {
                        int intial = b.index;
                        b = Q.vertices[b.next];
                        ba += (b.index + M - intial) % M;
                    }
                    a = P.vertices[index];
                }
            }

        } while (((aa < M) || (ba < M)) && (aa < 2 * M) && (ba < 2 * M));

        if (inflag.f == inflag.Unknown) {
            return unKnown;
        } else {
            return intersects;
        }
    }

    public static int contains(IndexedPolygon P, IndexedPolygon Q, IndexedPolygon res) {
        /* Deal with special cases: not implemented. */

        IndexedVertex a, b, a1, b1;
        int aHB, bHA;
        // p是正八边形
        // P的范围比Q大，P有可能包含Q
        // 需判断Q是在P的内部还是外部
        if (P.maxx - P.minx >= Q.maxx - Q.minx && P.maxy - P.miny >= Q.maxy - Q.miny) {

            a = Q.vertices[Q.head];
            b = P.vertices[P.head];
            b1 = P.vertices[b.prev];
            boolean notQinP = false;
            do {

                aHB = b1.AreaSign(b1, b, a); // bxa
                if (aHB < 0) { // Q的点不在P内，则Q不被P所包含
                    notQinP = true;
                    return disIntersect;
                } else {
                    b = P.vertices[b.next];
                    b1 = P.vertices[b.prev];
                }
            } while (b.index != P.head);

            // if (!notQinP) {
            // IndexedPolygon tmPolygon = Q;
            // Q = inters;
            // inters = tmPolygon;
            // }

            return PcontainsQ;
        }

        // Q的范围比P大，Q有可能包含P
        else if (P.maxx - P.minx <= Q.maxx - Q.minx && P.maxy - P.miny <= Q.maxy - Q.miny) {

            a = P.vertices[P.head];
            b = Q.vertices[Q.head];
            b1 = Q.vertices[b.prev];
            boolean notPinQ = false;

            do {
                aHB = b1.AreaSign(b1, b, a);
                if (aHB < 0) { // P的点不在Q内，则P不被Q所包含，不相交
                    notPinQ = true;
                    return disIntersect;
                } else {
                    b = Q.vertices[b.next];
                    b1 = Q.vertices[b.prev];
                }
            } while (b.index != Q.head);

            // if (!notPinQ) {
            // IndexedPolygon tmPolygon = P;
            // P = inters;
            // inters = tmPolygon;
            // }

            return QcontainsP;
        }

        // 不相交
        else {

            return disIntersect;
        }

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
    private static int advance_agg(int code, int counter, boolean inside, IndexedPolygon inters, IndexedPolygon P,
                                   IndexedVertex a, IndexedPolygon Q, IndexedVertex b) {

        if (inside) {
            if (counter == aAdvance) {
                InsertInters(a.x, a.y, inters, a.index);
            } else {
                InsertInters(b.x, b.y, inters, b.index);
            }
        }

        int index = 0;

        // �ཻ
        if (code == '1' || code == 'v') {

            int delta = (b.index + M - a.index) % M;

            // aҪ��ǰ��
            // P��������Σ�����ֱ�����
            if (counter == aAdvance) {
                index = (a.index + 2 * delta) % M;
                // a = P.vertices[index];
                aa += 2 * delta;

            }
            // bҪ��ǰ��
            else if (counter == bAdvance) {

                delta = (a.index + M - b.index) % M;
                int intial = b.index;
                index = (a.index + 1);
                ba += (delta + 1);
                for (int i = index; i != intial; ) {
                    i %= M;
                    if (Q.vertices[i].flag) {
                        index = i;
                        break;
                    }
                    i++;
                    ba++;
                    i %= M;

                }

            }
        }

        // ���ཻ�����
        else {
            // rule 3
            if (counter == aAdvance) {
                // inflag.f = inflag.Pin
                // a is inside

                // if(!inside && a.next == b.index){
                // int intial = a.index;
                // a = P.vertices[a.next];
                // index = a.next;
                // aa += (index + M - intial) % M;
                //
                // }
                // else{
                int intial = a.index;
                index = a.next;
                aa += (index + M - intial) % M;
                // }

            }

            // rule 4
            else if (counter == bAdvance) {

                // inflag.f = inflag.Pin
                // a is inside

                // if(!inside && b.next == a.index){
                // int intial = b.index;
                // b = Q.vertices[b.next];
                // index = b.next;
                // ba += (index + M - intial) % M;
                //
                // }
                // else{
                int intial = b.index;
                index = b.next;
                ba += (index + M - intial) % M;
                // }

            }

        }
        return index;
    }

    /*---------------------------------------------------------------------
    Prints out the double point of intersection, and toggles in/out flag.
    ---------------------------------------------------------------------*/
    static cInFlag InOut_agg(IndexedVertex a, IndexedVertex b, Point p, cInFlag inflag, int aHB, int bHA,
                             IndexedPolygon inters) {

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
     * ��P��Q�Ľ����������res�У� ���Ż�
     *
     * @param P
     * @param Q
     * @param res
     *            ���������
     * @return
     */
    public static int intersects(IndexedPolygon P, IndexedPolygon Q, IndexedPolygon res) {

        IndexedPolygon inters = res; /* intersection of the two polygons */
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
        aa = ba = 0;

        inflag = new cInFlag();
        FirstPoint = true;
        IndexedVertex a1, b1;
        A = new Point();
        B = new Point();
        p = new Point();
        q = new Point();
        a = P.vertices[P.head];
        b = Q.vertices[Q.head];

        // ���ཻ�����
        if (P.minx > Q.maxx || P.miny > Q.maxy || P.maxx < Q.minx || P.maxy < Q.miny) {
            // return inters;
            return disIntersect;
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
                    aa = ba = 0;
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
                    index = advance(code, bAdvance, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    b = Q.vertices[index];
                } else {
                    index = advance(code, aAdvance, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    a = P.vertices[index];
                }
            }

            /* Generic cases. */
            else if (cross >= 0) {
                if (bHA > 0) {
                    index = advance(code, aAdvance, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    a = P.vertices[index];
                } else {
                    index = advance(code, bAdvance, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    b = Q.vertices[index];
                }
            } else /* if ( cross < 0 ) */ {
                if (aHB > 0) {
                    index = advance(code, bAdvance, inflag.f == inflag.Qin, inters, P, a, Q, b);
                    b = Q.vertices[index];
                } else {
                    index = advance(code, aAdvance, inflag.f == inflag.Pin, inters, P, a, Q, b);
                    a = P.vertices[index];
                }
            }

        } while (((aa < P.n) || (ba < Q.n)) && (aa < 2 * P.n) && (ba < 2 * Q.n));

        if (inflag.f == inflag.Unknown) {
            return unKnown;
        } else {
            return intersects;
        }
    }

    static cInFlag InOut(IndexedVertex a, IndexedVertex b, Point p, cInFlag inflag, int aHB, int bHA,
                         IndexedPolygon inters) {

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

    private static int advance(int code, int counter, boolean inside, IndexedPolygon inters, IndexedPolygon P,
                               IndexedVertex a, IndexedPolygon Q, IndexedVertex b) {

        if (inside) {
            if (counter == aAdvance) {
                InsertInters(a.x, a.y, inters, a.index);
            } else {
                InsertInters(b.x, b.y, inters, b.index);
            }
        }

        int index = 0;
        if (counter == aAdvance) {
            index = a.next;
            aa++;
        } else if (counter == bAdvance) {
            index = b.next;
            ba++;
        }
        return index;
    }

    private static void InsertInters(double x, double y, IndexedPolygon inters, int index) {
        // Vertex v = new Vertex(x, y, index);
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
        testPolygonIntersects();

    }

    public static void testPolygonIntersects() {
        IndexedPolygon p = new IndexedPolygon(M);
        IndexedPolygon q = new IndexedPolygon(M);

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

        p = new IndexedPolygon(pList, 6);
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
        q = new IndexedPolygon(qList, 6);

        System.out.println("polyGon q:");
        q.PrintVertices();

        IndexedPolygon res = new IndexedPolygon(6);
        intersects(p, q, res);

        System.out.println("p intersects q" + res.n);

        res.PrintVertices();
        if (res.n == 0) {

            System.out.println("null");
        }

        intersects(q, p, res);

        System.out.println("q intersects p" + res.n);

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

        p = new IndexedPolygon(pList, 6);

        System.out.println("polyGon p:");
        p.PrintVertices();

        // res = p.intersects(res);
        intersects(p, res, q);
        res = q;

        System.out.println("res intersects p " + res.n);

        res.PrintVertices();
        if (res.n == 0) {
            System.out.println("null");
        }
    }

}

