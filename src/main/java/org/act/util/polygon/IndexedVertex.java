package org.act.util.polygon;


public class IndexedVertex {
    public double x;
    public double y;
    public int index;
    public int prev;
    public int next;
    public boolean flag;

    public IndexedVertex() {

    }

    public IndexedVertex(double x, double y, int index) {
        this.x = x;
        this.y = y;
        this.index = index;
    }

    public void setXY(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setIndex(double x, double y, int index) {
        this.x = x;
        this.y = y;
        this.index = index;
        this.flag = true;

    }

    /**
     * ba x ca�ķ���
     *
     * @param a
     * @param b
     * @param c
     * @return
     */
    public int AreaSign(IndexedVertex a, IndexedVertex b, IndexedVertex c) {
        double area2;

        area2 = (b.x - a.x) * (double) (c.y - a.y) - (c.x - a.x) * (double) (b.y - a.y);

        /* The area should be an integer. */
        if (area2 > 0)
            return 1;
        else if (area2 < 0)
            return -1;
        else
            return 0;
    }

    /*---------------------------------------------------------------------
      SegSegInt: Finds the point of intersection p between two closed
      segments ab and cd.  Returns p and a char with the following meaning:
      'e': The segments collinearly overlap, sharing a point.
      'v': An endpoint (vertex) of one segment is on the other segment,
      but 'e' doesn't hold.
      '1': The segments intersect properly (i.e., they share a point and
      neither 'v' nor 'e' holds).
      '0': The segments do not intersect (i.e., they share no points).
      Note that two collinear segments that share just one point, an endpoint
      of each, returns 'e' rather than 'v' as one might expect.
      ---------------------------------------------------------------------*/
    public char SegSegInt(IndexedVertex a, IndexedVertex b, IndexedVertex c, IndexedVertex d, Point p, Point q) {
        double s, t; /* The two parameters of the parametric eqns. */
        double num, denom; /* Numerator and denoninator of equations. */
        char code = '?'; /* Return char characterizing intersection. */
        p.x = p.y = 100.0; /* For testing purposes only... */

        denom = a.x * (double) (d.y - c.y) + b.x * (double) (c.y - d.y) + d.x * (double) (b.y - a.y)
                + c.x * (double) (a.y - b.y);

        /* If denom is zero, then segments are parallel: handle separately. */
        if (denom == 0.0)
            return ParallelInt(a, b, c, d, p, q);

        num = a.x * (double) (d.y - c.y) + c.x * (double) (a.y - d.y) + d.x * (double) (c.y - a.y);
        if ((num == 0.0) || (num == denom))
            code = 'v';
        s = num / denom;
        // System.out.println("SegSegInt: num=" + num + ",denom=" + denom + ",s="+s);

        num = -(a.x * (double) (c.y - b.y) + b.x * (double) (a.y - c.y) + c.x * (double) (b.y - a.y));
        if ((num == 0.0) || (num == denom))
            code = 'v';
        t = num / denom;
        // System.out.println("SegSegInt: num=" +num + ",denom=" + denom + ",t=" + t);

        if ((0.0 < s) && (s < 1.0) && (0.0 < t) && (t < 1.0))
            code = '1';
        else if ((0.0 > s) || (s > 1.0) || (0.0 > t) || (t > 1.0))
            code = '0';

        p.x = a.x + s * (b.x - a.x);
        p.y = a.y + s * (b.y - a.y);

        return code;
    }

    /*---------------------------------------------------------------------
      Returns TRUE iff point c lies on the closed segement ab.
      Assumes it is already known that abc are collinear.
      (This is the only difference with Between().)
      ---------------------------------------------------------------------*/
    public boolean Between1(IndexedVertex a, IndexedVertex b, IndexedVertex c) {
        Point ba, ca;

        /* If ab not vertical, check betweenness on x; else on y. */
        if (a.x != b.x)
            return ((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x));
        else
            return ((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y));
    }

    public boolean Collinear(IndexedVertex a, IndexedVertex b, IndexedVertex c) {
        return AreaSign(a, b, c) == 0;
    }

    public void Assigndi(Point p, IndexedVertex a) {
        p.x = a.x;
        p.y = a.y;
    }

    public char ParallelInt(IndexedVertex a, IndexedVertex b, IndexedVertex c, IndexedVertex d, Point p, Point q) {
        if (!a.Collinear(a, b, c))
            return '0';

        if (Between1(a, b, c) && Between1(a, b, d)) {
            Assigndi(p, c);
            Assigndi(q, d);
            return 'e';
        }
        if (Between1(c, d, a) && Between1(c, d, b)) {
            Assigndi(p, a);
            Assigndi(q, b);
            return 'e';
        }
        if (Between1(a, b, c) && Between1(c, d, b)) {
            Assigndi(p, c);
            Assigndi(q, b);
            return 'e';
        }
        if (Between1(a, b, c) && Between1(c, d, a)) {
            Assigndi(p, c);
            Assigndi(q, a);
            return 'e';
        }
        if (Between1(a, b, d) && Between1(c, d, b)) {
            Assigndi(p, d);
            Assigndi(q, b);
            return 'e';
        }
        if (Between1(a, b, d) && Between1(c, d, a)) {
            Assigndi(p, d);
            Assigndi(q, a);
            return 'e';
        }
        return '0';
        /*
         * if ( Between1( a, b, c ) ) { Assigndi( p, c ); return 'e'; } if ( Between1(
         * a, b, d ) ) { Assigndi( p, d ); return 'e'; } if ( Between1( c, d, a ) ) {
         * Assigndi( p, a ); return 'e'; } if ( Between1( c, d, b ) ) { Assigndi( p, b
         * ); return 'e'; } return '0';
         */
    }

    public void PrintVertex() {
        System.out.println("(" + x + "," + y + ")");
    }

    public void PrintVertex(int index) {
        System.out.print("V" + index + " = ");
        System.out.println("(" + x + "," + y + ")");
    }
}
