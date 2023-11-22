package org.act.util.polygon;

public class Vertex {
    Vertex prev;
    public Vertex next;
    public Point v;
    boolean ear = false;
    int vnum;
    // cEdge duplicate;
    boolean onhull; /* T iff point on hull. */
    boolean mark;

    public int index;

    Vertex() {
        prev = next = null;
        v = new Point();
        vnum = 0;
        // duplicate = null;
        onhull = false;
        mark = false;
    }

    public Vertex(double i, double j) {
        v = new Point();
        v.x = i;
        v.y = j;
        v.z = i * i + j * j;
        prev = next = null;
    }

    public Vertex(double i, double j, int index) {
        v = new Point();
        v.x = i;
        v.y = j;
        v.z = i * i + j * j;
        prev = next = null;
        this.index = index;
    }

    Vertex(double x, double y, double z) {
        v = new Point();
        v.x = x;
        v.y = y;
        v.z = z;
        prev = next = null;
    }

    Vertex(Vertex v) {
        this.v = new Point();
        this.v.x = v.v.x;
        this.v.y = v.v.y;
        this.v.z = v.v.z;
        this.prev = v.prev;
        this.next = v.next;

    }

    public void PrintVertex() {
        v.PrintPoint();
    }

    public void PrintVertex(int index) {
        System.out.print("V" + this.index + " = ");
        v.PrintPoint();
    }
}
