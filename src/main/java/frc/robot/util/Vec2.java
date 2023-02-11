package frc.robot.util;

public class Vec2 {
    private double x, y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vec2 plus(Vec2 other) { return new Vec2(x + other.x, y + other.y); }
    public Vec2 minus(Vec2 other) { return new Vec2(x - other.x, y - other.y); }
    public Vec2 times(double scalar) { return new Vec2(scalar * x, scalar * y); }
    public Vec2 unaryMinus() { return this.times(-1); }
    public Vec2 div(double scalar) { return this.times(1/scalar); }
    
    // @Override
    // public boolean equal(other: Object): Boolean = when (other) {
    //     is Vec2 -> (this - other).norm() < 0.0001 // Inequality b/c weird floating point errors 
    //     else -> throw Exception("Tried to check equality of Vec2 with something other than a vector")
    // }
    public String toString() { return "<" + x + ", " + y + ">"; }

    public double dot(Vec2 other) { return x * other.x + y * other.y; } 
    public double norm() { return Math.sqrt(x*x + y*y); }
    public Vec2 hat() { return this.div(this.norm()); }
    // fun rotate(rad: Double) = Vec2(cos(rad) * x - sin(rad) * y, sin(rad) * x + cos(rad) * y)
    // fun rotateDeg(deg: Double) = rotate(deg * PI/180.0)
}