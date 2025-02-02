package org.firstinspires.ftc.teamcode.hardware.drive;

public class Area {

    /**
     * The size of the area in the x direction.
     */
    private double width;
    /**
     * The size of the area in the y direction.
     */
    private double height;

    private final Pose topLeft;
    private final Pose topRight;
    private final Pose bottomLeft;
    private final Pose bottomRight;
    private final Pose center;

    public Area(Pose topLeft, double width, double height){
        this(topLeft, topLeft.plusX(width).plusY(-height));
    }

    public Area(Pose topLeft, Pose bottomRight){
        this.width = bottomRight.x() - topLeft.x();
        this.height = topLeft.y() - bottomRight.y();
        this.topLeft = topLeft;
        this.topRight = topLeft.plusX(width);
        this.bottomLeft = topLeft.plusY(-height);
        this.bottomRight = bottomRight;
        this.center = topLeft.plusX(width/2.0).plusY(-height/2.0);
    }

    public Pose getTopLeft(){
        return topLeft;
    }

    public Pose getTopRight() {
        return topRight;
    }

    public Pose getBottomLeft() {
        return bottomLeft;
    }

    public Pose getBottomRight() {
        return bottomRight;
    }

    public Pose getCenter() {
        return center;
    }

    public double getMinX(){
        return bottomLeft.x();
    }

    public double getMaxX(){
        return topRight.x();
    }

    public double getMinY(){
        return bottomLeft.y();
    }

    public double getMaxY(){
        return topRight.y();
    }

    /**
     * Get the size of the area in the y direction.
     */
    public double getHeight() {
        return height;
    }

    /**
     * Get the size of the area in the x direction.
     */
    public double getWidth() {
        return width;
    }

    public boolean containsPose(Pose pose){
        return (
                (pose.x() >= getMinX() && pose.x() <= getMaxX()) &&
                (pose.y() >= getMinY() && pose.y() <= getMaxY())
        );
    }
}
