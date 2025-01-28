package org.firstinspires.ftc.teamcode.hardware.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//todo finish this illustration

/**
 * <h3>A class intended to be a simple, universal bridge between different Pose classes.</h3>
 * <pre><code>
 *     //Create a new Pose
 *     Pose pose1 = new Pose(15, 25, 90);
 *
 *     //Create a pose from a Road Runner Pose2d
 *     Pose2d rrPose1 = new Pose2d(15, 25, 90);
 *     Pose pose2 = Pose.from(rrPose1);
 *
 *     //Create a pose from a FTC Nav Pose2D
 *     Pose2D navPose1 = new Pose2D(DistanceUnit.INCH, 5, 10, AngleUnit.DEGREES, 45);
 *     Pose pose3 = Pose.from(navPose1);
 *
 *     //Convert back
 *     Pose2d rrPose2 = pose2.toRR();
 *     Pose2D navPose2 = pose3.toNav();
 * </code></pre>
 * <br>
 * <h1>Visualizing Coordinates:</h1>
 * <hr>
 *
 * <pre>
 *                               ,_,
 *                               |_/
 *                               ||
 *                             +-|_/-------------------------------------------------+
 *                            /#+||-------------------------------------------------/|
 *                           /#/ ||                                                /#|    ___________
 *             ___________  /#/                                                   /#/    /          /
 *            /          / /#/                                                   /#/    /          /
 *           /          / /#/                                                   /#/    /          /
 *          /          / /#/                                                   /#/    /          /
 *         /          / /#/                                                   /#/    /   RED    /
 *        /   BLUE   / /#/                                                   /#/    /          /
 *       /          / /#/                                         _,_,      /#/    /          /
 *      /          / /#/                                          \|./     /#/    /          /
 *     /          / /#/                                           _| |    /#/    /          /
 *    /          / /#/                                            \|./   /#/    /__________/
 *   /          / /#/                                              | |  /#/
 *  /__________/ +-----------------------------------------------------+#/
 *               |/                                                | | |/
 *               +-----------------------------------------------------+
 *  </pre>
 */
public class Pose {
    private final double x, y, z;
    /**
     * <pre>
     *                 &#x293A; yaw (heading)
     *                 |
     *                 |  / &#x2197; forward &#x2197;
     *                 | /
     *                 |/
     *       __________|__________ &#x2939; pitch
     *                /
     *               /
     *              /
     *             &#x21B6; roll
     * </pre>
     */
    private final double yaw, pitch, roll;

    private Pose(double x, double y, double z, double yaw, double pitch, double roll, AngleUnit angleUnit, DistanceUnit distanceUnit){
        this.x = distanceUnit.fromUnit(distanceUnit, x);
        this.y = distanceUnit.fromUnit(distanceUnit, y);
        this.z = distanceUnit.fromUnit(distanceUnit, z);
        this.yaw = AngleUnit.normalizeDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, yaw));
        this.pitch = AngleUnit.normalizeDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, pitch));
        this.roll = AngleUnit.normalizeDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, roll));
    }

    public static Pose from(Pose2d rrPose){
        return new Pose(
                rrPose.getX(),
                rrPose.getY(),
                0,
                rrPose.getHeading(),
                0,
                0,
                AngleUnit.RADIANS,
                DistanceUnit.INCH
        );
    }

    public static Pose from(Pose2D navPose){
        return new Pose(
                navPose.getX(DistanceUnit.INCH),
                navPose.getY(DistanceUnit.INCH),
                0,
                navPose.getHeading(AngleUnit.DEGREES),
                0,
                0,
                AngleUnit.DEGREES,
                DistanceUnit.INCH
        );
    }

    public static Pose from(Pose3D navPose3D){
        Position position = navPose3D.getPosition();
        YawPitchRollAngles angles = navPose3D.getOrientation();
        return new Pose(
                position.x,
                position.y,
                position.z,
                angles.getYaw(AngleUnit.DEGREES),
                angles.getPitch(AngleUnit.DEGREES),
                angles.getRoll(AngleUnit.DEGREES),
                AngleUnit.DEGREES,
                position.unit
        );
    }

    public Pose(double xInches, double yInches, double headingDegrees){
        this(
                xInches,
                yInches,
                0,
                headingDegrees,
                0 ,
                0,
                AngleUnit.DEGREES,
                DistanceUnit.INCH
        );
    }

    public Pose(double xInches, double yInches, double zInches, double yawDegrees, double pitchDegrees, double rollDegrees){
        this(
                xInches,
                yInches,
                0,
                yawDegrees,
                0 ,
                0,
                AngleUnit.DEGREES,
                DistanceUnit.INCH
        );
    }

    /**
     * @return the x coordinate in inches.
     */
    public double x() {
        return x;
    }

    /**
     * @return the y coordinate in inches.
     */
    public double y() {
        return y;
    }

    /**
     * @return the z coordinate in inches.
     */
    public double z() {
        return z;
    }

    /**
     * @apiNote equivalent to heading()
     * @return the yaw angle in degrees.
     */
    public double yaw() {
        return yaw;
    }

    /**
     * @apiNote equivalent to yaw()
     * @return the heading angle in degrees.
     */
    public double heading() {
        return yaw;
    }

    /**
     * @return the pitch angle in degrees.
     */
    public double pitch() {
        return pitch;
    }

    /**
     * @return the roll angle in degrees.
     */
    public double roll() {
        return roll;
    }

    public double distanceTo(Pose otherPose){
        double dx = otherPose.x - x;
        double dy = otherPose.y - y;
        double dz = otherPose.z - z;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2));
    }

    public Pose2d toRR(){
        return new Pose2d(x, y, Math.toRadians(heading()));
    }

    public Pose2D toNav(){
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
    }

    public Pose3D toNav3D(){
        return new Pose3D(new Position(DistanceUnit.INCH, x, y, z, 0), new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0));
    }
}
