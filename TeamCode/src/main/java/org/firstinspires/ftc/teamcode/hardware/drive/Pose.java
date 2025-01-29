package org.firstinspires.ftc.teamcode.hardware.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//todo finish this illustration (add x and y)

/**
 * <h3>A class intended to be a simple, universal bridge between different Pose classes.</h3>
 * <pre><code>
 *     //Create a new Pose
 *     Pose newPose2D = new Pose(15, 25, 45);
 *     Pose newPose3D = new Pose(15, 25, 45, 120, 76, 50);
 *
 *     //Create a pose from a Road Runner Pose2d
 *     Pose2d rrPose1 = new Pose2d(15, 25, 90);
 *     Pose fromRRPose = Pose.from(rrPose1);
 *
 *     //Create a pose from a FTC Nav Pose2D
 *     Pose2D navPose2D1 = new Pose2D(DistanceUnit.INCH, 5, 10, AngleUnit.DEGREES, 45);
 *     Pose fromNav2D = Pose.from(navPose2D1);
 *
 *     //Create a pose from a FTC Nav Pose3D
 *     Pose2D navPose3D1 = new Pose3D(DistanceUnit.INCH, 5, 10, 2, AngleUnit.DEGREES, 45, 30, 60);
 *     Pose fromNav3D = Pose.from(navPose3D1);
 *
 *     //Convert a pose to another type of pose
 *     Pose2d rrPose2 = newPose2D.toRR();
 *     Pose2D navPose2D2 = newPose2D.toNav();
 *     Pose3D navPose3D2 = newPose3D.toNav3D();
 *
 *     //poses can be cross-converted even between 2D and 3D
 *     Pose myPose = Pose.from(new Pose2d(50, 30, 45));
 *     Pose3D myNavPose = myPose.toNav3D();
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
 *                           /#/ ||                                          /     /#|    ___________
 *             ___________  /#/                                             /     /#/    /          /
 *            /          / /#/                     |   /                    \    /#/    /          /
 *           /          / /#/                      |  /                      \  /#/    /          /
 *          /          / /#/                       | /                        \/#/    /          /
 *         /          / /#/                        |/                         /#/    /   RED    /
 *        /   BLUE   / /#/             ____________|____________             /#/    /          /
 *       /          / /#/                         /               _,_,      /#/    /          /
 *      /          / /#/ \                       /                \|./     /#/    /          /
 *     /          / /#/   \                     /                 _| |    /#/    /          /
 *    /          / /#/     \                   /                  \|./   /#/    /__________/
 *   /          / /#/      /                                       | |  /#/
 *  /__________/ +--------+----------------------------------------+-+-+#/
 *               |/      /                                         | | |/
 *               +-----------------------------------------------------+
 *  </pre>
 *  <br>
 *  <h1>Visualizing Orientation:</h1>
 *  <hr>
 *  <pre>
 *              &#x293A; yaw (heading)
 *              |
 *              |  / &#x2197; forward &#x2197;
 *              | /
 *              |/
 *    __________|__________ &#x2939; pitch
 *             /
 *            /
 *           /
 *             &#x21B6; roll
 *  </pre>
 */
public class Pose {
    private final double x, y, z;
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



    /**
     * @param unit the distance unit to return the coordinate in
     * @return the x coordinate in the provided distance unit.
     */
    public double x(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, x);
    }

    /**
     * @param unit the distance unit to return the coordinate in
     * @return the y coordinate in the provided distance unit.
     */
    public double y(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, y);
    }

    /**
     * @param unit the distance unit to return the coordinate in
     * @return the z coordinate in the provided distance unit.
     */
    public double z(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, z);
    }

    /**
     * @apiNote equivalent to heading()
     * @param unit the angle unit to return the angle in
     * @return the yaw angle in the provided angle unit.
     */
    public double yaw(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, yaw);
    }

    /**
     * @apiNote equivalent to heading()
     * @param unit the angle unit to return the angle in
     * @return the heading angle in the provided angle unit.
     */
    public double heading(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, yaw);
    }

    /**
     * @apiNote equivalent to heading()
     * @param unit the angle unit to return the angle in
     * @return the pitch angle in the provided angle unit.
     */
    public double pitch(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, pitch);
    }

    /**
     * @apiNote equivalent to heading()
     * @param unit the angle unit to return the angle in
     * @return the roll angle in the provided angle unit.
     */
    public double roll(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, roll);
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

    public Position getPosition(){
        return new Position(DistanceUnit.INCH, x, y, z, 0);
    }

    public YawPitchRollAngles getAngles(){
        return new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);
    }
}
