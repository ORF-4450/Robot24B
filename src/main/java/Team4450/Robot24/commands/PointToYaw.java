package Team4450.Robot24.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import Team4450.Robot24.subsystems.DriveBase;

public class PointToYaw extends Command {
    private DoubleSupplier  yawSupplier;
    private boolean         wait;
    private DriveBase       robotDrive;
    private PIDController   pidController = new PIDController(1, 0, 0);
    private Set<Subsystem>  requirements;

    private static final double NO_VALUE = Double.NaN;

    /**
     * Point to yaw
     * @param yawSupplier a supplier of desired yaw values (IN RADIANS!)
     * @param robotDrive the drive subsystem
     * @param wait whether or not to wait until it is completed to drive again
     */
    public PointToYaw(DoubleSupplier yawSupplier, DriveBase robotDrive, boolean wait) {
        Util.consoleLog();

        this.yawSupplier = yawSupplier;
        this.robotDrive = robotDrive;
        this.wait = wait;
        this.requirements = Set.of();

        if (wait) this.requirements = Set.of(robotDrive);
    }

    @Override
    public void execute() {
        double desiredYaw = yawSupplier.getAsDouble();

        pidController.setSetpoint(desiredYaw);

        if (Double.isNaN(desiredYaw)) {
            robotDrive.setTrackingRotation(Double.NaN);
            return;
        }

        double rotation = pidController.calculate(robotDrive.getYawR());

        if (wait) {
            robotDrive.drive(0,0,rotation,false);
        }

        robotDrive.setTrackingRotation(rotation);

        Util.consoleLog("%f", rotation);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (wait)
            return InterruptionBehavior.kCancelIncoming;
        else
            return InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        // pidController.setTolerance(.1);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        robotDrive.enableTracking();
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("ended interrupted: %b", interrupted);

        robotDrive.disableTracking();
        robotDrive.setTrackingRotation(0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    /**
    * Generate a yaw from a POV value.
    *
    * @param pov The POV value.
    * @return yaw
    */
    public static double yawFromPOV(double pov) {
        if (pov < 0)
            return NO_VALUE;
        else {
            double radians = Math.toRadians(pov);

            if (radians < -Math.PI) {
                double overshoot = radians + Math.PI;
                radians = -overshoot;
            }

            radians *= -1;
            return radians;
        }
    }
    
    /**
    * generate a yaw from axis values
    *
    * @param xAxis
    * @param yAxis
    * @return yaw
    */
    public static double yawFromAxes(double xAxis, double yAxis) {
        double theta = Math.atan2(xAxis, yAxis);
        double magnitude = Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2));

        if (magnitude > 0.2) {
            return theta;
        } else {
            return NO_VALUE;
        }
    }
}
