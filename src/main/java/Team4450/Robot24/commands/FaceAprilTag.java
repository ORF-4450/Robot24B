package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

public class FaceAprilTag extends PIDCommand {
    DriveBase  robotDrive;
    PhotonVision    camera;

    public FaceAprilTag(PhotonVision camera, DriveBase robotDrive) {
        super(
            new PIDController(0.001, 0, 0), // the PID Controller
            ()->camera.getYaw(), // measurement
            0, // setpoint
            (o) -> robotDrive.setTrackingRotation(camera.hasTargets() ? o : Double.NaN)
        );

        Util.consoleLog();

        getController().setTolerance(0.3);
        this.robotDrive = robotDrive;
        this.camera = camera;
        
        SmartDashboard.putData("AprilTag PID", getController());
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        robotDrive.enableTracking();
        
        super.initialize();
    }
    
    @Override
    public void execute() {
        // Util.consoleLog("camera yaw value: %f", camera.getYaw());
        super.execute();

        SmartDashboard.putBoolean("Has AprilTag", camera.hasTargets());
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);

        this.robotDrive.disableTracking();
    }
}
