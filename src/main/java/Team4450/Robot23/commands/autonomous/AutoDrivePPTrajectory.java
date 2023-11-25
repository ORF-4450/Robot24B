package Team4450.Robot23.commands.autonomous;

import static Team4450.Robot23.Constants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import Team4450.Lib.Util;
import Team4450.Robot23.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot23.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot23.subsystems.DriveBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A command that will follow a trajectory using a PPSwerveDriveController. All of the parameters
 * below are a guess. Need to characterize the robot to get good numbers. Works so-so with
 * the guesses.
 */
public class AutoDrivePPTrajectory extends PPSwerveControllerCommand
{
    private static double   kPX = 1.0, kIX = kPX / 100, kDX = 0, startTime;
    private static double   kPY = 1.0, kIY = kPY / 100, kDY = 0;
    private static double   kPR = 1.0, KIR = kPR / 100, KDR = 0;
    private int             iterations;

    private DriveBase               driveBase;
    private PathPlannerTrajectory   trajectory;
    private StopMotors              stop;
    private Brakes                  brakes;

    /**
     * Auto drive the given trajectory.
     * @param driveBase     Drive base to use.
     * @param trajectory    Trajectory to follow.
     * @param stop          Set stop or not at trajectory end.
     * @param brakes        If stopping, set if brakes on or off.
     */
    public AutoDrivePPTrajectory(DriveBase driveBase, PathPlannerTrajectory trajectory, StopMotors stop, Brakes brakes)
    {
        super(trajectory,
            driveBase::getRobotPose, 
            //DriveBase.getKinematics(),
            new PIDController(kPX, 0, 0),
            new PIDController(kPY, 0, 0),
            new PIDController(kPR, 0, 0),
            driveBase::setSpeeds,
            //driveBase::setModuleStates,
            true);

        this.driveBase = driveBase;
        this.trajectory = trajectory;
        this.stop = stop;
        this.brakes = brakes;
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        startTime = Util.timeStamp();
        
        super.initialize();

        if (brakes == Brakes.on)
            driveBase.setBrakeMode(true);
        else
            driveBase.setBrakeMode(false);

        // Set the current robot pose to match the starting pose of the trajectory. If all of your
        // autonomous moves are correctly coordinated the starting pose of the trajectory should
        // match the physical pose of the robot.
        
        Pose2d pose = trajectory.getInitialHolonomicPose();

        Util.consoleLog("initial traj poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        
        driveBase.setOdometry(pose);
    }

    @Override
    public void execute()
    {
        Util.consoleLog();

        // Causes the controller to advance one step, driving the robot one step along the path.
        super.execute();
        
        Pose2d pose = driveBase.getRobotPose();

        Util.consoleLog("robot poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

        iterations++;
    }
    
    @Override
    public boolean isFinished() 
    {
        // End when the controller has finished the trajectory.
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
        
        super.end(interrupted);

        if (stop == StopMotors.stop) driveBase.stop();
                
        Pose2d pose = driveBase.getRobotPose();

        Util.consoleLog("poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
    }
}
