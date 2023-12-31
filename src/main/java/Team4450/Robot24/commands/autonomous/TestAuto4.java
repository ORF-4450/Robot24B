package Team4450.Robot24.commands.autonomous;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot24.subsystems.DriveBase;

import static Team4450.Robot24.Constants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an example autonomous command based on 4450 customized version of
 * PathPlanner Trajectory following commands.
 */
public class TestAuto4 extends Command
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
    private Command					command = null;
	private	Pose2d					startingPose;   

	/**
	 * Creates a new TestAuto3 autonomous command. This command demonstrates one
	 * possible structure for an autonomous command and shows the use of the 
	 * autonomous trajectory driving support commands.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 * @param startingPose Starting pose of the robot.
	 */
	public TestAuto4(DriveBase driveBase, Pose2d startingPose) 
	{
		Util.consoleLog();
		
		this.driveBase = driveBase;

		this.startingPose = startingPose;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.driveBase);
	}
	
	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Autonomous Active", true);
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAuto4 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);
		
		// Reset drive base distance tracking.	  	
	  	driveBase.resetDistanceTraveled();
	  	
	  	// Set drive base yaw tracking to 0.
	  	driveBase.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work.
		driveBase.setOdometry(startingPose);

		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// NOTE: Here we run into the difference of X/Y axis definitions between
		// joystick axes and field axes. JS is Y fwd/back (down the field) and X is left/right
		// (paralell to the driver station wall). In the WPILib code, it is opposite. Note that
		// in the drive command we switch the JS axes to fix this issue before passing to the
		// swerve code. Here we are dealing directly with swerve drive code so X is fwd/back
		// down the field and Y is left/right (strafe).

		double spx = startingPose.getX();
		double spy = startingPose.getY();
		Rotation2d spr = startingPose.getRotation();

		PathPlannerTrajectory exampleTrajectory = PathPlanner.generatePath(
				new PathConstraints(MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL), 
				new PathPoint(new Translation2d(spx, spy), spr, spr),
				new PathPoint(new Translation2d(spx += 2, spy += 1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)), 
				new PathPoint(new Translation2d(spx += 3, spy += 0), Rotation2d.fromDegrees(0), spr) 
	        );
	
		// Mirror trajectory for starting on Red side. Force blue for testing use live alliance
		// for full scale testing or competition. We only do this here for manually created trajectories.
		// For GUI created trajectoryies, the PPSwerveControllerCommand does the mirroring automatically.

		exampleTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(exampleTrajectory,
																				 Alliance.Blue); 
																				 //DriverStation.getAlliance());

		command = new AutoDrivePPTrajectory(driveBase, exampleTrajectory, StopMotors.stop, Brakes.on);
		//command = new AutoDrivePPTrajectory(driveBase, RobotContainer.ppTestTrajectory, StopMotors.stop, Brakes.on);
		
		commands.addCommands(command);
		
		commands.schedule();
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 *  In this model, this command just idles while the Command Group we
	 *  created runs on its own executing the steps (commands) of this Auto
	 *  program.
	 */
	@Override
	public void execute() 
	{
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
		
		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingR());
		Util.consoleLog("end -----------------------------------------------------");

		SmartDashboard.putBoolean("Autonomous Active", false);
	}
	
	/**
	 *  Returns true when this command should end. That should be when
	 *  all the commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() 
	{
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		return !commands.isScheduled();
    }

}

