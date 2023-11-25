package Team4450.Robot23.commands.autonomous;

import static Team4450.Robot23.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.commands.LowerArm;
import Team4450.Robot23.commands.OpenClaw;
import Team4450.Robot23.commands.RaiseArm;
import Team4450.Robot23.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot23.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.Intake;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This an autonomous command score in lowest position then drive straight out 
 * a specified distance in meters. The distance is set by which starting pose 
 * is selected.
 */
public class AutoScoreLow extends Command
{
	private final DriveBase         driveBase;
    private final Winch             winch;
    private final Arm               arm;
    private final Intake            intake;
	
	private SequentialCommandGroup	commands = null;
	private Command					command = null;
	private	Pose2d					startingPose;
    private int                     startingPoseIndex;

	/**
	 * Creates a new ScoreLow autonomous command.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
     * @param winch Winch subsystem.
     * @param arm Arm subsystem.
     * @param intake Intake subsystem.
     * @param startingPose Start location pose.
     * @param startingPoseIndex The starting pose position 0-9.
	 */
	public AutoScoreLow(DriveBase driveBase, Winch winch, Arm arm, Intake intake,
                    Pose2d startingPose, Integer startingPoseIndex) 
	{
		Util.consoleLog("idx=%d", startingPoseIndex);
		
		this.driveBase = driveBase;
        this.winch = winch;
        this.arm = arm;
        this.intake = intake;

		this.startingPose = startingPose;

        this.startingPoseIndex = startingPoseIndex;
			  
		// Use addRequirements() here to declare subsystem dependencies.
		// This command is requiring the driveBase for itself and all 
		// commands added to the command list. If any command in the
		// list also requires the drive base, or any other subsystem
		// listed here, it will cause this command to be interrupted.
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
		
		//driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	LCD.printLine(LCD_1, "Mode: Auto - ScoreLow - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);

		SmartDashboard.putBoolean("Autonomous Active", true);

		// Set heading tracking to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work.
		driveBase.setOdometry(startingPose);

        double distance = 0;

        // Start position 1-9 are indexed as 1-9. Distance to drive in meters.

        if (startingPoseIndex == 1) distance = 4.0;
        if (startingPoseIndex == 9) distance = 4.0;
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done at run time.
		
		commands = new SequentialCommandGroup();
		
        // First action is to hold the game piece.

        command = new InstantCommand(intake::toggleHoldPosition);

		commands.addCommands(command);

        // Next action is to lower the arm to drop position.

		command = new LowerArm(winch, -60);	//-68

		commands.addCommands(command);

        // Next action is to drop the game piece.

        command = new InstantCommand(intake::dropGamePiece);

		commands.addCommands(command);

		// Next two commands will be run in parallel.

		ParallelCommandGroup pCommands = new ParallelCommandGroup();

		// Next action is to raise the arm.

		command = new RaiseArm(winch, 100);

		pCommands.addCommands(command);

		// Last action is to drive forward distance meters and stop.
		
		command = new AutoDriveProfiled(driveBase, distance, StopMotors.stop, Brakes.off);
		
		pCommands.addCommands(command);

		commands.addCommands(pCommands);

		// Launch autonomous command sequence.
		
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

		if (interrupted) commands.cancel();
		
		driveBase.drive(0, 0, 0);
		
		Util.consoleLog("end ---------------------------------------------------------------");

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

