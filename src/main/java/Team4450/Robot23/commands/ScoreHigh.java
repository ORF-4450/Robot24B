package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Intake;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Lower arm from fully up position to correct height for high scoring,
 * extend the arm to back scoring position.
 */
public class ScoreHigh extends Command 
{
    private final Winch             winch;
    private SequentialCommandGroup	commands;
    private ParallelCommandGroup    pCommands; 

    public ScoreHigh(Winch winch, Arm arm, Intake intake)
    {
        Util.consoleLog();

        this.winch = winch;

        addRequirements(winch, arm, intake);

        // Build the command sequence to move arm to high scoring position.
		
		commands = new SequentialCommandGroup();

		// Some commands will be run in parallel.

		pCommands = new ParallelCommandGroup();

		// First action is to lower the arm.

		Command command = new LowerArm(winch, getName(), -45);

		pCommands.addCommands(command);

        // Next action is to extend arms.

        command = new ExtendArm(arm, getName(), 242);

		pCommands.addCommands(command);

        // Add parallel commands to sequential commands.

        commands.addCommands(pCommands);

        // Now hold winch position.

        command = new InstantCommand(winch::toggleHoldPosition);

		commands.addCommands(command);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Run the commands, only if winch mostly up.

        if (winch.getPosition() > -5) commands.schedule();

        SmartDashboard.putBoolean(getName(), true);
    }

    @Override
    public boolean isFinished()
    {
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		return !commands.isScheduled();
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        if (interrupted) commands.cancel();

        SmartDashboard.putBoolean(getName(), false);
    }
}

