package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Opens the Claw by running the motor until Reverse limit
 * switch reports true.
 */
public class OpenClaw extends Command 
{
    private final Claw      claw;

    public OpenClaw(Claw claw)
    {
        Util.consoleLog();

        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        claw.setPower(.20);

        SmartDashboard.putBoolean("OpenClaw", true);
    }

    @Override
    public boolean isFinished()
    {
        return claw.getOpenSwitch();
    }

    @Override
    public void end(boolean interrupted) 
    {
        claw.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("OpenClaw", false);
    }
}

