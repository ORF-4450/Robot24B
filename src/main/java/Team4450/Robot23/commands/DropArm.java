package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drops the arm by running winch down until low position limit
 * switch reports true.
 */
public class DropArm extends Command 
{
    private final Winch     winch;
    private final Arm       arm;

    public DropArm(Winch winch, Arm arm)
    {
        Util.consoleLog();

        this.winch = winch;
        this.arm = arm;

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Do not drop if arm not mostly retracted.

        if (arm.getPosition() < 100)
        {
            winch.setPower(-.30);

            SmartDashboard.putBoolean("DropArm", true);
        }
    }

    @Override
    public boolean isFinished()
    {
        return winch.getPosition() <= winch.WINCH_MAX;
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("DropArm", false);
    }
}
