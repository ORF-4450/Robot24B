package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drops the arm by running winch down until low position limit
 * switch reports true.
 */
public class RaiseArmStart extends Command 
{
    private final Winch      winch;

    public RaiseArmStart(Winch winch)
    {
        Util.consoleLog();

        this.winch = winch;

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        winch.setPower(.50);

        SmartDashboard.putBoolean("RaiseArmStart", true);
    }

    @Override
    public boolean isFinished()
    {
        return winch.getUpperSwitch();
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("RaiseArmStart", false);
    }
}

