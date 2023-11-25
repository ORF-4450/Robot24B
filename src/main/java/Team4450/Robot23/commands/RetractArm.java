package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Retracts the arm by running arm inward until encoder
 * reports zero.
 */
public class RetractArm extends Command 
{
    private final Arm      arm;

    public RetractArm(Arm arm)
    {
        Util.consoleLog();

        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        arm.setPower(.60);

        SmartDashboard.putBoolean("RetractArm", true);
    }

    @Override
    public boolean isFinished()
    {
        if (arm.getPosition() <= 1) 
            return true;
        else 
            return false;

        //return arm.getSwitch();
    }

    @Override
    public void end(boolean interrupted) 
    {
        arm.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("RetractArm", false);
    }
}
