package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Holds winch at current position. Not being used at this time. Hold function
 * moved to Winch subsystem for more flexibility and doing hold in Autonomous
 * programs.
 */
public class HoldWinchPosition extends Command 
{
    private final Winch     winch;
    private SynchronousPID  controller = new SynchronousPID("HoldWinch", 0.1, 0, 0);
    private final double    maxPower = .10;
    private double          lastTimeCalled;

    /**
     * Holds winch at current position. Not being used at this time. Hold function
     * moved to Winch subsystem for more flexibility and doing hold in Autonomous
     * programs.
     * @param winch Winch subsystem.
     */
    public HoldWinchPosition(Winch winch)
    {
        Util.consoleLog();

        this.winch = winch;

        controller.setOutputRange(-maxPower, maxPower);

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        controller.setSetpoint(winch.getPosition());

        SmartDashboard.putBoolean("HoldWinch", true);

        lastTimeCalled = Util.timeStamp();
    }

    @Override
    public void execute()
    {
        double time = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        double power = controller.calculate(winch.getPosition(), time);

        winch.setPower(power);
    }

    @Override
    public boolean isFinished()
    {
        // Note: This commands runs until canceled.
        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("HoldWinch", false);
    }
}
