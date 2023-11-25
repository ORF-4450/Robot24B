package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves the arm to a target position going up.
 */
public class RaiseArm extends Command 
{
    private final Winch     winch;
    private double          targetPostion = 100;    // Revolutions of motor.
    private SynchronousPID  controller = new SynchronousPID("RaiseArm", .01, 0, 0);
    private final double    tolerance = 1, maxPower = .30;
    private double          lastTimeCalled;

    /**
     * Move winch to target position going upward.
     * @param winch Winch subsystem.
     * @param targetPosition Target position in winch motor revolutions (-).
     */
    public RaiseArm(Winch winch, double targetPosition)
    {
        Util.consoleLog();

        this.winch = winch;

        this.targetPostion = targetPosition;

        controller.setOutputRange(-maxPower, maxPower);

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        // Reset clears setpoint so we reestablish it here.
        
        controller.setSetpoint(targetPostion);

        SmartDashboard.putBoolean("RaiseArm", true);

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
        return controller.onTarget(tolerance) || winch.getUpperSwitch();
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("RaiseArm", false);
    }
}
