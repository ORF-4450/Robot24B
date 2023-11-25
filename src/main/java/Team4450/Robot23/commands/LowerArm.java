package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves the arm to a target position going down.
 */
public class LowerArm extends Command 
{
    private final Winch     winch;
    private SynchronousPID  controller = new SynchronousPID(getName(), .03, .003, .003);
    private final double    tolerance = .5, maxPower = .30;
    private double          lastTimeCalled;
    private static Integer  instances = 1;

    /**
     * Move winch to target position going downward.
     * @param winch Winch subsystem.
     * @param targetPosition Target position in winch motor revolutions (-).
     */
    public LowerArm(Winch winch, double targetPosition)
    {
        this(winch, instances.toString(), targetPosition);

        instances++;
    }

    /**
     * Move winch to target position going downward.
     * @param winch Winch subsystem.
     * @param name Title of command in LiveWindow.
     * @param targetPosition Target position in winch motor revolutions (-).
     */
    public LowerArm(Winch winch, String name, double targetPosition)
    {
        Util.consoleLog("%s: %.1f", name, targetPosition);

        this.winch = winch;
        
        controller.setSetpoint(targetPosition);

        controller.setOutputRange(-maxPower, maxPower);

        addRequirements(winch);
                
        setName(String.format("%s[%s]", getName(), name));

        controller.setName(getName());
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        SmartDashboard.putBoolean(getName(), true);

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
        return controller.onTarget(tolerance); // || winch.getLowerSwitch();
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean(getName(), false);
    }
}
