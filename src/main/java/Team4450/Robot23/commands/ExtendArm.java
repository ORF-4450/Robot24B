package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves the arm to a target position.
 */
public class ExtendArm extends Command 
{
    private final Arm       arm;
    private SynchronousPID  controller = new SynchronousPID(getName(), .15, .015, .015);
    private final double    tolerance = 0.5, maxPower = .80;
    private double          lastTimeCalled;
    private static Integer  instances = 1;

    /**
     * Move arm to target position.
     * @param arm Arm subsystem.
     * @param targetPosition Target position in arm motor revolutions.
     */
    public ExtendArm(Arm arm, double targetPosition)
    {
        this(arm, instances.toString(), targetPosition);

        instances++;
    }

    /**
     * Move arm to target position.
     * @param arm Arm subsystem.
     * @param name Title of command in LiveWindow.
     * @param targetPosition Target position in arm motor revolutions.
     */
    public ExtendArm(Arm arm, String name, double targetPosition)
    {
        Util.consoleLog("%s: %.1f", name, targetPosition);

        this.arm = arm;

        controller.setSetpoint(targetPosition);

        controller.setOutputRange(-maxPower, maxPower);

        controller.setTolerance(tolerance);

        addRequirements(arm);
        
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

        double power = controller.calculate(arm.getPosition(), time);

        // Invert power because calculation above returns + result but arm
        // wants - power to extend.

        arm.setPower(-power);
    }

    @Override
    public boolean isFinished()
    {
        return controller.onTarget(tolerance); 
    }

    @Override
    public void end(boolean interrupted) 
    {
        arm.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean(getName(), false);
    }
}