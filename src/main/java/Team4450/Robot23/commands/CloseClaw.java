package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves the Claw to a target position.
 * 
 * NOTE: this class has been modified from its original form to demonstrate how to
 * add the Sendable interface to a Command so we can adjust command parameters via
 * LiveWindow from one scheduling to the next. It also shows naming an instance of
 * the command so the name shows in LiveWindow.
 */
public class CloseClaw extends Command  
{
    private final Claw      claw;
    private double          targetPostion = 3000;    // Default is cube encoder count.
    private SynchronousPID  controller = new SynchronousPID(getName(), .0001, .00001, 0);
    private double          tolerance = 200, maxPower = .30;
    private double          lastTimeCalled;
    private static Integer  instances = 1;

    /**
     * Move claw to target position (closing it).
     * @param claw Claw subsystem.
     * @param targetPosition Target position in encoder counts.
     */
    public CloseClaw(Claw claw, double targetPosition)
    {
        this(claw, instances.toString(), targetPosition);

        instances++;
    }

    /**
     * Move claw to target position (closing it).
     * @param claw Claw subsystem.
     * @param name Title of command in LiveWindow.
     * @param targetPosition Target position in encoder counts.
     */
    public CloseClaw(Claw claw, String name, double targetPosition)
    {
        Util.consoleLog("%s: %.1f", name, targetPosition);

        this.claw = claw;

        this.targetPostion = targetPosition;

        controller.setOutputRange(-maxPower, maxPower);

        controller.setSetpoint(targetPostion);

        addRequirements(claw);

        setName(String.format("%s[%s]", getName(), name));

        controller.setName(getName());

        SendableRegistry.addLW(this, getName());
    }
    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        // We duplicate the range and setpoint initialization here so
        // it can be changed from one scheduling to the next via Sendable,
        // LiveWindow and OutlineViewer.

        controller.setOutputRange(-maxPower, maxPower);

        controller.setSetpoint(targetPostion);

        SmartDashboard.putBoolean(getName(), true);

        lastTimeCalled = Util.timeStamp();
    }

    @Override
    public void execute()
    {
        double time = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        // Note that the encoder will count + from fully open position
        // where it is reset. This results in the PID controller generating
        // a + power value. However, - power is required to close the claw.
        // Hence the inversion of the power below.

        double power = controller.calculate(claw.getPosition(), time);

        claw.setPower(-power);
    }

    @Override
    public boolean isFinished()
    {
        return controller.onTarget(tolerance);
    }

    @Override
    public void end(boolean interrupted) 
    {
        claw.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean(getName(), false);
    }

    public void setTargetPosition(double position)
    {
        targetPostion = position;
    }

    public void setMaxPower(double power)
    {
        maxPower = power;
    }
    
    @Override
	public void initSendable( SendableBuilder builder )
	{
        builder.setSmartDashboardType("Command");
        
		builder.addDoubleProperty("TargetPosition", ()-> targetPostion, this::setTargetPosition);
		builder.addDoubleProperty("MaxPower", ()-> maxPower, this::setMaxPower);
	}   	
}
