package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Intake;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Intake the cube by running the motor until current spikes (stall).
 */
public class IntakeCube extends Command 
{
    private final Intake    intake;
    private double          power = .15, stallCurrent = 50;

    public IntakeCube(Intake intake)
    {
        Util.consoleLog();

        this.intake = intake;

        addRequirements(intake);
        
        SendableRegistry.addLW(this, getName());
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        if (intake.isHoldingPosition()) intake.toggleHoldPosition();

        intake.setPower(power);

        SmartDashboard.putBoolean(getName(), true);
    }

    @Override
    public boolean isFinished()
    {
        return intake.getMotorCurrent() > stallCurrent;
    }

    @Override
    public void end(boolean interrupted) 
    {
        intake.stop();

        if (!interrupted) intake.toggleHoldPosition();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean(getName(), false);
    }

    private void setStallCurrent(double current)
    {
        stallCurrent = current;
    }

    private void setPower(double power)
    {
        this.power = power;
    }
    
    @Override
	public void initSendable( SendableBuilder builder )
	{
        builder.setSmartDashboardType("Command");

        builder.addDoubleProperty("StallCurrent", ()-> stallCurrent, this::setStallCurrent);
		builder.addDoubleProperty("Power", ()-> power, this::setPower);
	}   
}

