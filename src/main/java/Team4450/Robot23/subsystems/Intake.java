package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Intake extends SubsystemBase
{
    private WPI_TalonFX     motor = new WPI_TalonFX(INTAKE_MOTOR);
    private FXEncoder       encoder = new FXEncoder(motor);

    private final double    MAX_POWER = .50;
    private double          maxCurrent, lastTimeCalled, lastPower = -.15, maxTemp;
    private boolean         holdPosition;

    private SynchronousPID  controller = new SynchronousPID(getName() + "Hold", 0.0001, 0, 0);

    public Intake()
    {
        Util.consoleLog();

        motor.setNeutralMode(NeutralMode.Brake);
        
        addChild("TalonFX", motor);
        addChild("Encoder", encoder);
        addChild(controller.getName(), controller);
    }

    @Override
    public void periodic()
    {
        double current = motor.getStatorCurrent();

        if (current > maxCurrent) maxCurrent = current;

        if (motor.getTemperature() > maxTemp) maxTemp = motor.getTemperature();
        
        // Periodic function called on each scheduler loop so we can use
        // it to run the pid controller to hold position.

        if (holdPosition && robot.isDisabled()) toggleHoldPosition();   // Turn off hold when disabled.

        if (holdPosition)
        {
            double time = Util.getElaspedTime(lastTimeCalled);

            lastTimeCalled = Util.timeStamp();
    
            double power = controller.calculate(getPosition(), time);
    
            motor.set(power);
        }
    }
    
    /**
     * Sets motor power.
     * @param power + means intake cube, - means intake cone.
     */
    public void setPower(double power)
    {
        // If holding position, ignore zero power, non zero power turns off hold.

        if (holdPosition && power != 0) toggleHoldPosition();

        if (holdPosition) return;

        power = Util.clampValue(power, MAX_POWER);
        
        motor.set(power);

        lastPower = power;
    }

    public void dropGamePiece()
    {
        Util.consoleLog("hold=%b", holdPosition);

        if (!holdPosition) return;

        setPower(-lastPower);
        
        try {
            Thread.sleep(400);
        }
        catch (Exception e) {}

        stop();
    }
   

    public void stop()
    {
        motor.stopMotor();
    }

    /**
     * Return encoder tick count.
     * @return The current tick count.
     */
    public int getPosition()
    {
        return encoder.get();
    }

    /**
     * Reset encoder to zero.
     */
    public void resetPosition()
    {
        encoder.reset();
    }

    /**
     * Return current motor temperature.
     * @return Temperature degrees F.
     */
    public double getTemperature()
    {
        return (motor.getTemperature() * 1.8) + 32;
    }

    /**
     * Return max temp seen since last restart of robot.
     * @return Temperature in F.
     */
    public double getMaxTemperature()
    {
        return (maxTemp * 1.8) + 32;
    }

    /**
     * Return the motor's current draw.
     * @return Current draw in amps.
     */
    public double getMotorCurrent()
    {
        return motor.getStatorCurrent();
    }

    /**
     * Returns the max motor current seen since last reset.
     * @return Max current in amps.
     */
    public double getMaxMotorCurrent()
    {
        return maxCurrent;
    }

    /**
     * Reset max current tracking.
     */
    public void resetMaxCurrent()
    {
        maxCurrent = 0;
    }

    public boolean isHoldingPosition()
    {
        return holdPosition;
    }

    /**
     * Starts or stop intake position hold function. Non zero input
     * power also turns off hold function.
     */
    public void toggleHoldPosition()
    {
        if (!holdPosition) 
        {
            holdPosition = true;
            
            controller.reset();

            controller.setSetpoint(getPosition());
    
            lastTimeCalled = Util.timeStamp();
        }
        else
        {
            holdPosition = false;

            Util.consoleLog("amps=%.1f max=%.1f  temp=%.0f max=%.0f", getMotorCurrent(), getMaxMotorCurrent(),
                            getTemperature(), getMaxTemperature());
        }

        Util.consoleLog("%b", holdPosition);

        updateDS();
    }

    public void updateDS()
    {
        SmartDashboard.putBoolean("HoldIntake", holdPosition);
    }
        
    @Override
	public void initSendable( SendableBuilder builder )
	{
        super.initSendable(builder);

		builder.addDoubleProperty("1 Motor Current", this::getMotorCurrent, null);
		builder.addDoubleProperty("2 Max Motor Current", this::getMaxMotorCurrent, null);
		//builder.addDoubleProperty("3 Motor temp", this::getTemperature, null);
		//builder.addDoubleProperty("4 Max Motor temp", this::getMaxTemperature, null);
	}   	
}
