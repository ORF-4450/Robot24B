package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Winch  extends SubsystemBase
{
    private CANSparkMax     motor = new CANSparkMax(WINCH_MOTOR, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    //private DigitalInput    lowerLimitSwitch = new DigitalInput(WINCH_SWITCH_LOWER);
    private DigitalInput    upperLimitSwitch = new DigitalInput(WINCH_SWITCH_UPPER);
    private boolean         holdPosition;
    private SynchronousPID  controller = new SynchronousPID(getName() + "Hold", 0.2, 0, 0);
    private final double    PID_MAXPOWER = .10;
    private double          lastTimeCalled;

    public final double     WINCH_MAX = -111;        // Revolutions.
    private final double    WINCH_MAX_POWER = .70;
    
    public Winch()
    {
        Util.consoleLog();

        addChild("UpperLimitSwitch", upperLimitSwitch);

        // Winch will start at max up position so that is encoder zero. Encoder max will
        // be winch at lowest position.

        encoder.setPosition(0);
    
        controller.setOutputRange(-PID_MAXPOWER, PID_MAXPOWER);

        addChild(controller.getName(), controller);
    }

    /**
     * Set winch power.
     * @param power + is up, - is down.
     */
    public void setPower(double power)
    {
        // If holding position, ignore zero power, non zero power turns off hold.

        if (holdPosition && power != 0) toggleHoldPosition();

        if (holdPosition) return;

        // If power negative, which means go down, check encoder limit.
        // If power positive, which means go up, check limit switch for max height, stop if true.
        // Note we do not reset encoder at lower limit, only high limit and the encoder counts -
        // as we lower from high position.

        if (power > 0 && upperLimitSwitch.get()) power = 0;

        if (power < 0 && encoder.getPosition() <= WINCH_MAX) power = 0;

        if (upperLimitSwitch.get()) encoder.setPosition(0);

        power = Util.clampValue(power, WINCH_MAX_POWER);

        motor.set(power);
    }

    @Override
    public void periodic()
    {
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

    public void stop()
    {
        motor.stopMotor();
    }

    /**
     * Return Winch encoder position. Note winch encoder counts up (+) when power
     * is + (going up) and down (-) when power is - (going down).
     * @return Position in revolutions.
     */
    public double getPosition()
    {
        return encoder.getPosition();
    }

    /**
     * Reset Winch encoder to zero. Only reset at high position.
     */
    public void resetPosition()
    {
        encoder.setPosition(0);
    }

    /**
     * Returns state of lower position limit switch.
     * @return True is at low position.
     */
    // public boolean getLowerSwitch()
    // {
    //     return lowerLimitSwitch.get();
    // }

    /**
     * Returns state of upper position limit switch.
     * @return True is at high position.
     */
    public boolean getUpperSwitch()
    {
        return upperLimitSwitch.get();
    }

    /**
     * Starts or stop winch position hold function. Non zero input
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
            holdPosition = false;

        Util.consoleLog("%b", holdPosition);

        updateDS();
    }

    public void updateDS()
    {
        SmartDashboard.putBoolean("HoldWinch", holdPosition);
    }
}