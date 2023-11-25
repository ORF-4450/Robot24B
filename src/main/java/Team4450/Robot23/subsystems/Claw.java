package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Claw extends SubsystemBase
{
    private WPI_TalonFX     motor = null; //new WPI_TalonFX(CLAW_MOTOR);
    private FXEncoder       encoder = null; //new FXEncoder(motor);

    private final double    CLAW_MAX = 13500;

    public Claw()
    {
        Util.consoleLog();

        // addChild("TalonFX", motor);
        // addChild("Encoder", encoder);

        // motor.setInverted(true);

        // encoder.setInverted(true);

        // Disable automatic application of limit switches connected to the TalonFX controller.
        // The JST wire used to connect the switches to the controller is wired as follows:
        // black = forward switch, red = forward ground, white = reverse ground, yellow =
        // reverse switch. Once enabled the controller automatically obeys the switches.
        // unfortunately we guessed on what "forward/reverse" mean, so the robot is wired for 
        // reverse, which it turns out prevents claw from closing. So we turn auto application 
        // of the switches off and do it in our code below.

        // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        //                                      LimitSwitchNormal.Disabled,
        //                                      30);

        // motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        //                                      LimitSwitchNormal.Disabled,
        //                                      30);
    }

    /**
     * Sets claw motor power.
     * @param power + means open claw, - means close claw.
     */
    public void setPower(double power)
    {
        // If power positive, which means open, check limit switch stop if true.
        // If power negative, which means close, check encoder for max open, stop if there.

        if ((power > 0 && getOpenSwitch()) || (power < 0 && encoder.get() >= CLAW_MAX)) power = 0;

        if (getOpenSwitch()) encoder.reset();

        power = Util.clampValue(power, .20);
        
        motor.set(power);
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
     * Reset Claw encoder to zero.
     */
    public void resetPosition()
    {
        encoder.reset();
    }

    /**
     * Returns claw fully open switch state.
     * @return True when claw fully open.
     */
    public boolean getOpenSwitch()
    {
        if (motor.isRevLimitSwitchClosed() == 1)
            return false;
        else
            return true;
    }

    public void updateDS()
    {

    }
}
