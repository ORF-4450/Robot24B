package Team4450.Robot23.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;

import static Team4450.Robot23.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveClaw extends Command 
{
    private final Claw           claw;

    private final DoubleSupplier clawSupplier;

    public DriveClaw(Claw claw, DoubleSupplier clawSupplier)
    {
        Util.consoleLog();

        this.claw = claw;
        this.clawSupplier = clawSupplier;

        addRequirements(claw);
    }

    @Override
    public void execute()
    {
        double power = deadband(clawSupplier.getAsDouble(), THROTTLE_DEADBAND);

        power = Util.squareInput(power);

        claw.setPower(power);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }

    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}
