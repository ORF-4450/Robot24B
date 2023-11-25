package Team4450.Robot23.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import static Team4450.Robot23.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveWinch extends Command 
{
    private final Winch           winch;

    private final DoubleSupplier winchSupplier;

    public DriveWinch(Winch winch, DoubleSupplier winchSupplier)
    {
        Util.consoleLog();

        this.winch = winch;
        this.winchSupplier = winchSupplier;

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
    }

    @Override
    public void execute()
    {
        double power = deadband(winchSupplier.getAsDouble(), THROTTLE_DEADBAND);

        power = Util.squareInput(power);

        winch.setPower(power);
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
