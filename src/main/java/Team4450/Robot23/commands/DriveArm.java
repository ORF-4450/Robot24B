package Team4450.Robot23.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import static Team4450.Robot23.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveArm extends Command 
{
    private final Arm               arm;
    private final XboxController    controller;

    private final DoubleSupplier armSupplier;

    public DriveArm(Arm arm, DoubleSupplier armSupplier, XboxController controller)
    {
        Util.consoleLog();

        this.arm = arm;
        this.armSupplier = armSupplier;
        this.controller = controller;

        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
    }
    
    @Override
    public void execute()
    {
        double power = deadband(armSupplier.getAsDouble(), THROTTLE_DEADBAND);

        //power = Util.squareInput(power);

        if (controller.getAButton())
            arm.setPowerNoLimit(power);
        else
            arm.setPower(power);
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
