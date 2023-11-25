package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.LimeLight;
import Team4450.Robot23.subsystems.LimeLight.LedMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command uses target information returned by the LimeLight (via
 * network tables API). The targets yaw from the camera and the area of 
 * the target in the field of vision are used by two PID controllers to 
 * move the robot side to side and foward to scoring position. A third 
 * controller uses the NavX yaw to correct any rotation away from facing 
 * the scoring wall.
 */
public class AlignToTapeLL extends Command
{
    private LimeLight               limeLight;
    private DriveBase               driveBase;
    private SynchronousPID          strafeController = new SynchronousPID("AlignToTape", .03, .003, .003);
    private SynchronousPID          throttleController = new SynchronousPID("DriveToTape", 1.0, .1, .01);
    private SynchronousPID          rotateController = new SynchronousPID("RotateToTape", .03, .003, .003);
    private final double            maxSpeed = .20, maxRotate = .10;
    private double                  startTime, lastYaw, lastArea;
    private boolean                 strafeLocked, throttleLocked, noTarget;

    public AlignToTapeLL(LimeLight limeLight, DriveBase driveBase)
    {
        this.limeLight = limeLight;

        strafeController.setOutputRange(-maxSpeed, maxSpeed);
        throttleController.setOutputRange(-maxSpeed, maxSpeed);
        rotateController.setOutputRange(-maxRotate, maxRotate);

        // We want zero yaw to target as reported by LL.
        strafeController.setSetpoint(0);
        strafeController.setTolerance(.25);

        // We want to be close enough so that target is .80% of field
        // of vision as reported by LL.
        throttleController.setSetpoint(.80);
        throttleController.setTolerance(.03);

        rotateController.setSetpoint(0);
        rotateController.setTolerance(.5);

        this.driveBase = driveBase;

        addRequirements(driveBase);
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Select the reflective tape pipeline.
        limeLight.selectPipeline(0);

        limeLight.setLedMode(LedMode.on);

        startTime = Util.timeStamp();
        
        strafeController.reset();
        throttleController.reset();
        rotateController.reset();

        noTarget = strafeLocked = throttleLocked = false;
        lastArea = lastYaw = Double.NaN;

        SmartDashboard.putBoolean("AutoTarget", true);
        SmartDashboard.putBoolean("TargetLocked", false);
    }

    @Override
    public void execute()
    {
        boolean hasTarget;
        double  throttle = 0, strafe = 0;

        // Delay first test for targets to allow LL to get going after we turn on LED.

        if (Util.getElaspedTime(startTime) < 0.75) return;

        // Note: We strafe, that is align our position side to side and correct rotation
        // as a first phase, no throttle (forward). When strafe is completed (we are aligned),
        // then we apply throttle to move forward to to the scoring position. All this because
        // when we were doing all 3 motions simultaneously, we would drive forward before 
        // alignment was finished and hit the platform legs while still moving sideways.

        hasTarget = limeLight.processImage();

        if (hasTarget)
        {
            if (!strafeLocked)
            {
                lastYaw = limeLight.offsetX();

                strafe = -strafeController.calculate(lastYaw);
        
                if (strafeController.onTarget()) 
                {
                    strafe = 0;
                    strafeLocked = true;
                }
            }

            SmartDashboard.putNumber("Strafe Speed", strafe);
    
            if (strafeLocked)
            {
                lastArea = limeLight.getArea();

                throttle = -throttleController.calculate(lastArea);

                if (throttleController.onTarget()) 
                {
                    throttle = 0;
                    throttleLocked =  true;
                }
            } 

            SmartDashboard.putNumber("Throttle Speed", throttle);

            double yaw = RobotContainer.navx.getYaw();

            double rotate = -rotateController.calculate(yaw);

            Util.consoleLog("yaw=%.2f  strafe=%.3f  area=%.2f  throttle=%.3f  navx=%.1f  rot=%.2f", lastYaw, strafe, 
                            lastArea, throttle, yaw, rotate);

            driveBase.drive(throttle, strafe, rotate);
        } else 
            noTarget = true;    // No targets visible.
    }

    @Override
    public boolean isFinished()
    {
        return noTarget | (strafeLocked && throttleLocked);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b, last Yaw=%.2f, area=%.2f, not=%b, lock=%b", interrupted, lastYaw, 
                        lastArea, noTarget, (strafeLocked && throttleLocked));

        driveBase.stop();

        limeLight.setLedMode(LedMode.off);

        SmartDashboard.putBoolean("AutoTarget", false);

        if (strafeLocked && throttleLocked) SmartDashboard.putBoolean("TargetLocked", true);
    }
}
