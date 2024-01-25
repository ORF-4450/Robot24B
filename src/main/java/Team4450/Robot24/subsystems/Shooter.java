package Team4450.Robot24.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import Team4450.Robot24.Constants.GeneralConstants;

public class Shooter extends SubsystemBase {
    private CANSparkMax motor1 = new CANSparkMax(GeneralConstants.kShooterMotor1CanId, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(GeneralConstants.kShooterMotor2CanId, MotorType.kBrushless);

    private double  motorSpeed = 1;
    private boolean isrunning = false;

    public Shooter() {
        motor1.setInverted(true);

        motor2.follow(motor1, true);

        SmartDashboard.putNumber("Shooter speed", motorSpeed);

        Util.consoleLog("Shooter created!");
    }

    @Override
    public void periodic() {
        motorSpeed = SmartDashboard.getNumber("Shooter speed", motorSpeed);

        if (isrunning) motor1.set(motorSpeed);
    }

    public void start() {
        Util.consoleLog();

        isrunning = true;

        motor1.set(motorSpeed);

        updateDS();
    }

    public void stop() {
        Util.consoleLog();

        isrunning = false;

        motor1.stopMotor();

        updateDS();
    }

    private void updateDS()
    {
        SmartDashboard.putBoolean("Shooter", isrunning);
    }
}