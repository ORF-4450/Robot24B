
package Team4450.Robot23.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import Team4450.Lib.Swerve.Mk4iSwerveModuleHelper;
import Team4450.Lib.Swerve.ModuleConfiguration;
import Team4450.Lib.Swerve.SdsModuleConfigurations;
import Team4450.Lib.Swerve.SwerveModule;
import Team4450.Lib.Swerve.ModuleConfiguration.ModulePosition;
import Team4450.Robot23.RobotContainer;
import Team4450.Lib.Util;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import static Team4450.Robot23.Constants.*;

public class DriveBase extends SubsystemBase 
{
  private boolean       autoReturnToZero = false, fieldOriented = true, currentBrakeMode = false;
  private double        distanceTraveled;
  private double        yawAngle, lastYawAngle;
  private Pose2d        lastPose;

  private SimDouble     simAngle; // navx sim.

  //  Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
  //                   SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */

  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0);
          
  private static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  
  //private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final AHRS m_navx = RobotContainer.navx.getAHRS();

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  SwerveModulePosition modulePositions[] = new SwerveModulePosition[]
      { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition() }; 

  // TODO: Fix the vectors used to set std deviations for measurements. Using default
  // for now. Not sure how to determine the values.
  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      m_kinematics,
      getGyroRotation2d(),
      modulePositions,
      new Pose2d());
      // VecBuilder.fill(0.1, 0.1, 0.1),
      // VecBuilder.fill(0.05),
      // VecBuilder.fill(0.1, 0.1, 0.1));

  // Field2d object creates the field display on the simulation and gives us an API
  // to control what is displayed (the simulated robot).

  private final Field2d     field2d = new Field2d();

  public DriveBase() 
  {
    Util.consoleLog();

    // This thread will wait a bit and then reset the gyro while this constructor
    // continues to run. We do this because we have to wait a bit to reset the
    // gyro after creating it.

    new Thread(() -> {
      try {
        Thread.sleep(2000);
        zeroGyro();
        //m_navx.reset();
      } catch (Exception e) { }
    }).start();

    // Creates a tab on our driver station dashboard used by the swerve code to
    // display setup informmation.

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    if (RobotBase.isSimulation()) 
    {
      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    SmartDashboard.putData("Field2d", field2d);

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk4iSwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk4iSwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk4iSwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk4iSwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // NOTE: By default the swerve modules are created with the default Mk4ModuleConfiguration object.
    // The Mk4ModuleConfiguration object contains all of the configurable tuning parameters available for
    // modules. If you wish to adjust this configuration, create a Mk4MmoduleConfiguration object here and
    // call it's methods to set the parameters you wish to adjust and then pass that configuration object 
    // to each of the createNeo calls below, adding the configuration object just ahead of the GearRatio
    // parameter. The default Mk4ModuleConfiguration is currently customized for Neos.
    
    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.FL,
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kGrid)
                    .withSize(2, ModuleConfiguration.shuffleBoardRows) // 2,
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // Sets the module center translation from center of robot.
    m_frontLeftModule.setTranslation2d(new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    // We will do the same for the other modules

    m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.FR,
            tab.getLayout("Front Right Module", BuiltInLayouts.kGrid)
                    .withSize(2, ModuleConfiguration.shuffleBoardRows)
                    .withPosition(2, 0), // 2, 0
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_frontRightModule.setTranslation2d(new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.BL,
            tab.getLayout("Back Left Module", BuiltInLayouts.kGrid)
                    .withSize(2, ModuleConfiguration.shuffleBoardRows) // 2,4
                    .withPosition(4, 0),  // 4, 0
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backLeftModule.setTranslation2d(new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.BR,
            tab.getLayout("Back Right Module", BuiltInLayouts.kGrid)
                    .withSize(2, ModuleConfiguration.shuffleBoardRows) // 2,4
                    .withPosition(6, 0),  // 6,0
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    m_backRightModule.setTranslation2d(new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    // Set starting brake mode. Set by definition above. The swerve code has a built in default
    // of brakes on but turning brakes off is much better when battery is low.

    setBrakeMode(currentBrakeMode);

    // Encoders to zero.
    resetModuleEncoders();

    // Set default starting position on field.
    setOdometry(DEFAULT_STARTING_POSE);

    // Initialze drive code by issuing a no movement drive command.
    drive(0, 0, 0);
    
    updateDS();
  }

  /**
   * Sets the gyroscope yaw angle to zero. This can be used to set the direction 
   * the robot is currently facing to the 'forwards' direction.
   */
  public void zeroGyro() 
  {
    Util.consoleLog();

    m_navx.zeroYaw();
  }

  /**
   * Get gyro yaw from the angle of the robot at last gyro reset.
   * @return Gyro yaw in radians. + is left of zero (ccw) - is right (cw).
   */
  public Rotation2d getGyroRotation2d() 
  {
    if (m_navx.isMagnetometerCalibrated()) 
    {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    return Rotation2d.fromDegrees(-m_navx.getYaw());
  }

  /**
   * Get gyro yaw from the angle of the robot at last gyro reset.
   * @return Gyro yaw. + is left of zero (ccw) - is right (cw).
   */
  public double getGyroYaw()
  {
    return -m_navx.getYaw();
  }

  /**
   * Get Gyro angle in degrees.
   * @return Angle in degrees. 0 to +- 180.
   */
  public double getGyroAngleDegrees() 
  {
    return Math.IEEEremainder((-m_navx.getAngle()), 360);
  }

  /**
   * Get Gyro Angle in radians.
   * @return Angle as rotation2d (radians).
   */
  public Rotation2d getGyroAngleRotation2d() 
  {
    return Rotation2d.fromDegrees(getGyroAngleDegrees());
  }

  /**
   * With the supplied directional % values, generates a Chassis Speeds object which
   * is used to command the swerve modules to move. This is how we drive
   * the robot. The motion specified continues until drive is called again.
   * This method is typically called by the driving command passing inputs (typically
   * joystick inputs) on a regular basis.
   * @param throttle Throttle (Fwd/Back) as a % of maximum speed. + is forward.
   * @param strafe Strafe (Left/Right) as a % of maximum speed. + is left.
   * @param rotation Rotational speed as a % of maximum rotational speed. + is left.
   */
  public void drive(double throttle, double strafe, double rotation)
  {
    // Convert joystick % values into speeds.

    throttle *= MAX_VELOCITY_METERS_PER_SECOND;
    strafe   *= MAX_VELOCITY_METERS_PER_SECOND;
    rotation *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Create chassis speeds in either field or robot drive orientation.

    m_chassisSpeeds = fieldOriented
        ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getGyroAngleRotation2d())
        : new ChassisSpeeds(throttle, strafe, rotation);

    //LCD.printLine(4, "max vel=%.3fms  max ang vel=%.3frs  voltage=%.1f",
    //    MAX_VELOCITY_METERS_PER_SECOND,
    //    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //    MAX_VOLTAGE);

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    setModuleStates(states);
  }

  /**
   * Drive the robot with a chassis speeds object instead of module states. This method
   * is intended to support the PathPlanner swerve controller command.
   * @param speeds Speeds output by PPSwerveControllerCommand.
   */
  public void setSpeeds(ChassisSpeeds speeds)
  {
    Util.consoleLog("vxt=%.4f  vys=%.4f  vr=%.4f", speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 
                                                   speeds.omegaRadiansPerSecond);
    
    // Again with the need to invert rotation under sim for some reason...

    if (RobotBase.isSimulation())
      m_chassisSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 
                                          speeds.omegaRadiansPerSecond * -1);
    else
      m_chassisSpeeds = speeds;

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    setModuleStates(states);
  }

  /**
   * Set the swerve modules to desired states.
   * @param desiredStates Array of module states.
   */
  public void setModuleStates(SwerveModuleState[] states)
  {
    
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // Set the drive speed and direction of swerve module. Note that speed, as a % of max speed,
    // is coverted to voltage as the same % of max voltage.
    
    if (!autoReturnToZero && states[0].speedMetersPerSecond < 0.01)
        m_frontLeftModule.stop();
    else
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                              states[0].angle.getRadians(), states[0].speedMetersPerSecond);
        
    if (!autoReturnToZero && states[1].speedMetersPerSecond < 0.01)
        m_frontRightModule.stop();
    else
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                               states[1].angle.getRadians(), states[1].speedMetersPerSecond);
  
    if (!autoReturnToZero && states[2].speedMetersPerSecond < 0.01)
        m_backLeftModule.stop();
    else
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                             states[2].angle.getRadians(), states[2].speedMetersPerSecond);

    if (!autoReturnToZero && states[3].speedMetersPerSecond < 0.01)
        m_backRightModule.stop();
    else
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                              states[3].angle.getRadians(), states[3].speedMetersPerSecond);
  }

  /**
   * Called on every scheduler loop. Updates odometry tracking of
   * robot movement. Also updates robot position on field2d widget
   * used for simulation.
   */
  @Override
  public void periodic() 
  {    
    updateOdometry();

    field2d.setRobotPose(getRobotPose());

    setField2dModulePoses();
  }

  /**
   * Update robot pose (position & rotation) on the field in the odometry tracking
   * object. Used to drive the field2d object.
   */
  public void updateOdometry()
  {
    modulePositions[0] = m_frontLeftModule.getFieldPosition();
    modulePositions[1] = m_frontRightModule.getFieldPosition();
    modulePositions[2] = m_backLeftModule.getFieldPosition();
    modulePositions[3] = m_backRightModule.getFieldPosition();
    
    m_odometry.update(getGyroAngleRotation2d(), modulePositions);

    // Track the distance traveled by robot to support simulation of
    // a regular encoder. We do this by looking at the change in robot
    // pose and using the change in X,Y to compute the change in distance
    // traveled and tracking that. NOTE: This currently only works for
    // movements only in the X or Y direction. It will not be correct
    // for diagonal moves. It also does not track rotations as distance
    // traveled as technically a rotation is not moving a distance. This
    // is all a kludge to support the simple autonomus functions.

    Pose2d currentPose = m_odometry.getEstimatedPosition();

    Transform2d poseOffset = currentPose.minus(lastPose);
    
    lastPose = currentPose;
    
    double currentDistance = poseOffset.getX() + poseOffset.getY();
    //double currentDistance = Math.sqrt(Math.pow(poseOffset.getX(), 2) + Math.pow(poseOffset.getY(), 2));

    distanceTraveled += currentDistance;

    SmartDashboard.putNumber("Distance Traveled(m)", distanceTraveled);
    
    // Track gyro yaw to support simulation of resettable yaw.

    yawAngle += m_navx.getAngle() - lastYawAngle;

    lastYawAngle = m_navx.getAngle();

    SmartDashboard.putNumber("Yaw Angle", getYaw());

    // Now update the pose of each wheel (module).
    updateModulePose(m_frontLeftModule);
    updateModulePose(m_frontRightModule);
    updateModulePose(m_backLeftModule);
    updateModulePose(m_backRightModule);
  }

  /**
   * Update the pose of a swerve module on the field2d object. Module
   * pose is connected to the robot pose so they move together on the
   * field simulation display.
   * @param module Swerve module to update.
   */
  private void updateModulePose(SwerveModule module)
  {
    Translation2d modulePosition = module.getTranslation2d()
        //.rotateBy(getHeadingRotation2d())
        .rotateBy(getRobotPose().getRotation())
        .plus(getRobotPose().getTranslation());
    
    module.setModulePose(
        new Pose2d(modulePosition, module.getHeadingRotation2d().plus(getGyroAngleRotation2d())));
  }

  /**
   * Rotates the module icons on the field display so indicate where
   * the wheel is pointing.
   */
  private void setField2dModulePoses()
  {
    Pose2d      modulePoses[] = new Pose2d[4];
    
    modulePoses[0] = m_frontLeftModule.getPose();
    modulePoses[1] = m_frontRightModule.getPose();
    modulePoses[2] = m_backLeftModule.getPose();
    modulePoses[3] = m_backRightModule.getPose();

    field2d.getObject("Swerve Modules").setPoses(modulePoses);
  }

  /**
   * Returns the estimated position of the robot on the field
   * as determined by the odometry tracking object.
   * @return Position on the field and direction robot is pointing. X, Y 
   * in meters, rotation as a rotation2D.
   */
  public Pose2d getRobotPose() 
  {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Returns the odometry tracking object.
   * @return The odometry tracking object.
   */
  public SwerveDrivePoseEstimator getOdometry() 
  {
    return m_odometry;
  }

  /**
   * Set the odometry tracking to a new position.
   * @param pose The new position (x,y,angle).
   */
  public void setOdometry(Pose2d pose) 
  {
    Util.consoleLog("Pose: x=%.3f  y=%.3f  rot=%.2f deg", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

    modulePositions[0] = m_frontLeftModule.getFieldPosition();
    modulePositions[1] = m_frontRightModule.getFieldPosition();
    modulePositions[2] = m_backLeftModule.getFieldPosition();
    modulePositions[3] = m_backRightModule.getFieldPosition();

    m_odometry.resetPosition(getGyroAngleRotation2d(), modulePositions, pose);

    lastPose = pose;

    m_navx.reset();
  }  

  /**
   * Called on every scheduler loop when in simulation.
   */
  @Override
  public void simulationPeriodic() 
  {
    // We are not using this call now because the REV simulation does not work
    // correctly. Will leave the code in place in case this issue gets fixed.
    // Assumes Neos. SIM for 500s not implemented.
    //if (robot.isEnabled()) REVPhysicsSim.getInstance().run();
 
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = m_chassisSpeeds.omegaRadiansPerSecond * 1.1459155;

    temp += simAngle.get();

    simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  /**
   * Toggle the auto return to zero flag. Auto return to zero 
   * automatically retuns the wheels to straight ahead when 
   * joystick returns to neutral position.
   */
  public void toggleAutoReturnToZero()
  {
    Util.consoleLog();
    
     autoReturnToZero = !autoReturnToZero;

     updateDS();
  }

  /**
   * Return state of the wheel auto return to zero (or straight ahead)
   * flag. Auto return to zero sets the wheels back to straight ahead
   * when joysticks return to neutral position.
   * @return True if auto return to zero.
   */
  public boolean getAutoReturnToZero()
  {
     return autoReturnToZero;
  }

  /**
   * Toggle the drive mode between field or robot oriented.
   */
  public void toggleFieldOriented()
  {
      Util.consoleLog();
    
      fieldOriented = !fieldOriented;

      updateDS();
  }

  /**
   * Return the drive mode status.
   * @return True if field oriented, false if robot oriented.
   */
  public boolean getFieldOriented()
  {
      return fieldOriented;
  }

  private void updateDS()
  {
      SmartDashboard.putBoolean("Field Oriented", fieldOriented);
      SmartDashboard.putBoolean("Auto Return To Zero", autoReturnToZero);
      SmartDashboard.putBoolean("Brakes", currentBrakeMode);
  }

  /**
   * Set modules driving and steering encoders to zero. Should only be called
   * at initialization. Do not call after driving starts as it will crash the
   * swerve driving code.
   */
  public void resetModuleEncoders() 
  {
      Util.consoleLog();
    
      m_frontLeftModule.resetMotorEncoders(); 
      m_frontRightModule.resetMotorEncoders(); 
      m_backLeftModule.resetMotorEncoders(); 
      m_backRightModule.resetMotorEncoders(); 
  }
  
  /**
   * Not currently used.
   */
  // public void setModulesToAbsolute() 
  // {
  //     Util.consoleLog();

  //     m_frontLeftModule.resetSteerAngleToAbsolute();
  //     m_frontRightModule.resetSteerAngleToAbsolute();
  //     m_backLeftModule.resetSteerAngleToAbsolute();
  //     m_backRightModule.resetSteerAngleToAbsolute();
  // }

  /**
   * Set modules to point "forward". This is same as joystick
   * to neutral position with auto return to zero set true. The
   * calling command must require the drive base and should run
   * for 1 second so it prevents the DriveCommand from stopping
   * wheel turn prematurely.
   * TODO: Not currently working. Don't know why.
   */
  public void setModulesToForward()
  {
      Util.consoleLog();

      boolean saveARZ = autoReturnToZero;

      autoReturnToZero = true;
      
      drive(0, 0, 0);

      autoReturnToZero = saveARZ;
  }

  /**
   * Set modules to point in the "start" position. This is straight
   * forward, bevel gear on left side. This the wheel position the
   * code expects at start up. You can call this after driving and
   * usually it works to reorient the robot but not always. This
   * is mainly to get the wheels aligned for start of match.
   */
  public void setModulesToStartPosition()
  {
    Util.consoleLog();

    m_frontLeftModule.setStartingPosition();
    m_frontRightModule.setStartingPosition();
    m_backLeftModule.setStartingPosition();
    m_backRightModule.setStartingPosition();

    m_navx.reset(); // or zeroGyro();
    
    // Reseting encoders here seems logical...but in testing it did not
    // work as expected. Now, if you set to start position and disable
    // enable, you can continue driving. If you reset the encoders here
    // and disable/enable, the drive code is confused. Does not make
    // sense and suggests somewhere in the swerve code it is tracking
    // drive information that is not being reset and so with encoders
    // reset, the two bits of information are no longer in sync.
    // TODO: Research this further trying to explain why resetting the
    // encoders here does not work.
    //resetModuleEncoders();
  }

  /**
   * Gets the distance traveled by the robot drive wheels since the
   * last call to resetDistanceTraveled. This simulates a regular
   * encoder on the drive wheel. We can't use the actual wheel encoder
   * because resetting that encoder would crash the swerve drive code.
   * Note: This distance is only accurate for forward/backward and
   * strafe moves. 
   * @return
   */
  public double getDistanceTraveled()
  {
    return distanceTraveled; // * -1;
  }

  /**
   * Reset the distance traveled by the robot.
   */
  public void resetDistanceTraveled()
  {
    distanceTraveled = 0;
  }

  /**
   * Returns the current yaw angle of the robot measured from the last
   * call to resetYaw(). Angle sign is WPILib convention, inverse of NavX.
   * @return The yaw angle. - is right (cw) + is left (ccw).
   */
  public double getYaw()
  {
    return -yawAngle;
  }

  /**
   * Returns the current yaw angle of the robot measured from the last
   * call to resetYaw().
   * @return The yaw angle in radians.
   */
  public double getYawR()
  {
    return Math.toRadians(-yawAngle);
  }

  /**
   * Set yaw angle to zero.
   */
  public void resetYaw()
  {
    yawAngle = 0;
  }

  /**
   * Stop robot motion.
   */
  public void stop()
  {
    drive(0, 0, 0);
  }

  /**
   * Returns the swerve drive kinematics object.
   * @return The swerve kinematics object.
   */
  public static SwerveDriveKinematics getKinematics()
  {
    return m_kinematics;
  }

  /**
   * Set drive motor idle mode for each swerve module. Defaults to brake.
   * @param on True to set idle mode to brake, false sets to coast.
   */
  public void setBrakeMode(boolean on) 
  {
      Util.consoleLog("%b", on);

      currentBrakeMode = on;
    
      m_frontLeftModule.setBrakeMode(on); 
      m_frontRightModule.setBrakeMode(on); 
      m_backLeftModule.setBrakeMode(on); 
      m_backRightModule.setBrakeMode(on); 

      updateDS();
  }

  /**
   * Toggles state of brake mode (brake/coast) for drive motors.
   */
  public void toggleBrakeMode()
  {
    setBrakeMode(!currentBrakeMode);
  }
}