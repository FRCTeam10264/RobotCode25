package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase
    {
        private final MaxSwerveModule m_frontLeft = 
        new MaxSwerveModule
        (
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset
        );
    
        private final MaxSwerveModule m_frontRight = 
        new MaxSwerveModule
        (
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset
        );
        private final MaxSwerveModule m_rearLeft = 
        new MaxSwerveModule
        (
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset
        );
        private final MaxSwerveModule m_rearRight = 
        new MaxSwerveModule
        (
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset
        );
    
    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

SwerveDriveOdometry m_odometry = new SwerveDriveOdometry
    (
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[]
        {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    
    public DriveSubsystem() {}

    @Override
    public void periodic()
    {
        m_odometry.update
        (
            Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
            new SwerveModulePosition[]
            {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition(),
            }
        );
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose)
    {
        m_odometry.resetPosition
        (
            Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
            new SwerveModulePosition[]
            {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
        pose);
    }


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
    {
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates
        (
            fieldRelative
            ?
                ChassisSpeeds.fromFieldRelativeSpeeds
                (
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds
        (
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            m_rearLeft.setDesiredState(swerveModuleStates[2]);
            m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public Command setXComand()
    {
        return this.run
        (
            () -> {
                m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
                m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
                m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            }
        );
    }

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds
        (
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        );
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates [1]);
        m_rearLeft.setDesiredState(desiredStates [2]);
        m_rearRight.setDesiredState(desiredStates [3]);
    }

 public void resetEncoder()
 {
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_rearLeft.resetEncoder();
    m_rearRight.resetEncoder();
 }

public Command zeroHeadingCommand()
{
    return this.runOnce(() -> m_gyro.reset());
}

public double getHeading()
{
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
}

public double getTurnRate()
{
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}

}