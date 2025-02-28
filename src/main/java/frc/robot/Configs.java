package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public class Configs {
     public static final class MAXSwerveModule
    {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    
    static
    {
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDriveWheelFreeSpeedRps;
    double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

    drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
    drivingConfig
        .encoder
        .positionConversionFactor(drivingFactor)
        .velocityConversionFactor(drivingFactor / 60.0);
    drivingConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0, 0)//this may be changed (p, i, g}
        .velocityFF(drivingVelocityFeedForward)
        .outputRange(-1, 1);//(maxOutput=1}

    turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
    turningConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(turningFactor)
        .velocityConversionFactor(turningFactor / 60.0);
    turningConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(1, 0, 0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, turningFactor);
    }
}
}
