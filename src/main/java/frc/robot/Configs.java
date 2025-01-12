package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            drivingConfig.encoder
                    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor) // meters
                    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .velocityFF(ModuleConstants.kDrivingFF)
                    .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(ModuleConstants.kTurningEncoderInverted)
                    .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor) // radians
                    .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kTurningP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(ModuleConstants.kTurningPositionWrapping)
                    .positionWrappingInputRange(ModuleConstants.kTurningPositionPIDMinInput, ModuleConstants.kTurningPositionPIDMaxInput);
        }
    }
}
