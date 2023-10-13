package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {
    //ID
    public int moduleID;
    //Motors
    private CANSparkMax DriveMotor;
    private CANSparkMax AngleMotor;
    //Encoders
    private RelativeEncoder DriveEncoder;
    private RelativeEncoder AngleEncoder;
    private CANCoder AngleCANCoder;
    //PIDs
    private SparkMaxPIDController DrivePID;
    private SparkMaxPIDController AnglePID;
    //private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    //Angles
    private Rotation2d v_offsetAngle;
    private Rotation2d v_lastAngle;

    //public SwerveModule()
}
