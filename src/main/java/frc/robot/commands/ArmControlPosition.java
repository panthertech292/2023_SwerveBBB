// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControlPosition extends CommandBase {
  private final ArmSubsystem ArmSub;
  private double v_target;
  private PIDController armPID;

  /** Creates a new ArmControlPosition. */
  public ArmControlPosition(ArmSubsystem s_ArmSubsystem, double target, double p) {
    ArmSub = s_ArmSubsystem;
    this.v_target = target;
    armPID = new PIDController(p, 0.0008, 0.001);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double v_Speed = armPID.calculate(ArmSub.getArmRotatePosition(), v_target);
    SmartDashboard.putNumber("ARM PID SPEED", v_Speed);
    ArmSub.setArmRotateMotor(v_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotateMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
