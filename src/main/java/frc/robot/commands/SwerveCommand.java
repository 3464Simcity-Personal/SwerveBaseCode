// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {
  private SwerveSubsystem swerveSub;
  private PIDController rotationController;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier dampenSup;
  private DoubleSupplier dynamicHeadingSup;

  /** Creates a new SwerveCommand. */
  public SwerveCommand(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier dynamicHeadingSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSub = SwerveSubsystem.getInstance();
    addRequirements(swerveSub);

    rotationController = new PIDController(Constants.SwerveConstants.headingKP, Constants.SwerveConstants.headingKI, Constants.SwerveConstants.headingKD );
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Constants.SwerveConstants.headingTolerence);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.dampenSup = dampen;
    this.dynamicHeadingSup = dynamicHeadingSup;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
