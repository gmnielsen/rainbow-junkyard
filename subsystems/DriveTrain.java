// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {
  SparkMax leftFront, leftRear, rightFront, rightRear;
  SparkMaxConfig leftFrontConfig, leftRearConfig, rightFrontConfig, rightRearConfig;
  DifferentialDrive drivetrain;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new SparkMax(1, MotorType.kBrushless);
    leftRear = new SparkMax(3, MotorType.kBrushless);
    rightFront = new SparkMax(2, MotorType.kBrushless);
    rightRear = new SparkMax(4, MotorType.kBrushless);

    leftFrontConfig = new SparkMaxConfig();
    leftRearConfig = new SparkMaxConfig();
    rightFrontConfig = new SparkMaxConfig();
    rightRearConfig = new SparkMaxConfig();

    leftFront.configure(leftFrontConfig.
      inverted(false).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    leftRear.configure(leftRearConfig.
      idleMode(IdleMode.kBrake).
      follow(leftFront),
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    rightFront.configure(rightFrontConfig.
      inverted(true).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    rightRear.configure(rightRearConfig.
      idleMode(IdleMode.kBrake).
      follow(rightFront),
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);
    
    drivetrain = new DifferentialDrive(leftFront, rightFront);
    //drivetrain.setDeadband(0.1);
  }


  // GN : added rotation dead zone, 4th power for rotation
  public void arcadeDrive(double speed, double rotation) {
    // x 0.8 because we don't need to go that fast for this year's comp
    drivetrain.arcadeDrive(speed*0.8, rotation*0.8, true);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    drivetrain.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }

  public Command driveTank(DoubleSupplier left, DoubleSupplier right) {
    // Inline construction of command goes here.
    return run(
        () -> {
          leftFront.set(left.getAsDouble());
          rightFront.set(right.getAsDouble());
        });
  }

  public Command moveStraight(Double velocity) {
    return run(
      () -> {
        arcadeDrive(velocity, 0.0);
      });
  }

  public Command turn(Double velocity) {
    return run(
      () -> {
        leftFront.set(velocity);
        rightFront.set(-1 * velocity);
      });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
