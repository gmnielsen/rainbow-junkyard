// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Intake extends SubsystemBase {

  private static final int CURRENT_LIMIT = 40;
  private static final int CURRENT_THRESHOLD = 25;
  private static final long CURRENT_DURATION_THRESHOLD_NANOS = 100000;

  SparkMax intakeMotorLeft, intakeMotorRight;
  SparkMaxConfig intakeMotorLeftConfig, intakeMotorRightConfig;
  boolean intakeActive;
  long overCurrentStart;


  /** Creates a new Intake. */
  public Intake() {
    intakeActive = false;
    overCurrentStart = 0;

    intakeMotorLeft = new SparkMax(7, MotorType.kBrushless);
    intakeMotorRight = new SparkMax(8, MotorType.kBrushless);

    intakeMotorLeftConfig = new SparkMaxConfig();
    intakeMotorRightConfig = new SparkMaxConfig();

    intakeMotorLeft.configure(intakeMotorLeftConfig.
      inverted(false).
      idleMode(IdleMode.kBrake).
      smartCurrentLimit(CURRENT_LIMIT).
      disableFollowerMode(), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    intakeMotorRight.configure(intakeMotorRightConfig.
      idleMode(IdleMode.kBrake).
      smartCurrentLimit(CURRENT_LIMIT).
      follow(intakeMotorLeft.getDeviceId()), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);
  }

  public Command moveIntake(Double velocity) {
    // Inline construction of command goes here.
    return run(
        () -> {
          
          intakeMotorLeft.set(velocity);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftCurrentDraw = intakeMotorLeft.getOutputCurrent();
    double rightCurrentDraw = intakeMotorRight.getOutputCurrent();

    double maxCurrent = Math.max(leftCurrentDraw, rightCurrentDraw);
    SmartDashboard.putNumber("Left Intake Current", leftCurrentDraw);
    SmartDashboard.putNumber("Right Intake Current", rightCurrentDraw);
    SmartDashboard.putBoolean("Intake Active", intakeActive);

    if (intakeActive) {
      if (maxCurrent > CURRENT_THRESHOLD) {
        if (overCurrentStart == 0) {
          overCurrentStart = System.nanoTime();          
        } else {
          long overCurrentNanos = System.nanoTime() - overCurrentStart;
          if (overCurrentNanos >= CURRENT_DURATION_THRESHOLD_NANOS) {
            stopIntake();
            SmartDashboard.putNumber("Time Above CurrentThreshold", overCurrentNanos / 1000000);
            SmartDashboard.putBoolean("Intake Auto-Stop", true);
          }
        }
      } else {
        overCurrentStart = 0;
        SmartDashboard.putBoolean("Intake Auto-Stop", false);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stopIntake() {
    intakeMotorLeft.set(0);
    intakeMotorRight.set(0);
    intakeActive = false;
    overCurrentStart = 0;
  }
}
