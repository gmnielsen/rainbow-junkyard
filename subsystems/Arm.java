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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import static frc.robot.Constants.ArmConstants.kSetPointDelta;


public class Arm extends SubsystemBase {
  SparkMax armMotorLeft, armMotorRight;
  SparkMaxConfig armMotorLeftConfig, armMotorRightConfig;
  DutyCycleEncoder encoder;
  PIDController armP;

  /** Creates a new Arm. */
  public Arm() {
    armMotorLeft = new SparkMax(5, MotorType.kBrushless);
    armMotorRight = new SparkMax(6, MotorType.kBrushless);

    armMotorLeftConfig = new SparkMaxConfig();
    armMotorRightConfig = new SparkMaxConfig();

    armMotorLeftConfig.openLoopRampRate(0.5);
    armMotorLeft.configure(armMotorLeftConfig.
      inverted(false).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);
    
    
    armMotorRightConfig.openLoopRampRate(0.5);
    armMotorRight.configure(armMotorRightConfig.
      follow(armMotorLeft, true).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    encoder = new DutyCycleEncoder(0);
    armP = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);
    armP.setTolerance(.005);
  }

  public boolean isAtSetpoint() {
    return armP.atSetpoint();
  }

  public Command stop() {
    return runOnce(() -> {
      armMotorLeft.set(0);
    });
  }

  public Command climb() {
//    return moveArmToPosition(ArmConstants.positionClimbEnd, 0.5)
//      .until(this::isAtSetpoint)
//      .andThen(stop());
      return moveArmToPosition(ArmConstants.positionClimbEnd, 0.5);

  }

  public Command moveArmToPosition(Double position, double speedMultiplier) {
    return run(
        () -> {
          
          // Get the target position, clamped to (limited between) the lowest and highest arm positions
          double target = MathUtil.clamp(position, ArmConstants.armRearLimit, ArmConstants.armFrontLimit);

          // Calculate the PID result, and clamp to the arm's maximum velocity limit.
          double result =  MathUtil.clamp(armP.calculate(encoder.get(), target), -1 * ArmConstants.armVelocityLimit, ArmConstants.armVelocityLimit);

          armMotorLeft.set(result * speedMultiplier);

        });
  }

  public Command nudgeUp() {
      double currPosition = encoder.get();
      return moveArmToPosition(currPosition - kSetPointDelta, 0.5)
              .until(this::isAtSetpoint)
              .andThen(stop());
  }

  public Command nudgeDown() {
      double currPosition = encoder.get();
      return moveArmToPosition(currPosition + kSetPointDelta, 0.5)
              .until(this::isAtSetpoint)
              .andThen(stop());
  }

  public Command changeSetPoint(Double up, Double down) {
    return run(
      () -> {
        double newSetPoint = armP.getSetpoint();
        System.out.println("triggered");
        System.out.println(armP.getSetpoint());
        if (up > 0){
          //moveArmToPosition(newSetPoint - up, 1);
          //System.out.println("up triggered");
          newSetPoint = newSetPoint - up;
        }
        if (down > 0) {
          //System.out.println("down triggered");
          newSetPoint = newSetPoint + down;
        }
        moveArmToPosition(newSetPoint, 1);
        System.out.println(newSetPoint);
        //moveArmToPosition(armP.getSetpoint());

      }
    );

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armPosition", encoder.get());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
