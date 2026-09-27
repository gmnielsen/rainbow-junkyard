// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Climber extends SubsystemBase {
    SparkMax climberMotor;
    SparkMaxConfig climberMotorConfig;

    RelativeEncoder encoder;

    private final static double DEFAULT_SPEED = 0.25;

    /**
     * Creates a new Climber.
     */
    public Climber() {
        climberMotor = new SparkMax(9, MotorType.kBrushless);

        climberMotorConfig = new SparkMaxConfig();
        climberMotorConfig.encoder.positionConversionFactor(1 / (81.0*5));

        climberMotor.configure(climberMotorConfig.
                        inverted(false).
                        idleMode(IdleMode.kBrake).
                        smartCurrentLimit(20),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        encoder = climberMotor.getEncoder();
        encoder.setPosition(0);
    }

    public Command moveClimber(Double velocity) {
        // Inline construction of command goes here.
        return run(
                () -> {
                    climberMotor.set(velocity);
                });
    }

    public Command moveClimberTo(double distance, double timeout, double velocity) {

        return moveClimber(velocity)
                .until(() -> MathUtil.isNear(distance, encoder.getPosition(), .005))
                .andThen(stop())
                .withTimeout(timeout);
    }


    public Command grab() {
        return moveClimberTo(0.12, 2, DEFAULT_SPEED);
    }

    public Command release() {
        return moveClimberTo(0, 2, -1 * DEFAULT_SPEED);
    }

    public Command stop() {
        return runOnce(() -> {
            climberMotor.set(0);
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("clawPosition", encoder.getPosition());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
