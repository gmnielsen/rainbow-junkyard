// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {

    public static Command doNothing(DriveTrain drive) {
        return drive.moveStraight(0d);
    }

    public static Command leaveStartZone(DriveTrain drive) {
        return drive.moveStraight(0.25).withTimeout(0.75);
    }

    public static Command scoreStraight(DriveTrain drive, Arm arm, Intake intake) {
        return arm.moveArmToPosition(ArmConstants.positionIntakeAlgae, .5).until(arm::isAtSetpoint)
                .andThen(arm.stop())
                .andThen(drive.moveStraight(0.5).withTimeout(2))
                .andThen(intake.moveIntake(Constants.SpeedConstants.coralOut).withTimeout(2))
                .andThen(drive.moveStraight(-0.5).withTimeout(.5));
    }

    public static Command outAndBack(DriveTrain driveTrain) {
        return driveTrain.moveStraight(0.25).withTimeout(0.75)
                .andThen(driveTrain.moveStraight(-0.25).withTimeout(0.75));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
