// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

import java.util.Arrays;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrain m_drive = new DriveTrain();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();
    private final Climber m_climber = new Climber();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_computerController =
            new CommandXboxController(OperatorConstants.kComputerControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        autoChooser.setDefaultOption("Shoot and Back Up", Autos.scoreStraight(m_drive, m_arm, m_intake));
        autoChooser.addOption("Move Out", Autos.leaveStartZone(m_drive));
        autoChooser.addOption("Do Nothing", Autos.doNothing(m_drive));
        autoChooser.addOption("Out and Back", Autos.outAndBack(m_drive));
        SmartDashboard.putData(autoChooser);
        CameraServer.startAutomaticCapture();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.arcadeDrive(-m_driverController.getLeftY(),
                        -m_driverController.getRightX()), m_drive));
        m_intake.setDefaultCommand(m_intake.moveIntake(0.0));
        m_climber.setDefaultCommand(m_climber.moveClimber(0.0));

        // Arm bindings
        for (CommandXboxController controller : Arrays.asList(m_driverController, m_computerController)) {
            // Move to intake coral at the floor
            controller.button(OperatorConstants.kFloorButton)
                    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeCoral, 1));
            // Move to intake algae at the floor
            controller.button(OperatorConstants.kAlgaeButton)
                    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeAlgae, 1));

            // Move to remove low-reef algae and dump L1 coral
            controller.button(OperatorConstants.kReefButton)
                    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow, 1));

            // Move to get coral from coral station
            controller.button(OperatorConstants.kCoralStationButton)
                    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh, 1));

            // Move to start climb
            controller.button(OperatorConstants.kClimbStart)
                    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbStart, 1));

            // Move to finish climb
            controller.button(OperatorConstants.kClimbEnd).onTrue(m_arm.climb());
        }

        // intake bindings
        for (CommandXboxController controller : Arrays.asList(m_driverController, m_computerController)) {

            // Algae Out
            controller.button(OperatorConstants.kAlgaeOut)
                    .and(m_driverController.leftTrigger().negate())
                    .whileTrue(m_intake.moveIntake(Constants.SpeedConstants.algaeOut));

            // Algae In
            controller.button(OperatorConstants.kAlgaeIn)
                    .and(m_driverController.rightTrigger().negate())
                    .whileTrue(m_intake.moveIntake(Constants.SpeedConstants.algaeIn));

            controller.leftTrigger()
                    .and(m_driverController.button(OperatorConstants.kAlgaeOut).negate())
                    .whileTrue(m_intake.moveIntake(Constants.SpeedConstants.coralIn));

            controller.rightTrigger()
                    .and(m_driverController.button(OperatorConstants.kAlgaeIn).negate())
                    .whileTrue(m_intake.moveIntake(Constants.SpeedConstants.coralOut));
        }

        // Climber bindings
        for (CommandXboxController controller : Arrays.asList(m_driverController, m_computerController)) {
            controller.povRight().onTrue(m_climber.grab());
            controller.povLeft().onTrue(m_climber.release());
        }

        // manually adjust the arm's position
        m_computerController.povUp().onTrue(m_arm.nudgeUp());
        m_computerController.povDown().onTrue(m_arm.nudgeDown());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
