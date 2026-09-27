// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kComputerControllerPort = 1;
    public static int kFloorButton = XboxController.Button.kA.value;
    public static int kCoralStationButton = XboxController.Button.kB.value;
    public static int kReefButton = XboxController.Button.kX.value;
    public static int kAlgaeButton = XboxController.Button.kY.value;
    public static int kClimbStart = XboxController.Button.kBack.value;
    public static int kClimbEnd = XboxController.Button.kStart.value;
    public static int kAlgaeIn = XboxController.Button.kRightBumper.value;
    public static int kAlgaeOut = XboxController.Button.kLeftBumper.value;
    public static int kCoralIn = XboxController.Axis.kRightTrigger.value;
    public static int kCoralOut = XboxController.Axis.kLeftTrigger.value;
  }

  public static class ArmConstants {
      // Define Arm position constants
      public static Double positionIntakeCoral      = 0.845;
      public static Double positionIntakeAlgae      = 0.760;
      public static Double positionRemoveAlgaeLow   = 0.705;
      public static Double positionClimbStart       = 0.620;
      public static Double positionClimbEnd         = 0.768;
      public static Double positionRemoveAlgaeHigh  = 0.785;

      // Define Arm position limits
      public static Double armFrontLimit            = 0.845;
      public static Double armRearLimit             = 0.450;

      // Define Arm velocity limit
      public static Double armVelocityLimit         = 0.8;

      // Define Arm PID constants
      public static Double armkP                    = 12.1;//17.5; //3.5 
      public static Double armkI                    = 0.3;    //.25
      public static Double armkD                    = 0.1;//0.8;

      // manual change of set point
      public static Double kSetPointDelta = 0.05;
  }

  public static class SpeedConstants {
      public static Double coralOut = -0.15; // negative
      public static Double coralIn = 0.75;
      public static Double algaeOut = -0.75; // negative
      public static Double algaeIn = 0.75;
    
  }
}
