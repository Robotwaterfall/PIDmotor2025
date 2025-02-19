// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private SparkMax pidmotor = new SparkMax(13, MotorType.kBrushless);
  private TalonSRX LeftMasterMotor1 = new TalonSRX( 18);
  private TalonSRX LeftMasterMotor2 = new TalonSRX(3);
  private TalonSRX RightMasterMotor1 = new TalonSRX(4);
  private TalonSRX RightMasterMotor2 = new TalonSRX(1);

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  private double startTime;

  private Joystick joy1 = new Joystick(0);
 // this function below is run when the robot is enabled
  public Robot() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    pidmotor.getEncoder().setPosition(0); //resets encoder position to 0
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

    if (time - startTime < 3){
      pidmotor.set(setpoint = 5);
      LeftMasterMotor1.set(ControlMode.PercentOutput, 0.5);
      LeftMasterMotor2.set(ControlMode.PercentOutput, 0.5);
      RightMasterMotor1.set(ControlMode.PercentOutput, -0.5);
      RightMasterMotor2.set(ControlMode.PercentOutput, -0.5);
    } else {
      pidmotor.set(setpoint = 0);
      LeftMasterMotor1.set(ControlMode.PercentOutput, 0);
      LeftMasterMotor2.set(ControlMode.PercentOutput, 0);
      RightMasterMotor1.set(ControlMode.PercentOutput, 0);
      RightMasterMotor2.set(ControlMode.PercentOutput, 0);
    }

    // get sensor position
    double sensorPosition =  pidmotor.getEncoder().getPosition() * kDriveTick2Feet;

    // calculations
    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;

    // output to motors
    pidmotor.set(outputSpeed);


  }

  @Override
  public void teleopInit() {
    pidmotor.getEncoder().setPosition(0); //resets encoder position to 0

  }

  final double kP = 0.5;
  double setpoint = 0;

  @Override
  public void teleopPeriodic() {
    // get joystick command
    if (joy1.getRawButton(1)) {
      setpoint = 5;
    } else if (joy1.getRawButton(2)) {
      setpoint = 0;
    }

    // get sensor position
    double sensorPosition =  pidmotor.getEncoder().getPosition() * kDriveTick2Feet;

    // calculations
    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;

    // output to motors
    pidmotor.set(outputSpeed);

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
