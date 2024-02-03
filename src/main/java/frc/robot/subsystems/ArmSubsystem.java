// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class ArmSubsystem extends SubsystemBase {
  private double setPoint; 
   private CANSparkMax motor;
   private RelativeEncoder turnEncoder; 
   PIDController armPID; 
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
  motor = new CANSparkMax(10, MotorType.kBrushless); 
  armPID = new PIDController(0.5, 0.01, 0);
  turnEncoder =  motor.getEncoder();
  turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
  this.motor.setSmartCurrentLimit(30);
  this.motor.restoreFactoryDefaults();
  this.motor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power){
    motor.set(power);
  }
  public void run(){ 
    System.out.println(getTurningPosition()); 
    System.out.println("Testtesttest");
    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    armPID.enableContinuousInput(-Math.PI, Math.PI);
    motor.set(armPID.calculate(getTurningPosition(), setPosition(5)));
  }
  public double setPosition(double rad){
   return rad; 
  }
  public double getTurningPosition(){
    return turnEncoder.getPosition(); 
  }
  public void resetEncoder(){
    System.out.println("This encoder just got resetted");
    turnEncoder.setPosition(0);
  }
  public void stop(){
    this.motor.set(0);
  }
}
