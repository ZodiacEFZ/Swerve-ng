// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {
    /** Creates a new SwerveModule. */
    public SwerveModule(int vID, int aID, int zeropos) {
        velocityMotor = new TalonFX(vID);
        angleMotor = new TalonSRX(aID);
        angle = 0;
        targetAngle = 0;
        zeroPos = zeropos;
        velocityMotor.configFactoryDefault();
        velocityMotor.config_kP(0, 0);
        velocityMotor.config_kI(0, 0);
        velocityMotor.config_kD(0, 0);
        velocityMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        angleMotor.configFactoryDefault();
        angleMotor.config_kP(0, 0);
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        angleMotor.set(ControlMode.Position, zeroPos);
    }

    private TalonFX velocityMotor;
    private TalonSRX angleMotor;

    private double angle;
    private double targetAngle;
    private int zeroPos;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        angle = angleMotor.getSelectedSensorPosition();

    }
}
