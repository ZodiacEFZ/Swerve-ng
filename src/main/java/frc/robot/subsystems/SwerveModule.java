// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {
    /** Creates a new SwerveModule. */
    public SwerveModule(int number, int vID, int aID, int zeroPos, boolean inverse) {
        this.number = number;
        this.zeroPos = zeroPos;

        angle = 0;
        targetAngle = 0;

        velocityMotor = new WPI_TalonFX(vID);
        velocityMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        velocityMotor.configFactoryDefault();
        velocityMotor.config_kP(0, 0);
        velocityMotor.config_kI(0, 0);
        velocityMotor.config_kD(0, 0);
        velocityMotor.config_kF(0, 0);
        velocityMotor.setNeutralMode(NeutralMode.Brake);
        velocityMotor.setSensorPhase(true);
        velocityMotor.setInverted(inverse);

        angleMotor = new WPI_TalonSRX(aID);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        angleMotor.configFactoryDefault();
        angleMotor.config_kP(0, 0);
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0);
        angleMotor.config_kF(0, 0);
        // angleMotor.set(ControlMode.Position, zeroPos);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setSensorPhase(true);
        angleMotor.setInverted(inverse);
    }

    private WPI_TalonFX velocityMotor;
    private WPI_TalonSRX angleMotor;

    private double angle, targetAngle;
    private int zeroPos;
    private double kV;
    private int number;
    private double position;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        position = angleMotor.getSelectedSensorPosition();
        angle = (((position - zeroPos) / 4096 * 360) % 360 + 360) % 360;

        SmartDashboard.putNumber("Swerve Module" + number + " Angle", angle);
        SmartDashboard.putNumber("Swerve Module" + number + " rawAnglePos", position);
        SmartDashboard.putNumber("Swerve Module" + number + " Velocity", velocityMotor.getSelectedSensorVelocity());
    }

    public void setStatus(int velocityGoal, int angleGoal) {
        double deltaTheta = 0;
        double rawDeltaTheta = targetAngle - angle;

        if (Math.abs(rawDeltaTheta) <= 90) {
            deltaTheta = rawDeltaTheta;
            kV = 1;
        }

        if (rawDeltaTheta > 90 && rawDeltaTheta < 270) {
            deltaTheta = rawDeltaTheta - 180;
            kV = -1;
        }

        if (rawDeltaTheta >= 270 && rawDeltaTheta <= 360) {
            deltaTheta = rawDeltaTheta - 360;
            kV = 1;
        }

        if (rawDeltaTheta < -90 && rawDeltaTheta > -270) {
            deltaTheta = rawDeltaTheta + 180;
        }

        if (rawDeltaTheta <= -270 && rawDeltaTheta >= -360) {
            deltaTheta = rawDeltaTheta + 360;
        }

        double positionGoal = deltaTheta / 360 * 4096 + position;

        if (Math.abs(deltaTheta) > 0.5) {
            angleMotor.set(ControlMode.Position, positionGoal);
        }

        velocityMotor.set(ControlMode.Velocity, kV * velocityGoal);
    }
}
