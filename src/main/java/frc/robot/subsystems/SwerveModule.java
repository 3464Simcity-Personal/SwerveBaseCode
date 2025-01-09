// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.util.ModuleConstants;
import frc.robot.util.ModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {

    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX driveMotor;
    private CANSparkMax turnMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder turnEncoder;

    private CANcoderConfigurator angleEncoderConfigurator;
    private CANcoderConfiguration angleEncoderConfiguration;
    private TalonFXConfiguration driveMotorConfiguration;
    private TalonFXConfigurator driveMotorConfigurator;
    private CurrentLimitsConfigs driveSupplyLimit;

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int moduleNumber, ModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // turnMotor configuration
        turnMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configureTurnMotor();

        // driveMotor configuration
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configureDriveMotor();

        // ecoders configuration
        angleEncoder = new CANcoder(moduleConstants.canCoderID);
        configureEncoders();
    }

    public void setDesiredState(SwerveModuleState desiredState, Boolean isOpenLoop) {
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        // cos compensation for reducing skew, reduces the speed when it's not pointing in the right direction
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos(); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
          driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
          driveMotor.setControl(driveDutyCycle);
        } 
        else {
          driveVelocity.Velocity = desiredState.speedMetersPerSecond / SwerveConstants.driveRevToMeters;
          driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
          driveMotor.setControl(driveVelocity);
        }
      }

    public void setAngle(SwerveModuleState desiredState) {
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        if(Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01)) {
            turnMotor.stopMotor();
            return;
        }
        Rotation2d angle = desiredState.angle;
        SparkPIDController controller = turnMotor.getPIDController();
        double degreeReference = angle.getDegrees();
        controller.setReference(degreeReference, ControlType.kPosition, 0);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        turnEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            (driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.driveRevToMeters), 
            getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            (driveMotor.getPosition().getValueAsDouble() * SwerveConstants.driveRevToMeters),
            getAngle());
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    private void configureEncoders() {
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfiguration = new CANcoderConfiguration();
        angleEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoderConfigurator.apply(angleEncoderConfiguration);

        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(SwerveConstants.DegreesPerTurnRotation);
        turnEncoder.setVelocityConversionFactor(SwerveConstants.DegreesPerTurnRotation / 60); // this is degrees per sec
    }

    private void configureTurnMotor() {
        turnMotor.restoreFactoryDefaults();
        SparkPIDController controller = turnMotor.getPIDController();
        controller.setP(SwerveConstants.angleKP, 0);
        controller.setI(SwerveConstants.angleKI,0);
        controller.setD(SwerveConstants.angleKD,0);
        controller.setFF(SwerveConstants.angleKF,0);
        controller.setOutputRange(-SwerveConstants.anglePower, SwerveConstants.anglePower);
        turnMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
       
        turnMotor.setInverted(SwerveConstants.angleMotorInvert);
        turnMotor.setIdleMode(SwerveConstants.angleIdleMode);
    }

    private void configureDriveMotor(){     
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfigurator = driveMotor.getConfigurator();
        driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.driveSupplyCurrentLimitEnable;
        driveSupplyLimit.SupplyCurrentThreshold = SwerveConstants.driveSupplyCurrentThreshold;
        driveSupplyLimit.SupplyTimeThreshold = SwerveConstants.driveSupplyTimeThreshold;

        driveMotorConfiguration.Slot0.kP = SwerveConstants.driveKP;
        driveMotorConfiguration.Slot0.kI = SwerveConstants.driveKI;
        driveMotorConfiguration.Slot0.kD = SwerveConstants.driveKD;
        driveMotorConfiguration.Slot0.kS = SwerveConstants.driveKS; 
        driveMotorConfiguration.Slot0.kV = SwerveConstants.driveKV;
        driveMotorConfiguration.Slot0.kA = SwerveConstants.driveKA;
        driveMotorConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        driveMotorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;

        driveMotorConfigurator.apply(driveMotorConfiguration);
        driveMotorConfigurator.apply(driveSupplyLimit);

        driveMotor.setInverted(SwerveConstants.driveMotorInvert);
    }



}
