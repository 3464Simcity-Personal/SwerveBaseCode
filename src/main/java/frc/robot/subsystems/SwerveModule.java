// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.ModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.SwerveConfigs;


/** Add your docs here. */
public class SwerveModule {

    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX driveMotor;
    private CANSparkMax turnMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder turnEncoder;

    private CANcoderConfigurator angleEncoderConfigurator;
    private TalonFXConfiguration driveMotorConfiguration;
    private TalonFXConfigurator driveMotorConfigurator;
    private CurrentLimitsConfigs driveSupplyLimit;

    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(SwerveConfigs.driveKS, SwerveConfigs.driveKV, SwerveConfigs.driveKA);

    public SwerveModule(int moduleNumber, ModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // turnMotor configuration
        turnMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configureTurnMotor();

        // driveMotor configuration
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configureDriveMotor();

        // cancoder configuration
        angleEncoder = new CANcoder(moduleConstants.canCoderID);
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfigurator.apply(SwerveConfigs.canCoderConfiguration);


    }

    private void configureEncoders() {
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfigurator.apply(SwerveConfigs.canCoderConfiguration);

        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(SwerveConfigs.DegreesPerTurnRotation);
        turnEncoder.setVelocityConversionFactor(SwerveConfigs.DegreesPerTurnRotation / 60); // this is degrees per sec


    }

    private void configureTurnMotor() {
        turnMotor.restoreFactoryDefaults();
        SparkPIDController controller = turnMotor.getPIDController();
        controller.setP(SwerveConfigs.angleKP, 0);
        controller.setI(SwerveConfigs.angleKI,0);
        controller.setD(SwerveConfigs.angleKD,0);
        controller.setFF(SwerveConfigs.angleKF,0);
        controller.setOutputRange(-SwerveConfigs.anglePower, SwerveConfigs.anglePower);
        turnMotor.setSmartCurrentLimit(SwerveConfigs.angleContinuousCurrentLimit);
       
        turnMotor.setInverted(SwerveConfigs.angleMotorInvert);
        turnMotor.setIdleMode(SwerveConfigs.angleIdleMode);
    }

    private void configureDriveMotor(){     
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfigurator = driveMotor.getConfigurator();
        driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.StatorCurrentLimit = SwerveConfigs.driveStatorCurrentLimit;
        driveSupplyLimit.StatorCurrentLimitEnable = SwerveConfigs.drivestatorCurrentLimitEnable;
        driveSupplyLimit.SupplyCurrentLimit = SwerveConfigs.driveSupplyCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConfigs.driveSupplyCurrentLimitEnable;
        driveSupplyLimit.SupplyCurrentThreshold = SwerveConfigs.driveSupplyCurrentThreshold;
        driveSupplyLimit.SupplyTimeThreshold = SwerveConfigs.driveSupplyTimeThreshold;

        driveMotorConfiguration.Slot0.kP = SwerveConfigs.driveKP;
        driveMotorConfiguration.Slot0.kI = SwerveConfigs.driveKI;
        driveMotorConfiguration.Slot0.kD = SwerveConfigs.driveKD;
        driveMotorConfiguration.Slot0.kS = SwerveConfigs.driveKS; 
        driveMotorConfiguration.Slot0.kV = SwerveConfigs.driveKV;
        driveMotorConfiguration.Slot0.kA = SwerveConfigs.driveKA;
        driveMotorConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConfigs.openLoopRamp;
        driveMotorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConfigs.closedLoopRamp;

        driveMotorConfigurator.apply(driveMotorConfiguration);
        driveMotorConfigurator.apply(driveSupplyLimit);

        driveMotor.setInverted(SwerveConfigs.driveMotorInvert);
    }



}
