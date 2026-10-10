package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

    private final TalonFX leftShooter;
    private final TalonFX rightShooter;
    private final TalonFX topShooter;
    private final TalonFX indexer;

    

    private final VelocityVoltage leftVel = new VelocityVoltage(0).withUpdateFreqHz(50.0);
    private final VelocityVoltage rightVel = new VelocityVoltage(0).withUpdateFreqHz(50.0);
    private final VelocityVoltage topVel = new VelocityVoltage(0).withUpdateFreqHz(50.0);
    private final VelocityVoltage indexerVel = new VelocityVoltage(0).withUpdateFreqHz(50.0);

    private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kS,ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kV,ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kA);

    private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(ShooterConstants.MainRightShooterPIDFeedforwardConstants.kS,ShooterConstants.MainRightShooterPIDFeedforwardConstants.kV,ShooterConstants.MainRightShooterPIDFeedforwardConstants.kA);

    private final SimpleMotorFeedforward topFF = new SimpleMotorFeedforward(ShooterConstants.TopShooterPIDFeedforwardConstants.kS,ShooterConstants.TopShooterPIDFeedforwardConstants.kV,ShooterConstants.TopShooterPIDFeedforwardConstants.kA);

    private final SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(ShooterConstants.IndexerPIDFeedforwardConstants.kS,ShooterConstants.IndexerPIDFeedforwardConstants.kV,ShooterConstants.IndexerPIDFeedforwardConstants.kA);

    ShooterIOReal() {
        
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.indexerEnableCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.indexerShooterCurrentLimit;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        indexerConfig.Slot0.kP = ShooterConstants.IndexerPIDFeedforwardConstants.kP;
        indexerConfig.Slot0.kI = ShooterConstants.IndexerPIDFeedforwardConstants.kI;
        indexerConfig.Slot0.kD = ShooterConstants.IndexerPIDFeedforwardConstants.kD;

        TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
        topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.topShooterEnableCurrentLimit;
        topShooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.topShooterCurrentLimit;
        topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        topShooterConfig.Slot0.kP = ShooterConstants.TopShooterPIDFeedforwardConstants.kP;
        topShooterConfig.Slot0.kI = ShooterConstants.TopShooterPIDFeedforwardConstants.kI;
        topShooterConfig.Slot0.kD = ShooterConstants.TopShooterPIDFeedforwardConstants.kD;

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        leftConfig.Slot0.kP = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kP;
        leftConfig.Slot0.kI = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kI;
        leftConfig.Slot0.kD = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kD;

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.Slot0.kP = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kP;
        rightConfig.Slot0.kI = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kI;
        rightConfig.Slot0.kD = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kD;

        indexer = new TalonFX(ShooterConstants.indexerCID);
        indexer.getConfigurator().apply(indexerConfig);
        topShooter = new TalonFX(ShooterConstants.topShooterCID);
        topShooter.getConfigurator().apply(topShooterConfig);
        rightShooter = new TalonFX(ShooterConstants.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);
        leftShooter = new TalonFX(ShooterConstants.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        for (TalonFX motor : new TalonFX[] {leftShooter, rightShooter, topShooter, indexer}) {
            // Velocity control runs onboard; this feedback is used for logging.
            motor.getVelocity().setUpdateFrequency(25.0);
            BaseStatusSignal.setUpdateFrequencyForAll(2.0,
                    motor.getMotorVoltage(), motor.getStatorCurrent());
            motor.optimizeBusUtilization(0.0);
        }

    }

        @Override
        public void updateInputs(ShooterIOInputs inputs) {

        inputs.leftVelocityRPS = leftShooter.getVelocity().getValueAsDouble();
        inputs.leftAppliedVolts = leftShooter.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftShooter.getStatorCurrent().getValueAsDouble();

        inputs.rightVelocityRPS = rightShooter.getVelocity().getValueAsDouble();
        inputs.rightAppliedVolts = rightShooter.getMotorVoltage().getValueAsDouble();
        inputs.rightCurrentAmps = rightShooter.getStatorCurrent().getValueAsDouble();
        
        inputs.topVelocityRPS = topShooter.getVelocity().getValueAsDouble();
        inputs.topAppliedVolts = topShooter.getMotorVoltage().getValueAsDouble();
        inputs.topCurrentAmps = topShooter.getStatorCurrent().getValueAsDouble();

        inputs.indexerVelocityRPS = indexer.getVelocity().getValueAsDouble();
        inputs.indexerAppliedVolts = indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerCurrentAmps = indexer.getStatorCurrent().getValueAsDouble();
        }

        public void setShooterRPS(double mainRPS, double topRPS, double indexerRPS) {
            leftShooter.setControl(leftVel.withVelocity(mainRPS).withFeedForward(leftFF.calculate(mainRPS)));
            rightShooter.setControl(rightVel.withVelocity(mainRPS).withFeedForward(rightFF.calculate(mainRPS)));
            topShooter.setControl(topVel.withVelocity(topRPS).withFeedForward(topFF.calculate(topRPS)));
            indexer.setControl(indexerVel.withVelocity(indexerRPS).withFeedForward(indexerFF.calculate(indexerRPS)));

        }

        public void setIndexerRPS(double indexerRPS) {
            indexer.setControl(indexerVel.withVelocity(indexerRPS).withFeedForward(indexerFF.calculate(indexerRPS)));
        }

        public void stop() {
            rightShooter.set(0);
            leftShooter.set(0);
            indexer.set(0);
            topShooter.set(0);

        }
    






        
        


    }

    

