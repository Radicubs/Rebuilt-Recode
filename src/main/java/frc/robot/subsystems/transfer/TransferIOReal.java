package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.TransferConstants;
import frc.robot.util.CANSignalConfig;

public class TransferIOReal implements TransferIO {
    private SparkMax transferMotor;
    private RelativeEncoder transferEncoder;

    TransferIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(TransferConstants.transferMotorStallCurrentLimit, TransferConstants.transferMotorFreeCurrentLimit);
        CANSignalConfig.configureSpark(config.signals, false);
        
        transferMotor = new SparkMax(TransferConstants.transferMotorCID, SparkMax.MotorType.kBrushless);
        transferMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        transferEncoder = transferMotor.getEncoder();

        }

        @Override
        public void updateInputs(TransferIOInputs inputs) {
            inputs.velocityRPS = (transferEncoder).getVelocity() / 60.0;
            inputs.appliedVolts = transferMotor.getAppliedOutput() * transferMotor.getBusVoltage();
            inputs.currentAmps = transferMotor.getOutputCurrent();

        }
    
        @Override
        public void DutyCycle(double dutyCycle) {
            transferMotor.set(dutyCycle);
        }
    
}
