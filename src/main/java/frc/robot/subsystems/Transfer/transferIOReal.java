package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.TransferConstants;

public class transferIOReal implements transferIO {
    private SparkMax transferMotor;
    private RelativeEncoder transferEncoder;

    transferIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(TransferConstants.transferMotorStallCurrentLimit, TransferConstants.transferMotorFreeCurrentLimit);
        
        transferMotor = new SparkMax(TransferConstants.transferMotorCID, SparkMax.MotorType.kBrushless);
        transferMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        transferEncoder = transferMotor.getEncoder();

        }

        @Override
        public void updateInputs(transferIOInputs inputs) {
            inputs.velocityRPS = (transferEncoder).getVelocity() / 60.0;
            inputs.appliedVolts = transferMotor.getAppliedOutput() * transferMotor.getBusVoltage();
            inputs.currentAmps = transferMotor.getOutputCurrent();

        }
    
        @Override
        public void DutyCycle(double dutyCycle) {
            transferMotor.set(dutyCycle);
        }
    
}
