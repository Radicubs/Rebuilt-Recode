package frc.robot.subsystems.pivot;
import frc.robot.constants.PivotConstants;
import frc.robot.util.CANSignalConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PivotIOReal implements PivotIO {

    private final SparkMax pivotMotor;
    private final RelativeEncoder encoder;

    PivotIOReal() {
        pivotMotor = new SparkMax(PivotConstants.pivotMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.encoder.positionConversionFactor(1.0 / 60);
        pivotMotorConfig.encoder.velocityConversionFactor(1.0 / 60);
        pivotMotorConfig.smartCurrentLimit(PivotConstants.pivotMotorStallCurrentLimit, PivotConstants.pivotMotorFreeCurrentLimit);
        CANSignalConfig.configureSpark(pivotMotorConfig.signals, true);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = pivotMotor.getEncoder();

    }

    @Override
    public void setDutyCycle(double dutyCycle){
        pivotMotor.set(dutyCycle);
    }

    @Override
    public void setPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    } 
    
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.requestedDuty = pivotMotor.get();
        inputs.appliedDuty = pivotMotor.getAppliedOutput();
        inputs.currentAmps = pivotMotor.getOutputCurrent();
        inputs.positionRotations = encoder.getPosition();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    }

}
