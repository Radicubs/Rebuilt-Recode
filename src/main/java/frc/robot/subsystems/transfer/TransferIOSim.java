package frc.robot.subsystems.transfer;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.TransferConstants;

class TransferIOSim implements TransferIO {
    private static final double NOMINAL_BUS_VOLTAGE = 12.0;

    private final DCMotor gearbox = DCMotor.getNeo550(1);
    private final FlywheelSim transferflywheel;
    private double appliedVolts = 0.0;

    TransferIOSim() {
        LinearSystem<N1, N1, N1> transferPlant = LinearSystemId.createFlywheelSystem(
                gearbox,
                TransferConstants.SimConstants.momentOfInertiaKgMetersSquared,
                TransferConstants.SimConstants.gearing);
        transferflywheel = new FlywheelSim(transferPlant, gearbox);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        transferflywheel.update(0.02);
        inputs.velocityRPS = transferflywheel.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = transferflywheel.getCurrentDrawAmps();
    }

    @Override
    public void DutyCycle(double dutyCycle) {
        appliedVolts = dutyCycle * NOMINAL_BUS_VOLTAGE;
        transferflywheel.setInputVoltage(appliedVolts);
    }
}
