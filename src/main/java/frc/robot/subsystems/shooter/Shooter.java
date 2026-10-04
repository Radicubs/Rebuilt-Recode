package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.InterpolatingConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.subsystems.vision.VisionFunctions;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private boolean topPIDEnabled = false;
    private boolean indexerPIDEnabled = false;
    private boolean rightPIDEnabled = false;
    private boolean leftPIDEnabled = false;

    private final PIDController topcontroller;
    private final PIDController indexercontroller;
    private final PIDController rightcontroller;
    private final PIDController leftcontroller;

    private final SimpleMotorFeedforward topfeedforward;
    private final SimpleMotorFeedforward indexerfeedforward;
    private final SimpleMotorFeedforward leftfeedforward;
    private final SimpleMotorFeedforward rightfeedforward;

    
    
    public static Shooter getInstance(){
        if(INSTANCE == null) { INSTANCE = new Shooter(); }
        return INSTANCE;
    }

    public void setShooterRPS(double mainRPS, double topRPS, double indexerRPS) {
        io.setShooterRPS(mainRPS, topRPS, indexerRPS);
}

    public void stop() {
        io.stop();
}

    public double getTopShooterSpeed() {
        return inputs.topVelocityRPS;
    }

    public double getIndexerSpeed() {
        return inputs.indexerVelocityRPS;
    }

    public double getLeftShooterSpeed() {
        return inputs.leftVelocityRPS;
    }


    public double getRightShooterSpeed() {
        return inputs.rightVelocityRPS;
    }

    public double getRightSetSpeed() {
        return ShooterConstants.CloseShootSpeeds.mainShooterRPS;
    }

    public double getLeftSetSpeed() {
        return ShooterConstants.CloseShootSpeeds.mainShooterRPS;
    }

    public double getIndexerSetSpeed() {
        return ShooterConstants.CloseShootSpeeds.indexerRPS;
    }

    public double getTopSetSpeed() {
        return ShooterConstants.CloseShootSpeeds.topShaftRPS;
    }

    public double getDesiredSpeed() {
    InterpolatingDouble interpolatedDegrees =
        InterpolatingConstants.ShooterSpeedMap.getInterpolated(
            new InterpolatingDouble(VisionFunctions.getHubDistanceMeters()));
    return interpolatedDegrees.value;
    }
    
    private Shooter() {
        io = RobotBase.isSimulation() ? new ShooterIOSim() : new ShooterIOReal();
        
        topcontroller = new PIDController(ShooterConstants.TopShooterPIDFeedforwardConstants.kP, ShooterConstants.TopShooterPIDFeedforwardConstants.kI, ShooterConstants.TopShooterPIDFeedforwardConstants.kD);
        topcontroller.setTolerance(ShooterConstants.pidTolerance);

        indexercontroller = new PIDController(ShooterConstants.IndexerPIDFeedforwardConstants.kP, ShooterConstants.IndexerPIDFeedforwardConstants.kI, ShooterConstants.IndexerPIDFeedforwardConstants.kD);
        indexercontroller.setTolerance(ShooterConstants.pidTolerance);

        rightcontroller = new PIDController(ShooterConstants.MainRightShooterPIDFeedforwardConstants.kP, ShooterConstants.MainRightShooterPIDFeedforwardConstants.kI, ShooterConstants.MainRightShooterPIDFeedforwardConstants.kD);
        rightcontroller.setTolerance(ShooterConstants.pidTolerance);

        leftcontroller = new PIDController(ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kP, ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kI, ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kD);
        leftcontroller.setTolerance(ShooterConstants.pidTolerance);

        topfeedforward = new SimpleMotorFeedforward(ShooterConstants.TopShooterPIDFeedforwardConstants.kS, ShooterConstants.TopShooterPIDFeedforwardConstants.kV, ShooterConstants.TopShooterPIDFeedforwardConstants.kA);
        indexerfeedforward = new SimpleMotorFeedforward(ShooterConstants.IndexerPIDFeedforwardConstants.kS, ShooterConstants.IndexerPIDFeedforwardConstants.kV, ShooterConstants.IndexerPIDFeedforwardConstants.kA);
        rightfeedforward = new SimpleMotorFeedforward(ShooterConstants.MainRightShooterPIDFeedforwardConstants.kS, ShooterConstants.MainRightShooterPIDFeedforwardConstants.kV, ShooterConstants.MainRightShooterPIDFeedforwardConstants.kA);
        leftfeedforward = new SimpleMotorFeedforward(ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kS, ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kV, ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kA);
        
        ShooterLogger.publish(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        
    }
}