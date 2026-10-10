package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
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
    private double mainRPSAdjustment = 0.0;
    private double topRPSAdjustment = 0.0;
    private double mainSetRPS = 0.0;
    private double topSetRPS = 0.0;
    private double savedMainRPS = 0.0;
    private double savedTopRPS = 0.0;
    private double requestedIndexerRPS = 0.0;
    private boolean optimizedShotActive = false;
    private boolean waitForSpeed = false;
    private final Debouncer feedReady = new Debouncer(ShooterConstants.readyToFeedSeconds);
    
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
        setShooterRPS(mainRPS, topRPS, indexerRPS, false, false);
    }

    public void setShooterRPSWhenReady(double mainRPS, double topRPS, double indexerRPS) {
        setShooterRPS(mainRPS, topRPS, indexerRPS, false, true);
    }

    public void setOptimizedShotRPS(double indexerRPS) {
        setShooterRPS(getMainDesiredSpeed(), ShooterConstants.optimizedTopShooterRPS, indexerRPS, true, false);
    }

    private void setShooterRPS(double mainRPS, double topRPS, double indexerRPS,
            boolean optimizedShot, boolean waitForSpeed) {
        optimizedShotActive = optimizedShot;
        if (!this.waitForSpeed || !waitForSpeed) {
            feedReady.calculate(false);
        }
        this.waitForSpeed = waitForSpeed;
        mainSetRPS = mainRPS + mainRPSAdjustment;
        topSetRPS = optimizedShot ? ShooterConstants.optimizedTopShooterRPS : topRPS + topRPSAdjustment;
        savedMainRPS = mainSetRPS;
        savedTopRPS = topSetRPS;
        requestedIndexerRPS = !waitForSpeed || feedReady.calculate(isAtSpeed()) ? indexerRPS : 0.0;
        io.setShooterRPS(mainSetRPS, topSetRPS, requestedIndexerRPS);
}

    public boolean isAtSpeed() {
        return mainSetRPS > 0.0 && topSetRPS > 0.0
                && Math.abs(inputs.leftVelocityRPS - mainSetRPS) <= ShooterConstants.pidTolerance
                && Math.abs(inputs.rightVelocityRPS - mainSetRPS) <= ShooterConstants.pidTolerance
                && Math.abs(inputs.topVelocityRPS - topSetRPS) <= ShooterConstants.pidTolerance;
    }

    public void adjustTopRPS(double deltaRPS) {
        topRPSAdjustment += deltaRPS;
        // Keep the optimized top-wheel target fixed, even between command executions.
        if (!optimizedShotActive) {
            savedTopRPS += deltaRPS;
        }
        applyRPSAdjustments();
    }

    public void adjustMainRPS(double deltaRPS) {
        mainRPSAdjustment += deltaRPS;
        savedMainRPS += deltaRPS;
        applyRPSAdjustments();
    }

    private void applyRPSAdjustments() {
        mainSetRPS = savedMainRPS;
        topSetRPS = savedTopRPS;
        if (waitForSpeed && !isAtSpeed()) {
            feedReady.calculate(false);
            requestedIndexerRPS = 0.0;
        }
        io.setShooterRPS(mainSetRPS, topSetRPS, requestedIndexerRPS);
    }

    public void setIndexerRPS(double indexerRPS) {
        requestedIndexerRPS = indexerRPS;
        io.setIndexerRPS(indexerRPS);
    }

    public double getSavedMainRPS() {
        return savedMainRPS;
    }

    public double getSavedTopRPS() {
        return savedTopRPS;
    }

    public double getTopRPSAdjustment() {
        return topRPSAdjustment;
    }

    public double getMainRPSAdjustment() {
        return mainRPSAdjustment;
    }

    public void stop() {
        optimizedShotActive = false;
        waitForSpeed = false;
        feedReady.calculate(false);
        requestedIndexerRPS = 0.0;
        mainSetRPS = 0.0;
        topSetRPS = 0.0;
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
        return mainSetRPS;
    }

    public double getLeftSetSpeed() {
        return mainSetRPS;
    }

    public double getIndexerSetSpeed() {
        return requestedIndexerRPS;
    }

    public double getTopSetSpeed() {
        return topSetRPS;
    }

    // public double getTopDesiredSpeed() {
    //     InterpolatingDouble interpolatedDegrees =
    //         InterpolatingConstants.topShooterSpeedMap.getInterpolated(
    //             new InterpolatingDouble(VisionFunctions.getHubDistanceMeters()));
    //     return interpolatedDegrees.value;
    // }

    public double getMainDesiredSpeed() {
        InterpolatingDouble interpolatedDegrees =
            InterpolatingConstants.mainShooterSpeedMap.getInterpolated(
                new InterpolatingDouble(VisionFunctions.getHubDistanceMeters()));
        return interpolatedDegrees.value;
    }
    
    private Shooter() {
        this(RobotBase.isSimulation() ? new ShooterIOSim() : new ShooterIOReal());
        ShooterLogger.publish(this);
    }

    Shooter(ShooterIO io) {
        this.io = io;
        
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
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        ShooterLogger.log(this);
        
    }
}
