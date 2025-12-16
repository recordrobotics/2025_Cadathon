package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.field.FieldIntersection;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/** Represents the physical model of the robot, including mechanisms and their positions */
public final class RobotModel extends ManagedSubsystemBase {

    public interface RobotMechanism {
        int getPoseCount();

        void updatePoses(Pose3d[] poses, int i);
    }

    public class HoodShooter implements RobotMechanism {

        private double angle = 0.0;

        @Override
        public int getPoseCount() {
            return 1;
        }

        public void update(double angle) {
            this.angle = angle;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            Translation3d origin = new Translation3d(0.272, 0.0, 0.378);

            Pose3d pose = Pose3d.kZero.rotateAround(origin, new Rotation3d(0.0, angle, 0.0));
            poses[i] = pose;
        }
    }

    public class Intake implements RobotMechanism {

        private double angle;
        private double flapAngle;

        @Override
        public int getPoseCount() {
            return 2;
        }

        public void update(double angle, double flapAngle) {
            this.angle = angle;
            this.flapAngle = flapAngle;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            Translation3d origin = new Translation3d(-0.1, 0.0, 0.276);
            Translation3d originFlap = new Translation3d(0.207, 0.0, 0.681);

            Pose3d pose = Pose3d.kZero.rotateAround(origin, new Rotation3d(0.0, angle, 0.0));
            Pose3d poseFlap = Pose3d.kZero
                    .rotateAround(originFlap, new Rotation3d(0.0, flapAngle, 0.0))
                    .rotateAround(origin, new Rotation3d(0.0, angle, 0.0));

            poses[i++] = pose;
            poses[i] = poseFlap;
        }
    }

    public static class RobotGamePiece {
        private Supplier<Pose3d> poseSupplier;

        public RobotGamePiece(Supplier<Pose3d> poseSupplier) {
            this.poseSupplier = poseSupplier;
        }

        public Supplier<Pose3d> getPoseSupplier() {
            return poseSupplier;
        }

        public Pose3d getPose() {
            return poseSupplier.get();
        }

        public void setPoseSupplier(Supplier<Pose3d> poseSupplier) {
            this.poseSupplier = poseSupplier;
        }
    }

    public final HoodShooter hoodShooter = new HoodShooter();
    public final Intake intake = new Intake();

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] mechanismPoses = new Pose3d[hoodShooter.getPoseCount() + intake.getPoseCount()];

    private final RobotGamePiece robotCoral = new RobotGamePiece(() -> null);
    private final RobotGamePiece robotAlgae = new RobotGamePiece(() -> null);

    public RobotModel() {
        periodicManaged();
    }

    @Override
    public void periodicManaged() {
        updatePoses(hoodShooter, intake);

        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.DEBUG_SIM)) {
            Logger.recordOutput(
                    "IGamePositions",
                    IGamePosition.aggregatePositions(
                            Constants.Game.CoralPosition.values(),
                            Constants.Game.AlgaePosition.values(),
                            Constants.Game.SourcePosition.values(),
                            Constants.Game.ProcessorPosition.values()));
            FieldIntersection.logPolygons();
        }
    }

    private void updatePoses(RobotMechanism... mechanisms) {
        int i = 0;
        for (RobotMechanism mechanism : mechanisms) {
            if (i >= mechanismPoses.length) {
                ConsoleLogger.logError("RobotModel.updatePoses: too many mechanisms");
                break;
            }

            mechanism.updatePoses(mechanismPoses, i);
            i += mechanism.getPoseCount();
        }
    }

    @AutoLogLevel(level = Level.SIM)
    public Pose3d[] getCoralPositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> coralPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Coral");
            Pose3d robotCoralPose = robotCoral.poseSupplier.get();
            if (robotCoralPose != null) {
                coralPoses.add(robotCoralPose);
            }
            return coralPoses.toArray(new Pose3d[0]);
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose3d[] getAlgaePositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> algaePoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Algae");
            Pose3d robotAlgaePose = robotAlgae.poseSupplier.get();
            if (robotAlgaePose != null) {
                algaePoses.add(robotAlgaePose);
            }
            return algaePoses.toArray(new Pose3d[0]);
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose2d getRobot() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            return Pose2d.kZero;
            // return RobotContainer.drivetrain.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        } else {
            return Pose2d.kZero;
        }
    }

    public RobotGamePiece getRobotCoral() {
        return robotCoral;
    }

    public RobotGamePiece getRobotAlgae() {
        return robotAlgae;
    }
}
