package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hwak extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final LaserCan laserCan;
    private final DigitalInput bottomLimit;
    private final StatusSignal<Angle> elevatorPosition;

    public hwak() {
        elevatorMotor = new TalonFX(5);
        elevatorPosition = elevatorMotor.getPosition();
        laserCan = new LaserCan(0);
        bottomLimit = new DigitalInput(1);
    }
    public Command zeroElevator() {
        return run(() -> {
            elevatorMotor.set(-0.2);
        })
        .until(() -> bottomLimit.get())
        .finallyDo(() -> {
            elevatorMotor.stopMotor();
            elevatorMotor.setPosition(0);
        });
    }
    public boolean hasGamePiece() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement == null){
            return false;
        }
        return measurement.distance_mm < 75;
    }
    public void moveElevator(double speed) {
        if (speed < 0 && bottomLimit.get()){
            elevatorMotor.stopMotor();
            return;
        }
        elevatorMotor.set(speed);
    }
    @Override
    public void periodic() {
        elevatorPosition.refresh();
        SmartDashboard.putNumber("Position",elevatorPosition.getValueAsDouble());
    }
}