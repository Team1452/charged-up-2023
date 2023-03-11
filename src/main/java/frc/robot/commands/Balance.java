package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Utils;
import frc.robot.Constants;
import frc.robot.DriveSubsystem;
import frc.robot.Robot;

// TODO: Migrate this to PIDCommand()
public class Balance extends CommandBase {
    private static double UPDATE_LATENCY_MS = 20;    
    private float timeElapsed = 0;

    PIDController controller;
    DriveSubsystem drive;

    int ticksSinceLastUpdate = 0;

    public Balance(DriveSubsystem drive) {
        this.controller = new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD);
        // this.controller.setTolerance(DriveConstants.kBalanceToleranceDegrees);
        this.controller.setTolerance(DriveConstants.kBalanceToleranceDegrees);

        this.drive = drive;
        // super(
        //     new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD),
        //     drive::getPitch,
        //     0,
        //     output -> {
        //         System.out.println("Pitch: " + drive.getPitch());
        //         drive.differentialDrive(-output, 0);
        //     },
        //     drive);

        // getController()
        //     .setTolerance(DriveConstants.kBalanceToleranceDegrees, 0.01); // TODO: Move to constants
    }
    
    @Override
    public void initialize() {
        timeElapsed = 0;
    }

    @Override
    public void execute() {
        timeElapsed += Constants.PERIOD_MS/1000;
        // ticksSinceLastUpdate += 1;

        // if (ticksSinceLastUpdate * Constants.PERIOD_MS < UPDATE_LATENCY_MS - 1e-5) {
        //     return;
        // } else {
        //     ticksSinceLastUpdate = 0;
        // }

        controller.setP(Constants.DriveConstants.kBalanceP);
        controller.setI(Constants.DriveConstants.kBalanceI);
        controller.setD(Constants.DriveConstants.kBalanceD);

        double pitch = drive.getPitch();
        double output = controller.calculate(pitch, 0);

        output *= 2/(1 + Math.exp(-Robot.kA.getValue() * timeElapsed));
        System.out.println("Pitch: " + pitch + "; output: " + output + "; tolerance: " + controller.getPositionTolerance() + "; at setpoint: " + Utils.atSetpoint(controller));
        
        if (Utils.atSetpoint(controller))
            drive.differentialDrive(0, 0);
        else
            drive.differentialDrive(-output, 0);
    }

    @Override
    public boolean isFinished() {
        if (Utils.atSetpoint(controller)) {
            System.out.println("Finished balancing!");
        }

        // return false;
        return Utils.atSetpoint(controller);
    }
}
