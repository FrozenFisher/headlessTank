package frc.robot.commands.IntakerCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakerConstants;
import frc.robot.subsystems.Intaker.IntakerSubsystem;

/**
 * 进球（正转）命令：
 *  - 命令开始后，先以 HIGH_RPS 运行 RIGHT_TRIGGER_HIGH_DURATION 秒
 *  - 之后降为 LOW_RPS，直到命令结束
 *  - 命令结束时停转
 */
public class IntakeCommand extends Command {
    private final IntakerSubsystem intakeSubsystem;
    private final Timer timer = new Timer();

    public IntakeCommand(IntakerSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double elapsed = timer.get();
        if (elapsed < IntakerConstants.OUTTAKE_HIGH_DURATION) {
            intakeSubsystem.setRPS(IntakerConstants.OUTTAKE_HIGH_RPS);
        } else {
            intakeSubsystem.setRPS(IntakerConstants.OUTTAKE_LOW_RPS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


