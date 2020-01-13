package org.frc5587.lib.command;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ParallelCommand extends CommandBase {
    CommandBase[] commands;

    /**
     * This is a class used to run two or more commands simultaneously in a more
     * intuitive manner than with stock WPILib. Rather than relying on
     * CommandGroups for simply running a small number of commands in parallel and
     * the often confusing addParallel method in CommandGroups, one can simply use
     * this class in place of any normal command.
     */
    public ParallelCommand(CommandBase... commands) {
        this.commands = commands;
    }

    // Called just before this CommandBase runs the first time
    @Override
    public void initialize() {
        for (CommandBase command : commands) {
            command.initialize();
        }
    }

    // Called repeatedly when this CommandBase is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this CommandBase no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // Check if any commands are not complete and return false if they aren't
        for (CommandBase command : commands) {
            if (!command.isFinished()) {
                return false;
            }
        }
        // If all are complete, return true
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            for (CommandBase command : commands) {
                command.cancel();
            }
        }
    }
}