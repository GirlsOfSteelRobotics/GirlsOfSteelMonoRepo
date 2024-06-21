
package com.gos.crescendo2024.auton.modes.fournote;

import com.gos.crescendo2024.auton.GosAutoMode;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

import static com.gos.crescendo2024.PathPlannerUtils.followChoreoPath;

public class FourNoteMiddle521Choreo extends GosAutoMode {

    private static final String PATH_BASE = "FourNoteMiddle521";

    @SuppressWarnings("PMD.AvoidDuplicateLiterals")
    public FourNoteMiddle521Choreo() {
        super(
            "Three Note Amp Side - 03",
            StartPosition.STARTING_LOCATION_AMP_SIDE,
            List.of(0, 3),

            Commands.sequence(
                NamedCommands.getCommand("AimAndShootIntoSpeaker"),
                Commands.deadline(
                    followChoreoPath(PATH_BASE + ".1"),
                    NamedCommands.getCommand("IntakePiece")
                ),
                followChoreoPath(PATH_BASE + ".2"),
                NamedCommands.getCommand("AimAndShootIntoSpeaker"),
                Commands.parallel(
                    followChoreoPath(PATH_BASE + ".3"),
                    NamedCommands.getCommand("IntakePiece")
                ),
                Commands.deadline(
                    followChoreoPath(PATH_BASE + ".4"),
                    NamedCommands.getCommand("PrepSpeakerShot")
                ),
                NamedCommands.getCommand("AimAndShootIntoSpeaker"),
                Commands.deadline(
                    followChoreoPath(PATH_BASE + ".5"),
                    NamedCommands.getCommand("IntakePiece")
                ),
                NamedCommands.getCommand("AimAndShootIntoSpeaker")
            )
        );
    }
}

