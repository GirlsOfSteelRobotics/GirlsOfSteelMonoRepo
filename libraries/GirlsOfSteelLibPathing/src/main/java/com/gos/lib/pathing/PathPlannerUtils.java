package com.gos.lib.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.IOException;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class PathPlannerUtils {

    private PathPlannerUtils() {

    }

    public static void createTrajectoriesShuffleboardTab(Function<PathPlannerPath, Command> pathFactory) {
        ShuffleboardTab tab = Shuffleboard.getTab("PP Paths");

        File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

        Set<String> pathNames =
            Stream.of(pathFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".path"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toSet());
        for (String pathName : pathNames) {
            try {
                Command followPathCommand = pathFactory.apply(PathPlannerPath.fromPathFile(pathName))
                    .withName(pathName);
                tab.add(followPathCommand);
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static Command followChoreoPath(String trajectoryName) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(trajectoryName));
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
