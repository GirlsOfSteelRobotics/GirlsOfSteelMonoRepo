package com.gos.crescendo2024;

import com.gos.crescendo2024.subsystems.ChassisSubsystem;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.io.File;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class PathPlannerUtils {

    private PathPlannerUtils() {

    }

    public static void createTrajectoriesShuffleboardTab(ChassisSubsystem chassis) {
        ShuffleboardTab tab = Shuffleboard.getTab("PP Paths");

        File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

        Set<String> pathNames =
            Stream.of(pathFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".path"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toSet());
        System.out.println(Arrays.toString(pathFiles));
        for (String pathName : pathNames) {
            System.out.println("Loading " + pathName);
            tab.add(chassis.createFollowPathCommand(PathPlannerPath.fromPathFile(pathName), true).withName(pathName));
        }
    }
}
