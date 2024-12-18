import os
import shutil


def make_project_gradleable():
    projects = []
    projects.append("old_robots/y2019/2019DeepSpace")
    projects.append("old_robots/y2020/2020InfiniteRecharge")
    projects.append("old_robots/y2022/RapidReact")
    projects.append("old_robots/y2023/ChargedUp")

    truth_project = "y2024/Crescendo"

    phoenix5_truth = "codelabs/first_robot_code/vendordeps/Phoenix.json"
    navx_truth = "evergreen_robots/SCRAWestCoastDrive/vendordeps/navx_frc.json"

    for project in projects:
        truth_build_file = os.path.join(truth_project, "build.gradle")
        truth_vendordeps = os.path.join(truth_project, "vendordeps")

        project_build_file = os.path.join(project, "build.gradle")
        project_vendordeps = os.path.join(project, "vendordeps")

        shutil.copy(truth_build_file, project_build_file)
        shutil.copytree(truth_vendordeps, project_vendordeps, dirs_exist_ok=True)

        if "2019" in project:
            shutil.copy(phoenix5_truth, project_vendordeps)

        if "2020" in project:
            shutil.copy(phoenix5_truth, project_vendordeps)
            shutil.copy(navx_truth, project_vendordeps)

    with open("settings.gradle", 'a') as f:
        f.write("\n\n")
        for project in projects:
            f.write(f"include '{project.replace('/', ':')}'\n")


if __name__ == "__main__":
    #  py -m libraries.scripts.updater.make_project_gradleable
    make_project_gradleable()