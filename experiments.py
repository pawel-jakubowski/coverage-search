#!/usr/bin/python
import os
import errno
import subprocess

def mkdir(path):
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            pass
        else:
            raise


class CmakeCommand:
    threads = 4
    def __init__(self, srcDir, buildDir = None):
        if buildDir is None:
            buildDir = srcDir + "/build"
        self.srcDir = srcDir
        self.buildDir = buildDir
        self.flags = {}
    def makeBuildDir(self):
        mkdir(self.buildDir)
    def setFlag(self, key, value):
        self.flags[key] = value
    def configure(self):
        cmakeCmd = "cmake"
        for key, flag in self.flags.items():
            cmakeCmd += " -D" + key + "=" + str(flag)
        cmakeCmd += " " + self.srcDir
        self.makeBuildDir()
        os.chdir(self.buildDir)
        return subprocess.call(cmakeCmd, shell=True)
    def build(self, target):
        makeCmd = "make " + target + " -j" + str(self.threads)
        os.chdir(self.buildDir)
        return subprocess.call(makeCmd, shell=True)


def runExperiment(cmake, configuration):
    cmake.setFlag("CMAKE_BUILD_TYPE", "Release")
    cmake.setFlag("ARGOS_ROBOTS_NUMBER", configuration["robots"])
    cmake.setFlag("ARGOS_TARGETS_NUMBER", configuration["targets"])
    mkdir(configuration["log"]["path"])
    cmake.setFlag("ARGOS_LOG", configuration["log"]["path"] + "/" + configuration["log"]["name"])
    cmake.configure()
    cmake.build("experiment_" + configuration["experiment"])


sourceDir = os.getcwd()
buildDir = sourceDir + "/build/release"
cmake = CmakeCommand(sourceDir, buildDir)

configuration = {
        "experiment" : "dynamic_mbfo",
        "robots" : 5,
        "targets" : 5,
        }

configuration["log"] = {
        "path" : buildDir + "/results/" + configuration["experiment"],
        "name" : str(configuration["robots"]) + "_" + str(configuration["targets"]) + ".json"
        }

runExperiment(cmake, configuration)

