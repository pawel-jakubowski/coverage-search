#!/usr/bin/python
import os
import errno
import subprocess
import json
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

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
        returnValue = subprocess.call(cmakeCmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if returnValue != 0:
            raise
    def build(self, target):
        makeCmd = "make " + target + " -j" + str(self.threads)
        os.chdir(self.buildDir)
        returnValue = subprocess.call(makeCmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if returnValue != 0:
            raise


def runExperiments(cmake, config):
    for experiment in config["experiments"]:
        for targets in config["targets"]:
            for robots in config["robots"]:
                configuration = {
                        "experiment" : experiment,
                        "robots" : robots,
                        "targets" : targets,
                        "log" : {
                            "path" : cmake.buildDir + "/results/" + experiment,
                            "name" : str(robots) + "_" + str(targets) + ".json"
                            }
                        }
                runExperiment(cmake, configuration)


def runExperiment(cmake, config):
    print "Experiment " + config["experiment"] +\
          " with " + str(config["robots"]) + " robots and " +\
          str(config["targets"]) + " targets"
    cmake.setFlag("CMAKE_BUILD_TYPE", "Release")
    cmake.setFlag("ARGOS_ROBOTS_NUMBER", config["robots"])
    cmake.setFlag("ARGOS_TARGETS_NUMBER", config["targets"])
    mkdir(config["log"]["path"])
    cmake.setFlag("ARGOS_LOG", config["log"]["path"] + "/" + config["log"]["name"])
    print "\t configure..."
    cmake.configure()
    print "\t build & run..."
    cmake.build("experiment_" + config["experiment"])
    print "\t done!"


def plotExperiments(buildDir, config):
    figNumber = 1
    for experiment in config["experiments"]:
        fig = plt.figure(figNumber)
        fig.suptitle(experiment)
        ax = fig.gca()
        subplotNumber = len(config["targets"])*100 + 11
        for targets in config["targets"]:
            plt.subplot(subplotNumber)
            for robots in config["robots"]:
                configuration = {
                        "experiment" : experiment,
                        "robots" : robots,
                        "targets" : targets,
                        "log" : {
                            "path" : buildDir + "/results/" + experiment,
                            "name" : str(robots) + "_" + str(targets) + ".json"
                            }
                        }
                plotExperiment(ax, configuration)
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            ax.yaxis.set_major_locator(MaxNLocator(integer=True))
            plt.xlabel("Step")
            plt.ylabel("Found targets")
            plt.grid(True)
            plt.legend(loc="lower right")
            plt.title(str(targets) + " targets")
            plt.tight_layout()
            subplotNumber += 1
        figNumber += 1
    plt.show()


def plotExperiment(ax, config):
    file = config["log"]["path"] + "/" + config["log"]["name"]
    with open(file, mode='r') as json_file:
        json_data = json.load(json_file)

    timeArray = []
    targetsArray = []
    foundTargets = 0

    for target in json_data["targets"]:
        foundTargets += 1
        timeArray.append(target["step"])
        targetsArray.append(foundTargets)
    
    timeArray.sort()
    plotLabel = str(config["robots"]) + " robots"
    baseline, = plt.plot(timeArray, targetsArray, label=plotLabel)
    plt.plot(timeArray, targetsArray, 'o', color=baseline.get_color())


sourceDir = os.getcwd()
buildDir = sourceDir + "/build/release"
cmake = CmakeCommand(sourceDir, buildDir)

configuration = {
        "experiments" : ["mbfo", "dynamic_mbfo"],
        "robots" : [10, 25, 50],
        "targets" : [5, 10, 15]
        }

#runExperiments(cmake, configuration)
plotExperiments(buildDir, configuration)

