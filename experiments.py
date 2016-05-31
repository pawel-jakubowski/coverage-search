#!/usr/bin/python
import os
import sys
import argparse
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


def generateConfig(buildDir, experiment, robots, targets):
    return {
            "experiment" : experiment,
            "robots" : robots,
            "targets" : targets,
            "log" : {
                "path" : buildDir + "/results/" + experiment,
                "name" : str(robots) + "_" + str(targets) + ".json"
                }
            }


def getLogData(logConfig):
    file = logConfig["path"] + "/" + logConfig["name"]
    with open(file, mode='r') as jsonFile:
        jsonData = json.load(jsonFile)
    return jsonData


def joinPlot(dataLabel, x, y):
    baseline, = plt.plot(x, y, label=dataLabel)
    plt.plot(x, y, 'o', color=baseline.get_color())


def configureSubplot(ax, title, labels, integerAxis = ["default", "default"], xlim = None):
    if integerAxis[0] is "int":
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    if integerAxis[1] is "int":
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    if xlim is not None:
        plt.xlim(xlim)
    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.grid(True)
    plt.legend(loc="best")
    plt.title(title)


def plotFoundTargets(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        fig = plt.figure(figNumber)
        fig.suptitle(experiment)
        ax = fig.gca()
        subplotNumber = len(config["targets"])*100 + 11
        for targets in config["targets"]:
            plt.subplot(subplotNumber)
            for robots in config["robots"]:
                subplotFoundTargets(ax, generateConfig(buildDir, experiment, robots, targets))
                configureSubplot(ax, str(targets)+" targets", ["Found targets", "Step"], ["int", "int"], [0, targets])
            plt.tight_layout()
            subplotNumber += 1
        figNumber += 1
    return figNumber


def subplotFoundTargets(ax, config):
    data = getLogData(config["log"])

    timeArray = [0]
    targetsArray = [0]
    foundTargets = 0

    for target in data["targets"]:
        foundTargets += 1
        timeArray.append(target["step"])
        targetsArray.append(foundTargets)
    
    timeArray.sort()
    joinPlot(str(config["robots"]) + " robots", targetsArray, timeArray)


def plotCoverage(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        fig = plt.figure(figNumber)
        fig.suptitle(experiment)
        ax = fig.gca()
        subplotNumber = len(config["targets"])*100 + 11
        for targets in config["targets"]:
            plt.subplot(subplotNumber)
            for robots in config["robots"]:
                subplotCoverage(ax, generateConfig(buildDir, experiment, robots, targets))
                configureSubplot(ax, str(targets)+" targets", ["Coverage [%]", "Step"], ["default", "int"])
            plt.tight_layout()
            subplotNumber += 1
        figNumber += 1
    return figNumber


def subplotCoverage(ax, config):
    data = getLogData(config["log"])

    timeArray = []
    coverageArray = []

    for threshold in data["thresholds"]:
        timeArray.append(threshold["step"])
        coverageArray.append(threshold["coverage"])
    
    joinPlot(str(config["robots"]) + " robots", coverageArray, timeArray)


def plotCompareCoverage(buildDir, config, figNumber = 1):
    fig = plt.figure(figNumber)
    ax = fig.gca()
    subplotNumber = len(config["robots"])*100 + 11
    for robots in config["robots"]:
        plt.subplot(subplotNumber)
        for experiment in config["experiments"]:
            coverageArray = []
            timeArray = []
            for targets in config["targets"]:
                configuration = generateConfig(buildDir, experiment, robots, targets)
                index = 0
                for threshold in getLogData(configuration["log"])["thresholds"]:
                    try:
                        timeArray[index] = threshold["step"]
                        coverageArray[index] = threshold["coverage"]
                    except IndexError:
                        timeArray.append(threshold["step"])
                        coverageArray.append(threshold["coverage"])
                    index += 1
            numberOfSamples = len(timeArray)
            timeArray = [x / numberOfSamples for x in timeArray]
            joinPlot(experiment, coverageArray, timeArray)
            configureSubplot(ax, str(robots)+" robots", ["Coverage [%]", "Step"], ["default", "int"])
        plt.tight_layout()
        subplotNumber += 1
    return figNumber


def plotCompareTargets(buildDir, config, figNumber = 1):
    fig = plt.figure(figNumber)
    ax = fig.gca()
    subplotNumber = len(config["robots"])*100 + 11
    for robots in config["robots"]:
        plt.subplot(subplotNumber)
        for experiment in config["experiments"]:
            targetsArray = [0]
            timeArray = [0]
            targets = config["targets"][-1]
            configuration = generateConfig(buildDir, experiment, robots, targets)
            targetsNumber = 1
            for target in getLogData(configuration["log"])["targets"]:
                timeArray.append(target["step"])
                targetsArray.append(targetsNumber)
                targetsNumber += 1
            timeArray.sort()
            joinPlot(experiment, targetsArray, timeArray)
            configureSubplot(ax, str(robots)+" robots", ["Targets", "Step"], ["int", "int"], [0, targets])
        plt.tight_layout()
        subplotNumber += 1
    return figNumber



#======#
# MAIN #
#======#
def main():
    parser = argparse.ArgumentParser(description="Execute ARGoS experiments")
    plotTypes = ["targets", "coverage", "compare-coverage", "compare-targets"]
    parser.add_argument("--no-exe", action="store_true", help="Do not execute experiments")
    parser.add_argument("--plot", nargs="+", choices=plotTypes, default=[], type=str, help="Plot given plot-type")
    args = parser.parse_args()

    sourceDir = os.getcwd()
    buildDir = sourceDir + "/build/release"
    cmake = CmakeCommand(sourceDir, buildDir)

    configuration = {
            "experiments" : ["mbfo", "dynamic_mbfo"],
            "robots" : [10, 25, 50],
            "targets" : [5, 10, 15]
            }

    figNumber = 1
    if not args.no_exe:
        runExperiments(cmake, configuration)
    if "targets" in args.plot:
        figNumber = plotFoundTargets(buildDir, configuration, figNumber)
    if "coverage" in args.plot:
        figNumber = plotCoverage(buildDir, configuration, figNumber)
    if "compare-coverage" in args.plot:
        figNumber = plotCompareCoverage(buildDir, configuration, figNumber)
    if "compare-targets" in args.plot:
        figNumber = plotCompareTargets(buildDir, configuration, figNumber)
    plt.show()

if __name__ == "__main__":
    main()
