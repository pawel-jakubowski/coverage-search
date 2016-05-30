#!/usr/bin/python
import os
import sys
import getopt
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


def plotFoundTargets(buildDir, config, figNumber = 1):
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
                subplotFoundTargets(ax, configuration)
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            ax.yaxis.set_major_locator(MaxNLocator(integer=True))
            plt.xlim([0, targets])
            plt.ylabel("Step")
            plt.xlabel("Found targets")
            plt.grid(True)
            plt.legend(loc="best")
            plt.title(str(targets) + " targets")
            plt.tight_layout()
            subplotNumber += 1
        figNumber += 1
    return figNumber


def subplotFoundTargets(ax, config):
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
    baseline, = plt.plot(targetsArray, timeArray, label=plotLabel)
    plt.plot(targetsArray, timeArray, 'o', color=baseline.get_color())


def plotCoverage(buildDir, config, figNumber = 1):
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
                subplotCoverage(ax, configuration)
            ax.yaxis.set_major_locator(MaxNLocator(integer=True))
            plt.ylabel("Step")
            plt.xlabel("Coverage [%]")
            plt.grid(True)
            plt.legend(loc="best")
            plt.title(str(targets) + " targets")
            plt.tight_layout()
            subplotNumber += 1
        figNumber += 1
    return figNumber


def subplotCoverage(ax, config):
    file = config["log"]["path"] + "/" + config["log"]["name"]
    with open(file, mode='r') as json_file:
        json_data = json.load(json_file)

    timeArray = []
    coverageArray = []

    for threshold in json_data["thresholds"]:
        timeArray.append(threshold["step"])
        coverageArray.append(threshold["coverage"])
    
    plotLabel = str(config["robots"]) + " robots"
    baseline, = plt.plot(coverageArray, timeArray, label=plotLabel)
    plt.plot(coverageArray, timeArray, 'o', color=baseline.get_color())


#======#
# MAIN #
#======#
def main(argv):
    try:
        opts, args = getopt.getopt(argv,"ntc",["no-exe","targets","coverage"])
    except getopt.GetoptError:
        print sys.argv[0] + ' -n -t -c'
        sys.exit(2)
    execute = True
    targets = False
    coverage = False
    for opt, arg in opts:
        if opt in ("-n", "--no-exe"):
            execute = False
        elif opt in ("-t", "--targets"):
            targets = True
        elif opt in ("-c", "--coverage"):
            coverage = True

    sourceDir = os.getcwd()
    buildDir = sourceDir + "/build/release"
    cmake = CmakeCommand(sourceDir, buildDir)

    configuration = {
            "experiments" : ["mbfo", "dynamic_mbfo"],
            "robots" : [10, 25, 50],
            "targets" : [5, 10, 15]
            }

    figNumber = 1
    if execute:
        runExperiments(cmake, configuration)
    if targets:
        figNumber = plotFoundTargets(buildDir, configuration, figNumber)
    if coverage:
        figNumber = plotCoverage(buildDir, configuration, figNumber)
    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
