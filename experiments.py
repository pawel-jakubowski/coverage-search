#!/usr/bin/python
import os
import sys
import argparse
import errno
import subprocess
import shlex
import json
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

#matplotlib.style.use('ggplot')

def mkdir(path):
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            pass
        else:
            raise


class CmakeCommand:
    maxTries = 10 
    threads = 4
    def __init__(self, srcDir, buildDir = None):
        if buildDir is None:
            buildDir = srcDir + "/build"
        self.srcDir = srcDir
        self.buildDir = buildDir
        self.flags = {}
        self.stdout = subprocess.PIPE
        self.stderr = subprocess.PIPE
    def setVerbose(self):
        self.stdout = None
        self.stderr = None
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
        proc = subprocess.Popen(shlex.split(cmakeCmd), stdout=self.stdout, stderr=self.stderr)
        outs, errs = proc.communicate()
        if proc.returncode:
            print "Error!"
            print errs
            exit(-1)

    def build(self, target):
        makeCmd = "make " + target + " -j" + str(self.threads)
        os.chdir(self.buildDir)
        tryNumber = 0
        run = True

        while run:
            proc = subprocess.Popen(shlex.split(makeCmd), stdout=self.stdout, stderr=self.stderr)
            outs, errs = proc.communicate()
            tryNumber += 1
            if proc.returncode == 0 or tryNumber == self.maxTries:
                run = False

        if proc.returncode != 0:
            print "Error!"
            print errs
            exit(-1)


def printExperimentMsg(experiment, robots, targets, percentage):
    if percentage < 100:
        percMsg = "%2d%%" % percentage
    else:
        percMsg = "Done!"
    print "[%s] Experiment %s with %d robots and %d targets" % (percMsg, experiment, robots, targets)


def generateConfig(buildDir, experiment, robots, targets, i):
    return {
            "experiment" : experiment,
            "robots" : robots,
            "targets" : targets,
            "log" : {
                "path" : buildDir + "/results/" + experiment,
                "name" : "r" + str(robots) + "t" + str(targets) + "_" + str(i) + ".json"
                }
            }


def runExperiments(cmake, config, verbose):
    runs = config["repetitions"]
    for experiment in config["experiments"]:
        for targets in config["targets"]:
            for robots in config["robots"]:
                for i in range(0, runs):
                    configuration = generateConfig(cmake.buildDir, experiment, robots, targets, i) 
                    printExperimentMsg(experiment, robots, targets, i*100/runs)
                    runExperiment(cmake, configuration, config["debug"], verbose)
                    sys.stdout.write("\033[F")
                    if i+1 == runs:
                        printExperimentMsg(experiment, robots, targets, 100)


def runExperiment(cmake, config, debugMode, verbose):
    if debugMode:
        buildType = "Debug"
    else:
        buildType = "Release"
    cmake.setFlag("CMAKE_BUILD_TYPE", buildType)

    cmake.setFlag("ARGOS_ROBOTS_NUMBER", config["robots"])
    cmake.setFlag("ARGOS_TARGETS_NUMBER", config["targets"])
    mkdir(config["log"]["path"])
    cmake.setFlag("ARGOS_LOG", config["log"]["path"] + "/" + config["log"]["name"])
    if verbose:
        cmake.setVerbose()
    print "\t configure..."
    cmake.configure()
    print "\t build & run..."
    cmake.build("experiment_" + config["experiment"])
    sys.stdout.write("\033[F")
    sys.stdout.write("\033[F")


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


def plotLightDistance(buildDir, config, figNumber = 1):
    #for experiment in config["experiments"]:
    experiment = "pso"
    fig = plt.figure(figNumber)
    fig.suptitle(experiment)
    ax = fig.gca()
    for robots in config["robots"]:
        distancesArray = []
        timeArray = []
        configuration = generateConfig(buildDir, experiment, robots, 0)
        for distance in getLogData(configuration["log"])["closest"]:
            timeArray.append(distance["step"])
            distancesArray.append(distance["distance"])
        plt.plot(timeArray, distancesArray, label=str(robots) + " robots")
        configureSubplot(ax, "", ["Algorithm steps", "Distance [m]"], ["int", "default"])
    plt.tight_layout()
    figNumber += 1
    return figNumber

def plotLightMean(buildDir, config, figNumber = 1):
    experiment = pso
    plt.figure(figNumber, figsize=(10,5))
    for robots in config["robots"]:
        robotsDf = getLightDataFrame(buildDir, experiment, robots, config["repetitions"])
        print robotsDf
        ma = robotsDf.mean()
        msem = robotsDf.sem()
        baseline, = plt.plot(ma.index, ma, 'o')
        plt.plot(ma.index, ma, color=baseline.get_color(), label=str(robots)+" robots")
        plt.fill_between(msem.index, ma-msem, ma+msem, color=baseline.get_color(), alpha=0.2)
    plt.grid()
    configureSubplot(plt.gca(), experiment+", "+str(targets)+" targets", ["Found targets", "Algorithm steps"], ["int", "default"])
    figNumber += 1
    return figNumber

def getTargetsData(buildDir, experiment, robots, targets, repetitions):
    avgTimeArray = []
    timeArrayUpdates = []
    targetsArray = [0]

    for i in range(0, repetitions):
        config = generateConfig(buildDir, experiment, robots, targets, i)
        data = getLogData(config["log"])
        timeArray = [0]
        foundTargets = 0
        for target in data["targets"]:
            foundTargets += 1
            timeArray.append(target["step"])
            if foundTargets >= len(targetsArray):
                targetsArray.append(foundTargets)
        timeArray.sort()
        for index, time in enumerate(timeArray):
            try:
                avgTimeArray[index] += time
                timeArrayUpdates[index] += 1
            except IndexError:
                avgTimeArray.append(time)
                timeArrayUpdates.append(1)

    avgTimeArray = [x / timeArrayUpdates[index] for index, x in enumerate(avgTimeArray)]
    return avgTimeArray, targetsArray

def getExperimentDataFrame(buildDir, experiment, robots, targets, repetitions):
    df = pd.DataFrame()
    for i in range(0, repetitions):
        df[i] = pd.Series(np.full(targets, np.nan))
        config = generateConfig(buildDir, experiment, robots, targets, i)
        data = getLogData(config["log"])
        timeArray = []
        for target in data["targets"]:
            timeArray.append(target["step"])
        timeArray.sort()
        for j, time in enumerate(timeArray):
            df[i][j] = time
    return df.transpose()


def plotFoundTargets(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        fig = plt.figure(figNumber)
        fig.canvas.set_window_title(experiment + "_targets")
        ax = fig.gca()
        subplotNumber = len(config["targets"])*100 + 11
        for targets in config["targets"]:
            plt.subplot(subplotNumber)
            for robots in config["robots"]:
                timeArray, targetsArray = getTargetsData(buildDir, experiment, robots, targets, config["repetitions"])
                joinPlot(str(robots) + " robots", targetsArray, timeArray)
                configureSubplot(ax, str(targets)+" targets", ["Found targets", "Algorithm steps"], ["int", "int"], [0, targets])
            plt.tight_layout()
            subplotNumber += 1
        figNumber += 1
    return figNumber

def plotTargetsAccuracy(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        for targets in config["targets"]:
            df = pd.DataFrame()
            for robots in config["robots"]:
                robotsDf = getExperimentDataFrame(buildDir, experiment, robots, targets, config["repetitions"])
                df[str(robots) + " robots"] = robotsDf.count()
                print robotsDf.describe()
            df.index += 1
            print df
            df.plot(kind="bar", rot=0, figsize=(11,6), alpha=.7)
            plt.grid()
            configureSubplot(plt.gca(), experiment+", "+str(targets)+" targets", ["Found targets", "Experiments that found target [%]"])
    return figNumber

def plotBoxTargets(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        for targets in config["targets"]:
            for robots in config["robots"]:
                df = getExperimentDataFrame(buildDir, experiment, robots, targets, config["repetitions"])
                df.columns += 1
                df.plot.box(rot=0, showfliers=False, figsize=(10,5))
                plt.grid()
                configureSubplot(plt.gca(), experiment+", "+str(targets)+" targets, "+str(robots)+" robots", ["Found targets", "Algorithm steps"])
                print str(robots) + " robots"
                print df.describe().transpose()
                print df.sem()
    return figNumber

def plotTargetsMean(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        for targets in config["targets"]:
            plt.figure(figNumber, figsize=(10,5))
            for robots in config["robots"]:
                robotsDf = getExperimentDataFrame(buildDir, experiment, robots, targets, config["repetitions"])
                robotsDf.columns += 1
                #print robotsDf
                #print robotsDf.count()
                ma = robotsDf.mean()
                msem = robotsDf.sem()
                print ma
                print msem
                baseline, = plt.plot(ma.index, ma, 'o')
                plt.plot(ma.index, ma, color=baseline.get_color(), label=str(robots)+" robots")
                plt.fill_between(msem.index, ma-msem, ma+msem, color=baseline.get_color(), alpha=0.2)
            plt.grid()
            configureSubplot(plt.gca(), experiment+", "+str(targets)+" targets", ["Found targets", "Algorithm steps"], ["int", "default"])
            figNumber += 1
    return figNumber

def getCoverageData(buildDir, experiment, robots, targetsArray, repetitions):
    coverageArray = []
    coverageArrayUpdates = []
    timeArray = []
    timeArrayUpdates = []
    for targets in targetsArray:
        for i in range(0, repetitions):
            configuration = generateConfig(buildDir, experiment, robots, targets, i)
            index = 0
            for threshold in getLogData(configuration["log"])["thresholds"]:
                try:
                    timeArray[index] += threshold["step"]
                    timeArrayUpdates[index] += 1
                    coverageArray[index] += threshold["coverage"]
                    coverageArrayUpdates[index] += 1
                except IndexError:
                    timeArray.append(threshold["step"])
                    timeArrayUpdates.append(1)
                    coverageArray.append(threshold["coverage"])
                    coverageArrayUpdates.append(1)
                index += 1
    timeArray = [x / timeArrayUpdates[index] for index, x in enumerate(timeArray)]
    coverageArray = [x / coverageArrayUpdates[index] for index, x in enumerate(coverageArray)]
    return timeArray, coverageArray

def getCoverageDataFrame(buildDir, experiment, robots, targets, repetitions):
    df = pd.DataFrame()
    for i in range(0, repetitions):
        config = generateConfig(buildDir, experiment, robots, targets, i)
        data = getLogData(config["log"])
        timeArray = [0]
        coverageArray = [0]
        for threshold in data["thresholds"]:
            timeArray.append(threshold["step"])
            coverageArray.append(threshold["coverage"])
        timeArray.sort()
        df[i] = pd.Series(np.full(len(coverageArray), np.nan))
        for j, time in enumerate(timeArray):
            df[i][j] = time
    df.index = coverageArray
    return df.transpose()

def plotCoverage(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        if experiment == "pso" or experiment == "cellular_decomposition":
            continue
        fig = plt.figure(figNumber)
        fig.canvas.set_window_title(experiment + "_coverage")
        ax = fig.gca()
        for robots in config["robots"]:
            timeArray, coverageArray = getCoverageData(buildDir, experiment, robots, config["targets"], config["repetitions"])
            joinPlot(str(robots) + " robots", coverageArray, timeArray)
            configureSubplot(ax, "", ["Coverage [%]", "Algorithm steps"], ["default", "int"])
        plt.tight_layout()
        figNumber += 1
    return figNumber

def plotCoverageMean(buildDir, config, figNumber = 1):
    for experiment in config["experiments"]:
        if experiment == "pso" or experiment == "cellular_decomposition":
            continue
        plt.figure(figNumber, figsize=(10,5))
        for robots in config["robots"]:
            robotsDf = getCoverageDataFrame(buildDir, experiment, robots, config["targets"][-1], config["repetitions"])
            print robotsDf.mean()
            ma = robotsDf.mean()
            msem = robotsDf.sem()
            baseline, = plt.plot(ma.index, ma, 'o')
            plt.plot(ma.index, ma, color=baseline.get_color(), label=str(robots)+" robots")
            plt.fill_between(msem.index, ma-msem, ma+msem, color=baseline.get_color(), alpha=0.2)
        plt.grid()
        configureSubplot(plt.gca(), experiment+", "+str(config["targets"][-1])+" targets", ["Coverage [%]", "Algorithm steps"])
        figNumber += 1
    return figNumber

def plotCompareCoverage(buildDir, config, figNumber = 1):
    fig = plt.figure(figNumber)
    ax = fig.gca()
    fig.canvas.set_window_title("coverage_compare")
    subplotNumber = len(config["robots"])*100 + 11
    for robots in config["robots"]:
        plt.subplot(subplotNumber)
        for experiment in config["experiments"]:
            if experiment == "pso" or experiment == "cellular_decomposition":
                continue
            timeArray, coverageArray = getCoverageData(buildDir, experiment, robots, config["targets"], config["repetitions"])
            joinPlot(experiment, coverageArray, timeArray)
            configureSubplot(ax, str(robots)+" robots", ["Coverage [%]", "Algorithm steps"], ["default", "int"])
        plt.tight_layout()
        subplotNumber += 1
    return figNumber

def plotCompareCoverageMean(buildDir, config, figNumber = 1):
    for robots in config["robots"]:
        plt.figure(figNumber, figsize=(10,5))
        for experiment in config["experiments"]:
            if experiment == "pso" or experiment == "cellular_decomposition":
                continue
            robotsDf = getCoverageDataFrame(buildDir, experiment, robots, config["targets"][-1], config["repetitions"])
            print robotsDf.mean()
            ma = robotsDf.mean()
            msem = robotsDf.sem()
            baseline, = plt.plot(ma.index, ma, 'o')
            plt.plot(ma.index, ma, color=baseline.get_color(), label=experiment)
            plt.fill_between(msem.index, ma-msem, ma+msem, color=baseline.get_color(), alpha=0.2)
        plt.grid()
        configureSubplot(plt.gca(), str(config["targets"][-1])+" targets, "+str(robots)+" robots", ["Coverage [%]", "Algorithm steps"])
        figNumber += 1
    return figNumber

def plotCompareTargets(buildDir, config, figNumber = 1):
    fig = plt.figure(figNumber)
    ax = fig.gca()
    fig.canvas.set_window_title("targets_compare")
    subplotNumber = len(config["robots"])*100 + 11
    for robots in config["robots"]:
        plt.subplot(subplotNumber)
        for experiment in config["experiments"]:
            targets = config["targets"][-1]
            timeArray, targetsArray = getTargetsData(buildDir, experiment, robots, targets, config["repetitions"])
            joinPlot(experiment, targetsArray, timeArray)
            configureSubplot(ax, str(robots)+" robots", ["Targets", "Algorithm steps"], ["int", "int"], [0, targets])
        plt.tight_layout()
        subplotNumber += 1
    return figNumber

def plotCompareTargetsMean(buildDir, config, figNumber = 1):
    for robots in config["robots"]:
        plt.figure(figNumber, figsize=(10,5))
        for experiment in config["experiments"]:
            robotsDf = getExperimentDataFrame(buildDir, experiment, robots, config["targets"][-1], config["repetitions"])
            robotsDf.columns += 1
            print robotsDf.mean()
            ma = robotsDf.mean()
            msem = robotsDf.sem()
            baseline, = plt.plot(ma.index, ma, 'o')
            plt.plot(ma.index, ma, color=baseline.get_color(), label=experiment)
            plt.fill_between(msem.index, ma-msem, ma+msem, color=baseline.get_color(), alpha=0.2)
        plt.grid()
        configureSubplot(plt.gca(), str(config["targets"][-1])+" targets, "+str(robots)+" robots", ["Found targets", "Algorithm steps"])
        figNumber += 1
    return figNumber


#======#
# MAIN #
#======#
def main():
    parser = argparse.ArgumentParser(description="Execute ARGoS experiments")
    plotTypes = ["light", "targets", "coverage", "coverage-compare", "targets-compare", "targets-box", "targets-accuracy"]
    parser.add_argument("--no-exe", action="store_true", help="do not execute experiments")
    parser.add_argument("--verbose", action="store_true", help="print verbose log")
    parser.add_argument("--plot", nargs="+", choices=plotTypes, default=[], type=str, help="plot given plot-type")
    parser.add_argument("--custom", action="store_true", help="give custom experiments configuration")
    args = parser.parse_args()

    sourceDir = os.getcwd()
    buildDir = sourceDir + "/build/release"

    configuration = {
            "debug" : False,
            "experiments" : ["pso", "mbfo", "dynamic_mbfo", "cellular_decomposition"],
            "robots" : [10, 25, 50],
            "targets" : [5, 10, 15],
            "repetitions": 100
            }

    if args.custom:
        isDebug = raw_input("Debug mode (y/N): ").lower()
        if isDebug == 'y':
            configuration["debug"] = True
            buildDir = sourceDir + "/build/debug"
        elif not isDebug in ('n', ''):
            print "Error: Answer 'y' or 'n'"
            exit()

        configuration["experiments"] = raw_input("Experiments types: ").split(" ")
        configuration["robots"] = [int(x) for x in raw_input("Number of robots: ").split(" ")]
        configuration["targets"] = [int(x) for x in raw_input("Number of targets: ").split(" ")]
        configuration["repetitions"] = int(raw_input("Repetitions: "))
        
    print "Configuration:", 
    print configuration
    print "Build dir:",
    print buildDir

    cmake = CmakeCommand(sourceDir, buildDir)
    figNumber = 1
    if not args.no_exe:
        runExperiments(cmake, configuration, args.verbose)
    if "light" in args.plot:
        figNumber = plotLightDistance(buildDir, configuration, figNumber)
    if "targets" in args.plot:
        figNumber = plotTargetsMean(buildDir, configuration, figNumber)
    if "coverage" in args.plot:
        figNumber = plotCoverageMean(buildDir, configuration, figNumber)
    if "coverage-compare" in args.plot:
        figNumber = plotCompareCoverageMean(buildDir, configuration, figNumber)
    if "targets-compare" in args.plot:
        figNumber = plotCompareTargetsMean(buildDir, configuration, figNumber)
    if "targets-box" in args.plot:
        figNumber = plotBoxTargets(buildDir, configuration, figNumber)
    if "targets-accuracy" in args.plot:
        figNumber = plotTargetsAccuracy(buildDir, configuration, figNumber)

    plt.show()

if __name__ == "__main__":
    main()
