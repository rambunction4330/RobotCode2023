#!/usr/bin/env python3
import os, sys
import datetime
import json
import argparse

# parse arguments
argParser = argparse.ArgumentParser(
    description="Generate a compile_commands.json for an frc gradle project")
argParser.add_argument("--compiler", type=str,
                       help="path to the cross-compiler that gradleRIO uses. If not specified, the script will search for it")
argParser.add_argument("--build-type", "-bt",
                       choices=["Debug", "Release"], default="Debug", help="The type of build. Can be Debug or Release. Defaults to Debug")
argParser.add_argument("--year", "-y", type=int,
                       help="The year of the toolchain/project. Defaults to the current year", default=datetime.date.today().year)
argParser.add_argument("--source", "-S", "-s", type=str,
                       help="The root directory of the project(should be the directory with your build.gradle file. Defaults to cwd", default=os.getcwd())
args = argParser.parse_args()

year = args.year
buildType = args.build_type

if not os.path.exists(args.source):
    print("project directory does not exist!")
    exit(1)
elif not os.path.exists(os.path.join(args.source, "build.gradle")):
    print("project directory does not contain a build.gradle! Is this a project directory?")
    exit(1)
else:
    os.chdir(args.source)
# finds the roboRIO g++ cross compiler


def findCompiler() -> str:
    if args.compiler:
        if os.path.exists(args.compiler):
            return args.compiler
        else:
            print(f"Compiler path does not exist! Looking elsewhere...")

    name = os.path.sep + \
        os.path.join(f"{year}", "roborio", "bin",
                     f"arm-frc{year}-linux-gnueabi-g++")
    path = os.path.expanduser('~')
    for root, dirs, files in os.walk(path):
        for file in files:
            if name in os.path.join(root, file):
                return os.path.join(root, file)

# find the options file


def findOptionsTXT() -> str():
    name = os.path.join(
        f"compileFrcUserProgram{buildType}ExecutableFrcUserProgramCpp", "options.txt")
    for root, dirs, files in os.walk(os.getcwd()):
        for file in files:
            if name in os.path.join(root, file):
                return os.path.join(root, file)

# get the options from the file (replace newlines with spaces)


def getOptionString(filePath: str) -> str:
    optionString = None
    with open(filePath) as file:
        dat = file.read()
        optionsString = dat.replace("\n", " ")

    return optionsString

# find all cpp files


def getAllCppFiles():
    foundFiles = []
    for root, dirs, files in os.walk(os.getcwd()):
        for file in files:
            if file.endswith(".cpp"):
                foundFiles.append(os.path.join(root, file))

    return foundFiles


def findFile(filename: str, filepath: str) -> str:
    for root, dirs, files in os.walk(filepath):
        if filename in files:
            return os.path.join(root, filename)


def generateCompileCommands(compilerPath: str, optionString: str):
    model = []
    for file in getAllCppFiles():
        objectFile = os.path.basename(file).replace(".cpp", ".o")
        objectFile = findFile(objectFile, os.path.join(os.getcwd(), "build"))

        if objectFile == None:
            print(
                f"  failed to find object file for cpp file \"{file}\". Skipping...")
            continue
        else:
            print(f"  generating definition for {file}")

        fileObj = {
            "directory": os.path.join(os.getcwd(), "build"),
            "command": f"{compilerPath} {optionString} -o {objectFile} {file}",
            "file": f"{file}"
        }

        model.append(fileObj)

    print(f"dumping to {os.path.join(os.getcwd(), 'compile_commands.json')}")
    with open("compile_commands.json", mode='w') as compilationDatabaseFile:
        json.dump(model, compilationDatabaseFile)


def main():

    # find compiler
    print("searching for compiler... ", end="")
    compilerPath = findCompiler()
    if compilerPath == None:
        print("Failed to find compiler!")
        exit(1)
    print(f"using {compilerPath}")

    # find the options.txt file which lists the compile options
    print("searching for options.txt...", end="")
    optionsFilePath = findOptionsTXT()
    if optionsFilePath == None:
        print("Could not find options.txt. Perhaps you need to build?")
        exit(1)
    print(f"using {optionsFilePath}")

    # get the options
    print("getting compiler options... ", end="")
    compilerOptions = getOptionString(optionsFilePath)
    if compilerOptions == None:
        print("could not get compiler options for some reason!")
        exit(1)
    print("success")

    # generate the file
    print("generating compile_commands.json... ")
    generateCompileCommands(compilerPath, compilerOptions)


if __name__ == "__main__":
    main()
