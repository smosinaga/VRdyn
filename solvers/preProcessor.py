from models import tyreSystem
from models import carSystem
from models import sbSystem
from solvers import odenint
from solvers import posProcessor
import importlib, json, pathlib
import sys

def read(caseName):
    # Open the input file
    casePath = pathlib.Path(__file__).parent.absolute()/".."/"input"
    caseNameExt = caseName + ".json" #Case name with extension .json
    with open(casePath/caseNameExt, 'r') as f:
        caseDict = json.load(f) # Load the input file
    
    #tyre model
    tyreModel = getattr(tyreSystem, caseDict['tyreType'])(caseDict)

    # car model
    carModel = getattr(carSystem, caseDict['carType'])(caseDict)
    
    #speed bump model
    sbModel = getattr(sbSystem, caseDict['sbType'])(caseDict)
    
    #numerical integration
    sol = odenint.solution(caseDict, tyreModel, carModel, sbModel)
    
    #posProcessing - Plot profile
    if caseDict['conf']['plotProfile'] == "y":
        posProcessor.plotProfile(caseDict, sbModel)
    elif caseDict['conf']['plotProfile'] == "n":
        pass
    else:
        print("ERROR: plotProfile must be 'y' or 'n'"); sys.exit()
        
    #posProcessing - Plot Disp
    if caseDict['conf']['plotDisp'] == "y":
        posProcessor.plotDisp(sol)
    elif caseDict['conf']['plotDisp'] == "n":
        pass
    else:
        print("ERROR: plotDisp must be 'y' or 'n'"); sys.exit()

    #posProcessing - Plot Vel
    if caseDict['conf']['plotVelo'] == "y":
        posProcessor.plotVelo(sol)
    elif caseDict['conf']['plotVelo'] == "n":
        pass
    else:
        print("ERROR: plotVelo must be 'y' or 'n'"); sys.exit()
        
    #posProcessing - Plot forces
    if caseDict['conf']['plotForces'] == "y":
        posProcessor.plotForces(sol)
    elif caseDict['conf']['plotForces'] == "n":
        pass
    else:
        print("ERROR: plotForces must be 'y' or 'n'"); sys.exit()
    
    #posProcessing - exporting numerical data
    if caseDict['conf']['exportResults'] == "y":
        posProcessor.exportResults(caseName,sol)
    elif caseDict['conf']['exportResults'] == "n":
        pass
    else:
        print("ERROR: exportResults must be 'y' or 'n'"); sys.exit()

    return tyreModel,carModel, sbModel, sol