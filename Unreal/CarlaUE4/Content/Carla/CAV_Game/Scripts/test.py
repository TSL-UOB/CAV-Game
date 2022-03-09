# import os
# import sys


# class FileLine:
#     agent_type = str()
#     x = float()
#     y = float()
#     z = float()

#     def FileLine(splitString = []):
#         agent_type = splitString[0]
#         x = float(splitString[1])
#         y = float(splitString[2])
#         z = float(splitString[3])

#     def getAgentType(self):
#         return self.agent_type

#     def getX(self):
#         return self.x

#     def getY(self):
#         return self.y

#     def getZ(self):
#         return self.z


# class Parser(FileLine):
#     data_lines = []
#     FileName = "D:/University/3rdYear/Simulator/CAV/CAV-Game/Unreal/CarlaUE4/Data.txt"

#     def getLines (self):
#         lines = []
#         file_lines = []
#         with open(self.FileName, 'r') as f:
#             lines = f.readlines()
#         for x in lines:
#             splitString = x.split(",")
#             file_lines.append = FileLine(splitString)
#         return file_lines

#     def getEntityNr(self):
#         firstEntity = self.data_lines[0].getAgentType()
#         k = 1
#         for i in range (1, len(self.data_lines)):
#             currentAgent = self.data_lines[i].getAgentType()
#             if currentAgent == firstEntity:
#                 return k
#             else:
#                 k += 1
#         return k

#     def parseLine(self, lineToParse):
#         splitString = lineToParse.split(",")
#         return FileLine(splitString)

#     def getDataLines(self):
#         return self.data_lines


# class Writer(Parser):
#     data_lines = []
#     def Writer(self, fileName, mapPath):
#         k = 0
#         filePath = "D:/University/3rdYear/Simulator/CAV/CAV-Game/scenario_runner_0.9.13/srunner/examples/vehicle1.xosc"
#         with open(filePath, 'x') as f:
#             fileToCreate = f
#         if os.path.exists(fileToCreate):
#              os.remove(fileToCreate)
#         else:
#             print("Cannot delete the file as it doesn't exists")
#         parseInput = Parser()
#         pathToMap = mapPath
#         velocities = ""




#     def writeTopLines(self):
#         try:
#             fileToCreate = open("D:/University/3rdYear/Simulator/CAV/CAV-Game/scenario_runner_0.9.13/srunner/examples/vehicle_spawn.xosc", "w")
#             fileToCreate.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
#             fileToCreate.write()
#             fileToCreate.write()
#             fileToCreate.write()
#             fileToCreate.write()
#             fileToCreate.write()
#             fileToCreate.write()
#             fileToCreate.write()
#         except:
#             print("Could not create file!")




#     def writeEntities(self):
#         w 
    



#     def classifyObject(self):
#         w
    



#     def initActions(self):
#         W

    


#     def initStory(self):
#         W
    



#     def storyManeuvers(self):
#         W
    



#     def eventActions(self):
#         W
    



#     def setSpeedEvents(self):
#         W