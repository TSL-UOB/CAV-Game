import os
import sys
import csv
from datetime import datetime

data_file = "D:/University/3rdYear/Simulator/CAV/CAV-Game/Unreal/CarlaUE4/Data.csv"
openscenario_file = "D:/University/3rdYear/Simulator/CAV\CAV-Game/scenario_runner_0.9.13/srunner/examples/Scenario1.xosc"


with open(data_file, 'r') as f:
    lines = csv.reader(f, delimiter=',', quotechar='|')
    rows = list(lines)
#agentType = rows[1][0]
x = rows[1][1]
y = rows[1][2]
z = rows[1][3]
print(rows)


def writeTopLines(openscenario_file):
    with open(openscenario_file, 'w') as f:
        f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        f.write("<OpenSCENARIO>\n")
        f.write("  <FileHeader revMajor=\"1\" revMinor=\"0\" date=\"" + datetime.now().strftime("%Y-%m-%dT%H:%M:%S") + "\" description=\"CARLA:Converted from UI\" author=\"Rares Bucur\"/>\n")
        f.write(" <ParameterDeclarations>\n")
        f.write("  <ParameterDeclaration name=\"leadingSpeed\" parameterType=\"double\" value=\"2.0\"/>\n")
        f.write(" <ParameterDeclarations/>\n")
        f.write(" <CatalogLocations/>\n")
        f.write(" <RoadNetwork>\n")
        f.write("  <LogicFile filepath=\"Town01\"/>\n")
        f.write("  <SceneGraphFile filepath=""/>\n")
        f.write(" </RoadNetwork>\n")
        f.close()


def writeEntities(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write(" <Entities>\n")
        f.write("  <ScenarioObject name=\"hero\">\n")
        f.write("   <Vehicle name=\"vehicle.tesla.model3\" vehicleCategory=\"car\">\n")
        f.write("    <ParameterDeclarations/>\n")
        f.write("    <Performance maxSpeed=\"69.444\" maxAcceleration=\"200\" maxDeceleration=\"10.0\"/>\n")
        f.write("    <BoundingBox>\n")
        f.write("     <Center x=\"1.5\" y=\"0.0\" z=\"0.9\"/>\n")
        f.write("     <Dimensions width= \"2.1\" length= \"4.5\" height=\"1.8\"/>\n")
        f.write("    </BoundingBox>\n")
        f.write("    <Axles>\n")
        f.write("     <FrontAxle maxSteering=\"0.5\" wheelDiameter=\"0.6\" trackWidth=\"1.8\" positionX=\"3.1\" positionZ=\"0.3\"/>\n")
        f.write("     <RearAxle maxSteering=\"0.0\" wheelDiameter=\"0.6\" trackWidth=\"1.8\" positionX=\"0.0\" positionZ=\"0.3\"/>\n")
        f.write("    </Axles>\n")
        f.write("    <Properties>\n")
        f.write("     <Property name=\"type\" value=\"simulation\"/>\n")
        f.write("     <Property name=\"color\" value=\"255,0,0\"/>\n")
        f.write("    </Properties>\n")
        f.write("   </Vehicle>\n")
        f.write("  </ScenarioObject>\n")
        f.write(" </Entities>\n")
        f.close()


def writeInitActions(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write(" <Storyboard>\n")
        f.write("  <Init>\n")
        f.write("   <Actions>\n")
        f.write("    <GlobalAction>\n")
        f.write("     <EnvironmentAction>\n")
        f.write("      <Environment name=\"Environment1\">\n")
        f.write("       <TimeOfDay animation=\"false\" dateTime=\"" + datetime.now().strftime("%Y-%m-%dT%H:%M:%S") + "\"/>\n")
        f.write("       <Weather cloudState=\"free\">\n")
        f.write("        <Sun intensity=\"0.85\" azimuth=\"0\" elevation=\"1.31\"/>\n")
        f.write("        <Fog visualRange=\"100000.0\"/>\n")
        f.write("        <Precipitation precipitationType=\"dry\" intensity=\"0.0\"/>\n")
        f.write("       </Weather>\n")
        f.write("       <RoadCondition frictionScaleFactor=\"1.0\"/>\n")
        f.write("      </Environment>\n")
        f.write("     </EnvironmentAction>\n")
        f.write("    </GlobalAction>\n")
        f.close()


def initVehicles(openscenario_file, x, y, z):
    with open(openscenario_file, 'a') as f:
        f.write("    <Private entityRef=\"hero\">\n")
        f.write("     <PrivateAction>\n")
        f.write("      <TeleportAction>\n")
        f.write("       <Position>\n")
        f.write("        <WorldPosition x=\"" + x + "\" y=\"" + y + "\" z=\"" + z + "\" h=\"0\"/>\n")
        f.write("       </Position>\n")
        f.write("      </TeleportAction>\n")
        f.write("     </PrivateAction>\n")
        f.write("    </Private>\n")
        f.close()


def closeInitActions(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write("   </Actions>\n")
        f.write("  </Init>\n")
        f.close()


def writeStory(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write("  <Story name=\"FollowingRoute\">\n")
        f.write("   <Act name =\"Act1\">\n")
        f.write("    <ManeuverGroup name=\"Group1\" maximumExecutionCount=\"1\">\n")
        f.write("     <Actors selectTriggeringEntities=\"false\">\n")
        f.write("      <EntityRef entityRef=\"hero\"/>\n")
        f.write("     </Actors>\n")
        # f.write("     <Maneuver name=\"Standing\">\n")
        # f.write("      <ParameterDeclaration name=\"RunScenario\" parameterType=\"boolean\" value=\"false\"/>\n")
        # f.write("      <Event name=\"VehicleAwaits\" priority=\"overwrite\">\n")
        # f.write("       <Action name=\"VehicleWaits\">\n")
        # f.write("        <PrivateAction>\n")
        # f.write("         <LongitudinalAction>\n")
        # f.write("          <SpeedAction>\n")
        # f.write("           <SpeedActionDynamics dynamicsShape=\"step\" value=\"10\" dynamicsDimension=\"time\"/>\n")
        # f.write("           <SpeedActionTarget>\n")
        # f.write("            <AbsoluteTargetSpeed value=\"0.0\"/>\n")
        # f.write("           </SpeedActionTarget>\n")
        # f.write("          </SpeedAction>\n")
        # f.write("         </LongitudinalAction>\n")
        # f.write("        </PrivateAction>\n")
        # f.write("       </Action>\n")
        # f.write("       <StartTrigger>\n")
        # f.write("        <ConditionGroup>\n")
        # f.write("         <Condition name=\"StartCondition\" delay=\"0\" conditionEdge=\"rising\">\n")
        # f.write("          <ByValueCondition>\n")
        # f.write("           <ParameterCondition parameterRef=\"RunScenario\" value=\"false\" rule=\"equalTo\"/>\n")
        # f.write("          </ByValueCondition>\n")
        # f.write("         </Condition>\n")
        # f.write("        </ConditionGroup>\n")
        # f.write("       </StartTrigger>\n")
        # f.write("      </Event>\n")
        # f.write("     </Maneuver>\n")
        f.write("     <Maneuver name=\"FollowWaypoints\">\n")
        f.write("      <Event name=\"Route\" priority=\"overwrite\">\n")
        f.write("       <Action name =\"Assign Route\">\n")
        f.write("        <PrivateAction>\n")
        f.write("         <RoutingAction>\n")
        f.write("          <AssignRouteAction>\n")
        f.write("           <Route name=\"Route 1\" closed = \"false\">\n")
        f.write("           </Route>\n")
        f.write("          </AssignRouteAction>\n")
        f.write("         </RoutingAction>\n")
        f.write("        </PrivateAction>\n")
        f.write("       </Action>\n")
        f.write("       <StartTrigger>\n")
        f.write("        <ConditionGroup>\n")
        f.write("         <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("          <ByValueCondition>\n")
        f.write("           <SimulationTimeCondition value=\"0\" rule=\"greaterThan\"/>\n")
        f.write("          </ByValueCondition>\n")
        f.write("         </Condition>\n")
        f.write("        </ConditionGroup>\n")
        f.write("       </StartTrigger>\n")
        f.write("      </Event>\n")
        f.write("     </Maneuver>\n")
        f.write("    </ManeuverGroup>\n")
        f.write("    <StartTrigger>\n")
        f.write("     <ConditionGroup>\n")
        f.write("      <Condition name=\"StartCondition\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("       <ByValueCondition>\n")
        f.write("        <SimulationTimeCondition rule=\"greaterThan\" value=\"0.0\"/>\n")
        f.write("       </ByValueCondition>\n")
        f.write("      </Condition>\n")
        f.write("     </ConditionGroup>\n")
        f.write("    </StartTrigger>\n")
        f.write("   </Act>\n")
        f.write("  </Story>\n")
        f.write("  <StopTrigger>\n")
        f.write("   <ConditionGroup>\n")
        f.write("    <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("     <ByValueCondition>\n")
        f.write("      <SimulationTimeCondition value=\"30\" rule=\"greaterThan\"/>\n")
        f.write("     </ByValueCondition>\n")
        f.write("    </Condition>\n")
        f.write("   </ConditionGroup>\n")
        f.write("  </StopTrigger>\n")
        f.write(" </Storyboard>\n")
        f.write("</OpenSCENARIO>\n")



writeTopLines(openscenario_file)
writeEntities(openscenario_file)
writeInitActions(openscenario_file)
initVehicles(openscenario_file, x, y, z)
closeInitActions(openscenario_file)
writeStory(openscenario_file)




# FileName = "D:/University/3rdYear/Simulator/CAV/CAV-Game/Unreal/CarlaUE4/Data.csv"
# with open(FileName, 'r') as f:
#     lines = f.readlines()
# print(lines[0][1])

