import os
import sys
import csv
from datetime import datetime

data_file = "D:/University/3rdYear/Simulator/CAV/CAV-Game/Unreal/CarlaUE4/Data.csv"
openscenario_file = "D:/University/3rdYear/Simulator/CAV\CAV-Game/scenario_runner_0.9.13/srunner/examples/Scenario2.xosc"


with open(data_file, 'r') as f:
    lines = csv.reader(f, delimiter=',', quotechar='|')
    rows = list(lines)
# x = rows[1][1]
# y = rows[1][2]
# z = rows[1][3]
x = 190
y = 133
z = 0
print(rows)



def writeTopLines(openscenario_file):
    with open(openscenario_file, 'w') as f:
        f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        f.write("<OpenSCENARIO>\n")
        f.write("  <FileHeader revMajor=\"1\" revMinor=\"0\" date=\"" + datetime.now().strftime("%Y-%m-%dT%H:%M:%S") + "\" description=\"CARLA:Converted from UI\" author=\"Rares Bucur\"/>\n")
        f.write("  <ParameterDeclarations>\n")
        f.write("    <ParameterDeclaration name=\"leadingSpeed\" parameterType=\"double\" value=\"8.0\"/>\n")
        f.write("  </ParameterDeclarations>\n")
        f.write("  <CatalogLocations/>\n")
        f.write("  <RoadNetwork>\n")
        f.write("    <LogicFile filepath=\"Town01\"/>\n")
        f.write("    <SceneGraphFile filepath=\"\"/>\n")
        f.write("  </RoadNetwork>\n")
        f.close()



def writeEntities(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write("  <Entities>\n")
        f.write("    <ScenarioObject name=\"vehicle\">\n")
        f.write("      <Vehicle name=\"vehicle.tesla.model3\" vehicleCategory=\"car\">\n")
        f.write("        <ParameterDeclarations/>\n")
        f.write("        <Performance maxSpeed=\"69.444\" maxAcceleration=\"200\" maxDeceleration=\"10.0\"/>\n")
        f.write("        <BoundingBox>\n")
        f.write("          <Center x=\"1.5\" y=\"0.0\" z=\"0.9\"/>\n")
        f.write("          <Dimensions width= \"2.1\" length= \"4.5\" height=\"1.8\"/>\n")
        f.write("        </BoundingBox>\n")
        f.write("        <Axles>\n")
        f.write("          <FrontAxle maxSteering=\"0.5\" wheelDiameter=\"0.6\" trackWidth=\"1.8\" positionX=\"3.1\" positionZ=\"0.3\"/>\n")
        f.write("          <RearAxle maxSteering=\"0.0\" wheelDiameter=\"0.6\" trackWidth=\"1.8\" positionX=\"0.0\" positionZ=\"0.3\"/>\n")
        f.write("        </Axles>\n")
        f.write("        <Properties>\n")
        f.write("          <Property name=\"type\" value=\"simulation\"/>\n")
        f.write("          <Property name=\"color\" value=\"255,0,0\"/>\n")
        f.write("        </Properties>\n")
        f.write("      </Vehicle>\n")
        f.write("    </ScenarioObject>\n")
        f.write("  </Entities>\n")
        f.close()



def writeInitActions(openscenario_file, x, y, z):
    with open(openscenario_file, 'a') as f:
        f.write("  <Storyboard>\n")
        f.write("    <Init>\n")
        f.write("      <Actions>\n")
        f.write("        <Private entityRef=\"vehicle\">\n")
        f.write("          <PrivateAction>\n")
        f.write("            <TeleportAction>\n")
        f.write("              <Position>\n")
        f.write("                <WorldPosition x=\"" + str(x) + "\" y=\"" + str(y) + "\" z=\"" + str(z) + "\" h=\"0\"/>\n")
        f.write("              </Position>\n")
        f.write("            </TeleportAction>\n")
        f.write("          </PrivateAction>\n")
        f.write("        </Private>\n")
        f.write("      </Actions>\n")
        f.write("    </Init>\n")
        f.close()



def writeStory(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write("    <Story name=\"Scenario\">\n")
        f.write("      <Act name =\"Behaviour\">\n")
        f.write("        <ManeuverGroup name=\"ManeuverSequence\" maximumExecutionCount=\"1\">\n")
        f.write("          <Actors selectTriggeringEntities=\"false\">\n")
        f.write("            <EntityRef entityRef=\"vehicle\"/>\n")
        f.write("          </Actors>\n")
        f.write("          <Maneuver name=\"VehicleManeuver\">\n")
        f.write("            <Event name=\"VehicleReachingVelocity\" priority=\"overwrite\">\n")
        f.write("              <Action name=\"VehicleReachingVelocity\">\n")
        f.write("                <PrivateAction>\n")
        f.write("                  <LongitudinalAction>\n")
        f.write("                    <SpeedAction>\n")
        f.write("                      <SpeedActionDynamics dynamicsShape=\"step\" value=\"40\" dynamicsDimension=\"distance\"/>\n")
        f.write("                      <SpeedActionTarget>\n")
        f.write("                        <AbsoluteTargetSpeed value=\"$leadingSpeed\"/>\n")
        f.write("                      </SpeedActionTarget>\n")
        f.write("                    </SpeedAction>\n")
        f.write("                  </LongitudinalAction>\n")
        f.write("                </PrivateAction>\n")
        f.write("              </Action>\n")
        f.write("              <StartTrigger>\n")
        f.write("                <ConditionGroup>\n")
        f.write("                  <Condition name=\"StartConditionVehicleReachingVelocity\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("                    <ByEntityCondition>\n")
        f.write("                      <TriggeringEntities triggeringEntitiesRule=\"any\">\n")
        f.write("                        <EntityRef entityRef=\"vehicle\"/>\n")
        f.write("                      </TriggeringEntities>\n")
        f.write("                      <EntityCondition>\n")
        f.write("                        <RelativeDistanceCondition entityRef=\"vehicle\" relativeDistanceType=\"longitudinal\" value=\"1.0\" freespace=\"true\" rule=\"lessThan\"/>\n")
        f.write("                      </EntityCondition>\n")
        f.write("                    </ByEntityCondition>\n")
        f.write("                  </Condition>\n")
        f.write("                </ConditionGroup>\n")
        f.write("              </StartTrigger>\n")
        f.write("            </Event>\n")
        f.write("            <Event name=\"VehicleDecreasesVelocity\" priority=\"overwrite\">\n")
        f.write("              <Action name=\"VehicleDecreasesVelocity\">\n")
        f.write("                <PrivateAction>\n")
        f.write("                  <LongitudinalAction>\n")
        f.write("                    <SpeedAction>\n")
        f.write("                      <SpeedActionDynamics dynamicsShape=\"step\" value=\"5\" dynamicsDimension=\"time\"/>\n")
        f.write("                      <SpeedActionTarget>\n")
        f.write("                        <AbsoluteTargetSpeed value=\"0.0\"/>\n")
        f.write("                      </SpeedActionTarget>\n")
        f.write("                    </SpeedAction>\n")
        f.write("                  </LongitudinalAction>\n")
        f.write("                </PrivateAction>\n")
        f.write("              </Action>\n")
        f.write("              <StartTrigger>\n")
        f.write("                <ConditionGroup>\n")
        f.write("                  <Condition name=\"AfterVehicleReachingVelocity\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("                    <ByValueCondition>\n")
        f.write("                      <StoryboardElementStateCondition storyboardElementType=\"action\" storyboardElementRef=\"VehicleReachingVelocity\" state=\"endTransition\"/>\n")
        f.write("                    </ByValueCondition>\n")
        f.write("                  </Condition>\n")
        f.write("                </ConditionGroup>\n")
        f.write("              </StartTrigger>\n")
        f.write("            </Event>\n")
        f.write("          </Maneuver>\n")
        f.write("        </ManeuverGroup>\n")
        f.write("        <StartTrigger>\n")
        f.write("        </StartTrigger>\n")
        f.write("      </Act>\n")
        f.write("    </Story>\n")
        f.write("    <StopTrigger>\n")
        f.write("    </StopTrigger>\n")
        f.write("  </Storyboard>\n")
        f.write("</OpenSCENARIO>\n")
        f.close()



writeTopLines(openscenario_file)
writeEntities(openscenario_file)
writeInitActions(openscenario_file, x, y, z)
# initVehicles(openscenario_file, x, y, z)
# closeInitActions(openscenario_file)
writeStory(openscenario_file)




# FileName = "D:/University/3rdYear/Simulator/CAV/CAV-Game/Unreal/CarlaUE4/Data.csv"
# with open(FileName, 'r') as f:
#     lines = f.readlines()
# print(lines[0][1])

