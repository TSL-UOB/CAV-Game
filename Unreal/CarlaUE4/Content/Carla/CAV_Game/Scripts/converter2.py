import os
import sys
import csv
from datetime import datetime
import math



data_file = "D:/University/3rdYear/Simulator/CAV/CAV-Game/Unreal/CarlaUE4/waypoints.csv"
openscenario_file = "D:/University/3rdYear/Simulator/CAV\CAV-Game/scenario_runner_0.9.13/srunner/examples/Scenario2.xosc"



array = {}
with open(data_file, 'r') as f:
    lines = csv.reader(f, delimiter=',', quotechar='|')
    rows = list(lines)
    line = 0
    for row in rows:
        if line == 0:
            line += 1
            continue
        if int(row[0]) > len(array):
            array[row[0]] = []
        array[row[0]].append([row[1], row[2], row[3], row[4], row[5]])

# for key in array:
#         yaw = math.atan2((float(array[key][1][2]) - float(array[key][0][2])), (float(array[key][1][1]) - float(array[key][0][1])))

# print(yaw)
print(rows)
print(array)



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
        for key in array:
            f.write("    <ScenarioObject name=\"vehicle" + key + "\">\n")
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



def writeInitActions(openscenario_file):
    with open(openscenario_file, 'a') as f:
        f.write("  <Storyboard>\n")
        f.write("    <Init>\n")
        f.write("      <Actions>\n")
        for key in array:
            yaw = math.atan2((float(array[key][1][2]) - float(array[key][0][2])), (float(array[key][1][1]) - float(array[key][0][1])))
            f.write("        <Private entityRef=\"vehicle" + key + "\">\n")
            f.write("          <PrivateAction>\n")
            f.write("            <TeleportAction>\n")
            f.write("              <Position>\n")
            f.write("                <WorldPosition x=\"" + str(array[key][0][1]) + "\" y=\"" + str(array[key][0][2]) + "\" z=\"" + str(array[key][0][3]) + "\" h=\"" + str(yaw) + "\"/>\n")
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
        for key in array:
            f.write("        <ManeuverGroup name=\"ManeuverGroup" + key + "\" maximumExecutionCount=\"1\">\n")
            f.write("          <Actors selectTriggeringEntities=\"false\">\n")
            f.write("            <EntityRef entityRef=\"vehicle" + key + "\"/>\n")
            f.write("          </Actors>\n")
            f.write("          <Maneuver name=\"VehicleFollowingWaypoints\">\n")
            f.write("            <Event name=\"RouteEvent\" priority=\"overwrite\">\n")
            f.write("              <Action name=\"RouteAction\">\n")
            f.write("                <PrivateAction>\n")
            f.write("                  <RoutingAction>\n")
            f.write("                    <AssignRouteAction>\n")
            f.write("                      <Route name=\"Route" + key + "\" closed=\"false\">\n")
            for waypoints in array[key]:
                f.write("                        <Waypoint routeStrategy=\"shortest\">\n")
                f.write("                          <Position>\n")
                f.write("                            <WorldPosition x=\"" + str(waypoints[1]) + "\" y=\"" + str(waypoints[2]) + "\" z=\"" + str(waypoints[3]) + "\"/>\n")
                f.write("                          </Position>\n")
                f.write("                        </Waypoint>\n")
            f.write("                      </Route>\n")
            f.write("                    </AssignRouteAction>\n")
            f.write("                  </RoutingAction>\n")
            f.write("                </PrivateAction>\n")
            f.write("              </Action>\n")
            f.write("              <StartTrigger>\n")
            f.write("                <ConditionGroup>\n")
            f.write("                  <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">\n")
            f.write("                    <ByValueCondition>\n")
            f.write("                      <SimulationTimeCondition value=\"0\" rule=\"greaterThan\"/>\n")
            f.write("                    </ByValueCondition>\n")
            f.write("                  </Condition>\n")
            f.write("                </ConditionGroup>\n")
            f.write("              </StartTrigger>\n")
            f.write("            </Event>\n")
            f.write("            <Event name=\"SpeedEvent\" priority=\"parallel\">\n")
            f.write("              <Action name=\"SpeedAction\">\n")
            f.write("                <PrivateAction>\n")
            f.write("                  <LongitudinalAction>\n")
            f.write("                    <SpeedAction>\n")
            f.write("                      <SpeedActionDynamics dynamicsShape=\"step\" value=\"0\" dynamicsDimension=\"time\"/>\n")
            f.write("                      <SpeedActionTarget>\n")
            f.write("                        <AbsoluteTargetSpeed value=\"$leadingSpeed\"/>\n")
            f.write("                      </SpeedActionTarget>\n")
            f.write("                    </SpeedAction>\n")
            f.write("                  </LongitudinalAction>\n")
            f.write("                </PrivateAction>\n")
            f.write("              </Action>\n")
            f.write("              <StartTrigger>\n")
            f.write("                <ConditionGroup>\n")
            f.write("                  <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">\n")
            f.write("                    <ByEntityCondition>\n")
            f.write("                      <TriggeringEntities triggeringEntitiesRule=\"any\">\n")
            f.write("                        <EntityRef entityRef=\"vehicle" + key + "\"/>\n")
            f.write("                      </TriggeringEntities>\n")
            f.write("                      <EntityCondition>\n")
            f.write("                        <ReachPositionCondition tolerance=\"2\">\n")
            f.write("                          <Position>\n")
            f.write("                            <WorldPosition x=\"" + str(array[key][0][1]) + "\" y=\"" + str(array[key][0][2]) + "\" z=\"" + str(array[key][0][3]) + "\"/>\n")
            f.write("                          </Position>\n")
            f.write("                        </ReachPositionCondition>\n")
            f.write("                      </EntityCondition>\n")
            f.write("                    </ByEntityCondition>\n")
            f.write("                  </Condition>\n")
            f.write("                </ConditionGroup>\n")
            f.write("              </StartTrigger>\n")
            f.write("            </Event>\n")
            f.write("          </Maneuver>\n")
            f.write("        </ManeuverGroup>\n")
        f.write("        <StartTrigger>\n")
        f.write("          <ConditionGroup>\n")
        f.write("            <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("              <ByValueCondition>\n")
        f.write("                <SimulationTimeCondition value=\"0\" rule=\"greaterThan\"/>\n")
        f.write("              </ByValueCondition>\n")
        f.write("            </Condition>\n")
        f.write("          </ConditionGroup>\n")
        f.write("        </StartTrigger>\n")
        f.write("      </Act>\n")
        f.write("    </Story>\n")
        f.write("    <StopTrigger>\n")
        f.write("      <ConditionGroup>\n")
        f.write("        <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">\n")
        f.write("          <ByValueCondition>\n")
        f.write("            <SimulationTimeCondition value=\"30\" rule=\"greaterThan\"/>\n")
        f.write("          </ByValueCondition>\n")
        f.write("        </Condition>\n")
        f.write("      </ConditionGroup>\n")
        f.write("    </StopTrigger>\n")
        f.write("  </Storyboard>\n")
        f.write("</OpenSCENARIO>\n")
        f.close()



writeTopLines(openscenario_file)
writeEntities(openscenario_file)
writeInitActions(openscenario_file)
writeStory(openscenario_file)