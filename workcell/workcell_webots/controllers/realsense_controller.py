from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Geräte holen und aktivieren
# Namen müssen exakt mit den internen Namen im PROTO übereinstimmen!
# Ihr JS-Code generiert Namen wie "IntelRealsenseD415_rgb"
cam = robot.getDevice("IntelRealSenseD415_rgb")
dist = robot.getDevice("IntelRealSenseD415_depth")

cam.enable(timestep)
dist.enable(timestep)

while robot.step(timestep) != -1:
    pass