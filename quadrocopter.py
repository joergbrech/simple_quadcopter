import pybullet as p
from parcour import parcour


def read_sensors(body_idx, directions=[], range=2, draw_rays=True):
    # measure the distance along rays cast into the directions specified in the array dirs from body with index
    # body_idx. The ray's length can be modified with the range argument. If directions is empty, six rays are cast
    # into the x-, x+, y-, y+, z-, z+ directions from the body's center of mass in local body coordinates

    if not directions:
        directions = [[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]]

    sensor_data = []
    pos, orn = p.getBasePositionAndOrientation(body_idx)

    for direction in directions:
        local_dir = p.rotateVector(orn, direction)
        ray_to = [pos[0] + range*local_dir[0],
                  pos[1] + range*local_dir[1],
                  pos[2] + range*local_dir[2]]
        ray_info = p.rayTest(pos, ray_to)
        sensor_data.append(ray_info[0][2]*range)
        if draw_rays:
            if ray_info[0][0] > 0:
                draw_to = ray_info[0][3]
            else:
                draw_to = ray_to
            p.addUserDebugLine(pos, draw_to, [0, 1, 0], 1, 0.1)
    return sensor_data


if __name__ == '__main__':

    useGUI = True

    # initialize GUI
    if useGUI:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setGravity(0, 0, -10)

    # initialize the  quadrocopter and place into the world
    copter = p.loadURDF("data/quadrotor.urdf")
    p.resetBasePositionAndOrientation(copter, [0, 0, 2.], [0, 0, 0, 1])
    p.changeDynamics(copter, -1, linearDamping=0.9)

    # create "ground"
    p.loadURDF("data/plane.urdf")

    # create parcour
    parcour()

    # quadcopter color
    color = [0.4, 0.4, 0.4, 1]
    contactColor = [1., 0., 0., 1]

    # create some sliders to manipulate the camera view
    if useGUI:
        cameraPitchSlider = p.addUserDebugParameter("cameraPitch", -90, 90, -35)
        cameraDistanceSlider = p.addUserDebugParameter("cameraDistance", 0.5, 100, 5)
        cameraYawSlider = p.addUserDebugParameter("cameraYaw", 0, 360, 35)

    # initialize force variables for the four rotors
    baseForce = 0.
    force1Diff = 0.
    force2Diff = 0.
    force3Diff = 0.
    force4Diff = 0.

    # simulation loop
    while True:

        # read out distance sensors of quadrocopter
        sensor_data = read_sensors(copter)

        # reset camera position
        if useGUI:
            pos, orn = p.getBasePositionAndOrientation(copter)
            cameraTargetPosition = pos
            cameraPitch = p.readUserDebugParameter(cameraPitchSlider)
            cameraDistance = p.readUserDebugParameter(cameraDistanceSlider)
            cameraYaw = p.readUserDebugParameter(cameraYawSlider)
            p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        # read user input to steer quadrocopter
        keys = p.getKeyboardEvents()
        for k, v in keys.items():

            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force1Diff = 5
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED):
                force1Diff = 0.
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force2Diff = 5
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED):
                force2Diff = 0.

            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force3Diff = 5
            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED):
                force3Diff = 0.
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force4Diff = 5
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED):
                force4Diff = 0.

            if k == ord('n') and (v & p.KEY_IS_DOWN):
                baseForce = max(0, baseForce - 0.1)
                print("BaseForce = {}".format(baseForce))
            if k == ord('m') and (v & p.KEY_IS_DOWN):
                baseForce = min(10, baseForce + 0.1)
                print("BaseForce = {}".format(baseForce))

        # apply force to the four rotors
        force1 = [0., 0., force1Diff + baseForce]
        force2 = [0., 0., force2Diff + baseForce]
        force3 = [0., 0., force3Diff + baseForce]
        force4 = [0., 0., force4Diff + baseForce]

        p.applyExternalForce(copter, -1, force1, [0, 0.25, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force2, [0, -0.25, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force3, [0.25, 0, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force4, [-0.25, 0, 0], flags=p.LINK_FRAME)

        # change color of copter in case of collision
        if p.getContactPoints(copter):
            p.changeVisualShape(copter, -1, rgbaColor=contactColor)
        else:
            p.changeVisualShape(copter, -1, rgbaColor=color)

        # advance the simulation
        p.stepSimulation()
