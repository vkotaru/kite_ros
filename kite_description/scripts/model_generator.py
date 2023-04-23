from sdf_generator import *
import numpy as np
import warnings

colors = ['white', 'red', 'blue', 'green', 'brown', 'yellow']


def multiple_drones(name='multiple_drones', N=4):
    sdfgen = SDFGenerator(name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()
    for i in range(N):
        xy_ = 2*np.random.randn(2)
        pose_ = np.array([xy_[0], xy_[1], 0., 0., 0., 0.])
        sdfgen.addline(sdfgen.includeDrone(
            "falcon"+str(i), pose_, updateRate=500.0))
    sdfgen.close()


def flexible_cable(name='flexible_cable', l=1, r=0.00125, mass=0.1, N=10):
    body_length = l/float(N)
    half_body_length = body_length/2.0
    body_mass = mass/float(N)
    k = 0
    c = 0.01
    sdfgen = SDFGenerator(name=name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()

    #  link_name, pose, l=1, r=0.1, mass=0.005, ixx=0.0001,
    #  iyy=0.0001, izz=0.0001, ixy=0., ixz=0., iyz=0., material='Yellow'
    sdfgen.addline(sdfgen.addCylinder(
        link_name="link_0", pose=np.zeros(6), l=body_length, r=r, mass=body_mass))
    for i in range(N-1):
        pose_ = np.zeros(6)
        pose_[0] = (i+1)*body_length
        sdfgen.addline(sdfgen.addCylinder(
            link_name="link_"+str(i+1), pose=pose_, l=body_length, r=r, mass=body_mass))

        joint_name = "joint_"+str(i)
        link1_name = "link_"+str(i)
        link2_name = "link_"+str(i+1)
        rel_pose_ = np.array([-half_body_length, 0., 0., 0., 0., 0.])
        sdfgen.addline(sdfgen.addUniversalJoint(
            joint_name, link1_name, link2_name, rel_pose_, k=k, c=c))
    sdfgen.close()


def payload(name='payload', l=1, r=0.5, mass=3):
    volume = np.pi*r*r*l
    rho = mass/volume
    Ixx = mass*l*l/12.0 + mass*r*r/4.0
    Iyy = mass*l*l/12.0 + mass*r*r/4.0
    Izz = mass*r*r/2.0

    print(Ixx, Iyy, Izz)

    sdfgen = SDFGenerator(name=name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()
    sdfgen.addline(sdfgen.addCylinder(
        link_name="base_link", pose=np.array([0., 0., l/2., 0., 0., 0.]),
        l=l, r=r, mass=mass, ixx=Ixx, iyy=Iyy, izz=Izz, roll=0, pitch=0.))
    sp = """ 
    <plugin name="placeholder_odom_plugin" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>placeholder/odometry</topicName>
      <gaussianNoise>0.0</gaussianNoise>
      <frameName>world</frameName>
      <xyzOffsets>0 0 0</xyzOffsets>
      <rpyOffsets>0 0 0</rpyOffsets>
    </plugin>
    """
    sp = sp.replace("placeholder", name)
    sdfgen.addline(sp)
    sdfgen.close()


def payload_flexible_cable(nQ=4, name="mq_rb_fc"):
    # rigid payload
    payload_mass = 1.0
    payload_height = 0.1
    payload_radius = 0.5
    payload_name = "rigid_payload"
    payload(payload_name, l=payload_height,
            r=payload_radius, mass=payload_mass)

    # flexible cable
    cable_length = 1
    cable_radius = 0.00125
    cable_num_links = 20
    cable_mass = 0.01  # 100grams
    segment_length = cable_length/cable_num_links
    flexible_cable(name='flexible_cable', l=cable_length,
                   r=cable_radius, mass=cable_mass, N=cable_num_links)

    sdfgen = SDFGenerator(name=name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()
    sdfgen.addline(sdfgen.includeObject(
        payload_name, np.zeros(6), payload_name))

    th = 2*np.pi/nQ
    r = payload_radius + 0.5*segment_length
    r2 = payload_radius + cable_length
    z = payload_height

    colors = ['white', 'red', 'blue', 'green', 'brown', 'yellow']
    for i in range(nQ):
        cpose_ = np.array([r*np.cos(th*i), r*np.sin(th*i), z, 0., 0., i*th])
        sdfgen.addline(sdfgen.includeObject(
            "flexible_cable", cpose_, "flexible_cable_"+colors[i]))
        sdfgen.addline(sdfgen.addBallJoint("payload_cable_joint_"+colors[i],
                                           payload_name+"::base_link", "flexible_cable_" +
                                           colors[i]+"::link_0",
                                           np.array([-0.5*segment_length, 0., 0., 0., 0., 0.]), c=0.01))
        dpose_ = np.array([r2*np.cos(th*i), r2*np.sin(th*i), z, 0., 0., 0.])
        drone_name = colors[i]+"_falcon"
        sdfgen.addline("<include>")
        sdfgen.addline("<uri>model://"+drone_name+"</uri>")
        sdfgen.addline(sdfgen.addPoseTag(dpose_))
        sdfgen.addline("</include>")
        sdfgen.addline(sdfgen.addBallJoint("drone_cable_joint_"+colors[i],
                                           drone_name+"::base_link", "flexible_cable_" +
                                           colors[i]+"::link_" +
                                           str(cable_num_links-1),
                                           np.array([0.5*segment_length, 0., 0., 0., 0., 0.]), c=0.01))

    sdfgen.close()


def quad_flexible_cable_payload(name='blue', N=10):
    if name not in colors:
        warnings.warn(name + "not in the acceptable drones")

    model_name = name+'_falcon_flexible_cable_payload'
    sdfgen = SDFGenerator(name=model_name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()

    # add drone
    # ---------
    cable_offset_x = 0.0
    cable_offset_y = 0.0
    cable_offset_z = 0.035

    drone_name = name+"_falcon"
    sdfgen.addline("<include>")
    sdfgen.addline("<uri>model://"+drone_name+"</uri>")
    sdfgen.addline(sdfgen.addPoseTag(
        np.array([cable_offset_x, cable_offset_y, cable_offset_z, 0., 0., 0.])))
    sdfgen.addline("</include>")

    # adding cable
    # ------------
    # cable parameters
    l, r = 1, 0.00125
    mass = 0.01  # 10 grams

    body_length = l/float(N)
    half_body_length = body_length/2.0
    body_mass = mass/float(N)
    k, c = 0, 0.0001
    sdfgen.addline(sdfgen.addCylinder(
        link_name="link_0", pose=np.array([half_body_length, 0., 0., 0., 0., 0.]), l=body_length, r=r, mass=body_mass))
    for i in range(N-1):
        pose_ = np.zeros(6)
        pose_[0] = half_body_length+(i+1)*body_length
        sdfgen.addline(sdfgen.addCylinder(
            link_name="link_"+str(i+1), pose=pose_, l=body_length, r=r, mass=body_mass))

        joint_name = "joint_"+str(i)
        link1_name = "link_"+str(i)
        link2_name = "link_"+str(i+1)
        rel_pose_ = np.array([-half_body_length, 0., 0., 0., 0., 0.])
        sdfgen.addline(sdfgen.addUniversalJoint(
            joint_name, link1_name, link2_name, rel_pose_, k=k, c=c))

    # adding payload
    # --------------
    # payload parameters
    payload_radius, payload_height = 15.e-3, 22.0e-3
    payload_mass = 0.15  # 150 grams
    payload_pose = np.array([l+0.5*payload_height, 0., 0., 0., 0., 0])
    sdfgen.addline(sdfgen.addCylinder(
        link_name="payload", pose=payload_pose, l=payload_height, r=payload_radius, mass=payload_mass))

    sdfgen.addline(sdfgen.addUniversalJoint(name="payload_cable_joint", link1="payload", link2="link_" + str(N-1),
                                            pose=np.array([half_body_length, 0., 0., 0., 0., 0.]), c=0.01))

    sdfgen.addline(sdfgen.addBallJoint(name="quadrotor_cable_joint", plink=drone_name+"::base_link", clink="link_0",
                                       pose=np.array([-half_body_length, 0., 0., 0., 0., 0.]), c=0.01))

    sdfgen.addline(sdfgen.addGroundTruthPlugin(
        linkName="payload", topicName=drone_name+"/payload/odometry/mocap"))

    sdfgen.close()
    return


def triangle_payload_with_metalplates(name="triangle"):
    model_name = name+'_payload'
    sdfgen = SDFGenerator(name=model_name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()
    sdfgen.addline(sdfgen.addTriangularPlank(
        link_name="base_link", ixx=0.1666, iyy=0.1666, izz=0.1666))
    sdfgen.addline(sdfgen.addTriangularPlank(link_name="link1", pose=np.array(
        [-0.5+0.05, 0., 0.05, 0., 0., 0.]), side=0.1, material="Red", h=0.001, mass=0.1))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint1",
                   parent_link="base_link", child_link="link1"))
    sdfgen.addline(sdfgen.addTriangularPlank(link_name="link2", pose=np.array(
        [0.5-0.05, 0., 0.05, 0., 0., 0.]), side=0.1, material="Red", h=0.001, mass=0.1))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint2",
                   parent_link="base_link", child_link="link2"))
    sdfgen.addline(sdfgen.addTriangularPlank(link_name="link3", pose=np.array(
        [0., 0.866-0.0866, 0.05, 0., 0., 0.]), side=0.1, material="Red", h=0.001, mass=0.1))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint3",
                   parent_link="base_link", child_link="link3"))
    sdfgen.close()
    return


def plank_payload_with_metalplates(name="plank"):
    model_name = name+'_payload'
    sdfgen = SDFGenerator(name=model_name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()

    l = 1.83
    h = 0.02
    w = 0.02
    sdfgen.addline(sdfgen.addBox(link_name="base_link", pose=np.array(
        [0., 0., 0., 0., 0., 0.]), l=w, w=l, h=h, mass=0.516, ixx=0.1666, iyy=0.1666, izz=0.1666))
    dl = 0.1
    dh = 0.001
    sdfgen.addline(sdfgen.addBox(link_name="link1", pose=np.array(
        [0., -0.5*(l-dl), 0.5*(h+dh), 0., 0., 0.]), l=dl, w=dl, material="Red", h=dh, mass=0.01))
    sdfgen.addline(sdfgen.addBox(link_name="link2", pose=np.array(
        [0., 0.5*(l-dl), 0.5*(h+dh), 0., 0.,  0.]), l=dl, w=dl, material="Red", h=dh, mass=0.01))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint1",
                   parent_link="base_link", child_link="link1"))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint2",
                   parent_link="base_link", child_link="link2"))

    sdfgen.addline(sdfgen.addGroundTruthPlugin(
        linkName="base_link", topicName=name+"_payload/odometry/mocap"))
    sdfgen.close()
    return


def square_plank_with_metal_plates(name="square"):
    model_name = name+'_payload'
    sdfgen = SDFGenerator(name=model_name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()

    l = 1.22
    h = 0.02
    w = 1.22
    sdfgen.addline(sdfgen.addBox(link_name="base_link", l=l, w=w,
                   h=h, ixx=0.1666, iyy=0.1666, izz=0.1666, mass=1))
    dl = 0.1
    dh = 0.001
    sdfgen.addline(sdfgen.addBox(link_name="link1", pose=np.array(
        [0.5*(l-dl), 0.5*(w-dl), 0.5*(h+dh), 0., 0., 0.]), l=dl, w=dl, material="Red", h=dh, mass=0.01))
    sdfgen.addline(sdfgen.addBox(link_name="link2", pose=np.array(
        [-0.5*(l-dl), 0.5*(w-dl), 0.5*(h+dh), 0., 0., 0.]), l=dl, w=dl, material="Red", h=dh, mass=0.01))
    sdfgen.addline(sdfgen.addBox(link_name="link3", pose=np.array(
        [-0.5*(l-dl), -0.5*(w-dl), 0.5*(h+dh), 0., 0., 0.]), l=dl, w=dl, material="Red", h=dh, mass=0.01))
    sdfgen.addline(sdfgen.addBox(link_name="link4", pose=np.array(
        [0.5*(l-dl), -0.5*(w-dl), 0.5*(h+dh), 0., 0., 0.]), l=dl, w=dl, material="Red", h=dh, mass=0.01))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint1",
                   parent_link="base_link", child_link="link1"))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint2",
                   parent_link="base_link", child_link="link2"))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint3",
                   parent_link="base_link", child_link="link3"))
    sdfgen.addline(sdfgen.addFixedJoint(joint_name="joint4",
                   parent_link="base_link", child_link="link4"))

    sdfgen.addline(sdfgen.addGroundTruthPlugin(
        linkName="base_link", topicName=name+"_payload/odometry/mocap"))
    sdfgen.close()
    return


def triangle_payload_with_quads():
    model_name = 'triangle_payload_with_quads'
    sdfgen = SDFGenerator(name=model_name, folder='../models/')
    sdfgen.generateConfigFile()
    sdfgen.open()

    sdfgen.addline(sdfgen.includeObject("triangle_payload", pose=np.array(
        [0., 0., 0., 0., 0., 0.]), name="triangle_payload"))

    # adding cable
    # ------------
    N = 10
    # cable parameters
    l, r = 1, 0.00125
    mass = 0.01  # 10 grams

    body_length = l/float(N)
    half_body_length = body_length/2.0
    body_mass = mass/float(N)
    k, c = 0, 0.0001

    theta = 2*np.pi/3

    for iter in range(3):
        sdfgen.addline(sdfgen.addCylinder(
            link_name="corner_"+str(iter)+"_link_0", pose=np.array([half_body_length, 0., 0., 0., 0., iter*theta]), l=body_length, r=r, mass=body_mass))
        for i in range(N-1):
            pose_ = np.array([0, 0., 0., 0., 0., 0])
            pose_[0] = half_body_length+(i+1)*body_length
            sdfgen.addline(sdfgen.addCylinder(
                link_name="corner_"+str(iter)+"_link_"+str(i+1), pose=pose_, l=body_length, r=r, mass=body_mass))

            joint_name = "corner_"+str(iter)+"_joint_"+str(i)
            link1_name = "corner_"+str(iter)+"_link_"+str(i)
            link2_name = "corner_"+str(iter)+"_link_"+str(i+1)
            rel_pose_ = np.array([-half_body_length, 0., 0., 0., 0., 0.])
            sdfgen.addline(sdfgen.addUniversalJoint(
                joint_name, link1_name, link2_name, rel_pose_, k=k, c=c))

    sdfgen.close()
    return


if __name__ == '__main__':
    pass
