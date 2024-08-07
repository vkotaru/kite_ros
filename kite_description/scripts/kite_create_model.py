from sdf_generator import *
import numpy as np
import warnings
import argparse
import os

colors = ['white', 'red', 'blue', 'green', 'brown', 'yellow']


class KiteModelCreator(object):
    def __init__(self, args):
        self.name = "KITE_TMP_MDL"
        self.folder = "../models/"

        self.name = args.name

        self.sdf_ = SDFGenerator(name=self.name, folder=self.folder)
        self.sdf_.generateConfigFile()
        self.sdf_.open()

        if args.model == 'quadrotor':
            self.create_quadrotor(name=self.name)
        elif args.model == 'quadrotor_cs_payload':
            self.create_quadrotor_cs_payload(name=self.name)
        else:
            raise Exception("Model not implemented")
        return

    def __del__(self):
        self.sdf_.close()

    def addDrone(self,
                 mass,
                 inertia,
                 name="falcon",
                 pose=np.zeros(6),
                 color='red'):
        xml_str = self.sdf_.addInertial(pose=np.zeros(6),
                                        mass=mass,
                                        inertia=inertia)
        xml_str += self.sdf_.addCollision(pose=np.zeros(6),
                                          size=np.array([0.2505, 0.2505,
                                                         0.09]))
        xml_str += self.sdf_.addVisual(uri="model://{}/meshes/dega.dae".format(name),
                                       pose=np.array(
                                           [0, 0, 0, 0, 0, -1.570796]),
                                       scale=np.array([0.001, 0.001, 0.001]))
        xml_str = self.sdf_.addTag(xml_str, "link", name="base_link")

        grnd_truth_plugin_str = self.sdf_.addGroundTruthPlugin(
            plugin_name=name + "_groundtruth_sim",
            link_name="base_link",
            topic_name="/" + name + "/odometry/groundtruth")
        xml_str += "\n" + grnd_truth_plugin_str

        kite_plugin_str = self.sdf_.addKiteQuadrotorPlugin(
            plugin_name=name + "_kite_quadrotor_plugin",
            link_name="base_link",
            namespace=name)
        xml_str += "\n" + kite_plugin_str

        self.sdf_.addLine(xml_str)

    def addFlexibleCable(self, name, color="blue", N=10):
        if color not in colors:
            warnings.warn(name + " not in the acceptable colors.")

        # cable parameters
        l, r = 1, 0.00125
        mass = 0.01
        body_length = l / float(N)
        half_body_length = body_length / 2.0
        body_mass = mass / float(N)
        k, c = 0, 0.0001
        self.sdf_.addLine(
            self.sdf_.addCylinder(link_name="link_0",
                                  pose=np.array(
                                      [half_body_length, 0., 0., 0., 0., 0.]),
                                  l=body_length,
                                  r=r,
                                  mass=body_mass))
        for i in range(N - 1):
            pose_ = np.zeros(6)
            pose_[0] = half_body_length + (i + 1) * body_length
            self.sdf_.addLine(
                self.sdf_.addCylinder(link_name="link_" + str(i + 1),
                                      pose=pose_,
                                      l=body_length,
                                      r=r,
                                      mass=body_mass))

            joint_name = "joint_" + str(i)
            link1_name = "link_" + str(i)
            link2_name = "link_" + str(i + 1)
            rel_pose_ = np.array([-half_body_length, 0., 0., 0., 0., 0.])
            self.sdf_.addLine(
                self.sdf_.addUniversalJoint(joint_name,
                                            link1_name,
                                            link2_name,
                                            rel_pose_,
                                            k=k,
                                            c=c))

    def addPointMassPayload(self,
                            name="blue",
                            pose=np.array([1.5, 0., 0., 0., 0., 0.]),
                            mass=0.15):
        if name not in colors:
            warnings.warn(name + "not in the acceptable drones")

        # payload parameters
        payload_radius, payload_height = 15.e-3, 22.0e-3
        payload_mass = mass
        self.sdf_.addline(
            self.sdf_.addCylinder(link_name="payload",
                                  pose=pose,
                                  l=payload_height,
                                  r=payload_radius,
                                  mass=payload_mass))

        # self.sdf_.addline(self.sdf_.addUniversalJoint(name="payload_cable_joint", link1="payload", link2="link_" + str(N-1),
        #                                         pose=np.array([half_body_length, 0., 0., 0., 0., 0.]), c=0.01))

        # self.sdf_.addline(self.sdf_.addBallJoint(name="quadrotor_cable_joint", plink=drone_name+"::base_link", clink="link_0",
        #                                   pose=np.array([-half_body_length, 0., 0., 0., 0., 0.]), c=0.01))

        # self.sdf_.addline(self.sdf_.addGroundTruthPlugin(
        #     linkName="payload", topicName=drone_name+"/payload/odometry/mocap"))

    def create_quadrotor(self, name="falcon", color='Red'):
        print("Creating a quadrotor  model")
        mass = 0.9  # kg
        inertia = np.array(
            [0.0049, 0.0053, 0.0098, 0.0000055, 0.0000054, 0.000021])  # kg m^2
        self.addDrone(mass, inertia, name=name, color=color)
        
        dest_folder = "../models/" + name + "/meshes/"
        if not os.path.exists(dest_folder):
            os.makedirs(dest_folder)
        os.system("cp ../meshes/dega.dae ../models/" + name +"/meshes/")
        

    def create_quadrotor_cs_payload(self, name="falcon_cs_payload", N=10, color='Red'):
        print("Creating a quadrotor with cable-suspended payload model")
        mass = 0.9  # kg
        inertia = np.array(
            [0.0049, 0.0053, 0.0098, 0.0000055, 0.0000054, 0.000021])  # kg m^2
        self.addDrone(mass, inertia, name=name, color=color)
        self.addFlexibleCable(name=name, N=N)
        # self.addPointMassPayload(name=name)
        
        dest_folder = "../models/" + name + "/meshes/"
        if not os.path.exists(dest_folder):
            os.makedirs(dest_folder)
        os.system("cp ../meshes/dega.dae ../models/" + name +"/meshes/")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='KITE Model Creator')
    parser.add_argument('--name',
                        type=str,
                        default="KITE_TMP_MDL",
                        help='Name of the model')
    parser.add_argument('--model',
                        '-m',
                        required=True,
                        type=str,
                        choices=['quadrotor', 'quadrotor_cs_payload'],
                        help='model')
    args = parser.parse_args()

    kite = KiteModelCreator(args)
    pass
