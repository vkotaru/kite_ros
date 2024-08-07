#!/usr/bin/env python

import os
import json
import math
import numpy as np


class SDFGenerator(object):
    def __init__(self, name, folder=''):
        self.name = name
        self.file_ = None
        self.models_sdf_filename = ''

        self.dir_path = os.path.normpath(
            os.path.join(os.getcwd(), folder, name))
        dirExists = os.path.exists(self.dir_path)
        if not dirExists:
            os.makedirs(self.dir_path)

    def open(self):
        self.models_sdf_filename = os.path.join(self.dir_path, "model.sdf")
        if not os.path.exists(self.models_sdf_filename):
            open(self.models_sdf_filename, "w").close()
        self.file_ = open(self.models_sdf_filename, 'w')
        self.file_.truncate(0)
        self.file_.write("<?xml version=\"1.0\" ?>\n"
                         "<sdf version=\"1.5\">\n"
                         "<model name=\"" + self.name + "\">\n"
                         "<static> 0 </static>\n")

    def generateConfigFile(self):
        models_sdf_filename = os.path.join(self.dir_path, "model.config")
        if not os.path.exists(models_sdf_filename):
            open(models_sdf_filename, "w").close()
        config_file_ = open(models_sdf_filename, 'w')
        config_file_.truncate(0)
        s = """<?xml version="1.0" ?>
    <model>
    <name>{}</name>
    <version>1.0</version>
    <sdf version="1.7">model.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
            """.format(self.name)
        config_file_.write(s)
        config_file_.close()

    def addLine(self, s):
        print(s)
        self.file_.write(s + "\n")

    def close(self):
        self.addLine("</model>")
        self.addLine("</sdf>")
        self.file_.close()

    def addTag(self, s, tagname, name=None):
        if name is not None:
            return "\n<" + tagname + " name=\"" + name + "\">\n\t" + s + "\n</" + tagname + ">\n"
        return "\n<" + tagname + ">\n" + "\t{}\n".format(
            s) + "\n</" + tagname + ">"

    def addInlineTag(self, s, tagname, name=None):
        if name is not None:
            return "\n<" + tagname + " name=\"" + name + "\">" + s + "</" + tagname + ">"
        return "\n<" + tagname + ">" + "{}".format(s) + "</" + tagname + ">"

    def addInertial(self, pose, mass, inertia=None):
        s = ''
        s += self.addPoseTag(pose)
        s += self.addInlineTag(mass, "mass")
        inertia_str = ""
        if inertia is not None:
            if inertia.ndim == 2:
                inertia_str += self.addInlineTag(inertia[0, 0], "ixx")
                inertia_str += self.addInlineTag(inertia[0, 1], "ixy")
                inertia_str += self.addInlineTag(inertia[0, 2], "ixz")
                inertia_str += self.addInlineTag(inertia[1, 1], "iyy")
                inertia_str += self.addInlineTag(inertia[1, 2], "iyz")
                inertia_str += self.addInlineTag(inertia[2, 2], "izz")
            elif inertia.ndim == 1:
                inertia_str += self.addInlineTag(inertia[0], "ixx")
                inertia_str += self.addInlineTag(inertia[1], "iyy")
                inertia_str += self.addInlineTag(inertia[2], "izz")
                if inertia.shape[0] == 6:
                    inertia_str += self.addInlineTag(inertia[3], "ixy")
                    inertia_str += self.addInlineTag(inertia[4], "ixz")
                    inertia_str += self.addInlineTag(inertia[5], "iyz")
                else:
                    inertia_str += self.addInlineTag(0., "ixy")
                    inertia_str += self.addInlineTag(0., "ixz")
                    inertia_str += self.addInlineTag(0., "iyz")
            else:
                raise ValueError("Inertia must be 1D or 2D array")
            s += self.addTag(inertia_str, "inertia")
        s = self.addTag(s, "inertial")
        return s

    def addCollision(self, size, pose=np.zeros(6)):
        pose_str = self.addPoseTag(pose)
        geom_str = self.addInlineTag("{} {} {}".format(*size), "size")
        geom_str = self.addTag(geom_str, "box")
        geom_str = self.addTag(geom_str, "geometry")
        s = self.addTag(pose_str + geom_str, "collision", name="collision")
        return s

    def addVisual(self, uri, scale, color="Red", pose=np.zeros(6)):
        pose_str = self.addPoseTag(pose)
        geom_str = self.addInlineTag(uri, "uri")
        geom_str += self.addInlineTag("{} {} {}".format(*scale), "scale")
        geom_str = self.addTag(geom_str, "mesh")
        geom_str = self.addTag(geom_str, "geometry")
        material_str = self.addInlineTag(
            "file://media/materials/scripts/gazebo.material", "uri")
        material_str += self.addInlineTag("Gazebo/" + color, "name")
        material_str = self.addTag(material_str, "script")
        material_str = self.addTag(material_str, "material")
        s = self.addTag(pose_str + geom_str + material_str,
                        "visual",
                        name="visual")
        return s

    def addPoseTag(self, pose):
        s = ''
        s += "<pose>"
        for i in range(6):
            s += str(pose[i]) + "\t"
        s += "</pose>\n"
        return s

    def includeObject(self, object, pose, name):
        s = "<include>\n"
        s += "<uri>model://" + object + "</uri>\n"
        s += self.addPoseTag(pose)
        s += "<name>" + name + "</name>\n"
        s += "</include>\n"
        return s

    def includeDrone(self, name, pose, updateRate=500.0):
        # s = "<include>\n"
        # s += "<uri>model://drone</uri>\n"
        # s += self.addPoseTag(pose)
        # s += "<name>"+name+"</name>\n"
        # s += "</include>\n"
        s = self.includeObject("drone", pose, name)
        s += "<plugin filename=\"libqrotor_gazebo_plugin.so\" name=\"qrotor_plugin_" + name + "\">\n"
        s += "<linkName>" + name + "::base_link</linkName>\n"
        s += "<namespace>" + name + "</namespace>\n"
        s += "<updateRate>" + str(updateRate) + "</updateRate>\n</plugin>\n"
        return s

    def addGroundTruthPlugin(self,
                             plugin_name,
                             link_name,
                             topic_name,
                             update_rate=200.0):
        s = """<!-- Groundtruth plugin -->
<plugin name="{}" filename="libgazebo_ros_p3d.so">
  <alwaysOn>true</alwaysOn>
  <updateRate>{}</updateRate>
  <bodyName>{}</bodyName>
  <topicName>{}</topicName>
  <gaussianNoise>0.0</gaussianNoise>
  <frameName>world</frameName>
  <xyzOffsets>0 0 0</xyzOffsets>
  <rpyOffsets>0 0 0</rpyOffsets>
</plugin>
        """.format(plugin_name, update_rate, link_name, topic_name)
        return s

    def addKiteQuadrotorPlugin(self,
                         plugin_name,
                         publish_rate=200.0,
                         update_rate=500.0,
                         namespace="kite",
                         link_name="base_link",
                         position_rate=100.0,
                         attitude_rate=500.0):
        str = """<!-- Kite Quadrotor plugin-->
<plugin filename="libkite_quadrotor_plugin.so" name="{}">
  <publishRate>{}</publishRate>
  <positionControlRate>{}</positionControlRate>
  <attitudeControlRate>{}</attitudeControlRate>
  <linkName>{}</linkName>
  <namespace>{}</namespace>
</plugin>
        """.format(plugin_name, publish_rate, position_rate, attitude_rate,
                   link_name, namespace)
        return str

    def addCylinder(self,
                    link_name,
                    pose,
                    l=1,
                    r=0.1,
                    mass=0.005,
                    ixx=0.0001,
                    iyy=0.0001,
                    izz=0.0001,
                    ixy=0.,
                    ixz=0.,
                    iyz=0.,
                    material='Yellow',
                    roll=0.,
                    pitch=1.57079):
        s = "<link name = \"" + link_name + "\" >\n"
        s += self.addPoseTag(pose)
        s2 = """
      <inertial>
        <pose>0 0 0 0 -0 0</pose>
        <mass>body_mass</mass>
        <inertia>
          <ixx>body_ixx</ixx>
          <ixy>body_ixy</ixy>
          <ixz>body_ixz</ixz>
          <iyy>body_iyy</iyy>
          <iyz>body_iyz</iyz>
          <izz>body_izz</izz>
        </inertia>
      </inertial>
      <visual name='linkName_vis'>
        <pose>0 0 0 roll pitch 0</pose>
        <geometry>
          <cylinder>
            <length>body_length</length>
            <radius>body_radius</radius>
          </cylinder>
        </geometry>
        <material>
          <shader type='pixel'/>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/MaterialType</name>
          </script>
        </material>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
      <collision name='linkName_collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose>0  0 0 roll pitch 0</pose>
        <geometry>
          <cylinder>
            <length>body_length</length>
            <radius>body_radius</radius>
          </cylinder>
        </geometry>
      </collision>
    </link>"""
        s2 = s2.replace("linkName", link_name)
        s2 = s2.replace("body_mass", str(mass))
        s2 = s2.replace("body_ixx", str(ixx))
        s2 = s2.replace("body_iyy", str(iyy))
        s2 = s2.replace("body_izz", str(izz))
        s2 = s2.replace("body_ixy", str(ixy))
        s2 = s2.replace("body_ixz", str(ixz))
        s2 = s2.replace("body_iyz", str(iyz))
        s2 = s2.replace("body_length", str(l))
        s2 = s2.replace("body_radius", str(r))
        s2 = s2.replace("MaterialType", material)
        s2 = s2.replace("roll", str(roll))
        s2 = s2.replace("pitch", str(pitch))
        s += s2
        return s

    def addUniversalJoint(self, name, link1, link2, pose, k=0., c=0.):
        s = "<joint name='" + name + "' type='universal'>\n"
        s += "<parent>" + link1 + "</parent>\n"
        s += "<child>" + link2 + "</child>\n"
        s += self.addPoseTag(pose)
        #   s2 = """
        # <parent>link_<%=i-1%></parent>
        # <child>link_<%=i%></child>
        # <pose><%= -half_body_length %> 0 0 0 -0 0</pose>

        s2 = """<axis>
        <xyz>0 1 0</xyz>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>springStiffness</spring_stiffness>
          <damping>dampingCoefficient</damping>
          <friction>0</friction>
        </dynamics>
      </axis>
      <axis2>
        <xyz>0 0 1</xyz>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>springStiffness</spring_stiffness>
          <damping>dampingCoefficient</damping>
          <friction>0</friction>
        </dynamics>
      </axis2>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </limit>
          <suspension>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </suspension>
        </ode>
      </physics>
    </joint>"""
        s2 = s2.replace("springStiffness", str(k))
        s2 = s2.replace("dampingCoefficient", str(c))
        s += s2
        return s

    def addBallJoint(self, name, plink, clink, pose, k=0., c=0.):
        s = "<joint name='" + name + "' type='ball'>\n"
        s += "<parent>" + plink + "</parent>\n"
        s += "<child>" + clink + "</child>\n"
        s += self.addPoseTag(pose)
        s2 = """<axis>
  <xyz>0 0 1</xyz>
  <limit>
    <lower>-1e8</lower>
    <upper>1e8</upper>
    <effort>-1</effort>
    <velocity>-1</velocity>
    <dissipation>1</dissipation>
  </limit>
  <dynamics>
    <damping>dampingCoefficient</damping>
  </dynamics>
  <use_parent_model_frame>1</use_parent_model_frame>
</axis>
<physics>
  <ode>
    <limit>
      <cfm>0</cfm>
      <erp>0.2</erp>
    </limit>
    <suspension>
      <cfm>0</cfm>
      <erp>0.2</erp>
    </suspension>
  </ode>
</physics>
</joint>"""
        s2 = s2.replace("dampingCoefficient", str(c))
        s += s2
        return s

    def addTriangularPlank(self,
                           link_name,
                           pose=np.zeros(6),
                           side=1,
                           h=0.05,
                           mass=1.0,
                           ixx=0.0001,
                           iyy=0.0001,
                           izz=0.0001,
                           ixy=0.,
                           ixz=0.,
                           iyz=0.,
                           material='Yellow',
                           roll=0.,
                           pitch=1.57079):

        izz = mass * side * side / 2
        ixx = 0.5 * izz
        iyy = 0.5 * izz
        print(ixx, iyy, izz)

        shape = """<point>{} 0.0</point>
              <point>{} 0.0</point>
              <point>0.0 {}</point>
              <point>{} 0.0</point>
              <height>{}</height>""".format(-side / 2, side / 2, 0.866 * side,
                                            -side / 2, h)

        com = side / (2 * np.sqrt(3))

        s = """
      <link name="{}">
        {}
        <inertial>
          <pose>0 {} {} 0 0 0</pose>
          <mass>{}</mass>
          <inertia>
            <ixx>{}</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>{}</iyy>
            <iyz>0</iyz>
            <izz>{}</izz>
          </inertia>
        </inertial>

        <collision name="collision">
          <geometry>
            <polyline>
            {}
            </polyline>
          </geometry>
        </collision>

        <visual name="triangle">
          <geometry>
            <polyline>
              {}
            </polyline>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/{}</name>
            </script>
          </material>
        </visual>  
      </link>""".format(link_name, self.addPoseTag(pose), com, h / 2, mass,
                        ixx, iyy, izz, shape, shape, material)
        return s

    def addBox(self,
               link_name,
               pose=np.zeros(6),
               l=1.,
               w=0.5,
               h=0.05,
               mass=1.0,
               ixx=0.0001,
               iyy=0.0001,
               izz=0.0001,
               ixy=0.,
               ixz=0.,
               iyz=0.,
               material='Yellow',
               roll=0.,
               pitch=1.57079):

        s = """
      <link name="{}">
        {}
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>{}</mass>
          <inertia>
            <ixx>{}</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>{}</iyy>
            <iyz>0</iyz>
            <izz>{}</izz>
          </inertia>
        </inertial>

        <collision name="collision">
          <geometry>
            <box>
            <size>{} {} {}</size>
            </box>
          </geometry>
        </collision>

        <visual name="triangle">
          <geometry> 
          <box>
            <size>{} {} {}</size>
          </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/{}</name>
            </script>
          </material>
        </visual>  
      </link>""".format(link_name, self.addPoseTag(pose), mass, ixx, iyy, izz,
                        l, w, h, l, w, h, material)
        return s

    def addFixedJoint(self, joint_name, parent_link, child_link):
        s = """<joint name="{}" type="fixed">
        <parent>{}</parent>
        <child>{}</child>
      </joint>""".format(joint_name, parent_link, child_link)
        return s
