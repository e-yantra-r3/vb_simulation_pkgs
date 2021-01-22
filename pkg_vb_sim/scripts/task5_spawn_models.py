#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
# from gazebo_msgs.srv import ApplyBodyWrench, GetModelProperties, GetWorldProperties, SetModelState
from copy import deepcopy
from tf.transformations import quaternion_from_euler
import rospkg


def create_cube_request(sdf_model, modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    cube = deepcopy(sdf_model)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


def paint_colour(arg_red_sdf, arg_yellow_sdf, arg_green_sdf, arg_config_colour):
  if (arg_config_colour == 'red'):
    return arg_red_sdf
  
  elif (arg_config_colour == 'yellow'):
    return arg_yellow_sdf

  elif (arg_config_colour == 'green'):
    return arg_green_sdf
  
  else:
    return 'na'


if __name__ == '__main__':
    rospy.init_node('spawn_models')

    rp = rospkg.RosPack()
    str_pkg_path = rp.get_path('pkg_vb_sim')
    rospy.loginfo( "Package Path: {}".format(str_pkg_path) )


    sdf_cube = """<?xml version="1.0" ?>
    <sdf version="1.4">
      <model name="MODELNAME">
        <static>0</static>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <ixy>0.0</ixy>
              <ixz>0.0</ixz>
              <iyy>0.01</iyy>
              <iyz>0.0</iyz>
              <izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="stairs_collision0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <surface>
              <bounce />
              <friction>
                <ode>
                  <mu>1.0</mu>
                  <mu2>1.0</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>10000000.0</kp>
                  <kd>1.0</kd>
                  <min_depth>0.0</min_depth>
                  <max_vel>0.0</max_vel>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name="stairs_visual0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Blue</name>
              </script>
            </material>
          </visual>
          <velocity_decay>
            <linear>0.000000</linear>
            <angular>0.000000</angular>
          </velocity_decay>
          <self_collide>0</self_collide>
          <kinematic>0</kinematic>
          <gravity>1</gravity>
        </link>
      </model>
    </sdf>
    """

    sdf_dummy_cube = """<?xml version="1.0" ?>
    <sdf version="1.4">
      <model name="MODELNAME">
        <static>true</static>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <ixy>0.0</ixy>
              <ixz>0.0</ixz>
              <iyy>0.01</iyy>
              <iyz>0.0</iyz>
              <izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="stairs_collision0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <surface>
              <bounce />
              <friction>
                <ode>
                  <mu>10.0</mu>
                  <mu2>10.0</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>10000000.0</kp>
                  <kd>1.0</kd>
                  <min_depth>0.0</min_depth>
                  <max_vel>0.0</max_vel>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name="stairs_visual0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/DarkGray</name>
              </script>
            </material>
          </visual>
          <velocity_decay>
            <linear>0.000000</linear>
            <angular>0.000000</angular>
          </velocity_decay>
          <self_collide>0</self_collide>
          <kinematic>0</kinematic>
          <gravity>1</gravity>
        </link>
      </model>
    </sdf>
    """

    sdf_cube_red = """<?xml version="1.0" ?>
    <sdf version="1.4">
      <model name="MODELNAME">
        <static>0</static>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <ixy>0.0</ixy>
              <ixz>0.0</ixz>
              <iyy>0.01</iyy>
              <iyz>0.0</iyz>
              <izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="stairs_collision0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <surface>
              <bounce />
              <friction>
                <ode>
                  <mu>10.0</mu>
                  <mu2>10.0</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>10000000.0</kp>
                  <kd>1.0</kd>
                  <min_depth>0.0</min_depth>
                  <max_vel>0.0</max_vel>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name="stairs_visual0">
            <geometry>
            <mesh>
              <uri>{}/scripts/materials/red_pkg/red_pkg.obj</uri>
            </mesh>
            </geometry>
          </visual>
          <velocity_decay>
            <linear>0.000000</linear>
            <angular>0.000000</angular>
          </velocity_decay>
          <self_collide>0</self_collide>
          <kinematic>0</kinematic>
          <gravity>1</gravity>
        </link>
      </model>
    </sdf>
    """.format(str_pkg_path)



    sdf_cube_yellow = """<?xml version="1.0" ?>
    <sdf version="1.4">
      <model name="MODELNAME">
        <static>0</static>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <ixy>0.0</ixy>
              <ixz>0.0</ixz>
              <iyy>0.01</iyy>
              <iyz>0.0</iyz>
              <izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="stairs_collision0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <surface>
              <bounce />
              <friction>
                <ode>
                  <mu>10.0</mu>
                  <mu2>10.0</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>10000000.0</kp>
                  <kd>1.0</kd>
                  <min_depth>0.0</min_depth>
                  <max_vel>0.0</max_vel>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name="stairs_visual0">
            <geometry>
            <mesh>
              <uri>{}/scripts/materials/yellow_pkg/yellow_pkg.obj</uri>
            </mesh>
            </geometry>
          </visual>
          <velocity_decay>
            <linear>0.000000</linear>
            <angular>0.000000</angular>
          </velocity_decay>
          <self_collide>0</self_collide>
          <kinematic>0</kinematic>
          <gravity>1</gravity>
        </link>
      </model>
    </sdf>
    """.format(str_pkg_path)


    sdf_cube_green = """<?xml version="1.0" ?>
    <sdf version="1.4">
      <model name="MODELNAME">
        <static>0</static>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <ixy>0.0</ixy>
              <ixz>0.0</ixz>
              <iyy>0.01</iyy>
              <iyz>0.0</iyz>
              <izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="stairs_collision0">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <box>
                <size>SIZEXYZ</size>
              </box>
            </geometry>
            <surface>
              <bounce />
              <friction>
                <ode>
                  <mu>10.0</mu>
                  <mu2>10.0</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>10000000.0</kp>
                  <kd>1.0</kd>
                  <min_depth>0.0</min_depth>
                  <max_vel>0.0</max_vel>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name="stairs_visual0">
            <geometry>
            <mesh>
              <uri>{}/scripts/materials/green_pkg/green_pkg.obj</uri>
            </mesh>
            </geometry>
          </visual>
          <velocity_decay>
            <linear>0.000000</linear>
            <angular>0.000000</angular>
          </velocity_decay>
          <self_collide>0</self_collide>
          <kinematic>0</kinematic>
          <gravity>1</gravity>
        </link>
      </model>
    </sdf>
    """.format(str_pkg_path)

    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")
    
    # rospy.sleep(5)
    
    # Spawn Box

    req00 = create_cube_request(sdf_dummy_cube, "base_of_pkg_00",
                              0.28, 6.43, 1.83,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size

    req01 = create_cube_request(sdf_dummy_cube, "base_of_pkg_01",
                              0.0, 6.43, 1.83,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.28, 0.41, 0.025)  # size

    req02 = create_cube_request(sdf_dummy_cube, "base_of_pkg_02",
                              -0.28, 6.43, 1.83,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size

    # ----------------
    req10 = create_cube_request(sdf_dummy_cube, "base_of_pkg_10",
                              0.28, 6.43, 1.56,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size

    req11 = create_cube_request(sdf_dummy_cube, "base_of_pkg_11",
                              0.0, 6.43, 1.56,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.28, 0.41, 0.025)  # size

    req12 = create_cube_request(sdf_dummy_cube, "base_of_pkg_12",
                              -0.28, 6.43, 1.56,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size
    # ------------------
    req20 = create_cube_request(sdf_dummy_cube, "base_of_pkg_20",
                              0.28, 6.43, 1.34,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size

    req21 = create_cube_request(sdf_dummy_cube, "base_of_pkg_21",
                              0.0, 6.43, 1.34,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.28, 0.41, 0.025)  # size

    req22 = create_cube_request(sdf_dummy_cube, "base_of_pkg_22",
                              -0.28, 6.43, 1.34,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size
    # ------------------
    req30 = create_cube_request(sdf_dummy_cube, "base_of_pkg_30",
                              0.28, 6.43, 1.11,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size

    req31 = create_cube_request(sdf_dummy_cube, "base_of_pkg_31",
                              0.0, 6.43, 1.11,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.28, 0.41, 0.025)  # size

    req32 = create_cube_request(sdf_dummy_cube, "base_of_pkg_32",
                              -0.28, 6.43, 1.11,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.41, 0.025)  # size
    # ------------------
    # ------------------

    req_back_pad_00 = create_cube_request(sdf_dummy_cube, "back_pad_00",
                              0.28, 6.31, 1.96,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_01 = create_cube_request(sdf_dummy_cube, "back_pad_01",
                              0.0, 6.31, 1.96,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_02 = create_cube_request(sdf_dummy_cube, "back_pad_02",
                              -0.28, 6.31, 1.96,  # position`
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    
    
    req_back_pad_10 = create_cube_request(sdf_dummy_cube, "back_pad_10",
                              0.28, 6.31, 1.66,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_11 = create_cube_request(sdf_dummy_cube, "back_pad_11",
                              0.0, 6.31, 1.66,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_12 = create_cube_request(sdf_dummy_cube, "back_pad_12",
                              -0.28, 6.31, 1.66,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size


    
    
    req_back_pad_20 = create_cube_request(sdf_dummy_cube, "back_pad_20",
                              0.28, 6.31, 1.44,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_21 = create_cube_request(sdf_dummy_cube, "back_pad_21",
                              0.0, 6.31, 1.44,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_22 = create_cube_request(sdf_dummy_cube, "back_pad_22",
                              -0.28, 6.31, 1.44,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size


    
    
    req_back_pad_30 = create_cube_request(sdf_dummy_cube, "back_pad_30",
                              0.28, 6.31, 1.21,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_31 = create_cube_request(sdf_dummy_cube, "back_pad_31",
                              0.0, 6.31, 1.21,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size

    req_back_pad_32 = create_cube_request(sdf_dummy_cube, "back_pad_32",
                              -0.28, 6.31, 1.21,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.26, 0.25, 0.15)  # size
    
    

    param_config_package_colour = rospy.get_param('package_colour')
        
    colour_packagen00 = param_config_package_colour['package_00']
    colour_packagen01 = param_config_package_colour['package_01']
    colour_packagen02 = param_config_package_colour['package_02']

    colour_packagen10 = param_config_package_colour['package_10']
    colour_packagen11 = param_config_package_colour['package_11']
    colour_packagen12 = param_config_package_colour['package_12']

    colour_packagen20 = param_config_package_colour['package_20']
    colour_packagen21 = param_config_package_colour['package_21']
    colour_packagen22 = param_config_package_colour['package_22']

    colour_packagen30 = param_config_package_colour['package_30']
    colour_packagen31 = param_config_package_colour['package_31']
    colour_packagen32 = param_config_package_colour['package_32']

    # Spawn Box
    # ----------------------------
    req_pkg00 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen00), "packagen00",
                              0.28, 6.59, 1.93,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg01 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen01), "packagen01",
                              0.0, 6.59, 1.93,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg02 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen02), "packagen02",
                              -0.28, 6.59, 1.93,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size
    # ----------------------------

    # ----------------------------
    req_pkg10 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen10), "packagen10",
                              0.28, 6.59, 1.63,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg11 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen11), "packagen11",
                              0.0, 6.59, 1.63,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg12 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen12), "packagen12",
                              -0.28, 6.59, 1.63,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size
    # ----------------------------
    
    # ----------------------------
    req_pkg20 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen20), "packagen20",
                              0.28, 6.59, 1.41,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg21 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen21), "packagen21",
                              0.0, 6.59, 1.41,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg22 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen22), "packagen22",
                              -0.28, 6.59, 1.41,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size
    # ----------------------------



    # ----------------------------
    req_pkg30 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen30), "packagen30",
                              0.28, 6.59, 1.23,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg31 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen31), "packagen31",
                              0.0, 6.59, 1.23,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size

    req_pkg32 = create_cube_request(paint_colour(sdf_cube_red, sdf_cube_yellow, sdf_cube_green, colour_packagen32), "packagen32",
                              -0.28, 6.59, 1.23,  # position -x 1.2 -y -2.5 -z 0.94
                              0.0, 0.0, 0.0,  # rotation
                              0.15, 0.15, 0.15)  # size
    # ----------------------------

    rospy.sleep(3.0)

    rospy.sleep(0.1)
    spawn_srv.call(req_pkg00)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg01)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg02)

    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg10)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg11)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg12)


    rospy.sleep(0.1)
    spawn_srv.call(req_pkg20)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg21)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg22)

    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg30)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg31)
    
    rospy.sleep(0.1)
    spawn_srv.call(req_pkg32)

    rospy.sleep(1.0)
