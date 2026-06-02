# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:46:26 2019

@author: syuntoku
"""

import adsk, os
from xml.etree.ElementTree import Element, SubElement
from . import Link, Joint
from ..utils import utils

def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict, materials_dict=None, visual_mesh_extension='stl', visual_meshes=None):
    """
    Write links information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial
    
    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        materials_dict = materials_dict or {}
        visual_meshes = visual_meshes or {}
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        material_name = materials_dict.get('base_link', {}).get('name', 'silver')
        link = Link.Link(name='base_link', xyz=[0,0,0], 
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'],
            material_name=material_name,
            visual_mesh_extension=visual_mesh_extension,
            visual_meshes=visual_meshes.get('base_link'))
        links_xyz_dict[link.name] = link.xyz
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = \
                [ i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            material_name = materials_dict.get(name, {}).get('name', 'silver')
            link = Link.Link(name=name, xyz=joints_dict[joint]['xyz'],\
                center_of_mass=center_of_mass,\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia_tensor=inertial_dict[name]['inertia'],
                material_name=material_name,
                visual_mesh_extension=visual_mesh_extension,
                visual_meshes=visual_meshes.get(name))
            links_xyz_dict[link.name] = link.xyz            
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """
    Write joints and transmission information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """
    
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()
                
            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            joint.make_joint_xml()
            joint.make_transmission_xml()
            f.write(joint.joint_xml)
            f.write('\n')

def write_gazebo_endtag(file_name):
    """
    Write about gazebo_plugin and the </robot> tag at the end of the urdf
    
    
    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')


def write_humanoid_root_urdf(file_name, export_settings, shoulder_height=0.0):
    side = export_settings.get('arm_side') or 'arm'
    rpy = export_settings.get('root_rpy') or [0, 0, 0]
    rpy_text = ' '.join([str(_) for _ in rpy])
    shoulder_height_text = str(round(shoulder_height, 6))

    with open(file_name, mode='a') as f:
        f.write('<link name="humanoid_root"/>\n')
        f.write('\n')
        f.write('<joint name="{}_shoulder_mount" type="fixed">\n'.format(side))
        f.write('  <origin xyz="0 0 {}" rpy="{}"/>\n'.format(shoulder_height_text, rpy_text))
        f.write('  <parent link="humanoid_root"/>\n')
        f.write('  <child link="base_link"/>\n')
        f.write('</joint>\n')
        f.write('\n')


def write_material_definitions(file_name, materials_dict):
    materials = materials_dict or {}
    written = set()

    with open(file_name, mode='a') as f:
        if not materials:
            f.write('<material name="silver">\n')
            f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
            f.write('</material>\n')
            f.write('\n')
            return

        for link_name in sorted(materials.keys()):
            material = materials[link_name]
            name = material.get('name') or ('material_' + link_name)
            if name in written:
                continue
            rgba = material.get('rgba') or [0.7, 0.7, 0.7, 1.0]
            rgba_text = ' '.join(['{:.6f}'.format(_) for _ in rgba])
            f.write('<material name="{}">\n'.format(name))
            f.write('  <color rgba="{}"/>\n'.format(rgba_text))
            f.write('</material>\n')
            f.write('\n')
            written.add(name)
        

def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass 

    file_name = save_dir + '/urdf/' + robot_name + '.xacro'  # the name of urdf file
    repo = package_name + '/meshes/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/materials.xacro" />'.format(package_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.trans" />'.format(package_name, robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.gazebo" />'.format(package_name, robot_name))
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)
    write_gazebo_endtag(file_name)

def write_browser_urdf(joints_dict, links_xyz_dict, inertial_dict, robot_name, save_dir, export_settings=None, materials_dict=None, visual_meshes=None):
    """
    Write a plain URDF folder for browser loaders.

    The URDF is saved in robot/urdf and references STL meshes with paths
    relative to that file, matching urdf-loaders-obj's examples.
    """
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/' + robot_name + '.urdf'
    repo = '../meshes/'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n'.format(robot_name))
        f.write('\n')

    write_material_definitions(file_name, materials_dict)

    if export_settings and export_settings.get('is_humanoid'):
        shoulder_height = utils.humanoid_shoulder_height(
            joints_dict,
            export_settings.get('shoulder_height', utils.DEFAULT_HUMANOID_SHOULDER_HEIGHT),
        )
        write_humanoid_root_urdf(file_name, export_settings, shoulder_height)

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict, materials_dict, visual_mesh_extension='obj', visual_meshes=visual_meshes)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)
    write_gazebo_endtag(file_name)

def write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass  

    file_name = save_dir + '/urdf/materials.xacro'  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('</robot>\n')

def write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    """
    Write joints and transmission information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """
    
    file_name = save_dir + '/urdf/{}.trans'.format(robot_name)  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()
                
            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')

        f.write('</robot>\n')

def write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass  

    file_name = save_dir + '/urdf/' + robot_name + '.gazebo'  # the name of urdf file
    repo = robot_name + '/meshes/'  # the repository of binary stl files
    #repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <selfCollide>true</selfCollide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <selfCollide>true</selfCollide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')

def write_display_launch(package_name, robot_name, save_dir):
    """
    write display launch file "save_dir/launch/display.launch"


    Parameter
    ---------
    robot_name: str
    name of the robot
    save_dir: str
    path of the repository to save
    """   
    try: os.mkdir(save_dir + '/launch')
    except: pass     

    launch = Element('launch')     

    arg1 = SubElement(launch, 'arg')
    arg1.attrib = {'name':'model', 'default':'$(find {})/urdf/{}.xacro'.format(package_name, robot_name)}

    arg2 = SubElement(launch, 'arg')
    arg2.attrib = {'name':'gui', 'default':'true'}

    arg3 = SubElement(launch, 'arg')
    arg3.attrib = {'name':'rvizconfig', 'default':'$(find {})/launch/urdf.rviz'.format(package_name)}

    param1 = SubElement(launch, 'param')
    param1.attrib = {'name':'robot_description', 'command':'$(find xacro)/xacro $(arg model)'}

    param2 = SubElement(launch, 'param')
    param2.attrib = {'name':'use_gui', 'value':'$(arg gui)'}

    node1 = SubElement(launch, 'node')
    node1.attrib = {'name':'joint_state_publisher_gui', 'pkg':'joint_state_publisher_gui', 'type':'joint_state_publisher_gui'}

    node2 = SubElement(launch, 'node')
    node2.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher', 'type':'robot_state_publisher'}

    node3 = SubElement(launch, 'node')
    node3.attrib = {'name':'rviz', 'pkg':'rviz', 'args':'-d $(arg rvizconfig)', 'type':'rviz', 'required':'true'}

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])        

    file_name = save_dir + '/launch/display.launch'    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)

def write_gazebo_launch(package_name, robot_name, save_dir):
    """
    write gazebo launch file "save_dir/launch/gazebo.launch"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """
    
    try: os.mkdir(save_dir + '/launch')
    except: pass     
    
    launch = Element('launch')
    param = SubElement(launch, 'param')
    param.attrib = {'name':'robot_description', 'command':'$(find xacro)/xacro $(find {})/urdf/{}.xacro'.format(package_name, robot_name)}

    node = SubElement(launch, 'node')
    node.attrib = {'name':'spawn_urdf', 'pkg':'gazebo_ros', 'type':'spawn_model',\
                    'args':'-param robot_description -urdf -model {}'.format(robot_name)}

    include_ =  SubElement(launch, 'include')
    include_.attrib = {'file':'$(find gazebo_ros)/launch/empty_world.launch'}        
    
    number_of_args = 5
    args = [None for i in range(number_of_args)]
    args_name_value_pairs = [['paused', 'true'], ['use_sim_time', 'true'],
                             ['gui', 'true'], ['headless', 'false'], 
                             ['debug', 'false']]
                             
    for i, arg in enumerate(args):
        arg = SubElement(include_, 'arg')
        arg.attrib = {'name' : args_name_value_pairs[i][0] , 
        'value' : args_name_value_pairs[i][1]}


    
    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])        
    
    file_name = save_dir + '/launch/' + 'gazebo.launch'    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)


def write_control_launch(package_name, robot_name, save_dir, joints_dict):
    """
    write control launch file "save_dir/launch/controller.launch"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    
    try: os.mkdir(save_dir + '/launch')
    except: pass     
    
    #launch = Element('launch')

    controller_name = robot_name + '_controller'
    #rosparam = SubElement(launch, 'rosparam')
    #rosparam.attrib = {'file':'$(find {})/launch/controller.yaml'.format(package_name),
    #                   'command':'load'}
                       
    controller_args_str = ""
    for j in joints_dict:
        joint_type = joints_dict[j]['type']
        if joint_type != 'fixed':
            controller_args_str += j + '_position_controller '
    controller_args_str += 'joint_state_controller '

    node_controller = Element('node')
    node_controller.attrib = {'name':'controller_spawner', 'pkg':'controller_manager', 'type':'spawner',\
                    'respawn':'false', 'output':'screen', 'ns':robot_name,\
                    'args':'{}'.format(controller_args_str)}
    
    node_publisher = Element('node')
    node_publisher.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher',\
                    'type':'robot_state_publisher', 'respawn':'false', 'output':'screen'}
    remap = SubElement(node_publisher, 'remap')
    remap.attrib = {'from':'/joint_states',\
                    'to':'/' + robot_name + '/joint_states'}
    
    #launch_xml  = "\n".join(utils.prettify(launch).split("\n")[1:])   
    launch_xml  = "\n".join(utils.prettify(node_controller).split("\n")[1:])   
    launch_xml += "\n".join(utils.prettify(node_publisher).split("\n")[1:])   

    file_name = save_dir + '/launch/controller.launch'    
    with open(file_name, mode='w') as f:
        f.write('<launch>\n')
        f.write('\n')
        #for some reason ROS is very picky about the attribute ordering, so we'll bitbang this element
        f.write('<rosparam file="$(find {})/launch/controller.yaml" command="load"/>'.format(package_name))
        f.write('\n')
        f.write(launch_xml)
        f.write('\n')
        f.write('</launch>')
        

def write_yaml(package_name, robot_name, save_dir, joints_dict):
    """
    write yaml file "save_dir/launch/controller.yaml"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass 

    controller_name = robot_name + '_controller'
    file_name = save_dir + '/launch/controller.yaml'
    with open(file_name, 'w') as f:
        f.write(controller_name + ':\n')
        # joint_state_controller
        f.write('  # Publish all joint states -----------------------------------\n')
        f.write('  joint_state_controller:\n')
        f.write('    type: joint_state_controller/JointStateController\n')  
        f.write('    publish_rate: 50\n\n')
        # position_controllers
        f.write('  # Position Controllers --------------------------------------\n')
        for joint in joints_dict:
            joint_type = joints_dict[joint]['type']
            if joint_type != 'fixed':
                f.write('  ' + joint + '_position_controller:\n')
                f.write('    type: effort_controllers/JointPositionController\n')
                f.write('    joint: '+ joint + '\n')
                f.write('    pid: {p: 100.0, i: 0.01, d: 10.0}\n')

