# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils


def _link_index(link_name):
    if link_name == 'base_link':
        return 0

    match = re.match(r'^link(\d+)$', link_name or '')
    if match:
        return int(match.group(1))

    return None


def _previous_link_name(link_name):
    index = _link_index(link_name)
    if index is None or index <= 0:
        return None
    if index == 1:
        return 'base_link'
    return 'link' + str(index - 1)


def _normalize_parent_child(parent, child):
    parent_index = _link_index(parent)
    child_index = _link_index(child)

    if parent_index is not None and child_index is not None and parent_index > child_index:
        return child, parent

    return parent, child


def _is_adjacent_numbered_edge(parent, child):
    parent_index = _link_index(parent)
    child_index = _link_index(child)
    return parent_index is not None and child_index is not None and child_index == parent_index + 1


def _last_numbered_link(link_occurrences):
    last_link = None
    last_index = None

    for link in link_occurrences:
        index = _link_index(link['link_name'])
        if index is None:
            continue
        if last_index is None or index > last_index:
            last_index = index
            last_link = link['link_name']

    return last_link


def _has_link(link_occurrences, link_name):
    for link in link_occurrences:
        if link['link_name'] == link_name:
            return True
    return False


def _edge_joint_name(parent, child):
    return utils.sanitize_name(parent + '_to_' + child)


def _joint_xyz(joint):
    #There seem to be a problem with geometryOrOriginTwo. To calcualte the correct orogin of the generated stl files following approach was used.
    #https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767
    #Thanks to Masaki Yamamoto!

    # Coordinate transformation by matrix
    # M: 4x4 transformation matrix
    # a: 3D vector
    def trans(M, a):
        ex = [M[0],M[4],M[8]]
        ey = [M[1],M[5],M[9]]
        ez = [M[2],M[6],M[10]]
        oo = [M[3],M[7],M[11]]
        b = [0, 0, 0]
        for i in range(3):
            b[i] = a[0]*ex[i]+a[1]*ey[i]+a[2]*ez[i]+oo[i]
        return(b)

    # Returns True if two arrays are element-wise equal within a tolerance
    def allclose(v1, v2, tol=1e-6):
        return( max([abs(a-b) for a,b in zip(v1, v2)]) < tol )

    try:
        xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray() # Relative Joint pos
        xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray() # Relative Joint pos
        xyz_of_one            = joint.occurrenceOne.transform.translation.asArray() # Link origin
        M_two = joint.occurrenceTwo.transform.asArray() # Matrix as a 16 element array.

        # Compose joint position
        case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
        case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
        if case1 or case2:
            xyz_of_joint = xyz_from_two_to_joint
        else:
            xyz_of_joint = trans(M_two, xyz_from_two_to_joint)

        return [round(i / 100.0, 6) for i in xyz_of_joint]  # converted to meter

    except:
        try:
            if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                data = joint.geometryOrOriginTwo.geometry.origin.asArray()
            else:
                data = joint.geometryOrOriginTwo.origin.asArray()
            return [round(i / 100.0, 6) for i in data]  # converted to meter
        except:
            try:
                if type(joint.geometryOrOriginOne)==adsk.fusion.JointOrigin:
                    data = joint.geometryOrOriginOne.geometry.origin.asArray()
                else:
                    data = joint.geometryOrOriginOne.origin.asArray()
                return [round(i / 100.0, 6) for i in data]  # converted to meter
            except:
                return None


class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'
        
        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints
    
    
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status
        
    Returns
    ----------
    joints_dict: 
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    joints_dict = {}
    skipped_joints = []
    link_occurrences = utils.collect_link_occurrences(root)
    used_edges = set()
    
    for owner_component, joint in utils.all_design_joints(root):
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        if joint_type != 'revolute':
            skipped_joints.append({
                'name': joint.name,
                'owner_component': owner_component.name,
                'reason': 'ignored non-revolute joint type ' + joint_type,
                'occurrence_one': joint.occurrenceOne.fullPathName if joint.occurrenceOne else None,
                'occurrence_two': joint.occurrenceTwo.fullPathName if joint.occurrenceTwo else None,
            })
            continue

        joint_dict['type'] = joint_type
        
        # swhich by the type of the joint
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0
        
        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == 'revolute':
            joint_dict['axis'] = [round(i, 6) for i in \
                joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'

        parent = utils.link_name_for_occurrence(joint.occurrenceTwo, link_occurrences)
        child = utils.link_name_for_occurrence(joint.occurrenceOne, link_occurrences)
        inferred_reason = None

        if parent and child:
            parent, child = _normalize_parent_child(parent, child)

        if not parent or not child:
            skipped_joints.append({
                'name': joint.name,
                'owner_component': owner_component.name,
                'reason': 'could not map occurrenceOne/occurrenceTwo to root link occurrence',
                'occurrence_one': joint.occurrenceOne.fullPathName if joint.occurrenceOne else None,
                'occurrence_two': joint.occurrenceTwo.fullPathName if joint.occurrenceTwo else None,
            })
            continue

        if parent == child:
            skipped_joints.append({
                'name': joint.name,
                'owner_component': owner_component.name,
                'reason': 'joint is internal to collapsed link ' + parent,
                'occurrence_one': joint.occurrenceOne.fullPathName if joint.occurrenceOne else None,
                'occurrence_two': joint.occurrenceTwo.fullPathName if joint.occurrenceTwo else None,
            })
            continue

        if not _is_adjacent_numbered_edge(parent, child):
            skipped_joints.append({
                'name': joint.name,
                'owner_component': owner_component.name,
                'reason': 'joint endpoints are not adjacent exported links ' + parent + ' -> ' + child,
                'occurrence_one': joint.occurrenceOne.fullPathName if joint.occurrenceOne else None,
                'occurrence_two': joint.occurrenceTwo.fullPathName if joint.occurrenceTwo else None,
            })
            continue

        edge = (parent, child)
        if edge in used_edges:
            skipped_joints.append({
                'name': joint.name,
                'owner_component': owner_component.name,
                'reason': 'duplicate collapsed top-level joint ' + parent + ' -> ' + child,
                'occurrence_one': joint.occurrenceOne.fullPathName if joint.occurrenceOne else None,
                'occurrence_two': joint.occurrenceTwo.fullPathName if joint.occurrenceTwo else None,
            })
            continue
        used_edges.add(edge)

        joint_dict['parent'] = parent
        joint_dict['child'] = child

        xyz = _joint_xyz(joint)
        if xyz is None:
            msg = joint.name + " doesn't have joint origin. Please set it and run again."
            break
        joint_dict['xyz'] = xyz
        if inferred_reason:
            joint_dict['inferred_reason'] = inferred_reason
        
        joint_name = _edge_joint_name(parent, child)
        if joint_name in joints_dict:
            suffix = 2
            base_name = joint_name
            while joint_name in joints_dict:
                joint_name = base_name + '_' + str(suffix)
                suffix += 1
        joints_dict[joint_name] = joint_dict

    if _has_link(link_occurrences, 'gripper'):
        parent = _last_numbered_link(link_occurrences)
        edge = (parent, 'gripper')
        if parent and edge not in used_edges:
            joints_dict[_edge_joint_name(parent, 'gripper')] = {
                'type': 'fixed',
                'axis': [0, 0, 0],
                'upper_limit': 0.0,
                'lower_limit': 0.0,
                'parent': parent,
                'child': 'gripper',
                'xyz': [0, 0, 0],
                'inferred_reason': 'synthetic fixed joint from last numbered link to gripper',
            }
            used_edges.add(edge)

    make_joints_dict.skipped_joints = skipped_joints
    return joints_dict, msg
