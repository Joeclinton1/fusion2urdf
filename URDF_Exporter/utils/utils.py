# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import json
import os.path, re
from datetime import datetime
from xml.etree import ElementTree
from xml.dom import minidom
import shutil  # Replaced distutils with shutil
import fileinput
import sys
import math

DEFAULT_HUMANOID_SHOULDER_HEIGHT = 0.45
MIN_MODELED_HUMANOID_HEIGHT = 0.3
DEFAULT_CENTER_POSITIVE_JOINT_LIMITS = True

def sanitize_name(name):
    return re.sub('[ :()]', '_', name)


def _link_index(link_name):
    if link_name == 'base_link' or link_name == 'link0':
        return 0

    match = re.match(r'^link(\d+)$', link_name or '')
    if match:
        return int(match.group(1))

    return None


def _joint_xyz(joint):
    xyz = joint.get('xyz') or [0, 0, 0]
    try:
        return [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    except:
        return None


def _distance(point_a, point_b):
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(point_a, point_b)]))


def _ordered_numbered_chain_joints(joints_dict):
    ordered = []

    for joint in joints_dict.values():
        if joint.get('type') == 'fixed':
            continue

        parent_index = _link_index(joint.get('parent'))
        child_index = _link_index(joint.get('child'))
        if parent_index is None or child_index is None:
            continue
        if child_index != parent_index + 1:
            continue

        xyz = _joint_xyz(joint)
        if xyz is None:
            continue

        ordered.append((child_index, xyz))

    ordered.sort(key=lambda item: item[0])
    return [xyz for _, xyz in ordered]


def humanoid_arm_length(joints_dict):
    """
    Estimate reach from the ordered revolute joint origins.

    The exporter collapses each numbered top-level component into a URDF link.
    For humanoid placement, the useful fallback height is the modeled chain
    length, not the tabletop Z position of the first joint.
    """
    joint_positions = _ordered_numbered_chain_joints(joints_dict)
    if len(joint_positions) < 2:
        return 0.0

    length = 0.0
    previous = joint_positions[0]
    for current in joint_positions[1:]:
        length += _distance(previous, current)
        previous = current

    return round(length, 6)


def _settings_path():
    appdata = os.environ.get('APPDATA')
    if appdata:
        settings_dir = os.path.join(appdata, 'Fusion2URDF')
    else:
        settings_dir = os.path.join(os.path.expanduser('~'), '.fusion2urdf')
    try:
        os.makedirs(settings_dir)
    except:
        pass
    return os.path.join(settings_dir, 'settings.json')


def settings_path():
    return _settings_path()


def _load_settings():
    try:
        with open(_settings_path(), mode='r') as f:
            return json.load(f)
    except:
        return {}


def _save_settings(settings):
    try:
        with open(_settings_path(), mode='w') as f:
            json.dump(settings, f, indent=2)
    except:
        pass


def _load_export_profile():
    try:
        settings = _load_settings()
        return _normalize_export_profile(settings.get('export_profile', 'tabletop'))
    except:
        return 'tabletop'


def _save_export_profile(profile):
    try:
        settings = _load_settings()
        settings['export_profile'] = profile
        _save_settings(settings)
    except:
        pass


def _normalize_export_profile(value):
    value = (value or '').strip().lower().replace('_', '-')
    value = re.sub(r'\s+', '-', value)

    aliases = {
        'robot': 'tabletop',
        'normal': 'tabletop',
        'non-humanoid': 'tabletop',
        'nonhumanoid': 'tabletop',
        'humanoid': 'humanoid-left',
        'left': 'humanoid-left',
        'l': 'humanoid-left',
        'right': 'humanoid-right',
        'r': 'humanoid-right',
    }
    return aliases.get(value, value)


def export_profile_settings(profile, saved_settings=None):
    profile = _normalize_export_profile(profile)
    saved_settings = saved_settings or {}
    settings = {
        'profile': profile,
        'is_humanoid': False,
        'arm_side': None,
        'root_link': 'base_link',
        'root_rpy': [0, 0, 0],
        'shoulder_height': DEFAULT_HUMANOID_SHOULDER_HEIGHT,
        'center_positive_joint_limits': saved_settings.get(
            'center_positive_joint_limits',
            DEFAULT_CENTER_POSITIVE_JOINT_LIMITS,
        ),
        'joint_limits': saved_settings.get('joint_limits', {}),
    }

    if profile == 'humanoid-left':
        settings.update({
            'is_humanoid': True,
            'arm_side': 'left',
            'root_link': 'humanoid_root',
            'root_rpy': [0, 0, round(math.pi / 2, 6)],
        })
    elif profile == 'humanoid-right':
        settings.update({
            'is_humanoid': True,
            'arm_side': 'right',
            'root_link': 'humanoid_root',
            'root_rpy': [0, 0, round(-math.pi / 2, 6)],
        })

    return settings


def _limit_number(value, degrees=False):
    try:
        number = float(value)
    except:
        return None
    if degrees:
        number = math.radians(number)
    return round(number, 6)


def parse_joint_limit_override(value):
    """
    Return [lower, upper] in radians from a JSON setting.

    Supported forms:
      "joint": [-1.57, 1.57]
      "joint": {"lower": -90, "upper": 90, "unit": "deg"}
      "joint": {"degrees": [-90, 90]}
      "joint": {"radians": [-1.57, 1.57]}
    """
    degrees = False
    lower = None
    upper = None

    if isinstance(value, dict):
        if 'degrees' in value:
            value = value.get('degrees')
            degrees = True
        elif 'radians' in value:
            value = value.get('radians')
        else:
            unit = str(value.get('unit', '')).lower()
            degrees = unit in ('deg', 'degree', 'degrees')
            lower = value.get('lower')
            upper = value.get('upper')

    if isinstance(value, list) or isinstance(value, tuple):
        if len(value) < 2:
            return None
        lower = value[0]
        upper = value[1]
        if len(value) >= 3:
            unit = str(value[2]).lower()
            degrees = unit in ('deg', 'degree', 'degrees')

    lower = _limit_number(lower, degrees)
    upper = _limit_number(upper, degrees)
    if lower is None or upper is None:
        return None
    if lower > upper:
        lower, upper = upper, lower
    return [lower, upper]


def center_positive_joint_limit(joint_dict):
    if joint_dict.get('type') != 'revolute':
        return False

    try:
        lower = float(joint_dict.get('lower_limit'))
        upper = float(joint_dict.get('upper_limit'))
    except:
        return False

    if lower < -1e-6 or upper <= lower:
        return False

    span = upper - lower
    if span <= 1e-6:
        return False

    joint_dict['lower_limit'] = round(-span / 2.0, 6)
    joint_dict['upper_limit'] = round(span / 2.0, 6)
    joint_dict['limit_source'] = 'centered positive Fusion range'
    return True


def apply_joint_limit_settings(joint_name, joint_dict, export_settings=None):
    export_settings = export_settings or {}
    overrides = export_settings.get('joint_limits') or {}
    override = parse_joint_limit_override(overrides.get(joint_name))

    if override:
        joint_dict['type'] = 'revolute'
        joint_dict['lower_limit'] = override[0]
        joint_dict['upper_limit'] = override[1]
        joint_dict['limit_source'] = 'settings override'
        return

    if export_settings.get('center_positive_joint_limits', DEFAULT_CENTER_POSITIVE_JOINT_LIMITS):
        center_positive_joint_limit(joint_dict)


def humanoid_shoulder_height(joints_dict, default_height=DEFAULT_HUMANOID_SHOULDER_HEIGHT):
    """
    Return a URDF Z shoulder height for humanoid profiles.

    If the source model is already placed at a body-height shoulder, preserve
    that. Otherwise lift tabletop-modeled arms by their measured chain length.
    """
    first_joint = None
    first_child_index = None

    for joint in joints_dict.values():
        child_index = _link_index(joint.get('child'))
        if child_index is None:
            continue
        if first_child_index is None or child_index < first_child_index:
            first_child_index = child_index
            first_joint = joint

    if not first_joint:
        return 0.0

    xyz = _joint_xyz(first_joint)
    try:
        modeled_height = float(xyz[2])
        if abs(modeled_height) >= MIN_MODELED_HUMANOID_HEIGHT:
            return round(modeled_height, 6)
    except:
        pass

    arm_length = humanoid_arm_length(joints_dict)
    if arm_length > 0.0:
        return arm_length

    return round(float(default_height), 6)


def prompt_export_settings(ui):
    saved_settings = _load_settings()
    default_profile = _normalize_export_profile(saved_settings.get('export_profile', 'tabletop'))
    prompt = (
        'Export profile:\n'
        '  tabletop\n'
        '  humanoid-left\n'
        '  humanoid-right\n\n'
        'Humanoid profiles add a fixed shoulder mount that rotates the whole arm.\n\n'
        'Optional joint limit overrides can be edited in:\n'
        '{}'.format(_settings_path())
    )

    try:
        value, cancelled = ui.inputBox(prompt, 'Fusion2URDF settings', default_profile)
        if cancelled:
            return None
    except:
        value = default_profile

    profile = _normalize_export_profile(value)
    if profile not in ('tabletop', 'humanoid-left', 'humanoid-right'):
        ui.messageBox(
            'Unknown export profile "{}". Use tabletop, humanoid-left, or humanoid-right.'.format(value),
            'Fusion2URDF'
        )
        return None

    _save_export_profile(profile)
    saved_settings['export_profile'] = profile
    return export_profile_settings(profile, saved_settings)


def make_unique_export_dir(parent_dir, package_name):
    """
    Create a new export directory without overwriting an earlier export.
    """
    base_dir = os.path.join(parent_dir, package_name)
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
        return base_dir

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    candidate = os.path.join(parent_dir, package_name + '_' + timestamp)
    suffix = 1
    while os.path.exists(candidate):
        candidate = os.path.join(parent_dir, package_name + '_' + timestamp + '_' + str(suffix))
        suffix += 1

    os.makedirs(candidate)
    return candidate


def collect_link_occurrences(root):
    """
    Treat each visible root occurrence as one URDF link.

    Nested components often represent CAD/detail structure, not robot links. A
    root occurrence's physical properties include its nested bodies, and Fusion
    can export the occurrence as one STL.
    """
    links = []
    used_names = set()

    for occs in root.occurrences:
        component_name = sanitize_name(occs.component.name)
        occurrence_base_name = sanitize_name(occs.name.split(':')[0])

        if component_name in ('base_link', 'link0') or occurrence_base_name in ('base_link', 'link0'):
            link_name = 'base_link'
        else:
            link_name = component_name

        base_name = link_name
        suffix = 2
        while link_name in used_names:
            link_name = base_name + '_' + str(suffix)
            suffix += 1

        used_names.add(link_name)
        links.append({
            'occurrence': occs,
            'occurrence_name': occs.name,
            'component_name': occs.component.name,
            'link_name': link_name,
        })

    return links


def _iter_occurrence_tree(occurrence):
    yield occurrence
    try:
        for child in occurrence.childOccurrences:
            for nested in _iter_occurrence_tree(child):
                yield nested
    except:
        pass


def _iter_occurrence_bodies(occurrence):
    for occ in _iter_occurrence_tree(occurrence):
        try:
            for body in occ.bRepBodies:
                yield body
        except:
            pass


def _color_to_rgba(color):
    try:
        result = color.getColor()
        if len(result) == 5:
            _, red, green, blue, opacity = result
        else:
            red, green, blue, opacity = result
        return [
            round(red / 255.0, 6),
            round(green / 255.0, 6),
            round(blue / 255.0, 6),
            round(opacity / 255.0, 6),
        ]
    except:
        pass

    try:
        return [
            round(color.red / 255.0, 6),
            round(color.green / 255.0, 6),
            round(color.blue / 255.0, 6),
            round(color.opacity / 255.0, 6),
        ]
    except:
        return None


def _rgba_from_appearance(appearance):
    if not appearance:
        return None

    try:
        color_property = appearance.appearanceProperties.itemById('opaque_albedo')
        colors = color_property.values
        if colors and len(colors) > 0:
            return _color_to_rgba(colors[0])
    except:
        pass

    try:
        properties = appearance.appearanceProperties
        for index in range(properties.count):
            prop = properties.item(index)
            try:
                colors = prop.values
                if colors and len(colors) > 0:
                    rgba = _color_to_rgba(colors[0])
                    if rgba:
                        return rgba
            except:
                pass
    except:
        pass

    return None


def _appearance_name(appearance, fallback_name):
    try:
        if appearance and appearance.name:
            return sanitize_name(appearance.name)
    except:
        pass
    return fallback_name


def _material_from_appearance(appearance, fallback_name):
    rgba = _rgba_from_appearance(appearance)
    if not rgba:
        return None

    return {
        'name': _appearance_name(appearance, fallback_name),
        'rgba': rgba,
    }


def _body_material(body, fallback_name):
    for appearance_getter in [
        lambda: body.appearance,
        lambda: body.material.appearance,
    ]:
        try:
            material = _material_from_appearance(appearance_getter(), fallback_name)
            if material:
                return material
        except:
            pass
    return None


def gripper_visual_meshes(link_occurrences, visual_mesh_extension='obj'):
    """
    Return browser visual mesh overrides for gripper children that should stay
    separately addressable in the viewer.
    """
    visual_meshes = {}

    for link in link_occurrences:
        link_name = link['link_name']
        if link_name != 'gripper':
            continue

        occurrence = link['occurrence']
        meshes = []
        used_names = set()

        try:
            children = occurrence.childOccurrences
        except:
            children = []

        for child in children:
            child_name = sanitize_name(child.name.split(':')[0])
            component_name = sanitize_name(child.component.name)
            combined = (child_name + ' ' + component_name).lower()

            if 'pincopen' in combined:
                mesh_name = link_name + '_pincopen'
            elif 'camera' in combined:
                mesh_name = link_name + '_camera'
            else:
                continue

            base_name = mesh_name
            suffix = 2
            while mesh_name in used_names:
                mesh_name = base_name + '_' + str(suffix)
                suffix += 1
            used_names.add(mesh_name)

            meshes.append({
                'occurrence': child,
                'mesh_name': mesh_name,
                'extension': visual_mesh_extension,
                'source_occurrence_name': child.name,
                'source_component_name': child.component.name,
            })

        if meshes:
            visual_meshes[link_name] = meshes

    return visual_meshes


def _matrix_array(occurrence):
    try:
        return [float(_) for _ in occurrence.transform2.asArray()]
    except:
        try:
            return [float(_) for _ in occurrence.transform.asArray()]
        except:
            return None


def write_gripper_visual_debug(save_dir, visual_meshes):
    debug = {}
    for link_name, meshes in (visual_meshes or {}).items():
        debug[link_name] = []
        for mesh in meshes:
            occurrence = mesh.get('occurrence')
            data = dict((key, value) for key, value in mesh.items() if key != 'occurrence')
            data['transform_cm'] = _matrix_array(occurrence)
            debug[link_name].append(data)

    file_name = os.path.join(save_dir, 'gripper_visual_debug.json')
    with open(file_name, mode='w') as f:
        json.dump(debug, f, indent=2)


def collect_link_materials(link_occurrences):
    """
    Return one URDF material per exported top-level link.

    The current exporter emits one STL per link, so a link can only reference one
    URDF material. If a link contains multiple body colors, the first body color
    found in the occurrence tree is used.
    """
    materials = {}

    for link in link_occurrences:
        link_name = link['link_name']
        fallback_name = 'material_' + link_name
        occurrence = link['occurrence']
        material = None

        try:
            material = _material_from_appearance(occurrence.appearance, fallback_name)
        except:
            pass

        if not material:
            for body in _iter_occurrence_bodies(occurrence):
                material = _body_material(body, fallback_name)
                if material:
                    break

        if not material:
            material = {
                'name': fallback_name,
                'rgba': [0.7, 0.7, 0.7, 1.0],
            }

        material['name'] = fallback_name
        materials[link_name] = material

    return materials


def link_name_for_occurrence(occurrence, link_occurrences):
    """
    Map a joint endpoint occurrence to its top-level URDF link name.
    """
    if not occurrence:
        return None

    top_name = None
    try:
        full_path_name = occurrence.fullPathName
        if full_path_name:
            top_name = full_path_name.split('+')[0]
    except:
        pass

    if not top_name:
        try:
            top_name = occurrence.name
        except:
            return None

    for link in link_occurrences:
        if top_name == link['occurrence_name']:
            return link['link_name']

    return None


def all_design_joints(root):
    try:
        design = root.parentDesign
        components = design.allComponents
    except:
        try:
            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)
            components = design.allComponents
        except:
            components = [root]

    for component in components:
        try:
            for joint in component.joints:
                yield component, joint
        except:
            pass


def write_export_debug(save_dir, root, link_occurrences, joints_dict, skipped_joints):
    debug = {
        'root_component': root.name,
        'links': [{
            'occurrence_name': link['occurrence_name'],
            'component_name': link['component_name'],
            'link_name': link['link_name'],
        } for link in link_occurrences],
        'joints': joints_dict,
        'skipped_joints': skipped_joints,
    }

    file_name = os.path.join(save_dir, 'fusion2urdf_export_debug.json')
    with open(file_name, mode='w') as f:
        json.dump(debug, f, indent=2)


def copy_occs(root):    
    """    
    duplicate all body-containing root occurrences for STL export
    """    
    export_state = {'created_occurrences': [], 'renamed_components': []}

    def copy_body(allOccs, occs):
        """    
        copy the old occs to new component
        """
        
        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()
        
        # Create new components from occs
        # This support even when a component has some occses. 

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        export_state['created_occurrences'].append(new_occs)
        export_state['renamed_components'].append((occs.component, occs.component.name))
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = re.sub('[ :()]', '_', occs.name)
        new_occs = allOccs.item((allOccs.count-1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)
    
    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'

    return export_state


def restore_occs(export_state):
    """
    Remove temporary export occurrences and restore component names changed by copy_occs.
    """
    if not export_state:
        return

    for occs in reversed(export_state.get('created_occurrences', [])):
        try:
            occs.deleteMe()
        except:
            pass

    for component, original_name in reversed(export_state.get('renamed_components', [])):
        try:
            component.name = original_name
        except:
            pass


def export_stl(design, save_dir, components):  
    """
    export stl files into "save_dir/"
    
    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """
          
    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'  
    # export the occurrence one by one in the component to a specified file
    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name              
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + ' has something wrong.')


def export_stl_links(design, save_dir, link_occurrences):
    """
    Export one STL per top-level URDF link occurrence.
    """
    exportMgr = design.exportManager
    try: os.mkdir(save_dir + '/meshes')
    except: pass

    scriptDir = save_dir + '/meshes'
    for link in link_occurrences:
        occ = link['occurrence']
        fileName = scriptDir + "/" + link['link_name']
        try:
            stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exportMgr.execute(stlExportOptions)
        except Exception as e:
            print('Component ' + link['occurrence_name'] + ' failed STL export: ' + str(e))


def _set_export_option(options, name, value):
    try:
        setattr(options, name, value)
        return True
    except:
        return False


def export_obj_links(design, save_dir, link_occurrences, visual_meshes=None):
    """
    Export one OBJ visual mesh per top-level URDF link occurrence.

    OBJ keeps Fusion appearance colors through its companion MTL file while the
    existing STL export remains the collision mesh.
    """
    exportMgr = design.exportManager
    try: os.mkdir(save_dir + '/meshes')
    except: pass

    scriptDir = save_dir + '/meshes'
    visual_meshes = visual_meshes or {}
    for link in link_occurrences:
        export_meshes = visual_meshes.get(link['link_name']) or [{
            'occurrence': link['occurrence'],
            'mesh_name': link['link_name'],
        }]

        for visual_mesh in export_meshes:
            occ = visual_mesh['occurrence']
            fileName = scriptDir + "/" + visual_mesh['mesh_name']
            try:
                objExportOptions = exportMgr.createOBJExportOptions(occ, fileName)
                _set_export_option(objExportOptions, 'meshRefinement', adsk.fusion.MeshRefinementSettings.MeshRefinementLow)
                _set_export_option(objExportOptions, 'unitType', adsk.fusion.DistanceUnits.MillimeterDistanceUnits)
                exportMgr.execute(objExportOptions)
            except Exception as e:
                print('Component ' + link['occurrence_name'] + ' failed OBJ export: ' + str(e))


def file_dialog(ui):     
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def write_model_snapshot(root, joints_dict, inertial_dict, save_dir):
    """
    Write model structure that can be inspected outside Fusion during development.
    """
    snapshot = {
        'root_component': root.name,
        'occurrences': [],
        'joints': joints_dict,
        'inertial_links': inertial_dict,
    }

    for occs in root.occurrences:
        try:
            body_count = occs.bRepBodies.count
        except:
            body_count = None

        snapshot['occurrences'].append({
            'occurrence_name': occs.name,
            'component_name': occs.component.name,
            'body_count': body_count,
        })

    file_name = os.path.join(save_dir, 'fusion2urdf_snapshot.json')
    with open(file_name, mode='w') as f:
        json.dump(snapshot, f, indent=2)


def save_viewport_image(save_dir):
    """
    Best-effort viewport capture for debugging. Some Fusion environments do not
    expose image capture to scripts, so failures are intentionally ignored.
    """
    try:
        app = adsk.core.Application.get()
        viewport = app.activeViewport
        file_name = os.path.join(save_dir, 'fusion_view.png')
        viewport.saveAsImageFile(file_name, 1600, 1000)
    except:
        pass


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate

    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2,
                         -x*y, -y*z, -x*z]
    return [round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element
    
    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    try:
        # Check if the target directory exists, if not, create it
        if not os.path.exists(save_dir + '/launch'):
            os.mkdir(save_dir + '/launch')
        if not os.path.exists(save_dir + '/urdf'):
            os.mkdir(save_dir + '/urdf')
        
        # Check if the package directory exists and copy it
        if os.path.exists(package_dir):
            shutil.copytree(package_dir, save_dir, dirs_exist_ok=True)  # dirs_exist_ok=True allows overwriting
        else:
            print(f"Package directory '{package_dir}' does not exist.")
        
    except Exception as e:
        print(f"Error copying package: {e}")


def update_cmakelists(save_dir, package_name):
    file_name = save_dir + '/CMakeLists.txt'

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)


def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
