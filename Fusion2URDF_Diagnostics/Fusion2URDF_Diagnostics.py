# Description: Read-only diagnostic dump for Fusion2URDF development.

import adsk.core
import adsk.fusion
import json
import os
import traceback
from datetime import datetime


JOINT_TYPES = [
    'fixed',
    'revolute',
    'prismatic',
    'cylindrical',
    'pinSlot',
    'planar',
    'ball',
]


def _safe_text(value):
    try:
        if value is None:
            return None
        return str(value)
    except:
        return '<unprintable>'


def _safe_number(value):
    try:
        return float(value)
    except:
        return None


def _object_type(obj):
    try:
        return obj.objectType
    except:
        return type(obj).__name__


def _as_array(obj):
    try:
        return [_safe_number(v) for v in obj.asArray()]
    except:
        return None


def _matrix(matrix):
    arr = _as_array(matrix)
    if arr is None:
        return None
    return {
        'array': arr,
        'translation_cm': [arr[3], arr[7], arr[11]] if len(arr) >= 12 else None,
    }


def _collection_count(collection):
    try:
        return collection.count
    except:
        try:
            return len(collection)
        except:
            return None


def _iter_collection(collection):
    try:
        for item in collection:
            yield item
        return
    except:
        pass

    try:
        count = collection.count
        for index in range(count):
            yield collection.item(index)
    except:
        return


def _body_summary(body):
    data = {
        'name': _safe_text(getattr(body, 'name', None)),
        'object_type': _object_type(body),
        'is_visible': None,
        'is_solid': None,
        'volume_cm3': None,
        'area_cm2': None,
    }

    for attr_name, key in [
        ('isVisible', 'is_visible'),
        ('isSolid', 'is_solid'),
        ('volume', 'volume_cm3'),
        ('area', 'area_cm2'),
    ]:
        try:
            data[key] = getattr(body, attr_name)
        except:
            pass

    return data


def _component_summary(component):
    data = {
        'name': _safe_text(getattr(component, 'name', None)),
        'object_type': _object_type(component),
        'bodies_count': None,
        'occurrences_count': None,
        'joints_count': None,
        'as_built_joints_count': None,
        'bodies': [],
    }

    try:
        data['bodies_count'] = component.bRepBodies.count
        data['bodies'] = [_body_summary(body) for body in _iter_collection(component.bRepBodies)]
    except:
        pass

    try:
        data['occurrences_count'] = component.occurrences.count
    except:
        pass

    try:
        data['joints_count'] = component.joints.count
    except:
        pass

    try:
        data['as_built_joints_count'] = component.asBuiltJoints.count
    except:
        pass

    return data


def _physical_properties(occurrence):
    try:
        props = occurrence.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
    except Exception as exc:
        return {'error': _safe_text(exc)}

    data = {
        'mass_kg': _safe_number(getattr(props, 'mass', None)),
        'area_cm2': _safe_number(getattr(props, 'area', None)),
        'volume_cm3': _safe_number(getattr(props, 'volume', None)),
        'center_of_mass_cm': _as_array(getattr(props, 'centerOfMass', None)),
        'xyz_moments_of_inertia': None,
    }

    try:
        result = props.getXYZMomentsOfInertia()
        data['xyz_moments_of_inertia'] = [_safe_number(v) for v in result]
    except:
        pass

    return data


def _occurrence_summary(occurrence, path, depth):
    data = {
        'path': path,
        'depth': depth,
        'name': _safe_text(getattr(occurrence, 'name', None)),
        'full_path_name': None,
        'object_type': _object_type(occurrence),
        'component_name': None,
        'is_referenced_component': None,
        'is_grounded': None,
        'is_light_bulb_on': None,
        'is_visible': None,
        'transform': None,
        'bodies_count': None,
        'child_occurrences_count': None,
        'physical_properties': None,
        'component': None,
        'children': [],
    }

    for attr_name, key in [
        ('fullPathName', 'full_path_name'),
        ('isReferencedComponent', 'is_referenced_component'),
        ('isGrounded', 'is_grounded'),
        ('isLightBulbOn', 'is_light_bulb_on'),
        ('isVisible', 'is_visible'),
    ]:
        try:
            data[key] = getattr(occurrence, attr_name)
        except:
            pass

    try:
        data['transform'] = _matrix(occurrence.transform)
    except:
        pass

    try:
        data['component_name'] = occurrence.component.name
        data['component'] = _component_summary(occurrence.component)
    except:
        pass

    try:
        data['bodies_count'] = occurrence.bRepBodies.count
    except:
        pass

    try:
        data['child_occurrences_count'] = occurrence.childOccurrences.count
    except:
        pass

    data['physical_properties'] = _physical_properties(occurrence)

    try:
        for child in _iter_collection(occurrence.childOccurrences):
            child_path = path + '/' + child.name
            data['children'].append(_occurrence_summary(child, child_path, depth + 1))
    except:
        pass

    return data


def _flat_occurrences_from_tree(node):
    items = [dict((k, v) for k, v in node.items() if k != 'children')]
    for child in node.get('children', []):
        items.extend(_flat_occurrences_from_tree(child))
    return items


def _joint_type_name(joint_motion):
    try:
        index = joint_motion.jointType
        if 0 <= index < len(JOINT_TYPES):
            return JOINT_TYPES[index]
        return 'unknown({})'.format(index)
    except:
        return None


def _joint_occurrence(occurrence):
    if not occurrence:
        return None

    data = {
        'name': _safe_text(getattr(occurrence, 'name', None)),
        'full_path_name': None,
        'component_name': None,
        'transform': None,
    }

    try:
        data['full_path_name'] = occurrence.fullPathName
    except:
        pass

    try:
        data['component_name'] = occurrence.component.name
    except:
        pass

    try:
        data['transform'] = _matrix(occurrence.transform)
    except:
        pass

    return data


def _geometry_or_origin(value):
    if not value:
        return None

    data = {'object_type': _object_type(value), 'origin_cm': None, 'geometry_origin_cm': None}

    try:
        data['origin_cm'] = _as_array(value.origin)
    except:
        pass

    try:
        data['geometry_origin_cm'] = _as_array(value.geometry.origin)
    except:
        pass

    return data


def _limits(limits):
    if not limits:
        return None

    data = {}
    for attr_name in [
        'isMaximumValueEnabled',
        'isMinimumValueEnabled',
        'maximumValue',
        'minimumValue',
    ]:
        try:
            data[attr_name] = getattr(limits, attr_name)
        except:
            pass
    return data


def _joint_motion(motion):
    if not motion:
        return None

    data = {
        'object_type': _object_type(motion),
        'joint_type_index': None,
        'joint_type_name': _joint_type_name(motion),
        'rotation_axis_vector': None,
        'slide_direction_vector': None,
        'rotation_limits': None,
        'slide_limits': None,
    }

    try:
        data['joint_type_index'] = motion.jointType
    except:
        pass

    try:
        data['rotation_axis_vector'] = _as_array(motion.rotationAxisVector)
    except:
        pass

    try:
        data['slide_direction_vector'] = _as_array(motion.slideDirectionVector)
    except:
        pass

    try:
        data['rotation_limits'] = _limits(motion.rotationLimits)
    except:
        pass

    try:
        data['slide_limits'] = _limits(motion.slideLimits)
    except:
        pass

    return data


def _joint_summary(joint, owner_component_name):
    data = {
        'owner_component_name': owner_component_name,
        'name': _safe_text(getattr(joint, 'name', None)),
        'object_type': _object_type(joint),
        'is_suppressed': None,
        'occurrence_one': None,
        'occurrence_two': None,
        'geometry_or_origin_one': None,
        'geometry_or_origin_two': None,
        'joint_motion': None,
    }

    try:
        data['is_suppressed'] = joint.isSuppressed
    except:
        pass

    try:
        data['occurrence_one'] = _joint_occurrence(joint.occurrenceOne)
    except:
        pass

    try:
        data['occurrence_two'] = _joint_occurrence(joint.occurrenceTwo)
    except:
        pass

    try:
        data['geometry_or_origin_one'] = _geometry_or_origin(joint.geometryOrOriginOne)
    except:
        pass

    try:
        data['geometry_or_origin_two'] = _geometry_or_origin(joint.geometryOrOriginTwo)
    except:
        pass

    try:
        data['joint_motion'] = _joint_motion(joint.jointMotion)
    except:
        pass

    return data


def _as_built_joint_summary(joint, owner_component_name):
    data = {
        'owner_component_name': owner_component_name,
        'name': _safe_text(getattr(joint, 'name', None)),
        'object_type': _object_type(joint),
        'is_suppressed': None,
        'occurrence_one': None,
        'occurrence_two': None,
        'joint_motion': None,
    }

    for attr_name, key in [
        ('isSuppressed', 'is_suppressed'),
        ('occurrenceOne', 'occurrence_one'),
        ('occurrenceTwo', 'occurrence_two'),
    ]:
        try:
            value = getattr(joint, attr_name)
            data[key] = _joint_occurrence(value) if key.startswith('occurrence') else value
        except:
            pass

    try:
        data['joint_motion'] = _joint_motion(joint.jointMotion)
    except:
        pass

    return data


def _all_joints(design):
    joints = []
    as_built_joints = []

    for component in _iter_collection(design.allComponents):
        component_name = _safe_text(getattr(component, 'name', None))

        try:
            for joint in _iter_collection(component.joints):
                joints.append(_joint_summary(joint, component_name))
        except:
            pass

        try:
            for joint in _iter_collection(component.asBuiltJoints):
                as_built_joints.append(_as_built_joint_summary(joint, component_name))
        except:
            pass

    return joints, as_built_joints


def _save_viewport(save_dir):
    try:
        app = adsk.core.Application.get()
        viewport = app.activeViewport
        path = os.path.join(save_dir, 'fusion_view.png')
        viewport.saveAsImageFile(path, 1800, 1200)
        return path
    except Exception as exc:
        return {'error': _safe_text(exc)}


def _folder_dialog(ui):
    folder_dlg = ui.createFolderDialog()
    folder_dlg.title = 'Fusion2URDF Diagnostics Output Folder'
    result = folder_dlg.showDialog()
    if result == adsk.core.DialogResults.DialogOK:
        return folder_dlg.folder
    return None


def _make_output_dir(parent_dir, design_name):
    safe_name = ''.join(c if c.isalnum() or c in ('-', '_') else '_' for c in design_name)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join(parent_dir, safe_name + '_fusion2urdf_diagnostics_' + timestamp)
    os.makedirs(output_dir)
    return output_dir


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF Diagnostics'

        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        parent_dir = _folder_dialog(ui)
        if not parent_dir:
            ui.messageBox('Diagnostics canceled', title)
            return

        root = design.rootComponent
        output_dir = _make_output_dir(parent_dir, root.name)

        occurrence_tree = []
        for occurrence in _iter_collection(root.occurrences):
            occurrence_tree.append(_occurrence_summary(occurrence, occurrence.name, 0))

        flat_occurrences = []
        for node in occurrence_tree:
            flat_occurrences.extend(_flat_occurrences_from_tree(node))

        joints, as_built_joints = _all_joints(design)

        data = {
            'generated_at': datetime.now().isoformat(),
            'fusion': {
                'version': _safe_text(getattr(app, 'version', None)),
                'active_document_name': _safe_text(getattr(app.activeDocument, 'name', None)),
            },
            'design': {
                'root_component_name': root.name,
                'all_components_count': _collection_count(design.allComponents),
                'root_occurrences_count': _collection_count(root.occurrences),
                'root_joints_count': _collection_count(root.joints),
                'root_as_built_joints_count': _collection_count(root.asBuiltJoints),
            },
            'components': [_component_summary(component) for component in _iter_collection(design.allComponents)],
            'occurrence_tree': occurrence_tree,
            'flat_occurrences': flat_occurrences,
            'joints': joints,
            'as_built_joints': as_built_joints,
            'viewport_image': _save_viewport(output_dir),
        }

        json_path = os.path.join(output_dir, 'fusion2urdf_diagnostics.json')
        with open(json_path, mode='w') as f:
            json.dump(data, f, indent=2)

        summary_path = os.path.join(output_dir, 'summary.txt')
        with open(summary_path, mode='w') as f:
            f.write('Fusion2URDF diagnostics\n')
            f.write('Root component: {}\n'.format(root.name))
            f.write('Components: {}\n'.format(data['design']['all_components_count']))
            f.write('Root occurrences: {}\n'.format(data['design']['root_occurrences_count']))
            f.write('Flat occurrences: {}\n'.format(len(flat_occurrences)))
            f.write('Joints: {}\n'.format(len(joints)))
            f.write('As-built joints: {}\n'.format(len(as_built_joints)))
            f.write('JSON: {}\n'.format(json_path))

        ui.messageBox('Diagnostics written to:\n{}'.format(output_dir), title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
