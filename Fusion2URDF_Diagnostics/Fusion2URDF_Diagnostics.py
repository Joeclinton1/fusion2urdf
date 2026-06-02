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


def _sanitize_name(name):
    if name is None:
        return None
    return ''.join('_' if c in ' :()' else c for c in str(name))


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


def _top_occurrence_name(full_path_name):
    if not full_path_name:
        return None
    return full_path_name.split('+')[0]


def _link_index(link_name):
    if link_name == 'base_link':
        return 0

    if not link_name or not link_name.startswith('link'):
        return None

    try:
        return int(link_name[4:])
    except:
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


def _last_numbered_link(link_map):
    last_link = None
    last_index = None

    for link in link_map:
        index = _link_index(link.get('link_name'))
        if index is None:
            continue
        if last_index is None or index > last_index:
            last_index = index
            last_link = link.get('link_name')

    return last_link


def _has_link(link_map, link_name):
    for link in link_map:
        if link.get('link_name') == link_name:
            return True
    return False


def _build_link_map(root_occurrences):
    links = []
    used_names = set()

    for occurrence in root_occurrences:
        component_name = _sanitize_name(occurrence.get('component_name'))
        occurrence_name = occurrence.get('name')
        occurrence_base_name = _sanitize_name(occurrence_name.split(':')[0] if occurrence_name else None)

        if component_name in ('base_link', 'link0') or occurrence_base_name in ('base_link', 'link0'):
            link_name = 'base_link'
            base_reason = 'component_or_occurrence_named_base_link_or_link0'
        else:
            link_name = component_name
            base_reason = None

        original_link_name = link_name
        suffix = 2
        while link_name in used_names:
            link_name = original_link_name + '_' + str(suffix)
            suffix += 1

        used_names.add(link_name)
        links.append({
            'occurrence_name': occurrence_name,
            'component_name': occurrence.get('component_name'),
            'path': occurrence.get('path'),
            'link_name': link_name,
            'is_base_link': link_name == 'base_link',
            'base_reason': base_reason,
            'mass_kg': occurrence.get('physical_properties', {}).get('mass_kg'),
            'center_of_mass_cm': occurrence.get('physical_properties', {}).get('center_of_mass_cm'),
            'bodies_count': occurrence.get('bodies_count'),
            'child_occurrences_count': occurrence.get('child_occurrences_count'),
            'is_referenced_component': occurrence.get('is_referenced_component'),
        })

    return links


def _link_for_full_path(full_path_name, link_map):
    top_name = _top_occurrence_name(full_path_name)
    if not top_name:
        return None

    for link in link_map:
        if link.get('occurrence_name') == top_name:
            return link.get('link_name')

    return None


def _name_collisions(items, field):
    buckets = {}
    for item in items:
        value = item.get(field)
        if not value:
            continue
        buckets.setdefault(value, []).append(item)

    return [
        {'value': value, 'count': len(matches), 'items': matches}
        for value, matches in buckets.items()
        if len(matches) > 1
    ]


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


def _collapsed_joint_summary(joint_summary, link_map):
    occurrence_one_path = None
    occurrence_two_path = None

    try:
        occurrence_one_path = joint_summary['occurrence_one']['full_path_name']
    except:
        pass

    try:
        occurrence_two_path = joint_summary['occurrence_two']['full_path_name']
    except:
        pass

    child = _link_for_full_path(occurrence_one_path, link_map)
    parent = _link_for_full_path(occurrence_two_path, link_map)
    reason = None
    inferred_reason = None
    included = True
    joint_type = joint_summary.get('joint_motion', {}).get('joint_type_name') if joint_summary.get('joint_motion') else None

    if joint_type != 'revolute':
        return {
            'source_name': joint_summary.get('name'),
            'owner_component_name': joint_summary.get('owner_component_name'),
            'type': joint_type,
            'child_link': child,
            'parent_link': parent,
            'included_by_exporter': False,
            'skip_reason': 'ignored non-revolute joint',
            'inferred_reason': None,
            'occurrence_one_full_path': occurrence_one_path,
            'occurrence_two_full_path': occurrence_two_path,
            'axis': joint_summary.get('joint_motion', {}).get('rotation_axis_vector') if joint_summary.get('joint_motion') else None,
            'slide_axis': joint_summary.get('joint_motion', {}).get('slide_direction_vector') if joint_summary.get('joint_motion') else None,
            'geometry_or_origin_one': joint_summary.get('geometry_or_origin_one'),
            'geometry_or_origin_two': joint_summary.get('geometry_or_origin_two'),
            'rotation_limits': joint_summary.get('joint_motion', {}).get('rotation_limits') if joint_summary.get('joint_motion') else None,
            'slide_limits': joint_summary.get('joint_motion', {}).get('slide_limits') if joint_summary.get('joint_motion') else None,
        }

    if not parent and child:
        inferred_parent = _previous_link_name(child)
        if inferred_parent:
            parent = inferred_parent
            inferred_reason = 'inferred parent from numbered child link'

    if parent == child:
        owner_link = _sanitize_name(joint_summary.get('owner_component_name'))
        if owner_link == child:
            inferred_parent = _previous_link_name(child)
            if inferred_parent:
                parent = inferred_parent
                inferred_reason = 'inferred parent from internal numbered-link revolute'

    if parent and child:
        parent, child = _normalize_parent_child(parent, child)

    if not child or not parent:
        included = False
        reason = 'could_not_map_endpoint_to_top_level_link'
    elif child == parent:
        included = False
        reason = 'internal_to_collapsed_link'

    return {
        'source_name': joint_summary.get('name'),
        'owner_component_name': joint_summary.get('owner_component_name'),
        'type': joint_type,
        'child_link': child,
        'parent_link': parent,
        'included_by_exporter': included,
        'skip_reason': reason,
        'inferred_reason': inferred_reason,
        'occurrence_one_full_path': occurrence_one_path,
        'occurrence_two_full_path': occurrence_two_path,
        'axis': joint_summary.get('joint_motion', {}).get('rotation_axis_vector') if joint_summary.get('joint_motion') else None,
        'slide_axis': joint_summary.get('joint_motion', {}).get('slide_direction_vector') if joint_summary.get('joint_motion') else None,
        'geometry_or_origin_one': joint_summary.get('geometry_or_origin_one'),
        'geometry_or_origin_two': joint_summary.get('geometry_or_origin_two'),
        'rotation_limits': joint_summary.get('joint_motion', {}).get('rotation_limits') if joint_summary.get('joint_motion') else None,
        'slide_limits': joint_summary.get('joint_motion', {}).get('slide_limits') if joint_summary.get('joint_motion') else None,
    }


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
        link_map = _build_link_map([dict((k, v) for k, v in node.items() if k != 'children') for node in occurrence_tree])
        collapsed_joints = [_collapsed_joint_summary(joint, link_map) for joint in joints]
        used_edges = set()
        for joint in collapsed_joints:
            if not joint['included_by_exporter']:
                continue

            edge = (joint['parent_link'], joint['child_link'])
            if edge in used_edges:
                joint['included_by_exporter'] = False
                joint['skip_reason'] = 'duplicate collapsed top-level joint {} -> {}'.format(edge[0], edge[1])
                continue

            used_edges.add(edge)

        if _has_link(link_map, 'gripper'):
            parent = _last_numbered_link(link_map)
            edge = (parent, 'gripper')
            if parent and edge not in used_edges:
                collapsed_joints.append({
                    'source_name': 'synthetic_fixed_' + parent + '_to_gripper',
                    'owner_component_name': None,
                    'type': 'fixed',
                    'child_link': 'gripper',
                    'parent_link': parent,
                    'included_by_exporter': True,
                    'skip_reason': None,
                    'inferred_reason': 'synthetic fixed joint from last numbered link to gripper',
                    'occurrence_one_full_path': None,
                    'occurrence_two_full_path': None,
                    'axis': [0, 0, 0],
                    'slide_axis': None,
                    'geometry_or_origin_one': None,
                    'geometry_or_origin_two': None,
                    'rotation_limits': None,
                    'slide_limits': None,
                })
                used_edges.add(edge)

        included_collapsed_joints = [joint for joint in collapsed_joints if joint['included_by_exporter']]
        skipped_collapsed_joints = [joint for joint in collapsed_joints if not joint['included_by_exporter']]

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
            'exporter_preview': {
                'link_map': link_map,
                'base_link_count': len([link for link in link_map if link['is_base_link']]),
                'base_link_candidates': [link for link in link_map if link['is_base_link']],
                'link_name_collisions': _name_collisions(link_map, 'link_name'),
                'component_name_collisions': _name_collisions(link_map, 'component_name'),
                'collapsed_joints': collapsed_joints,
                'included_collapsed_joints': included_collapsed_joints,
                'skipped_collapsed_joints': skipped_collapsed_joints,
            },
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
            f.write('Exporter preview links: {}\n'.format(len(link_map)))
            f.write('Exporter preview base_link count: {}\n'.format(data['exporter_preview']['base_link_count']))
            f.write('Exporter preview included joints: {}\n'.format(len(included_collapsed_joints)))
            f.write('Exporter preview skipped joints: {}\n'.format(len(skipped_collapsed_joints)))
            f.write('JSON: {}\n'.format(json_path))

        ui.messageBox('Diagnostics written to:\n{}'.format(output_dir), title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
