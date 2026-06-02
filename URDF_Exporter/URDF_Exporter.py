#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
from .utils import utils
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg
    export_state = None
    
    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 
        link_occurrences = utils.collect_link_occurrences(root)
        materials_dict = utils.collect_link_materials(link_occurrences)

        # set the names        
        robot_name = utils.sanitize_name(root.name.split()[0])
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        export_settings = utils.prompt_export_settings(ui)
        if not export_settings:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        save_dir = utils.make_unique_export_dir(save_dir, robot_name)
        
        # --------------------
        # set dictionaries
        
        # Generate joints_dict. All joints are related to root. 
        joints_dict, msg = Joint.make_joints_dict(root, msg, export_settings)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0   
        
        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0
        
        links_xyz_dict = {}
        visual_meshes = utils.gripper_visual_meshes(link_occurrences, visual_mesh_extension='obj')

        # --------------------
        # Generate URDF
        Write.write_browser_urdf(joints_dict, links_xyz_dict, inertial_dict, robot_name, save_dir, export_settings, materials_dict, visual_meshes)

        # Generate STL files directly from top-level link occurrences. This keeps
        # nested CAD components inside their containing robot link and avoids
        # mutating the Fusion design for export.
        utils.export_stl_links(design, save_dir, link_occurrences)
        utils.export_obj_links(design, save_dir, link_occurrences, visual_meshes)
        utils.write_gripper_visual_debug(save_dir, visual_meshes)
        
        ui.messageBox(msg + '\n\nProfile: ' + export_settings['profile'] + '\n' + save_dir, title)
        
    except:
        if export_state:
            try:
                utils.restore_occs(export_state)
            except:
                pass
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
