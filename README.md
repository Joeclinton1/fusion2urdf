# Fusion2URDF — flexible component exporter

Turn a Fusion 360 assembly into a clean, browser-ready URDF without flattening your CAD model first.

This fork builds on [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf) and is aimed at robots whose links contain real subassemblies: bearings, brackets, cameras, gripper parts, electronics, and other nested components. Keep that useful structure in Fusion; the exporter treats each root-level occurrence as one URDF link and includes everything nested beneath it.

## What changed from the original

| Original exporter | This fork |
| --- | --- |
| A link component had to contain bodies only | A root-level link can contain nested components and bodies |
| Nested assemblies could produce missing or incorrect links | Nested content is collapsed into its containing root-level URDF link |
| Export focused on a ROS 1/Xacro package | Export produces a compact plain-URDF folder with relative mesh paths |
| STL meshes were used for the model | OBJ/MTL visuals preserve Fusion colours; STL meshes remain available for collision geometry |
| Limited help when Fusion's component or joint structure was unexpected | An optional read-only diagnostics script previews link mapping and reports skipped joints |
| One model orientation | `tabletop`, `humanoid-left`, and `humanoid-right` export profiles are available |
| Fusion joint limits were copied directly | Positive-only revolute ranges can be centred automatically, with per-joint overrides in a settings file |
| Older directory utilities could fail on current Fusion Python versions | File handling works with Fusion's Python 3.12 environment and avoids overwriting earlier exports |

The exporter also:

- leaves the Fusion design structure alone during export;
- calculates mass, centre of mass, and inertia for each collapsed link;
- maps joints found anywhere in the design back to their root-level links;
- ignores joints internal to a collapsed link and duplicate edges;
- converts Fusion slider limits from centimetres to URDF metres;
- supports an optional `gripper` link, including separate `PincOpen` and camera visual meshes when those child names are present;
- creates a new timestamped output directory when an export with the same name already exists.

## Install

Download or clone this repository, then copy `URDF_Exporter` into Fusion 360's scripts directory. Copy `Fusion2URDF_Diagnostics` as well if you want the troubleshooting tool.

### Windows PowerShell

```powershell
$fusionScripts = Join-Path $env:APPDATA "Autodesk\Autodesk Fusion 360\API\Scripts"
Copy-Item ".\URDF_Exporter" -Destination $fusionScripts -Recurse -Force
Copy-Item ".\Fusion2URDF_Diagnostics" -Destination $fusionScripts -Recurse -Force
```

### macOS

```bash
fusion_scripts="$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts"
cp -R ./URDF_Exporter "$fusion_scripts/"
cp -R ./Fusion2URDF_Diagnostics "$fusion_scripts/"
```

Restart Fusion 360, or refresh the **Scripts** tab in **Scripts and Add-Ins**, after installing or updating the folders.

## Prepare the Fusion model

The root of the design should contain one occurrence per moving robot link. Nested components belong inside those occurrences and are welcome.

```text
Robot design root
├── base_link
│   ├── chassis
│   └── electronics
├── link1
│   ├── arm casting
│   └── bearings
├── link2
│   └── wrist assembly
└── gripper                 optional
    ├── PincOpen            optional special visual
    └── camera              optional special visual
```

Use these rules:

1. Name the first root-level link `base_link`. `link0` is also accepted and is exported as `base_link`.
2. Name the moving links `link1`, `link2`, and so on, with no gaps in the chain.
3. Put any detailed CAD hierarchy inside the appropriate root-level link. Joints wholly inside one link are intentionally left out of the URDF.
4. Connect neighbouring links with Fusion **Revolute** or **Slider** joints. The exported chain is `base_link → link1 → link2 → …`.
5. Set both minimum and maximum limits on every Slider joint. A Revolute joint with no limits becomes a continuous joint.
6. Keep the model Z axis upright if you want it upright in the exported viewer.
7. If present, name the final root-level tool `gripper`. It is attached to the highest-numbered link with a fixed joint when no suitable joint already exists.

Names are sanitised for URDF use, but the numbered chain names above should be used exactly. Non-adjacent joints, unsupported joint types, duplicate joints between the same exported links, and joints that cannot be mapped to root-level links are skipped.

## Export a URDF

1. Open the design in Fusion 360.
2. Open **Utilities → Add-Ins → Scripts and Add-Ins**.
3. On the **Scripts** tab, select `URDF_Exporter` and click **Run**.
4. Choose the parent folder for the export.
5. Enter one of the export profiles when prompted:

   - `tabletop` keeps `base_link` as the URDF root.
   - `humanoid-left` adds `humanoid_root`, mounts the arm at shoulder height, and rotates it +90° around Z.
   - `humanoid-right` does the same for the right side with a -90° Z rotation.

6. Wait for the success message. It shows the selected profile and the exact output directory.

The robot name comes from the first word of the Fusion root-component name. A typical export looks like this:

```text
my_robot/
├── urdf/
│   └── my_robot.urdf
├── meshes/
│   ├── base_link.obj       coloured visual mesh
│   ├── base_link.mtl
│   ├── base_link.stl       collision mesh
│   ├── link1.obj
│   ├── link1.mtl
│   └── link1.stl
└── gripper_visual_debug.json
```

Open `urdf/my_robot.urdf` in your URDF loader or viewer. Mesh references are relative (`../meshes/...`), so keep the `urdf` and `meshes` directories together. If your application serves assets in a browser, serve the whole export directory rather than opening the URDF as an isolated file.

## Export profiles and joint limits

The last selected profile is remembered. Export settings live here:

- Windows: `%APPDATA%\Fusion2URDF\settings.json`
- macOS: `~/.fusion2urdf/settings.json`

The exporter names chain joints from their links, for example `base_link_to_link1` and `link1_to_link2`. You can override limits by those exported names:

```json
{
  "export_profile": "tabletop",
  "center_positive_joint_limits": true,
  "joint_limits": {
    "base_link_to_link1": {
      "degrees": [-90, 90]
    },
    "link1_to_link2": {
      "radians": [-1.2, 1.2]
    }
  }
}
```

With `center_positive_joint_limits` enabled, a positive-only Fusion revolute range is centred around zero while preserving its total travel. Explicit `joint_limits` entries take priority.

For humanoid profiles, the shoulder height is taken from the model when the arm is already positioned at body height. For a tabletop-modelled arm, the exporter estimates shoulder height from the chain length, falling back to 0.45 m when necessary.

## Diagnose a difficult model

Run `Fusion2URDF_Diagnostics` from Fusion's **Scripts** tab before changing a model just to make it export. The diagnostics script is read-only with respect to the design. Choose an output folder and it creates a timestamped report containing:

- `summary.txt` with link and joint counts;
- `fusion2urdf_diagnostics.json` with the component tree, link mapping, joint endpoints, included joints, and skip reasons;
- `fusion_view.png` showing the active viewport at the time of the report.

The most useful section is `exporter_preview`. Check that it finds exactly one `base_link`, assigns the expected root occurrences to `link1`, `link2`, and so on, and includes one joint for every adjacent pair.

## Current boundaries

- The generated file is plain URDF, not the ROS 1 description package generated by the original project.
- The automatic kinematic layout is a single numbered chain with an optional fixed gripper. Branched trees and closed kinematic loops need manual URDF work after export.
- Revolute and Slider joints are exported. Other Fusion joint types are skipped.
- Each normal link gets one visual material based on the first usable Fusion appearance found in that occurrence tree.
- The `PincOpen` and camera split-visual behaviour is name-based and specific to a `gripper` root-level link.

## Credits and citation

This project remains rooted in Toshinori Kitamura's original Fusion2URDF work. If you use it in academic work, cite the original project:

```bibtex
@misc{toshinori2020fusion2urdf,
  author       = {Toshinori Kitamura},
  title        = {Fusion2URDF},
  year         = {2020},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/syuntoku14/fusion2urdf}}
}
```
