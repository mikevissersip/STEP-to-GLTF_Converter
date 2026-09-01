# STEP to glTF converter

This CLI converts `.stp`/`.step` files to a meter-scaled glTF 2.0 file and a
same-name external `.bin` buffer. STEP surface/general colors are preserved as
glTF PBR base colors.
Each STEP component is exported as its own glTF mesh and scene node, so Blender imports
the parts as separate objects that can be moved and edited independently.

## Setup

Use Python 3.10-3.12 in a virtual environment, then install the Open Cascade
runtime:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## Convert

```powershell
python .\step_to_gltf.py ".\STEP\1012.000(1).stp" .\GLTF\converted.gltf
```

The default tessellation tolerance is `0.1` in the STEP source units. Lower it
for more detail and a larger buffer; raise it for a smaller buffer.