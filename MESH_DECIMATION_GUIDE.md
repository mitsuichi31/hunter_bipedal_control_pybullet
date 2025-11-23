# Professional Mesh Decimation Guide

## Problem
PyBullet cannot render large STL meshes (200k+ triangles) in batched robot visualizations.

## Solution: Use Professional Decimation Tools

These tools use smart algorithms that preserve edges, features, and visual appearance much better than simple triangle skipping.

---

## Option 1: MeshLab (Recommended)

**MeshLab** is a free, open-source mesh processing tool with excellent decimation.

### Installation
```bash
# Ubuntu/Debian
sudo apt-get install meshlab

# Or download from: https://www.meshlab.net/
```

### Steps to Decimate

1. **Open mesh file:**
   - File → Import Mesh → Select `leg_r2_link.STL`

2. **Apply Quadric Edge Collapse Decimation:**
   - Filters → Remeshing, Simplification and Reconstruction → **Quadric Edge Collapse Decimation**

3. **Set target:**
   - Target number of faces: `80,000` (for 200k triangle mesh)
   - Or: Percentage reduction: `60%` (keeps 40%)
   - ✅ Check "Preserve Topology"
   - ✅ Check "Preserve Boundary"
   - ✅ Check "Optimal position of simplified vertices"
   - Click Apply

4. **Export:**
   - File → Export Mesh As → `leg_r2_link.STL`
   - Format: STL Binary

5. **Repeat for other large meshes:**
   - `leg_r3_link.STL`: Target ~90k faces (from 225k)
   - `leg_r4_link.STL`: Target ~40k faces (from 57k)
   - Same for left leg meshes

---

## Option 2: Blender

**Blender** has a powerful Decimate modifier.

### Steps

1. **Import STL:**
   - File → Import → STL → Select mesh file

2. **Add Decimate Modifier:**
   - Select mesh in outliner
   - Go to Modifiers panel (wrench icon)
   - Add Modifier → Decimate

3. **Configure:**
   - Decimate Type: **Collapse**
   - Ratio: `0.4` (keeps 40% of faces)
   - ✅ Enable "Symmetry" if applicable

4. **Apply and Export:**
   - Click "Apply" on the modifier
   - File → Export → STL
   - Enable "Binary" format
   - Export

---

## Option 3: Python with trimesh (Advanced)

If you want automated processing:

```bash
pip install trimesh numpy-stl
```

```python
import trimesh

# Load mesh
mesh = trimesh.load('leg_r2_link.STL')

# Decimate to target face count
target_faces = 80000
decimated = mesh.simplify_quadric_decimation(target_faces)

# Export
decimated.export('leg_r2_link.STL')
```

---

## Target Triangle Counts

Based on PyBullet rendering limits with ~11 links:

| Mesh File | Original | Target | Reduction |
|-----------|----------|--------|-----------|
| `leg_r2_link.STL` | 200,784 | 80,000 | 60% |
| `leg_r3_link.STL` | 225,086 | 90,000 | 60% |
| `leg_r4_link.STL` | 57,094 | 40,000 | 30% |
| `leg_r5_link.STL` | 6,694 | Keep | 0% |

Apply same reductions to left leg for consistency.

---

## Quality Comparison

**Simple decimation** (current script):
- ❌ Loses detail uniformly
- ❌ Can create holes or artifacts
- ✅ Fast and simple

**Professional tools:**
- ✅ Preserves edges and features
- ✅ Smart vertex placement
- ✅ Maintains surface curvature
- ✅ Better visual appearance

---

## Testing

After decimation, test with:

```bash
cd /workspace/hunter
python3 src/main_simulation.py --mode standing --duration 10
```

**Check:**
- ✅ Both legs appear
- ✅ Visual quality acceptable
- ✅ No rendering lag

If legs disappear again, triangle count is still too high - reduce further.

---

## Backup

Original meshes backed up as:
```
models/meshes/leg_r2_link.STL.backup
models/meshes/leg_r3_link.STL.backup
...
```

To restore:
```bash
cd models/meshes
cp leg_r2_link.STL.backup leg_r2_link.STL
```
