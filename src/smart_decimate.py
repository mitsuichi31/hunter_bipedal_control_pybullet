#!/usr/bin/env python3
"""
Smart mesh decimation using quadric edge collapse algorithm

This preserves edges, features, and visual quality much better than simple decimation.
Requires: pip install trimesh
"""

import os
import sys

try:
    import trimesh
    import numpy as np
except ImportError:
    print("ERROR: trimesh not installed")
    print("\nInstall with:")
    print("  pip install trimesh")
    sys.exit(1)

def smart_decimate(mesh_file, target_faces, output_file=None):
    """
    Apply quadric edge collapse decimation

    This algorithm:
    - Preserves sharp edges
    - Maintains surface curvature
    - Smartly places simplified vertices
    - Much better quality than simple decimation
    """
    if output_file is None:
        output_file = mesh_file

    print(f"\nProcessing: {os.path.basename(mesh_file)}")

    # Load mesh
    mesh = trimesh.load(mesh_file)
    original_faces = len(mesh.faces)

    print(f"  Original: {original_faces:,} faces")
    print(f"  Target: {target_faces:,} faces")

    # Calculate what fraction to keep (not reduce)
    # trimesh wants the fraction to KEEP, not reduce
    percent_to_keep = target_faces / original_faces

    # Apply quadric decimation with fraction
    decimated = mesh.simplify_quadric_decimation(percent_to_keep)

    final_faces = len(decimated.faces)
    reduction = (1 - final_faces/original_faces) * 100

    print(f"  Result: {final_faces:,} faces ({reduction:.1f}% reduction)")

    # Export
    decimated.export(output_file)
    print(f"  ✓ Saved to: {output_file}")

    return decimated

def main():
    mesh_dir = os.path.join(os.path.dirname(__file__), '../models/meshes')

    print("="*80)
    print("SMART MESH DECIMATION (Quadric Edge Collapse)")
    print("="*80)
    print("\nThis uses the same algorithm as MeshLab's Quadric Edge Collapse")
    print("Much better quality than simple triangle skipping!\n")

    # First, restore from backups
    print("Step 1: Restoring from backups...")
    import shutil

    meshes_to_process = [
        'leg_r2_link.STL',
        'leg_r3_link.STL',
        'leg_r4_link.STL',
        'leg_l2_link.STL',
        'leg_l3_link.STL',
        'leg_l4_link.STL',
    ]

    for mesh in meshes_to_process:
        backup = os.path.join(mesh_dir, mesh + '.backup')
        target = os.path.join(mesh_dir, mesh)
        if os.path.exists(backup):
            shutil.copy2(backup, target)
            print(f"  ✓ Restored {mesh}")

    print("\nStep 2: Applying smart decimation...")

    # More aggressive targets to ensure rendering works
    # But using smart algorithm for better quality
    decimation_targets = [
        ('leg_r2_link.STL', 80000),   # 200k → 80k (60% reduction)
        ('leg_r3_link.STL', 90000),   # 225k → 90k (60% reduction)
        ('leg_r4_link.STL', 40000),   # 57k → 40k (30% reduction)
        ('leg_l2_link.STL', 80000),
        ('leg_l3_link.STL', 90000),
        ('leg_l4_link.STL', 40000),
    ]

    for mesh_file, target_faces in decimation_targets:
        mesh_path = os.path.join(mesh_dir, mesh_file)
        if os.path.exists(mesh_path):
            try:
                smart_decimate(mesh_path, target_faces)
            except Exception as e:
                print(f"  ✗ Error: {e}")

    print("\n" + "="*80)
    print("SMART DECIMATION COMPLETE!")
    print("="*80)
    print("\nTest the URDF now - quality should be much better!")
    print("\nIf quality is still too low, try increasing target face counts:")
    print("  - leg_r2: 100000 (instead of 80000)")
    print("  - leg_r3: 110000 (instead of 90000)")
    print("\nIf legs don't render, decrease target face counts:")
    print("  - leg_r2: 60000 (instead of 80000)")
    print("  - leg_r3: 70000 (instead of 90000)")

if __name__ == "__main__":
    main()
