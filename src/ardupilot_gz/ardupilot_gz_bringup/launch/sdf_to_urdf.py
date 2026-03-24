#!/usr/bin/env python3
"""Convert Gazebo SDF model to URDF for RViz display."""

import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def resolve_sdf_includes(sdf_content, sdf_dir):
    """Resolve package:// and model:// URIs in SDF to actual file paths."""
    import re
    
    def replace_uri(match):
        uri = match.group(1)
        
        # Handle package:// URIs
        if uri.startswith('package://'):
            pkg_name, path = uri[10:].split('/', 1)
            try:
                pkg_dir = get_package_share_directory(pkg_name)
                return f'{pkg_dir}/{path}'
            except:
                print(f"Warning: Could not resolve package {pkg_name}")
                return uri
        
        # Handle model:// URIs (Gazebo models)
        elif uri.startswith('model://'):
            model_name = uri[8:].rstrip('/')
            gazebo_pkg = get_package_share_directory('ardupilot_gazebo')
            model_path = f'{gazebo_pkg}/models/{model_name}'
            if os.path.exists(model_path):
                return model_path
            else:
                print(f"Warning: Could not find model {model_name}")
                return uri
        
        return uri
    
    # Replace <uri> tags
    sdf_content = re.sub(
        r'<uri>(.*?)</uri>',
        lambda m: f'<uri>{replace_uri(m)}</uri>',
        sdf_content
    )
    return sdf_content

def expand_sdf_includes(sdf_content, base_dir):
    """Expand <include> tags in SDF by reading referenced files."""
    import xml.etree.ElementTree as ET
    import re
    
    # Parse the SDF
    try:
        root = ET.fromstring(sdf_content)
    except:
        print("Error parsing SDF")
        return sdf_content
    
    # Find all include elements
    includes = root.findall('.//include')
    
    for include in includes:
        uri_elem = include.find('uri')
        if uri_elem is not None and uri_elem.text:
            uri = uri_elem.text.strip()
            
            # Check if it's a file path (after resolution)
            if os.path.isdir(uri):
                model_sdf = os.path.join(uri, 'model.sdf')
                if os.path.exists(model_sdf):
                    try:
                        with open(model_sdf, 'r') as f:
                            included_content = f.read()
                        
                        # Recursively expand includes in the included file
                        included_content = expand_sdf_includes(included_content, uri)
                        
                        # Parse the included SDF and extract model/link elements
                        included_root = ET.fromstring(included_content)
                        model_elem = included_root.find('.//model')
                        
                        if model_elem is not None:
                            # Get the pose from include if specified
                            pose_elem = include.find('pose')
                            if pose_elem is not None and pose_elem.text:
                                # Update the included model's pose
                                model_pose = model_elem.find('pose')
                                if model_pose is None:
                                    pose = ET.SubElement(model_elem, 'pose')
                                    pose.text = pose_elem.text
                                # Note: In a real implementation, we'd merge poses
                            
                            # Add the included model's children to parent
                            parent = include.getparent() if hasattr(include, 'getparent') else root
                            
                            # For simplicity, extract links and joints from included model
                            idx = list(root).index(include)
                            for link in model_elem.findall('link'):
                                root.insert(idx, link)
                                idx += 1
                            for joint in model_elem.findall('joint'):
                                root.insert(idx, joint)
                                idx += 1
                            
                            # Remove the include element
                            root.remove(include)
                    except Exception as e:
                        print(f"Error processing include {uri}: {e}")
    
    return ET.tostring(root, encoding='unicode')

def sdf_to_urdf(sdf_file):
    """Convert SDF model to URDF using sdformat_urdf."""
    try:
        from sdformat_urdf.sdformat_urdf import convert
    except ImportError:
        print("Error: sdformat_urdf not available. Installing...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "sdformat_urdf"])
        from sdformat_urdf.sdformat_urdf import convert
    
    # Read and resolve the SDF file
    sdf_dir = os.path.dirname(sdf_file)
    with open(sdf_file, 'r') as f:
        sdf_content = f.read()
    
    # Resolve URIs
    sdf_content = resolve_sdf_includes(sdf_content, sdf_dir)
    
    # Try to expand includes
    try:
        sdf_content = expand_sdf_includes(sdf_content, sdf_dir)
    except Exception as e:
        print(f"Warning: Could not expand includes: {e}")
    
    # Write resolved SDF to temp file
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
        f.write(sdf_content)
        temp_sdf = f.name
    
    try:
        # Convert SDF to URDF
        urdf_content = convert(temp_sdf)
        return urdf_content
    except Exception as e:
        print(f"Error converting SDF to URDF: {e}")
        raise
    finally:
        os.unlink(temp_sdf)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: sdf_to_urdf.py <sdf_file>")
        sys.exit(1)
    
    sdf_file = sys.argv[1]
    urdf_content = sdf_to_urdf(sdf_file)
    print(urdf_content)
