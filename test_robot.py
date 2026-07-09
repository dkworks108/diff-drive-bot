import os
import sys
import xml.etree.ElementTree as ET

# find urdf file
urdf_file = os.path.join(os.path.dirname(__file__), 'urdf', 'robot.urdf')

if not os.path.exists(urdf_file):
    print(f"FAIL: File not found: {urdf_file}")
    sys.exit(1)

try:
    tree = ET.parse(urdf_file)
    root = tree.getroot()
except Exception as e:
    print(f"FAIL: XML parse error: {e}")
    sys.exit(1)

# get links and joints
links = {link.attrib['name']: link for link in root.findall('link')}
joints = {joint.attrib['name']: joint for joint in root.findall('joint')}

print(f"Total links: {len(links)}")
print(f"Total joints: {len(joints)}")

continuous_joints = []
all_joints_valid = True

for name, j in joints.items():
    jtype = j.attrib.get('type')
    parent = j.find('parent').attrib.get('link') if j.find('parent') is not None else None
    child = j.find('child').attrib.get('link') if j.find('child') is not None else None

    print(f" - Joint: {name} | Type: {jtype} | Parent: {parent} -> Child: {child}")

    if parent not in links or child not in links:
        print(f"   ERROR: Joint {name} links missing: parent={parent}, child={child}")
        all_joints_valid = False

    if jtype == 'continuous':
        continuous_joints.append(name)

print(f"Continuous joints: {continuous_joints}")

# check ground clearance / wheel radius alignment
bf_joint = joints.get('base_footprint_joint')
base_z_offset = 0.0
if bf_joint is not None and bf_joint.find('origin') is not None:
    xyz = bf_joint.find('origin').attrib.get('xyz', '0 0 0').split()
    base_z_offset = float(xyz[2])

# get wheel radius
wheel_link = links.get('left_wheel_link')
wheel_radius = 0.0
if wheel_link is not None:
    geom = wheel_link.find('visual/geometry/cylinder')
    if geom is not None:
        wheel_radius = float(geom.attrib.get('radius', 0.0))

print(f"Base link z-offset from ground: {base_z_offset} m")
print(f"Wheel radius: {wheel_radius} m")
wheels_on_ground = abs(base_z_offset - wheel_radius) < 1e-4

if all_joints_valid and len(continuous_joints) == 2 and wheels_on_ground and "base_footprint" in links:
    print("PASS: Diff Drive Robot verification passed! Wheels correctly sit on ground plane.")
    sys.exit(0)
else:
    print("FAIL: Verification failed!")
    sys.exit(1)
