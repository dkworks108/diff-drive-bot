import os
import sys
import xml.etree.ElementTree as ET
import pybullet as p

report_lines = []

def log(msg):
    print(msg)
    report_lines.append(msg)

cur_dir = os.path.dirname(os.path.abspath(__file__))
urdf_file = os.path.join(cur_dir, 'urdf', 'robot.urdf')

log("=== Extra Tests for 03_diff_drive_robot ===")

# parse xml
try:
    root = ET.parse(urdf_file).getroot()
except Exception as e:
    log(f"FAIL: XML parse error: {e}")
    sys.exit(1)

# 1. duplicate links or joints
links = [l.attrib['name'] for l in root.findall('link')]
joints = [j.attrib['name'] for j in root.findall('joint')]

dup_links = len(links) != len(set(links))
dup_joints = len(joints) != len(set(joints))
log(f"Links count: {len(links)} (duplicate: {dup_links})")
log(f"Joints count: {len(joints)} (duplicate: {dup_joints})")

# 2. single root link and reachability
parent_map = {}
children_map = {l: [] for l in links}
for j in root.findall('joint'):
    p_name = j.find('parent').attrib['link']
    c_name = j.find('child').attrib['link']
    parent_map[c_name] = p_name
    if p_name in children_map:
        children_map[p_name].append(c_name)

roots = [l for l in links if l not in parent_map]
log(f"Root link(s): {roots}")

reachable = set()
if len(roots) == 1:
    queue = [roots[0]]
    while queue:
        curr = queue.pop(0)
        reachable.add(curr)
        for child in children_map.get(curr, []):
            if child not in reachable:
                queue.append(child)
log(f"Reachable links: {len(reachable)} / {len(links)}")

# 3. link mass > 0 and positive inertia (base_footprint is allowed as massless root frame)
mass_ok = True
for link in root.findall('link'):
    name = link.attrib['name']
    if name == 'base_footprint':
        continue
    inertial = link.find('inertial')
    if inertial is None:
        log(f"FAIL: Link {name} has no inertial tag")
        mass_ok = False
        continue
    mass_elem = inertial.find('mass')
    inertia_elem = inertial.find('inertia')
    if mass_elem is None or float(mass_elem.attrib.get('value', 0)) <= 0:
        log(f"FAIL: Link {name} mass <= 0")
        mass_ok = False
    if inertia_elem is None:
        log(f"FAIL: Link {name} missing inertia element")
        mass_ok = False
    else:
        ixx = float(inertia_elem.attrib.get('ixx', 0))
        iyy = float(inertia_elem.attrib.get('iyy', 0))
        izz = float(inertia_elem.attrib.get('izz', 0))
        if ixx <= 0 or iyy <= 0 or izz <= 0:
            log(f"FAIL: Link {name} inertia diagonal zero or negative ({ixx}, {iyy}, {izz})")
            mass_ok = False
log(f"All links have mass > 0 and positive inertia: {mass_ok}")

# 4. joint limits and effort/velocity (for revolute/prismatic)
limits_ok = True
for j in root.findall('joint'):
    jtype = j.attrib.get('type')
    jname = j.attrib.get('name')
    if jtype in ['revolute', 'prismatic']:
        lim = j.find('limit')
        if lim is None:
            log(f"FAIL: Joint {jname} missing limit tag")
            limits_ok = False
            continue
        lower = float(lim.attrib.get('lower', 0))
        upper = float(lim.attrib.get('upper', 0))
        effort = float(lim.attrib.get('effort', 0))
        vel = float(lim.attrib.get('velocity', 0))
        if lower >= upper:
            log(f"FAIL: Joint {jname} lower ({lower}) >= upper ({upper})")
            limits_ok = False
        if effort <= 0 or vel <= 0:
            log(f"FAIL: Joint {jname} invalid effort={effort} or velocity={vel}")
            limits_ok = False
log(f"All joint limits, velocity and effort valid: {limits_ok}")

# 5. mesh file existence
meshes_ok = True
for m in root.findall('.//mesh'):
    fn = m.attrib.get('filename', '')
    fn_clean = fn.replace('package://03_diff_drive_robot/', '')
    mesh_path = os.path.join(cur_dir, fn_clean) if not os.path.isabs(fn_clean) else fn_clean
    if not os.path.exists(mesh_path):
        log(f"FAIL: Mesh file missing: {mesh_path}")
        meshes_ok = False
log(f"All mesh files exist: {meshes_ok}")

# 6. forward kinematics sanity check
fk_ok = True
cid = p.connect(p.DIRECT)
rid = p.loadURDF(urdf_file)

# zero pose
for i in range(p.getNumJoints(rid)):
    pos = p.getLinkState(rid, i)[4]
    if any(abs(x) > 10.0 or x != x for x in pos):
        log(f"FAIL: FK position out of bounds at zero pose for link {i}: {pos}")
        fk_ok = False

# mid pose
for i in range(p.getNumJoints(rid)):
    info = p.getJointInfo(rid, i)
    if info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC] and info[8] < info[9]:
        p.resetJointState(rid, i, 0.5 * (info[8] + info[9]))
    elif info[2] == 0: # continuous
        p.resetJointState(rid, i, 0.785)

for i in range(p.getNumJoints(rid)):
    pos = p.getLinkState(rid, i)[4]
    if any(abs(x) > 10.0 or x != x for x in pos):
        log(f"FAIL: FK position out of bounds at mid pose for link {i}: {pos}")
        fk_ok = False

p.disconnect()
log(f"Forward kinematics within valid range (<10m, non-NaN): {fk_ok}")

# 7. project-specific: wheel radius = wheel z offset (wheels touch ground)
bf_joint = next((j for j in root.findall('joint') if j.attrib.get('name') == 'base_footprint_joint'), None)
z_offset = 0.0
if bf_joint is not None and bf_joint.find('origin') is not None:
    z_offset = float(bf_joint.find('origin').attrib.get('xyz', '0 0 0').split()[2])

wheel_link = next((l for l in root.findall('link') if l.attrib.get('name') == 'left_wheel_link'), None)
wheel_radius = 0.0
if wheel_link is not None:
    cyl = wheel_link.find('.//cylinder')
    if cyl is not None:
        wheel_radius = float(cyl.attrib.get('radius', 0))

wheels_on_ground = abs(z_offset - wheel_radius) < 1e-4 and wheel_radius > 0
log(f"Project 03 specific: z_offset={z_offset}, wheel_radius={wheel_radius} -> wheels_on_ground={wheels_on_ground}")

passed = (not dup_links and not dup_joints and len(roots) == 1 and 
          len(reachable) == len(links) and mass_ok and limits_ok and 
          meshes_ok and fk_ok and wheels_on_ground)

if passed:
    log("PASS: 03_diff_drive_robot extra tests passed successfully!")
else:
    log("FAIL: 03_diff_drive_robot extra tests failed!")

# save report
os.makedirs(os.path.join(cur_dir, 'screenshots'), exist_ok=True)
with open(os.path.join(cur_dir, 'screenshots', 'test_report.txt'), 'w') as f:
    f.write('\n'.join(report_lines) + '\n')

sys.exit(0 if passed else 1)
