# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 18:07:44 2020

@author: mfocchi
"""
import os
import psutil
#from pinocchio.visualize import GepettoVisualizer
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import sys
from termcolor import colored
import rospkg
import rosparam
import rospy as ros
import rosnode
import roslaunch
import rosgraph
import tf2_ros
from geometry_msgs.msg import TransformStamped
from roslaunch.parent import ROSLaunchParent
import copy
from base_controllers.utils.utils import Utils
import subprocess
import pinocchio
from operator import itemgetter

#from urdf_parser_py.urdf import URDF
#make plot interactive
plt.ion()
plt.close() 

lw_des=7
lw_act=4   
marker_size= 0   

u = Utils()

# globals
labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift", "3 - Elbow", "4 - Wrist 1", "5 - Wrist 2", "6 - Wrist 3"]
labels_quadruped = ["LF_HAA", "LF_HFE","LF_KFE","LH_HAA", "LH_HFE","LH_KFE","RF_HAA", "RF_HFE","RF_KFE","RH_HAA", "RH_HFE","RH_KFE"]
labels_flywheel2 = labels_quadruped + ["left_wheel", "right_wheel"]
labels_flywheel4 = labels_quadruped + ["back_wheel", "front_wheel", "left_wheel", "right_wheel"]

class Twist:
    linear = np.empty((3))*np.nan
    angular = np.empty((3))*np.nan
    def set(self, value):
        self.linear = copy.deepcopy(value[:3])
        self.angular = copy.deepcopy(value[3:])

class Pose:
    position = np.empty((3))*np.nan
    orientation = np.empty((3))*np.nan
    def set(self, value):
        self.position = copy.deepcopy(value[:3])
        self.orientation = copy.deepcopy(value[3:])
            
    
class State:
    
    def __init__(self, desired = False):
        self.pose = Pose()
        self.twist = Twist()
        if (desired):
            self.accel = Twist()
            
    def set(self, value):
        self.pose.set(value.getPose())
        self.twist.set(value.getTwist())
            
    def getPose(self):
        return np.hstack([self.pose.position, self.pose.orientation]) 
            
    def getTwist(self):
        return np.hstack([self.twist.linear, self.twist.angular])
        
def checkRosMaster():
    if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
        print(colored('ROS MASTER is Online','red'))
    else:
        print(colored('ROS MASTER is NOT Online, Starting roscore!!','red'))
        parent = ROSLaunchParent("roscore", [], is_core=True)  # run_id can be any string
        parent.start()

def load_rosparams_from_package(package_name: str, rel_path: str, target_namespace: str = '/'):
    """
    Load a YAML file from a package and upload its parameters to the ROS parameter server.

    :param package_name: name of ROS package (like 'pronto_aliengo')
    :param rel_path: path inside the package to the yaml file (like 'config/aliengo_state_estimator.yaml')
    :param target_namespace: namespace where to upload params (default '/')
    """
    rp = rospkg.RosPack()
    pkg_path = rp.get_path(package_name)
    yaml_path = os.path.join(pkg_path, rel_path)

    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")

    # rosparam.load_file returns a list of (params_dict, namespace) tuples
    loaded = rosparam.load_file(yaml_path)
    if not loaded:
        ros.logwarn(f"No params found in {yaml_path}")
        return

    for params_dict, ns_in_file in loaded:
        # Decide final namespace to upload into:
        # If the YAML sets a namespace, use it; otherwise use the provided target_namespace.
        ns = ns_in_file if ns_in_file not in (None, '') else target_namespace

        # rosparam.upload_params(namespace, params_dict)
        rosparam.upload_params(ns, params_dict)
        ros.loginfo(f"Uploaded {len(params_dict)} params to namespace '{ns}' from {yaml_path}")


def launchFileNode(package,launch_file, additional_args=None):
    launch_file = rospkg.RosPack().get_path(package) + '/launch/'+launch_file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = [launch_file]
    if additional_args is not None:
        cli_args.extend(additional_args)
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()

def launchFileGeneric(launch_file):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

def startNode(package, executable, args='', name=None):
    nodes = rosnode.get_node_names()
    #kill previous instances
    if package in nodes:
        print(colored(f"Killing previous {executable} node","red"))
        os.system("rosnode kill /"+package)
    package = package
    executable = executable
    if name is None:
        name = executable
    namespace = ''
    node = roslaunch.core.Node(package, executable, name, namespace, args=args, output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)


def loadXacro(package_name, model_name, param_name = None, xacro_model_custom_path=None):
    if param_name is None:
        param_name = '/'+model_name+'/robot_description'

    if xacro_model_custom_path is None:
        print(colored(f"Loading xacro for  {model_name} inside {package_name}", "blue"))
        xacro_path = rospkg.RosPack().get_path(package_name) + '/robots/' + model_name + '.urdf.xacro'
    else:
        print(colored(f"Loading xacro  {xacro_model_custom_path}", "blue"))
        xacro_path = rospkg.RosPack().get_path(package_name) + '/'+xacro_model_custom_path

    # generate robot description
    if not os.path.isfile(xacro_path):
        print(colored(f"Xacro file {model_name}.urdf.xacro does not exist!", "red"))
    command_string = "rosrun xacro xacro "+xacro_path

    try:
        robot_description_param = subprocess.check_output(command_string,shell=True,  stderr=subprocess.STDOUT).decode("utf-8") # shell=True is fundamental to load env variables!
    except subprocess.CalledProcessError as process_error:
        ros.logfatal('Failed to run xacro command with error: \n%s', process_error.output)
        sys.exit(1)

    # set param robot_description on param server
    ros.set_param(param_name, robot_description_param)

def spawnModel(package_name, model_name='',  spawn_pos=np.array([0.,0.,0.]), spawn_orient = np.array([0.,0.,0.]) ):
    #loads the xacro of model in the parameter server
    loadXacro(package_name, model_name, param_name=model_name)
    print(colored(f"Spawning {model_name}", "blue"))
    package = 'gazebo_ros'
    executable = 'spawn_model'
    name = model_name
    namespace = '/'
    args = '-urdf -param ' +model_name +' -model ' + model_name +' -x '+ str(spawn_pos[0])+ ' -y ' + str(spawn_pos[1]) +' -z ' + str(spawn_pos[2]) \
           + ' -R ' + str(spawn_orient[0]) + ' -P ' + str(spawn_orient[1]) + ' -Y ' + str(spawn_orient[2])
    node = roslaunch.core.Node(package, executable, name, namespace,args=args,output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

def checkRosControllerRunning(controller = '', robot_name=''):
    cmd = ["rosservice", "call", f"/{robot_name}/controller_manager/list_controllers"]
    result = subprocess.check_output(cmd).decode()
    if controller not in result or "state: \"running\"" not in result:
        return False
    else:
        return True

def spawnMesh(mesh_x, mesh_y, mesh_z, position=np.array([0,0,0]), store_location_mesh="/tmp/", texture_path=None):
    try:
        import meshio
    except ImportError:
        raise RuntimeError("You need to install meshio with: pip install meshio")
    print(colored("Spawning mesh","red"))

    # Build triangles
    n_z = mesh_x.shape[0]
    n_y = mesh_y.shape[0]

    #Normals determine which side of a triangle is "front".
    #RViz and Gazebo render only front-facing surfaces.
    #By default, a triangle's normal is defined by vertex order: counter-clockwise (CCW) is "front".
    #If your mesh:
    # 1) is not a heightfield but rather an arbitrary 3D surface -> trimesh
    # 2) is a heightfield but it has an unstructured set of 2D points (e.g., scattered or irregular) -> use  Delaunay2D matplotlib.tri.Triangulation(x, y) would flatten the grid
    # 3) is a heightfield and you have a Structured meshgrid (you already know how the points are connected)-> you can just build triangles row by row
    triangles = []
    for j in range(n_y - 1):
        for i in range(n_z - 1):
            p1 = j * n_z + i
            p2 = p1 + 1
            p3 = p1 + n_z
            p4 = p3 + 1
            triangles.append([p3, p2, p1])
            triangles.append([p3, p4, p2])

    triangles = np.array(triangles)


    # 3. Prepare data for meshio
    points = np.column_stack((mesh_x.flatten(), mesh_y.flatten(), mesh_z.flatten()))

    #debug/ visualize normals
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot_trisurf(points[:, 0], points[:, 1], points[:, 2], triangles=triangles, cmap='terrain', alpha=0.8)
    # centers = []
    # normals = []
    # for tri in triangles:
    #     p1, p2, p3 = points[tri[0]], points[tri[1]], points[tri[2]]
    #     center = (p1 + p2 + p3) / 3
    #     normal = np.cross(p2 - p1, p3 - p1)
    #     normal /= np.linalg.norm(normal) + 1e-8  # normalize
    #     centers.append(center)
    #     normals.append(normal)
    # centers = np.array(centers)
    # normals = np.array(normals)
    # # Scale normals for visibility
    # normal_length = 0.05
    # ax.quiver(centers[:, 0], centers[:, 1], centers[:, 2],
    #           normals[:, 0], normals[:, 1], normals[:, 2],
    #           length=normal_length, color='red', normalize=True)
    # # Adjust view
    # ax.set_xlabel("X (height)")
    # ax.set_ylabel("Y")
    # ax.set_zlabel("Z")
    # ax.view_init(elev=45, azim=135)
    # plt.title("Mesh with Face Normals")
    # plt.show()

    # Always write STL (collision + fallback visual)
    stl_path = store_location_mesh+"runtime_mesh.stl"
    mesh = meshio.Mesh(points=points, cells=[("triangle", triangles)])
    mesh.write(stl_path)

    # Optionally write textured DAE for RViz
    if texture_path is not None:
        obj_path =  store_location_mesh+"runtime_mesh.obj"
        write_textured_obj(points, triangles, obj_path, texture_path)
        # Use OBJ (textured) for VISUAL
        visual_uri = f"file://{obj_path}"
        material_block = ""  # DO NOT override texture
    else:
        visual_uri = f"file://{stl_path}"
        #use standard reddish material
        material_block = """
                <material>
                  <ambient>0.545 0.271 0.075 1.0</ambient>
                  <diffuse>0.545 0.271 0.075 1.0</diffuse>
                  <specular>0.1 0.1 0.1 1.0</specular>
                  <emissive>0.4 0.2 0.1 1.0</emissive>
                </material>
        """

    # === Step 3: Spawn in Gazebo ===
    sdf_template = f"""
    <sdf version="1.6">
      <model name="runtime_mesh">
        <static>true</static>
        <link name="link">
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>file://{visual_uri}</uri>
              </mesh>
            </geometry>
            {material_block}
          </visual>
          <collision name="collision">
            <geometry>
              <mesh>
                <uri>file://{stl_path}</uri>
              </mesh>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>
    """

    sdf_path = "/tmp/runtime_mesh.sdf"
    with open(sdf_path, 'w') as f:
        f.write(sdf_template)
    try:
        command_string = [
            "rosrun", "gazebo_ros", "spawn_model",
            "-file", sdf_path,
            "-sdf", "-model", "runtime_mesh",
            "-x", f"{position[0]}", "-y", f"{position[1]}", "-z", f"{position[2]}"
        ]
        subprocess.run(command_string, stdout=sys.stdout, stderr=sys.stderr, check=True)
    except subprocess.CalledProcessError as process_error:
        ros.logfatal('Failed to run spawnModel command with error: \n%s', process_error.output)
        sys.exit(1)

# writes the texture in addition to the mesh
def write_textured_obj(points, triangles, obj_path, texture_path):
    import os, shutil
    import numpy as np

    obj_dir = os.path.dirname(obj_path)
    base = os.path.splitext(os.path.basename(obj_path))[0]
    mtl_name = base + ".mtl"
    mtl_path = os.path.join(obj_dir, mtl_name)

    # Copy texture next to OBJ
    tex_name = os.path.basename(texture_path)
    tex_dst = os.path.join(obj_dir, tex_name)
    if os.path.abspath(texture_path) != os.path.abspath(tex_dst):
        shutil.copy(texture_path, tex_dst)

    # ---- UVs (planar) ----
    x, y = points[:, 0], points[:, 1]
    x0, x1 = x.min(), x.max()
    y0, y1 = y.min(), y.max()
    u = (x - x0) / max(x1 - x0, 1e-6)
    v = 1.0 - (y - y0) / max(y1 - y0, 1e-6)
    uvs = np.column_stack([u, v])

    # ---- normals ----
    normals = compute_vertex_normals(points, triangles)

    # ---- MTL ----
    with open(mtl_path, "w") as f:
        f.write("newmtl rock_material\n")
        f.write("Ka 1.0 1.0 1.0\n")
        f.write("Kd 1.0 1.0 1.0\n")
        f.write("Ks 0.2 0.2 0.2\n")
        f.write("Ns 50.0\n")
        f.write(f"map_Kd {tex_name}\n")

    # ---- OBJ ----
    with open(obj_path, "w") as f:
        f.write(f"mtllib {mtl_name}\n")
        f.write("usemtl rock_material\n")

        for p in points:
            f.write(f"v {p[0]} {p[1]} {p[2]}\n")

        for uv in uvs:
            f.write(f"vt {uv[0]} {uv[1]}\n")

        for n in normals:
            f.write(f"vn {n[0]} {n[1]} {n[2]}\n")

        # faces: v / vt / vn
        for tri in triangles:
            a, b, c = tri + 1
            f.write(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n")

def compute_vertex_normals(points, triangles):
    normals = np.zeros_like(points)

    for tri in triangles:
        p0, p1, p2 = points[tri]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm > 1e-12:
            n /= norm
        for idx in tri:
            normals[idx] += n

    # normalize
    norms = np.linalg.norm(normals, axis=1)
    norms[norms == 0] = 1.0
    normals /= norms[:, None]
    return normals

def sendStaticTransform(parent, child, x_pos = np.zeros(3), quat=np.array([1,0,0,0]), static_broadcaster=None):
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = ros.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = child
    static_transformStamped.transform.translation.x = 0.
    static_transformStamped.transform.translation.y = 0.
    static_transformStamped.transform.translation.z = 0.
    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1
    if static_broadcaster is None:
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_broadcaster.sendTransform(static_transformStamped)

def getRobotModelFloating(robot_name="hyq"):
    ERROR_MSG = 'You should set the environment variable LOCOSIM_DIR"\n'
    path = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
    if rosgraph.is_master_online():
        urdf = ros.get_param('/robot_description', None) or ros.get_param('/' + robot_name + '/robot_description', None)
        if urdf == None:
            print('Failed to retrieve robot_description: issues in URDF generation for Pinocchio, did not succeed')
            loadXacro(package_name=robot_name+"_description",model_name=robot_name, param_name='/' + robot_name +'/robot_description')
            urdf = ros.get_param('/' + robot_name +'/robot_description')
            print(urdf)
        print("URDF generated_commons")
        os.makedirs(path + "/robot_urdf/generated_urdf/", exist_ok=True)
        urdf_location = path + "/robot_urdf/generated_urdf/" + robot_name + ".urdf"
        print(urdf_location)
        text_file = open(urdf_location, "w")
        text_file.write(urdf)
        text_file.close()
        robot = RobotWrapper.BuildFromURDF(urdf_location, root_joint=pinocchio.JointModelFreeFlyer())
    else: #this is used when you run stuff online (i.e. unit tests)
        try:
            urdf_location = path + "/robot_urdf/generated_urdf/" + robot_name + ".urdf"
            robot = RobotWrapper.BuildFromURDF(urdf_location, root_joint=pinocchio.JointModelFreeFlyer())
        except:
            print('you are running offline, urdf is not present in robot_urdf/generated_urdf folder')

    return robot


def getRobotModel(robot_name="hyq", generate_urdf=False, xacro_path=None, additional_urdf_args=None):
    ERROR_MSG = 'You should set the environment variable LOCOSIM_DIR"\n'
    path = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
    srdf = path + "/robot_urdf/" + robot_name + ".srdf"

    if (generate_urdf):
        try:
            # old way
            if (xacro_path is None):
                xacro_path = rospkg.RosPack().get_path(
                    robot_name + '_description') + '/robots/' + robot_name + '.urdf.xacro'

            package = 'xacro'
            executable = 'xacro'
            name = 'xacro'
            namespace = '/'
            # with gazebo 11 you should set in the ros_impedance_controllerXX.launch the new_gazebo_version = true
            # note we generate the urdf with the floating base joint (new gazebo version should be false by default in the xacro of the robot! because Pinocchio needs it!
            args = xacro_path + ' --inorder -o ' + os.environ[
                'LOCOSIM_DIR'] + '/robot_urdf/generated_urdf/' + robot_name + '.urdf'

            try:
                flywheel = ros.get_param('/flywheel4')
                args += ' flywheel4:=' + flywheel
            except:
                pass

            try:
                flywheel2 = ros.get_param('/flywheel2')
                args += ' flywheel2:=' + flywheel2
            except:
                pass

            try:
                angle = ros.get_param('/angle_deg')
                args += ' angle_deg:=' + angle
            except:
                pass

            try:
                anchorZ = ros.get_param('/anchorZ')
                args += ' anchorZ:=' + anchorZ
            except:
                pass

            if additional_urdf_args is not None:
                args += ' ' + additional_urdf_args

            os.system("rosrun xacro xacro " + args)
            # os.system("rosparam get /robot_description > "+os.environ['LOCOSIM_DIR']+'/robot_urdf/'+robot_name+'.urdf')
            # urdf = URDF.from_parameter_server()
            print("URDF generated_commons")
            urdf_location = path + "/robot_urdf/generated_urdf/" + robot_name + ".urdf"
            print(urdf_location)
            robot = RobotWrapper.BuildFromURDF(urdf_location)
            print("URDF loaded in Pinocchio")
        except:
            print('Issues in URDF generation for Pinocchio, did not succeed')
    else:

        urdf = path + "/robot_urdf/" + robot_name + ".urdf"
        robot = RobotWrapper.BuildFromURDF(urdf, [path, srdf])

    return robot

def getRobotModel(robot_name="hyq", generate_urdf = False, xacro_path = None, additional_urdf_args = None, floating_base=False):
    ERROR_MSG = 'You should set the environment variable LOCOSIM_DIR"\n';
    path  = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
    srdf      = path + "/robot_urdf/" + robot_name + ".srdf"

    if (generate_urdf):  
        try:       
            #old way
            if (xacro_path is None):
                xacro_path = rospkg.RosPack().get_path(robot_name+'_description')+ '/robots/'+robot_name+'.urdf.xacro'
            
            package = 'xacro'
            executable = 'xacro'
            name = 'xacro'
            namespace = '/'
            # with gazebo 11 you should set in the ros_impedance_controllerXX.launch the new_gazebo_version = true
            # note we generate the urdf with the floating base joint (new gazebo version should be false by default in the xacro of the robot! because Pinocchio needs it!
            args = xacro_path+ ' --inorder -o '+os.environ['LOCOSIM_DIR']+'/robot_urdf/generated_urdf/'+robot_name+'.urdf'
     
     
       
            try:
                flywheel = ros.get_param('/flywheel4')
                args+=' flywheel4:='+flywheel
            except:
                pass

            try:
                flywheel2 = ros.get_param('/flywheel2')
                args += ' flywheel2:=' + flywheel2
            except:
                pass

            try:
                angle = ros.get_param('/angle_deg')
                args += ' angle_deg:=' + angle
            except:
                pass

            try:
                anchorZ = ros.get_param('/anchorZ')
                args += ' anchorZ:=' + anchorZ
            except:
                pass

            if additional_urdf_args is not None:
                args += ' '+additional_urdf_args
            
            os.system("rosrun xacro xacro "+args)  
            #os.system("rosparam get /robot_description > "+os.environ['LOCOSIM_DIR']+'/robot_urdf/'+robot_name+'.urdf')  
            #urdf = URDF.from_parameter_server()
            print("URDF generated_commons")
            urdf_location      = path + "/robot_urdf/generated_urdf/" + robot_name+ ".urdf"
            print(urdf_location)
            if floating_base:
                robot = RobotWrapper.BuildFromURDF(urdf_location, root_joint=pinocchio.JointModelFreeFlyer())
            else:
                robot = RobotWrapper.BuildFromURDF(urdf_location)
            print("URDF loaded in Pinocchio")
        except:
            print ('Issues in URDF generation for Pinocchio, did not succeed')
    else:

        urdf      = path + "/robot_urdf/" + robot_name+ ".urdf"
        robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
    
    return robot

def getLinkState(model_name = None, link_name=None, debug=False):
    from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
    from tf.transformations import euler_from_quaternion
    ros.wait_for_service('/gazebo/get_link_state')
    get_link_state = ros.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    req = GetLinkStateRequest()
    req.link_name = model_name+"::"+link_name  # IMPORTANT format
    req.reference_frame = "world"  # or another link
    resp = get_link_state(req)
    pose = resp.link_state.pose
    twist = resp.link_state.twist
    if debug:
        print("Position:", pose.position.x, pose.position.y, pose.position.z)
        print("Orientation:", pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = pose.orientation
    rpy =  np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
    return position, rpy

class SafeTFBroadcaster:
    ''' avoids the annoying TF_REPEATED issue when the message is published twice with the same timestamp '''
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.last_stamp = ros.Time(0)
        self.last_payload = None  # (tx,ty,tz,qx,qy,qz,qw)
        # If sim time is enabled, wait until /clock has published
        if ros.get_param("/use_sim_time", False):
            while ros.Time.now() == ros.Time(0) and not ros.is_shutdown():
                ros.sleep(0.01)
    def sendTransform(self, trans, quat, stamp, child="base_link", parent="world"):
        # 1) enforce monotonic time
        if stamp <= self.last_stamp:
            stamp = self.last_stamp + ros.Duration(nsecs=1)

        # 2) translation: flatten + check
        trans = np.asarray(trans).flatten()
        if trans.shape[0] != 3:
            raise ValueError(f"Translation must have 3 elements, got {trans}")

        # 3) quaternion: unwrap, flatten + check
        if hasattr(quat, "coeffs"):  # Pinocchio Quaternion
            quat = quat.coeffs()
        if len(quat) == 1 and isinstance(quat[0], (tuple, list, np.ndarray)):
            quat = quat[0]

        quat = np.asarray(quat).flatten()
        if quat.shape[0] != 4:
            raise ValueError(f"Quaternion must have 4 elements (x,y,z,w), got {quat}")

        # normalize (optional but safe)
        norm = np.linalg.norm(quat)
        if not np.isclose(norm, 1.0, atol=1e-6) and norm > 0:
            quat = quat / norm

        # 4) skip exact duplicates
        payload = tuple(trans) + tuple(quat)
        if self.last_payload == payload and stamp == self.last_stamp:
            return

        # 5) build message
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = trans
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = quat

        # 6) broadcast
        self.br.sendTransform(msg)

        # 7) update state
        self.last_stamp = stamp
        self.last_payload = payload


#plot functions
def subplot(n_rows, n_cols, n_subplot, sharex=False, sharey=False, ax_to_share=None):
    if sharex and sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharex=ax_to_share, sharey=ax_to_share)
    if sharex and not sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharex=ax_to_share)
    if not sharex and sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharey=ax_to_share)
    if not sharex and not sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot)
    return ax

def plotJoint(name, time_log, q_log=None, q_des_log=None, qd_log=None, qd_des_log=None, qdd_log=None, qdd_des_log=None, tau_log=None, tau_ffwd_log = None, tau_des_log = None, joint_names = None, q_adm = None,
              sharex=True, sharey=False, start=0, end=-1, title=None, subset_index=None):
    plot_var_log = None
    plot_var_des_log = None
    if name=='position':
        unit = '[rad]'
        if   (q_log is not None):
            plot_var_log = q_log
        else:
            plot_var_log = None
        if   (q_des_log is not None):
            plot_var_des_log = q_des_log
        else:
            plot_var_des_log = None

    if name=='velocity':
        unit = '[rad/s]'
        if   (qd_log is not None):
            plot_var_log = qd_log
        else:
            plot_var_log = None
        if   (qd_des_log is not None):
            plot_var_des_log  = qd_des_log
        else:
            plot_var_des_log = None

    if name=='acceleration':
        unit = '[rad/s^2]'
        if   (qdd_log is not None):
            plot_var_log = qdd_log
        else:
            plot_var_log = None
        if   (qdd_des_log is not None):
            plot_var_des_log  = qdd_des_log
        else:
            plot_var_des_log = None

    if name=='torque':
        unit = '[Nm]'
        if   (tau_log is not None):
            plot_var_log = tau_log
        else:
            plot_var_log = None
        if   (tau_des_log is not None):
            plot_var_des_log  = tau_des_log
        else:
          plot_var_des_log = None                                                

    dt = np.round(time_log[1] - time_log[0], 3)
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if plot_var_log is not None:
        njoints = min(plot_var_log.shape)
    elif plot_var_des_log is not None:
        njoints = min(plot_var_des_log.shape)
    else:
        print("no log var has been defined")

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums())+1
    fig = plt.figure(figure_id)

    if title is not None:
        fig.suptitle(title, fontsize=20)
    else:
        fig.suptitle(name, fontsize=20)

    if joint_names is None:
        if njoints <= 6:
            labels = labels_ur
        if njoints == 12:
            labels = labels_quadruped
        if njoints == 14:
            labels = labels_flywheel2
        if njoints == 16:
            labels = labels_flywheel4
        subset_index = range(njoints)
    else:
        if subset_index is None:
            njoints = len(joint_names)
            subset_index = range(njoints)
            labels = joint_names
        else:
            njoints = len(subset_index)
            labels = itemgetter(*subset_index)(joint_names)

    if (njoints % 3 == 0): #divisible by 3
        n_rows = int(njoints/ 3)
        n_cols = 3
    elif (njoints % 2 == 0): #divisible by 2
        n_rows = int(njoints / 2)
        n_cols = 2
    else:  # put in a single columnn
        n_rows = njoints
        n_cols = 1


    for jidx in range(njoints):
        if jidx == 0:
            ax = subplot(n_rows, n_cols, jidx + 1)
        else:
            subplot(n_rows, n_cols, jidx + 1, sharex=sharex, sharey=sharey, ax_to_share=ax)

        if jidx + n_cols >= njoints:
            plt.xlabel("Time [s]")


        plt.ylabel(labels[jidx] + ' '+ unit)

        if name == 'torque' and tau_ffwd_log is not None:
            plt.plot(time_log[start:end], tau_ffwd_log[subset_index[jidx], start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des,
                     color='green')
        if   (plot_var_des_log is not None):
             plt.plot(time_log[start:end], plot_var_des_log[subset_index[jidx], start:end], linestyle='-', marker="o",markersize=marker_size, lw=lw_des,color = 'red')
        if (plot_var_log is not None):
            plt.plot(time_log[start:end], plot_var_log[subset_index[jidx],start:end],linestyle='-',marker="o",markersize=marker_size, lw=lw_act,color = 'blue')

        if (q_adm is not None):
            plt.plot(time_log[start:end], q_adm[subset_index[jidx], start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='black')
        plt.grid()

    if njoints == 12:
        fig.align_ylabels(fig.axes[0:12:3])
        fig.align_ylabels(fig.axes[1:12:4])
        fig.align_ylabels(fig.axes[2:12:4])

    return fig


def plotEndeff(name, figure_id, time_log, plot_var_log, plot_var_des_log = None):

    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)                   
    plt.subplot(3,1,1)
    plt.ylabel("x")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[0,:], lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[0,:], lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,2)
    plt.ylabel("y")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[1,:], lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[1,:], lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,3)
    plt.ylabel("z")
    if   (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[2,:], lw=lw_des, color = 'red')                                        
    plt.plot(time_log, plot_var_log[2,:], lw=lw_act, color = 'blue')
    plt.grid()


def plotAdmittanceTracking(figure_id, time_log, x_log, x_des_log, x_des_log_adm, f_log):

    fig = plt.figure(figure_id)
    fig.suptitle("admittance tracking", fontsize=20)
    plt.subplot(4, 1, 1)
    plt.ylabel("end-effector x")
    plt.plot(time_log, x_log[0, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[0, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[0, :], lw=2, color='black')
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.ylabel("end-effector y")
    plt.plot(time_log, x_log[1, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[1, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[1, :], lw=2, color='black')
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.ylabel("end-effector z")
    plt.plot(time_log, x_log[2, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[2, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[2, :], lw=2, color='black')
    plt.grid()

    f_norm = []
    for i in range(f_log.shape[1]):
        f_norm.append(np.linalg.norm(f_log[:,i]))

    plt.subplot(4, 1, 4)
    plt.plot(time_log, f_norm, lw=2, color='blue')
    plt.ylabel("norm of ee force")
    plt.grid()

def plotFrame(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None, des_Acc_log=None, Acc_log=None,
              des_Wrench_log=None, Wrench_log=None, title=None, frame=None, sharex=False, sharey=False, start=0, end=-1):
    plot_var_des_log = None
    if name == 'position':
        labels = ["x", "y", "z", "R", "P", "Y"]
        lin_unit = '[m]'
        ang_unit = '[rad]'
        if Pose_log is not None:
            plot_var_log = Pose_log
        if (des_Pose_log is not None):
            plot_var_des_log = des_Pose_log
    elif name == 'velocity':
        labels = ["x", "y", "z", "R", "P", "Y"]
        lin_unit = '[m/s]'
        ang_unit = '[rad/s]'
        if Twist_log is not None:
            plot_var_log = Twist_log
        if   (des_Twist_log is not None):
            plot_var_des_log  = des_Twist_log
    elif name == 'acceleration':
        labels = ["x", "y", "z", "R", "P", "Y"]
        lin_unit = '[m/s^2]'
        ang_unit = '[rad/s^2]'
        if Acc_log is not None:
            plot_var_log = Acc_log
        if   (des_Acc_log is not None):
            plot_var_des_log  = des_Acc_log
    elif name == 'wrench':
        labels = ["FX", "FY", "FZ", "MX", "MY", "MX"]
        lin_unit = '[N]'
        ang_unit = '[Nm]'
        if Wrench_log is not None:
            plot_var_log = Wrench_log
        if (des_Wrench_log is not None):
            plot_var_des_log = des_Wrench_log
    else:
       print("wrong choice")

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title+= ' ' + frame

    dt = np.round(time_log[1] - time_log[0], 3)
    if type(start) == str:
        start = int(float(start)/dt + 1)
    if type(end) == str:
        end = int(float(end)/dt + 1)

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 2, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " "+lin_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    plt.grid()
    ax.ticklabel_format(useOffset=False)

    subplot(3, 2, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " "+lin_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 5, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " "+lin_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[3] + " "+ang_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[3, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[3, start:end].T, linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 4, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[4] + " "+ang_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[4, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[4, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 6, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[5] + " "+ang_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[5, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[5, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)


    fig.align_ylabels(fig.axes[:3])
    fig.align_ylabels(fig.axes[3:])

    return fig

def plotFrameLinear(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None, des_Acc_log=None, Acc_log=None,
              des_Wrench_log=None, Wrench_log=None, title=None, frame=None, sharex=True, sharey=False, start=0, end=-1, wrapp_labels=None, custom_labels=None):
    plot_var_log = None
    plot_var_des_log = None
    labels = ["", "", ""]
    if name == 'position':
        labels = ["x", "y", "z"]
        lin_unit = '[m]'
        if Pose_log is not None:
            if Pose_log.shape[0] == 6:
                plot_var_log = u.linPart(Pose_log)
            elif Pose_log.shape[0] == 3:
                plot_var_log = Pose_log
        if (des_Pose_log is not None):
            if des_Pose_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Pose_log)
            elif des_Pose_log.shape[0] == 3:
                plot_var_des_log = des_Pose_log

    elif name == 'velocity':
        labels = ["x", "y", "z"]
        lin_unit = '[m/s]'
        if Twist_log is not None:
            if Twist_log.shape[0] == 6:
                plot_var_log = u.linPart(Twist_log)
            elif Twist_log.shape[0] == 3:
                plot_var_log = Twist_log
        if (des_Twist_log is not None):
            if des_Twist_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Twist_log)
            elif des_Twist_log.shape[0] == 3:
                plot_var_des_log = des_Twist_log

    elif name == 'acceleration':
        labels = ["x", "y", "z"]
        lin_unit = '[m/s^2]'
        if Acc_log is not None:
            if Acc_log.shape[0] == 6:
                plot_var_log = u.linPart(Acc_log)
            elif Acc_log.shape[0] == 3:
                plot_var_log = Acc_log
        if (des_Acc_log is not None):
            if des_Acc_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Acc_log)
            elif des_Acc.shape[0] == 3:
                plot_var_des_log = des_Acc_log

    elif name == 'wrench':
        labels = ["FX", "FY", "FZ"]
        lin_unit = '[N]'
        if Wrench_log is not None:
            if Wrench_log.shape[0] == 6:
                plot_var_log = u.linPart(Wrench_log)
            elif Wrench_log.shape[0] == 3:
                plot_var_log = Wrench_log
        if (des_Wrench_log is not None):
            if des_Wrench_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Wrench_log)
            elif des_Wrench_log.shape[0] == 3:
                plot_var_des_log = des_Wrench_log
    else:
       print("wrong choice")

    if custom_labels is not None:
        labels = custom_labels

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title+= ' ' + frame

    dt = np.round(time_log[1] - time_log[0], 3)
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 1, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " "+lin_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    plt.grid()

    subplot(3, 1, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " "+lin_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()

    subplot(3, 1, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " "+lin_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])

    return fig


def plotFrameAngular(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None, des_Acc_log=None, Acc_log=None,
                    des_Wrench_log=None, Wrench_log=None, title=None, frame=None, sharex=True, sharey=True, start=0, end=-1):
    plot_var_log = None
    plot_var_des_log = None
    if name == 'position':
        labels = ["R", "P", "Y"]
        ang_unit = '[rad]'
        if Pose_log is not None:
            if Pose_log.shape[0] == 6:
                plot_var_log = u.angPart(Pose_log)
            elif Pose_log.shape[0] == 3:
                plot_var_log = Pose_log
        if (des_Pose_log is not None):
            if des_Pose_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Pose_log)
            elif des_Pose_log.shape[0] == 3:
                plot_var_des_log = Pose_log

    elif name == 'velocity':
        labels = ["R", "P", "Y"]
        ang_unit = '[rad]'
        if Twist_log is not None:
            if Twist_log.shape[0] == 6:
                plot_var_log = u.angPart(Twist_log)
            elif Twist_log.shape[0] == 3:
                plot_var_log = Twist_log
        if (des_Twist_log is not None):
            if des_Twist_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Twist_log)
            elif des_Twist_log.shape[0] == 3:
                plot_var_des_log = Twist_log

    elif name == 'acceleration':
        labels = ["R", "P", "Y"]
        ang_unit = '[rad]'
        if Acc_log is not None:
            if Acc_log.shape[0] == 6:
                plot_var_log = u.angPart(Acc_log)
            elif Acc_log.shape[0] == 3:
                plot_var_log = Acc_log
        if (des_Acc_log is not None):
            if des_Acc_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Acc_log)
            elif des_Acc.shape[0] == 3:
                plot_var_des_log = des_Acc_log

    elif name == 'wrench':
        labels = ["MX", "MY", "MZ"]
        ang_unit = '[Nm]'
        if Wrench_log is not None:
            if Wrench_log.shape[0] == 6:
                plot_var_log = u.angPart(Wrench_log)
            elif Wrench_log.shape[0] == 3:
                plot_var_log = Wrench_log
        if (des_Wrench_log is not None):
            if des_Wrench_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Wrench_log)
            elif des_Wrench_log.shape[0] == 3:
                plot_var_des_log = des_Wrench_log
    else:
        print("wrong choice")

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title += ' ' + frame

    dt = np.round(time_log[1] - time_log[0], 3)
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 1, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " " + ang_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_act, color='blue')
    plt.grid()

    subplot(3, 1, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " " + ang_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_act,
                 color='blue')
    plt.grid()

    subplot(3, 1, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " " + ang_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_act,
                 color='blue')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])

    return fig


def plotContacts(name, time_log, des_LinPose_log=None, LinPose_log=None, des_LinTwist_log=None, LinTwist_log=None, des_Forces_log=None,
                 Forces_log=None, gt_Forces_log=None, contact_states=None, frame=None, sharex=True, sharey=True, start=0, end=-1, title=None):
    # %% Input plots
    plot_var_log = None
    plot_var_des_log = None
    if name == 'position':
        unit = '[m]'
        if LinPose_log is not None:
            plot_var_log = LinPose_log
        if (des_LinPose_log is not None):
            plot_var_des_log = des_LinPose_log

    elif name == 'velocity':
        unit = '[m/s]'
        if LinTwist_log is not None:
            plot_var_log = LinTwist_log
        if (des_LinTwist_log is not None):
            plot_var_des_log = des_LinTwist_log

    elif name == 'GRFs':
        labels = ["FX", "FY", "FZ"]
        unit = '[N]'
        if Forces_log is not None:
            plot_var_log = Forces_log
        if (des_Forces_log is not None):
            plot_var_des_log = des_Forces_log
    else:
        print("wrong choice")

    if title is None:
        title = 'Contacts ' + name
        if frame is not None:
            title += ' ' + frame

    dt = np.round(time_log[1] - time_log[0], 3)
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums())+1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)

    ##########
    # LF leg #
    ##########
    # x
    idx = u.leg_map['LF']
    ax = subplot(6, 2, 1)
    plt.ylabel("$LF_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax.twinx()
        ax2.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax.plot(time_log[start:end], plot_var_des_log[3 * idx, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        ax.plot(time_log[start:end], plot_var_log[3 * idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax.grid()


    # y
    ax1 = subplot(6, 2, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LF_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        ax2.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6, 2, 5, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LF_z " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])

    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()

    ##########
    # RF leg #
    ##########
    # x
    idx = u.leg_map['RF']
    ax1 = subplot(6,2,2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RF_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # y
    ax1 = subplot(6,2,4, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RF_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6,2,6, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RF_z " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    ##########
    # LH leg #
    ##########
    # x
    idx = u.leg_map['LH']
    ax1 = subplot(6,2,7, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LH_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3 * idx, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # y
    ax1 = subplot(6,2,9, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LH_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6, 2, 11, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LH_z " + unit +"$", fontsize=10)
    plt.xlabel("Time [s]")
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    ##########
    # RH leg #
    ##########
    # x
    idx = u.leg_map['RH']
    ax1 = subplot(6, 2, 8, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RH_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # y
    ax1 = subplot(6, 2, 10, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RH_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6, 2, 12, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RH_z " + unit +"$", fontsize=10)
    plt.xlabel("Time [s]")
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()

    

    # axes = fig.axes
    # for i in range(6):
    #     yticks = axes[i].get_yticks()
    #     ymin = min(-0.01, min(yticks))
    #     ymax = max(0.01, max(yticks))
    #     axes[i].set_ylim([ymin, ymax])
    #     yticks = axes[i].get_yticks()
    #     axes[i].set_yticks(np.unique(np.around(yticks, 2)))


    fig.align_ylabels(fig.axes[0:12:4])
    fig.align_ylabels(fig.axes[1:12:4])
    fig.align_ylabels(fig.axes[2:12:4])
    fig.align_ylabels(fig.axes[3:12:4])

    return fig



def plotConstraitViolation(figure_id,constr_viol_log):
    fig = plt.figure(figure_id)            
    plt.plot(constr_viol_log[0,:],label="LF")
    plt.plot(constr_viol_log[1,:],label="RF")
    plt.plot(constr_viol_log[2,:],label="LH")
    plt.plot(constr_viol_log[3,:],label="RH")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("Constr violation", fontsize=10)
    plt.grid()                                                                     

def plotEndeffImpedance(name, figure_id, x_log, x_des_log, f_log):                  
    
    title=""    
    
    if name == 'position':
        title="Force vs Displacement" 
    elif name == 'velocity':
        title="Force vs Velocity" 
    elif name == 'acceleration':
        title="Force vs Acceleration"                           
    else:
        print("wrong choice in impedance plotting")
 
    lw_act=4  
    lw_des=7
                    
#    fig = plt.figure(figure_id)    
    fig, axs = plt.subplots(3, 3)
    fig.suptitle(title, fontsize=20)
    
    axs[0, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 0].set_title('Fx vs X')
    axs[0, 0].grid()
    
    axs[0, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 1].set_title('Fx vs Y')
    axs[0, 1].grid()
    
    axs[0, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 2].set_title('Fx vs Z')
    axs[0, 2].grid()
    
    axs[1, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 0].set_title('Fy vs X')
    axs[1, 0].grid()
    
    axs[1, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 1].set_title('Fy vs Y')
    axs[1, 1].grid()
    
    axs[1, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 2].set_title('Fy vs Z')
    axs[1, 2].grid()
    
    axs[2, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 0].set_title('Fz vs X')
    axs[2, 0].grid()
    
    axs[2, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 1].set_title('Fz vs Y')
    axs[2, 1].grid()
    
    axs[2, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 2].set_title('Fz vs Z')
    axs[2, 2].grid()

    return fig
    
def plotJointImpedance(name, q_log, q_des_log, tau_log):
    
    title=""
    
    if name == 'position':
        title="Torque vs Angular Displacement"      
    elif name == 'velocity':
        title="Torue vs Angular Velocity" 
    elif name == 'acceleration':
        title="Torque vs Angular Acceleration"                           
    else:
        print("wrong choice in impedance plotting")
 
    lw_act=4  
    lw_des=3

    #Number of joints
    njoints = q_log.shape[0]                                                            
    
    #neet to transpose the matrix other wise it cannot be plot with numpy array    
    fig = plt.figure()                
    fig.suptitle(name, fontsize=20)             
    labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift","3 - Elbow","4 - Wrist 1","5 - Wrist 2","6 - Wrist 3"]
    labels_hyq = ["LF_HAA", "LF_HFE","LF_KFE","RF_HAA", "RF_HFE","RF_KFE","LH_HAA", "LH_HFE","LH_KFE","RH_HAA", "RH_HFE","RH_KFE"]

    if njoints == 6:
        labels = labels_ur         
    if njoints == 12:
        labels = labels_hyq                  
                
    
    for jidx in range(njoints):
                
        plt.subplot(njoints/2,2,jidx+1)
        plt.ylabel(labels[jidx])    
        plt.plot(q_log[jidx,:].T-q_des_log[jidx,:].T, tau_log[jidx,:].T, linestyle='-', lw=lw_des,color = 'blue')
        plt.grid()


def polar_chart(name, figure_id, phase_deg, mag_solid, mag_dashed, legend = None):
    import matplotlib as mpl
    mpl.use('pgf')
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    import logging
    logging.basicConfig(level=logging.DEBUG)


    # size_font = 16
    # mpl.rcdefaults()
    # mpl.rcParams['lines.linewidth'] = 10
    # mpl.rcParams['lines.markersize'] = 6
    # #mpl.rcParams['patch.linewidth'] = 4
    # mpl.rcParams['axes.grid'] = True
    # mpl.rcParams['axes.labelsize'] = 20
    # mpl.rcParams['font.family'] = 'sans-serif'
    # mpl.rcParams['font.size'] = 20
    # mpl.rcParams['font.serif'] = ['Times New Roman', 'Times', 'Bitstream Vera Serif', 'DejaVu Serif',
    #                               'New Century Schoolbook',
    #                               'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L',
    #                               'Palatino',
    #                               'Charter', 'serif']
    # mpl.rcParams['text.usetex'] = False
    # mpl.rcParams['legend.fontsize'] = 20
    # plt.rcParams['legend.title_fontsize'] = 20
    # mpl.rcParams['legend.loc'] = 'best'
    # mpl.rcParams['figure.facecolor'] = 'white'
    # mpl.rcParams['figure.figsize'] = 10,6
    # mpl.rcParams['savefig.format'] = 'pdf'

    size_font = 16
    mpl.rcdefaults()
    mpl.rcParams['lines.linewidth'] = 2
    # mpl.rcParams['lines.markersize'] = 6
    # mpl.rcParams['patch.linewidth'] = 4
    mpl.rcParams['axes.grid'] = True
    mpl.rcParams['axes.labelsize'] = size_font
    mpl.rcParams['font.family'] = 'sans-serif'
    mpl.rcParams['font.size'] = size_font
    mpl.rcParams['legend.fontsize'] = size_font - 2
    mpl.rcParams['legend.title_fontsize'] = size_font - 2
    mpl.rcParams['legend.loc'] = 'best'
    mpl.rcParams['figure.facecolor'] = 'white'
    mpl.rcParams['figure.figsize'] = 6, 4
    mpl.rcParams['savefig.format'] = 'pdf'
    mpl.rcParams['font.serif'] = ['Times New Roman', 'Times', 'Bitstream Vera Serif', 'DejaVu Serif',
                                  'New Century Schoolbook',
                                  'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L',
                                  'Palatino',
                                  'Charter', 'serif']

    # mpl.rcParams['mathtext.fontset'] = 'dejavuserif'
    # mpl.rcParams['mathtext.bf'] = 'serif:bold'
    plt.rcParams.update(
        {
            "text.usetex": True,
            "pgf.texsystem": "pdflatex",
            "text.latex.preamble": r"\usepackage{bm}",

            # Enforce default LaTeX font.
            # "font.family": "serif",
            "font.serif": ["Computer Modern"],
        }
    )


    phase_rad = []
    for deg in phase_deg:
        rad = deg * np.pi/180
        phase_rad.append(rad)

    patches_solid = []
    for mag in mag_solid:
        if mag is not None:
            poly = np.zeros((len(phase_rad), 2))
            for i in range(len(phase_rad)):
                poly[i, :] = np.array([phase_rad[i], mag[i]])
            patches_solid.append(Polygon(poly))

    patches_dashed = []
    for mag in mag_dashed:
        if mag is not None:
            poly = np.zeros((len(phase_rad), 2))
            for i in range(len(phase_rad)):
                poly[i, :] = np.array([phase_rad[i], mag[i]])
            patches_dashed.append(Polygon(poly))

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    # plt.subplots_adjust(left=0.04, bottom=0.04, top=0.96, right=0.96)


    fcolors = ['none']*3
    ecolors = ['darkgreen', 'b', 'r', 'orange']

    p_solid = PatchCollection(patches_solid, alpha=1, linewidth=4)
    p_solid.set_edgecolor(ecolors)
    p_solid.set_facecolor(fcolors)

    p_dashed = PatchCollection(patches_dashed, alpha=1, linestyles='--', linewidth=4)
    p_dashed.set_edgecolor(ecolors)
    p_dashed.set_facecolor(fcolors)



    ax.set_rmax(3)
    step = np.abs(phase_deg[0]-phase_deg[1])
    phase_rad =np.arange(0,360, step)*np.pi/180
    ax.set_xticks(phase_rad)
    ax.tick_params(axis='x', which='major', pad=12)

    rticks = np.arange(0,4,0.5)
    ax.set_rticks(rticks)

    #rticks_show = np.arange(0, 4, 1)
    ax.set_yticklabels(['0', '', '1', '', '2', '', '3', 'm/s'])
    ax.add_collection(p_solid)
    ax.add_collection(p_dashed)

    if legend is not None:
        legend_elements = [Line2D([0], [0], color=ecolors[i], lw=4, label=legend[i]) for i in range(len(legend))]
        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.2, 0.5), ncol=1, title="drop height [m]")

    fig.suptitle(name)


    fig.tight_layout()
    plt.show()
    return fig, ax

    
def plotWrenches(name, figure_id, time_log, des_Wrench_fb_log=None, des_Wrench_ffwd_log=None, des_Wrench_g_log=None):
    labels = ["FX", "FY", "FZ", "MX", "MY", "MZ"]
    lin_unit = '[N]'
    ang_unit = '[Nm]'
    plot_var_des_log = None
    if name=='feedback' or name=='fb':
        plot_var_des_log = des_Wrench_fb_log
    elif name=='feedforward' or name=='ffwd':
        plot_var_des_log = des_Wrench_ffwd_log
    elif name=='gravity' or name=='g':
        plot_var_des_log = des_Wrench_g_log

    # neet to transpose the matrix other wise it cannot be plot with numpy array
    fig = plt.figure(figure_id)
    fig.suptitle('Wrench ' + name, fontsize=20)
    plt.subplot(3, 2, 1)
    plt.ylabel(labels[0])
    plt.plot(time_log, plot_var_des_log[0, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 3)
    plt.ylabel(labels[1])
    plt.plot(time_log, plot_var_des_log[1, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 5)
    plt.ylabel(labels[2])
    plt.xlabel("Time [s]")
    plt.plot(time_log, plot_var_des_log[2, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 2)
    plt.ylabel(labels[3] )
    plt.plot(time_log, plot_var_des_log[3, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 4)
    plt.ylabel(labels[4] )
    plt.plot(time_log, plot_var_des_log[4, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.ylabel(labels[5] )
    plt.xlabel("Time [s]")
    plt.plot(time_log, plot_var_des_log[5, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])
    fig.align_ylabels(fig.axes[3:])

    return fig
