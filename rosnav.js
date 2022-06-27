'use strict';

var ROSLIB = require('roslib');
var xs = require('xstream');
var three = require('three');
var STLLoader = require('three/examples/jsm/loaders/STLLoader.js');
var ColladaLoader = require('three/examples/jsm/loaders/ColladaLoader.js');
var SpriteText = require('three-spritetext');
var resizeObserver = require('@juggle/resize-observer');
var debounce = require('lodash.debounce');
var threeFreeformControls = require('three-freeform-controls');
var randomColor = require('randomcolor');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var ROSLIB__default = /*#__PURE__*/_interopDefaultLegacy(ROSLIB);
var xs__default = /*#__PURE__*/_interopDefaultLegacy(xs);
var three__default = /*#__PURE__*/_interopDefaultLegacy(three);
var STLLoader__default = /*#__PURE__*/_interopDefaultLegacy(STLLoader);
var ColladaLoader__default = /*#__PURE__*/_interopDefaultLegacy(ColladaLoader);
var SpriteText__default = /*#__PURE__*/_interopDefaultLegacy(SpriteText);
var debounce__default = /*#__PURE__*/_interopDefaultLegacy(debounce);
var randomColor__default = /*#__PURE__*/_interopDefaultLegacy(randomColor);

class RosTopicDataSource {
    constructor(options) {
        var _a;
        this.createdAt = new Date();
        this.isStreamLive = false;
        this.isStreamPaused = false;
        this.internalListener = null;
        this.listeners = new Set();
        this.rosConnectionHook = null;
        this.rosCloseHook = null;
        this.rosErrorHook = null;
        this.addRosHealthHooks = () => {
            this.rosCloseHook = () => {
                this.isStreamLive = false;
                this.cleanStream('complete');
            };
            this.rosErrorHook = error => {
                this.isStreamLive = false;
                this.cleanStream('error', error);
            };
            this.ros.on('error', this.rosErrorHook);
            this.ros.on('close', this.rosCloseHook);
        };
        this.removeRosHealthHooks = () => {
            // type definitions for ros do not expose "off" function
            this.ros.off('error', this.rosErrorHook);
            this.ros.off('close', this.rosCloseHook);
            this.rosErrorHook = null;
            this.rosCloseHook = null;
        };
        this.addListener = (listener) => {
            if (this.listeners.has(listener)) {
                return { success: false, reason: 'listener already present' };
            }
            this.listeners.add(listener);
            if (this.isStreamLive) {
                this.stream.addListener(listener);
            }
            return { success: true };
        };
        this.removeListener = (listener) => {
            if (!this.listeners.has(listener)) {
                return { success: false, reason: 'listener not present' };
            }
            this.listeners.delete(listener);
            this.stream.removeListener(listener);
            return { success: true };
        };
        this.cleanStream = (event, error) => {
            if (event === 'error') {
                this.stream.shamefullySendError(error);
            }
            else {
                this.stream.shamefullySendComplete();
            }
            this.listeners.forEach(listener => {
                this.stream.removeListener(listener);
            });
        };
        this.removeAllListeners = () => {
            this.listeners.forEach(listener => {
                this.stream.removeListener(listener);
            });
            this.listeners.clear();
            return { success: true };
        };
        this.pause = () => {
            this.isStreamPaused = true;
            return { success: true };
        };
        this.resume = () => {
            this.isStreamPaused = false;
            return { success: true };
        };
        this.ros = options.ros;
        this.hasMemory = (_a = options.memory) !== null && _a !== void 0 ? _a : false;
        const topicOptions = {
            ros: this.ros,
            name: options.topicName,
            messageType: options.messageType,
        };
        if (options.compression) {
            topicOptions.compression = options.compression;
        }
        if (options.queueSize) {
            topicOptions.queue_size = options.queueSize;
        }
        if (options.queueLength) {
            topicOptions.queue_length = options.queueLength;
        }
        this.topic = new ROSLIB.Topic(topicOptions);
        this.producer = {
            start: listener => {
                if (!this.rosCloseHook && !this.rosErrorHook) {
                    this.addRosHealthHooks();
                }
                this.internalListener = (message) => {
                    if (this.isStreamPaused) {
                        return;
                    }
                    listener.next(message);
                };
                this.topic.subscribe(this.internalListener);
            },
            stop: () => {
                this.topic.unsubscribe(this.internalListener);
                this.removeRosHealthHooks();
            },
        };
        this.stream = this.hasMemory
            ? xs__default["default"].createWithMemory(this.producer)
            : xs__default["default"].create(this.producer);
        this.isStreamLive = true;
        this.rosConnectionHook = () => {
            if (this.isStreamLive) {
                return;
            }
            this.stream = this.hasMemory
                ? xs__default["default"].createWithMemory(this.producer)
                : xs__default["default"].create(this.producer);
            this.isStreamLive = true;
            this.listeners.forEach(listener => {
                this.stream.addListener(listener);
            });
        };
        this.ros.on('connection', this.rosConnectionHook);
    }
}

/* eslint-disable prettier/prettier */
const OBJECT_TYPE_ARROW = 'Arrow';
const OBJECT_TYPE_ARROW_WITH_CIRCLE = 'ArrowWithCircle';
const OBJECT_TYPE_AXES = 'Axes';
const OBJECT_TYPE_FLAT_ARROW = 'FlatArrow';
const MAX_POINTCLOUD_POINTS = 5000000;
const DEFAULT_BACKGROUND_COLOR = 0xF0F0F0;
const DEFAULT_GRID_SIZE = 30;
const DEFAULT_GRID_DIVISIONS = 30;
const DEFAULT_GRID_COLOR = 0xAAAAAA;
const DEFAULT_GRID_COLOR_CENTERLINE = 0x707070;
const INTERACTIVE_MARKER_ORIENTATION_MODES = {
    INHERIT: 0,
    FIXED: 1,
    VIEW_FACING: 2,
};
const UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES = [
    INTERACTIVE_MARKER_ORIENTATION_MODES.VIEW_FACING,
];
const INTERACTIVE_MARKER_INTERACTION_MODES = {
    NONE: 0,
    MENU: 1,
    BUTTON: 2,
    MOVE_AXIS: 3,
    MOVE_PLANE: 4,
    ROTATE_AXIS: 5,
    MOVE_ROTATE: 6,
    MOVE_3D: 7,
    ROTATE_3D: 8,
    MOVE_ROTATE_3D: 9,
};
/* * ***************************
 *   Message types
 * ************************** */
const MESSAGE_TYPE_ROBOT_MODEL = 'robot_description';
const MESSAGE_TYPE_ACCELSTAMPED = 'geometry_msgs/AccelStamped';
const MESSAGE_TYPE_POINTSTAMPED = 'geometry_msgs/PointStamped';
const MESSAGE_TYPE_POINTSTAMPED2 = 'geometry_msgs/msg/PointStamped';
const MESSAGE_TYPE_POLYGONSTAMPED = 'geometry_msgs/PolygonStamped';
const MESSAGE_TYPE_POSEARRAY = 'geometry_msgs/PoseArray';
const MESSAGE_TYPE_POSEARRAY2 = 'geometry_msgs/msg/PoseArray';
const MESSAGE_TYPE_POSECOVARIANCE = 'geometry_msgs/PoseWithCovariance';
const MESSAGE_TYPE_POSECOVARIANCE2 = 'geometry_msgs/msg/PoseWithCovariance';
const MESSAGE_TYPE_POSESTAMPED = 'geometry_msgs/PoseStamped';
const MESSAGE_TYPE_POSESTAMPED2 = 'geometry_msgs/msg/PoseStamped';
const MESSAGE_TYPE_TWISTSTAMPED = 'geometry_msgs/TwistStamped';
const MESSAGE_TYPE_VECTOR3STAMPED = 'geometry_msgs/Vector3Stamped';
const MESSAGE_TYPE_WRENCHSTAMPED = 'geometry_msgs/WrenchStamped';
const MESSAGE_TYPE_WRENCHSTAMPED2 = 'geometry_msgs/msg/WrenchStamped';
const MESSAGE_TYPE_BOUNDINGVOLUME = 'moveit_msgs/BoundingVolume';
const MESSAGE_TYPE_COLLISION_OBJECT = 'moveit_msgs/CollisionObject';
const MESSAGE_TYPE_DISPLAYROBOTSTATE = 'moveit_msgs/DisplayRobotState';
const MESSAGE_TYPE_DISPLAYTRAJECTORY = 'moveit_msgs/DisplayTrajectory';
const MESSAGE_TYPE_PLANNINGSCENE = 'moveit_msgs/PlanningScene';
const MESSAGE_TYPE_OCCUPANCYGRID = 'nav_msgs/OccupancyGrid';
const MESSAGE_TYPE_OCCUPANCYGRID2 = 'nav_msgs/msg/OccupancyGrid';
const MESSAGE_TYPE_ODOMETRY = 'nav_msgs/Odometry';
const MESSAGE_TYPE_ODOMETRY2 = 'nav_msgs/msg/Odometry';
const MESSAGE_TYPE_PATH = 'nav_msgs/Path';
const MESSAGE_TYPE_PATH2 = 'nav_msgs/msg/Path';
const MESSAGE_TYPE_COMPRESSEDIMAGE = 'sensor_msgs/CompressedImage';
const MESSAGE_TYPE_DISPLAYJOINTSTATE = 'sensor_msgs/JointState';
const MESSAGE_TYPE_IMAGE = 'sensor_msgs/Image';
const MESSAGE_TYPE_IMAGE2 = 'sensor_msgs/msg/Image';
const MESSAGE_TYPE_LASERSCAN = 'sensor_msgs/LaserScan';
const MESSAGE_TYPE_LASERSCAN2 = 'sensor_msgs/msg/LaserScan';
const MESSAGE_TYPE_MAGNETICFIELD = 'sensor_msgs/MagneticField';
const MESSAGE_TYPE_POINTCLOUD = 'sensor_msgs/PointCloud';
const MESSAGE_TYPE_POINTCLOUD2 = 'sensor_msgs/PointCloud2';
const MESSAGE_TYPE_ROS2POINTCLOUD2 = 'sensor_msgs/msg/PointCloud2';
const MESSAGE_TYPE_RANGE = 'sensor_msgs/Range';
const MESSAGE_TYPE_RANGE2 = 'sensor_msgs/msg/Range';
const MESSAGE_TYPE_TF = 'tf/tfMessage';
const MESSAGE_TYPE_ROS2_TF = 'tf/msg/tfMessage';
const MESSAGE_TYPE_TF2 = 'tf2_msgs/TFMessage';
const MESSAGE_TYPE_ROS2_TF2 = 'tf2_msgs/msg/TFMessage';
const MESSAGE_TYPE_MARKER = 'visualization_msgs/Marker';
const MESSAGE_TYPE_MARKER2 = 'visualization_msgs/msg/Marker';
const MESSAGE_TYPE_MARKERARRAY = 'visualization_msgs/MarkerArray';
const MESSAGE_TYPE_MARKERARRAY2 = 'visualization_msgs/msg/MarkerArray';
const MESSAGE_TYPE_INTERACTIVEMARKER = 'visualization_msgs/InteractiveMarkerInit';
const MESSAGE_TYPE_INTERACTIVEMARKER2 = 'visualization_msgs/msg/InteractiveMarkerInit';
const MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE = 'visualization_msgs/InteractiveMarkerUpdate';
const MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2 = 'visualization_msgs/msg/InteractiveMarkerUpdate';
const MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK = 'visualization_msgs/InteractiveMarkerFeedback';
const MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2 = 'visualization_msgs/msg/InteractiveMarkerFeedback';
/** ***************************
 *   Visualizations
 * ************************** */
const VIZ_TYPE_IMAGE = 'Image';
const VIZ_TYPE_INTERACTIVEMARKER = 'InteractiveMarker';
const VIZ_TYPE_LASERSCAN = 'LaserScan';
const VIZ_TYPE_MAP = 'Map';
const VIZ_TYPE_MARKER = 'Marker';
const VIZ_TYPE_MARKERARRAY = 'MarkerArray';
const VIZ_TYPE_ODOMETRY = 'Odometry';
const VIZ_TYPE_PATH = 'Path';
const VIZ_TYPE_POINT = 'Point';
const VIZ_TYPE_POINTCLOUD = 'PointCloud';
const VIZ_TYPE_POLYGON = 'Polygon';
const VIZ_TYPE_POSE = 'Pose';
const VIZ_TYPE_POSEARRAY = 'PoseArray';
const VIZ_TYPE_RANGE = 'Range';
const VIZ_TYPE_ROBOTMODEL = 'RobotModel';
const VIZ_TYPE_TF = 'Tf';
const VIZ_TYPE_WRENCH = 'Wrench';
/* * ***************************
 *   Viz specific constants
 * ************************** */
const POINT_FIELD_DATATYPES = {
    INT8: 1,
    UINT8: 2,
    INT16: 3,
    UINT16: 4,
    INT32: 5,
    UINT32: 6,
    FLOAT32: 7,
    FLOAT64: 8,
};
const COLLISION_OBJECT_OPERATIONS = {
    ADD: 0,
    REMOVE: 1,
    APPEND: 2,
    MOVE: 3,
};
const SOLID_PRIMITIVE_TYPES = {
    BOX: 1,
    SPHERE: 2,
    CYLINDER: 3,
    CONE: 4,
};
const MARKER_OBJECT_TYPES = {
    ARROW: 0,
    CUBE: 1,
    SPHERE: 2,
    CYLINDER: 3,
    LINE_STRIP: 4,
    LINE_LIST: 5,
    CUBE_LIST: 6,
    SPHERE_LIST: 7,
    POINTS: 8,
    TEXT_VIEW_FACING: 9,
    MESH_RESOURCE: 10,
    TRIANGLE_LIST: 11,
};
const LINE_STYLES = {
    LINES: 'Lines',
    BILLBOARDS: 'Billboards',
};
const MAP_COLOR_SCHEMES = {
    MAP: 'map',
    CONST_MAP: 'constmap',
    RAW: 'raw',
};
const LASERSCAN_STYLES = {
    SQUARES: 'squares',
    POINTS: 'points',
    FLAT_SQUARES: 'flat_squares',
    SPHERES: 'spheres',
    BOXES: 'boxes',
};
const COLOR_TRANSFORMERS = {
    INTENSITY: 'Intensity',
    AXIS_COLOR: 'AxisColor',
    FLAT_COLOR: 'FlatColor',
};
const DEPTHCLOUD_STREAMTYPES = {
    VP8: 'vp8',
    MJPEG: 'mjpeg',
};
const POINTCLOUD_COLOR_CHANNELS = {
    RGB: 'rgb',
    INTENSITY: 'intensity',
};
const AXES = {
    X: 'x',
    Y: 'y',
    Z: 'z',
};
const INTENSITY_CHANNEL_OPTIONS = {
    INTENSITY: 'intensity',
    ...AXES,
};
const ODOMETRY_OBJECT_TYPES = {
    arrow: OBJECT_TYPE_ARROW,
    axes: OBJECT_TYPE_AXES,
};
const POSE_OBJECT_TYPES = {
    arrow: OBJECT_TYPE_ARROW,
    axes: OBJECT_TYPE_AXES,
    flatArrow: OBJECT_TYPE_FLAT_ARROW,
};
const WRENCH_OBJECT_TYPES = {
    arrow: OBJECT_TYPE_ARROW,
    arrowWithCircle: OBJECT_TYPE_ARROW_WITH_CIRCLE,
};
/** ***************************
 *   Default Options
 * ************************** */
const DEFAULT_CYLINDER_HEIGHT = 1;
const DEFAULT_CYLINDER_RADIUS = 1;
const DEFAULT_RADIAL_SEGMENTS = 32;
const DEFAULT_CONE_HEIGHT = 1;
const DEFAULT_CONE_RADIUS = 1;
const DEFAULT_COLOR_X_AXIS = '#ff0000';
const DEFAULT_COLOR_Y_AXIS = '#008000';
const DEFAULT_COLOR_Z_AXIS = '#0000ff';
const DEFAULT_COLOR_ARROW = '#f0ff00';
const DEFAULT_COLOR_LINE = '#f0ff00';
const DEFAULT_OPTIONS_SCENE = {
    backgroundColor: DEFAULT_BACKGROUND_COLOR,
    gridSize: DEFAULT_GRID_SIZE,
    gridDivisions: DEFAULT_GRID_DIVISIONS,
    gridColor: DEFAULT_GRID_COLOR,
    gridCenterlineColor: DEFAULT_GRID_COLOR_CENTERLINE,
};
const DEFAULT_OPTIONS_TF_VIEWER = {
    selectedFrame: '',
};
const DEFAULT_OPTIONS_ARROW = {
    shaftLength: 1,
    shaftRadius: 0.05,
    headLength: 0.3,
    headRadius: 0.1,
};
const DEFAULT_OPTIONS_AXES = {
    axesLength: 1,
    axesRadius: 0.1,
};
const DEFAULT_OPTIONS_FLATARROW = {
    arrowLength: 0.3,
};
const DEFAULT_OPTIONS_DISPLAYTRAJECTORY = {
    robot: new three.Group(),
    loop: true,
};
const DEFAULT_OPTIONS_INTENSITY = {
    channelName: INTENSITY_CHANNEL_OPTIONS.INTENSITY,
    useRainbow: false,
    invertRainbow: false,
    minColor: '#000000',
    maxColor: '#ffffff',
    autocomputeIntensityBounds: false,
    maxIntensity: 3730,
    minIntensity: 388,
};
const DEFAULT_OPTIONS_COLLISION_OBJECT = {};
const DEFAULT_OPITONS_AXIS_COLOR = {
    axis: AXES.X,
    autocomputeValueBounds: false,
    useFixedFrame: false,
    minAxisValue: 0,
    maxAxisValue: 0,
};
const DEFAULT_OPTIONS_DEPTHCLOUD = {
    streamType: DEPTHCLOUD_STREAMTYPES.MJPEG,
    f: 526,
    maxDepthPerTile: 1.0,
    pointSize: 3,
    width: 1024,
    height: 1024,
    whiteness: 0,
    varianceThreshold: 0.000016667,
};
const DEFAULT_OPTIONS_IMAGE = {
    queueSize: 1,
    width: 320,
    height: 240,
    compression: 'cbor',
};
const DEFAULT_OPTIONS_IMAGE_STREAM = {
    width: 320,
    height: 240,
};
const DEFAULT_OPTIONS_LASERSCAN = {
    selectable: false,
    style: LASERSCAN_STYLES.FLAT_SQUARES,
    size: 0.05,
    alpha: 1,
    decayTime: 0,
    queueSize: 10,
    compression: 'cbor',
    colorTransformer: COLOR_TRANSFORMERS.INTENSITY,
    flatColor: '#ffffff',
    ...DEFAULT_OPTIONS_INTENSITY,
    ...DEFAULT_OPITONS_AXIS_COLOR,
};
const DEFAULT_OPTIONS_MAP = {
    alpha: 1,
    colorScheme: MAP_COLOR_SCHEMES.MAP,
    compression: 'cbor',
    drawBehind: false,
};
const DEFAULT_OPTIONS_MARKER = {
    queueSize: 1,
    namespaces: [],
};
const DEFAULT_OPTIONS_MARKERARRAY = {
    queueSize: 1,
    namespaces: [],
    throttleRate: 0,
};
const DEFAULT_OPTIONS_INTERACTIVE_MARKER = {
    queueSize: 1,
    namespaces: [],
    throttleRate: 0,
    publishThrottleRate: 0,
    hideOtherHandlesOnSelect: true,
    hideOtherControlsInstancesOnSelect: true,
};
const DEFAULT_OPTIONS_ODOMETRY = {
    type: OBJECT_TYPE_ARROW,
    color: DEFAULT_COLOR_X_AXIS,
    alpha: 1,
    ...DEFAULT_OPTIONS_ARROW,
    ...DEFAULT_OPTIONS_AXES,
    ...DEFAULT_OPTIONS_FLATARROW,
    positionTolerance: 0.1,
    angleTolerance: 0.1,
    keep: 100,
};
const DEFAULT_OPTIONS_PATH = {
    color: '#ffffff',
    alpha: 1,
};
const DEFAULT_OPTIONS_PLANNINGSCENE = {};
const DEFAULT_OPTIONS_POINTCLOUD = {
    compression: 'cbor',
    colorChannel: POINTCLOUD_COLOR_CHANNELS.RGB,
    size: 0.0125,
    useRainbow: true,
    queueSize: 1,
};
const DEFAULT_OPTIONS_POINT = {
    color: '#ff0000',
    alpha: 1,
    radius: 0.2,
};
const DEFAULT_OPTIONS_POLYGON = {
    color: '#ffffff',
    alpha: 1,
};
const DEFAULT_OPTIONS_POSE = {
    color: DEFAULT_COLOR_X_AXIS,
    alpha: 1,
    ...DEFAULT_OPTIONS_ARROW,
    ...DEFAULT_OPTIONS_AXES,
    type: POSE_OBJECT_TYPES.arrow,
};
const DEFAULT_OPTIONS_TORUS = {
    circleRadius: 0.3,
    tube: 0.03,
    radialSegments: 16,
    tubularSegments: 100,
    arc: 1.7 * Math.PI,
};
const DEFAULT_OPTIONS_ARROW_WITH_CIRCLE = {
    ...DEFAULT_OPTIONS_ARROW,
    ...DEFAULT_OPTIONS_TORUS,
    circleConeLength: 0.1,
    circleConeRadius: 0.1,
};
const DEFAULT_OPTIONS_WRENCH = {
    forceColor: DEFAULT_COLOR_X_AXIS,
    torqueColor: DEFAULT_COLOR_ARROW,
    alpha: 1,
    forceArrowScale: 1,
    torqueArrowScale: 1,
    arrowWidth: 1,
    ...DEFAULT_OPTIONS_ARROW_WITH_CIRCLE,
    type: WRENCH_OBJECT_TYPES.arrow,
};
const DEFAULT_OPTIONS_POSEARRAY = {
    color: DEFAULT_COLOR_X_AXIS,
    alpha: 1,
    ...DEFAULT_OPTIONS_ARROW,
    ...DEFAULT_OPTIONS_AXES,
    ...DEFAULT_OPTIONS_FLATARROW,
    type: POSE_OBJECT_TYPES.arrow,
};
const DEFAULT_OPTIONS_RANGE = {
    color: DEFAULT_COLOR_X_AXIS,
    alpha: 1,
};
const DEFAULT_OPTIONS_ROBOTMODEL = {};
const DEFAULT_OPTIONS_TF = {};
/** ***************************
 *   Zethus exports
 * ************************** */
const SUPPORTED_MESSAGE_TYPES = [
    MESSAGE_TYPE_IMAGE,
    MESSAGE_TYPE_IMAGE2,
    MESSAGE_TYPE_LASERSCAN,
    MESSAGE_TYPE_LASERSCAN2,
    MESSAGE_TYPE_MARKER,
    MESSAGE_TYPE_MARKER2,
    MESSAGE_TYPE_MARKERARRAY,
    MESSAGE_TYPE_MARKERARRAY2,
    MESSAGE_TYPE_OCCUPANCYGRID,
    MESSAGE_TYPE_OCCUPANCYGRID2,
    MESSAGE_TYPE_ODOMETRY,
    MESSAGE_TYPE_ODOMETRY2,
    MESSAGE_TYPE_PATH,
    MESSAGE_TYPE_PATH2,
    MESSAGE_TYPE_PLANNINGSCENE,
    MESSAGE_TYPE_POINTCLOUD2,
    MESSAGE_TYPE_ROS2POINTCLOUD2,
    MESSAGE_TYPE_POLYGONSTAMPED,
    MESSAGE_TYPE_POSESTAMPED,
    MESSAGE_TYPE_POSESTAMPED2,
    MESSAGE_TYPE_POSEARRAY,
    MESSAGE_TYPE_POSEARRAY2,
    MESSAGE_TYPE_POSESTAMPED,
    MESSAGE_TYPE_POSESTAMPED2,
    MESSAGE_TYPE_ROBOT_MODEL,
    MESSAGE_TYPE_TF,
    MESSAGE_TYPE_ROS2_TF,
    MESSAGE_TYPE_TF2,
    MESSAGE_TYPE_ROS2_TF2,
];
const UNSUPPORTED_MESSAGE_TYPES = [
    MESSAGE_TYPE_ACCELSTAMPED,
    MESSAGE_TYPE_BOUNDINGVOLUME,
    MESSAGE_TYPE_COMPRESSEDIMAGE,
    MESSAGE_TYPE_DISPLAYJOINTSTATE,
    MESSAGE_TYPE_DISPLAYROBOTSTATE,
    MESSAGE_TYPE_DISPLAYTRAJECTORY,
    MESSAGE_TYPE_INTERACTIVEMARKER,
    MESSAGE_TYPE_MAGNETICFIELD,
    MESSAGE_TYPE_POINTCLOUD,
    MESSAGE_TYPE_POSECOVARIANCE,
    MESSAGE_TYPE_RANGE,
    MESSAGE_TYPE_TWISTSTAMPED,
    MESSAGE_TYPE_VECTOR3STAMPED,
    MESSAGE_TYPE_WRENCHSTAMPED,
];

var CONSTANTS = /*#__PURE__*/Object.freeze({
    __proto__: null,
    OBJECT_TYPE_ARROW: OBJECT_TYPE_ARROW,
    OBJECT_TYPE_ARROW_WITH_CIRCLE: OBJECT_TYPE_ARROW_WITH_CIRCLE,
    OBJECT_TYPE_AXES: OBJECT_TYPE_AXES,
    OBJECT_TYPE_FLAT_ARROW: OBJECT_TYPE_FLAT_ARROW,
    MAX_POINTCLOUD_POINTS: MAX_POINTCLOUD_POINTS,
    DEFAULT_BACKGROUND_COLOR: DEFAULT_BACKGROUND_COLOR,
    DEFAULT_GRID_SIZE: DEFAULT_GRID_SIZE,
    DEFAULT_GRID_DIVISIONS: DEFAULT_GRID_DIVISIONS,
    DEFAULT_GRID_COLOR: DEFAULT_GRID_COLOR,
    DEFAULT_GRID_COLOR_CENTERLINE: DEFAULT_GRID_COLOR_CENTERLINE,
    INTERACTIVE_MARKER_ORIENTATION_MODES: INTERACTIVE_MARKER_ORIENTATION_MODES,
    UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES: UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES,
    INTERACTIVE_MARKER_INTERACTION_MODES: INTERACTIVE_MARKER_INTERACTION_MODES,
    MESSAGE_TYPE_ROBOT_MODEL: MESSAGE_TYPE_ROBOT_MODEL,
    MESSAGE_TYPE_ACCELSTAMPED: MESSAGE_TYPE_ACCELSTAMPED,
    MESSAGE_TYPE_POINTSTAMPED: MESSAGE_TYPE_POINTSTAMPED,
    MESSAGE_TYPE_POINTSTAMPED2: MESSAGE_TYPE_POINTSTAMPED2,
    MESSAGE_TYPE_POLYGONSTAMPED: MESSAGE_TYPE_POLYGONSTAMPED,
    MESSAGE_TYPE_POSEARRAY: MESSAGE_TYPE_POSEARRAY,
    MESSAGE_TYPE_POSEARRAY2: MESSAGE_TYPE_POSEARRAY2,
    MESSAGE_TYPE_POSECOVARIANCE: MESSAGE_TYPE_POSECOVARIANCE,
    MESSAGE_TYPE_POSECOVARIANCE2: MESSAGE_TYPE_POSECOVARIANCE2,
    MESSAGE_TYPE_POSESTAMPED: MESSAGE_TYPE_POSESTAMPED,
    MESSAGE_TYPE_POSESTAMPED2: MESSAGE_TYPE_POSESTAMPED2,
    MESSAGE_TYPE_TWISTSTAMPED: MESSAGE_TYPE_TWISTSTAMPED,
    MESSAGE_TYPE_VECTOR3STAMPED: MESSAGE_TYPE_VECTOR3STAMPED,
    MESSAGE_TYPE_WRENCHSTAMPED: MESSAGE_TYPE_WRENCHSTAMPED,
    MESSAGE_TYPE_WRENCHSTAMPED2: MESSAGE_TYPE_WRENCHSTAMPED2,
    MESSAGE_TYPE_BOUNDINGVOLUME: MESSAGE_TYPE_BOUNDINGVOLUME,
    MESSAGE_TYPE_COLLISION_OBJECT: MESSAGE_TYPE_COLLISION_OBJECT,
    MESSAGE_TYPE_DISPLAYROBOTSTATE: MESSAGE_TYPE_DISPLAYROBOTSTATE,
    MESSAGE_TYPE_DISPLAYTRAJECTORY: MESSAGE_TYPE_DISPLAYTRAJECTORY,
    MESSAGE_TYPE_PLANNINGSCENE: MESSAGE_TYPE_PLANNINGSCENE,
    MESSAGE_TYPE_OCCUPANCYGRID: MESSAGE_TYPE_OCCUPANCYGRID,
    MESSAGE_TYPE_OCCUPANCYGRID2: MESSAGE_TYPE_OCCUPANCYGRID2,
    MESSAGE_TYPE_ODOMETRY: MESSAGE_TYPE_ODOMETRY,
    MESSAGE_TYPE_ODOMETRY2: MESSAGE_TYPE_ODOMETRY2,
    MESSAGE_TYPE_PATH: MESSAGE_TYPE_PATH,
    MESSAGE_TYPE_PATH2: MESSAGE_TYPE_PATH2,
    MESSAGE_TYPE_COMPRESSEDIMAGE: MESSAGE_TYPE_COMPRESSEDIMAGE,
    MESSAGE_TYPE_DISPLAYJOINTSTATE: MESSAGE_TYPE_DISPLAYJOINTSTATE,
    MESSAGE_TYPE_IMAGE: MESSAGE_TYPE_IMAGE,
    MESSAGE_TYPE_IMAGE2: MESSAGE_TYPE_IMAGE2,
    MESSAGE_TYPE_LASERSCAN: MESSAGE_TYPE_LASERSCAN,
    MESSAGE_TYPE_LASERSCAN2: MESSAGE_TYPE_LASERSCAN2,
    MESSAGE_TYPE_MAGNETICFIELD: MESSAGE_TYPE_MAGNETICFIELD,
    MESSAGE_TYPE_POINTCLOUD: MESSAGE_TYPE_POINTCLOUD,
    MESSAGE_TYPE_POINTCLOUD2: MESSAGE_TYPE_POINTCLOUD2,
    MESSAGE_TYPE_ROS2POINTCLOUD2: MESSAGE_TYPE_ROS2POINTCLOUD2,
    MESSAGE_TYPE_RANGE: MESSAGE_TYPE_RANGE,
    MESSAGE_TYPE_RANGE2: MESSAGE_TYPE_RANGE2,
    MESSAGE_TYPE_TF: MESSAGE_TYPE_TF,
    MESSAGE_TYPE_ROS2_TF: MESSAGE_TYPE_ROS2_TF,
    MESSAGE_TYPE_TF2: MESSAGE_TYPE_TF2,
    MESSAGE_TYPE_ROS2_TF2: MESSAGE_TYPE_ROS2_TF2,
    MESSAGE_TYPE_MARKER: MESSAGE_TYPE_MARKER,
    MESSAGE_TYPE_MARKER2: MESSAGE_TYPE_MARKER2,
    MESSAGE_TYPE_MARKERARRAY: MESSAGE_TYPE_MARKERARRAY,
    MESSAGE_TYPE_MARKERARRAY2: MESSAGE_TYPE_MARKERARRAY2,
    MESSAGE_TYPE_INTERACTIVEMARKER: MESSAGE_TYPE_INTERACTIVEMARKER,
    MESSAGE_TYPE_INTERACTIVEMARKER2: MESSAGE_TYPE_INTERACTIVEMARKER2,
    MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE: MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE,
    MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2: MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2,
    MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK: MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK,
    MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2: MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2,
    VIZ_TYPE_IMAGE: VIZ_TYPE_IMAGE,
    VIZ_TYPE_INTERACTIVEMARKER: VIZ_TYPE_INTERACTIVEMARKER,
    VIZ_TYPE_LASERSCAN: VIZ_TYPE_LASERSCAN,
    VIZ_TYPE_MAP: VIZ_TYPE_MAP,
    VIZ_TYPE_MARKER: VIZ_TYPE_MARKER,
    VIZ_TYPE_MARKERARRAY: VIZ_TYPE_MARKERARRAY,
    VIZ_TYPE_ODOMETRY: VIZ_TYPE_ODOMETRY,
    VIZ_TYPE_PATH: VIZ_TYPE_PATH,
    VIZ_TYPE_POINT: VIZ_TYPE_POINT,
    VIZ_TYPE_POINTCLOUD: VIZ_TYPE_POINTCLOUD,
    VIZ_TYPE_POLYGON: VIZ_TYPE_POLYGON,
    VIZ_TYPE_POSE: VIZ_TYPE_POSE,
    VIZ_TYPE_POSEARRAY: VIZ_TYPE_POSEARRAY,
    VIZ_TYPE_RANGE: VIZ_TYPE_RANGE,
    VIZ_TYPE_ROBOTMODEL: VIZ_TYPE_ROBOTMODEL,
    VIZ_TYPE_TF: VIZ_TYPE_TF,
    VIZ_TYPE_WRENCH: VIZ_TYPE_WRENCH,
    POINT_FIELD_DATATYPES: POINT_FIELD_DATATYPES,
    COLLISION_OBJECT_OPERATIONS: COLLISION_OBJECT_OPERATIONS,
    SOLID_PRIMITIVE_TYPES: SOLID_PRIMITIVE_TYPES,
    MARKER_OBJECT_TYPES: MARKER_OBJECT_TYPES,
    LINE_STYLES: LINE_STYLES,
    MAP_COLOR_SCHEMES: MAP_COLOR_SCHEMES,
    LASERSCAN_STYLES: LASERSCAN_STYLES,
    COLOR_TRANSFORMERS: COLOR_TRANSFORMERS,
    POINTCLOUD_COLOR_CHANNELS: POINTCLOUD_COLOR_CHANNELS,
    AXES: AXES,
    INTENSITY_CHANNEL_OPTIONS: INTENSITY_CHANNEL_OPTIONS,
    ODOMETRY_OBJECT_TYPES: ODOMETRY_OBJECT_TYPES,
    POSE_OBJECT_TYPES: POSE_OBJECT_TYPES,
    WRENCH_OBJECT_TYPES: WRENCH_OBJECT_TYPES,
    DEFAULT_CYLINDER_HEIGHT: DEFAULT_CYLINDER_HEIGHT,
    DEFAULT_CYLINDER_RADIUS: DEFAULT_CYLINDER_RADIUS,
    DEFAULT_RADIAL_SEGMENTS: DEFAULT_RADIAL_SEGMENTS,
    DEFAULT_CONE_HEIGHT: DEFAULT_CONE_HEIGHT,
    DEFAULT_CONE_RADIUS: DEFAULT_CONE_RADIUS,
    DEFAULT_COLOR_X_AXIS: DEFAULT_COLOR_X_AXIS,
    DEFAULT_COLOR_Y_AXIS: DEFAULT_COLOR_Y_AXIS,
    DEFAULT_COLOR_Z_AXIS: DEFAULT_COLOR_Z_AXIS,
    DEFAULT_COLOR_ARROW: DEFAULT_COLOR_ARROW,
    DEFAULT_COLOR_LINE: DEFAULT_COLOR_LINE,
    DEFAULT_OPTIONS_SCENE: DEFAULT_OPTIONS_SCENE,
    DEFAULT_OPTIONS_TF_VIEWER: DEFAULT_OPTIONS_TF_VIEWER,
    DEFAULT_OPTIONS_ARROW: DEFAULT_OPTIONS_ARROW,
    DEFAULT_OPTIONS_AXES: DEFAULT_OPTIONS_AXES,
    DEFAULT_OPTIONS_FLATARROW: DEFAULT_OPTIONS_FLATARROW,
    DEFAULT_OPTIONS_DISPLAYTRAJECTORY: DEFAULT_OPTIONS_DISPLAYTRAJECTORY,
    DEFAULT_OPTIONS_COLLISION_OBJECT: DEFAULT_OPTIONS_COLLISION_OBJECT,
    DEFAULT_OPTIONS_DEPTHCLOUD: DEFAULT_OPTIONS_DEPTHCLOUD,
    DEFAULT_OPTIONS_IMAGE: DEFAULT_OPTIONS_IMAGE,
    DEFAULT_OPTIONS_IMAGE_STREAM: DEFAULT_OPTIONS_IMAGE_STREAM,
    DEFAULT_OPTIONS_LASERSCAN: DEFAULT_OPTIONS_LASERSCAN,
    DEFAULT_OPTIONS_MAP: DEFAULT_OPTIONS_MAP,
    DEFAULT_OPTIONS_MARKER: DEFAULT_OPTIONS_MARKER,
    DEFAULT_OPTIONS_MARKERARRAY: DEFAULT_OPTIONS_MARKERARRAY,
    DEFAULT_OPTIONS_INTERACTIVE_MARKER: DEFAULT_OPTIONS_INTERACTIVE_MARKER,
    DEFAULT_OPTIONS_ODOMETRY: DEFAULT_OPTIONS_ODOMETRY,
    DEFAULT_OPTIONS_PATH: DEFAULT_OPTIONS_PATH,
    DEFAULT_OPTIONS_PLANNINGSCENE: DEFAULT_OPTIONS_PLANNINGSCENE,
    DEFAULT_OPTIONS_POINTCLOUD: DEFAULT_OPTIONS_POINTCLOUD,
    DEFAULT_OPTIONS_POINT: DEFAULT_OPTIONS_POINT,
    DEFAULT_OPTIONS_POLYGON: DEFAULT_OPTIONS_POLYGON,
    DEFAULT_OPTIONS_POSE: DEFAULT_OPTIONS_POSE,
    DEFAULT_OPTIONS_TORUS: DEFAULT_OPTIONS_TORUS,
    DEFAULT_OPTIONS_ARROW_WITH_CIRCLE: DEFAULT_OPTIONS_ARROW_WITH_CIRCLE,
    DEFAULT_OPTIONS_WRENCH: DEFAULT_OPTIONS_WRENCH,
    DEFAULT_OPTIONS_POSEARRAY: DEFAULT_OPTIONS_POSEARRAY,
    DEFAULT_OPTIONS_RANGE: DEFAULT_OPTIONS_RANGE,
    DEFAULT_OPTIONS_ROBOTMODEL: DEFAULT_OPTIONS_ROBOTMODEL,
    DEFAULT_OPTIONS_TF: DEFAULT_OPTIONS_TF,
    SUPPORTED_MESSAGE_TYPES: SUPPORTED_MESSAGE_TYPES,
    UNSUPPORTED_MESSAGE_TYPES: UNSUPPORTED_MESSAGE_TYPES
});

const checkToleranceThresholdExceed = (oldPose, newPose, options) => {
    const { angleTolerance, positionTolerance } = options;
    const { position, quaternion } = newPose;
    const { position: oldPosition, quaternion: oldQuaternion } = oldPose;
    const positionToleranceBool = oldPosition.distanceTo(position) > positionTolerance;
    const angleToleranceBool = oldQuaternion.angleTo(quaternion) > angleTolerance;
    return positionToleranceBool || angleToleranceBool;
};
const setObjectDimension = (object, options) => {
    switch (options.type) {
        case OBJECT_TYPE_ARROW: {
            const { alpha, color, headLength, headRadius, shaftLength, shaftRadius, } = options;
            object.setHeadDimensions({ radius: headRadius, length: headLength });
            object.setShaftDimensions({ radius: shaftRadius, length: shaftLength });
            object.setAlpha(alpha);
            object.setColor({
                cone: new three.Color(color),
                cylinder: new three.Color(color),
            });
            break;
        }
        case OBJECT_TYPE_AXES: {
            const { axesLength, axesRadius } = options;
            object.setLength(axesLength);
            object.setRadius(axesRadius);
            break;
        }
        case OBJECT_TYPE_FLAT_ARROW: {
            const { arrowLength, color } = options;
            object.setLength(arrowLength);
            object.setColor(new three.Color(color));
            break;
        }
        case OBJECT_TYPE_ARROW_WITH_CIRCLE:
            const { alpha, circleConeLength, circleConeRadius, circleRadius, color, headLength, headRadius, shaftLength, shaftRadius, tube, } = options;
            object.setHeadDimensions({ radius: headRadius, length: headLength });
            object.setShaftDimensions({ radius: shaftRadius, length: shaftLength });
            object.setTorusDimensions({ radius: circleRadius, tube });
            object.setCircleConeDimensions({
                radius: circleConeRadius,
                length: circleConeLength,
            });
            object.setAlpha(alpha);
            object.setColor({
                cone: new three.Color(color),
                cylinder: new three.Color(color),
                torus: new three.Color(color),
                circleCone: new three.Color(color),
            });
            break;
    }
};
function assertIsDefined(val) {
    if (val === undefined || val === null) {
        throw new TypeError(`Expected 'val' to be defined, but received ${val}`);
    }
}
function assertIsMesh(val) {
    if (!(val instanceof three.Mesh)) {
        throw new TypeError(`Expected 'val' to be mesh`);
    }
}
function assertIsMaterial(val) {
    if (!(val instanceof three.Material)) {
        throw new TypeError(`Expected 'val' to be Material`);
    }
}
function assertIsMeshBasicMaterial(val) {
    if (!(val instanceof three.MeshBasicMaterial)) {
        throw new TypeError(`Expected 'val' to be MeshBasicMaterial`);
    }
}
function assertIsMaterialWithColor(val) {
    if (!('color' in val)) {
        throw new TypeError(`Expected 'val' to be MeshBasicMaterial with color`);
    }
}
function assertBehavesLikeArray(val) {
    if (val.length === undefined) {
        throw new TypeError(`Expected 'val' to be an array`);
    }
}
function isObject3D(val) {
    return val instanceof three.Object3D;
}
function isHTMLElement(val) {
    return val instanceof HTMLElement;
}
function assertIsTorusGeometry(val) {
    if (!(val instanceof three.TorusGeometry)) {
        throw new Error('the provided geometry was not TorusGeometry');
    }
}
function assertIsHTMLVideoElement(val) {
    if (!(val instanceof HTMLVideoElement)) {
        throw new Error('the provided element must be an HTMLVideoElement');
    }
}

class LegacyCore {
    constructor(ros, resourceName, messageType, options = {}) {
        const { onHeaderChange } = options;
        this.options = options;
        this.ros = ros;
        this.headerFrameId = '';
        if (ros && resourceName) {
            this.changeTopic(resourceName, messageType, false);
        }
        this.onHeaderChange = onHeaderChange || (() => { });
        this.update = this.update.bind(this);
    }
    hide() {
        assertIsDefined(this.object);
        this.object.visible = false;
    }
    show() {
        assertIsDefined(this.object);
        this.object.visible = true;
    }
    destroy() {
        this.unsubscribe();
        if (this.object && this.object.parent) {
            this.object.parent.remove(this.object);
            this.object = null;
        }
    }
    reset() { }
    subscribe() {
        if (!this.dataSourceInstances) {
            return;
        }
        this.dataSourceInstances.forEach(t => {
            const listener = {
                next: this.update,
                error: (error) => console.log(error),
                complete: () => console.log('stream complete'),
            };
            t.addListener(listener);
        });
    }
    unsubscribe() {
        if (!this.dataSourceInstances) {
            return;
        }
        this.dataSourceInstances.forEach(t => {
            t.removeAllListeners();
        });
    }
    update(message) {
        const header = message.header ? message.header.frame_id : '';
        if (header !== this.headerFrameId) {
            this.headerFrameId = header;
            this.onHeaderChange(this.headerFrameId);
        }
    }
    updateOptions(options) {
        this.options = {
            ...this.options,
            ...options,
        };
    }
    changeTopic(resourceName, type, autoSubscribe = true) {
        const { compression, queueSize, throttleRate } = this.options;
        if (autoSubscribe) {
            this.unsubscribe();
        }
        // unused variable; left for legacy purposes
        this.topicName = resourceName;
        this.messageType = type || this.messageType;
        this.dataSourceInstances = (Array.isArray(resourceName)
            ? resourceName
            : [{ name: resourceName, messageType: type }]).map(({ name, messageType }) => new RosTopicDataSource({
            ros: this.ros,
            topicName: name,
            messageType,
            compression: compression || 'none',
            throttleRate: throttleRate || 0,
            queueSize: queueSize || 10,
        }));
        if (autoSubscribe) {
            this.reset();
            this.subscribe();
        }
    }
}

const setTransform = (object, transform) => {
    const { translation: { x: posX, y: posY, z: posZ }, rotation: { w: orientW, x: orientX, y: orientY, z: orientZ }, } = transform;
    object.position.set(posX, posY, posZ);
    object.quaternion.copy(new three.Quaternion(orientX, orientY, orientZ, orientW).normalize());
};
const setScale = (object, scale) => {
    const { x, y, z } = scale;
    object.scale.set(x, y, z);
};
const setColor = (object, color) => {
    assertIsMaterialWithColor(object.material);
    if (typeof color === 'string' || typeof color === 'number') {
        object.material.color = new three.Color(color);
    }
    else {
        object.material.color.setRGB(color.r, color.g, color.b);
    }
};

class Group extends three.Group {
    setTransform(transform) {
        setTransform(this, transform);
    }
    setScale(scale) {
        setScale(this, scale);
    }
    setColor(colors) {
        this.children.forEach(child => {
            setColor(child, colors);
        });
    }
}

class Mesh extends three.Mesh {
    setTransform(transform) {
        setTransform(this, transform);
    }
    setScale(scale) {
        if (scale) {
            setScale(this, scale);
        }
    }
    setColor(colors) {
        if (colors) {
            setColor(this, colors);
        }
    }
    setAlpha(alpha) {
        if (Array.isArray(this.material)) {
            this.material.forEach(material => {
                material.opacity = three.MathUtils.clamp(alpha, 0, 1);
            });
        }
        else {
            debugger;
            this.material.opacity = three.MathUtils.clamp(alpha, 0, 1);
        }
    }
}

class Box extends Mesh {
    constructor() {
        super();
        this.geometry = new three.BoxGeometry();
        this.material = new three.MeshStandardMaterial();
    }
}

class CollisionObject extends LegacyCore {
    constructor(ros, topicName, options = DEFAULT_OPTIONS_COLLISION_OBJECT) {
        super(ros !== null && ros !== void 0 ? ros : null, topicName !== null && topicName !== void 0 ? topicName : null, MESSAGE_TYPE_COLLISION_OBJECT, {
            ...DEFAULT_OPTIONS_COLLISION_OBJECT,
            ...options,
        });
        this.object = new Group();
        this.updateOptions({
            ...DEFAULT_OPTIONS_COLLISION_OBJECT,
            ...options,
        });
    }
    update(message) {
        var _a, _b;
        super.update(message);
        const { id, operation, primitive_poses: poses, primitives } = message;
        const existingObject = this.object.getObjectByName(id);
        switch (operation) {
            case COLLISION_OBJECT_OPERATIONS.ADD: {
                if (existingObject) {
                    (_a = existingObject.parent) === null || _a === void 0 ? void 0 : _a.remove(existingObject);
                }
                const newObject = new Group();
                newObject.name = id;
                primitives.forEach((primitiveInfo, index) => {
                    const primitive = CollisionObject.getNewPrimitive(primitiveInfo);
                    primitive === null || primitive === void 0 ? void 0 : primitive.setTransform({
                        translation: poses[index].position,
                        rotation: poses[index].orientation,
                    });
                    if (primitive) {
                        newObject.add(primitive);
                    }
                });
                this.object.add(newObject);
                break;
            }
            case COLLISION_OBJECT_OPERATIONS.REMOVE: {
                if (existingObject) {
                    (_b = existingObject.parent) === null || _b === void 0 ? void 0 : _b.remove(existingObject);
                }
                break;
            }
            case COLLISION_OBJECT_OPERATIONS.APPEND: {
                primitives.forEach((primitiveInfo, index) => {
                    const primitive = CollisionObject.getNewPrimitive(primitiveInfo);
                    primitive === null || primitive === void 0 ? void 0 : primitive.setTransform({
                        translation: poses[index].position,
                        rotation: poses[index].orientation,
                    });
                    if (primitive) {
                        existingObject === null || existingObject === void 0 ? void 0 : existingObject.add(primitive);
                    }
                });
                break;
            }
        }
    }
    static getNewPrimitive(args) {
        const { type, dimensions } = args;
        switch (type) {
            case SOLID_PRIMITIVE_TYPES.BOX:
                const primitive = new Box();
                const [x, y, z] = dimensions;
                primitive.setScale({ x, y, z });
                return primitive;
            default:
                return null;
        }
    }
}

class DisplayTrajectory extends LegacyCore {
    constructor(ros, topicName, options = DEFAULT_OPTIONS_DISPLAYTRAJECTORY) {
        super(ros, topicName, MESSAGE_TYPE_DISPLAYTRAJECTORY, options);
        this.pointsUpdateIds = [];
        this.object = new Group();
        this.updateOptions({
            ...DEFAULT_OPTIONS_DISPLAYTRAJECTORY,
            ...options,
        });
        const { robot } = this.options;
        this.robotClone = robot.clone(true);
        Object.keys(this.robotClone.links).forEach(linkName => {
            const link = this.robotClone.links[linkName];
            link.traverse(child => {
                if (child.material) {
                    child.material = new three.MeshPhongMaterial({
                        color: '#ff0000',
                    });
                }
            });
        });
    }
    updateOptions(options) {
        super.updateOptions(options);
        if (options.loop && this.lastMessage) {
            this.update(this.lastMessage, true);
        }
        else {
            this.resetLoopback();
        }
    }
    resetLoopback() {
        clearTimeout(this.loopbackId);
        clearTimeout(this.poseRemovalId);
        if (this.pointsUpdateIds) {
            this.pointsUpdateIds.map(x => clearTimeout(x));
        }
        this.pointsUpdateIds = [];
        if (this.robotClone && this.robotClone.parent) {
            this.robotClone.parent.remove(this.robotClone);
        }
    }
    update(message, loopback) {
        var _a;
        const { loop } = this.options;
        this.resetLoopback();
        if (!loopback) {
            this.lastMessage = message;
        }
        super.update(message);
        const { trajectory: [{ joint_trajectory: { joint_names: jointNames, points }, },], trajectory_start: { joint_state: { name: initialNames, position: initialPositions }, }, } = message;
        (_a = this.object) === null || _a === void 0 ? void 0 : _a.add(this.robotClone);
        initialNames.forEach((name, index) => {
            const joint = this.robotClone.getObjectByName(name);
            if (joint) {
                joint.setAngle(initialPositions[index]);
            }
        });
        points.forEach(point => {
            const { positions, time_from_start: { nsec, secs }, } = point;
            this.pointsUpdateIds.push(window.setTimeout(() => {
                jointNames.forEach((jointName, index) => {
                    const joint = this.robotClone.getObjectByName(jointName);
                    if (joint) {
                        joint.setAngle(positions[index]);
                    }
                });
            }, 1000 * secs + nsec / 1000000));
        });
        if (points.length > 0) {
            const { time_from_start: { nsec: lastNsec, secs: lastSec }, } = points[points.length - 1];
            this.poseRemovalId = window.setTimeout(() => {
                var _a;
                (_a = this.robotClone.parent) === null || _a === void 0 ? void 0 : _a.remove(this.robotClone);
                if (loop) {
                    this.loopbackId = window.setTimeout(() => {
                        if (this.lastMessage) {
                            this.update(this.lastMessage, true);
                        }
                    }, 1000);
                }
            }, 1000 * lastSec + lastNsec / 1000000);
        }
    }
}

class PlanningScene extends LegacyCore {
    constructor(ros, topicName, options = DEFAULT_OPTIONS_PLANNINGSCENE) {
        super(ros, topicName, MESSAGE_TYPE_PLANNINGSCENE, {
            ...DEFAULT_OPTIONS_PLANNINGSCENE,
            ...options,
        });
        this.collisionObjectViz = new CollisionObject();
        this.attachedCollisionObjects = new Map();
        this.object = new Group();
        this.object.add(this.collisionObjectViz.object);
        this.updateOptions({
            ...DEFAULT_OPTIONS_PLANNINGSCENE,
            ...options,
        });
    }
    update(message) {
        super.update(message);
        const { robot_state: { attached_collision_objects: attachedCollisionObjects = [], is_diff: isRobotStateDiff, joint_state: { name, position }, }, world: { collision_objects: worldCollisionObjects }, } = message;
        worldCollisionObjects.forEach(collisionMessage => {
            this.collisionObjectViz.update(collisionMessage);
        });
        if (attachedCollisionObjects.length) {
            attachedCollisionObjects.map(({ link_name: linkName, object }) => {
                var _a;
                const collisionVizInstance = new CollisionObject();
                const collisionVizObject = (_a = this.object) === null || _a === void 0 ? void 0 : _a.getObjectByName(linkName);
                assertIsDefined(collisionVizObject);
                collisionVizInstance.object = collisionVizObject;
                collisionVizInstance.update(object);
                this.attachedCollisionObjects.set(object.id, collisionVizInstance);
            });
        }
        if (!isRobotStateDiff) {
            const newCollisionObjectNames = attachedCollisionObjects.map(({ object: { id } }) => id);
            for (const [collObjName, collObj,] of this.attachedCollisionObjects.entries()) {
                if (newCollisionObjectNames.indexOf(collObjName) === -1) {
                    collObj.update({
                        id: collObjName,
                        operation: COLLISION_OBJECT_OPERATIONS.REMOVE,
                    });
                    this.attachedCollisionObjects.delete(collObjName);
                }
            }
        }
        name.forEach((jointName, index) => {
            var _a;
            const joint = (_a = this.object) === null || _a === void 0 ? void 0 : _a.getObjectByName(jointName);
            if (joint) {
                joint.setAngle(position[index]);
            }
        });
    }
}

class Cylinder extends Mesh {
    constructor(color = DEFAULT_COLOR_ARROW, radius = DEFAULT_CYLINDER_RADIUS, height = DEFAULT_CYLINDER_HEIGHT) {
        super();
        this.geometry = new three.CylinderGeometry(radius, radius, height, DEFAULT_RADIAL_SEGMENTS);
        this.material = new three.MeshStandardMaterial({ color });
        this.material.transparent = true;
        this.rotateX(Math.PI / 2);
    }
}

class Cone extends Mesh {
    constructor(color = DEFAULT_COLOR_X_AXIS) {
        super();
        this.geometry = new three.ConeGeometry(DEFAULT_CONE_RADIUS, DEFAULT_CONE_HEIGHT, DEFAULT_RADIAL_SEGMENTS);
        this.material = new three.MeshStandardMaterial({ color });
        this.material.transparent = true;
    }
}

class Arrow extends three.Group {
    constructor() {
        super();
        this.cone = new Cone(DEFAULT_COLOR_X_AXIS);
        this.cylinder = new Cylinder(DEFAULT_COLOR_X_AXIS);
        this.cone.rotateZ(-Math.PI / 2);
        this.cylinder.rotateZ(-Math.PI / 2);
        this.cone.setScale({
            x: DEFAULT_CONE_RADIUS,
            y: DEFAULT_CONE_HEIGHT,
            z: DEFAULT_CONE_RADIUS,
        });
        this.cylinder.setScale({
            x: DEFAULT_CYLINDER_RADIUS,
            y: DEFAULT_CYLINDER_HEIGHT,
            z: DEFAULT_CYLINDER_RADIUS,
        });
        this.cylinder.translateY(this.cylinder.scale.y / 2);
        this.cone.translateY(this.cylinder.scale.y + this.cone.scale.y / 2);
        this.add(this.cone);
        this.add(this.cylinder);
    }
    setTransform(transform) {
        setTransform(this, transform);
    }
    setColor(color) {
        const { cone, cylinder } = color;
        if (cone) {
            this.cone.setColor(cone);
        }
        if (cylinder) {
            this.cylinder.setColor(cylinder);
        }
    }
    setHeadDimensions(dimensions) {
        const { radius, length } = dimensions;
        if (radius) {
            const { y } = this.cone.scale;
            this.cone.setScale({ x: radius, y, z: radius });
        }
        if (length) {
            const { x, z } = this.cone.scale;
            this.cone.setScale({ x, y: length, z });
            this.cone.position.set(0, 0, 0);
            this.cone.translateY(this.cylinder.scale.y + length / 2);
        }
    }
    setShaftDimensions(dimensions) {
        const { radius, length } = dimensions;
        if (radius) {
            const { y } = this.cylinder.scale;
            this.cylinder.setScale({ x: radius, y, z: radius });
        }
        if (length) {
            const { x, z } = this.cylinder.scale;
            this.cylinder.setScale({ x, y: length, z });
            this.cylinder.position.set(0, 0, 0);
            this.cylinder.translateY(length / 2);
            this.setHeadDimensions({ length: this.cone.scale.y });
        }
    }
    setAlpha(alpha) {
        this.cylinder.setAlpha(alpha);
        this.cone.setAlpha(alpha);
    }
    setScale(scale) {
        const { x } = scale;
        const [y, z] = [x / 2, x / 2];
        setScale(this, { x, y, z });
    }
}

class Axes extends Group {
    constructor(radius = DEFAULT_CYLINDER_RADIUS, height = DEFAULT_CYLINDER_HEIGHT) {
        super();
        this.objectType = OBJECT_TYPE_AXES;
        this.x = new Cylinder(DEFAULT_COLOR_X_AXIS, radius, height);
        this.y = new Cylinder(DEFAULT_COLOR_Y_AXIS, radius, height);
        this.z = new Cylinder(DEFAULT_COLOR_Z_AXIS, radius, height);
        this.x.translateX(height / 2);
        this.y.translateZ(-height / 2);
        this.z.translateY(height / 2);
        this.x.rotateZ(-Math.PI / 2);
        this.y.rotateX(Math.PI / 2);
        this.add(this.x);
        this.add(this.y);
        this.add(this.z);
    }
    setLength(length) {
        const parsedLength = +`${length}`;
        [this.x, this.y, this.z].forEach(axis => {
            axis.position.set(0, 0, 0);
            axis.scale.setY(parsedLength);
        });
        this.x.translateY(parsedLength / 2);
        this.y.translateY(-parsedLength / 2);
        this.z.translateY(parsedLength / 2);
    }
    setRadius(radius) {
        const parsedRadius = +`${radius}`;
        this.children.forEach(child => {
            child.scale.setX(parsedRadius);
            child.scale.setZ(parsedRadius);
        });
    }
}

class Line extends three.Line {
    constructor(color = DEFAULT_COLOR_LINE, disableVertexColor = false) {
        super();
        this.geometry = new three.Geometry();
        this.geometry.vertices.push(new three.Vector3(0, 0, 0));
        const colorOptions = {};
        if (!disableVertexColor) {
            colorOptions.vertexColors = three.VertexColors;
        }
        this.material = new three.LineBasicMaterial({ ...colorOptions });
        this.material.transparent = true;
        this.setColor(color);
    }
    setColor(color) {
        if (!color) {
            return;
        }
        setColor(this, color);
    }
    updatePoints(points, colors = []) {
        this.geometry.vertices = points.map(({ x, y, z }) => new three.Vector3(x, y, z));
        this.geometry.verticesNeedUpdate = true;
        const color = [];
        colors.forEach(c => {
            if (typeof c === 'string' || typeof c === 'number') {
                color.push(new three.Color(c));
            }
            else if (c) {
                const { r, g, b } = c;
                color.push(new three.Color(r, g, b));
            }
        });
        this.geometry.colors = color;
        this.geometry.colorsNeedUpdate = true;
    }
    setTransform(transform) {
        setTransform(this, transform);
    }
    setAlpha(alpha) {
        assertIsMaterial(this.material);
        this.material.opacity = alpha;
    }
}

class LineArrow extends Group {
    constructor() {
        super();
        this.objectType = OBJECT_TYPE_FLAT_ARROW;
        this.arrowTop = new Line(0xff0000, true);
        this.topPoints = [];
        this.arrowLength = new Line(0xff0000, true);
        this.topPoints.push(new three.Vector3(2, 1, 0));
        this.topPoints.push(new three.Vector3(3, 0, 0));
        this.topPoints.push(new three.Vector3(2, -1, 0));
        this.arrowTop.updatePoints(this.topPoints);
        this.add(this.arrowTop);
        this.arrowLength.updatePoints([new three.Vector3(0, 0, 0), new three.Vector3(3, 0, 0)]);
        this.add(this.arrowLength);
        this.scale.set(0.1, 0.1, 0.1);
    }
    setLength(length) {
        this.scale.set(length, length, length);
    }
    setColor(color) {
        const { r, g, b } = color;
        this.arrowTop.material.color.setRGB(r, g, b);
        this.arrowLength.material.color.setRGB(r, g, b);
    }
}

class LiveCore {
    constructor(args) {
        var _a;
        this.onHeaderChange = (headerFrameId) => { };
        this.headerFrameId = '';
        this.hide = () => {
            assertIsDefined(this.object);
            if (isObject3D(this.object)) {
                this.object.visible = false;
            }
            else if (isHTMLElement(this.object)) {
                this.object.style.visibility = 'hidden';
            }
        };
        this.show = () => {
            assertIsDefined(this.object);
            if (isObject3D(this.object)) {
                this.object.visible = true;
            }
            else if (isHTMLElement(this.object)) {
                this.object.style.visibility = 'visible';
            }
        };
        this.destroy = () => {
            var _a, _b, _c, _d;
            this.unsubscribe();
            if (isObject3D(this.object)) {
                (_b = (_a = this.object) === null || _a === void 0 ? void 0 : _a.parent) === null || _b === void 0 ? void 0 : _b.remove(this.object);
            }
            else if (isHTMLElement(this.object)) {
                (_d = (_c = this.object) === null || _c === void 0 ? void 0 : _c.parentElement) === null || _d === void 0 ? void 0 : _d.removeChild(this.object);
            }
            this.object = undefined;
        };
        this.reset = () => { };
        this.subscribe = () => {
            this.sources.forEach(source => {
                const listener = {
                    next: this.update,
                    error: error => console.log(error),
                    complete: () => { },
                };
                source.addListener(listener);
            });
        };
        this.unsubscribe = () => {
            this.sources.forEach(source => {
                source.removeAllListeners();
            });
        };
        this.sources = args.sources;
        this.options = (_a = args.options) !== null && _a !== void 0 ? _a : {};
        this.update = this.update.bind(this);
        this.updateOptions = this.updateOptions.bind(this);
    }
    update(message) {
        var _a, _b;
        const headerFrameId = (_b = (_a = message.header) === null || _a === void 0 ? void 0 : _a.frame_id) !== null && _b !== void 0 ? _b : '';
        if (headerFrameId !== this.headerFrameId) {
            this.headerFrameId = headerFrameId;
            this.onHeaderChange(this.headerFrameId);
        }
    }
    updateOptions(options) {
        this.options = {
            ...this.options,
            ...options,
        };
    }
    changeSources(sources) {
        this.unsubscribe();
        this.sources = sources;
        this.subscribe();
    }
}

class Pose extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_POSE) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_POSE,
                ...options,
            },
        });
        this.object = new three.Group();
        this.primitive = null;
        this.updateOptions({
            ...DEFAULT_OPTIONS_POSE,
            ...options,
        });
    }
    static getNewPrimitive(options) {
        const { type } = options;
        let newObject = null;
        switch (type) {
            case POSE_OBJECT_TYPES.arrow:
                newObject = new Arrow();
                break;
            case POSE_OBJECT_TYPES.axes:
                newObject = new Axes();
                break;
            case POSE_OBJECT_TYPES.flatArrow:
                newObject = new LineArrow();
                break;
        }
        return newObject;
    }
    updateOptions(options) {
        var _a, _b, _c;
        super.updateOptions(options);
        const { type } = this.options;
        if (((_a = this.primitive) === null || _a === void 0 ? void 0 : _a.type) !== type) {
            (_b = this.object) === null || _b === void 0 ? void 0 : _b.remove(this.primitive);
            this.primitive = null;
        }
        if (!this.primitive) {
            const primitive = Pose.getNewPrimitive(this.options);
            if (primitive) {
                this.primitive = primitive;
                (_c = this.object) === null || _c === void 0 ? void 0 : _c.add(this.primitive);
            }
        }
        setObjectDimension(this.primitive, this.options);
    }
    update(message) {
        var _a;
        super.update(message);
        const { pose: { orientation, position }, } = message;
        (_a = this.primitive) === null || _a === void 0 ? void 0 : _a.setTransform({
            translation: position,
            rotation: orientation,
        });
    }
}

class ArrowWithCircle extends Arrow {
    constructor() {
        super();
        this.circleCone = new Cone(DEFAULT_COLOR_X_AXIS);
        this.material = new three.MeshStandardMaterial({
            color: this.cone.material.color,
        });
        this.add(this.circleCone);
        this.material.transparent = true;
        const torusOptions = DEFAULT_OPTIONS_TORUS;
        const torusGeometry = new three.TorusGeometry(torusOptions.circleRadius, torusOptions.tube, torusOptions.radialSegments, torusOptions.tubularSegments, torusOptions.arc);
        this.torus = new Mesh(torusGeometry, this.material);
        this.add(this.torus);
        this.circleCone.rotateX(torusGeometry.parameters.arc);
        this.circleCone.translateZ(-Math.cos(-torusGeometry.parameters.arc) *
            torusGeometry.parameters.radius);
        this.circleCone.translateY(-Math.sin(-torusGeometry.parameters.arc) *
            torusGeometry.parameters.radius);
        this.circleCone.translateX(this.cylinder.scale.y / 2);
        this.torus.translateX(this.cylinder.scale.y / 2);
        this.torus.rotateY(Math.PI / 2);
    }
    setTorusDimensions(args) {
        const { radius, tube } = args;
        const torusOptions = DEFAULT_OPTIONS_TORUS;
        this.torus.geometry = new three.TorusGeometry(radius, tube, torusOptions.radialSegments, torusOptions.tubularSegments, torusOptions.arc);
        this.torus.position.set(0, 0, 0);
        this.torus.rotation.set(0, 0, 0);
        this.torus.translateX(this.cylinder.scale.y / 2);
        this.torus.rotateY(Math.PI / 2);
    }
    setCircleConeDimensions(args) {
        const { radius, length } = args;
        const parsedRadius = +`${radius}`;
        const parsedLength = +`${length}`;
        assertIsTorusGeometry(this.torus.geometry);
        if (parsedRadius) {
            const { y } = this.circleCone.scale;
            this.circleCone.setScale({ x: radius, y, z: radius });
        }
        if (parsedLength) {
            const { x, z } = this.circleCone.scale;
            this.circleCone.setScale({ x, y: parsedLength, z });
        }
        this.circleCone.position.set(0, 0, 0);
        this.circleCone.rotation.set(0, 0, 0);
        this.circleCone.translateX(this.cylinder.scale.y / 2);
        this.circleCone.translateZ(-Math.cos(-this.torus.geometry.parameters.arc) *
            this.torus.geometry.parameters.radius);
        this.circleCone.translateY(-Math.sin(-this.torus.geometry.parameters.arc) *
            this.torus.geometry.parameters.radius);
        this.circleCone.rotateX(this.torus.geometry.parameters.arc);
    }
    setColor(args) {
        const { cone, cylinder, torus, circleCone } = args;
        if (cone) {
            this.cone.setColor(cone);
        }
        if (cylinder) {
            this.cylinder.setColor(cylinder);
        }
        if (torus) {
            setColor(this.torus, this.cone.material.color);
        }
        if (circleCone) {
            this.circleCone.setColor(circleCone);
        }
    }
    setAlpha(alpha) {
        super.setAlpha(alpha);
        assertIsMaterial(this.torus.material);
        this.torus.material.opacity = three.MathUtils.clamp(alpha, 0, 1);
        this.circleCone.setAlpha(alpha);
    }
}

class Wrench extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_WRENCH) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_WRENCH,
                ...options,
            },
        });
        this.primitiveX = null;
        this.primitiveY = null;
        this.object = new three.Group();
    }
    static getNewPrimitive(options) {
        const { type } = options;
        let newObject = null;
        switch (type) {
            case WRENCH_OBJECT_TYPES.arrow:
                newObject = new Arrow();
                break;
            case WRENCH_OBJECT_TYPES.arrowWithCircle:
                newObject = new ArrowWithCircle();
                break;
        }
        return newObject;
    }
    getOrUpdatePrimitive(primitive, type) {
        var _a, _b;
        const mustUpdatePrimitive = primitive === null || (primitive === null || primitive === void 0 ? void 0 : primitive.type) !== type;
        if (mustUpdatePrimitive) {
            if (primitive) {
                (_a = this.object) === null || _a === void 0 ? void 0 : _a.remove(primitive);
            }
            const object = Wrench.getNewPrimitive({ type });
            if (object) {
                (_b = this.object) === null || _b === void 0 ? void 0 : _b.add(object);
            }
            return object;
        }
        return primitive;
    }
    updateOptions(options) {
        super.updateOptions(options);
        this.primitiveX = this.getOrUpdatePrimitive(this.primitiveX, WRENCH_OBJECT_TYPES.arrow);
        this.primitiveY = this.getOrUpdatePrimitive(this.primitiveY, WRENCH_OBJECT_TYPES.arrowWithCircle);
        const forceOptions = {
            color: this.options.forceColor,
            alpha: this.options.alpha,
            headLength: this.options.headLength * this.options.forceArrowScale,
            headRadius: this.options.headRadius * this.options.arrowWidth,
            shaftLength: this.options.shaftLength * this.options.forceArrowScale,
            shaftRadius: this.options.shaftRadius * this.options.arrowWidth,
            type: WRENCH_OBJECT_TYPES.arrow,
        };
        const torqueOptions = {
            color: this.options.torqueColor,
            alpha: this.options.alpha,
            arc: this.options.arc,
            circleConeRadius: this.options.circleConeRadius * this.options.arrowWidth,
            circleConeLength: this.options.circleConeLength * this.options.arrowWidth,
            circleRadius: this.options.circleRadius * this.options.torqueArrowScale,
            headLength: this.options.headLength * this.options.torqueArrowScale,
            headRadius: this.options.headRadius * this.options.arrowWidth,
            shaftLength: this.options.shaftLength * this.options.torqueArrowScale,
            shaftRadius: this.options.shaftRadius * this.options.arrowWidth,
            tube: this.options.tube * this.options.arrowWidth,
            tubularSegments: this.options.tubularSegments,
            type: WRENCH_OBJECT_TYPES.arrowWithCircle,
        };
        setObjectDimension(this.primitiveX, forceOptions);
        setObjectDimension(this.primitiveY, torqueOptions);
    }
    update(message) {
        var _a, _b;
        super.update(message);
        const { wrench: { force, torque }, } = message;
        const translationVector = new three.Vector3(1, 0, 0);
        const forceVector = new three.Vector3(force.x, force.y, force.z).normalize();
        const torqueVector = new three.Vector3(torque.x, torque.y, torque.z).normalize();
        const forceQuaternion = new three.Quaternion().setFromUnitVectors(translationVector, forceVector);
        const torqueQuaternion = new three.Quaternion().setFromUnitVectors(translationVector, torqueVector);
        (_a = this.primitiveX) === null || _a === void 0 ? void 0 : _a.setTransform({
            translation: { y: 0, x: 0, z: 0 },
            rotation: forceQuaternion,
        });
        (_b = this.primitiveY) === null || _b === void 0 ? void 0 : _b.setTransform({
            translation: { y: 0, x: 0, z: 0 },
            rotation: torqueQuaternion,
        });
    }
}

class Polygon extends LegacyCore {
    constructor(ros, topicName, options = DEFAULT_OPTIONS_POLYGON) {
        super(ros, topicName, MESSAGE_TYPE_POLYGONSTAMPED, {
            ...DEFAULT_OPTIONS_POLYGON,
            ...options,
        });
        this.object = new Group();
        this.line = new Line(null, true);
        this.updateOptions({
            ...DEFAULT_OPTIONS_POLYGON,
            ...options,
        });
    }
    updateOptions(options) {
        super.updateOptions(options);
        const { alpha, color } = this.options;
        this.line.setColor(new three.Color(color));
        this.line.setAlpha(alpha);
    }
    update(message) {
        super.update(message);
        const { polygon: { points }, } = message;
        points.push(points[0]);
        this.line.updatePoints(points);
        this.object.add(this.line);
    }
}

class Text extends Mesh {
    constructor(text) {
        super();
        this.geometry = new three.Geometry();
        this.material = new three.Material();
        const loader = new three.FontLoader();
        this.material = new three.MeshBasicMaterial({ color: 0xdddddd });
        loader.load('https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/fonts/helvetiker_regular.typeface.json', font => {
            this.geometry = new three.TextGeometry(text, {
                font,
                size: 0.05,
                height: 0.005,
                curveSegments: 12,
                bevelEnabled: false,
                bevelThickness: 10,
                bevelSize: 8,
                bevelSegments: 5,
            });
        });
        this.rotateX(Math.PI / 2);
        this.rotateY(Math.PI);
    }
}

class TfFrame extends Group {
    constructor(frameId) {
        super();
        this.arrow = new Arrow();
        this.add(new Axes(0.015, 0.25));
        const textObject = new Text(frameId);
        textObject
            .rotateY(Math.PI)
            .translateX(0.03)
            .translateY(0.03);
        this.add(textObject);
        this.arrow.setHeadDimensions({
            length: (DEFAULT_CONE_HEIGHT * 0.3) / 2,
            radius: (DEFAULT_CONE_RADIUS * 0.1) / 2,
        });
        this.arrow.setShaftDimensions({
            length: DEFAULT_CYLINDER_HEIGHT * 0.85,
            radius: (DEFAULT_CYLINDER_RADIUS * 0.05) / 6,
        });
        this.arrow.setColor({
            cone: new three.Color('#FF1493'),
            cylinder: new three.Color(DEFAULT_COLOR_ARROW),
        });
        this.add(this.arrow);
        this.name = TfFrame.getName(frameId);
    }
    static getName(frameId) {
        return `tf-${frameId}`;
    }
}

class Tf extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_TF) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_TF,
                ...options,
            },
        });
        this.object = new three.Group();
        this.object.name = 'tf-tree';
    }
    update(message) {
        var _a;
        const { transforms } = message;
        transforms.forEach(({ header: { frame_id: parentFrameId }, child_frame_id: childFrameId, transform, }) => {
            const [childFrame, parentFrame] = [
                this.getFrameOrCreate(childFrameId),
                this.getFrameOrCreate(parentFrameId),
            ];
            parentFrame.add(childFrame);
            childFrame.setTransform(transform);
            if (childFrame.position.length() < 0.1) {
                childFrame.arrow.visible = false;
            }
            else {
                childFrame.arrow.lookAt(parentFrame.getWorldPosition(new three.Vector3()));
                childFrame.arrow.rotateY(-Math.PI / 2);
                childFrame.arrow.visible = true;
                const arrowConeLength = childFrame.arrow.cone.scale.y;
                childFrame.arrow.setShaftDimensions({
                    length: childFrame.position.length() - arrowConeLength,
                });
            }
        });
        (_a = this.object) === null || _a === void 0 ? void 0 : _a.children.forEach(child => {
            child.arrow.visible = false;
        });
    }
    getFrameOrCreate(frameId) {
        var _a, _b;
        const existingFrame = (_a = this.object) === null || _a === void 0 ? void 0 : _a.getObjectByName(TfFrame.getName(frameId));
        if (existingFrame) {
            return existingFrame;
        }
        const newFrame = new TfFrame(frameId);
        (_b = this.object) === null || _b === void 0 ? void 0 : _b.add(newFrame);
        return newFrame;
    }
}

var commonjsGlobal = typeof globalThis !== 'undefined' ? globalThis : typeof window !== 'undefined' ? window : typeof global !== 'undefined' ? global : typeof self !== 'undefined' ? self : {};

function unwrapExports (x) {
	return x && x.__esModule && Object.prototype.hasOwnProperty.call(x, 'default') ? x['default'] : x;
}

function createCommonjsModule(fn, module) {
	return module = { exports: {} }, fn(module, module.exports), module.exports;
}

var umd = createCommonjsModule(function (module, exports) {
(function (global, factory) {
	factory(exports, three__default["default"]) ;
}(commonjsGlobal, function (exports, THREE) {
	// Ripped and modified From THREE.js Mesh raycast
	// https://github.com/mrdoob/three.js/blob/0aa87c999fe61e216c1133fba7a95772b503eddf/src/objects/Mesh.js#L115
	var vA = new THREE.Vector3();
	var vB = new THREE.Vector3();
	var vC = new THREE.Vector3();

	var uvA = new THREE.Vector2();
	var uvB = new THREE.Vector2();
	var uvC = new THREE.Vector2();

	var intersectionPoint = new THREE.Vector3();
	var intersectionPointWorld = new THREE.Vector3();

	function checkIntersection( object, material, raycaster, ray, pA, pB, pC, point ) {

		var intersect;
		if ( material.side === THREE.BackSide ) {

			intersect = ray.intersectTriangle( pC, pB, pA, true, point );

		} else {

			intersect = ray.intersectTriangle( pA, pB, pC, material.side !== THREE.DoubleSide, point );

		}

		if ( intersect === null ) return null;

		intersectionPointWorld.copy( point );
		intersectionPointWorld.applyMatrix4( object.matrixWorld );

		var distance = raycaster.ray.origin.distanceTo( intersectionPointWorld );

		if ( distance < raycaster.near || distance > raycaster.far ) return null;

		return {
			distance: distance,
			point: intersectionPointWorld.clone(),
			object: object
		};

	}

	function checkBufferGeometryIntersection( object, raycaster, ray, position, uv, a, b, c ) {

		vA.fromBufferAttribute( position, a );
		vB.fromBufferAttribute( position, b );
		vC.fromBufferAttribute( position, c );

		var intersection = checkIntersection( object, object.material, raycaster, ray, vA, vB, vC, intersectionPoint );

		if ( intersection ) {

			if ( uv ) {

				uvA.fromBufferAttribute( uv, a );
				uvB.fromBufferAttribute( uv, b );
				uvC.fromBufferAttribute( uv, c );

				intersection.uv = THREE.Triangle.getUV( intersectionPoint, vA, vB, vC, uvA, uvB, uvC, new THREE.Vector2( ) );

			}

			var normal = new THREE.Vector3();
			intersection.face = new THREE.Face3( a, b, c, THREE.Triangle.getNormal( vA, vB, vC, normal ) );
			intersection.faceIndex = a;

		}

		return intersection;

	}


	// https://github.com/mrdoob/three.js/blob/0aa87c999fe61e216c1133fba7a95772b503eddf/src/objects/Mesh.js#L258
	function intersectTri( mesh, geo, raycaster, ray, tri, intersections ) {

		const triOffset = tri * 3;
		const a = geo.index.getX( triOffset );
		const b = geo.index.getX( triOffset + 1 );
		const c = geo.index.getX( triOffset + 2 );

		const intersection = checkBufferGeometryIntersection( mesh, raycaster, ray, geo.attributes.position, geo.attributes.uv, a, b, c );

		if ( intersection ) {

			intersection.faceIndex = tri;
			if ( intersections ) intersections.push( intersection );
			return intersection;

		}

		return null;

	}

	function intersectTris( mesh, geo, raycaster, ray, offset, count, intersections ) {

		for ( let i = offset, end = offset + count; i < end; i ++ ) {

			intersectTri( mesh, geo, raycaster, ray, i, intersections );

		}

	}

	function intersectClosestTri( mesh, geo, raycaster, ray, offset, count ) {

		let dist = Infinity;
		let res = null;
		for ( let i = offset, end = offset + count; i < end; i ++ ) {

			const intersection = intersectTri( mesh, geo, raycaster, ray, i );
			if ( intersection && intersection.distance < dist ) {

				res = intersection;
				dist = intersection.distance;

			}

		}

		return res;

	}

	// Returns a Float32Array representing the bounds data for box.
	function boxToArray( bx ) {

		const arr = new Float32Array( 6 );

		arr[ 0 ] = bx.min.x;
		arr[ 1 ] = bx.min.y;
		arr[ 2 ] = bx.min.z;

		arr[ 3 ] = bx.max.x;
		arr[ 4 ] = bx.max.y;
		arr[ 5 ] = bx.max.z;

		return arr;

	}

	function arrayToBox( arr, target ) {

		target.min.x = arr[ 0 ];
		target.min.y = arr[ 1 ];
		target.min.z = arr[ 2 ];

		target.max.x = arr[ 3 ];
		target.max.y = arr[ 4 ];
		target.max.z = arr[ 5 ];

		return target;

	}

	function getLongestEdgeIndex( bounds ) {

		let splitDimIdx = - 1;
		let splitDist = - Infinity;

		for ( let i = 0; i < 3; i ++ ) {

			const dist = bounds[ i + 3 ] - bounds[ i ];
			if ( dist > splitDist ) {

				splitDist = dist;
				splitDimIdx = i;

			}

		}

		return splitDimIdx;

	}

	class SeparatingAxisBounds {

		constructor() {

			this.min = Infinity;
			this.max = - Infinity;

		}

		setFromPointsField( points, field ) {

			let min = Infinity;
			let max = - Infinity;
			for ( let i = 0, l = points.length; i < l; i ++ ) {

				const p = points[ i ];
				const val = p[ field ];
				min = Math.min( val, min );
				max = Math.max( val, max );

			}

			this.min = min;
			this.max = max;


		}

		setFromPoints( axis, points ) {

			let min = Infinity;
			let max = - Infinity;
			for ( let i = 0, l = points.length; i < l; i ++ ) {

				const p = points[ i ];
				const val = axis.dot( p );
				min = Math.min( val, min );
				max = Math.max( val, max );

			}

			this.min = min;
			this.max = max;

		}

		isSeparated( other ) {

			return this.min > other.max || other.min > this.max;

		}

	}

	SeparatingAxisBounds.prototype.setFromBox = ( function () {

		const p = new THREE.Vector3();
		return function setFromBox( axis, box ) {

			const boxMin = box.min;
			const boxMax = box.max;
			let min = Infinity;
			let max = - Infinity;
			for ( let x = 0; x <= 1; x ++ ) {

				for ( let y = 0; y <= 1; y ++ ) {

					for ( let z = 0; z <= 1; z ++ ) {

						p.x = boxMin.x * x + boxMax.x * ( 1 - x );
						p.y = boxMin.y * y + boxMax.y * ( 1 - y );
						p.z = boxMin.z * z + boxMax.z * ( 1 - z );

						const val = axis.dot( p );
						min = Math.min( val, min );
						max = Math.max( val, max );

					}

				}

			}

			this.min = min;
			this.max = max;

		};

	} )();

	( (function () {

		const cacheSatBounds = new SeparatingAxisBounds();
		return function areIntersecting( shape1, shape2 ) {

			const points1 = shape1.points;
			const satAxes1 = shape1.satAxes;
			const satBounds1 = shape1.satBounds;

			const points2 = shape2.points;
			const satAxes2 = shape2.satAxes;
			const satBounds2 = shape2.satBounds;

			// check axes of the first shape
			for ( let i = 0; i < 3; i ++ ) {

				const sb = satBounds1[ i ];
				const sa = satAxes1[ i ];
				cacheSatBounds.setFromPoints( sa, points2 );
				if ( sb.isSeparated( cacheSatBounds ) ) return false;

			}

			// check axes of the second shape
			for ( let i = 0; i < 3; i ++ ) {

				const sb = satBounds2[ i ];
				const sa = satAxes2[ i ];
				cacheSatBounds.setFromPoints( sa, points1 );
				if ( sb.isSeparated( cacheSatBounds ) ) return false;

			}

		};

	}) )();

	const closestPointLineToLine = ( function () {

		// https://github.com/juj/MathGeoLib/blob/master/src/Geometry/Line.cpp#L56
		const dir1 = new THREE.Vector3();
		const dir2 = new THREE.Vector3();
		const v02 = new THREE.Vector3();
		return function closestPointLineToLine( l1, l2, result ) {

			const v0 = l1.start;
			const v10 = dir1;
			const v2 = l2.start;
			const v32 = dir2;

			v02.subVectors( v0, v2 );
			dir1.subVectors( l1.end, l2.start );
			dir2.subVectors( l2.end, l2.start );

			// float d0232 = v02.Dot(v32);
			const d0232 = v02.dot( v32 );

			// float d3210 = v32.Dot(v10);
			const d3210 = v32.dot( v10 );

			// float d3232 = v32.Dot(v32);
			const d3232 = v32.dot( v32 );

			// float d0210 = v02.Dot(v10);
			const d0210 = v02.dot( v10 );

			// float d1010 = v10.Dot(v10);
			const d1010 = v10.dot( v10 );

			// float denom = d1010*d3232 - d3210*d3210;
			const denom = d1010 * d3232 - d3210 * d3210;

			let d, d2;
			if ( denom !== 0 ) {

				d = ( d0232 * d3210 - d0210 * d3232 ) / denom;

			} else {

				d = 0;

			}

			d2 = ( d0232 + d * d3210 ) / d3232;

			result.x = d;
			result.y = d2;

		};

	} )();

	const closestPointsSegmentToSegment = ( function () {

		// https://github.com/juj/MathGeoLib/blob/master/src/Geometry/LineSegment.cpp#L187
		const paramResult = new THREE.Vector2();
		const temp1 = new THREE.Vector3();
		const temp2 = new THREE.Vector3();
		return function closestPointsSegmentToSegment( l1, l2, target1, target2 ) {

			closestPointLineToLine( l1, l2, paramResult );

			let d = paramResult.x;
			let d2 = paramResult.y;
			if ( d >= 0 && d <= 1 && d2 >= 0 && d2 <= 1 ) {

				l1.at( d, target1 );
				l2.at( d2, target2 );

				return;

			} else if ( d >= 0 && d <= 1 ) {

				// Only d2 is out of bounds.
				if ( d2 < 0 ) {

					l2.at( 0, target2 );

				} else {

					l2.at( 1, target2 );

				}

				l1.closestPointToPoint( target2, true, target1 );
				return;

			} else if ( d2 >= 0 && d2 <= 1 ) {

				// Only d is out of bounds.
				if ( d < 0 ) {

					l1.at( 0, target1 );

				} else {

					l1.at( 1, target1 );

				}

				l2.closestPointToPoint( target1, true, target2 );
				return;

			} else {

				// Both u and u2 are out of bounds.
				let p;
				if ( d < 0 ) {

					p = l1.start;

				} else {

					p = l1.end;

				}

				let p2;
				if ( d2 < 0 ) {

					p2 = l2.start;

				} else {

					p2 = l2.end;

				}

				const closestPoint = temp1;
				const closestPoint2 = temp2;
				l1.closestPointToPoint( p2, true, temp1 );
				l2.closestPointToPoint( p, true, temp2 );

				if ( closestPoint.distanceToSquared( p2 ) <= closestPoint2.distanceToSquared( p ) ) {

					target1.copy( closestPoint );
					target2.copy( p2 );
					return;

				} else {

					target1.copy( p );
					target2.copy( closestPoint2 );
					return;

				}

			}

		};

	} )();


	const sphereIntersectTriangle = ( function () {

		// https://stackoverflow.com/questions/34043955/detect-collision-between-sphere-and-triangle-in-three-js
		const closestPointTemp = new THREE.Vector3();
		const projectedPointTemp = new THREE.Vector3();
		const planeTemp = new THREE.Plane();
		const lineTemp = new THREE.Line3();
		return function sphereIntersectTriangle( sphere, triangle ) {

			const { radius, center } = sphere;
			const { a, b, c } = triangle;

			// phase 1
			lineTemp.start = a;
			lineTemp.end = b;
			const closestPoint1 = lineTemp.closestPointToPoint( center, true, closestPointTemp );
			if ( closestPoint1.distanceTo( center ) <= radius ) return true;

			lineTemp.start = a;
			lineTemp.end = c;
			const closestPoint2 = lineTemp.closestPointToPoint( center, true, closestPointTemp );
			if ( closestPoint2.distanceTo( center ) <= radius ) return true;

			lineTemp.start = b;
			lineTemp.end = c;
			const closestPoint3 = lineTemp.closestPointToPoint( center, true, closestPointTemp );
			if ( closestPoint3.distanceTo( center ) <= radius ) return true;

			// phase 2
			const plane = triangle.getPlane( planeTemp );
			const dp = Math.abs( plane.distanceToPoint( center ) );
			if ( dp <= radius ) {

				const pp = plane.projectPoint( center, projectedPointTemp );
				const cp = triangle.containsPoint( pp );
				if ( cp ) return true;

			}

			return false;

		};

	} )();

	class SeparatingAxisTriangle extends THREE.Triangle {

		constructor( ...args ) {

			super( ...args );

			this.isSeparatingAxisTriangle = true;
			this.satAxes = new Array( 4 ).fill().map( () => new THREE.Vector3() );
			this.satBounds = new Array( 4 ).fill().map( () => new SeparatingAxisBounds() );
			this.points = [ this.a, this.b, this.c ];
			this.sphere = new THREE.Sphere();

		}

	}

	SeparatingAxisTriangle.prototype.update = ( function () {

		const arr = new Array( 3 );
		return function update( ) {

			const a = this.a;
			const b = this.b;
			const c = this.c;

			arr[ 0 ] = this.a;
			arr[ 1 ] = this.b;
			arr[ 2 ] = this.c;

			const satAxes = this.satAxes;
			const satBounds = this.satBounds;

			const axis0 = satAxes[ 0 ];
			const sab0 = satBounds[ 0 ];
			this.getNormal( axis0 );
			sab0.setFromPoints( axis0, arr );

			const axis1 = satAxes[ 1 ];
			const sab1 = satBounds[ 1 ];
			axis1.subVectors( a, b );
			sab1.setFromPoints( axis1, arr );

			const axis2 = satAxes[ 2 ];
			const sab2 = satBounds[ 2 ];
			axis2.subVectors( b, c );
			sab2.setFromPoints( axis2, arr );

			const axis3 = satAxes[ 3 ];
			const sab3 = satBounds[ 3 ];
			axis3.subVectors( c, a );
			sab3.setFromPoints( axis3, arr );

			this.sphere.setFromPoints( this.points );

		};

	} )();

	SeparatingAxisTriangle.prototype.intersectsTriangle = ( function () {

		const saTri2 = new SeparatingAxisTriangle();
		const arr1 = new Array( 3 );
		const arr2 = new Array( 3 );
		const cachedSatBounds = new SeparatingAxisBounds();
		const cachedSatBounds2 = new SeparatingAxisBounds();
		const cachedAxis = new THREE.Vector3();
		return function intersectsTriangle( other ) {

			if ( ! other.isSeparatingAxisTriangle ) {

				saTri2.copy( other );
				saTri2.update();
				other = saTri2;

			}

			const satBounds1 = this.satBounds;
			const satAxes1 = this.satAxes;
			arr2[ 0 ] = other.a;
			arr2[ 1 ] = other.b;
			arr2[ 2 ] = other.c;
			for ( let i = 0; i < 4; i ++ ) {

				const sb = satBounds1[ i ];
				const sa = satAxes1[ i ];
				cachedSatBounds.setFromPoints( sa, arr2 );
				if ( sb.isSeparated( cachedSatBounds ) ) return false;

			}

			const satBounds2 = other.satBounds;
			const satAxes2 = other.satAxes;
			arr1[ 0 ] = this.a;
			arr1[ 1 ] = this.b;
			arr1[ 2 ] = this.c;
			for ( let i = 0; i < 4; i ++ ) {

				const sb = satBounds2[ i ];
				const sa = satAxes2[ i ];
				cachedSatBounds.setFromPoints( sa, arr1 );
				if ( sb.isSeparated( cachedSatBounds ) ) return false;

			}

			// check crossed axes
			for ( let i = 0; i < 4; i ++ ) {

				const sa1 = satAxes1[ i ];
				for ( let i2 = 0; i2 < 4; i2 ++ ) {

					const sa2 = satAxes2[ i2 ];
					cachedAxis.crossVectors( sa1, sa2 );
					cachedSatBounds.setFromPoints( cachedAxis, arr1 );
					cachedSatBounds2.setFromPoints( cachedAxis, arr2 );
					if ( cachedSatBounds.isSeparated( cachedSatBounds2 ) ) return false;

				}

			}

			return true;

		};

	} )();


	SeparatingAxisTriangle.prototype.distanceToPoint = ( function () {

		const target = new THREE.Vector3();
		return function distanceToPoint( point ) {

			this.closestPointToPoint( point, target );
			return point.distanceTo( target );

		};

	} )();


	SeparatingAxisTriangle.prototype.distanceToTriangle = ( function () {

		const point = new THREE.Vector3();
		const point2 = new THREE.Vector3();
		const cornerFields = [ 'a', 'b', 'c' ];
		const line1 = new THREE.Line3();
		const line2 = new THREE.Line3();

		return function distanceToTriangle( other, target1 = null, target2 = null ) {

			if ( this.intersectsTriangle( other ) ) {

				// TODO: This will not result in a point that lies on
				// the intersection line of the triangles
				if ( target1 || target2 ) {

					this.getMidpoint( point );
					other.closestPointToPoint( point, point2 );
					this.closestPointToPoint( point2, point );

					if ( target1 ) target1.copy( point );
					if ( target2 ) target2.copy( point2 );

				}

				return 0;

			}

			let closestDistanceSq = Infinity;

			// check all point distances
			for ( let i = 0; i < 3; i ++ ) {

				let dist;
				const field = cornerFields[ i ];
				const otherVec = other[ field ];
				this.closestPointToPoint( otherVec, point );

				dist = otherVec.distanceToSquared( point );

				if ( dist < closestDistanceSq ) {

					closestDistanceSq = dist;
					if ( target1 ) target1.copy( point );
					if ( target2 ) target2.copy( otherVec );

				}


				const thisVec = this[ field ];
				other.closestPointToPoint( thisVec, point );

				dist = thisVec.distanceToSquared( point );

				if ( dist < closestDistanceSq ) {

					closestDistanceSq = dist;
					if ( target1 ) target1.copy( thisVec );
					if ( target2 ) target2.copy( point );

				}

			}

			for ( let i = 0; i < 3; i ++ ) {

				const f11 = cornerFields[ i ];
				const f12 = cornerFields[ ( i + 1 ) % 3 ];
				line1.set( this[ f11 ], this[ f12 ] );
				for ( let i2 = 0; i2 < 3; i2 ++ ) {

					const f21 = cornerFields[ i2 ];
					const f22 = cornerFields[ ( i2 + 1 ) % 3 ];
					line2.set( other[ f21 ], other[ f22 ] );

					closestPointsSegmentToSegment( line1, line2, point, point2 );

					const dist = point.distanceToSquared( point2 );
					if ( dist < closestDistanceSq ) {

						closestDistanceSq = dist;
						if ( target1 ) target1.copy( point );
						if ( target2 ) target2.copy( point2 );

					}

				}

			}

			return Math.sqrt( closestDistanceSq );

		};

	} )();

	class OrientedBox extends THREE.Box3 {

		constructor( ...args ) {

			super( ...args );

			this.isOrientedBox = true;
			this.matrix = new THREE.Matrix4();
			this.invMatrix = new THREE.Matrix4();
			this.points = new Array( 8 ).fill().map( () => new THREE.Vector3() );
			this.satAxes = new Array( 3 ).fill().map( () => new THREE.Vector3() );
			this.satBounds = new Array( 3 ).fill().map( () => new SeparatingAxisBounds() );
			this.alignedSatBounds = new Array( 3 ).fill().map( () => new SeparatingAxisBounds() );
			this.sphere = new THREE.Sphere();

		}

		set( min, max, matrix ) {

			super.set( min, max );
			this.matrix = matrix;

		}

		copy( other ) {

			super.copy( other );
			this.matrix.copy( other.matrix );

		}

	}

	OrientedBox.prototype.update = ( function () {

		return function update() {

			const matrix = this.matrix;
			const min = this.min;
			const max = this.max;

			const points = this.points;
			for ( let x = 0; x <= 1; x ++ ) {

				for ( let y = 0; y <= 1; y ++ ) {

					for ( let z = 0; z <= 1; z ++ ) {

						const i = ( ( 1 << 0 ) * x ) | ( ( 1 << 1 ) * y ) | ( ( 1 << 2 ) * z );
						const v = points[ i ];
						v.x = x ? max.x : min.x;
						v.y = y ? max.y : min.y;
						v.z = z ? max.z : min.z;

						v.applyMatrix4( matrix );

					}

				}

			}

			this.sphere.setFromPoints( this.points );

			const satBounds = this.satBounds;
			const satAxes = this.satAxes;
			const minVec = points[ 0 ];
			for ( let i = 0; i < 3; i ++ ) {

				const axis = satAxes[ i ];
				const sb = satBounds[ i ];
				const index = 1 << i;
				const pi = points[ index ];

				axis.subVectors( minVec, pi );
				sb.setFromPoints( axis, points );

			}

			const alignedSatBounds = this.alignedSatBounds;
			alignedSatBounds[ 0 ].setFromPointsField( points, 'x' );
			alignedSatBounds[ 1 ].setFromPointsField( points, 'y' );
			alignedSatBounds[ 2 ].setFromPointsField( points, 'z' );

			this.invMatrix.getInverse( this.matrix );

		};

	} )();

	OrientedBox.prototype.intersectsBox = ( function () {

		const aabbBounds = new SeparatingAxisBounds();
		return function intersectsBox( box ) {

			if ( ! box.intersectsSphere( this.sphere ) ) return false;

			const min = box.min;
			const max = box.max;
			const satBounds = this.satBounds;
			const satAxes = this.satAxes;
			const alignedSatBounds = this.alignedSatBounds;

			aabbBounds.min = min.x;
			aabbBounds.max = max.x;
			if ( alignedSatBounds[ 0 ].isSeparated( aabbBounds ) ) return false;

			aabbBounds.min = min.y;
			aabbBounds.max = max.y;
			if ( alignedSatBounds[ 1 ].isSeparated( aabbBounds ) ) return false;

			aabbBounds.min = min.z;
			aabbBounds.max = max.z;
			if ( alignedSatBounds[ 2 ].isSeparated( aabbBounds ) ) return false;

			for ( let i = 0; i < 3; i ++ ) {

				const axis = satAxes[ i ];
				const sb = satBounds[ i ];
				aabbBounds.setFromBox( axis, box );
				if ( sb.isSeparated( aabbBounds ) ) return false;

			}

			return true;

		};

	} )();

	OrientedBox.prototype.intersectsTriangle = ( function () {

		const saTri = new SeparatingAxisTriangle();
		const pointsArr = new Array( 3 );
		const cachedSatBounds = new SeparatingAxisBounds();
		const cachedSatBounds2 = new SeparatingAxisBounds();
		const cachedAxis = new THREE.Vector3();
		return function intersectsTriangle( triangle ) {

			if ( ! triangle.isSeparatingAxisTriangle ) {

				saTri.copy( triangle );
				saTri.update();
				triangle = saTri;

			}

			const satBounds = this.satBounds;
			const satAxes = this.satAxes;

			pointsArr[ 0 ] = triangle.a;
			pointsArr[ 1 ] = triangle.b;
			pointsArr[ 2 ] = triangle.c;

			for ( let i = 0; i < 3; i ++ ) {

				const sb = satBounds[ i ];
				const sa = satAxes[ i ];
				cachedSatBounds.setFromPoints( sa, pointsArr );
				if ( sb.isSeparated( cachedSatBounds ) ) return false;

			}

			const triSatBounds = triangle.satBounds;
			const triSatAxes = triangle.satAxes;
			const points = this.points;
			for ( let i = 0; i < 3; i ++ ) {

				const sb = triSatBounds[ i ];
				const sa = triSatAxes[ i ];
				cachedSatBounds.setFromPoints( sa, points );
				if ( sb.isSeparated( cachedSatBounds ) ) return false;

			}

			// check crossed axes
			for ( let i = 0; i < 3; i ++ ) {

				const sa1 = satAxes[ i ];
				for ( let i2 = 0; i2 < 4; i2 ++ ) {

					const sa2 = triSatAxes[ i2 ];
					cachedAxis.crossVectors( sa1, sa2 );
					cachedSatBounds.setFromPoints( cachedAxis, pointsArr );
					cachedSatBounds2.setFromPoints( cachedAxis, points );
					if ( cachedSatBounds.isSeparated( cachedSatBounds2 ) ) return false;

				}

			}

			return true;

		};

	} )();

	OrientedBox.prototype.closestPointToPoint = ( function () {

		return function closestPointToPoint( point, target1 ) {

			target1
				.copy( point )
				.applyMatrix4( this.invMatrix )
				.clamp( this.min, this.max )
				.applyMatrix4( this.matrix );

			return target1;

		};

	} )();

	OrientedBox.prototype.distanceToPoint = ( function () {

		const target = new THREE.Vector3();
		return function distanceToPoint( point ) {

			this.closestPointToPoint( point, target );
			return point.distanceTo( target );

		};

	} )();


	OrientedBox.prototype.distanceToBox = ( function () {

		const xyzFields = [ 'x', 'y', 'z' ];
		const segments1 = new Array( 12 ).fill().map( () => new THREE.Line3() );
		const segments2 = new Array( 12 ).fill().map( () => new THREE.Line3() );

		const point1 = new THREE.Vector3();
		const point2 = new THREE.Vector3();

		return function distanceToBox( box, threshold = 0, target1 = null, target2 = null ) {

			if ( this.intersectsBox( box ) ) {

				if ( target1 || target2 ) {

					box.getCenter( point2 );
					this.closestPointToPoint( point2, point1 );
					box.closestPointToPoint( point1, point2 );

					if ( target1 ) target1.copy( point1 );
					if ( target2 ) target2.copy( point2 );

				}
				return 0;

			}

			const threshold2 = threshold * threshold;
			const min = box.min;
			const max = box.max;
			const points = this.points;


			// iterate over every edge and compare distances
			let closestDistanceSq = Infinity;

			// check over all these points
			for ( let i = 0; i < 8; i ++ ) {

				const p = points[ i ];
				point2.copy( p ).clamp( min, max );

				const dist = p.distanceToSquared( point2 );
				if ( dist < closestDistanceSq ) {

					closestDistanceSq = dist;
					if ( target1 ) target1.copy( p );
					if ( target2 ) target2.copy( point2 );

					if ( dist < threshold2 ) return Math.sqrt( dist );

				}

			}

			// generate and check all line segment distances
			let count = 0;
			for ( let i = 0; i < 3; i ++ ) {

				for ( let i1 = 0; i1 <= 1; i1 ++ ) {

					for ( let i2 = 0; i2 <= 1; i2 ++ ) {

						const nextIndex = ( i + 1 ) % 3;
						const nextIndex2 = ( i + 2 ) % 3;

						// get obb line segments
						const index = i1 << nextIndex | i2 << nextIndex2;
						const index2 = 1 << i | i1 << nextIndex | i2 << nextIndex2;
						const p1 = points[ index ];
						const p2 = points[ index2 ];
						const line1 = segments1[ count ];
						line1.set( p1, p2 );


						// get aabb line segments
						const f1 = xyzFields[ i ];
						const f2 = xyzFields[ nextIndex ];
						const f3 = xyzFields[ nextIndex2 ];
						const line2 = segments2[ count ];
						const start = line2.start;
						const end = line2.end;

						start[ f1 ] = min[ f1 ];
						start[ f2 ] = i1 ? min[ f2 ] : max[ f2 ];
						start[ f3 ] = i2 ? min[ f3 ] : max[ f2 ];

						end[ f1 ] = max[ f1 ];
						end[ f2 ] = i1 ? min[ f2 ] : max[ f2 ];
						end[ f3 ] = i2 ? min[ f3 ] : max[ f2 ];

						count ++;

					}

				}

			}

			// check all the other boxes point
			for ( let x = 0; x <= 1; x ++ ) {

				for ( let y = 0; y <= 1; y ++ ) {

					for ( let z = 0; z <= 1; z ++ ) {

						point2.x = x ? max.x : min.x;
						point2.y = y ? max.y : min.y;
						point2.z = z ? max.z : min.z;

						this.closestPointToPoint( point2, point1 );
						const dist = point2.distanceToSquared( point1 );
						if ( dist < closestDistanceSq ) {

							closestDistanceSq = dist;
							if ( target1 ) target1.copy( point1 );
							if ( target2 ) target2.copy( point2 );

							if ( dist < threshold2 ) return Math.sqrt( dist );

						}

					}

				}

			}

			for ( let i = 0; i < 12; i ++ ) {

				const l1 = segments1[ i ];
				for ( let i2 = 0; i2 < 12; i2 ++ ) {

					const l2 = segments2[ i2 ];
					closestPointsSegmentToSegment( l1, l2, point1, point2 );
					const dist = point1.distanceToSquared( point2 );
					if ( dist < closestDistanceSq ) {

						closestDistanceSq = dist;
						if ( target1 ) target1.copy( point1 );
						if ( target2 ) target2.copy( point2 );

						if ( dist < threshold2 ) return Math.sqrt( dist );

					}

				}

			}

			return Math.sqrt( closestDistanceSq );

		};

	} )();

	const boundingBox = new THREE.Box3();
	const boxIntersection = new THREE.Vector3();
	const xyzFields = [ 'x', 'y', 'z' ];

	function setTriangle( tri, i, index, pos ) {

		const ta = tri.a;
		const tb = tri.b;
		const tc = tri.c;

		let i3 = index.getX( i );
		ta.x = pos.getX( i3 );
		ta.y = pos.getY( i3 );
		ta.z = pos.getZ( i3 );

		i3 = index.getX( i + 1 );
		tb.x = pos.getX( i3 );
		tb.y = pos.getY( i3 );
		tb.z = pos.getZ( i3 );

		i3 = index.getX( i + 2 );
		tc.x = pos.getX( i3 );
		tc.y = pos.getY( i3 );
		tc.z = pos.getZ( i3 );

	}

	class MeshBVHNode {

		constructor() {

			// internal nodes have boundingData, left, right, and splitAxis
			// leaf nodes have offset and count (referring to primitives in the mesh geometry)

		}

		intersectRay( ray, target ) {

			arrayToBox( this.boundingData, boundingBox );

			return ray.intersectBox( boundingBox, target );

		}

		raycast( mesh, raycaster, ray, intersects ) {

			if ( this.count ) intersectTris( mesh, mesh.geometry, raycaster, ray, this.offset, this.count, intersects );
			else {

				if ( this.left.intersectRay( ray, boxIntersection ) )
					this.left.raycast( mesh, raycaster, ray, intersects );
				if ( this.right.intersectRay( ray, boxIntersection ) )
					this.right.raycast( mesh, raycaster, ray, intersects );

			}

		}

		raycastFirst( mesh, raycaster, ray ) {

			if ( this.count ) {

				return intersectClosestTri( mesh, mesh.geometry, raycaster, ray, this.offset, this.count );

			} else {


				// consider the position of the split plane with respect to the oncoming ray; whichever direction
				// the ray is coming from, look for an intersection among that side of the tree first
				const splitAxis = this.splitAxis;
				const xyzAxis = xyzFields[ splitAxis ];
				const rayDir = ray.direction[ xyzAxis ];
				const leftToRight = rayDir >= 0;

				// c1 is the child to check first
				let c1, c2;
				if ( leftToRight ) {

					c1 = this.left;
					c2 = this.right;

				} else {

					c1 = this.right;
					c2 = this.left;

				}

				const c1Intersection = c1.intersectRay( ray, boxIntersection );
				const c1Result = c1Intersection ? c1.raycastFirst( mesh, raycaster, ray ) : null;

				// if we got an intersection in the first node and it's closer than the second node's bounding
				// box, we don't need to consider the second node because it couldn't possibly be a better result
				if ( c1Result ) {

					// check only along the split axis
					const rayOrig = ray.origin[ xyzAxis ];
					const toPoint = rayOrig - c1Result.point[ xyzAxis ];
					const toChild1 = rayOrig - c2.boundingData[ splitAxis ];
					const toChild2 = rayOrig - c2.boundingData[ splitAxis + 3 ];

					const toPointSq = toPoint * toPoint;
					if ( toPointSq <= toChild1 * toChild1 && toPointSq <= toChild2 * toChild2 ) {

						return c1Result;

					}

				}

				// either there was no intersection in the first node, or there could still be a closer
				// intersection in the second, so check the second node and then take the better of the two
				const c2Intersection = c2.intersectRay( ray, boxIntersection );
				const c2Result = c2Intersection ? c2.raycastFirst( mesh, raycaster, ray ) : null;

				if ( c1Result && c2Result ) {

					return c1Result.distance <= c2Result.distance ? c1Result : c2Result;

				} else {

					return c1Result || c2Result || null;

				}

			}

		}

	}

	MeshBVHNode.prototype.shapecast = ( function () {

		const triangle = new SeparatingAxisTriangle();
		const cachedBox1 = new THREE.Box3();
		const cachedBox2 = new THREE.Box3();
		return function shapecast( mesh, intersectsBoundsFunc, intersectsTriangleFunc = null, nodeScoreFunc = null ) {

			if ( this.count && intersectsTriangleFunc ) {

				const geometry = mesh.geometry;
				const index = geometry.index;
				const pos = geometry.attributes.position;
				const offset = this.offset;
				const count = this.count;

				for ( let i = offset * 3, l = ( count + offset ) * 3; i < l; i += 3 ) {

					setTriangle( triangle, i, index, pos );
					triangle.update();

					if ( intersectsTriangleFunc( triangle, i, i + 1, i + 2 ) ) {

						return true;

					}

				}

				return false;

			} else {

				const left = this.left;
				const right = this.right;
				let c1 = left;
				let c2 = right;

				let score1, score2;
				let box1, box2;
				if ( nodeScoreFunc ) {

					box1 = cachedBox1;
					box2 = cachedBox2;

					arrayToBox( c1.boundingData, box1 );
					arrayToBox( c2.boundingData, box2 );

					score1 = nodeScoreFunc( box1 );
					score2 = nodeScoreFunc( box2 );

					if ( score2 < score1 ) {

						c1 = right;
						c2 = left;

						const temp = score1;
						score1 = score2;
						score2 = temp;

						const tempBox = box1;
						box1 = box2;
						box2 = tempBox;

					}

				}

				if ( ! box1 ) {

					box1 = cachedBox1;
					arrayToBox( c1.boundingData, box1 );

				}

				const isC1Leaf = ! ! c1.count;
				const c1Intersection =
					intersectsBoundsFunc( box1, isC1Leaf, score1, c1 ) &&
					c1.shapecast( mesh, intersectsBoundsFunc, intersectsTriangleFunc, nodeScoreFunc );

				if ( c1Intersection ) return true;


				if ( ! box2 ) {

					box2 = cachedBox2;
					arrayToBox( c2.boundingData, box2 );

				}

				const isC2Leaf = ! ! c2.count;
				const c2Intersection =
					intersectsBoundsFunc( box2, isC2Leaf, score2, c2 ) &&
					c2.shapecast( mesh, intersectsBoundsFunc, intersectsTriangleFunc, nodeScoreFunc );

				if ( c2Intersection ) return true;

				return false;

			}

		};

	} )();

	MeshBVHNode.prototype.intersectsGeometry = ( function () {

		const triangle = new SeparatingAxisTriangle();
		const triangle2 = new SeparatingAxisTriangle();
		const cachedMesh = new THREE.Mesh();
		const invertedMat = new THREE.Matrix4();

		const obb = new OrientedBox();
		const obb2 = new OrientedBox();

		return function intersectsGeometry( mesh, geometry, geometryToBvh, cachedObb = null ) {

			if ( cachedObb === null ) {

				if ( ! geometry.boundingBox ) {

					geometry.computeBoundingBox();

				}

				obb.set( geometry.boundingBox.min, geometry.boundingBox.max, geometryToBvh );
				obb.update();
				cachedObb = obb;

			}

			if ( this.count ) {

				const thisGeometry = mesh.geometry;
				const thisIndex = thisGeometry.index;
				const thisPos = thisGeometry.attributes.position;

				const index = geometry.index;
				const pos = geometry.attributes.position;

				const offset = this.offset;
				const count = this.count;

				// get the inverse of the geometry matrix so we can transform our triangles into the
				// geometry space we're trying to test. We assume there are fewer triangles being checked
				// here.
				invertedMat.getInverse( geometryToBvh );

				if ( geometry.boundsTree ) {

					arrayToBox( this.boundingData, obb2 );
					obb2.matrix.copy( invertedMat );
					obb2.update();

					cachedMesh.geometry = geometry;
					const res = geometry.boundsTree.shapecast( cachedMesh, box => obb2.intersectsBox( box ), function ( tri ) {

						tri.a.applyMatrix4( geometryToBvh );
						tri.b.applyMatrix4( geometryToBvh );
						tri.c.applyMatrix4( geometryToBvh );
						tri.update();

						for ( let i = offset * 3, l = ( count + offset ) * 3; i < l; i += 3 ) {

							// this triangle needs to be transformed into the current BVH coordinate frame
							setTriangle( triangle2, i, thisIndex, thisPos );
							triangle2.update();
							if ( tri.intersectsTriangle( triangle2 ) ) {

								return true;

							}

						}

						return false;

					} );
					cachedMesh.geometry = null;

					return res;

				} else {

					for ( let i = offset * 3, l = ( count + offset * 3 ); i < l; i += 3 ) {

						// this triangle needs to be transformed into the current BVH coordinate frame
						setTriangle( triangle, i, thisIndex, thisPos );
						triangle.a.applyMatrix4( invertedMat );
						triangle.b.applyMatrix4( invertedMat );
						triangle.c.applyMatrix4( invertedMat );
						triangle.update();

						for ( let i2 = 0, l2 = index.count; i2 < l2; i2 += 3 ) {

							setTriangle( triangle2, i2, index, pos );
							triangle2.update();

							if ( triangle.intersectsTriangle( triangle2 ) ) {

								return true;

							}

						}

					}

				}

			} else {

				const left = this.left;
				const right = this.right;

				arrayToBox( left.boundingData, boundingBox );
				const leftIntersection =
					cachedObb.intersectsBox( boundingBox ) &&
					left.intersectsGeometry( mesh, geometry, geometryToBvh, cachedObb );

				if ( leftIntersection ) return true;


				arrayToBox( right.boundingData, boundingBox );
				const rightIntersection =
					cachedObb.intersectsBox( boundingBox ) &&
					right.intersectsGeometry( mesh, geometry, geometryToBvh, cachedObb );

				if ( rightIntersection ) return true;

				return false;

			}

		};

	} )();

	MeshBVHNode.prototype.intersectsBox = ( function () {

		const obb = new OrientedBox();

		return function intersectsBox( mesh, box, boxToBvh ) {

			obb.set( box.min, box.max, boxToBvh );
			obb.update();

			return this.shapecast(
				mesh,
				box => obb.intersectsBox( box ),
				tri => obb.intersectsTriangle( tri )
			);

		};

	} )();

	MeshBVHNode.prototype.intersectsSphere = ( function () {

		return function intersectsSphere( mesh, sphere ) {

			return this.shapecast(
				mesh,
				box => sphere.intersectsBox( box ),
				tri => sphereIntersectTriangle( sphere, tri )
			);

		};

	} )();

	MeshBVHNode.prototype.closestPointToPoint = ( function () {

		// early out if under minThreshold
		// skip checking if over maxThreshold
		// set minThreshold = maxThreshold to quickly check if a point is within a threshold
		// returns Infinity if no value found

		const temp = new THREE.Vector3();
		return function closestPointToPoint( mesh, point, target = null, minThreshold = 0, maxThreshold = Infinity ) {

			let closestDistance = Infinity;
			this.shapecast(

				mesh,
				( box, isLeaf, score ) => score < closestDistance && score < maxThreshold,
				tri => {

					tri.closestPointToPoint( point, temp );
					const dist = point.distanceTo( temp );
					if ( dist < closestDistance ) {

						if ( target ) target.copy( temp );
						closestDistance = dist;

					}
					if ( dist < minThreshold ) return true;
					return false;

				},
				box => box.distanceToPoint( point )

			);

			return closestDistance;

		};

	} )();

	MeshBVHNode.prototype.closestPointToGeometry = ( function () {

		// early out if under minThreshold
		// skip checking if over maxThreshold
		// set minThreshold = maxThreshold to quickly check if a point is within a threshold
		// returns Infinity if no value found

		const tri2 = new SeparatingAxisTriangle();
		const obb = new OrientedBox();

		const temp1 = new THREE.Vector3();
		const temp2 = new THREE.Vector3();
		return function closestPointToGeometry( mesh, geometry, geometryToBvh, target1 = null, target2 = null, minThreshold = 0, maxThreshold = Infinity ) {

			if ( ! geometry.boundingBox ) geometry.computeBoundingBox();
			obb.set( geometry.boundingBox.min, geometry.boundingBox.max, geometryToBvh );
			obb.update();

			const pos = geometry.attributes.position;
			const index = geometry.index;

			let tempTarget1, tempTarget2;
			if ( target1 ) tempTarget1 = temp1;
			if ( target2 ) tempTarget2 = temp2;

			let closestDistance = Infinity;
			this.shapecast(
				mesh,
				( box, isLeaf, score ) => score < closestDistance && score < maxThreshold,
				tri => {

					const sphere1 = tri.sphere;
					for ( let i2 = 0, l2 = index.count; i2 < l2; i2 += 3 ) {

						setTriangle( tri2, i2, index, pos );
						tri2.a.applyMatrix4( geometryToBvh );
						tri2.b.applyMatrix4( geometryToBvh );
						tri2.c.applyMatrix4( geometryToBvh );
						tri2.sphere.setFromPoints( tri2.points );

						const sphere2 = tri2.sphere;
						const sphereDist = sphere2.center.distanceTo( sphere1.center ) - sphere2.radius - sphere1.radius;
						if ( sphereDist > closestDistance ) continue;

						tri2.update();

						const dist = tri.distanceToTriangle( tri2, tempTarget1, tempTarget2 );
						if ( dist < closestDistance ) {

							if ( target1 ) target1.copy( tempTarget1 );
							if ( target2 ) target2.copy( tempTarget2 );
							closestDistance = dist;

						}
						if ( dist < minThreshold ) return true;

					}

					return false;

				},
				box => obb.distanceToBox( box, Math.min( closestDistance, maxThreshold ) )

			);

			return closestDistance;

		};

	} )();

	// Split strategy constants
	const CENTER = 0;
	const AVERAGE = 1;
	const SAH = 2;

	const xyzFields$1 = [ 'x', 'y', 'z' ];

	// precomputes the bounding box for each triangle; required for quickly calculating tree splits.
	// result is an array of size tris.length * 6 where triangle i maps to a
	// [x_center, x_delta, y_center, y_delta, z_center, z_delta] tuple starting at index i * 6,
	// representing the center and half-extent in each dimension of triangle i
	function computeBounds( geo ) {

		const verts = geo.attributes.position.array;
		const index = geo.index.array;
		const triCount = index.length / 3;
		const bounds = new Float32Array( triCount * 6 );

		for ( let tri = 0; tri < triCount; tri ++ ) {

			const ai = index[ 3 * tri + 0 ] * 3;
			const bi = index[ 3 * tri + 1 ] * 3;
			const ci = index[ 3 * tri + 2 ] * 3;

			for ( let el = 0; el < 3; el ++ ) {

				const a = verts[ ai + el ];
				const b = verts[ bi + el ];
				const c = verts[ ci + el ];
				const min = Math.min( a, b, c );
				const max = Math.max( a, b, c );
				const halfExtents = ( max - min ) / 2;
				bounds[ tri * 6 + el * 2 + 0 ] = min + halfExtents;
				bounds[ tri * 6 + el * 2 + 1 ] = halfExtents;

			}

		}

		return bounds;

	}

	const boxtemp = new THREE.Box3();

	class BVHConstructionContext {

		constructor( geo, options ) {

			this.geo = geo;
			this.options = options;
			this.bounds = computeBounds( geo );

			// SAH Initialization
			this.sahplanes = null;
			if ( options.strategy === SAH ) {

				const triCount = geo.index.count / 3;
				this.sahplanes = [ new Array( triCount ), new Array( triCount ), new Array( triCount ) ];
				for ( let tri = 0; tri < triCount; tri ++ ) {

					for ( let el = 0; el < 3; el ++ ) {

						this.sahplanes[ el ][ tri ] = { p: this.bounds[ tri * 6 + el * 2 ], tri };

					}

				}

			}

		}

		// returns the average coordinate on the specified axis of the all the provided triangles
		getAverage( offset, count, axis ) {

			let avg = 0;
			const bounds = this.bounds;

			for ( let i = offset, end = offset + count; i < end; i ++ ) {

				avg += bounds[ i * 6 + axis * 2 ];

			}

			return avg / count;

		}

		// computes the union of the bounds of all of the given triangles and puts the resulting box in target
		getBounds( offset, count, target ) {

			let minx = Infinity;
			let miny = Infinity;
			let minz = Infinity;
			let maxx = - Infinity;
			let maxy = - Infinity;
			let maxz = - Infinity;
			const bounds = this.bounds;

			for ( let i = offset, end = offset + count; i < end; i ++ ) {

				const cx = bounds[ i * 6 + 0 ];
				const hx = bounds[ i * 6 + 1 ];
				minx = Math.min( minx, cx - hx );
				maxx = Math.max( maxx, cx + hx );
				const cy = bounds[ i * 6 + 2 ];
				const hy = bounds[ i * 6 + 3 ];
				miny = Math.min( miny, cy - hy );
				maxy = Math.max( maxy, cy + hy );
				const cz = bounds[ i * 6 + 4 ];
				const hz = bounds[ i * 6 + 5 ];
				minz = Math.min( minz, cz - hz );
				maxz = Math.max( maxz, cz + hz );

			}

			target[ 0 ] = minx;
			target[ 1 ] = miny;
			target[ 2 ] = minz;

			target[ 3 ] = maxx;
			target[ 4 ] = maxy;
			target[ 5 ] = maxz;

			return target;

		}

		// reorders `tris` such that for `count` elements after `offset`, elements on the left side of the split
		// will be on the left and elements on the right side of the split will be on the right. returns the index
		// of the first element on the right side, or offset + count if there are no elements on the right side.
		partition( offset, count, split ) {

			let left = offset;
			let right = offset + count - 1;
			const pos = split.pos;
			const axisOffset = split.axis * 2;
			const index = this.geo.index.array;
			const bounds = this.bounds;
			const sahplanes = this.sahplanes;

			// hoare partitioning, see e.g. https://en.wikipedia.org/wiki/Quicksort#Hoare_partition_scheme
			while ( true ) {

				while ( left <= right && bounds[ left * 6 + axisOffset ] < pos ) {

					left ++;

				}

				while ( left <= right && bounds[ right * 6 + axisOffset ] >= pos ) {

					right --;

				}

				if ( left < right ) {

					// we need to swap all of the information associated with the triangles at index
					// left and right; that's the verts in the geometry index, the bounds,
					// and perhaps the SAH planes

					for ( let i = 0; i < 3; i ++ ) {

						let t0 = index[ left * 3 + i ];
						index[ left * 3 + i ] = index[ right * 3 + i ];
						index[ right * 3 + i ] = t0;
						let t1 = bounds[ left * 6 + i * 2 + 0 ];
						bounds[ left * 6 + i * 2 + 0 ] = bounds[ right * 6 + i * 2 + 0 ];
						bounds[ right * 6 + i * 2 + 0 ] = t1;
						let t2 = bounds[ left * 6 + i * 2 + 1 ];
						bounds[ left * 6 + i * 2 + 1 ] = bounds[ right * 6 + i * 2 + 1 ];
						bounds[ right * 6 + i * 2 + 1 ] = t2;

					}

					if ( sahplanes ) {

						for ( let i = 0; i < 3; i ++ ) {

							let t = sahplanes[ i ][ left ];
							sahplanes[ i ][ left ] = sahplanes[ i ][ right ];
							sahplanes[ i ][ right ] = t;

						}

					}

					left ++;
					right --;

				} else {

					return left;

				}

			}

		}

		getOptimalSplit( bounds, offset, count, strategy ) {

			let axis = - 1;
			let pos = 0;

			// Center
			if ( strategy === CENTER ) {

				axis = getLongestEdgeIndex( bounds );
				if ( axis !== - 1 ) {

					pos = ( bounds[ axis + 3 ] + bounds[ axis ] ) / 2;

				}

			} else if ( strategy === AVERAGE ) {

				axis = getLongestEdgeIndex( bounds );
				if ( axis !== - 1 ) {

					pos = this.getAverage( offset, count, axis );

				}

			} else if ( strategy === SAH ) {

				// Surface Area Heuristic
				// In order to make this code more terse, the x, y, and z
				// variables of various structures have been stuffed into
				// 0, 1, and 2 array indices so they can be easily computed
				// and accessed within array iteration

				// Cost values defineed for operations. We're using bounds for traversal, so
				// the cost of traversing one more layer is more than intersecting a triangle.
				const TRAVERSAL_COST = 3;
				const INTERSECTION_COST = 1;
				const bb = arrayToBox( bounds, boxtemp );

				// Define the width, height, and depth of the bounds as a box
				const dim = [
					bb.max.x - bb.min.x,
					bb.max.y - bb.min.y,
					bb.max.z - bb.min.z
				];
				const sa = 2 * ( dim[ 0 ] * dim[ 1 ] + dim[ 0 ] * dim[ 2 ] + dim[ 1 ] * dim[ 2 ] );

				// Get the precalculated planes based for the triangles we're
				// testing here
				const filteredLists = [[], [], []];
				for ( let i = offset, end = offset + count; i < end; i ++ ) {

					for ( let v = 0; v < 3; v ++ ) {

						filteredLists[ v ].push( this.sahplanes[ v ][ i ] );

					}

				}
				filteredLists.forEach( planes => planes.sort( ( a, b ) => a.p - b.p ) );

				// this bounds surface area, left bound SA, left triangles, right bound SA, right triangles
				const getCost = ( sa, sal, nl, sar, nr ) =>
					  TRAVERSAL_COST + INTERSECTION_COST * ( ( sal / sa ) * nl + ( sar / sa ) * nr );

				// the cost of _not_ splitting into smaller bounds
				const noSplitCost = INTERSECTION_COST * count;

				axis = - 1;
				let bestCost = noSplitCost;
				for ( let i = 0; i < 3; i ++ ) {

					// o1 and o2 represent the _other_ two axes in the
					// the space. So if we're checking the x (0) dimension,
					// then o1 and o2 would be y and z (1 and 2)
					const o1 = ( i + 1 ) % 3;
					const o2 = ( i + 2 ) % 3;

					const bmin = bb.min[ xyzFields$1[ i ] ];
					const bmax = bb.max[ xyzFields$1[ i ] ];
					const planes = filteredLists[ i ];

					// The number of left and right triangles on either side
					// given the current split
					let nl = 0;
					let nr = count;
					for ( let p = 0; p < planes.length; p ++ ) {

						const pinfo = planes[ p ];

						// As the plane moves, we have to increment or decrement the
						// number of triangles on either side of the plane
						nl ++;
						nr --;

						// the distance from the plane to the edge of the broader bounds
						const ldim = pinfo.p - bmin;
						const rdim = bmax - pinfo.p;

						// same for the other two dimensions
						let ldimo1 = dim[ o1 ], rdimo1 = dim[ o1 ];
						let ldimo2 = dim[ o2 ], rdimo2 = dim[ o2 ];

						/*
						// compute the other bounding planes for the box
						// if only the current triangles are considered to
						// be in the box
						// This is really slow and probably not really worth it
						const o1planes = this.sahplanes[o1];
						const o2planes = this.sahplanes[o2];
						let lmin = Infinity, lmax = -Infinity;
						let rmin = Infinity, rmax = -Infinity;
						planes.forEach((p, i) => {
						const tri2 = p.tri * 2;
						const inf1 = o1planes[tri2 + 0];
						const inf2 = o1planes[tri2 + 1];
						if (i <= nl) {
						lmin = Math.min(inf1.p, inf2.p, lmin);
						lmax = Math.max(inf1.p, inf2.p, lmax);
						}
						if (i >= nr) {
						rmin = Math.min(inf1.p, inf2.p, rmin);
						rmax = Math.max(inf1.p, inf2.p, rmax);
						}
						})
						ldimo1 = Math.min(lmax - lmin, ldimo1);
						rdimo1 = Math.min(rmax - rmin, rdimo1);

						planes.forEach((p, i) => {
						const tri2 = p.tri * 2;
						const inf1 = o2planes[tri2 + 0];
						const inf2 = o2planes[tri2 + 1];
						if (i <= nl) {
						lmin = Math.min(inf1.p, inf2.p, lmin);
						lmax = Math.max(inf1.p, inf2.p, lmax);
						}
						if (i >= nr) {
						rmin = Math.min(inf1.p, inf2.p, rmin);
						rmax = Math.max(inf1.p, inf2.p, rmax);
						}
						})
						ldimo2 = Math.min(lmax - lmin, ldimo2);
						rdimo2 = Math.min(rmax - rmin, rdimo2);
						*/

						// surface areas and cost
						const sal = 2 * ( ldimo1 * ldimo2 + ldimo1 * ldim + ldimo2 * ldim );
						const sar = 2 * ( rdimo1 * rdimo2 + rdimo1 * rdim + rdimo2 * rdim );
						const cost = getCost( sa, sal, nl, sar, nr );

						if ( cost < bestCost ) {

							axis = i;
							pos = pinfo.p;
							bestCost = cost;

						}

					}

				}

			}

			return { axis, pos };

		}

	}

	class MeshBVH {

		constructor( geo, options = {} ) {

			if ( ! geo.isBufferGeometry ) {

				throw new Error( 'MeshBVH: Only BufferGeometries are supported.' );

			} else if ( geo.attributes.position.isInterleavedBufferAttribute ) {

				throw new Error( 'MeshBVH: InterleavedBufferAttribute is not supported for the position attribute.' );

			} else if ( geo.index && geo.index.isInterleavedBufferAttribute ) {

				throw new Error( 'MeshBVH: InterleavedBufferAttribute is not supported for the index attribute.' );

			}

			// default options
			options = Object.assign( {

				strategy: CENTER,
				maxDepth: 40,
				maxLeafTris: 10,
				verbose: true

			}, options );
			options.strategy = Math.max( 0, Math.min( 2, options.strategy ) );

			this._roots = this._buildTree( geo, options );


		}

		/* Private Functions */

		_ensureIndex( geo ) {

			if ( ! geo.index ) {

				const vertexCount = geo.attributes.position.count;
				const index = new ( vertexCount > 65535 ? Uint32Array : Uint16Array )( vertexCount );
				geo.setIndex( new THREE.BufferAttribute( index, 1 ) );

				for ( let i = 0; i < vertexCount; i ++ ) {

					index[ i ] = i;

				}

			}

		}

		// Computes the set of { offset, count } ranges which need independent BVH roots. Each
		// region in the geometry index that belongs to a different set of material groups requires
		// a separate BVH root, so that triangles indices belonging to one group never get swapped
		// with triangle indices belongs to another group. For example, if the groups were like this:
		//
		// [-------------------------------------------------------------]
		// |__________________|
		//   g0 = [0, 20]  |______________________||_____________________|
		//                      g1 = [16, 40]           g2 = [41, 60]
		//
		// we would need four BVH roots: [0, 15], [16, 20], [21, 40], [41, 60].
		//
		_getRootIndexRanges( geo ) {

			if ( ! geo.groups || ! geo.groups.length ) {

				return [ { offset: 0, count: geo.index.count / 3 } ];

			}

			const ranges = [];
			const rangeBoundaries = new Set();
			for ( const group of geo.groups ) {

				rangeBoundaries.add( group.start );
				rangeBoundaries.add( group.start + group.count );

			}

			// note that if you don't pass in a comparator, it sorts them lexicographically as strings :-(
			const sortedBoundaries = Array.from( rangeBoundaries.values() ).sort( ( a, b ) => a - b );
			for ( let i = 0; i < sortedBoundaries.length - 1; i ++ ) {

				const start = sortedBoundaries[ i ], end = sortedBoundaries[ i + 1 ];
				ranges.push( { offset: ( start / 3 ), count: ( end - start ) / 3 } );

			}
			return ranges;

		}

		_buildTree( geo, options ) {

			this._ensureIndex( geo );

			const ctx = new BVHConstructionContext( geo, options );
			let reachedMaxDepth = false;

			// either recursively splits the given node, creating left and right subtrees for it, or makes it a leaf node,
			// recording the offset and count of its triangles and writing them into the reordered geometry index.
			const splitNode = ( node, offset, count, depth = 0 ) => {

				if ( depth >= options.maxDepth ) {

					reachedMaxDepth = true;

				}

				// early out if we've met our capacity
				if ( count <= options.maxLeafTris || depth >= options.maxDepth ) {

					node.offset = offset;
					node.count = count;
					return node;

				}

				// Find where to split the volume
				const split = ctx.getOptimalSplit( node.boundingData, offset, count, options.strategy );
				if ( split.axis === - 1 ) {

					node.offset = offset;
					node.count = count;
					return node;

				}

				const splitOffset = ctx.partition( offset, count, split );

				// create the two new child nodes
				if ( splitOffset === offset || splitOffset === offset + count ) {

					node.offset = offset;
					node.count = count;

				} else {

					node.splitAxis = split.axis;

					// create the left child and compute its bounding box
					const left = node.left = new MeshBVHNode();
					const lstart = offset, lcount = splitOffset - offset;
					left.boundingData = ctx.getBounds( lstart, lcount, new Float32Array( 6 ) );
					splitNode( left, lstart, lcount, depth + 1 );

					// repeat for right
					const right = node.right = new MeshBVHNode();
					const rstart = splitOffset, rcount = count - lcount;
					right.boundingData = ctx.getBounds( rstart, rcount, new Float32Array( 6 ) );
					splitNode( right, rstart, rcount, depth + 1 );

				}

				return node;

			};

			const roots = [];
			const ranges = this._getRootIndexRanges( geo );

			if ( ranges.length === 1 ) {

				const root = new MeshBVHNode();
				const range = ranges[ 0 ];

				if ( geo.boundingBox != null ) {

					root.boundingData = boxToArray( geo.boundingBox );

				} else {

					root.boundingData = ctx.getBounds( range.offset, range.count, new Float32Array( 6 ) );

				}

				splitNode( root, range.offset, range.count );
				roots.push( root );

			} else {

				for ( let range of ranges ) {

					const root = new MeshBVHNode();
					root.boundingData = ctx.getBounds( range.offset, range.count, new Float32Array( 6 ) );
					splitNode( root, range.offset, range.count );
					roots.push( root );

				}

			}

			if ( reachedMaxDepth && options.verbose ) {

				console.warn( `MeshBVH: Max depth of ${ options.maxDepth } reached when generating BVH. Consider increasing maxDepth.` );
				console.warn( this, geo );

			}

			// if the geometry doesn't have a bounding box, then let's politely populate it using
			// the work we did to determine the BVH root bounds

			if ( geo.boundingBox == null ) {

				const rootBox = new THREE.Box3();
				geo.boundingBox = new THREE.Box3();

				for ( let root of roots ) {

					geo.boundingBox.union( arrayToBox( root.boundingData, rootBox ) );

				}

			}

			return roots;

		}

		raycast( mesh, raycaster, ray, intersects ) {

			for ( const root of this._roots ) {

				root.raycast( mesh, raycaster, ray, intersects );

			}

		}

		raycastFirst( mesh, raycaster, ray ) {

			let closestResult = null;

			for ( const root of this._roots ) {

				const result = root.raycastFirst( mesh, raycaster, ray );
				if ( result != null && ( closestResult == null || result.distance < closestResult.distance ) ) {

					closestResult = result;

				}

			}

			return closestResult;

		}

		intersectsGeometry( mesh, geometry, geomToMesh ) {

			for ( const root of this._roots ) {

				if ( root.intersectsGeometry( mesh, geometry, geomToMesh ) ) return true;

			}

			return false;

		}

		shapecast( mesh, intersectsBoundsFunc, intersectsTriangleFunc = null, orderNodesFunc = null ) {

			for ( const root of this._roots ) {

				if ( root.shapecast( mesh, intersectsBoundsFunc, intersectsTriangleFunc, orderNodesFunc ) ) return true;

			}

			return false;

		}

		intersectsBox( mesh, box, boxToMesh ) {

			for ( const root of this._roots ) {

				if ( root.intersectsBox( mesh, box, boxToMesh ) ) return true;

			}

			return false;

		}

		intersectsSphere( mesh, sphere ) {

			for ( const root of this._roots ) {

				if ( root.intersectsSphere( mesh, sphere ) ) return true;

			}

			return false;

		}

		closestPointToGeometry( mesh, geom, matrix, target1, target2, minThreshold, maxThreshold ) {

			let closestDistance = Infinity;
			for ( const root of this._roots ) {

				const dist = root.closestPointToGeometry( mesh, geom, matrix, target1, target2, minThreshold, maxThreshold );
				if ( dist < closestDistance ) closestDistance = dist;
				if ( dist < minThreshold ) return dist;

			}

			return closestDistance;

		}

		distanceToGeometry( mesh, geom, matrix, minThreshold, maxThreshold ) {

			return this.closestPointToGeometry( mesh, geom, matrix, null, null, minThreshold, maxThreshold );

		}

		closestPointToPoint( mesh, point, target, minThreshold, maxThreshold ) {

			let closestDistance = Infinity;
			for ( const root of this._roots ) {

				const dist = root.closestPointToPoint( mesh, point, target, minThreshold, maxThreshold );
				if ( dist < closestDistance ) closestDistance = dist;
				if ( dist < minThreshold ) return dist;

			}

			return closestDistance;

		}

		distanceToPoint( mesh, point, minThreshold, maxThreshold ) {

			return this.closestPointToPoint( mesh, point, null, minThreshold, maxThreshold );

		}

	}

	const wiremat = new THREE.LineBasicMaterial( { color: 0x00FF88, transparent: true, opacity: 0.3 } );
	const boxGeom = new THREE.Box3Helper().geometry;
	let boundingBox$1 = new THREE.Box3();

	class MeshBVHRootVisualizer extends THREE.Object3D {

		constructor( mesh, depth = 10, group = 0 ) {

			super( 'MeshBVHRootVisualizer' );

			this.depth = depth;
			this._oldDepth = - 1;
			this._mesh = mesh;
			this._boundsTree = null;
			this._group = group;

			this.update();

		}

		update() {

			if ( this._mesh.geometry.boundsTree !== this._boundsTree || this._oldDepth !== this.depth ) {

				this._oldDepth = this.depth;
				this._boundsTree = this._mesh.geometry.boundsTree;

				let requiredChildren = 0;
				if ( this._boundsTree ) {

					const recurse = ( n, d ) => {

						let isLeaf = 'count' in n;

						if ( d === this.depth ) return;

						if ( d === this.depth - 1 || isLeaf ) {

							let m = requiredChildren < this.children.length ? this.children[ requiredChildren ] : null;
							if ( ! m ) {

								m = new THREE.LineSegments( boxGeom, wiremat );
								m.raycast = () => [];
								this.add( m );

							}
							requiredChildren ++;
							arrayToBox( n.boundingData, boundingBox$1 );
							boundingBox$1.getCenter( m.position );
							m.scale.subVectors( boundingBox$1.max, boundingBox$1.min ).multiplyScalar( 0.5 );

							if ( m.scale.x === 0 ) m.scale.x = Number.EPSILON;
							if ( m.scale.y === 0 ) m.scale.y = Number.EPSILON;
							if ( m.scale.z === 0 ) m.scale.z = Number.EPSILON;

						}

						if ( ! isLeaf ) {

							recurse( n.left, d + 1 );
							recurse( n.right, d + 1 );

						}

					};

					recurse( this._boundsTree._roots[ this._group ], 0 );

				}

				while ( this.children.length > requiredChildren ) this.remove( this.children.pop() );

			}

		}

	}

	class MeshBVHVisualizer extends THREE.Object3D {

		constructor( mesh, depth = 10 ) {

			super( 'MeshBVHVisualizer' );

			this.depth = depth;
			this._mesh = mesh;
			this._roots = [];

			this.update();

		}

		update() {

			const bvh = this._mesh.geometry.boundsTree;
			const totalRoots = bvh ? bvh._roots.length : 0;
			while ( this._roots.length > totalRoots ) {

				this._roots.pop();

			}

			for ( let i = 0; i < totalRoots; i ++ ) {

				if ( i >= this._roots.length ) {

					const root = new MeshBVHRootVisualizer( this._mesh, this.depth, i );
					this.add( root );
					this._roots.push( root );

				} else {

					let root = this._roots[ i ];
					root.depth = this.depth;
					root.update();

				}

			}

			this.position.copy( this._mesh.position );
			this.rotation.copy( this._mesh.rotation );
			this.scale.copy( this._mesh.scale );

		}

	}

	const ray = new THREE.Ray();
	const tmpInverseMatrix = new THREE.Matrix4();
	const origMeshRaycastFunc = THREE.Mesh.prototype.raycast;

	function acceleratedRaycast( raycaster, intersects ) {

		if ( this.geometry.boundsTree ) {

			if ( this.material === undefined ) return;

			tmpInverseMatrix.getInverse( this.matrixWorld );
			ray.copy( raycaster.ray ).applyMatrix4( tmpInverseMatrix );

			if ( raycaster.firstHitOnly === true ) {

				const res = this.geometry.boundsTree.raycastFirst( this, raycaster, ray );
				if ( res ) intersects.push( res );

			} else {

				this.geometry.boundsTree.raycast( this, raycaster, ray, intersects );

			}

		} else {

			origMeshRaycastFunc.call( this, raycaster, intersects );

		}

	}

	function computeBoundsTree( options ) {

		this.boundsTree = new MeshBVH( this, options );
		return this.boundsTree;

	}

	function disposeBoundsTree() {

		this.boundsTree = null;

	}

	exports.MeshBVH = MeshBVH;
	exports.Visualizer = MeshBVHVisualizer;
	exports.acceleratedRaycast = acceleratedRaycast;
	exports.computeBoundsTree = computeBoundsTree;
	exports.disposeBoundsTree = disposeBoundsTree;
	exports.CENTER = CENTER;
	exports.AVERAGE = AVERAGE;
	exports.SAH = SAH;

	Object.defineProperty(exports, '__esModule', { value: true });

}));

});

unwrapExports(umd);

var URDFLoader = createCommonjsModule(function (module, exports) {
(function (global, factory) {
  module.exports = factory(three__default["default"], STLLoader__default["default"], ColladaLoader__default["default"], umd) ;
}(commonjsGlobal, function (THREE, STLLoader_js, ColladaLoader_js, threeMeshBvh) {
  function _typeof(obj) {
    if (typeof Symbol === "function" && typeof Symbol.iterator === "symbol") {
      _typeof = function (obj) {
        return typeof obj;
      };
    } else {
      _typeof = function (obj) {
        return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj;
      };
    }

    return _typeof(obj);
  }

  function _classCallCheck(instance, Constructor) {
    if (!(instance instanceof Constructor)) {
      throw new TypeError("Cannot call a class as a function");
    }
  }

  function _defineProperties(target, props) {
    for (var i = 0; i < props.length; i++) {
      var descriptor = props[i];
      descriptor.enumerable = descriptor.enumerable || false;
      descriptor.configurable = true;
      if ("value" in descriptor) descriptor.writable = true;
      Object.defineProperty(target, descriptor.key, descriptor);
    }
  }

  function _createClass(Constructor, protoProps, staticProps) {
    if (protoProps) _defineProperties(Constructor.prototype, protoProps);
    if (staticProps) _defineProperties(Constructor, staticProps);
    return Constructor;
  }

  function _inherits(subClass, superClass) {
    if (typeof superClass !== "function" && superClass !== null) {
      throw new TypeError("Super expression must either be null or a function");
    }

    subClass.prototype = Object.create(superClass && superClass.prototype, {
      constructor: {
        value: subClass,
        writable: true,
        configurable: true
      }
    });
    if (superClass) _setPrototypeOf(subClass, superClass);
  }

  function _getPrototypeOf(o) {
    _getPrototypeOf = Object.setPrototypeOf ? Object.getPrototypeOf : function _getPrototypeOf(o) {
      return o.__proto__ || Object.getPrototypeOf(o);
    };
    return _getPrototypeOf(o);
  }

  function _setPrototypeOf(o, p) {
    _setPrototypeOf = Object.setPrototypeOf || function _setPrototypeOf(o, p) {
      o.__proto__ = p;
      return o;
    };

    return _setPrototypeOf(o, p);
  }

  function _assertThisInitialized(self) {
    if (self === void 0) {
      throw new ReferenceError("this hasn't been initialised - super() hasn't been called");
    }

    return self;
  }

  function _possibleConstructorReturn(self, call) {
    if (call && (typeof call === "object" || typeof call === "function")) {
      return call;
    }

    return _assertThisInitialized(self);
  }

  function _superPropBase(object, property) {
    while (!Object.prototype.hasOwnProperty.call(object, property)) {
      object = _getPrototypeOf(object);
      if (object === null) break;
    }

    return object;
  }

  function _get(target, property, receiver) {
    if (typeof Reflect !== "undefined" && Reflect.get) {
      _get = Reflect.get;
    } else {
      _get = function _get(target, property, receiver) {
        var base = _superPropBase(target, property);

        if (!base) return;
        var desc = Object.getOwnPropertyDescriptor(base, property);

        if (desc.get) {
          return desc.get.call(receiver);
        }

        return desc.value;
      };
    }

    return _get(target, property, receiver || target);
  }

  function _slicedToArray(arr, i) {
    return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _nonIterableRest();
  }

  function _toConsumableArray(arr) {
    return _arrayWithoutHoles(arr) || _iterableToArray(arr) || _nonIterableSpread();
  }

  function _arrayWithoutHoles(arr) {
    if (Array.isArray(arr)) {
      for (var i = 0, arr2 = new Array(arr.length); i < arr.length; i++) arr2[i] = arr[i];

      return arr2;
    }
  }

  function _arrayWithHoles(arr) {
    if (Array.isArray(arr)) return arr;
  }

  function _iterableToArray(iter) {
    if (Symbol.iterator in Object(iter) || Object.prototype.toString.call(iter) === "[object Arguments]") return Array.from(iter);
  }

  function _iterableToArrayLimit(arr, i) {
    if (!(Symbol.iterator in Object(arr) || Object.prototype.toString.call(arr) === "[object Arguments]")) {
      return;
    }

    var _arr = [];
    var _n = true;
    var _d = false;
    var _e = undefined;

    try {
      for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) {
        _arr.push(_s.value);

        if (i && _arr.length === i) break;
      }
    } catch (err) {
      _d = true;
      _e = err;
    } finally {
      try {
        if (!_n && _i["return"] != null) _i["return"]();
      } finally {
        if (_d) throw _e;
      }
    }

    return _arr;
  }

  function _nonIterableSpread() {
    throw new TypeError("Invalid attempt to spread non-iterable instance");
  }

  function _nonIterableRest() {
    throw new TypeError("Invalid attempt to destructure non-iterable instance");
  }

  function URDFColliderClone() {
    var _proto$clone;

    var proto = Object.getPrototypeOf(this);

    for (var _len = arguments.length, args = new Array(_len), _key = 0; _key < _len; _key++) {
      args[_key] = arguments[_key];
    }

    var result = (_proto$clone = proto.clone).call.apply(_proto$clone, [this].concat(args));

    result.isURDFCollider = true;
    return result;
  }

  function makeURDFCollider(object) {
    object.isURDFCollider = true;
    object.clone = URDFColliderClone;
  }

  var URDFLink =
  /*#__PURE__*/
  function (_Object3D) {
    _inherits(URDFLink, _Object3D);

    function URDFLink() {
      var _getPrototypeOf2;

      var _this;

      _classCallCheck(this, URDFLink);

      for (var _len2 = arguments.length, args = new Array(_len2), _key2 = 0; _key2 < _len2; _key2++) {
        args[_key2] = arguments[_key2];
      }

      _this = _possibleConstructorReturn(this, (_getPrototypeOf2 = _getPrototypeOf(URDFLink)).call.apply(_getPrototypeOf2, [this].concat(args)));
      _this.isURDFLink = true;
      _this.type = 'URDFLink';
      _this.urdfNode = null;
      return _this;
    }

    _createClass(URDFLink, [{
      key: "show",
      value: function show() {
        this.visible = true;
      }
    }, {
      key: "hide",
      value: function hide() {
        this.visible = false;
      }
    }, {
      key: "delete",
      value: function _delete() {
        var _this2 = this;

        this.parent.remove(this);
        this.children.map(function (child) {
          _this2.remove(child);
        });
      }
    }, {
      key: "copy",
      value: function copy(source, recursive) {
        _get(_getPrototypeOf(URDFLink.prototype), "copy", this).call(this, source, recursive);

        this.urdfNode = source.urdfNode;
        return this;
      }
    }]);

    return URDFLink;
  }(THREE.Object3D);

  var URDFJoint =
  /*#__PURE__*/
  function (_Object3D2) {
    _inherits(URDFJoint, _Object3D2);

    _createClass(URDFJoint, [{
      key: "jointType",
      get: function get() {
        return this._jointType;
      },
      set: function set(v) {
        if (this.jointType === v) return;
        this._jointType = v;

        switch (v) {
          case 'fixed':
          case 'continuous':
          case 'revolute':
          case 'prismatic':
            this.jointValue = 0;
            break;

          case 'planar':
            this.jointValue = new Array(2).fill(0);
            break;

          case 'floating':
            this.jointValue = new Array(6).fill(0);
            break;
        }
      }
    }, {
      key: "angle",
      get: function get() {
        return this.jointValue;
      }
    }]);

    function URDFJoint() {
      var _getPrototypeOf3;

      var _this3;

      _classCallCheck(this, URDFJoint);

      for (var _len3 = arguments.length, args = new Array(_len3), _key3 = 0; _key3 < _len3; _key3++) {
        args[_key3] = arguments[_key3];
      }

      _this3 = _possibleConstructorReturn(this, (_getPrototypeOf3 = _getPrototypeOf(URDFJoint)).call.apply(_getPrototypeOf3, [this].concat(args)));
      _this3.isURDFJoint = true;
      _this3.type = 'URDFJoint';
      _this3.urdfNode = null;
      _this3.jointValue = null;
      _this3.jointType = 'fixed';
      _this3.axis = null;
      _this3.limit = {
        lower: 0,
        upper: 0
      };
      _this3.ignoreLimits = false;
      _this3.origPosition = null;
      _this3.origQuaternion = null;
      return _this3;
    }
    /* Overrides */


    _createClass(URDFJoint, [{
      key: "copy",
      value: function copy(source, recursive) {
        _get(_getPrototypeOf(URDFJoint.prototype), "copy", this).call(this, source, recursive);

        this.urdfNode = source.urdfNode;
        this.jointType = source.jointType;
        this.axis = source.axis ? source.axis.clone() : null;
        this.limit.lower = source.limit.lower;
        this.limit.upper = source.limit.upper;
        this.ignoreLimits = false;
        this.jointValue = Array.isArray(source.jointValue) ? _toConsumableArray(source.jointValue) : source.jointValue;
        this.origPosition = source.origPosition ? source.origPosition.clone() : null;
        this.origQuaternion = source.origQuaternion ? source.origQuaternion.clone() : null;
        return this;
      }
      /* Public Functions */

    }, {
      key: "setAngle",
      value: function setAngle() {
        return this.setOffset.apply(this, arguments);
      }
    }, {
      key: "setOffset",
      value: function setOffset() {
        for (var _len4 = arguments.length, values = new Array(_len4), _key4 = 0; _key4 < _len4; _key4++) {
          values[_key4] = arguments[_key4];
        }

        values = values.map(function (v) {
          return parseFloat(v);
        });

        if (!this.origPosition || !this.origQuaternion) {
          this.origPosition = this.position.clone();
          this.origQuaternion = this.quaternion.clone();
        }

        switch (this.jointType) {
          case 'fixed':
            {
              break;
            }

          case 'continuous':
          case 'revolute':
            {
              var angle = values[0];
              if (angle == null) break;
              if (angle === this.jointValue) break;

              if (!this.ignoreLimits && this.jointType === 'revolute') {
                angle = Math.min(this.limit.upper, angle);
                angle = Math.max(this.limit.lower, angle);
              } // FromAxisAngle seems to rotate the opposite of the
              // expected angle for URDF, so negate it here


              var delta = new THREE.Quaternion().setFromAxisAngle(this.axis, angle);
              this.quaternion.multiplyQuaternions(this.origQuaternion, delta);
              this.jointValue = angle;
              this.matrixWorldNeedsUpdate = true;
              break;
            }

          case 'prismatic':
            {
              var _angle = values[0];
              if (_angle == null) break;
              if (_angle === this.jointValue) break;

              if (!this.ignoreLimits) {
                _angle = Math.min(this.limit.upper, _angle);
                _angle = Math.max(this.limit.lower, _angle);
              }

              this.position.copy(this.origPosition);
              this.position.addScaledVector(this.axis, _angle);
              this.jointValue = _angle;
              this.worldMatrixNeedsUpdate = true;
              break;
            }

          case 'floating':
          case 'planar':
            // TODO: Support these joint types
            console.warn("'".concat(this.jointType, "' joint not yet supported"));
        }

        return this.jointValue;
      }
    }]);

    return URDFJoint;
  }(THREE.Object3D);

  var URDFRobot =
  /*#__PURE__*/
  function (_URDFLink) {
    _inherits(URDFRobot, _URDFLink);

    function URDFRobot() {
      var _getPrototypeOf4;

      var _this4;

      _classCallCheck(this, URDFRobot);

      for (var _len5 = arguments.length, args = new Array(_len5), _key5 = 0; _key5 < _len5; _key5++) {
        args[_key5] = arguments[_key5];
      }

      _this4 = _possibleConstructorReturn(this, (_getPrototypeOf4 = _getPrototypeOf(URDFRobot)).call.apply(_getPrototypeOf4, [this].concat(args)));
      _this4.isURDFRobot = true;
      _this4.urdfNode = null;
      _this4.urdfRobotNode = null;
      _this4.robotName = null;
      _this4.links = null;
      _this4.joints = null;
      return _this4;
    }

    _createClass(URDFRobot, [{
      key: "copy",
      value: function copy(source, recursive) {
        var _this5 = this;

        _get(_getPrototypeOf(URDFRobot.prototype), "copy", this).call(this, source, recursive);

        this.urdfRobotNode = source.urdfRobotNode;
        this.robotName = source.robotName;
        this.links = {};
        this.joints = {};
        this.traverse(function (c) {
          if (c.isURDFJoint && c.name in source.joints) {
            _this5.joints[c.name] = c;
          }

          if (c.isURDFLink && c.name in source.links) {
            _this5.links[c.name] = c;
          }
        });
        return this;
      }
    }, {
      key: "setAngle",
      value: function setAngle(jointName) {
        var joint = this.joints[jointName];

        if (joint) {
          for (var _len6 = arguments.length, angle = new Array(_len6 > 1 ? _len6 - 1 : 0), _key6 = 1; _key6 < _len6; _key6++) {
            angle[_key6 - 1] = arguments[_key6];
          }

          return joint.setAngle.apply(joint, angle);
        }

        return null;
      }
    }, {
      key: "setAngles",
      value: function setAngles(angles) {
        // TODO: How to handle other, multi-dimensional joint types?
        for (var name in angles) {
          this.setAngle(name, angles[name]);
        }
      }
    }]);

    return URDFRobot;
  }(URDFLink);

  /*
  Reference coordinate frames for THREE.js and ROS.
  Both coordinate systems are right handed so the URDF is instantiated without
  frame transforms. The resulting model can be rotated to rectify the proper up,
  right, and forward directions

  THREE.js
     Y
     |
     |
     .-----X
   ／
  Z

  ROS URDf
         Z
         |   X
         | ／
   Y-----.

  */

  var tempQuaternion = new THREE.Quaternion();
  var tempEuler = new THREE.Euler(); // take a vector "x y z" and process it into
  // an array [x, y, z]

  function processTuple(val) {
    if (!val) return [0, 0, 0];
    return val.trim().split(/\s+/g).map(function (num) {
      return parseFloat(num);
    });
  } // applies a rotation a threejs object in URDF order


  function applyRotation(obj, rpy) {
    var additive = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : false;
    // if additive is true the rotation is applied in
    // addition to the existing rotation
    if (!additive) obj.rotation.set(0, 0, 0);
    tempEuler.set(rpy[0], rpy[1], rpy[2], 'ZYX');
    tempQuaternion.setFromEuler(tempEuler);
    tempQuaternion.multiply(obj.quaternion);
    obj.quaternion.copy(tempQuaternion);
  }
  /* URDFLoader Class */
  // Loads and reads a URDF file into a THREEjs Object3D format


  var URDFLoader =
  /*#__PURE__*/
  function () {
    function URDFLoader(manager) {
      var allowMeshBVH = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;

      _classCallCheck(this, URDFLoader);

      this.manager = manager || THREE.DefaultLoadingManager;
      this.allowMeshBVH = allowMeshBVH;
      this.retryMap = {};
    }
    /* Public API */
    // urdf:    The path to the URDF within the package OR absolute
    // onComplete:      Callback that is passed the model once loaded


    _createClass(URDFLoader, [{
      key: "load",
      value: function load(urdf, onComplete, onProgress, onError, options) {
        var _this = this;

        // Check if a full URI is specified before
        // prepending the package info
        var manager = this.manager;
        var workingPath = THREE.LoaderUtils.extractUrlBase(urdf);
        var urdfPath = this.manager.resolveURL(urdf);
        var errors = {};

        var managerOnErrorDefault = function managerOnErrorDefault() {};

        var managerOnProgressDefault = function managerOnProgressDefault() {};

        var managerOnLoadDefault = function managerOnLoadDefault() {};

        var model;

        if (manager.onError) {
          managerOnErrorDefault = manager.onProgress.bind(manager);
        }

        if (manager.onProgress) {
          managerOnProgressDefault = manager.onProgress.bind(manager);
        }

        if (manager.onLoad) {
          managerOnLoadDefault = manager.onLoad.bind(manager);
        }

        var that = this;

        manager.onError = function (url) {
          errors[url] = 'Error in loading resource';

          if (onError) {
            onError({
              url: url,
              retry: that.retryMap[url]
            });
          }

          managerOnErrorDefault(url);
        };

        manager.onProgress = function (url, itemsLoaded, itemsTotal) {
          if (onProgress) {
            onProgress(url, itemsLoaded, itemsTotal);
          }

          managerOnProgressDefault(url, itemsLoaded, itemsTotal);
        };

        manager.onLoad = function () {
          if (onComplete) {
            var partialErrors = Object.keys(errors).length === 0 ? undefined : errors;
            onComplete(model, partialErrors);
          }

          managerOnLoadDefault();
        };

        options = Object.assign({
          workingPath: workingPath
        }, options);
        manager.itemStart(urdfPath);
        fetch(urdfPath, options.fetchOptions).then(function (res) {
          return res.text();
        }).then(function (data) {
          model = _this.parse(data, options);
          window.model = model;
          manager.itemEnd(urdfPath);
        })["catch"](function (e) {
          console.error('URDFLoader: Error parsing file.', e);
          manager.itemError(urdfPath);
          manager.itemEnd(urdfPath);
        });
      }
    }, {
      key: "parse",
      value: function parse(content) {
        var _this2 = this;

        var options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};
        var packages = options.packages || '';
        var loadMeshCb = options.loadMeshCb || this.defaultMeshLoader.bind(this);
        var workingPath = options.workingPath || '';
        var parseVisual = 'parseVisual' in options ? options.parseVisual : true;
        var parseCollision = options.parseCollision || false;
        var manager = this.manager;
        var linkMap = {};
        var jointMap = {};
        var materialMap = {}; // Resolves the path of mesh files

        function resolvePath(path) {
          if (!/^package:\/\//.test(path)) {
            return workingPath ? workingPath + path : path;
          } // Remove "package://" keyword and split meshPath at the first slash


          var _path$replace$split = path.replace(/^package:\/\//, '').split(/\/(.+)/),
              _path$replace$split2 = _slicedToArray(_path$replace$split, 2),
              targetPkg = _path$replace$split2[0],
              relPath = _path$replace$split2[1];

          if (typeof packages === 'string') {
            // "pkg" is one single package
            if (packages.endsWith(targetPkg)) {
              // "pkg" is the target package
              return packages + '/' + relPath;
            } else {
              // Assume "pkg" is the target package's parent directory
              return packages + '/' + targetPkg + '/' + relPath;
            }
          } else if (_typeof(packages) === 'object') {
            // "pkg" is a map of packages
            if (targetPkg in packages) {
              return packages[targetPkg] + '/' + relPath;
            } else {
              console.error("URDFLoader : ".concat(targetPkg, " not found in provided package list."));
              return null;
            }
          }
        } // Process the URDF text format


        var processUrdf = function processUrdf(data) {
          var parser = new DOMParser();
          var urdf = parser.parseFromString(data, 'text/xml');

          var children = _toConsumableArray(urdf.children);

          var robotNode = children.filter(function (c) {
            return c.nodeName === 'robot';
          }).pop();
          return processRobot.call(_this2, robotNode);
        }; // Process the <robot> node


        function processRobot(robot) {
          var _this3 = this;

          var robotNodes = _toConsumableArray(robot.children);

          var links = robotNodes.filter(function (c) {
            return c.nodeName.toLowerCase() === 'link';
          });
          var joints = robotNodes.filter(function (c) {
            return c.nodeName.toLowerCase() === 'joint';
          });
          var materials = robotNodes.filter(function (c) {
            return c.nodeName.toLowerCase() === 'material';
          });
          var obj = new URDFRobot();
          obj.robotName = robot.getAttribute('name');
          obj.urdfRobotNode = robot; // Create the <material> map

          materials.forEach(function (m) {
            var name = m.getAttribute('name');
            materialMap[name] = processMaterial.call(_this3, m);
          }); // Create the <link> map

          links.forEach(function (l) {
            var name = l.getAttribute('name');
            var isRoot = robot.querySelector("child[link=\"".concat(name, "\"]")) === null;
            linkMap[name] = processLink.call(_this3, l, isRoot ? obj : null);
          }); // Create the <joint> map

          joints.forEach(function (j) {
            var name = j.getAttribute('name');
            jointMap[name] = processJoint.call(_this3, j);
          });
          obj.joints = jointMap;
          obj.links = linkMap;
          return obj;
        } // Process joint nodes and parent them


        function processJoint(joint) {
          var children = _toConsumableArray(joint.children);

          var jointType = joint.getAttribute('type');
          var obj = new URDFJoint();
          obj.urdfNode = joint;
          obj.name = joint.getAttribute('name');
          obj.jointType = jointType;
          var parent = null;
          var child = null;
          var xyz = [0, 0, 0];
          var rpy = [0, 0, 0]; // Extract the attributes

          children.forEach(function (n) {
            var type = n.nodeName.toLowerCase();

            if (type === 'origin') {
              xyz = processTuple(n.getAttribute('xyz'));
              rpy = processTuple(n.getAttribute('rpy'));
            } else if (type === 'child') {
              child = linkMap[n.getAttribute('link')];
            } else if (type === 'parent') {
              parent = linkMap[n.getAttribute('link')];
            } else if (type === 'limit') {
              obj.limit.lower = parseFloat(n.getAttribute('lower') || obj.limit.lower);
              obj.limit.upper = parseFloat(n.getAttribute('upper') || obj.limit.upper);
            }
          }); // Join the links

          parent.add(obj);
          obj.add(child);
          applyRotation(obj, rpy);
          obj.position.set(xyz[0], xyz[1], xyz[2]); // Set up the rotate function

          var axisNode = children.filter(function (n) {
            return n.nodeName.toLowerCase() === 'axis';
          })[0];

          if (axisNode) {
            var axisXYZ = axisNode.getAttribute('xyz').split(/\s+/g).map(function (num) {
              return parseFloat(num);
            });
            obj.axis = new THREE.Vector3(axisXYZ[0], axisXYZ[1], axisXYZ[2]);
            obj.axis.normalize();
          }

          return obj;
        } // Process the <link> nodes


        function processLink(link) {
          var _this4 = this;

          var target = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;

          if (target === null) {
            target = new URDFLink();
          }

          var children = _toConsumableArray(link.children);

          target.name = link.getAttribute('name');
          target.urdfNode = link;

          if (parseVisual) {
            var visualNodes = children.filter(function (n) {
              return n.nodeName.toLowerCase() === 'visual';
            });
            visualNodes.forEach(function (vn) {
              return processLinkElement.call(_this4, vn, target, materialMap);
            });
          }

          if (parseCollision) {
            var collisionNodes = children.filter(function (n) {
              return n.nodeName.toLowerCase() === 'collision';
            });
            collisionNodes.forEach(function (vn) {
              return processLinkElement.call(_this4, vn, target);
            });
          }

          return target;
        }

        function processMaterial(node) {
          var _this5 = this;

          var matNodes = _toConsumableArray(node.children);

          var material = new THREE.MeshPhongMaterial();
          material.name = node.getAttribute('name') || '';
          matNodes.forEach(function (n) {
            var type = n.nodeName.toLowerCase();

            if (type === 'color') {
              var rgba = n.getAttribute('rgba').split(/\s/g).map(function (v) {
                return parseFloat(v);
              });
              material.color.setRGB(rgba[0], rgba[1], rgba[2]);
              material.opacity = rgba[3];
              material.transparent = rgba[3] < 1;
            } else if (type === 'texture') {
              var loader = new THREE.TextureLoader(manager);
              var filename = n.getAttribute('filename');
              var filePath = resolvePath(filename);

              var onError = function onError() {
                _this5.retryMap[filePath] = function () {
                  return loader.load(filePath, function () {
                    return null;
                  }, function () {
                    return null;
                  }, onError);
                };
              };

              material.map = loader.load(filePath, function () {
                return null;
              }, function () {
                return null;
              }, onError);
            }
          });
          return material;
        } // Process the visual and collision nodes into meshes


        function processLinkElement(vn, linkObj) {
          var _this6 = this;

          var materialMap = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};
          var isCollisionNode = vn.nodeName.toLowerCase() === 'collision';
          var xyz = [0, 0, 0];
          var rpy = [0, 0, 0];
          var scale = [1, 1, 1];

          var children = _toConsumableArray(vn.children);

          var material = null;
          var primitiveModel = null; // get the material first

          var materialNode = children.filter(function (n) {
            return n.nodeName.toLowerCase() === 'material';
          })[0];

          if (materialNode) {
            var name = materialNode.getAttribute('name');

            if (name && name in materialMap) {
              material = materialMap[name];
            } else {
              material = processMaterial.call(this, materialNode);
            }
          } else {
            material = new THREE.MeshPhongMaterial();
          }

          children.forEach(function (n) {
            var type = n.nodeName.toLowerCase();

            if (type === 'geometry') {
              var geoType = n.children[0].nodeName.toLowerCase();

              if (geoType === 'mesh') {
                var filename = n.children[0].getAttribute('filename');
                var filePath = resolvePath(filename); // file path is null if a package directory is not provided.

                if (filePath !== null) {
                  var scaleAttr = n.children[0].getAttribute('scale');
                  if (scaleAttr) scale = processTuple(scaleAttr);

                  var cb = function cb(obj, err) {
                    if (err) {
                      console.error('URDFLoader: Error loading mesh.', err);

                      _this6.retryMap[filePath] = function () {
                        return loadMeshCb(filePath, manager, cb);
                      };
                    } else if (obj) {
                      if (obj instanceof THREE.Mesh) {
                        obj.material = material;

                        if (_this6.allowMeshBVH) {
                          obj.raycast = threeMeshBvh.acceleratedRaycast;
                          obj.geometry.boundsTree = new threeMeshBvh.MeshBVH(obj.geometry);
                        }
                      }

                      linkObj.add(obj);
                      obj.position.set(xyz[0], xyz[1], xyz[2]);
                      obj.rotation.set(0, 0, 0); // multiply the existing scale by the scale components because
                      // the loaded model could have important scale values already applied
                      // to the root. Collada files, for example, can load in with a scale
                      // to convert the model units to meters.

                      obj.scale.x *= scale[0];
                      obj.scale.y *= scale[1];
                      obj.scale.z *= scale[2];
                      applyRotation(obj, rpy);

                      if (isCollisionNode) {
                        makeURDFCollider(obj);
                      }
                    }
                  };

                  loadMeshCb(filePath, manager, cb);
                }
              } else if (geoType === 'box') {
                primitiveModel = new THREE.Mesh();
                primitiveModel.geometry = new THREE.BoxBufferGeometry(1, 1, 1);
                primitiveModel.material = material;

                if (_this6.allowMeshBVH) {
                  primitiveModel.raycast = threeMeshBvh.acceleratedRaycast;
                  primitiveModel.geometry.boundsTree = new threeMeshBvh.MeshBVH(primitiveModel.geometry);
                }

                var size = processTuple(n.children[0].getAttribute('size'));
                linkObj.add(primitiveModel);
                primitiveModel.scale.set(size[0], size[1], size[2]);

                if (isCollisionNode) {
                  makeURDFCollider(primitiveModel);
                }
              } else if (geoType === 'sphere') {
                primitiveModel = new THREE.Mesh();
                primitiveModel.geometry = new THREE.SphereBufferGeometry(1, 30, 30);
                primitiveModel.material = material;

                if (_this6.allowMeshBVH) {
                  primitiveModel.raycast = threeMeshBvh.acceleratedRaycast;
                  primitiveModel.geometry.boundsTree = new threeMeshBvh.MeshBVH(primitiveModel.geometry);
                }

                var radius = parseFloat(n.children[0].getAttribute('radius')) || 0;
                primitiveModel.scale.set(radius, radius, radius);
                linkObj.add(primitiveModel);

                if (isCollisionNode) {
                  makeURDFCollider(primitiveModel);
                }
              } else if (geoType === 'cylinder') {
                primitiveModel = new THREE.Mesh();
                primitiveModel.geometry = new THREE.CylinderBufferGeometry(1, 1, 1, 30);
                primitiveModel.material = material;

                if (_this6.allowMeshBVH) {
                  primitiveModel.raycast = threeMeshBvh.acceleratedRaycast;
                  primitiveModel.geometry.boundsTree = new threeMeshBvh.MeshBVH(primitiveModel.geometry);
                }

                var _radius = parseFloat(n.children[0].getAttribute('radius')) || 0;

                var length = parseFloat(n.children[0].getAttribute('length')) || 0;
                primitiveModel.scale.set(_radius, length, _radius);
                primitiveModel.rotation.set(Math.PI / 2, 0, 0);
                linkObj.add(primitiveModel);

                if (isCollisionNode) {
                  makeURDFCollider(primitiveModel);
                }
              }
            } else if (type === 'origin') {
              xyz = processTuple(n.getAttribute('xyz'));
              rpy = processTuple(n.getAttribute('rpy'));
            }
          }); // apply the position and rotation to the primitive geometry after
          // the fact because it's guaranteed to have been scraped from the child
          // nodes by this point

          if (primitiveModel) {
            applyRotation(primitiveModel, rpy, true);
            primitiveModel.position.set(xyz[0], xyz[1], xyz[2]);
          }
        }

        return processUrdf(content);
      } // Default mesh loading function

    }, {
      key: "defaultMeshLoader",
      value: function defaultMeshLoader(path, manager, done) {
        if (/\.stl(?:\?|$)/i.test(path)) {
          var loader = new STLLoader_js.STLLoader(manager);
          loader.load(path, function (geom) {
            var mesh = new THREE.Mesh(geom, new THREE.MeshPhongMaterial());
            done(mesh);
          });
        } else if (/\.dae(?:\?|$)/i.test(path)) {
          var _loader = new ColladaLoader_js.ColladaLoader(manager);

          _loader.load(path, function (dae) {
            return done(dae.scene);
          });
        } else {
          console.warn("URDFLoader: Could not load model at ".concat(path, ".\nNo loader available"));
        }
      }
    }]);

    return URDFLoader;
  }();

  return URDFLoader;

}));

});

var URDFLoader$1 = unwrapExports(URDFLoader);

class URDFCore extends URDFLoader$1 {
    constructor(ros, paramName, options = DEFAULT_OPTIONS_ROBOTMODEL) {
        super(three.DefaultLoadingManager);
        this.packages = {};
        // getPackages(onComplete: (packages: string[]) => void) {
        //   this.param.get(robotString => {
        //     const parser = new DOMParser();
        //     const urdf = parser.parseFromString(robotString, 'text/xml');
        //     const packages = [...Array.from(urdf.querySelectorAll('mesh'))].map(
        //       mesh => {
        //         const [targetPkg] = mesh
        //           ?.getAttribute('filename')
        //           ?.replace(/^package:\/\//, '')
        //           .split(/\/(.+)/);
        //         return targetPkg;
        //       },
        //     );
        //     onComplete([...new Set(packages)]);
        //   });
        // }
        this.hide = () => {
            var _a, _b;
            assertIsDefined(this.object);
            Object.values((_b = (_a = this.urdfObject) === null || _a === void 0 ? void 0 : _a.links) !== null && _b !== void 0 ? _b : []).forEach((link) => {
                link.hide();
            });
        };
        this.show = () => {
            var _a, _b;
            assertIsDefined(this.object);
            Object.values((_b = (_a = this.urdfObject) === null || _a === void 0 ? void 0 : _a.links) !== null && _b !== void 0 ? _b : []).forEach((link) => {
                link.show();
            });
        };
        this.destroy = () => {
            var _a, _b, _c, _d;
            Object.values((_b = (_a = this.urdfObject) === null || _a === void 0 ? void 0 : _a.links) !== null && _b !== void 0 ? _b : []).forEach((link) => {
                link.delete();
            });
            (_d = (_c = this.object) === null || _c === void 0 ? void 0 : _c.parent) === null || _d === void 0 ? void 0 : _d.remove(this.object);
            this.object = undefined;
        };
        this.reset = () => { };
        this.options = options;
        this.packages = options.packages;
        this.param = new ROSLIB__default["default"].Param({
            ros,
            name: paramName,
        });
        this.updateOptions = this.updateOptions.bind(this);
        // this.getPackages = this.getPackages.bind(this);
        this.defaultLoadMeshCallback = this.defaultLoadMeshCallback.bind(this);
        this.onComplete = this.onComplete.bind(this);
        this.loadURDF = this.loadURDF.bind(this);
        this.loadFromParam = this.loadFromParam.bind(this);
    }
    onComplete(object) { }
    loadFromParam(onComplete = this.onComplete, options = {}) {
        this.param.get(urdfString => {
            this.loadURDF(urdfString, onComplete, options);
        });
    }
    loadURDF(urdfString, onComplete = this.onComplete, options) {
        const urdfObject = super.parse(urdfString, {
            packages: options.packages || this.packages,
            loadMeshCb: options.loadMeshCb || this.defaultLoadMeshCallback,
            fetchOptions: { mode: 'cors', credentials: 'same-origin' },
            ...options,
        });
        assertIsDefined(this.object);
        this.urdfObject = urdfObject;
        this.object.add(urdfObject);
        this.object.name = urdfObject.robotName;
        onComplete(this.object);
    }
    defaultLoadMeshCallback(path, ext, done) {
        super.defaultMeshLoader(path, ext, (mesh) => {
            done(mesh);
        });
    }
    updateOptions(options) {
        this.options = {
            ...this.options,
            ...options,
        };
    }
}

class RobotModel extends URDFCore {
    constructor(ros, paramName, options = DEFAULT_OPTIONS_ROBOTMODEL) {
        super(ros, paramName, options);
        this.object = new Group();
        this.updateOptions({
            ...DEFAULT_OPTIONS_ROBOTMODEL,
            ...options,
        });
    }
}

class Sphere extends Mesh {
    constructor() {
        super();
        // Radius handled through scale
        this.geometry = new three.SphereGeometry(1, DEFAULT_RADIAL_SEGMENTS, DEFAULT_RADIAL_SEGMENTS);
        this.material = new three.MeshStandardMaterial();
        this.material.transparent = true;
    }
}

class Point extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_POINT) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_POINT,
                ...options,
            },
        });
        const { alpha, color, radius } = this.options;
        this.object = new Group();
        this.sphere = new Sphere();
        this.object.add(this.sphere);
        this.sphere.setColor(color);
        this.sphere.setAlpha(alpha);
        this.sphere.setScale({ x: radius, y: radius, z: radius });
    }
    updateOptions(options) {
        super.updateOptions(options);
    }
    update(message) {
        var _a;
        super.update(message);
        const { point: { x, y, z }, } = message;
        (_a = this.object) === null || _a === void 0 ? void 0 : _a.position.set(x, y, z);
    }
}

class Cube extends Mesh {
    constructor() {
        super();
        this.geometry = new three.BoxGeometry();
        this.material = new three.MeshStandardMaterial();
    }
}

class LineSegments extends three.LineSegments {
    constructor(color = DEFAULT_COLOR_LINE, linewidth = 5) {
        super();
        this.geometry = new three.Geometry();
        this.material = new three.LineBasicMaterial({ linewidth, color: three.VertexColors });
        // this.material.color = VertexColors;
    }
    isValid(points) {
        return points.length <= this.geometry.vertices.length;
    }
    setColor(color) {
        setColor(this, color);
    }
    updatePoints(points, colors) {
        this.geometry.vertices = points.map(({ x, y, z }) => new three.Vector3(x, y, z));
        this.geometry.verticesNeedUpdate = true;
        if (colors.length > 0) {
            this.geometry.colors = colors.map(({ r, g, b }) => new three.Color(r, g, b));
            this.geometry.colorsNeedUpdate = true;
        }
    }
    setTransform(transform) {
        setTransform(this, transform);
    }
}

class ObjectCacher {
    constructor(objectPool, Primitive) {
        this.objectPool = objectPool;
        this.Primitive = Primitive;
    }
    setObjectDimension(object, position, color, scale) {
        const { x, y, z } = position;
        object.setColor(color);
        object.setScale(scale);
        object.position.set(x, y, z);
    }
    reusePool(points, colors, options) {
        const { scale } = options;
        const currentCount = points.length;
        for (let i = 0; i < currentCount; i++) {
            const currentChild = this.objectPool.children[i];
            this.setObjectDimension(currentChild, points[i], colors[i], scale);
        }
        for (let i = currentCount; i < this.objectPool.children.length; i++) {
            this.objectPool.children[i].visible = false;
        }
    }
    increasePool(points, colors, options) {
        const currentCount = this.objectPool.children.length;
        const { scale } = options;
        for (let i = 0; i < currentCount; i++) {
            const currentChild = this.objectPool.children[i];
            this.setObjectDimension(currentChild, points[i], colors[i], scale);
        }
        for (let i = currentCount; i < points.length; i++) {
            const sphere = new this.Primitive();
            const { x, y, z } = points[i];
            sphere.setColor(colors[i]);
            sphere.setScale(scale);
            sphere.position.set(x, y, z);
            this.objectPool.add(sphere);
        }
    }
}

class SphereList extends Mesh {
    constructor() {
        super();
        this.geometry = new three.Geometry();
        this.material = new three.MeshBasicMaterial();
        this.objectCacher = new ObjectCacher(this, Sphere);
    }
    updatePoints(points, colors, options) {
        const optionsOverride = {
            ...options,
            subtype: MARKER_OBJECT_TYPES.SPHERE,
        };
        if (points.length < this.children.length) {
            this.objectCacher.reusePool(points, colors, optionsOverride);
        }
        else {
            this.objectCacher.increasePool(points, colors, optionsOverride);
        }
    }
}

class Points$1 extends three.Points {
    constructor() {
        super();
        this.geometry = new three.Geometry();
        this.material = new three.PointsMaterial({
            color: three.VertexColors,
        });
    }
    setTransform(transform) {
        setTransform(this, transform);
    }
    updatePoints(points, colors, options) {
        const { scale: { x }, } = options;
        this.material.size = x;
        this.geometry.vertices = points.map(vertex => new three.Vector3(vertex.x, vertex.y, vertex.z));
        this.geometry.verticesNeedUpdate = true;
        if (colors.length > 0) {
            this.geometry.colors = colors.map(color => new three.Color(color.r, color.g, color.b));
            this.geometry.colorsNeedUpdate = true;
        }
    }
    setScale(scale) {
        if (typeof scale === 'number') {
            this.material.size = scale;
        }
        else {
            this.material.size = scale.x;
        }
    }
}

class TriangleList extends Mesh {
    constructor() {
        super();
        this.geometry = new three.Geometry();
        this.material = new three.MeshBasicMaterial({
            color: three.FaceColors,
        });
        this.material.side = three.DoubleSide;
    }
    updatePoints(points, colors = [], options) {
        const vertices = [];
        const faces = [];
        const { scale: { x, y, z }, } = options;
        this.scale.set(x, y, z);
        for (let index = 0, l = points.length / 3; index < l; index++) {
            const verticesArray = [
                points[3 * index],
                points[3 * index + 1],
                points[3 * index + 2],
            ];
            verticesArray.map(side => {
                vertices.push(new three.Vector3(side.x, side.y, side.z));
            });
            const color = colors.length === 0 ? { r: 1, g: 0, b: 0 } : colors[3 * index];
            faces.push(new three.Face3(3 * index, 3 * index + 2, 3 * index + 1, new three.Vector3(), new three.Color(color.r, color.g, color.b)));
        }
        this.geometry.vertices = vertices;
        this.geometry.faces = faces;
        this.geometry.computeFaceNormals();
        this.geometry.computeVertexNormals();
        this.geometry.elementsNeedUpdate = true;
        this.geometry.verticesNeedUpdate = true;
    }
}

class CubeList extends Mesh {
    constructor() {
        super();
        this.geometry = new three.Geometry();
        this.material = new three.MeshBasicMaterial();
        this.objectCacher = new ObjectCacher(this, Cube);
    }
    updatePoints(points, colors, options) {
        const optionsOverride = {
            ...options,
            subType: MARKER_OBJECT_TYPES.CUBE,
        };
        if (points.length < this.children.length) {
            this.objectCacher.reusePool(points, colors, optionsOverride);
        }
        else {
            this.objectCacher.increasePool(points, colors, optionsOverride);
        }
    }
}

// @ts-ignore
class ViewFacingText extends Mesh {
    constructor(text) {
        super();
        this.color = new three.Color(1, 1, 1);
        this.text = new SpriteText__default["default"](text, 1, `#${this.color.getHexString()}`);
        this.add(this.text);
    }
    setColor(color) {
        const { b, g, r } = color;
        if (!(this.color.r === r && this.color.g === g && this.color.b === b)) {
            this.color = new three.Color(r, g, b);
            this.text.color = `#${this.color.getHexString()}`;
        }
    }
}

const getNewPrimitive = (marker) => {
    switch (marker.type) {
        case MARKER_OBJECT_TYPES.CUBE:
            return new Cube();
        case MARKER_OBJECT_TYPES.SPHERE:
            return new Sphere();
        case MARKER_OBJECT_TYPES.CYLINDER: {
            const group = new Group();
            group.add(new Cylinder());
            return group;
        }
        case MARKER_OBJECT_TYPES.LINE_LIST:
            return new LineSegments();
        case MARKER_OBJECT_TYPES.LINE_STRIP:
            return new Line();
        case MARKER_OBJECT_TYPES.SPHERE_LIST:
            return new SphereList();
        case MARKER_OBJECT_TYPES.POINTS:
            return new Points$1();
        case MARKER_OBJECT_TYPES.TRIANGLE_LIST:
            return new TriangleList();
        case MARKER_OBJECT_TYPES.CUBE_LIST:
            return new CubeList();
        case MARKER_OBJECT_TYPES.TEXT_VIEW_FACING:
            return new ViewFacingText(marker.text);
        case MARKER_OBJECT_TYPES.ARROW:
        default: {
            const arrow = new Arrow();
            arrow.setHeadDimensions({
                radius: DEFAULT_CONE_RADIUS * 0.1,
                length: DEFAULT_CONE_HEIGHT * 0.3,
            });
            arrow.setShaftDimensions({
                radius: DEFAULT_CYLINDER_RADIUS * 0.05,
                length: DEFAULT_CYLINDER_HEIGHT * 0.7,
            });
            return arrow;
        }
    }
};

class MarkerLifetime {
    constructor(onTimeout) {
        this.timeouts = {};
        this.timeouts = {};
        this.onTimeout = onTimeout;
    }
    track(id, timeoutSecs) {
        clearInterval(this.timeouts[id]);
        if (timeoutSecs === 0) {
            return;
        }
        this.timeouts[id] = window.setTimeout(() => {
            this.onTimeout(id);
        }, timeoutSecs * 1000);
    }
    untrack(id) {
        clearInterval(this.timeouts[id]);
        delete this.timeouts[id];
    }
    destroy() {
        const ids = Object.keys(this.timeouts);
        for (const id of ids) {
            clearInterval(this.timeouts[id]);
            delete this.timeouts[id];
        }
    }
}

class MarkerManager {
    constructor(rootObject, onChangeCb) {
        this.objectMap = {};
        this.namespaces = {};
        this.object = rootObject;
        this.onChangeCb = onChangeCb;
        this.markerLifetime = new MarkerLifetime(this.onMarkerLifetimeOver.bind(this));
    }
    onMarkerLifetimeOver(id) {
        const marker = this.objectMap[id];
        if (!marker) {
            return;
        }
        this.removeObject(id);
    }
    getMarkerOrCreate(marker) {
        const id = MarkerManager.getId(marker);
        if (!this.objectMap[id]) {
            const object = getNewPrimitive(marker);
            this.objectMap[id] = object;
            this.object.add(object);
        }
        else {
            // We need to create a new geometry when there are more or less points
            if (this.objectMap[id] instanceof LineSegments) {
                if (!this.objectMap[id].isValid(marker.points)) {
                    this.removeObject(id);
                    this.objectMap[id] = getNewPrimitive(marker);
                    this.object.add(this.objectMap[id]);
                }
            }
        }
        this.objectMap[id].visible = this.namespaces[marker.ns];
        return this.objectMap[id];
    }
    extractNameSpace(str) {
        const tokens = str.split('-');
        return tokens[0];
    }
    updateOptions(options) {
        const { namespaces } = options;
        this.namespaces = namespaces;
        for (const key in this.objectMap) {
            // eslint-disable-next-line no-prototype-builtins
            if (this.objectMap.hasOwnProperty(key)) {
                const namespace = this.extractNameSpace(key);
                this.objectMap[key].visible = this.namespaces[namespace];
            }
        }
    }
    onChange() {
        this.onChangeCb();
    }
    updateMarker(marker) {
        const { color, colors, lifetime, points, pose: { orientation, position }, scale, } = marker;
        // markerObject should be of the type MarkerObjectType
        // certain functions used below are not available on
        // all types, hence any
        const markerObject = this.getMarkerOrCreate(marker);
        const markerId = MarkerManager.getId(marker);
        this.markerLifetime.track(markerId, lifetime.secs);
        if (markerObject.updatePoints) {
            markerObject.updatePoints(points, colors, marker);
        }
        markerObject.setTransform({ translation: position, rotation: orientation });
        // To avoid settings these properties for list types: LINE, TRIANGLE, CUBELIST etc
        if (markerObject.setScale && !markerObject.updatePoints) {
            markerObject.setScale({ x: scale.x, y: scale.y, z: scale.z });
        }
        if (markerObject.setColor && colors.length <= 0) {
            markerObject.setColor(color);
        }
        const { ns } = marker;
        if (!(ns in this.namespaces)) {
            this.namespaces[ns] = true;
            this.onChange();
        }
    }
    removeObject(id) {
        var _a;
        const obj = this.objectMap[id];
        (_a = obj.parent) === null || _a === void 0 ? void 0 : _a.remove(obj);
        delete this.objectMap[id];
    }
    reset() {
        this.namespaces = {};
        this.markerLifetime.destroy();
        this.onChange();
        Object.keys(this.objectMap).forEach(id => {
            this.removeObject(id);
        });
    }
    static getId(marker) {
        const { ns, id } = marker;
        return `${ns}-${id}`;
    }
}

class MarkerArray extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_MARKERARRAY) {
        var _a;
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_MARKERARRAY,
                ...options,
            },
        });
        this.reset = () => {
            this.markerManager.reset();
        };
        this.object = new Group();
        this.markerManager = new MarkerManager(this.object, (_a = this.onNameSpaceChange) !== null && _a !== void 0 ? _a : (() => null));
    }
    updateOptions(options) {
        super.updateOptions(options);
        this.markerManager.updateOptions(this.options);
    }
    update(message) {
        super.update(message);
        message.markers.forEach(marker => {
            this.markerManager.updateMarker(marker);
        });
    }
}

class Points {
    constructor(options = {}) {
        this.maxPts = options.max_pts || 10000;
        this.rootObject = options.rootObject || new three.Object3D();
    }
    setup(type, size, alpha) {
        this.rootObject.children.forEach(child => {
            this.rootObject.remove(child);
        });
        this.positions = new three.BufferAttribute(new Float32Array(this.maxPts * 3), 3, false);
        this.colors = new three.BufferAttribute(new Float32Array(this.maxPts * 3), 3, false);
        let options = {};
        if (type === LASERSCAN_STYLES.POINTS) {
            const sprite = new three.TextureLoader().load('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/sprites/circle.png');
            options = {
                map: sprite,
                alphaTest: 0.5,
            };
        }
        this.geometry = new three.BufferGeometry();
        this.geometry.setAttribute('position', this.positions.setUsage(three.DynamicDrawUsage));
        this.geometry.setAttribute('color', this.colors.setUsage(three.DynamicDrawUsage));
        this.material = new three.PointsMaterial({
            color: 0x888888,
            size,
            ...options,
        });
        //this.material.vertexColors = VertexColors;
        this.material.transparent = true;
        this.material.opacity = alpha;
        this.object = new three.Points(this.geometry, this.material);
        this.rootObject.add(this.object);
    }
    update(l) {
        assertIsDefined(this.geometry);
        assertIsDefined(this.positions);
        assertIsDefined(this.colors);
        this.geometry.setDrawRange(0, l);
        this.positions.needsUpdate = true;
        this.colors.needsUpdate = true;
        this.positions.updateRange.count = l * this.positions.itemSize;
        this.colors.updateRange.count = l * this.colors.itemSize;
    }
    setAlpha(alpha) {
        assertIsDefined(this.material);
        this.material.opacity = alpha;
    }
    setSize(size) {
        assertIsDefined(this.material);
        this.material.size = size;
    }
}

class LaserScan extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_LASERSCAN) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_LASERSCAN,
                ...options,
            },
        });
        this.cachedMessage = null;
        this.points = new Points();
        this.sphereList = new SphereList();
        this.cubeList = new CubeList();
        this.object = new Group();
        this.object.add(this.points.rootObject);
        this.object.add(this.sphereList);
        this.object.add(this.cubeList);
    }
    getNormalizedIntensity(intensity) {
        const { maxIntensity, minIntensity } = this.options;
        return (intensity - minIntensity) / (maxIntensity - minIntensity);
    }
    applyIntensityTransform(intensity, position) {
        const { channelName, maxColor, minColor } = this.options;
        const { x, y, z } = position;
        let normI;
        switch (channelName) {
            case INTENSITY_CHANNEL_OPTIONS.INTENSITY:
                normI = this.getNormalizedIntensity(intensity);
                break;
            case INTENSITY_CHANNEL_OPTIONS.X:
                normI = this.getNormalizedIntensity(x);
                break;
            case INTENSITY_CHANNEL_OPTIONS.Y:
                normI = this.getNormalizedIntensity(y);
                break;
            case INTENSITY_CHANNEL_OPTIONS.Z:
                normI = this.getNormalizedIntensity(z);
                break;
        }
        const minColorHex = new three.Color(minColor);
        const maxColorHex = new three.Color(maxColor);
        assertIsDefined(normI);
        const finalColor = normI * maxColorHex.getHex() + (1 - normI) * minColorHex.getHex();
        return new three.Color(finalColor);
    }
    getNormalizedAxisValue(axisValue) {
        const { maxAxisValue, minAxisValue } = this.options;
        return (axisValue - minAxisValue) / (maxAxisValue - minAxisValue);
    }
    applyAxisColorTransform(position) {
        const { axis, maxAxisValue, minAxisValue } = this.options;
        const { x, y, z } = position;
        let normI;
        switch (axis) {
            case AXES.X:
                normI = this.getNormalizedAxisValue(x);
                break;
            case AXES.Y:
                normI = this.getNormalizedAxisValue(y);
                break;
            case AXES.Z:
                normI = this.getNormalizedAxisValue(z);
                break;
        }
        assertIsDefined(normI);
        const finalColor = normI * maxAxisValue + (1 - normI) * minAxisValue;
        return new three.Color(finalColor);
    }
    colorTransformer(intensity, position) {
        const { colorTransformer, flatColor } = this.options;
        switch (colorTransformer) {
            case COLOR_TRANSFORMERS.INTENSITY:
                return this.applyIntensityTransform(intensity, position);
            case COLOR_TRANSFORMERS.AXIS_COLOR:
                return this.applyAxisColorTransform(position);
            case COLOR_TRANSFORMERS.FLAT_COLOR:
                return new three.Color(flatColor);
            default:
                return null;
        }
    }
    setupPoints(index, position, color) {
        assertIsDefined(color);
        assertIsDefined(this.points.colors);
        assertIsDefined(this.points.positions);
        assertBehavesLikeArray(this.points.colors.array);
        assertBehavesLikeArray(this.points.positions.array);
        this.points.colors.array[index] = color.r;
        this.points.positions.array[index++] = position.x;
        this.points.colors.array[index] = color.g;
        this.points.positions.array[index++] = position.y;
        this.points.colors.array[index] = color.b;
        this.points.positions.array[index++] = position.z;
    }
    hideAllObjects() {
        this.points.rootObject.visible = false;
        this.sphereList.visible = false;
        this.cubeList.visible = false;
    }
    setStyleDimensions(message) {
        const { alpha, style } = this.options;
        const { size } = this.options;
        const { intensities, ranges } = message;
        const n = ranges.length;
        const positions = [];
        const colors = [];
        if (size < 0.001 || !size) {
            return;
        }
        this.hideAllObjects();
        this.points.setup(style, size, alpha);
        let j = 0;
        for (let i = 0; i < n; i++) {
            const range = message.ranges[i];
            if (range >= message.range_min && range <= message.range_max) {
                const angle = message.angle_min + i * message.angle_increment;
                const position = {
                    x: range * Math.cos(angle),
                    y: range * Math.sin(angle),
                    z: 0,
                };
                const color = this.colorTransformer(intensities[i], position);
                switch (style) {
                    case LASERSCAN_STYLES.POINTS:
                    case LASERSCAN_STYLES.SQUARES:
                    case LASERSCAN_STYLES.FLAT_SQUARES: {
                        this.setupPoints(j, position, color);
                        j += 3;
                        break;
                    }
                    default:
                        positions.push(position);
                        colors.push(color);
                        break;
                }
            }
        }
        const options = { scale: { x: size, y: size, z: size } };
        switch (style) {
            case LASERSCAN_STYLES.SPHERES: {
                this.sphereList.visible = true;
                this.sphereList.updatePoints(positions, colors, options);
                break;
            }
            case LASERSCAN_STYLES.BOXES: {
                this.cubeList.visible = true;
                this.cubeList.updatePoints(positions, colors, options);
                break;
            }
            default:
                this.points.rootObject.visible = true;
                this.points.update(j / 3);
                break;
        }
    }
    updateOptions(options) {
        super.updateOptions(options);
        if (this.cachedMessage) {
            this.setStyleDimensions(this.cachedMessage);
        }
    }
    update(message) {
        super.update(message);
        this.setStyleDimensions(message);
        this.cachedMessage = message;
    }
}

const populateImageDataFromNavMsg = (imageData, width, height, dataSource) => {
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const mapI = col + (height - row - 1) * width;
            const data = dataSource[mapI];
            const val = new DataView(Uint8Array.from([data]).buffer).getUint8(0);
            let i = (col + row * width) * 4;
            if (val >= 0 && val <= 100) {
                const v = 255 - (255 * val) / 100;
                imageData.data[i] = v; // red
                imageData.data[++i] = v; // green
                imageData.data[++i] = v; // blue
                imageData.data[++i] = 255; // alpha
            }
            else if (val >= 101 && val <= 127) {
                // illegal positive values in green
                imageData.data[i] = 0; // red
                imageData.data[++i] = 255; // green
                imageData.data[++i] = 0; // blue
                imageData.data[++i] = 255; // alpha
            }
            else if (val >= 128 && val <= 254) {
                // illegal negative (char) values in shades of red/yellow
                imageData.data[i] = 255; // red
                imageData.data[++i] = (255 * (val - 128)) / (254 - 128); // green
                imageData.data[++i] = 0; // blue
                imageData.data[++i] = 255; // alpha
            }
            else {
                // legal -1 value is tasteful blueish greenish grayish color
                imageData.data[i] = 0x70; // red
                imageData.data[++i] = 0x89; // green
                imageData.data[++i] = 0x86; // blue
                imageData.data[++i] = 255; // alpha
            }
        }
    }
};
const populateRawImageDataFromNavMsg = (imageData, width, height, dataSource) => {
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const mapI = col + (height - row - 1) * width;
            const data = dataSource[mapI];
            let i = (col + row * width) * 4;
            const Uint8DV = new DataView(Uint8Array.from([data]).buffer);
            const val = Uint8DV.getUint8(0);
            imageData.data[i] = val;
            imageData.data[++i] = val;
            imageData.data[++i] = val;
            imageData.data[++i] = 255;
        }
    }
};
const populateConstImageDataFromNavMsg = (imageData, width, height, dataSource) => {
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const mapI = col + (height - row - 1) * width;
            const data = dataSource[mapI];
            const val = new DataView(Uint8Array.from([data]).buffer).getUint8(0);
            let i = (col + row * width) * 4;
            if (val === 0) {
                imageData.data[i] = 0;
                imageData.data[++i] = 0;
                imageData.data[++i] = 0;
                imageData.data[++i] = 0; // alpha
            }
            else if (val >= 1 && val <= 98) {
                // Blue to red spectrum for most normal cost values
                const v = (255 * val) / 100;
                imageData.data[i] = v; // red
                imageData.data[++i] = 0; // green
                imageData.data[++i] = 255 - v; // blue
                imageData.data[++i] = 255; // alpha
            }
            else if (val === 99) {
                // inscribed obstacle values (99) in cyan
                imageData.data[i] = 0; // red
                imageData.data[++i] = 255; // green
                imageData.data[++i] = 255; // blue
                imageData.data[++i] = 255; // alpha
            }
            else if (val === 100) {
                // lethal obstacle values (100) in purple
                imageData.data[i] = 255; // red
                imageData.data[++i] = 0; // green
                imageData.data[++i] = 255; // blue
                imageData.data[++i] = 255; // alpha
            }
            else if (val > 100 && val <= 127) {
                // illegal positive values in green
                imageData.data[i] = 0; // red
                imageData.data[++i] = 255; // green
                imageData.data[++i] = 0; // blue
                imageData.data[++i] = 255; // alpha
            }
            else if (val >= 128 && val <= 254) {
                // illegal negative (char) values in shades of red/yellow
                imageData.data[i] = 255; // red
                imageData.data[++i] = (255 * (val - 128)) / (254 - 128); // green
                imageData.data[++i] = 0; // blue
                imageData.data[++i] = 255; // alpha
            }
            else {
                // legal -1 value is tasteful blueish greenish grayish color
                imageData.data[i] = 0x70; // red
                imageData.data[++i] = 0x89; // green
                imageData.data[++i] = 0x86; // blue
                imageData.data[++i] = 255; // alpha
            }
        }
    }
};
const imageDataToCanvas = (imageData) => {
    const canvas = document.createElement('canvas');
    canvas.width = imageData.width;
    canvas.height = imageData.height;
    const context = canvas.getContext('2d');
    context === null || context === void 0 ? void 0 : context.putImageData(imageData, 0, 0);
    return canvas;
};
const populateImageDataFromImageMsg = (message, offset, rgbaOrder, imageData) => {
    const { data: rawData, height, step } = message;
    const typedArray = Uint8Array.from(rawData);
    // endianness is required for > 8bit encodings
    const encodedDataView = new DataView(typedArray.buffer);
    let j = 0;
    for (let i = 0; i < step * height; i += offset) {
        for (let k = 0; k < rgbaOrder.length; k++) {
            imageData.data[j++] = encodedDataView.getUint8(i + rgbaOrder[k]);
        }
        if (rgbaOrder.length === 3) {
            imageData.data[j++] = 255;
        }
    }
};

class Plane extends Mesh {
    constructor() {
        super();
        this.geometry = new three.PlaneGeometry();
        this.material = new three.MeshBasicMaterial();
    }
}

class Map$1 extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_MAP) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_MAP,
                ...options,
            },
        });
        this.cachedMessage = null;
        this.object = new Plane();
        assertIsMesh(this.object);
        assertIsMaterial(this.object.material);
        this.object.material.transparent = true;
        this.updateOptions({
            ...DEFAULT_OPTIONS_MAP,
            ...options,
        });
    }
    updateOptions(options) {
        assertIsMesh(this.object);
        assertIsMaterial(this.object.material);
        super.updateOptions(options);
        const { alpha, drawBehind } = this.options;
        this.object.material.opacity = alpha;
        if (drawBehind) {
            this.object.material.side = three.DoubleSide;
        }
        else {
            this.object.material.side = three.FrontSide;
        }
        this.object.material.needsUpdate = true;
        if (this.cachedMessage) {
            this.setCanvasData(this.cachedMessage);
        }
    }
    updateCanvasDimensions(message) {
        var _a, _b, _c;
        const { info: { height, origin: { orientation: { w: qw, x: qx, y: qy, z: qz }, position: { x, y, z }, }, resolution, width, }, } = message;
        (_a = this.object) === null || _a === void 0 ? void 0 : _a.scale.set(width * resolution, -1 * height * resolution, 1);
        const translatedX = (width * resolution) / 2 + x;
        const translatedY = (height * resolution) / 2 + y;
        (_b = this.object) === null || _b === void 0 ? void 0 : _b.position.set(translatedX, translatedY, z || 0.01);
        (_c = this.object) === null || _c === void 0 ? void 0 : _c.quaternion.copy(new three.Quaternion(qx, qy, qz, qw).normalize());
    }
    setCanvasData(message) {
        const { colorScheme } = this.options;
        const { data, info: { height, width }, } = message;
        const imageData = new ImageData(width, height);
        let bitmapCanvas = null;
        switch (colorScheme) {
            case MAP_COLOR_SCHEMES.MAP:
                populateImageDataFromNavMsg(imageData, width, height, data);
                bitmapCanvas = imageDataToCanvas(imageData);
                break;
            case MAP_COLOR_SCHEMES.CONST_MAP:
                populateConstImageDataFromNavMsg(imageData, width, height, data);
                bitmapCanvas = imageDataToCanvas(imageData);
                break;
            case MAP_COLOR_SCHEMES.RAW:
                populateRawImageDataFromNavMsg(imageData, width, height, data);
                bitmapCanvas = imageDataToCanvas(imageData);
                break;
        }
        assertIsMesh(this.object);
        assertIsMeshBasicMaterial(this.object.material);
        if (bitmapCanvas) {
            this.object.material.map = new three.CanvasTexture(bitmapCanvas);
            this.object.material.map.minFilter = three.NearestFilter;
            this.object.material.map.magFilter = three.NearestFilter;
            this.object.material.needsUpdate = true;
        }
        this.updateCanvasDimensions(message);
    }
    update(message) {
        super.update(message);
        this.setCanvasData(message);
        this.cachedMessage = message;
    }
}

class Odometry extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_ODOMETRY) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_ODOMETRY,
                ...options,
            },
        });
        this.objectPool = [];
        this.keepSize = 100;
        this.currentObjectIndex = -1;
        this.object = new Group();
    }
    setKeepSize(size) {
        var _a;
        let newKeepList = [];
        if (size === 0) {
            this.keepSize = 0;
            return;
        }
        if (size < this.keepSize && size < this.objectPool.length) {
            const removeCount = this.objectPool.length - size;
            for (let i = 0; i < removeCount; i++) {
                (_a = this.object) === null || _a === void 0 ? void 0 : _a.remove(this.objectPool[i]);
            }
            const slicedList = this.objectPool.slice(this.objectPool.length - size, this.objectPool.length);
            newKeepList = [...slicedList];
        }
        else {
            newKeepList = [...this.objectPool];
        }
        this.objectPool = newKeepList;
        this.keepSize = size;
        this.currentObjectIndex = this.objectPool.length - 1;
    }
    removeAllObjects() {
        this.objectPool.forEach((obj, index) => {
            var _a;
            (_a = obj.parent) === null || _a === void 0 ? void 0 : _a.remove(obj);
            delete this.objectPool[index];
        });
        this.objectPool = [];
    }
    checkToleranceThresholdExceed(newPose) {
        if (this.objectPool.length === 0) {
            return true;
        }
        const oldPose = {
            position: this.objectPool[this.currentObjectIndex].position,
            quaternion: this.objectPool[this.currentObjectIndex].quaternion,
        };
        return checkToleranceThresholdExceed(oldPose, newPose, this.options);
    }
    getObject() {
        const { type } = this.options;
        switch (type) {
            case ODOMETRY_OBJECT_TYPES.arrow:
                return new Arrow();
            case ODOMETRY_OBJECT_TYPES.axes:
                return new Axes();
        }
        return new three.Object3D();
    }
    changeObjectPoolType() {
        const tempObjectPool = [];
        // remove prev type objects and push the new ones in place of them.
        this.objectPool.forEach((object, index) => {
            var _a, _b;
            const { position, quaternion } = object;
            (_a = object.parent) === null || _a === void 0 ? void 0 : _a.remove(object);
            delete this.objectPool[index];
            const newObj = this.getObject();
            newObj.position.set(position.x, position.y, position.z);
            newObj.quaternion.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
            tempObjectPool.push(newObj);
            (_b = this.object) === null || _b === void 0 ? void 0 : _b.add(newObj);
            setObjectDimension(newObj, this.options);
        });
        this.objectPool = tempObjectPool;
    }
    updateOptions(options) {
        const { type: currentType } = this.options;
        super.updateOptions(options);
        const { keep, type } = this.options;
        if (type !== currentType) {
            this.changeObjectPoolType();
        }
        this.objectPool.forEach(object => {
            setObjectDimension(object, this.options);
        });
        this.setKeepSize(keep);
    }
    update(message) {
        var _a, _b;
        super.update(message);
        if (!this.keepSize) {
            this.removeAllObjects();
            return;
        }
        const { pose: { pose: { orientation, position }, }, } = message;
        const transform = {
            translation: position,
            rotation: orientation,
        };
        const newPose = {
            position: new three.Vector3(position.x, position.y, position.z),
            quaternion: new three.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        };
        const toleranceThresholdExceed = this.checkToleranceThresholdExceed(newPose);
        if (toleranceThresholdExceed) {
            const newObject = this.getObject();
            setObjectDimension(newObject, this.options);
            this.objectPool.push(newObject);
            this.currentObjectIndex += 1;
            this.currentObjectIndex = three.MathUtils.clamp(this.currentObjectIndex, 0, this.keepSize - 1);
            (_a = this.object) === null || _a === void 0 ? void 0 : _a.add(newObject);
            setTransform(newObject, transform);
            // remove excess object from object pool wrt to keepsize
            if (this.objectPool.length > this.keepSize) {
                const objToRemove = this.objectPool[0];
                (_b = this.object) === null || _b === void 0 ? void 0 : _b.remove(objToRemove);
                delete this.objectPool[0];
                const newObjectPool = this.objectPool.slice(1);
                this.objectPool = [...newObjectPool];
            }
        }
    }
}

class PoseArray extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_POSEARRAY) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_POSEARRAY,
                ...options,
            },
        });
        this.object = new three.Group();
    }
    update(message) {
        assertIsDefined(this.object);
        super.update(message);
        this.object.children.forEach(obj => {
            var _a;
            (_a = obj.parent) === null || _a === void 0 ? void 0 : _a.remove(obj);
        });
        this.object.children = [];
        for (let i = 0; i < message.poses.length; i++) {
            const primitive = Pose.getNewPrimitive(this.options);
            if (primitive) {
                setTransform(primitive, {
                    translation: message.poses[i].position,
                    rotation: message.poses[i].orientation,
                });
                setObjectDimension(primitive, this.options);
                this.object.add(primitive);
            }
        }
    }
}

class Path extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_PATH) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_PATH,
                ...options,
            },
        });
        const { alpha, color } = this.options;
        this.object = new Group();
        this.line = new Line(null, true);
        this.line.setColor(new three.Color(color));
        this.line.setAlpha(alpha);
    }
    updateOptions(options) {
        super.updateOptions(options);
        const { alpha, color } = this.options;
        this.line.setColor(new three.Color(color));
        this.line.setAlpha(alpha);
    }
    update(message) {
        var _a;
        super.update(message);
        const { poses } = message;
        const points = (poses || []).map(poseData => poseData.pose.position);
        this.line.updatePoints(points);
        (_a = this.object) === null || _a === void 0 ? void 0 : _a.add(this.line);
    }
}

class Image extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_IMAGE) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_IMAGE,
                ...options,
            },
        });
        this.hide = () => {
            assertIsDefined(this.object);
            this.object.style.visibility = 'hidden';
        };
        this.show = () => {
            assertIsDefined(this.object);
            this.object.style.visibility = 'visible';
        };
        const { height, width } = this.options;
        this.object = document.createElement('canvas');
        this.shadowObject = document.createElement('canvas');
        this.object.width = width;
        this.object.height = height;
    }
    applyImageData(message) {
        const { encoding, height, width } = message;
        assertIsDefined(this.object);
        const ctx = this.object.getContext('2d');
        assertIsDefined(ctx);
        const shadowCtx = this.shadowObject.getContext('2d');
        assertIsDefined(shadowCtx);
        const imgData = shadowCtx.createImageData(width, height);
        const aspectRatio = width / height;
        switch (encoding) {
            // not using encoding.find(bayer) because js switch statement
            // is much faster
            case 'bayer_rggb8':
            case 'bayer_bggr8':
            case 'bayer_gbrg8':
            case 'bayer_grbg8':
            case 'bayer_rggb16':
            case 'bayer_bggr16':
            case 'bayer_gbrg16':
            case 'bayer_grbg16':
            case '8UC1':
            case '8SC1':
            case 'mono8': {
                populateImageDataFromImageMsg(message, 1, [0, 0, 0], imgData);
                break;
            }
            case '8UC3':
            case '8SC3':
            case 'bgr8': {
                populateImageDataFromImageMsg(message, 3, [2, 1, 0], imgData);
                break;
            }
            case '8UC4':
            case '8SC4':
            case 'bgra8': {
                populateImageDataFromImageMsg(message, 4, [2, 1, 0, 3], imgData);
                break;
            }
            case 'rgb8': {
                populateImageDataFromImageMsg(message, 3, [0, 1, 2], imgData);
                break;
            }
            case 'rgba8': {
                populateImageDataFromImageMsg(message, 4, [0, 1, 2, 3], imgData);
                break;
            }
        }
        ctx.clearRect(0, 0, this.object.width, this.object.height);
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, this.object.width, this.object.height);
        shadowCtx.clearRect(0, 0, this.shadowObject.width, this.shadowObject.height);
        shadowCtx.putImageData(imgData, 0, 0);
        const scaledImageHeight = this.object.width / aspectRatio;
        const scaledImageWidth = this.object.height * aspectRatio;
        if (aspectRatio >= this.object.width / this.object.height) {
            ctx.drawImage(this.shadowObject, 0, 0, width, height, 0, (this.object.height - scaledImageHeight) / 2, this.object.width, scaledImageHeight);
        }
        else {
            ctx.drawImage(this.shadowObject, 0, 0, width, height, (this.object.width - scaledImageWidth) / 2, 0, scaledImageWidth, this.object.height);
        }
    }
    updateDimensions(width, height) {
        assertIsDefined(this.object);
        this.object.width = width;
        this.object.height = height;
    }
    update(message) {
        const { height, width } = message;
        this.shadowObject.width = width;
        this.shadowObject.height = height;
        this.applyImageData(message);
    }
}

class StaticCore {
    constructor(args) {
        var _a;
        this.hide = () => {
            assertIsDefined(this.object);
            if (isObject3D(this.object)) {
                this.object.visible = false;
            }
            else if (isHTMLElement(this.object)) {
                this.object.style.visibility = 'hidden';
            }
        };
        this.show = () => {
            assertIsDefined(this.object);
            if (isObject3D(this.object)) {
                this.object.visible = true;
            }
            else if (isHTMLElement(this.object)) {
                this.object.style.visibility = 'visible';
            }
        };
        this.destroy = () => {
            var _a, _b, _c, _d;
            if (isObject3D(this.object)) {
                (_b = (_a = this.object) === null || _a === void 0 ? void 0 : _a.parent) === null || _b === void 0 ? void 0 : _b.remove(this.object);
            }
            else if (isHTMLElement(this.object)) {
                (_d = (_c = this.object) === null || _c === void 0 ? void 0 : _c.parentElement) === null || _d === void 0 ? void 0 : _d.removeChild(this.object);
            }
            this.object = undefined;
        };
        this.reset = () => { };
        this.options = (_a = args.options) !== null && _a !== void 0 ? _a : {};
        this.updateOptions = this.updateOptions.bind(this);
    }
    updateOptions(options) {
        this.options = {
            ...this.options,
            ...options,
        };
    }
}

class ImageStream extends StaticCore {
    constructor(url, options = DEFAULT_OPTIONS_IMAGE_STREAM) {
        super({
            options: {
                ...DEFAULT_OPTIONS_IMAGE_STREAM,
                ...options,
            },
        });
        this.hide = () => {
            assertIsDefined(this.object);
            this.object.style.visibility = 'hidden';
        };
        this.show = () => {
            assertIsDefined(this.object);
            this.object.style.visibility = 'visible';
        };
        const { height, width } = this.options;
        this.url = url;
        this.object = document.createElement('img');
        this.object.src = this.url;
        this.object.setAttribute('draggable', 'false');
        this.object.width = width;
        this.object.height = height;
    }
}

class Marker extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_MARKER) {
        var _a;
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_MARKER,
                ...options,
            },
        });
        this.reset = () => {
            this.markerManager.reset();
        };
        this.object = new Group();
        this.markerManager = new MarkerManager(this.object, (_a = this.onNameSpaceChange) !== null && _a !== void 0 ? _a : (() => null));
        this.updateOptions({
            ...DEFAULT_OPTIONS_MARKER,
            ...options,
        });
    }
    updateOptions(options) {
        super.updateOptions(options);
        this.markerManager.updateOptions(this.options);
    }
    update(message) {
        super.update(message);
        this.markerManager.updateMarker(message);
    }
}

class Range extends LiveCore {
    constructor(source, options = DEFAULT_OPTIONS_RANGE) {
        super({
            sources: [source],
            options: {
                ...DEFAULT_OPTIONS_RANGE,
                ...options,
            },
        });
        const { alpha, color } = this.options;
        this.object = new Group();
        this.cylinder = new Cylinder(this.options.color, 0.01, 0.01);
        this.cylinder.rotateZ(Math.PI / 2);
        this.object.add(this.cylinder);
        this.cylinder.setAlpha(alpha);
        this.cylinder.setColor(new three.Color(color));
    }
    update(message) {
        super.update(message);
        const { field_of_view: fov, max_range: max, min_range: min } = message;
        this.cylinder.geometry = new three.CylinderGeometry((min * Math.tan(fov)) / 4, (max * Math.tan(fov)) / 4, max - min, DEFAULT_RADIAL_SEGMENTS, DEFAULT_RADIAL_SEGMENTS);
        this.cylinder.geometry.verticesNeedUpdate = true;
        this.cylinder.position.setX(0.5 * (min + max));
    }
}

var stats_min = createCommonjsModule(function (module, exports) {
!function(e,t){module.exports=t();}(commonjsGlobal,function(){var c=function(){var n=0,l=document.createElement("div");function e(e){return l.appendChild(e.dom),e}function t(e){for(var t=0;t<l.children.length;t++)l.children[t].style.display=t===e?"block":"none";n=e;}l.style.cssText="position:fixed;top:0;left:0;cursor:pointer;opacity:0.9;z-index:10000",l.addEventListener("click",function(e){e.preventDefault(),t(++n%l.children.length);},!1);var i=(performance||Date).now(),a=i,o=0,f=e(new c.Panel("FPS","#0ff","#002")),r=e(new c.Panel("MS","#0f0","#020"));if(self.performance&&self.performance.memory)var d=e(new c.Panel("MB","#f08","#201"));return t(0),{REVISION:16,dom:l,addPanel:e,showPanel:t,begin:function(){i=(performance||Date).now();},end:function(){o++;var e=(performance||Date).now();if(r.update(e-i,200),a+1e3<=e&&(f.update(1e3*o/(e-a),100),a=e,o=0,d)){var t=performance.memory;d.update(t.usedJSHeapSize/1048576,t.jsHeapSizeLimit/1048576);}return e},update:function(){i=this.end();},domElement:l,setMode:t}};return c.Panel=function(n,l,i){var a=1/0,o=0,f=Math.round,r=f(window.devicePixelRatio||1),d=80*r,e=48*r,c=3*r,p=2*r,u=3*r,s=15*r,m=74*r,h=30*r,y=document.createElement("canvas");y.width=d,y.height=e,y.style.cssText="width:80px;height:48px";var v=y.getContext("2d");return v.font="bold "+9*r+"px Helvetica,Arial,sans-serif",v.textBaseline="top",v.fillStyle=i,v.fillRect(0,0,d,e),v.fillStyle=l,v.fillText(n,c,p),v.fillRect(u,s,m,h),v.fillStyle=i,v.globalAlpha=.9,v.fillRect(u,s,m,h),{dom:y,update:function(e,t){a=Math.min(a,e),o=Math.max(o,e),v.fillStyle=i,v.globalAlpha=1,v.fillRect(0,0,d,s),v.fillStyle=l,v.fillText(f(e)+" "+n+" ("+f(a)+"-"+f(o)+")",c,p),v.drawImage(y,u+r,s,m-r,h,u,s,m-r,h),v.fillRect(u+m-r,s,r,h),v.fillStyle=i,v.globalAlpha=.9,v.fillRect(u+m-r,s,r,f((1-e/t)*h));}}},c});
});

// @ts-ignore
class Scene extends three.Scene {
    constructor(options = DEFAULT_OPTIONS_SCENE) {
        super();
        this.options = DEFAULT_OPTIONS_SCENE;
        this.vizWrapper = new three.Group();
        this.add(this.vizWrapper);
        this.stats = new stats_min();
        this.stats.showPanel(0);
        this.initLights();
        this.initGrid();
        this.updateOptions(options);
    }
    initLights() {
        [
            [-1, 0],
            [1, 0],
            [0, -1],
            [0, 1],
        ].forEach(positions => {
            const directionalLight = new three.DirectionalLight(0xffffff, 0.4);
            [directionalLight.position.x, directionalLight.position.y] = positions;
            directionalLight.position.z = 1;
            this.add(directionalLight);
        });
        const ambientLight = new three.AmbientLight(0xffffff, 0.2);
        this.add(ambientLight);
    }
    initGrid() {
        this.grid = new three.GridHelper(0, 0);
        this.add(this.grid);
    }
    addObject(object) {
        this.vizWrapper.add(object);
    }
    addVisualization(args) {
        const { object } = args;
        if (!object) {
            return;
        }
        if (!this.vizWrapper.getObjectById(object.id)) {
            this.addObject(object);
        }
    }
    getObjectByName(name) {
        return this.vizWrapper.getObjectByName(name);
    }
    updateOptions(options = DEFAULT_OPTIONS_SCENE) {
        var _a, _b;
        this.options = {
            ...DEFAULT_OPTIONS_SCENE,
            ...options,
        };
        const { backgroundColor, gridCenterlineColor, gridColor, gridDivisions, gridSize, } = this.options;
        (_a = this.grid) === null || _a === void 0 ? void 0 : _a.copy(new three.GridHelper(gridSize, gridDivisions, gridCenterlineColor, gridColor));
        (_b = this.grid) === null || _b === void 0 ? void 0 : _b.rotateX(-Math.PI / 2);
        this.background = new three.Color(backgroundColor);
    }
}

/**
 * Software License Agreement (BSD License)

 Copyright (c) 2014, Worcester Polytechnic Institute, Robert Bosch
 LLC, Yujin Robot. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the following
 disclaimer in the documentation and/or other materials provided
 with the distribution.
 * Neither the name of Worcester Polytechnic Institute, Robert
 Bosch LLC, Yujin Robot nor the names of its contributors may be
 used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */
const vertexShader = `uniform sampler2D map;

uniform float width;
uniform float height;
uniform float nearClipping, farClipping;

uniform float pointSize;
uniform float zOffset;

uniform float focallength;
uniform float maxDepthPerTile;
uniform float resolutionFactor;

varying vec2 vUvP;
varying vec2 colorP;

varying float depthVariance;
varying float maskVal;

float sampleDepth(vec2 pos)
  {
    float depth;

    vec2 vUv = vec2( pos.x / (width*2.0), pos.y / (height*2.0)+0.5 );
    vec2 vUv2 = vec2( pos.x / (width*2.0)+0.5, pos.y / (height*2.0)+0.5 );

    vec4 depthColor = texture2D( map, vUv );

    depth = ( depthColor.r + depthColor.g + depthColor.b ) / 3.0 ;

    if (depth>0.99)
    {
      vec4 depthColor2 = texture2D( map, vUv2 );
      float depth2 = ( depthColor2.r + depthColor2.g + depthColor2.b ) / 3.0 ;
      depth = 0.99+depth2;
    }

    return depth;
  }

float median(float a, float b, float c)
  {
    float r=a;

    if ( (a<b) && (b<c) )
    {
      r = b;
    }
    if ( (a<c) && (c<b) )
    {
      r = c;
    }
    return r;
  }

float variance(float d1, float d2, float d3, float d4, float d5, float d6, float d7, float d8, float d9)
  {
    float mean = (d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8 + d9) / 9.0;
    float t1 = (d1-mean);
    float t2 = (d2-mean);
    float t3 = (d3-mean);
    float t4 = (d4-mean);
    float t5 = (d5-mean);
    float t6 = (d6-mean);
    float t7 = (d7-mean);
    float t8 = (d8-mean);
    float t9 = (d9-mean);
    float v = (t1*t1+t2*t2+t3*t3+t4*t4+t5*t5+t6*t6+t7*t7+t8*t8+t9*t9)/9.0;
    return v;
  }

vec2 decodeDepth(vec2 pos)
  {
    vec2 ret;


    float depth1 = sampleDepth(vec2(position.x-1.0, position.y-1.0));
    float depth2 = sampleDepth(vec2(position.x, position.y-1.0));
    float depth3 = sampleDepth(vec2(position.x+1.0, position.y-1.0));
    float depth4 = sampleDepth(vec2(position.x-1.0, position.y));
    float depth5 = sampleDepth(vec2(position.x, position.y));
    float depth6 = sampleDepth(vec2(position.x+1.0, position.y));
    float depth7 = sampleDepth(vec2(position.x-1.0, position.y+1.0));
    float depth8 = sampleDepth(vec2(position.x, position.y+1.0));
    float depth9 = sampleDepth(vec2(position.x+1.0, position.y+1.0));

    float median1 = median(depth1, depth2, depth3);
    float median2 = median(depth4, depth5, depth6);
    float median3 = median(depth7, depth8, depth9);

    ret.x = median(median1, median2, median3);
    ret.y = variance(depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9);

    return ret;

  }


void main() {

  vUvP = vec2( position.x / (width*2.0), position.y / (height*2.0)+0.5 );
  colorP = vec2( position.x / (width*2.0)+0.5 , position.y / (height*2.0)  );

  vec4 pos = vec4(0.0,0.0,0.0,0.0);
  depthVariance = 0.0;

  if ( (vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>0.0))
  {
    vec2 smp = decodeDepth(vec2(position.x, position.y));
    float depth = smp.x;
    depthVariance = smp.y;

    float z = -depth;

    pos = vec4(
      ( position.x / width - 0.5 ) * z * 0.5 * maxDepthPerTile * resolutionFactor * (1000.0/focallength) * -1.0,
      ( position.y / height - 0.5 ) * z * 0.5 * maxDepthPerTile * resolutionFactor * (1000.0/focallength),
      (- z + zOffset / 1000.0) * maxDepthPerTile,
      1.0);

    vec2 maskP = vec2( position.x / (width*2.0), position.y / (height*2.0)  );
    vec4 maskColor = texture2D( map, maskP );
    maskVal = ( maskColor.r + maskColor.g + maskColor.b ) / 3.0 ;
  }

  gl_PointSize = pointSize;
  gl_Position = projectionMatrix * modelViewMatrix * pos;

}`;
const fragmentShader = `uniform sampler2D map;
uniform float varianceThreshold;
uniform float whiteness;

varying vec2 vUvP;
varying vec2 colorP;

varying float depthVariance;
varying float maskVal;


void main() {

  vec4 color;

  if ( (depthVariance>varianceThreshold) || (maskVal>0.5) ||(vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>1.0))
  {
    discard;
  }
  else
  {
    color = texture2D( map, colorP );

    float fader = whiteness /100.0;

    color.r = color.r * (1.0-fader)+ fader;

    color.g = color.g * (1.0-fader)+ fader;

    color.b = color.b * (1.0-fader)+ fader;

    color.a = 1.0;//smoothstep( 20000.0, -20000.0, gl_FragCoord.z / gl_FragCoord.w );
  }

  gl_FragColor = vec4( color.r, color.g, color.b, color.a );

}`;
class DepthCloudObject extends three.Object3D {
    constructor(url, options = DEFAULT_OPTIONS_DEPTHCLOUD) {
        super();
        this.options = DEFAULT_OPTIONS_DEPTHCLOUD;
        this.metaLoaded = false;
        this.options = {
            ...DEFAULT_OPTIONS_DEPTHCLOUD,
            ...options,
        };
        this.url = url;
        this.animate = this.animate.bind(this);
        this.onMetaLoaded = this.onMetaLoaded.bind(this);
    }
    onMetaLoaded() {
        this.metaLoaded = true;
        this.initStreamer();
    }
    isMjpeg() {
        const { streamType } = this.options;
        return streamType.toLowerCase() === 'mjpeg';
    }
    initVideo() {
        const { height, width } = this.options;
        this.video = document.createElement(this.isMjpeg() ? 'img' : 'video');
        this.video.width = width;
        this.video.height = height;
        this.video.addEventListener(this.isMjpeg() ? 'load' : 'loadedmetadata', this.onMetaLoaded, false);
        if (!this.isMjpeg()) {
            assertIsHTMLVideoElement(this.video);
            this.video.loop = true;
        }
        this.video.src = this.url;
        this.video.crossOrigin = 'Anonymous';
        this.video.setAttribute('crossorigin', 'Anonymous');
    }
    initStreamer() {
        const { f, height, maxDepthPerTile, pointSize, varianceThreshold, whiteness, width, } = this.options;
        const resolutionFactor = Math.max(width, height) / 1024;
        if (this.metaLoaded) {
            this.texture = new three.Texture(this.video);
            this.geometry = new three.Geometry();
            const halfWidth = width / 2;
            const halfHeight = height / 2;
            for (let i = 0, l = halfWidth * halfHeight; i < l; i += 1) {
                this.geometry.vertices.push(new three.Vector3(i % halfWidth, i / halfWidth, 0));
            }
            this.material = new three.ShaderMaterial({
                uniforms: {
                    map: { value: this.texture },
                    width: { value: halfWidth },
                    height: { value: halfHeight },
                    focallength: { value: f },
                    pointSize: { value: pointSize },
                    zOffset: { value: 0 },
                    whiteness: { value: whiteness },
                    varianceThreshold: { value: varianceThreshold },
                    maxDepthPerTile: { value: maxDepthPerTile },
                    resolutionFactor: { value: resolutionFactor },
                },
                vertexShader,
                fragmentShader,
            });
            this.mesh = new three.Points(this.geometry, this.material);
            this.mesh.frustumCulled = false;
            this.mesh.position.set(0, 0, 0);
            this.add(this.mesh);
            requestAnimationFrame(this.animate);
        }
    }
    animate() {
        var _a;
        if (!this.texture) {
            return;
        }
        if (this.isMjpeg()) {
            this.texture.needsUpdate = true;
        }
        else {
            assertIsHTMLVideoElement(this.video);
            if (this.video.readyState === this.video.HAVE_ENOUGH_DATA) {
                this.texture.needsUpdate = true;
            }
        }
        if ((_a = this.video) === null || _a === void 0 ? void 0 : _a.src) {
            requestAnimationFrame(this.animate);
        }
    }
    startStream() {
        if (!this.isMjpeg()) {
            assertIsHTMLVideoElement(this.video);
            this.video.play();
        }
    }
    stopStream() {
        var _a;
        if (!this.isMjpeg()) {
            assertIsHTMLVideoElement(this.video);
            this.video.pause();
        }
        (_a = this.video) === null || _a === void 0 ? void 0 : _a.removeAttribute('src');
    }
}

class DepthCloud extends LiveCore {
    constructor(url, options = DEFAULT_OPTIONS_DEPTHCLOUD) {
        super({
            sources: [],
            options: {
                ...DEFAULT_OPTIONS_DEPTHCLOUD,
                ...options,
            },
        });
        this.subscribe = () => {
            var _a, _b;
            (_a = this.object) === null || _a === void 0 ? void 0 : _a.initVideo();
            (_b = this.object) === null || _b === void 0 ? void 0 : _b.startStream();
        };
        this.unsubscribe = () => {
            var _a;
            (_a = this.object) === null || _a === void 0 ? void 0 : _a.stopStream();
        };
        this.object = new DepthCloudObject(url, options);
    }
}

// @ts-nocheck
const MapControls2D = function (object, domElement) {
    domElement = domElement !== undefined ? domElement : document;
    // API
    this.enabled = true;
    this.center = new three.Vector3();
    this.panSpeed = 0.01;
    this.zoomSpeed = 0.05;
    this.rotationSpeed = 0.005;
    // internals
    const scope = this;
    const vector = new three.Vector3();
    const delta = new three.Vector3();
    const box = new three.Box3();
    const STATE = { NONE: -1, ROTATE: 0, ZOOM: 1, PAN: 2 };
    let state = STATE.NONE;
    const { center } = this;
    const normalMatrix = new three.Matrix3();
    const pointer = new three.Vector2();
    const pointerOld = new three.Vector2();
    const spherical = new three.Spherical();
    const sphere = new three.Sphere();
    // events
    const changeEvent = { type: 'change' };
    this.focus = function (target) {
        let distance;
        box.setFromObject(target);
        if (box.isEmpty() === false) {
            box.getCenter(center);
            distance = box.getBoundingSphere(sphere).radius;
        }
        else {
            // Focusing on an Group, AmbientLight, etc
            center.setFromMatrixPosition(target.matrixWorld);
            distance = 0.1;
        }
        delta.set(0, 0, 1);
        delta.applyQuaternion(object.quaternion);
        delta.multiplyScalar(distance * 4);
        object.position.copy(center).add(delta);
        scope.dispatchEvent(changeEvent);
    };
    this.pan = function (positionDelta) {
        // var distance = object.position.distanceTo( center );
        const { zoom } = object;
        positionDelta.multiplyScalar(scope.panSpeed / zoom);
        positionDelta.applyMatrix3(normalMatrix.getNormalMatrix(object.matrix));
        object.position.add(positionDelta);
        center.add(positionDelta);
        scope.dispatchEvent(changeEvent);
    };
    this.zoom = function (zoomDelta) {
        object.zoom -= (zoomDelta.z || 0) * object.zoom * 0.1;
        object.updateProjectionMatrix();
        scope.dispatchEvent(changeEvent);
    };
    this.rotate = function (rotationDelta) {
        vector.copy(object.position).sub(center);
        // spherical.setFromVector3( vector );
        spherical.setFromCartesianCoords(-1 * vector.x, vector.z, vector.y);
        spherical.theta += rotationDelta.x * scope.rotationSpeed;
        // spherical.phi += rotationDelta.y * scope.rotationSpeed;
        spherical.makeSafe();
        const tempRelPosition = vector.setFromSpherical(spherical);
        vector.set(-1 * tempRelPosition.x, tempRelPosition.z, tempRelPosition.y);
        object.position.copy(center).add(vector);
        object.lookAt(center);
        scope.dispatchEvent(changeEvent);
    };
    // mouse
    function onMouseMove(event) {
        if (scope.enabled === false)
            return;
        pointer.set(event.clientX, event.clientY);
        const movementX = pointer.x - pointerOld.x;
        const movementY = pointer.y - pointerOld.y;
        if (state === STATE.ROTATE) {
            scope.rotate(delta.set(-movementX, -movementY, 0));
        }
        else if (state === STATE.ZOOM) {
            scope.zoom(delta.set(0, 0, movementY));
        }
        else if (state === STATE.PAN) {
            scope.pan(delta.set(-movementX, movementY, 0));
        }
        pointerOld.set(event.clientX, event.clientY);
    }
    function onMouseUp() {
        domElement.removeEventListener('mousemove', onMouseMove, false);
        domElement.removeEventListener('mouseup', onMouseUp, false);
        domElement.removeEventListener('mouseout', onMouseUp, false);
        domElement.removeEventListener('dblclick', onMouseUp, false);
        state = STATE.NONE;
    }
    function onMouseWheel(event) {
        event.preventDefault();
        // Normalize deltaY due to https://bugzilla.mozilla.org/show_bug.cgi?id=1392460
        scope.zoom(delta.set(0, 0, event.deltaY > 0 ? 1 : -1));
    }
    function onMouseDown(event) {
        if (scope.enabled === false)
            return;
        if (event.button === 0) {
            state = STATE.ROTATE;
        }
        else if (event.button === 1) {
            state = STATE.ZOOM;
        }
        else if (event.button === 2) {
            state = STATE.PAN;
        }
        pointerOld.set(event.clientX, event.clientY);
        domElement.addEventListener('mousemove', onMouseMove, false);
        domElement.addEventListener('mouseup', onMouseUp, false);
        domElement.addEventListener('mouseout', onMouseUp, false);
        domElement.addEventListener('dblclick', onMouseUp, false);
    }
    function contextmenu(event) {
        event.preventDefault();
    }
    domElement.addEventListener('contextmenu', contextmenu, false);
    domElement.addEventListener('mousedown', onMouseDown, false);
    domElement.addEventListener('wheel', onMouseWheel, false);
    // touch
    const touches = [new three.Vector3(), new three.Vector3(), new three.Vector3()];
    const prevTouches = [new three.Vector3(), new three.Vector3(), new three.Vector3()];
    let prevDistance = null;
    function touchStart(event) {
        if (scope.enabled === false)
            return;
        switch (event.touches.length) {
            case 1:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                break;
            case 2:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[1].pageX, event.touches[1].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                prevDistance = touches[0].distanceTo(touches[1]);
                break;
        }
        prevTouches[0].copy(touches[0]);
        prevTouches[1].copy(touches[1]);
    }
    function touchMove(event) {
        if (scope.enabled === false)
            return;
        event.preventDefault();
        event.stopPropagation();
        function getClosest(touch, allTouches) {
            let closest = allTouches[0];
            for (const i in allTouches) {
                if (closest.distanceTo(touch) > allTouches[i].distanceTo(touch)) {
                    closest = allTouches[i];
                }
            }
            return closest;
        }
        switch (event.touches.length) {
            case 1:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                scope.rotate(touches[0]
                    .sub(getClosest(touches[0], prevTouches))
                    .multiplyScalar(-1));
                break;
            case 2:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[1].pageX, event.touches[1].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                const distance = touches[0].distanceTo(touches[1]);
                scope.zoom(delta.set(0, 0, prevDistance - distance));
                prevDistance = distance;
                const offset0 = touches[0]
                    .clone()
                    .sub(getClosest(touches[0], prevTouches));
                const offset1 = touches[1]
                    .clone()
                    .sub(getClosest(touches[1], prevTouches));
                offset0.x = -offset0.x;
                offset1.x = -offset1.x;
                scope.pan(offset0.add(offset1));
                break;
        }
        prevTouches[0].copy(touches[0]);
        prevTouches[1].copy(touches[1]);
    }
    domElement.addEventListener('touchstart', touchStart, false);
    domElement.addEventListener('touchmove', touchMove, false);
    this.dispose = function () {
        domElement.removeEventListener('contextmenu', contextmenu, false);
        domElement.removeEventListener('mousedown', onMouseDown, false);
        domElement.removeEventListener('wheel', onMouseWheel, false);
        domElement.removeEventListener('mousemove', onMouseMove, false);
        domElement.removeEventListener('mouseup', onMouseUp, false);
        domElement.removeEventListener('mouseout', onMouseUp, false);
        domElement.removeEventListener('dblclick', onMouseUp, false);
        domElement.removeEventListener('touchstart', touchStart, false);
        domElement.removeEventListener('touchmove', touchMove, false);
    };
};
MapControls2D.prototype = Object.create(three.EventDispatcher.prototype);
MapControls2D.prototype.constructor = MapControls2D;

const ResizeObserver$1 = window.ResizeObserver || resizeObserver.ResizeObserver;
class Viewer2d {
    constructor(scene, options = DEFAULT_OPTIONS_SCENE) {
        this.options = DEFAULT_OPTIONS_SCENE;
        this.previousWidth = 0;
        this.previousHeight = 0;
        this.options = {
            ...DEFAULT_OPTIONS_SCENE,
            ...options,
        };
        this.scene = scene || new Scene();
        this.ro = new ResizeObserver$1((entries) => {
            if (entries.length > 0) {
                this.onWindowResize();
            }
        });
        this.initCamera();
        this.animate = this.animate.bind(this);
        this.onWindowResize = this.onWindowResize.bind(this);
    }
    animate() {
        var _a;
        this.scene.stats.begin();
        this.scene.updateMatrixWorld();
        assertIsDefined(this.camera);
        (_a = this.renderer) === null || _a === void 0 ? void 0 : _a.render(this.scene, this.camera);
        this.scene.stats.end();
        requestAnimationFrame(this.animate);
    }
    initCamera() {
        this.camera = new three.OrthographicCamera(-100, 100, 100, -100, 0.1, 1000);
        this.camera.zoom = 0.5;
        this.camera.position.set(0, 0, 10);
        this.camera.up = new three.Vector3(0, 0, 1);
        this.camera.lookAt(new three.Vector3());
        this.scene.add(this.camera);
    }
    setContainer(domNode) {
        this.container = domNode;
        this.initRenderer();
        // @ts-ignore
        this.controls = new MapControls2D(this.camera, this.container);
        this.controls.enableDamping = true;
        this.ro.observe(this.container);
        requestAnimationFrame(this.animate);
        this.onWindowResize();
    }
    initRenderer() {
        const renderer = new three.WebGLRenderer({ antialias: true });
        renderer.autoClear = false;
        renderer.setPixelRatio(window.devicePixelRatio);
        assertIsDefined(this.container);
        renderer.setSize(this.container.offsetWidth, this.container.offsetHeight);
        this.renderer = renderer;
        this.container.appendChild(renderer.domElement);
    }
    updateOptions(options = DEFAULT_OPTIONS_SCENE) {
        this.options = {
            ...this.options,
            ...options,
        };
        this.scene.updateOptions(this.options);
    }
    destroy() {
        this.ro.unobserve(this.container);
    }
    onWindowResize() {
        var _a;
        assertIsDefined(this.container);
        assertIsDefined(this.camera);
        const { offsetHeight, offsetWidth } = this.container;
        if (Math.abs(offsetWidth - this.previousWidth) > 10 ||
            Math.abs(offsetHeight - this.previousHeight) > 10) {
            const [cameraWidth, cameraHeight] = [
                offsetWidth / 100,
                offsetHeight / 100,
            ];
            this.camera.left = -cameraWidth / 2;
            this.camera.right = cameraWidth / 2;
            this.camera.top = cameraHeight / 2;
            this.camera.bottom = -cameraHeight / 2;
            this.camera.updateProjectionMatrix();
            (_a = this.renderer) === null || _a === void 0 ? void 0 : _a.setSize(offsetWidth, offsetHeight);
            this.previousWidth = offsetWidth;
            this.previousHeight = offsetHeight;
        }
    }
    addVisualization(vizObject) {
        this.scene.addVisualization(vizObject);
    }
}

// @ts-nocheck
const EditorControls = function (object, domElement) {
    domElement = domElement !== undefined ? domElement : document;
    // API
    this.enabled = true;
    this.center = new three.Vector3();
    this.panSpeed = 0.002;
    this.zoomSpeed = 0.1;
    this.rotationSpeed = 0.005;
    // internals
    const scope = this;
    const vector = new three.Vector3();
    const delta = new three.Vector3();
    const box = new three.Box3();
    const STATE = { NONE: -1, ROTATE: 0, ZOOM: 1, PAN: 2 };
    let state = STATE.NONE;
    const { center } = this;
    const normalMatrix = new three.Matrix3();
    const pointer = new three.Vector2();
    const pointerOld = new three.Vector2();
    const spherical = new three.Spherical();
    const sphere = new three.Sphere();
    // events
    const changeEvent = { type: 'change' };
    this.focus = function (target) {
        let distance;
        box.setFromObject(target);
        if (box.isEmpty() === false) {
            box.getCenter(center);
            distance = box.getBoundingSphere(sphere).radius;
        }
        else {
            // Focusing on an Group, AmbientLight, etc
            center.setFromMatrixPosition(target.matrixWorld);
            distance = 0.1;
        }
        delta.set(0, 0, 1);
        delta.applyQuaternion(object.quaternion);
        delta.multiplyScalar(distance * 4);
        object.position.copy(center).add(delta);
        scope.dispatchEvent(changeEvent);
    };
    this.pan = function (delta) {
        const distance = object.position.distanceTo(center);
        delta.multiplyScalar(distance * scope.panSpeed);
        delta.applyMatrix3(normalMatrix.getNormalMatrix(object.matrix));
        object.position.add(delta);
        center.add(delta);
        scope.dispatchEvent(changeEvent);
    };
    this.zoom = function (delta) {
        const distance = object.position.distanceTo(center);
        delta.multiplyScalar(distance * scope.zoomSpeed);
        if (delta.length() > distance)
            return;
        delta.applyMatrix3(normalMatrix.getNormalMatrix(object.matrix));
        object.position.add(delta);
        scope.dispatchEvent(changeEvent);
    };
    this.rotate = function (delta) {
        vector.copy(object.position).sub(center);
        // spherical.setFromVector3( vector );
        spherical.setFromCartesianCoords(-1 * vector.x, vector.z, vector.y);
        spherical.theta += delta.x * scope.rotationSpeed;
        spherical.phi += delta.y * scope.rotationSpeed;
        spherical.makeSafe();
        const tempRelPosition = vector.setFromSpherical(spherical);
        vector.set(-1 * tempRelPosition.x, tempRelPosition.z, tempRelPosition.y);
        object.position.copy(center).add(vector);
        object.lookAt(center);
        scope.dispatchEvent(changeEvent);
    };
    // mouse
    function onMouseDown(event) {
        if (scope.enabled === false)
            return;
        if (event.button === 0) {
            state = STATE.ROTATE;
        }
        else if (event.button === 1) {
            state = STATE.ZOOM;
        }
        else if (event.button === 2) {
            state = STATE.PAN;
        }
        pointerOld.set(event.clientX, event.clientY);
        domElement.addEventListener('mousemove', onMouseMove, false);
        domElement.addEventListener('mouseup', onMouseUp, false);
        domElement.addEventListener('mouseout', onMouseUp, false);
        domElement.addEventListener('dblclick', onMouseUp, false);
    }
    function onMouseMove(event) {
        if (scope.enabled === false)
            return;
        pointer.set(event.clientX, event.clientY);
        const movementX = pointer.x - pointerOld.x;
        const movementY = pointer.y - pointerOld.y;
        if (state === STATE.ROTATE) {
            scope.rotate(delta.set(-movementX, -movementY, 0));
        }
        else if (state === STATE.ZOOM) {
            scope.zoom(delta.set(0, 0, movementY));
        }
        else if (state === STATE.PAN) {
            scope.pan(delta.set(-movementX, movementY, 0));
        }
        pointerOld.set(event.clientX, event.clientY);
    }
    function onMouseUp(event) {
        domElement.removeEventListener('mousemove', onMouseMove, false);
        domElement.removeEventListener('mouseup', onMouseUp, false);
        domElement.removeEventListener('mouseout', onMouseUp, false);
        domElement.removeEventListener('dblclick', onMouseUp, false);
        state = STATE.NONE;
    }
    function onMouseWheel(event) {
        event.preventDefault();
        // Normalize deltaY due to https://bugzilla.mozilla.org/show_bug.cgi?id=1392460
        scope.zoom(delta.set(0, 0, event.deltaY > 0 ? 1 : -1));
    }
    function contextmenu(event) {
        event.preventDefault();
    }
    this.dispose = function () {
        domElement.removeEventListener('contextmenu', contextmenu, false);
        domElement.removeEventListener('mousedown', onMouseDown, false);
        domElement.removeEventListener('wheel', onMouseWheel, false);
        domElement.removeEventListener('mousemove', onMouseMove, false);
        domElement.removeEventListener('mouseup', onMouseUp, false);
        domElement.removeEventListener('mouseout', onMouseUp, false);
        domElement.removeEventListener('dblclick', onMouseUp, false);
        domElement.removeEventListener('touchstart', touchStart, false);
        domElement.removeEventListener('touchmove', touchMove, false);
    };
    domElement.addEventListener('contextmenu', contextmenu, false);
    domElement.addEventListener('mousedown', onMouseDown, false);
    domElement.addEventListener('wheel', onMouseWheel, false);
    // touch
    const touches = [new three.Vector3(), new three.Vector3(), new three.Vector3()];
    const prevTouches = [new three.Vector3(), new three.Vector3(), new three.Vector3()];
    let prevDistance = null;
    function touchStart(event) {
        if (scope.enabled === false)
            return;
        switch (event.touches.length) {
            case 1:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                break;
            case 2:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[1].pageX, event.touches[1].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                prevDistance = touches[0].distanceTo(touches[1]);
                break;
        }
        prevTouches[0].copy(touches[0]);
        prevTouches[1].copy(touches[1]);
    }
    function touchMove(event) {
        if (scope.enabled === false)
            return;
        event.preventDefault();
        event.stopPropagation();
        function getClosest(touch, touches) {
            let closest = touches[0];
            for (const i in touches) {
                if (closest.distanceTo(touch) > touches[i].distanceTo(touch)) {
                    closest = touches[i];
                }
            }
            return closest;
        }
        switch (event.touches.length) {
            case 1:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                scope.rotate(touches[0]
                    .sub(getClosest(touches[0], prevTouches))
                    .multiplyScalar(-1));
                break;
            case 2:
                touches[0]
                    .set(event.touches[0].pageX, event.touches[0].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                touches[1]
                    .set(event.touches[1].pageX, event.touches[1].pageY, 0)
                    .divideScalar(window.devicePixelRatio);
                let distance = touches[0].distanceTo(touches[1]);
                scope.zoom(delta.set(0, 0, prevDistance - distance));
                prevDistance = distance;
                let offset0 = touches[0]
                    .clone()
                    .sub(getClosest(touches[0], prevTouches));
                let offset1 = touches[1]
                    .clone()
                    .sub(getClosest(touches[1], prevTouches));
                offset0.x = -offset0.x;
                offset1.x = -offset1.x;
                scope.pan(offset0.add(offset1));
                break;
        }
        prevTouches[0].copy(touches[0]);
        prevTouches[1].copy(touches[1]);
    }
    domElement.addEventListener('touchstart', touchStart, false);
    domElement.addEventListener('touchmove', touchMove, false);
};
EditorControls.prototype = Object.create(three.EventDispatcher.prototype);
EditorControls.prototype.constructor = EditorControls;

const ResizeObserver = window.ResizeObserver || resizeObserver.ResizeObserver;
class Viewer3d {
    constructor(scene, options = {}) {
        this.options = {};
        this.previousWidth = 0;
        this.previousHeight = 0;
        this.options = {
            ...DEFAULT_OPTIONS_SCENE,
            ...options,
        };
        this.scene =
            scene || new Scene(this.options);
        this.ro = new ResizeObserver((entries) => {
            if (entries.length > 0) {
                this.onWindowResize();
            }
        });
        this.initCamera();
        this.animate = this.animate.bind(this);
        this.onWindowResize = this.onWindowResize.bind(this);
    }
    animate() {
        var _a;
        this.scene.stats.begin();
        this.scene.updateMatrixWorld();
        assertIsDefined(this.camera);
        (_a = this.renderer) === null || _a === void 0 ? void 0 : _a.render(this.scene, this.camera);
        this.scene.stats.end();
        requestAnimationFrame(this.animate);
    }
    initCamera() {
        this.camera = new three.PerspectiveCamera(50, 1, 0.01, 1000);
        this.camera.position.set(0, 5, 10);
        this.camera.up = new three.Vector3(0, 0, 1);
        this.camera.lookAt(new three.Vector3());
        this.scene.add(this.camera);
    }
    setContainer(domNode) {
        this.container = domNode;
        this.initRenderer();
        // @ts-ignore
        this.controls = new EditorControls(this.camera, this.container);
        this.controls.enableDamping = true;
        this.ro.observe(this.container);
        requestAnimationFrame(this.animate);
        this.onWindowResize();
    }
    initRenderer() {
        const renderer = new three.WebGLRenderer({ antialias: true });
        renderer.autoClear = false;
        renderer.setPixelRatio(window.devicePixelRatio);
        assertIsDefined(this.container);
        renderer.setSize(this.container.offsetWidth, this.container.offsetHeight);
        this.renderer = renderer;
        this.container.appendChild(renderer.domElement);
    }
    updateOptions(options = DEFAULT_OPTIONS_SCENE) {
        this.options = {
            ...this.options,
            ...options,
        };
        this.scene.updateOptions(this.options);
    }
    destroy() {
        this.ro.unobserve(this.container);
    }
    onWindowResize() {
        var _a;
        assertIsDefined(this.container);
        assertIsDefined(this.camera);
        const { offsetHeight, offsetWidth } = this.container;
        if (Math.abs(offsetWidth - this.previousWidth) > 10 ||
            Math.abs(offsetHeight - this.previousHeight) > 10) {
            this.camera.aspect = offsetWidth / offsetHeight;
            this.camera.updateProjectionMatrix();
            (_a = this.renderer) === null || _a === void 0 ? void 0 : _a.setSize(offsetWidth, offsetHeight);
            this.previousWidth = offsetWidth;
            this.previousHeight = offsetHeight;
        }
    }
    addVisualization(vizObject) {
        this.scene.addVisualization(vizObject);
    }
}

class TfViewer extends Viewer3d {
    constructor(rosInstance, options = {}) {
        super(null, {
            ...DEFAULT_OPTIONS_TF_VIEWER,
            ...options,
        });
        this.options = {};
        this.framesList = [];
        this.ros = rosInstance;
        this.onFramesListUpdate = options.onFramesListUpdate || (() => { });
        this.initRosEvents();
        this.getTFMessages = this.getTFMessages.bind(this);
        this.setFrameTransform = this.setFrameTransform.bind(this);
        this.addRobot = this.addRobot.bind(this);
        this.onRosConnection = this.onRosConnection.bind(this);
    }
    initRosEvents() {
        this.ros.on('connection', () => {
            this.onRosConnection();
        });
        // types for ros are not upto date
        if (this.ros.isConnected) {
            this.onRosConnection();
        }
    }
    onRosConnection() {
        // types for ros are not upto date
        // @ts-ignore
        this.ros.getTopics((rosTopics) => {
            ['/tf', '/tf_static'].forEach(name => {
                const topic = new ROSLIB__default["default"].Topic({
                    ros: this.ros,
                    name,
                    messageType: rosTopics.types[rosTopics.topics.indexOf(name)],
                });
                topic.subscribe(this.getTFMessages);
            });
        });
    }
    getTFMessages(args) {
        const { transforms } = args;
        transforms.forEach(({ header: { frame_id: parentFrameId }, child_frame_id: childFrameId, transform: { rotation: { w: rw, x: rx, y: ry, z: rz }, translation: { x, y, z }, }, }) => {
            const [childObject, parentObject] = [
                this.getObjectOrCreate(`${childFrameId}-tf-connector`),
                this.getObjectOrCreate(`${parentFrameId}-tf-connector`),
            ];
            parentObject.add(childObject);
            childObject.position.set(x, y, z);
            childObject.quaternion.set(rx, ry, rz, rw);
            [parentFrameId, childFrameId].forEach(frame => {
                if (this.framesList.indexOf(`${frame}-tf-connector`) === -1) {
                    this.framesList.push(`${frame}-tf-connector`);
                }
            });
        });
        this.setFrameTransform();
    }
    getObjectOrCreate(frameId) {
        const { scene: { vizWrapper }, } = this;
        if (this.framesList.indexOf(frameId) === -1) {
            this.framesList.push(frameId);
            this.onFramesListUpdate(this.framesList);
        }
        const existingFrame = vizWrapper.getObjectByName(frameId);
        if (existingFrame) {
            return existingFrame;
        }
        const newFrame = new three.Group();
        newFrame.name = frameId;
        this.scene.addObject(newFrame);
        return newFrame;
    }
    setFrameTransform() {
        const { options: { selectedFrame }, scene: { vizWrapper }, } = this;
        if (!selectedFrame) {
            return;
        }
        const currentFrameObject = this.getObjectOrCreate(selectedFrame);
        if (currentFrameObject) {
            vizWrapper.position.set(0, 0, 0);
            vizWrapper.quaternion.set(0, 0, 0, 1);
            vizWrapper.updateMatrixWorld();
            const worldPos = new three.Vector3();
            const worldQuat = new three.Quaternion();
            currentFrameObject.getWorldQuaternion(worldQuat);
            const { w: quatw, x: quatx, y: quaty, z: quatz } = worldQuat;
            vizWrapper.quaternion.set(-quatx, -quaty, -quatz, quatw);
            vizWrapper.updateMatrixWorld();
            currentFrameObject.getWorldPosition(worldPos);
            const oppPos = worldPos.negate();
            vizWrapper.position.set(oppPos.x, oppPos.y, oppPos.z);
        }
    }
    addVisualization(vizObject) {
        super.addVisualization(vizObject);
        vizObject.onHeaderChange = newFrameId => {
            const frameObject = this.getObjectOrCreate(`${newFrameId}-tf-connector`);
            frameObject.add(vizObject.object);
        };
    }
    attachObjectOutsideTree(object) {
        if (!object.frameId) {
            return;
        }
        const frameObject = this.getObjectOrCreate(`${object.frameId}-tf-connector`);
        frameObject.attach(object);
    }
    addRobot(robotModel) {
        robotModel.loadFromParam(() => {
            var _a;
            super.addVisualization(robotModel);
            const links = ((_a = robotModel.object) === null || _a === void 0 ? void 0 : _a.children[0]).links;
            const linkNames = Object.keys(links);
            linkNames.map(linkName => {
                const link = links[linkName];
                const connector = this.getObjectOrCreate(`${linkName}-tf-connector`);
                connector.add(link);
            });
        });
    }
    setFixedFrame() { }
}

class InteractiveMarkerManager {
    constructor(rootObject, viewer, options) {
        this.markerManagerMap = {};
        this.controlsMap = {};
        this.objectMap = {};
        this.namespaces = {};
        this.callback = () => null;
        this.object = rootObject;
        this.viewer = viewer;
        this.hideOtherHandlesOnSelect = options.hideOtherHandlesOnSelect;
        this.hideOtherControlsInstancesOnSelect =
            options.hideOtherControlsInstancesOnSelect;
        this.visible = options.visible;
        this.onChange = this.onChange.bind(this);
    }
    getMarkerManagerOrCreate(interactiveMarkerName) {
        const id = interactiveMarkerName;
        if (!this.markerManagerMap[id] || !this.objectMap[id]) {
            const markersHolder = new Group();
            markersHolder.name = id || '';
            this.markerManagerMap[id] = new MarkerManager(markersHolder, this.onChange);
            this.objectMap[id] = markersHolder;
            this.object.add(markersHolder);
        }
        return {
            manager: this.markerManagerMap[id],
            object: this.objectMap[id],
        };
    }
    onChange() {
        if (this.callback) {
            this.callback();
        }
    }
    onNamespaceChange(callback) {
        this.callback = callback;
    }
    initMarkers(interactiveMarker, controlsManager, visible) {
        let { controls, header: { frame_id }, name, pose: { orientation, position }, scale, } = interactiveMarker;
        if (scale === 0) {
            scale = 1;
        }
        const { manager, object } = this.getMarkerManagerOrCreate(name);
        object.visible = visible;
        object.frameId = frame_id;
        if (this.viewer.attachObjectOutsideTree) {
            this.viewer.attachObjectOutsideTree(object);
        }
        object.setTransform({ translation: position, rotation: orientation });
        object.userData.control = {
            frameId: interactiveMarker.header.frame_id,
            markerName: interactiveMarker.name,
        };
        object.userData.handlesControlsMap = {};
        controls.forEach((control, index) => {
            var _a, _b;
            // cannot rely on the control.name being present or unique
            const key = name;
            if (this.controlsMap[key] === undefined) {
                this.controlsMap[key] = [];
            }
            if (((_a = this.controlsMap[key]) === null || _a === void 0 ? void 0 : _a.length) !== controls.length) {
                const attachMode = control.orientation_mode ===
                    INTERACTIVE_MARKER_ORIENTATION_MODES.FIXED
                    ? threeFreeformControls.ANCHOR_MODE.FIXED
                    : threeFreeformControls.ANCHOR_MODE.INHERIT;
                const controls = controlsManager.anchor(object, {
                    separationT: {
                        x: 0.4,
                        y: 0.4,
                        z: 0.4,
                    },
                    orientation: control.orientation,
                    mode: attachMode,
                    hideOtherHandlesOnSelect: this.hideOtherHandlesOnSelect,
                    hideOtherControlsInstancesOnSelect: this
                        .hideOtherControlsInstancesOnSelect,
                    showHelperPlane: true,
                });
                controls.scale.set(scale, scale, scale);
                controls.visible = visible;
                InteractiveMarkerManager.enableControls(object, control.interaction_mode, control.orientation_mode, control.name, controls, randomColor__default["default"]({
                    seed: `${key}-${control.name}-${2 * index}`, // 2 * for more variable color
                }));
                (_b = this.controlsMap[key]) === null || _b === void 0 ? void 0 : _b.push(controls);
            }
            if (control.markers.length > 0) {
                control.markers.forEach(marker => {
                    manager.updateMarker(marker);
                });
            }
        });
    }
    setVisible(visible) {
        this.visible = visible;
        const controlGroups = Object.values(this.controlsMap);
        controlGroups.map(controls => {
            controls === null || controls === void 0 ? void 0 : controls.map(manager => {
                manager.visible = visible;
                manager.object.visible = visible;
            });
        });
    }
    updatePose(markerMessage) {
        const { name, pose } = markerMessage;
        const markerObject = this.objectMap[name];
        if (markerObject && pose) {
            const { orientation, position } = pose;
            markerObject.setTransform({
                translation: position,
                rotation: orientation,
            });
        }
    }
    static enableControls(object, interactionMode, orientationMode, controlName, controls, givenColor) {
        controls.showAll(false);
        const controlsManagerOrientation = new three.Quaternion();
        controls.getWorldQuaternion(controlsManagerOrientation);
        const controlsManagerRotation = new three.Euler().setFromQuaternion(controlsManagerOrientation);
        const alignmentColor = InteractiveMarkerManager.getAlignmentColor(controlsManagerRotation);
        const color = alignmentColor !== null && alignmentColor !== void 0 ? alignmentColor : givenColor;
        // currently only ROTATE_AXIS is supported for VIEW_FACING mode
        // this is the most useful one
        if (UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES.indexOf(orientationMode) !== -1 &&
            interactionMode === INTERACTIVE_MARKER_INTERACTION_MODES.ROTATE_AXIS) {
            object.userData.handlesControlsMap = {
                ...object.userData.handlesControlsMap,
                [threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.ER]: controlName,
            };
            controls.showByNames([threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.ER], true);
            return;
        }
        let handles = [];
        switch (interactionMode) {
            case INTERACTIVE_MARKER_INTERACTION_MODES.NONE:
            case INTERACTIVE_MARKER_INTERACTION_MODES.MENU:
            case INTERACTIVE_MARKER_INTERACTION_MODES.BUTTON: {
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.MOVE_AXIS: {
                handles = [
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XPT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XNT,
                ];
                controls.translationXP.setColor(color);
                controls.translationXN.setColor(color);
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.MOVE_PLANE: {
                handles = [threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.PICK_PLANE_YZ];
                controls.pickPlaneYZ.setColor(color);
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.ROTATE_AXIS: {
                handles = [threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XR];
                controls.rotationX.setColor(color);
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.MOVE_ROTATE: {
                handles = [
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XR,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.PICK_PLANE_YZ,
                ];
                controls.pickPlaneYZ.setColor(color);
                controls.rotationX.setColor(color);
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.MOVE_3D: {
                handles = [
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XPT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XNT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.YPT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.YNT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.ZPT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.ZNT,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.PICK,
                ];
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.ROTATE_3D: {
                handles = [
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.XR,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.YR,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.ZR,
                ];
                break;
            }
            case INTERACTIVE_MARKER_INTERACTION_MODES.MOVE_ROTATE_3D: {
                // these handles are currently commented out
                // because these overlap with the extra 6-DOF controls
                // that are available in the example and also in kompose
                // proper fix requires increasing the scale of the below
                // handles so that they don't overlap
                handles = [
                    // DEFAULT_HANDLE_GROUP_NAME.XPT,
                    // DEFAULT_HANDLE_GROUP_NAME.XNT,
                    // DEFAULT_HANDLE_GROUP_NAME.YPT,
                    // DEFAULT_HANDLE_GROUP_NAME.YNT,
                    // DEFAULT_HANDLE_GROUP_NAME.ZPT,
                    // DEFAULT_HANDLE_GROUP_NAME.ZNT,
                    // DEFAULT_HANDLE_GROUP_NAME.XR,
                    // DEFAULT_HANDLE_GROUP_NAME.YR,
                    // DEFAULT_HANDLE_GROUP_NAME.ZR,
                    threeFreeformControls.DEFAULT_HANDLE_GROUP_NAME.PICK,
                ];
                break;
            }
        }
        handles.map(handle => {
            object.userData.handlesControlsMap[handle] = controlName;
        });
        controls.showByNames(handles, true);
    }
    static getAlignmentColor(rotation) {
        const threshold = 10 ** -3;
        const [x, y, z] = rotation.toArray();
        for (let i = -4; i <= 4; i++) {
            const angle = (i * Math.PI) / 2;
            if (Math.abs(x - angle) < threshold &&
                Math.abs(y) < threshold &&
                Math.abs(z) < threshold) {
                return 'red';
            }
            if (Math.abs(y - angle) < threshold &&
                Math.abs(x) < threshold &&
                Math.abs(z) < threshold) {
                return 'green';
            }
            if (Math.abs(z - angle) < threshold &&
                Math.abs(y) < threshold &&
                Math.abs(x) < threshold) {
                return 'blue';
            }
        }
        return null;
    }
    removeObject(id) {
        var _a;
        const obj = this.objectMap[id];
        (_a = obj.parent) === null || _a === void 0 ? void 0 : _a.remove(obj);
        delete this.objectMap[id];
    }
    reset(destroy) {
        this.onChange();
        if (destroy) {
            Object.keys(this.objectMap).forEach(id => {
                this.removeObject(id);
            });
        }
    }
}

class InteractiveMarkers extends LegacyCore {
    constructor(ros, topicName, viewer, options = DEFAULT_OPTIONS_INTERACTIVE_MARKER) {
        super(ros, topicName, MESSAGE_TYPE_INTERACTIVEMARKER, {
            ...DEFAULT_OPTIONS_INTERACTIVE_MARKER,
            ...options,
        });
        this.init = false;
        this.object = new Group();
        this.interactiveMarkersNames = new Set();
        this.interactiveMarkersFrameIds = new Set();
        this.clientId = `amphion-${Math.round(Math.random() * 10 ** 8)}`;
        this.messageSequence = 0;
        this.feedbackTopic = null;
        this.init = false;
        this.viewer = viewer;
        this.updateOptions({
            ...DEFAULT_OPTIONS_INTERACTIVE_MARKER,
            ...options,
        });
        this.interactiveMarkerManager = new InteractiveMarkerManager(this.object, viewer, this.options);
        const { publishThrottleRate } = this.options;
        this.debouncedPublish = debounce__default["default"](this.publish.bind(this), publishThrottleRate && publishThrottleRate > 0 ? publishThrottleRate : 0);
        this.initFreeformControls();
        this.publishManual = this.publishManual.bind(this);
    }
    hide() {
        super.hide();
        this.interactiveMarkerManager.setVisible(false);
    }
    show() {
        super.show();
        this.interactiveMarkerManager.setVisible(true);
    }
    destroy() {
        var _a;
        super.destroy();
        (_a = this.freeformControls) === null || _a === void 0 ? void 0 : _a.destroy();
        this.freeformControls = null;
        this.interactiveMarkerManager.reset(true);
        this.interactiveMarkersNames = new Set();
    }
    initFreeformControls() {
        const { camera, controls, renderer, scene } = this.viewer;
        assertIsDefined(camera);
        assertIsDefined(renderer);
        this.freeformControls = new threeFreeformControls.ControlsManager(camera, renderer.domElement);
        scene.add(this.freeformControls);
        this.freeformControls.listen(threeFreeformControls.RAYCASTER_EVENTS.DRAG_START, () => {
            controls.enabled = false;
        });
        // TODO: need to update three-freeform-controls
        // @ts-ignore
        this.freeformControls.listen(threeFreeformControls.RAYCASTER_EVENTS.DRAG, (object, handleName) => this.debouncedPublish(object, handleName));
        this.freeformControls.listen(threeFreeformControls.RAYCASTER_EVENTS.DRAG_STOP, 
        // TODO: need to update three-freeform-controls
        // @ts-ignore
        (object, handleName) => {
            this.debouncedPublish(object, handleName);
            controls.enabled = true;
        });
    }
    static makeInteractiveMarkerFeedbackMessage(args) {
        const { seq, frame_id: frameId, client_id: clientId, marker_name: markerName, control_name: controlName, position, quaternion, } = args;
        return new ROSLIB__default["default"].Message({
            header: {
                seq,
                frame_id: frameId,
                stamp: {
                    secs: 0,
                    nsecs: 0,
                },
            },
            client_id: clientId,
            marker_name: markerName,
            control_name: controlName,
            pose: {
                position: {
                    x: position.x,
                    y: position.y,
                    z: position.z,
                },
                orientation: {
                    x: quaternion.x,
                    y: quaternion.y,
                    z: quaternion.z,
                    w: quaternion.w,
                },
            },
            event_type: 1,
            menu_entry_id: 0,
            mouse_point: {
                x: 0,
                y: 0,
                z: 0,
            },
            mouse_point_valid: false,
        });
    }
    static makeInteractiveMarkerFeedbackTopic(ros, name) {
        return new ROSLIB__default["default"].Topic({
            ros: ros,
            name: name,
            messageType: MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK,
        });
    }
    publish(object, handleName) {
        var _a;
        if (!object) {
            return;
        }
        const { frameId, markerName } = object.userData.control;
        const controlName = object.userData.handlesControlsMap[handleName];
        const message = InteractiveMarkers.makeInteractiveMarkerFeedbackMessage({
            seq: this.messageSequence,
            client_id: this.clientId,
            frame_id: frameId,
            marker_name: markerName,
            control_name: controlName,
            position: object.position,
            quaternion: object.quaternion,
        });
        if (this.feedbackTopic !== null) {
            (_a = this.feedbackTopic) === null || _a === void 0 ? void 0 : _a.publish(message);
        }
        this.messageSequence++;
    }
    publishManual(pose) {
        const message = InteractiveMarkers.makeInteractiveMarkerFeedbackMessage({
            seq: this.messageSequence,
            client_id: this.clientId,
            ...pose,
            frame_id: Array.from(this.interactiveMarkersFrameIds)[0],
            marker_name: Array.from(this.interactiveMarkersNames)[0],
        });
        if (this.feedbackTopic !== null) {
            this.feedbackTopic.publish(message);
        }
        this.messageSequence++;
    }
    updateOptions(options) {
        if (options.feedbackTopicName !== undefined) {
            if (!this.feedbackTopic ||
                this.feedbackTopic.name !== options.feedbackTopicName.name) {
                assertIsDefined(this.ros);
                this.feedbackTopic = InteractiveMarkers.makeInteractiveMarkerFeedbackTopic(this.ros, options.feedbackTopicName);
            }
        }
        else {
            this.feedbackTopic = null;
        }
        // need a better way to handle interdependent topics
        const shouldSubscriptionChange = this.options.updateTopicName !== options.topicName && this.init;
        const guardAgainstOtherOptionsChange = this.topicName === this.options.updateTopicName;
        if (shouldSubscriptionChange && options.updateTopicName !== undefined) {
            const { messageType, name } = options.updateTopicName;
            this.changeTopic(name, messageType, true);
        }
        else if (shouldSubscriptionChange && guardAgainstOtherOptionsChange) {
            this.unsubscribe();
        }
        super.updateOptions(options);
    }
    update(message) {
        super.update(message);
        if (message.markers && message.markers.length > 0) {
            message.markers.forEach(interactiveMarker => {
                if (!this.interactiveMarkersNames.has(interactiveMarker.name)) {
                    this.interactiveMarkerManager.initMarkers(interactiveMarker, this.freeformControls, this.options.visible);
                }
                this.interactiveMarkersNames.add(interactiveMarker.name);
                if (interactiveMarker.header) {
                    this.interactiveMarkersFrameIds.add(interactiveMarker.header.frame_id);
                }
                message.markers.forEach(poseObject => {
                    this.interactiveMarkerManager.updatePose(poseObject);
                });
            });
            // need a better way to handle interdependent topics
            if (!this.init) {
                this.init = true;
                if (this.options.updateTopicName !== undefined) {
                    const { messageType, name } = this.options.updateTopicName;
                    this.init = true;
                    this.changeTopic(name, messageType, true);
                }
                else {
                    this.unsubscribe();
                }
            }
        }
        // for InteractiveMarkerUpdate sub-message (InteractiveMarkerPose)
        if (message.poses && message.poses.length > 0) {
            message.poses.forEach(pose => {
                this.interactiveMarkerManager.updatePose(pose);
            });
        }
        // for InteractiveMarkerUpdate sub-message
        if (message.erases) ;
    }
    reset() {
        this.interactiveMarkerManager.reset(false);
    }
}

const { open } = require('rosbag');
class RosbagBucket {
    constructor() {
        this.files = new Set();
        this.topics = [];
        this.readers = {};
        this.addFile = async (file) => {
            if (this.files.has(file)) {
                throw new Error('file already exists in the bucket');
            }
            this.files.add(file);
            this.readers[file.name] = { topics: {}, queue: [] };
            await this.processFile(file);
        };
        this.removeFile = async (file, cb) => {
            if (!this.files.has(file)) {
                throw new Error('file does not exist in the bucket');
            }
            this.files.delete(file);
            clearTimeout(this.readers[file.name].timer);
            clearTimeout(this.readers[file.name].queueStartWaitTimer);
            this.readers[file.name] = {
                topics: {},
                queue: [],
            };
            this.topics = this.topics.filter(x => x.rosbagFileName !== file.name);
            cb();
        };
        this.addReader = (topic, fileName, reader) => {
            var _a;
            const topicReadersMap = this.readers[fileName].topics;
            if (!topicReadersMap) {
                throw new Error(`add ${fileName} to the bucket first`);
            }
            if (!topicReadersMap[topic]) {
                topicReadersMap[topic] = new Set();
            }
            (_a = topicReadersMap[topic]) === null || _a === void 0 ? void 0 : _a.add(reader);
        };
        this.removeReader = (topic, fileName, reader) => {
            var _a, _b;
            const topicReadersMap = this.readers[fileName].topics;
            if (!topicReadersMap) {
                throw new Error(`add ${fileName} to the bucket first`);
            }
            if (!((_a = topicReadersMap[topic]) === null || _a === void 0 ? void 0 : _a.has(reader))) {
                throw new Error('reader has not yet been added to the bucket');
            }
            (_b = topicReadersMap[topic]) === null || _b === void 0 ? void 0 : _b.delete(reader);
        };
        this.processQueue = (file) => {
            const { queue } = this.readers[file.name];
            if (queue.length === 0) {
                this.readers[file.name].queueStartWaitTimer = window.setTimeout(() => {
                    this.processQueue(file);
                }, 500);
                return;
            }
            this.enqueueProcessing(0, file);
        };
        this.enqueueProcessing = (index, file) => {
            const { queue } = this.readers[file.name];
            const first = queue[index];
            const second = queue[index + 1];
            this.processBagReadResult(first, file);
            if (!second) {
                // loop over at queue end
                // note that this loop also starts if the queue is still being
                // constructed while the message consumption outpaces the construction
                this.enqueueProcessing(0, file);
                return;
            }
            const { sec: secFirst, nsec: nsecFirst } = first.timestamp;
            const { sec, nsec } = second.timestamp;
            const firstTimestampMs = Math.floor(secFirst * 10 ** 3 + nsecFirst / 10 ** 6);
            const timestampMs = Math.floor(sec * 10 ** 3 + nsec / 10 ** 6);
            let diff = 0;
            if (timestampMs > firstTimestampMs) {
                diff = timestampMs - firstTimestampMs;
            }
            // setTimeout is throttled to 4ms in HTML5 spec
            // not bypassing it in this case will cause the visualisation to slow down
            if (diff < 4) {
                this.enqueueProcessing(index + 1, file);
            }
            else {
                this.readers[file.name].timer = window.setTimeout(() => {
                    this.enqueueProcessing(index + 1, file);
                }, diff);
            }
        };
        this.processBagReadResult = (result, file) => {
            const topicsReadersMap = this.readers[file.name].topics;
            const globalReaders = topicsReadersMap['*'];
            const specificReaders = topicsReadersMap[result.topic];
            globalReaders === null || globalReaders === void 0 ? void 0 : globalReaders.forEach(reader => reader(file, result));
            specificReaders === null || specificReaders === void 0 ? void 0 : specificReaders.forEach(reader => reader(file, result));
        };
    }
    async processFile(file) {
        const bag = await open(file);
        Object.keys(bag.connections).forEach((id) => {
            const connection = bag.connections[id];
            const { topic, type } = connection;
            const existing = this.topics.findIndex(x => x.name === topic && x.rosbagFileName === file.name);
            if (existing === -1) {
                this.topics.push({
                    name: topic,
                    messageType: type,
                    rosbagFileName: file.name,
                });
            }
        });
        bag.readMessages({}, (result) => {
            this.readers[file.name].queue.push(result);
        });
        // processing starts without waiting for the full bag read to finish
        // this enables us to implement remote rosbags
        // to wait for the full bag read before processing, use await on
        // bag.readMessages above
        this.processQueue(file);
    }
}

class RosbagDataSource {
    constructor(options) {
        var _a;
        this.createdAt = new Date();
        this.isStreamPaused = false;
        this.internalListener = null;
        this.listeners = new Set();
        this.addListener = (listener) => {
            if (this.listeners.has(listener)) {
                return { success: false, reason: 'listener already present' };
            }
            this.listeners.add(listener);
            this.stream.addListener(listener);
            return { success: true };
        };
        this.removeListener = (listener) => {
            if (!this.listeners.has(listener)) {
                return { success: false, reason: 'listener not present' };
            }
            this.listeners.delete(listener);
            this.stream.removeListener(listener);
            return { success: true };
        };
        this.removeAllListeners = () => {
            this.listeners.forEach(listener => {
                this.stream.removeListener(listener);
            });
            this.listeners.clear();
            return { success: true };
        };
        this.pause = () => {
            this.isStreamPaused = true;
            return { success: true };
        };
        this.resume = () => {
            this.isStreamPaused = false;
            return { success: true };
        };
        this.hasMemory = (_a = options.memory) !== null && _a !== void 0 ? _a : false;
        this.bucket = options.bucket;
        this.producer = {
            start: listener => {
                this.internalListener = (file, bagReadResult) => {
                    if (this.isStreamPaused) {
                        return;
                    }
                    listener.next(bagReadResult.message);
                };
                this.bucket.addReader(options.topicName, options.fileName, this.internalListener);
            },
            stop: () => {
                if (!this.internalListener) {
                    return;
                }
                this.bucket.removeReader(options.topicName, options.fileName, this.internalListener);
            },
        };
        this.stream = this.hasMemory
            ? xs__default["default"].createWithMemory(this.producer)
            : xs__default["default"].create(this.producer);
    }
}

var index = {
    EditorControls,
    CollisionObject,
    DepthCloud,
    DisplayTrajectory,
    PlanningScene,
    Point,
    Polygon,
    Pose,
    Wrench,
    PoseArray,
    Tf,
    RobotModel,
    MarkerArray,
    LaserScan,
    Map: Map$1,
    Odometry,
    Path,
    Image,
    ImageStream,
    Marker,
    Range,
    Scene,
    Viewer2d,
    Viewer3d,
    TfViewer,
    InteractiveMarkers,
    RosbagBucket,
    RosTopicDataSource,
    RosbagDataSource,
    CONSTANTS,
};

module.exports = index;
//# sourceMappingURL=rosnav.js.map
