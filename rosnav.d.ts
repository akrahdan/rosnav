import ROSLIB, { Ros, Message } from 'roslib';
import { Object3D, Mesh as Mesh$1, BoxGeometry, MeshStandardMaterial, CylinderGeometry, ConeGeometry, Group as Group$1, LoadingManager, Color, Vector3, Quaternion, Scene as Scene$1, WebGLRenderer, PerspectiveCamera } from 'three';
import { Listener } from 'xstream';
import URDFLoader from 'urdf-js/umd/URDFLoader';
import Stats from 'stats-js';

declare class LegacyCore {
    options: {
        [p: string]: any;
    };
    readonly ros: Ros | null;
    private headerFrameId;
    private readonly onHeaderChange;
    object?: Object3D | null;
    private dataSourceInstances?;
    topicName?: string | Array<{
        name: string;
        messageType: string;
    }>;
    messageType?: string;
    constructor(ros: Ros | null, resourceName: string | null, messageType: string, options?: {
        [p: string]: any;
    });
    hide(): void;
    show(): void;
    destroy(): void;
    reset(): void;
    subscribe(): void;
    unsubscribe(): void;
    update(message: RosMessage.Base): void;
    updateOptions(options: {
        [p: string]: any;
    }): void;
    changeTopic(resourceName: string | Array<{
        name: string;
        messageType: string;
    }>, type: string, autoSubscribe?: boolean): void;
}

declare class Mesh extends Mesh$1 {
    setTransform(transform: {
        translation: RosMessage.Point;
        rotation: RosMessage.Quaternion;
    }): void;
    setScale(scale?: RosMessage.Point): void;
    setColor(colors: RosMessage.Color | string | null): void;
    setAlpha(alpha: number): void;
}

declare class Box extends Mesh {
    readonly geometry: BoxGeometry;
    readonly material: MeshStandardMaterial;
    constructor();
}

declare class CollisionObject extends LegacyCore {
    object: Object3D;
    constructor(ros?: Ros, topicName?: string, options?: {});
    update(message: RosMessage.CollisionObject): void;
    static getNewPrimitive(args: {
        type: number;
        dimensions: number[];
    }): Box | null;
}

declare class DisplayTrajectory extends LegacyCore {
    private readonly robotClone;
    private lastMessage?;
    private loopbackId?;
    private poseRemovalId?;
    private pointsUpdateIds;
    constructor(ros: Ros, topicName: string, options?: any);
    updateOptions(options: any): void;
    resetLoopback(): void;
    update(message: RosMessage.DisplayTrajectory, loopback?: boolean): void;
}

declare class PlanningScene extends LegacyCore {
    private readonly collisionObjectViz;
    private readonly attachedCollisionObjects;
    constructor(ros: Ros, topicName: string, options?: {});
    update(message: RosMessage.PlanningScene): void;
}

declare class Cylinder extends Mesh {
    geometry: CylinderGeometry;
    material: MeshStandardMaterial;
    constructor(color?: string, radius?: number, height?: number);
}

declare class Cone extends Mesh {
    readonly material: MeshStandardMaterial;
    readonly geometry: ConeGeometry;
    constructor(color?: string);
}

declare class Arrow extends Group$1 {
    readonly cone: Cone;
    readonly cylinder: Cylinder;
    constructor();
    setTransform(transform: {
        translation: RosMessage.Point;
        rotation: RosMessage.Quaternion;
    }): void;
    setColor(color: {
        cone: RosMessage.Color;
        cylinder: RosMessage.Color;
    }): void;
    setHeadDimensions(dimensions: {
        radius?: number;
        length?: number;
    }): void;
    setShaftDimensions(dimensions: {
        radius?: number;
        length?: number;
    }): void;
    setAlpha(alpha: number): void;
    setScale(scale: RosMessage.Point): void;
}

declare class Group extends Group$1 {
    setTransform(transform: {
        translation: RosMessage.Point;
        rotation: RosMessage.Quaternion;
    }): void;
    setScale(scale: RosMessage.Point): void;
    setColor(colors: RosMessage.Color): void;
}

declare class Axes extends Group {
    private readonly x;
    private readonly y;
    private readonly z;
    readonly objectType = "Axes";
    constructor(radius?: number, height?: number);
    setLength(length: number): void;
    setRadius(radius: number): void;
}

declare class LineArrow extends Group {
    private objectType;
    private readonly arrowTop;
    private readonly topPoints;
    private readonly arrowLength;
    constructor();
    setLength(length: number): void;
    setColor(color: RosMessage.Color): void;
}

interface DataSourceAction {
    success: boolean;
    reason?: string;
}
declare abstract class DataSource<T> {
    abstract createdAt: Date;
    abstract hasMemory: boolean;
    abstract resume: () => DataSourceAction;
    abstract pause: () => DataSourceAction;
    abstract addListener: (listener: Listener<T>) => DataSourceAction;
    abstract removeListener: (listener: Listener<T>) => DataSourceAction;
    abstract removeAllListeners: () => DataSourceAction;
}

interface CoreOptions$1<T> {
    sources: Array<DataSource<T>>;
    options?: {
        [k: string]: any;
    };
}
declare class LiveCore<T extends RosMessage.Base, V extends Object3D | HTMLElement> {
    private onHeaderChange;
    private sources;
    protected options: {
        [k: string]: any;
    };
    private headerFrameId;
    object?: V;
    constructor(args: CoreOptions$1<T>);
    hide: () => void;
    show: () => void;
    destroy: () => void;
    reset: () => void;
    subscribe: () => void;
    unsubscribe: () => void;
    update(message: T): void;
    updateOptions(options: {
        [k: string]: any;
    }): void;
    changeSources(sources: Array<DataSource<T>>): void;
}

declare class Pose extends LiveCore<RosMessage.PoseStamped, Group$1> {
    private primitive;
    constructor(source: DataSource<RosMessage.PoseStamped>, options?: {
        type: string;
        axesLength: number;
        axesRadius: number;
        shaftLength: number;
        shaftRadius: number;
        headLength: number;
        headRadius: number;
        color: string;
        alpha: number;
    });
    static getNewPrimitive(options: {
        [k: string]: any;
    }): Arrow | Axes | LineArrow | null;
    updateOptions(options: {
        [k: string]: any;
    }): void;
    update(message: RosMessage.PoseStamped): void;
}

declare class ArrowWithCircle extends Arrow {
    private readonly circleCone;
    readonly material: MeshStandardMaterial;
    private readonly torus;
    constructor();
    setTorusDimensions(args: {
        radius: number;
        tube: number;
    }): void;
    setCircleConeDimensions(args: {
        radius: number;
        length: number;
    }): void;
    setColor(args: {
        [object: string]: RosMessage.Color | string;
    }): void;
    setAlpha(alpha: number): void;
}

declare class Wrench extends LiveCore<RosMessage.WrenchStamped, Group$1> {
    private primitiveX;
    private primitiveY;
    constructor(source: DataSource<RosMessage.WrenchStamped>, options?: {
        type: string;
        circleConeLength: number;
        circleConeRadius: number;
        circleRadius: number;
        tube: number;
        radialSegments: number;
        tubularSegments: number;
        arc: number;
        shaftLength: number;
        shaftRadius: number;
        headLength: number;
        headRadius: number;
        forceColor: string;
        torqueColor: string;
        alpha: number;
        forceArrowScale: number;
        torqueArrowScale: number;
        arrowWidth: number;
    });
    static getNewPrimitive(options: {
        type: string;
    }): Arrow | null;
    getOrUpdatePrimitive(primitive: Arrow | ArrowWithCircle | null, type: string): Arrow | null;
    updateOptions(options: {
        [p: string]: any;
    }): void;
    update(message: RosMessage.WrenchStamped): void;
}

declare class Polygon extends LegacyCore {
    readonly object: Group;
    private readonly line;
    constructor(ros: Ros, topicName: string, options?: {
        color: string;
        alpha: number;
    });
    updateOptions(options: any): void;
    update(message: RosMessage.PolygonStamped): void;
}

declare class TfFrame extends Group {
    readonly arrow: Arrow;
    constructor(frameId: string);
    static getName(frameId: string): string;
}

declare class Tf extends LiveCore<RosMessage.TFMessage, Group$1> {
    constructor(source: DataSource<RosMessage.TFMessage>, options?: {});
    update(message: RosMessage.TFMessage): void;
    getFrameOrCreate(frameId: string): TfFrame;
}

declare class URDFCore<V extends Object3D> extends URDFLoader {
    private readonly param;
    protected options: {
        [k: string]: any;
    };
    private urdfObject?;
    private readonly packages;
    object?: V;
    constructor(ros: Ros, paramName: string, options?: any);
    onComplete(object: Object3D): void;
    loadFromParam(onComplete?: (object: Object3D) => void, options?: {}): void;
    loadURDF(urdfString: string, onComplete: ((object: Object3D) => void) | undefined, options: any): void;
    defaultLoadMeshCallback(path: string, ext: LoadingManager, done: (mesh: Object3D) => void): void;
    hide: () => void;
    show: () => void;
    destroy: () => void;
    reset: () => void;
    updateOptions(options: {
        [k: string]: any;
    }): void;
}

declare class RobotModel extends URDFCore<Object3D> {
    constructor(ros: Ros, paramName: string, options?: {});
}

declare class Point extends LiveCore<RosMessage.PointStamped, Group> {
    private readonly sphere;
    constructor(source: DataSource<RosMessage.PointStamped>, options?: {
        color: string;
        alpha: number;
        radius: number;
    });
    updateOptions(options: {
        [p: string]: any;
    }): void;
    update(message: RosMessage.PointStamped): void;
}

declare class MarkerArray extends LiveCore<RosMessage.MarkerArray, Group> {
    private markerManager;
    private onNameSpaceChange?;
    constructor(source: DataSource<RosMessage.MarkerArray>, options?: {
        queueSize: number;
        namespaces: never[];
        throttleRate: number;
    });
    updateOptions(options: {
        [p: string]: any;
    }): void;
    update(message: RosMessage.MarkerArray): void;
    reset: () => void;
}

declare class LaserScan extends LiveCore<RosMessage.LaserScan, Group> {
    private points;
    private readonly sphereList;
    private readonly cubeList;
    private cachedMessage;
    constructor(source: DataSource<RosMessage.LaserScan>, options?: {
        axis: string;
        autocomputeValueBounds: boolean;
        useFixedFrame: boolean;
        minAxisValue: number;
        maxAxisValue: number;
        channelName: string;
        useRainbow: boolean;
        invertRainbow: boolean;
        minColor: string;
        maxColor: string;
        autocomputeIntensityBounds: boolean;
        maxIntensity: number;
        minIntensity: number;
        selectable: boolean;
        style: string;
        size: number;
        alpha: number;
        decayTime: number;
        queueSize: number;
        compression: string;
        colorTransformer: string;
        flatColor: string;
    });
    getNormalizedIntensity(intensity: number): number;
    applyIntensityTransform(intensity: number, position: RosMessage.Point): Color;
    getNormalizedAxisValue(axisValue: number): number;
    applyAxisColorTransform(position: RosMessage.Point): Color;
    colorTransformer(intensity: number, position: RosMessage.Point): Color | null;
    setupPoints(index: number, position: RosMessage.Point, color: RosMessage.Color | null): void;
    hideAllObjects(): void;
    setStyleDimensions(message: RosMessage.LaserScan): void;
    updateOptions(options: {
        [p: string]: any;
    }): void;
    update(message: RosMessage.LaserScan): void;
}

declare class Plane extends Mesh {
    constructor();
}

interface RosTopicDataSourceOptions {
    ros: Ros;
    topicName: string;
    messageType: string;
    memory?: boolean;
    compression?: 'png' | 'cbor';
    throttleRate?: number;
    queueSize?: number;
    queueLength?: number;
}
declare class RosTopicDataSource<T extends Message> implements DataSource<T> {
    readonly createdAt: Date;
    readonly hasMemory: boolean;
    private readonly ros;
    private readonly topic;
    private readonly producer;
    private stream;
    private isStreamLive;
    private isStreamPaused;
    private internalListener;
    private readonly listeners;
    private readonly rosConnectionHook;
    private rosCloseHook;
    private rosErrorHook;
    constructor(options: RosTopicDataSourceOptions);
    private addRosHealthHooks;
    private removeRosHealthHooks;
    addListener: (listener: Listener<T>) => {
        success: boolean;
        reason: string;
    } | {
        success: boolean;
        reason?: undefined;
    };
    removeListener: (listener: Listener<T>) => {
        success: boolean;
        reason: string;
    } | {
        success: boolean;
        reason?: undefined;
    };
    private cleanStream;
    removeAllListeners: () => {
        success: boolean;
    };
    pause: () => {
        success: boolean;
    };
    resume: () => {
        success: boolean;
    };
}

declare class Map extends LiveCore<RosMessage.OccupancyGrid, Plane> {
    private cachedMessage;
    constructor(source: RosTopicDataSource<RosMessage.OccupancyGrid>, options?: {
        alpha: number;
        colorScheme: string;
        compression: string;
        drawBehind: boolean;
    });
    updateOptions(options: {
        [k: string]: any;
    }): void;
    updateCanvasDimensions(message: RosMessage.OccupancyGrid): void;
    setCanvasData(message: RosMessage.OccupancyGrid): void;
    update(message: RosMessage.OccupancyGrid): void;
}

declare class Odometry extends LiveCore<RosMessage.Odometry, Group> {
    private objectPool;
    private keepSize;
    private currentObjectIndex;
    constructor(source: DataSource<RosMessage.Odometry>, options?: {
        positionTolerance: number;
        angleTolerance: number;
        keep: number;
        arrowLength: number;
        axesLength: number;
        axesRadius: number;
        shaftLength: number;
        shaftRadius: number;
        headLength: number;
        headRadius: number;
        type: string;
        color: string;
        alpha: number;
    });
    setKeepSize(size: number): void;
    removeAllObjects(): void;
    checkToleranceThresholdExceed(newPose: {
        position: Vector3;
        quaternion: Quaternion;
    }): boolean;
    getObject(): Object3D;
    changeObjectPoolType(): void;
    updateOptions(options: {
        [p: string]: any;
    }): void;
    update(message: RosMessage.Odometry): void;
}

declare class PoseArray extends LiveCore<RosMessage.PoseArray, Group$1> {
    constructor(source: DataSource<RosMessage.PoseArray>, options?: {
        type: string;
        arrowLength: number;
        axesLength: number;
        axesRadius: number;
        shaftLength: number;
        shaftRadius: number;
        headLength: number;
        headRadius: number;
        color: string;
        alpha: number;
    });
    update(message: RosMessage.PoseArray): void;
}

declare class Path extends LiveCore<RosMessage.Path, Group> {
    private readonly line;
    constructor(source: DataSource<RosMessage.Path>, options?: {
        color: string;
        alpha: number;
    });
    updateOptions(options: {
        [p: string]: any;
    }): void;
    update(message: RosMessage.Path): void;
}

declare class Image extends LiveCore<RosMessage.Image, HTMLCanvasElement> {
    private readonly shadowObject;
    constructor(source: DataSource<RosMessage.Image>, options?: {
        queueSize: number;
        width: number;
        height: number;
        compression: string;
    });
    applyImageData(message: RosMessage.Image): void;
    updateDimensions(width: number, height: number): void;
    update(message: RosMessage.Image): void;
    hide: () => void;
    show: () => void;
}

interface CoreOptions {
    options?: {
        [k: string]: any;
    };
}
declare class StaticCore<V extends Object3D | HTMLElement> {
    protected options: {
        [k: string]: any;
    };
    object?: V;
    constructor(args: CoreOptions);
    hide: () => void;
    show: () => void;
    destroy: () => void;
    reset: () => void;
    updateOptions(options: {
        [k: string]: any;
    }): void;
}

declare class ImageStream extends StaticCore<HTMLImageElement> {
    private readonly url;
    constructor(url: string, options?: {
        width: number;
        height: number;
    });
    hide: () => void;
    show: () => void;
}

declare class Marker extends LiveCore<RosMessage.Marker, Group> {
    private markerManager;
    private onNameSpaceChange?;
    constructor(source: RosTopicDataSource<RosMessage.Marker>, options?: {
        queueSize: number;
        namespaces: never[];
    });
    updateOptions(options: {
        [k: string]: any;
    }): void;
    update(message: RosMessage.Marker): void;
    reset: () => void;
}

declare class Range extends LiveCore<RosMessage.Range, Group> {
    private readonly cylinder;
    constructor(source: DataSource<RosMessage.Range>, options?: {
        color: string;
        alpha: number;
    });
    update(message: RosMessage.Range): void;
}

declare class Scene extends Scene$1 {
    readonly vizWrapper: Group$1;
    readonly stats: Stats;
    private grid?;
    private options;
    constructor(options?: {
        backgroundColor: number;
        gridSize: number;
        gridDivisions: number;
        gridColor: number;
        gridCenterlineColor: number;
    });
    initLights(): void;
    initGrid(): void;
    addObject(object: Object3D): void;
    addVisualization(args: {
        object?: Object3D;
    }): void;
    getObjectByName(name: string): Object3D | undefined;
    updateOptions(options?: {
        backgroundColor: number;
        gridSize: number;
        gridDivisions: number;
        gridColor: number;
        gridCenterlineColor: number;
    }): void;
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

declare class DepthCloudObject extends Object3D {
    private readonly options;
    private readonly url;
    private metaLoaded;
    private video;
    private texture;
    private geometry;
    private material;
    private mesh;
    constructor(url: string, options?: {
        streamType: string;
        f: number;
        maxDepthPerTile: number;
        pointSize: number;
        width: number;
        height: number;
        whiteness: number;
        varianceThreshold: number;
    });
    onMetaLoaded(): void;
    isMjpeg(): boolean;
    initVideo(): void;
    initStreamer(): void;
    animate(): void;
    startStream(): void;
    stopStream(): void;
}

declare class DepthCloud extends LiveCore<{}, DepthCloudObject> {
    constructor(url: string, options?: {
        streamType: string;
        f: number;
        maxDepthPerTile: number;
        pointSize: number;
        width: number;
        height: number;
        whiteness: number;
        varianceThreshold: number;
    });
    subscribe: () => void;
    unsubscribe: () => void;
}

declare class Viewer2d {
    private options;
    private readonly scene;
    private previousWidth;
    private previousHeight;
    private readonly ro;
    private renderer?;
    private camera?;
    private container?;
    private controls?;
    constructor(scene: Scene, options?: {
        backgroundColor: number;
        gridSize: number;
        gridDivisions: number;
        gridColor: number;
        gridCenterlineColor: number;
    });
    animate(): void;
    initCamera(): void;
    setContainer(domNode: HTMLElement): void;
    initRenderer(): void;
    updateOptions(options?: {
        backgroundColor: number;
        gridSize: number;
        gridDivisions: number;
        gridColor: number;
        gridCenterlineColor: number;
    }): void;
    destroy(): void;
    onWindowResize(): void;
    addVisualization(vizObject: {
        object: Object3D;
    }): void;
}

declare class Viewer3d {
    options: {};
    readonly scene: Scene;
    private previousWidth;
    private previousHeight;
    private readonly ro;
    renderer?: WebGLRenderer;
    camera?: PerspectiveCamera;
    private container?;
    controls?: any;
    constructor(scene: Scene | null, options?: {});
    animate(): void;
    initCamera(): void;
    setContainer(domNode: HTMLElement): void;
    initRenderer(): void;
    updateOptions(options?: {
        backgroundColor: number;
        gridSize: number;
        gridDivisions: number;
        gridColor: number;
        gridCenterlineColor: number;
    }): void;
    destroy(): void;
    onWindowResize(): void;
    addVisualization(vizObject: {
        object?: Object3D;
    }): void;
}

declare class TfViewer extends Viewer3d {
    options: {
        onFramesListUpdate?: () => void;
        selectedFrame?: string;
    };
    private readonly ros;
    private readonly framesList;
    private readonly onFramesListUpdate;
    constructor(rosInstance: Ros, options?: {
        onFramesListUpdate?: (framesList: string[]) => void;
        selectedFrame?: string;
    });
    initRosEvents(): void;
    onRosConnection(): void;
    getTFMessages(args: {
        transforms: RosMessage.TransformStamped[];
    }): void;
    getObjectOrCreate(frameId: string): Object3D;
    setFrameTransform(): void;
    addVisualization(vizObject: {
        object: Object3D;
        onHeaderChange: (frameId: string) => void;
    }): void;
    attachObjectOutsideTree(object: Object3D & {
        frameId?: string;
    }): void;
    addRobot(robotModel: URDFCore<Object3D>): void;
    setFixedFrame(): void;
}

declare class InteractiveMarkers extends LegacyCore {
    private init;
    readonly object: Group;
    private readonly viewer;
    private readonly interactiveMarkerManager;
    private interactiveMarkersNames;
    private readonly interactiveMarkersFrameIds;
    private readonly clientId;
    private messageSequence;
    private feedbackTopic;
    private readonly debouncedPublish;
    private freeformControls?;
    constructor(ros: Ros, topicName: string, viewer: TfViewer, options?: {
        queueSize: number;
        namespaces: never[];
        throttleRate: number;
        publishThrottleRate: number;
        hideOtherHandlesOnSelect: boolean;
        hideOtherControlsInstancesOnSelect: boolean;
    });
    hide(): void;
    show(): void;
    destroy(): void;
    initFreeformControls(): void;
    static makeInteractiveMarkerFeedbackMessage(args: {
        seq: number;
        frame_id: string;
        client_id: string;
        marker_name: string;
        control_name?: string;
        position: RosMessage.Point;
        quaternion: RosMessage.Quaternion;
    }): ROSLIB.Message;
    static makeInteractiveMarkerFeedbackTopic(ros: Ros, name: string): ROSLIB.Topic<ROSLIB.Message>;
    publish(object: Object3D, handleName: string): void;
    publishManual(pose: {
        position: RosMessage.Point;
        quaternion: RosMessage.Quaternion;
    }): void;
    updateOptions(options: any): void;
    update(message: RosMessage.InteractiveMarker & RosMessage.InteractiveMarkerUpdate): void;
    reset(): void;
}

declare const OBJECT_TYPE_ARROW = "Arrow";
declare const OBJECT_TYPE_ARROW_WITH_CIRCLE = "ArrowWithCircle";
declare const OBJECT_TYPE_AXES = "Axes";
declare const OBJECT_TYPE_FLAT_ARROW = "FlatArrow";
declare const MAX_POINTCLOUD_POINTS = 5000000;
declare const DEFAULT_BACKGROUND_COLOR = 15790320;
declare const DEFAULT_GRID_SIZE = 30;
declare const DEFAULT_GRID_DIVISIONS = 30;
declare const DEFAULT_GRID_COLOR = 11184810;
declare const DEFAULT_GRID_COLOR_CENTERLINE = 7368816;
declare const INTERACTIVE_MARKER_ORIENTATION_MODES: {
    INHERIT: number;
    FIXED: number;
    VIEW_FACING: number;
};
declare const UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES: number[];
declare const INTERACTIVE_MARKER_INTERACTION_MODES: {
    NONE: number;
    MENU: number;
    BUTTON: number;
    MOVE_AXIS: number;
    MOVE_PLANE: number;
    ROTATE_AXIS: number;
    MOVE_ROTATE: number;
    MOVE_3D: number;
    ROTATE_3D: number;
    MOVE_ROTATE_3D: number;
};
declare const MESSAGE_TYPE_ROBOT_MODEL = "robot_description";
declare const MESSAGE_TYPE_ACCELSTAMPED = "geometry_msgs/AccelStamped";
declare const MESSAGE_TYPE_POINTSTAMPED = "geometry_msgs/PointStamped";
declare const MESSAGE_TYPE_POINTSTAMPED2 = "geometry_msgs/msg/PointStamped";
declare const MESSAGE_TYPE_POLYGONSTAMPED = "geometry_msgs/PolygonStamped";
declare const MESSAGE_TYPE_POSEARRAY = "geometry_msgs/PoseArray";
declare const MESSAGE_TYPE_POSEARRAY2 = "geometry_msgs/msg/PoseArray";
declare const MESSAGE_TYPE_POSECOVARIANCE = "geometry_msgs/PoseWithCovariance";
declare const MESSAGE_TYPE_POSECOVARIANCE2 = "geometry_msgs/msg/PoseWithCovariance";
declare const MESSAGE_TYPE_POSESTAMPED = "geometry_msgs/PoseStamped";
declare const MESSAGE_TYPE_POSESTAMPED2 = "geometry_msgs/msg/PoseStamped";
declare const MESSAGE_TYPE_TWISTSTAMPED = "geometry_msgs/TwistStamped";
declare const MESSAGE_TYPE_VECTOR3STAMPED = "geometry_msgs/Vector3Stamped";
declare const MESSAGE_TYPE_WRENCHSTAMPED = "geometry_msgs/WrenchStamped";
declare const MESSAGE_TYPE_WRENCHSTAMPED2 = "geometry_msgs/msg/WrenchStamped";
declare const MESSAGE_TYPE_BOUNDINGVOLUME = "moveit_msgs/BoundingVolume";
declare const MESSAGE_TYPE_COLLISION_OBJECT = "moveit_msgs/CollisionObject";
declare const MESSAGE_TYPE_DISPLAYROBOTSTATE = "moveit_msgs/DisplayRobotState";
declare const MESSAGE_TYPE_DISPLAYTRAJECTORY = "moveit_msgs/DisplayTrajectory";
declare const MESSAGE_TYPE_PLANNINGSCENE = "moveit_msgs/PlanningScene";
declare const MESSAGE_TYPE_OCCUPANCYGRID = "nav_msgs/OccupancyGrid";
declare const MESSAGE_TYPE_OCCUPANCYGRID2 = "nav_msgs/msg/OccupancyGrid";
declare const MESSAGE_TYPE_ODOMETRY = "nav_msgs/Odometry";
declare const MESSAGE_TYPE_ODOMETRY2 = "nav_msgs/msg/Odometry";
declare const MESSAGE_TYPE_PATH = "nav_msgs/Path";
declare const MESSAGE_TYPE_PATH2 = "nav_msgs/msg/Path";
declare const MESSAGE_TYPE_COMPRESSEDIMAGE = "sensor_msgs/CompressedImage";
declare const MESSAGE_TYPE_DISPLAYJOINTSTATE = "sensor_msgs/JointState";
declare const MESSAGE_TYPE_IMAGE = "sensor_msgs/Image";
declare const MESSAGE_TYPE_IMAGE2 = "sensor_msgs/msg/Image";
declare const MESSAGE_TYPE_LASERSCAN = "sensor_msgs/LaserScan";
declare const MESSAGE_TYPE_LASERSCAN2 = "sensor_msgs/msg/LaserScan";
declare const MESSAGE_TYPE_MAGNETICFIELD = "sensor_msgs/MagneticField";
declare const MESSAGE_TYPE_POINTCLOUD = "sensor_msgs/PointCloud";
declare const MESSAGE_TYPE_POINTCLOUD2 = "sensor_msgs/PointCloud2";
declare const MESSAGE_TYPE_ROS2POINTCLOUD2 = "sensor_msgs/msg/PointCloud2";
declare const MESSAGE_TYPE_RANGE = "sensor_msgs/Range";
declare const MESSAGE_TYPE_RANGE2 = "sensor_msgs/msg/Range";
declare const MESSAGE_TYPE_TF = "tf/tfMessage";
declare const MESSAGE_TYPE_ROS2_TF = "tf/msg/tfMessage";
declare const MESSAGE_TYPE_TF2 = "tf2_msgs/TFMessage";
declare const MESSAGE_TYPE_ROS2_TF2 = "tf2_msgs/msg/TFMessage";
declare const MESSAGE_TYPE_MARKER = "visualization_msgs/Marker";
declare const MESSAGE_TYPE_MARKER2 = "visualization_msgs/msg/Marker";
declare const MESSAGE_TYPE_MARKERARRAY = "visualization_msgs/MarkerArray";
declare const MESSAGE_TYPE_MARKERARRAY2 = "visualization_msgs/msg/MarkerArray";
declare const MESSAGE_TYPE_INTERACTIVEMARKER = "visualization_msgs/InteractiveMarkerInit";
declare const MESSAGE_TYPE_INTERACTIVEMARKER2 = "visualization_msgs/msg/InteractiveMarkerInit";
declare const MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE = "visualization_msgs/InteractiveMarkerUpdate";
declare const MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2 = "visualization_msgs/msg/InteractiveMarkerUpdate";
declare const MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK = "visualization_msgs/InteractiveMarkerFeedback";
declare const MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2 = "visualization_msgs/msg/InteractiveMarkerFeedback";
/** ***************************
 *   Visualizations
 * ************************** */
declare const VIZ_TYPE_IMAGE = "Image";
declare const VIZ_TYPE_INTERACTIVEMARKER = "InteractiveMarker";
declare const VIZ_TYPE_LASERSCAN = "LaserScan";
declare const VIZ_TYPE_MAP = "Map";
declare const VIZ_TYPE_MARKER = "Marker";
declare const VIZ_TYPE_MARKERARRAY = "MarkerArray";
declare const VIZ_TYPE_ODOMETRY = "Odometry";
declare const VIZ_TYPE_PATH = "Path";
declare const VIZ_TYPE_POINT = "Point";
declare const VIZ_TYPE_POINTCLOUD = "PointCloud";
declare const VIZ_TYPE_POLYGON = "Polygon";
declare const VIZ_TYPE_POSE = "Pose";
declare const VIZ_TYPE_POSEARRAY = "PoseArray";
declare const VIZ_TYPE_RANGE = "Range";
declare const VIZ_TYPE_ROBOTMODEL = "RobotModel";
declare const VIZ_TYPE_TF = "Tf";
declare const VIZ_TYPE_WRENCH = "Wrench";
declare const POINT_FIELD_DATATYPES: {
    INT8: number;
    UINT8: number;
    INT16: number;
    UINT16: number;
    INT32: number;
    UINT32: number;
    FLOAT32: number;
    FLOAT64: number;
};
declare const COLLISION_OBJECT_OPERATIONS: {
    ADD: number;
    REMOVE: number;
    APPEND: number;
    MOVE: number;
};
declare const SOLID_PRIMITIVE_TYPES: {
    BOX: number;
    SPHERE: number;
    CYLINDER: number;
    CONE: number;
};
declare const MARKER_OBJECT_TYPES: {
    ARROW: number;
    CUBE: number;
    SPHERE: number;
    CYLINDER: number;
    LINE_STRIP: number;
    LINE_LIST: number;
    CUBE_LIST: number;
    SPHERE_LIST: number;
    POINTS: number;
    TEXT_VIEW_FACING: number;
    MESH_RESOURCE: number;
    TRIANGLE_LIST: number;
};
declare const LINE_STYLES: {
    LINES: string;
    BILLBOARDS: string;
};
declare const MAP_COLOR_SCHEMES: {
    MAP: string;
    CONST_MAP: string;
    RAW: string;
};
declare const LASERSCAN_STYLES: {
    SQUARES: string;
    POINTS: string;
    FLAT_SQUARES: string;
    SPHERES: string;
    BOXES: string;
};
declare const COLOR_TRANSFORMERS: {
    INTENSITY: string;
    AXIS_COLOR: string;
    FLAT_COLOR: string;
};
declare const POINTCLOUD_COLOR_CHANNELS: {
    RGB: string;
    INTENSITY: string;
};
declare const AXES: {
    X: string;
    Y: string;
    Z: string;
};
declare const INTENSITY_CHANNEL_OPTIONS: {
    X: string;
    Y: string;
    Z: string;
    INTENSITY: string;
};
declare const ODOMETRY_OBJECT_TYPES: {
    arrow: string;
    axes: string;
};
declare const POSE_OBJECT_TYPES: {
    arrow: string;
    axes: string;
    flatArrow: string;
};
declare const WRENCH_OBJECT_TYPES: {
    arrow: string;
    arrowWithCircle: string;
};
/** ***************************
 *   Default Options
 * ************************** */
declare const DEFAULT_CYLINDER_HEIGHT = 1;
declare const DEFAULT_CYLINDER_RADIUS = 1;
declare const DEFAULT_RADIAL_SEGMENTS = 32;
declare const DEFAULT_CONE_HEIGHT = 1;
declare const DEFAULT_CONE_RADIUS = 1;
declare const DEFAULT_COLOR_X_AXIS = "#ff0000";
declare const DEFAULT_COLOR_Y_AXIS = "#008000";
declare const DEFAULT_COLOR_Z_AXIS = "#0000ff";
declare const DEFAULT_COLOR_ARROW = "#f0ff00";
declare const DEFAULT_COLOR_LINE = "#f0ff00";
declare const DEFAULT_OPTIONS_SCENE: {
    backgroundColor: number;
    gridSize: number;
    gridDivisions: number;
    gridColor: number;
    gridCenterlineColor: number;
};
declare const DEFAULT_OPTIONS_TF_VIEWER: {
    selectedFrame: string;
};
declare const DEFAULT_OPTIONS_ARROW: {
    shaftLength: number;
    shaftRadius: number;
    headLength: number;
    headRadius: number;
};
declare const DEFAULT_OPTIONS_AXES: {
    axesLength: number;
    axesRadius: number;
};
declare const DEFAULT_OPTIONS_FLATARROW: {
    arrowLength: number;
};
declare const DEFAULT_OPTIONS_DISPLAYTRAJECTORY: {
    robot: Group$1;
    loop: boolean;
};
declare const DEFAULT_OPTIONS_COLLISION_OBJECT: {};
declare const DEFAULT_OPTIONS_DEPTHCLOUD: {
    streamType: string;
    f: number;
    maxDepthPerTile: number;
    pointSize: number;
    width: number;
    height: number;
    whiteness: number;
    varianceThreshold: number;
};
declare const DEFAULT_OPTIONS_IMAGE: {
    queueSize: number;
    width: number;
    height: number;
    compression: string;
};
declare const DEFAULT_OPTIONS_IMAGE_STREAM: {
    width: number;
    height: number;
};
declare const DEFAULT_OPTIONS_LASERSCAN: {
    axis: string;
    autocomputeValueBounds: boolean;
    useFixedFrame: boolean;
    minAxisValue: number;
    maxAxisValue: number;
    channelName: string;
    useRainbow: boolean;
    invertRainbow: boolean;
    minColor: string;
    maxColor: string;
    autocomputeIntensityBounds: boolean;
    maxIntensity: number;
    minIntensity: number;
    selectable: boolean;
    style: string;
    size: number;
    alpha: number;
    decayTime: number;
    queueSize: number;
    compression: string;
    colorTransformer: string;
    flatColor: string;
};
declare const DEFAULT_OPTIONS_MAP: {
    alpha: number;
    colorScheme: string;
    compression: string;
    drawBehind: boolean;
};
declare const DEFAULT_OPTIONS_MARKER: {
    queueSize: number;
    namespaces: never[];
};
declare const DEFAULT_OPTIONS_MARKERARRAY: {
    queueSize: number;
    namespaces: never[];
    throttleRate: number;
};
declare const DEFAULT_OPTIONS_INTERACTIVE_MARKER: {
    queueSize: number;
    namespaces: never[];
    throttleRate: number;
    publishThrottleRate: number;
    hideOtherHandlesOnSelect: boolean;
    hideOtherControlsInstancesOnSelect: boolean;
};
declare const DEFAULT_OPTIONS_ODOMETRY: {
    positionTolerance: number;
    angleTolerance: number;
    keep: number;
    arrowLength: number;
    axesLength: number;
    axesRadius: number;
    shaftLength: number;
    shaftRadius: number;
    headLength: number;
    headRadius: number;
    type: string;
    color: string;
    alpha: number;
};
declare const DEFAULT_OPTIONS_PATH: {
    color: string;
    alpha: number;
};
declare const DEFAULT_OPTIONS_PLANNINGSCENE: {};
declare const DEFAULT_OPTIONS_POINTCLOUD: {
    compression: string;
    colorChannel: string;
    size: number;
    useRainbow: boolean;
    queueSize: number;
};
declare const DEFAULT_OPTIONS_POINT: {
    color: string;
    alpha: number;
    radius: number;
};
declare const DEFAULT_OPTIONS_POLYGON: {
    color: string;
    alpha: number;
};
declare const DEFAULT_OPTIONS_POSE: {
    type: string;
    axesLength: number;
    axesRadius: number;
    shaftLength: number;
    shaftRadius: number;
    headLength: number;
    headRadius: number;
    color: string;
    alpha: number;
};
declare const DEFAULT_OPTIONS_TORUS: {
    circleRadius: number;
    tube: number;
    radialSegments: number;
    tubularSegments: number;
    arc: number;
};
declare const DEFAULT_OPTIONS_ARROW_WITH_CIRCLE: {
    circleConeLength: number;
    circleConeRadius: number;
    circleRadius: number;
    tube: number;
    radialSegments: number;
    tubularSegments: number;
    arc: number;
    shaftLength: number;
    shaftRadius: number;
    headLength: number;
    headRadius: number;
};
declare const DEFAULT_OPTIONS_WRENCH: {
    type: string;
    circleConeLength: number;
    circleConeRadius: number;
    circleRadius: number;
    tube: number;
    radialSegments: number;
    tubularSegments: number;
    arc: number;
    shaftLength: number;
    shaftRadius: number;
    headLength: number;
    headRadius: number;
    forceColor: string;
    torqueColor: string;
    alpha: number;
    forceArrowScale: number;
    torqueArrowScale: number;
    arrowWidth: number;
};
declare const DEFAULT_OPTIONS_POSEARRAY: {
    type: string;
    arrowLength: number;
    axesLength: number;
    axesRadius: number;
    shaftLength: number;
    shaftRadius: number;
    headLength: number;
    headRadius: number;
    color: string;
    alpha: number;
};
declare const DEFAULT_OPTIONS_RANGE: {
    color: string;
    alpha: number;
};
declare const DEFAULT_OPTIONS_ROBOTMODEL: {};
declare const DEFAULT_OPTIONS_TF: {};
/** ***************************
 *   Zethus exports
 * ************************** */
declare const SUPPORTED_MESSAGE_TYPES: string[];
declare const UNSUPPORTED_MESSAGE_TYPES: string[];

declare const CONSTANTS_OBJECT_TYPE_ARROW: typeof OBJECT_TYPE_ARROW;
declare const CONSTANTS_OBJECT_TYPE_ARROW_WITH_CIRCLE: typeof OBJECT_TYPE_ARROW_WITH_CIRCLE;
declare const CONSTANTS_OBJECT_TYPE_AXES: typeof OBJECT_TYPE_AXES;
declare const CONSTANTS_OBJECT_TYPE_FLAT_ARROW: typeof OBJECT_TYPE_FLAT_ARROW;
declare const CONSTANTS_MAX_POINTCLOUD_POINTS: typeof MAX_POINTCLOUD_POINTS;
declare const CONSTANTS_DEFAULT_BACKGROUND_COLOR: typeof DEFAULT_BACKGROUND_COLOR;
declare const CONSTANTS_DEFAULT_GRID_SIZE: typeof DEFAULT_GRID_SIZE;
declare const CONSTANTS_DEFAULT_GRID_DIVISIONS: typeof DEFAULT_GRID_DIVISIONS;
declare const CONSTANTS_DEFAULT_GRID_COLOR: typeof DEFAULT_GRID_COLOR;
declare const CONSTANTS_DEFAULT_GRID_COLOR_CENTERLINE: typeof DEFAULT_GRID_COLOR_CENTERLINE;
declare const CONSTANTS_INTERACTIVE_MARKER_ORIENTATION_MODES: typeof INTERACTIVE_MARKER_ORIENTATION_MODES;
declare const CONSTANTS_UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES: typeof UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES;
declare const CONSTANTS_INTERACTIVE_MARKER_INTERACTION_MODES: typeof INTERACTIVE_MARKER_INTERACTION_MODES;
declare const CONSTANTS_MESSAGE_TYPE_ROBOT_MODEL: typeof MESSAGE_TYPE_ROBOT_MODEL;
declare const CONSTANTS_MESSAGE_TYPE_ACCELSTAMPED: typeof MESSAGE_TYPE_ACCELSTAMPED;
declare const CONSTANTS_MESSAGE_TYPE_POINTSTAMPED: typeof MESSAGE_TYPE_POINTSTAMPED;
declare const CONSTANTS_MESSAGE_TYPE_POINTSTAMPED2: typeof MESSAGE_TYPE_POINTSTAMPED2;
declare const CONSTANTS_MESSAGE_TYPE_POLYGONSTAMPED: typeof MESSAGE_TYPE_POLYGONSTAMPED;
declare const CONSTANTS_MESSAGE_TYPE_POSEARRAY: typeof MESSAGE_TYPE_POSEARRAY;
declare const CONSTANTS_MESSAGE_TYPE_POSEARRAY2: typeof MESSAGE_TYPE_POSEARRAY2;
declare const CONSTANTS_MESSAGE_TYPE_POSECOVARIANCE: typeof MESSAGE_TYPE_POSECOVARIANCE;
declare const CONSTANTS_MESSAGE_TYPE_POSECOVARIANCE2: typeof MESSAGE_TYPE_POSECOVARIANCE2;
declare const CONSTANTS_MESSAGE_TYPE_POSESTAMPED: typeof MESSAGE_TYPE_POSESTAMPED;
declare const CONSTANTS_MESSAGE_TYPE_POSESTAMPED2: typeof MESSAGE_TYPE_POSESTAMPED2;
declare const CONSTANTS_MESSAGE_TYPE_TWISTSTAMPED: typeof MESSAGE_TYPE_TWISTSTAMPED;
declare const CONSTANTS_MESSAGE_TYPE_VECTOR3STAMPED: typeof MESSAGE_TYPE_VECTOR3STAMPED;
declare const CONSTANTS_MESSAGE_TYPE_WRENCHSTAMPED: typeof MESSAGE_TYPE_WRENCHSTAMPED;
declare const CONSTANTS_MESSAGE_TYPE_WRENCHSTAMPED2: typeof MESSAGE_TYPE_WRENCHSTAMPED2;
declare const CONSTANTS_MESSAGE_TYPE_BOUNDINGVOLUME: typeof MESSAGE_TYPE_BOUNDINGVOLUME;
declare const CONSTANTS_MESSAGE_TYPE_COLLISION_OBJECT: typeof MESSAGE_TYPE_COLLISION_OBJECT;
declare const CONSTANTS_MESSAGE_TYPE_DISPLAYROBOTSTATE: typeof MESSAGE_TYPE_DISPLAYROBOTSTATE;
declare const CONSTANTS_MESSAGE_TYPE_DISPLAYTRAJECTORY: typeof MESSAGE_TYPE_DISPLAYTRAJECTORY;
declare const CONSTANTS_MESSAGE_TYPE_PLANNINGSCENE: typeof MESSAGE_TYPE_PLANNINGSCENE;
declare const CONSTANTS_MESSAGE_TYPE_OCCUPANCYGRID: typeof MESSAGE_TYPE_OCCUPANCYGRID;
declare const CONSTANTS_MESSAGE_TYPE_OCCUPANCYGRID2: typeof MESSAGE_TYPE_OCCUPANCYGRID2;
declare const CONSTANTS_MESSAGE_TYPE_ODOMETRY: typeof MESSAGE_TYPE_ODOMETRY;
declare const CONSTANTS_MESSAGE_TYPE_ODOMETRY2: typeof MESSAGE_TYPE_ODOMETRY2;
declare const CONSTANTS_MESSAGE_TYPE_PATH: typeof MESSAGE_TYPE_PATH;
declare const CONSTANTS_MESSAGE_TYPE_PATH2: typeof MESSAGE_TYPE_PATH2;
declare const CONSTANTS_MESSAGE_TYPE_COMPRESSEDIMAGE: typeof MESSAGE_TYPE_COMPRESSEDIMAGE;
declare const CONSTANTS_MESSAGE_TYPE_DISPLAYJOINTSTATE: typeof MESSAGE_TYPE_DISPLAYJOINTSTATE;
declare const CONSTANTS_MESSAGE_TYPE_IMAGE: typeof MESSAGE_TYPE_IMAGE;
declare const CONSTANTS_MESSAGE_TYPE_IMAGE2: typeof MESSAGE_TYPE_IMAGE2;
declare const CONSTANTS_MESSAGE_TYPE_LASERSCAN: typeof MESSAGE_TYPE_LASERSCAN;
declare const CONSTANTS_MESSAGE_TYPE_LASERSCAN2: typeof MESSAGE_TYPE_LASERSCAN2;
declare const CONSTANTS_MESSAGE_TYPE_MAGNETICFIELD: typeof MESSAGE_TYPE_MAGNETICFIELD;
declare const CONSTANTS_MESSAGE_TYPE_POINTCLOUD: typeof MESSAGE_TYPE_POINTCLOUD;
declare const CONSTANTS_MESSAGE_TYPE_POINTCLOUD2: typeof MESSAGE_TYPE_POINTCLOUD2;
declare const CONSTANTS_MESSAGE_TYPE_ROS2POINTCLOUD2: typeof MESSAGE_TYPE_ROS2POINTCLOUD2;
declare const CONSTANTS_MESSAGE_TYPE_RANGE: typeof MESSAGE_TYPE_RANGE;
declare const CONSTANTS_MESSAGE_TYPE_RANGE2: typeof MESSAGE_TYPE_RANGE2;
declare const CONSTANTS_MESSAGE_TYPE_TF: typeof MESSAGE_TYPE_TF;
declare const CONSTANTS_MESSAGE_TYPE_ROS2_TF: typeof MESSAGE_TYPE_ROS2_TF;
declare const CONSTANTS_MESSAGE_TYPE_TF2: typeof MESSAGE_TYPE_TF2;
declare const CONSTANTS_MESSAGE_TYPE_ROS2_TF2: typeof MESSAGE_TYPE_ROS2_TF2;
declare const CONSTANTS_MESSAGE_TYPE_MARKER: typeof MESSAGE_TYPE_MARKER;
declare const CONSTANTS_MESSAGE_TYPE_MARKER2: typeof MESSAGE_TYPE_MARKER2;
declare const CONSTANTS_MESSAGE_TYPE_MARKERARRAY: typeof MESSAGE_TYPE_MARKERARRAY;
declare const CONSTANTS_MESSAGE_TYPE_MARKERARRAY2: typeof MESSAGE_TYPE_MARKERARRAY2;
declare const CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER: typeof MESSAGE_TYPE_INTERACTIVEMARKER;
declare const CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER2: typeof MESSAGE_TYPE_INTERACTIVEMARKER2;
declare const CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE: typeof MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE;
declare const CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2: typeof MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2;
declare const CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK: typeof MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK;
declare const CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2: typeof MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2;
declare const CONSTANTS_VIZ_TYPE_IMAGE: typeof VIZ_TYPE_IMAGE;
declare const CONSTANTS_VIZ_TYPE_INTERACTIVEMARKER: typeof VIZ_TYPE_INTERACTIVEMARKER;
declare const CONSTANTS_VIZ_TYPE_LASERSCAN: typeof VIZ_TYPE_LASERSCAN;
declare const CONSTANTS_VIZ_TYPE_MAP: typeof VIZ_TYPE_MAP;
declare const CONSTANTS_VIZ_TYPE_MARKER: typeof VIZ_TYPE_MARKER;
declare const CONSTANTS_VIZ_TYPE_MARKERARRAY: typeof VIZ_TYPE_MARKERARRAY;
declare const CONSTANTS_VIZ_TYPE_ODOMETRY: typeof VIZ_TYPE_ODOMETRY;
declare const CONSTANTS_VIZ_TYPE_PATH: typeof VIZ_TYPE_PATH;
declare const CONSTANTS_VIZ_TYPE_POINT: typeof VIZ_TYPE_POINT;
declare const CONSTANTS_VIZ_TYPE_POINTCLOUD: typeof VIZ_TYPE_POINTCLOUD;
declare const CONSTANTS_VIZ_TYPE_POLYGON: typeof VIZ_TYPE_POLYGON;
declare const CONSTANTS_VIZ_TYPE_POSE: typeof VIZ_TYPE_POSE;
declare const CONSTANTS_VIZ_TYPE_POSEARRAY: typeof VIZ_TYPE_POSEARRAY;
declare const CONSTANTS_VIZ_TYPE_RANGE: typeof VIZ_TYPE_RANGE;
declare const CONSTANTS_VIZ_TYPE_ROBOTMODEL: typeof VIZ_TYPE_ROBOTMODEL;
declare const CONSTANTS_VIZ_TYPE_TF: typeof VIZ_TYPE_TF;
declare const CONSTANTS_VIZ_TYPE_WRENCH: typeof VIZ_TYPE_WRENCH;
declare const CONSTANTS_POINT_FIELD_DATATYPES: typeof POINT_FIELD_DATATYPES;
declare const CONSTANTS_COLLISION_OBJECT_OPERATIONS: typeof COLLISION_OBJECT_OPERATIONS;
declare const CONSTANTS_SOLID_PRIMITIVE_TYPES: typeof SOLID_PRIMITIVE_TYPES;
declare const CONSTANTS_MARKER_OBJECT_TYPES: typeof MARKER_OBJECT_TYPES;
declare const CONSTANTS_LINE_STYLES: typeof LINE_STYLES;
declare const CONSTANTS_MAP_COLOR_SCHEMES: typeof MAP_COLOR_SCHEMES;
declare const CONSTANTS_LASERSCAN_STYLES: typeof LASERSCAN_STYLES;
declare const CONSTANTS_COLOR_TRANSFORMERS: typeof COLOR_TRANSFORMERS;
declare const CONSTANTS_POINTCLOUD_COLOR_CHANNELS: typeof POINTCLOUD_COLOR_CHANNELS;
declare const CONSTANTS_AXES: typeof AXES;
declare const CONSTANTS_INTENSITY_CHANNEL_OPTIONS: typeof INTENSITY_CHANNEL_OPTIONS;
declare const CONSTANTS_ODOMETRY_OBJECT_TYPES: typeof ODOMETRY_OBJECT_TYPES;
declare const CONSTANTS_POSE_OBJECT_TYPES: typeof POSE_OBJECT_TYPES;
declare const CONSTANTS_WRENCH_OBJECT_TYPES: typeof WRENCH_OBJECT_TYPES;
declare const CONSTANTS_DEFAULT_CYLINDER_HEIGHT: typeof DEFAULT_CYLINDER_HEIGHT;
declare const CONSTANTS_DEFAULT_CYLINDER_RADIUS: typeof DEFAULT_CYLINDER_RADIUS;
declare const CONSTANTS_DEFAULT_RADIAL_SEGMENTS: typeof DEFAULT_RADIAL_SEGMENTS;
declare const CONSTANTS_DEFAULT_CONE_HEIGHT: typeof DEFAULT_CONE_HEIGHT;
declare const CONSTANTS_DEFAULT_CONE_RADIUS: typeof DEFAULT_CONE_RADIUS;
declare const CONSTANTS_DEFAULT_COLOR_X_AXIS: typeof DEFAULT_COLOR_X_AXIS;
declare const CONSTANTS_DEFAULT_COLOR_Y_AXIS: typeof DEFAULT_COLOR_Y_AXIS;
declare const CONSTANTS_DEFAULT_COLOR_Z_AXIS: typeof DEFAULT_COLOR_Z_AXIS;
declare const CONSTANTS_DEFAULT_COLOR_ARROW: typeof DEFAULT_COLOR_ARROW;
declare const CONSTANTS_DEFAULT_COLOR_LINE: typeof DEFAULT_COLOR_LINE;
declare const CONSTANTS_DEFAULT_OPTIONS_SCENE: typeof DEFAULT_OPTIONS_SCENE;
declare const CONSTANTS_DEFAULT_OPTIONS_TF_VIEWER: typeof DEFAULT_OPTIONS_TF_VIEWER;
declare const CONSTANTS_DEFAULT_OPTIONS_ARROW: typeof DEFAULT_OPTIONS_ARROW;
declare const CONSTANTS_DEFAULT_OPTIONS_AXES: typeof DEFAULT_OPTIONS_AXES;
declare const CONSTANTS_DEFAULT_OPTIONS_FLATARROW: typeof DEFAULT_OPTIONS_FLATARROW;
declare const CONSTANTS_DEFAULT_OPTIONS_DISPLAYTRAJECTORY: typeof DEFAULT_OPTIONS_DISPLAYTRAJECTORY;
declare const CONSTANTS_DEFAULT_OPTIONS_COLLISION_OBJECT: typeof DEFAULT_OPTIONS_COLLISION_OBJECT;
declare const CONSTANTS_DEFAULT_OPTIONS_DEPTHCLOUD: typeof DEFAULT_OPTIONS_DEPTHCLOUD;
declare const CONSTANTS_DEFAULT_OPTIONS_IMAGE: typeof DEFAULT_OPTIONS_IMAGE;
declare const CONSTANTS_DEFAULT_OPTIONS_IMAGE_STREAM: typeof DEFAULT_OPTIONS_IMAGE_STREAM;
declare const CONSTANTS_DEFAULT_OPTIONS_LASERSCAN: typeof DEFAULT_OPTIONS_LASERSCAN;
declare const CONSTANTS_DEFAULT_OPTIONS_MAP: typeof DEFAULT_OPTIONS_MAP;
declare const CONSTANTS_DEFAULT_OPTIONS_MARKER: typeof DEFAULT_OPTIONS_MARKER;
declare const CONSTANTS_DEFAULT_OPTIONS_MARKERARRAY: typeof DEFAULT_OPTIONS_MARKERARRAY;
declare const CONSTANTS_DEFAULT_OPTIONS_INTERACTIVE_MARKER: typeof DEFAULT_OPTIONS_INTERACTIVE_MARKER;
declare const CONSTANTS_DEFAULT_OPTIONS_ODOMETRY: typeof DEFAULT_OPTIONS_ODOMETRY;
declare const CONSTANTS_DEFAULT_OPTIONS_PATH: typeof DEFAULT_OPTIONS_PATH;
declare const CONSTANTS_DEFAULT_OPTIONS_PLANNINGSCENE: typeof DEFAULT_OPTIONS_PLANNINGSCENE;
declare const CONSTANTS_DEFAULT_OPTIONS_POINTCLOUD: typeof DEFAULT_OPTIONS_POINTCLOUD;
declare const CONSTANTS_DEFAULT_OPTIONS_POINT: typeof DEFAULT_OPTIONS_POINT;
declare const CONSTANTS_DEFAULT_OPTIONS_POLYGON: typeof DEFAULT_OPTIONS_POLYGON;
declare const CONSTANTS_DEFAULT_OPTIONS_POSE: typeof DEFAULT_OPTIONS_POSE;
declare const CONSTANTS_DEFAULT_OPTIONS_TORUS: typeof DEFAULT_OPTIONS_TORUS;
declare const CONSTANTS_DEFAULT_OPTIONS_ARROW_WITH_CIRCLE: typeof DEFAULT_OPTIONS_ARROW_WITH_CIRCLE;
declare const CONSTANTS_DEFAULT_OPTIONS_WRENCH: typeof DEFAULT_OPTIONS_WRENCH;
declare const CONSTANTS_DEFAULT_OPTIONS_POSEARRAY: typeof DEFAULT_OPTIONS_POSEARRAY;
declare const CONSTANTS_DEFAULT_OPTIONS_RANGE: typeof DEFAULT_OPTIONS_RANGE;
declare const CONSTANTS_DEFAULT_OPTIONS_ROBOTMODEL: typeof DEFAULT_OPTIONS_ROBOTMODEL;
declare const CONSTANTS_DEFAULT_OPTIONS_TF: typeof DEFAULT_OPTIONS_TF;
declare const CONSTANTS_SUPPORTED_MESSAGE_TYPES: typeof SUPPORTED_MESSAGE_TYPES;
declare const CONSTANTS_UNSUPPORTED_MESSAGE_TYPES: typeof UNSUPPORTED_MESSAGE_TYPES;
declare namespace CONSTANTS {
  export {
    CONSTANTS_OBJECT_TYPE_ARROW as OBJECT_TYPE_ARROW,
    CONSTANTS_OBJECT_TYPE_ARROW_WITH_CIRCLE as OBJECT_TYPE_ARROW_WITH_CIRCLE,
    CONSTANTS_OBJECT_TYPE_AXES as OBJECT_TYPE_AXES,
    CONSTANTS_OBJECT_TYPE_FLAT_ARROW as OBJECT_TYPE_FLAT_ARROW,
    CONSTANTS_MAX_POINTCLOUD_POINTS as MAX_POINTCLOUD_POINTS,
    CONSTANTS_DEFAULT_BACKGROUND_COLOR as DEFAULT_BACKGROUND_COLOR,
    CONSTANTS_DEFAULT_GRID_SIZE as DEFAULT_GRID_SIZE,
    CONSTANTS_DEFAULT_GRID_DIVISIONS as DEFAULT_GRID_DIVISIONS,
    CONSTANTS_DEFAULT_GRID_COLOR as DEFAULT_GRID_COLOR,
    CONSTANTS_DEFAULT_GRID_COLOR_CENTERLINE as DEFAULT_GRID_COLOR_CENTERLINE,
    CONSTANTS_INTERACTIVE_MARKER_ORIENTATION_MODES as INTERACTIVE_MARKER_ORIENTATION_MODES,
    CONSTANTS_UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES as UNSUPPORTED_INTERACTIVE_MARKER_ORIENTATION_MODES,
    CONSTANTS_INTERACTIVE_MARKER_INTERACTION_MODES as INTERACTIVE_MARKER_INTERACTION_MODES,
    CONSTANTS_MESSAGE_TYPE_ROBOT_MODEL as MESSAGE_TYPE_ROBOT_MODEL,
    CONSTANTS_MESSAGE_TYPE_ACCELSTAMPED as MESSAGE_TYPE_ACCELSTAMPED,
    CONSTANTS_MESSAGE_TYPE_POINTSTAMPED as MESSAGE_TYPE_POINTSTAMPED,
    CONSTANTS_MESSAGE_TYPE_POINTSTAMPED2 as MESSAGE_TYPE_POINTSTAMPED2,
    CONSTANTS_MESSAGE_TYPE_POLYGONSTAMPED as MESSAGE_TYPE_POLYGONSTAMPED,
    CONSTANTS_MESSAGE_TYPE_POSEARRAY as MESSAGE_TYPE_POSEARRAY,
    CONSTANTS_MESSAGE_TYPE_POSEARRAY2 as MESSAGE_TYPE_POSEARRAY2,
    CONSTANTS_MESSAGE_TYPE_POSECOVARIANCE as MESSAGE_TYPE_POSECOVARIANCE,
    CONSTANTS_MESSAGE_TYPE_POSECOVARIANCE2 as MESSAGE_TYPE_POSECOVARIANCE2,
    CONSTANTS_MESSAGE_TYPE_POSESTAMPED as MESSAGE_TYPE_POSESTAMPED,
    CONSTANTS_MESSAGE_TYPE_POSESTAMPED2 as MESSAGE_TYPE_POSESTAMPED2,
    CONSTANTS_MESSAGE_TYPE_TWISTSTAMPED as MESSAGE_TYPE_TWISTSTAMPED,
    CONSTANTS_MESSAGE_TYPE_VECTOR3STAMPED as MESSAGE_TYPE_VECTOR3STAMPED,
    CONSTANTS_MESSAGE_TYPE_WRENCHSTAMPED as MESSAGE_TYPE_WRENCHSTAMPED,
    CONSTANTS_MESSAGE_TYPE_WRENCHSTAMPED2 as MESSAGE_TYPE_WRENCHSTAMPED2,
    CONSTANTS_MESSAGE_TYPE_BOUNDINGVOLUME as MESSAGE_TYPE_BOUNDINGVOLUME,
    CONSTANTS_MESSAGE_TYPE_COLLISION_OBJECT as MESSAGE_TYPE_COLLISION_OBJECT,
    CONSTANTS_MESSAGE_TYPE_DISPLAYROBOTSTATE as MESSAGE_TYPE_DISPLAYROBOTSTATE,
    CONSTANTS_MESSAGE_TYPE_DISPLAYTRAJECTORY as MESSAGE_TYPE_DISPLAYTRAJECTORY,
    CONSTANTS_MESSAGE_TYPE_PLANNINGSCENE as MESSAGE_TYPE_PLANNINGSCENE,
    CONSTANTS_MESSAGE_TYPE_OCCUPANCYGRID as MESSAGE_TYPE_OCCUPANCYGRID,
    CONSTANTS_MESSAGE_TYPE_OCCUPANCYGRID2 as MESSAGE_TYPE_OCCUPANCYGRID2,
    CONSTANTS_MESSAGE_TYPE_ODOMETRY as MESSAGE_TYPE_ODOMETRY,
    CONSTANTS_MESSAGE_TYPE_ODOMETRY2 as MESSAGE_TYPE_ODOMETRY2,
    CONSTANTS_MESSAGE_TYPE_PATH as MESSAGE_TYPE_PATH,
    CONSTANTS_MESSAGE_TYPE_PATH2 as MESSAGE_TYPE_PATH2,
    CONSTANTS_MESSAGE_TYPE_COMPRESSEDIMAGE as MESSAGE_TYPE_COMPRESSEDIMAGE,
    CONSTANTS_MESSAGE_TYPE_DISPLAYJOINTSTATE as MESSAGE_TYPE_DISPLAYJOINTSTATE,
    CONSTANTS_MESSAGE_TYPE_IMAGE as MESSAGE_TYPE_IMAGE,
    CONSTANTS_MESSAGE_TYPE_IMAGE2 as MESSAGE_TYPE_IMAGE2,
    CONSTANTS_MESSAGE_TYPE_LASERSCAN as MESSAGE_TYPE_LASERSCAN,
    CONSTANTS_MESSAGE_TYPE_LASERSCAN2 as MESSAGE_TYPE_LASERSCAN2,
    CONSTANTS_MESSAGE_TYPE_MAGNETICFIELD as MESSAGE_TYPE_MAGNETICFIELD,
    CONSTANTS_MESSAGE_TYPE_POINTCLOUD as MESSAGE_TYPE_POINTCLOUD,
    CONSTANTS_MESSAGE_TYPE_POINTCLOUD2 as MESSAGE_TYPE_POINTCLOUD2,
    CONSTANTS_MESSAGE_TYPE_ROS2POINTCLOUD2 as MESSAGE_TYPE_ROS2POINTCLOUD2,
    CONSTANTS_MESSAGE_TYPE_RANGE as MESSAGE_TYPE_RANGE,
    CONSTANTS_MESSAGE_TYPE_RANGE2 as MESSAGE_TYPE_RANGE2,
    CONSTANTS_MESSAGE_TYPE_TF as MESSAGE_TYPE_TF,
    CONSTANTS_MESSAGE_TYPE_ROS2_TF as MESSAGE_TYPE_ROS2_TF,
    CONSTANTS_MESSAGE_TYPE_TF2 as MESSAGE_TYPE_TF2,
    CONSTANTS_MESSAGE_TYPE_ROS2_TF2 as MESSAGE_TYPE_ROS2_TF2,
    CONSTANTS_MESSAGE_TYPE_MARKER as MESSAGE_TYPE_MARKER,
    CONSTANTS_MESSAGE_TYPE_MARKER2 as MESSAGE_TYPE_MARKER2,
    CONSTANTS_MESSAGE_TYPE_MARKERARRAY as MESSAGE_TYPE_MARKERARRAY,
    CONSTANTS_MESSAGE_TYPE_MARKERARRAY2 as MESSAGE_TYPE_MARKERARRAY2,
    CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER as MESSAGE_TYPE_INTERACTIVEMARKER,
    CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER2 as MESSAGE_TYPE_INTERACTIVEMARKER2,
    CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE as MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE,
    CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2 as MESSAGE_TYPE_INTERACTIVEMARKER_UPDATE2,
    CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK as MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK,
    CONSTANTS_MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2 as MESSAGE_TYPE_INTERACTIVEMARKER_FEEDBACK2,
    CONSTANTS_VIZ_TYPE_IMAGE as VIZ_TYPE_IMAGE,
    CONSTANTS_VIZ_TYPE_INTERACTIVEMARKER as VIZ_TYPE_INTERACTIVEMARKER,
    CONSTANTS_VIZ_TYPE_LASERSCAN as VIZ_TYPE_LASERSCAN,
    CONSTANTS_VIZ_TYPE_MAP as VIZ_TYPE_MAP,
    CONSTANTS_VIZ_TYPE_MARKER as VIZ_TYPE_MARKER,
    CONSTANTS_VIZ_TYPE_MARKERARRAY as VIZ_TYPE_MARKERARRAY,
    CONSTANTS_VIZ_TYPE_ODOMETRY as VIZ_TYPE_ODOMETRY,
    CONSTANTS_VIZ_TYPE_PATH as VIZ_TYPE_PATH,
    CONSTANTS_VIZ_TYPE_POINT as VIZ_TYPE_POINT,
    CONSTANTS_VIZ_TYPE_POINTCLOUD as VIZ_TYPE_POINTCLOUD,
    CONSTANTS_VIZ_TYPE_POLYGON as VIZ_TYPE_POLYGON,
    CONSTANTS_VIZ_TYPE_POSE as VIZ_TYPE_POSE,
    CONSTANTS_VIZ_TYPE_POSEARRAY as VIZ_TYPE_POSEARRAY,
    CONSTANTS_VIZ_TYPE_RANGE as VIZ_TYPE_RANGE,
    CONSTANTS_VIZ_TYPE_ROBOTMODEL as VIZ_TYPE_ROBOTMODEL,
    CONSTANTS_VIZ_TYPE_TF as VIZ_TYPE_TF,
    CONSTANTS_VIZ_TYPE_WRENCH as VIZ_TYPE_WRENCH,
    CONSTANTS_POINT_FIELD_DATATYPES as POINT_FIELD_DATATYPES,
    CONSTANTS_COLLISION_OBJECT_OPERATIONS as COLLISION_OBJECT_OPERATIONS,
    CONSTANTS_SOLID_PRIMITIVE_TYPES as SOLID_PRIMITIVE_TYPES,
    CONSTANTS_MARKER_OBJECT_TYPES as MARKER_OBJECT_TYPES,
    CONSTANTS_LINE_STYLES as LINE_STYLES,
    CONSTANTS_MAP_COLOR_SCHEMES as MAP_COLOR_SCHEMES,
    CONSTANTS_LASERSCAN_STYLES as LASERSCAN_STYLES,
    CONSTANTS_COLOR_TRANSFORMERS as COLOR_TRANSFORMERS,
    CONSTANTS_POINTCLOUD_COLOR_CHANNELS as POINTCLOUD_COLOR_CHANNELS,
    CONSTANTS_AXES as AXES,
    CONSTANTS_INTENSITY_CHANNEL_OPTIONS as INTENSITY_CHANNEL_OPTIONS,
    CONSTANTS_ODOMETRY_OBJECT_TYPES as ODOMETRY_OBJECT_TYPES,
    CONSTANTS_POSE_OBJECT_TYPES as POSE_OBJECT_TYPES,
    CONSTANTS_WRENCH_OBJECT_TYPES as WRENCH_OBJECT_TYPES,
    CONSTANTS_DEFAULT_CYLINDER_HEIGHT as DEFAULT_CYLINDER_HEIGHT,
    CONSTANTS_DEFAULT_CYLINDER_RADIUS as DEFAULT_CYLINDER_RADIUS,
    CONSTANTS_DEFAULT_RADIAL_SEGMENTS as DEFAULT_RADIAL_SEGMENTS,
    CONSTANTS_DEFAULT_CONE_HEIGHT as DEFAULT_CONE_HEIGHT,
    CONSTANTS_DEFAULT_CONE_RADIUS as DEFAULT_CONE_RADIUS,
    CONSTANTS_DEFAULT_COLOR_X_AXIS as DEFAULT_COLOR_X_AXIS,
    CONSTANTS_DEFAULT_COLOR_Y_AXIS as DEFAULT_COLOR_Y_AXIS,
    CONSTANTS_DEFAULT_COLOR_Z_AXIS as DEFAULT_COLOR_Z_AXIS,
    CONSTANTS_DEFAULT_COLOR_ARROW as DEFAULT_COLOR_ARROW,
    CONSTANTS_DEFAULT_COLOR_LINE as DEFAULT_COLOR_LINE,
    CONSTANTS_DEFAULT_OPTIONS_SCENE as DEFAULT_OPTIONS_SCENE,
    CONSTANTS_DEFAULT_OPTIONS_TF_VIEWER as DEFAULT_OPTIONS_TF_VIEWER,
    CONSTANTS_DEFAULT_OPTIONS_ARROW as DEFAULT_OPTIONS_ARROW,
    CONSTANTS_DEFAULT_OPTIONS_AXES as DEFAULT_OPTIONS_AXES,
    CONSTANTS_DEFAULT_OPTIONS_FLATARROW as DEFAULT_OPTIONS_FLATARROW,
    CONSTANTS_DEFAULT_OPTIONS_DISPLAYTRAJECTORY as DEFAULT_OPTIONS_DISPLAYTRAJECTORY,
    CONSTANTS_DEFAULT_OPTIONS_COLLISION_OBJECT as DEFAULT_OPTIONS_COLLISION_OBJECT,
    CONSTANTS_DEFAULT_OPTIONS_DEPTHCLOUD as DEFAULT_OPTIONS_DEPTHCLOUD,
    CONSTANTS_DEFAULT_OPTIONS_IMAGE as DEFAULT_OPTIONS_IMAGE,
    CONSTANTS_DEFAULT_OPTIONS_IMAGE_STREAM as DEFAULT_OPTIONS_IMAGE_STREAM,
    CONSTANTS_DEFAULT_OPTIONS_LASERSCAN as DEFAULT_OPTIONS_LASERSCAN,
    CONSTANTS_DEFAULT_OPTIONS_MAP as DEFAULT_OPTIONS_MAP,
    CONSTANTS_DEFAULT_OPTIONS_MARKER as DEFAULT_OPTIONS_MARKER,
    CONSTANTS_DEFAULT_OPTIONS_MARKERARRAY as DEFAULT_OPTIONS_MARKERARRAY,
    CONSTANTS_DEFAULT_OPTIONS_INTERACTIVE_MARKER as DEFAULT_OPTIONS_INTERACTIVE_MARKER,
    CONSTANTS_DEFAULT_OPTIONS_ODOMETRY as DEFAULT_OPTIONS_ODOMETRY,
    CONSTANTS_DEFAULT_OPTIONS_PATH as DEFAULT_OPTIONS_PATH,
    CONSTANTS_DEFAULT_OPTIONS_PLANNINGSCENE as DEFAULT_OPTIONS_PLANNINGSCENE,
    CONSTANTS_DEFAULT_OPTIONS_POINTCLOUD as DEFAULT_OPTIONS_POINTCLOUD,
    CONSTANTS_DEFAULT_OPTIONS_POINT as DEFAULT_OPTIONS_POINT,
    CONSTANTS_DEFAULT_OPTIONS_POLYGON as DEFAULT_OPTIONS_POLYGON,
    CONSTANTS_DEFAULT_OPTIONS_POSE as DEFAULT_OPTIONS_POSE,
    CONSTANTS_DEFAULT_OPTIONS_TORUS as DEFAULT_OPTIONS_TORUS,
    CONSTANTS_DEFAULT_OPTIONS_ARROW_WITH_CIRCLE as DEFAULT_OPTIONS_ARROW_WITH_CIRCLE,
    CONSTANTS_DEFAULT_OPTIONS_WRENCH as DEFAULT_OPTIONS_WRENCH,
    CONSTANTS_DEFAULT_OPTIONS_POSEARRAY as DEFAULT_OPTIONS_POSEARRAY,
    CONSTANTS_DEFAULT_OPTIONS_RANGE as DEFAULT_OPTIONS_RANGE,
    CONSTANTS_DEFAULT_OPTIONS_ROBOTMODEL as DEFAULT_OPTIONS_ROBOTMODEL,
    CONSTANTS_DEFAULT_OPTIONS_TF as DEFAULT_OPTIONS_TF,
    CONSTANTS_SUPPORTED_MESSAGE_TYPES as SUPPORTED_MESSAGE_TYPES,
    CONSTANTS_UNSUPPORTED_MESSAGE_TYPES as UNSUPPORTED_MESSAGE_TYPES,
  };
}

interface BagReadResult {
    topic: string;
    message: RosMessage.Base;
    timestamp: {
        sec: number;
        nsec: number;
    };
    data: number[];
    chunkOffset: number;
    totalChunks: number;
}
declare type Reader = (file: File, result: BagReadResult) => void;
declare class RosbagBucket {
    files: Set<File>;
    topics: Array<{
        name: string;
        messageType: string;
        rosbagFileName: string;
    }>;
    private readers;
    addFile: (file: File) => Promise<void>;
    removeFile: (file: File, cb: () => void) => Promise<void>;
    addReader: (topic: string, fileName: string, reader: Reader) => void;
    removeReader: (topic: string, fileName: string, reader: Reader) => void;
    processQueue: (file: File) => void;
    enqueueProcessing: (index: number, file: File) => void;
    processBagReadResult: (result: BagReadResult, file: File) => void;
    processFile(file: File): Promise<void>;
}

interface RosBagDataSourceOptions {
    bucket: RosbagBucket;
    memory?: boolean;
    topicName: string;
    fileName: string;
}
declare class RosbagDataSource<T extends Message> implements DataSource<T> {
    readonly createdAt: Date;
    readonly hasMemory: boolean;
    private readonly bucket;
    private readonly producer;
    private stream;
    private isStreamPaused;
    private internalListener;
    private readonly listeners;
    constructor(options: RosBagDataSourceOptions);
    addListener: (listener: Listener<T>) => {
        success: boolean;
        reason: string;
    } | {
        success: boolean;
        reason?: undefined;
    };
    removeListener: (listener: Listener<T>) => {
        success: boolean;
        reason: string;
    } | {
        success: boolean;
        reason?: undefined;
    };
    removeAllListeners: () => {
        success: boolean;
    };
    pause: () => {
        success: boolean;
    };
    resume: () => {
        success: boolean;
    };
}

declare const _default: {
    EditorControls: {
        (object: any, domElement: any): void;
        prototype: any;
    };
    CollisionObject: typeof CollisionObject;
    DepthCloud: typeof DepthCloud;
    DisplayTrajectory: typeof DisplayTrajectory;
    PlanningScene: typeof PlanningScene;
    Point: typeof Point;
    Polygon: typeof Polygon;
    Pose: typeof Pose;
    Wrench: typeof Wrench;
    PoseArray: typeof PoseArray;
    Tf: typeof Tf;
    RobotModel: typeof RobotModel;
    MarkerArray: typeof MarkerArray;
    LaserScan: typeof LaserScan;
    Map: typeof Map;
    Odometry: typeof Odometry;
    Path: typeof Path;
    Image: typeof Image;
    ImageStream: typeof ImageStream;
    Marker: typeof Marker;
    Range: typeof Range;
    Scene: typeof Scene;
    Viewer2d: typeof Viewer2d;
    Viewer3d: typeof Viewer3d;
    TfViewer: typeof TfViewer;
    InteractiveMarkers: typeof InteractiveMarkers;
    RosbagBucket: typeof RosbagBucket;
    RosTopicDataSource: typeof RosTopicDataSource;
    RosbagDataSource: typeof RosbagDataSource;
    CONSTANTS: typeof CONSTANTS;
};

export { _default as default };
