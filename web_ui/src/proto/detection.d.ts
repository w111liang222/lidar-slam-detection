import * as $protobuf from "protobufjs";
/** Properties of a Point3D. */
export interface IPoint3D {
  /** Point3D x */
  x: number;

  /** Point3D y */
  y: number;

  /** Point3D z */
  z: number;
}

/** Represents a Point3D. */
export class Point3D implements IPoint3D {
  /**
   * Constructs a new Point3D.
   * @param [properties] Properties to set
   */
  constructor(properties?: IPoint3D);

  /** Point3D x. */
  public x: number;

  /** Point3D y. */
  public y: number;

  /** Point3D z. */
  public z: number;

  /**
   * Creates a new Point3D instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Point3D instance
   */
  public static create(properties?: IPoint3D): Point3D;

  /**
   * Encodes the specified Point3D message. Does not implicitly {@link Point3D.verify|verify} messages.
   * @param message Point3D message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IPoint3D, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Point3D message, length delimited. Does not implicitly {@link Point3D.verify|verify} messages.
   * @param message Point3D message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IPoint3D, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Point3D message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Point3D
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Point3D;

  /**
   * Decodes a Point3D message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Point3D
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Point3D;

  /**
   * Verifies a Point3D message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Point3D message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Point3D
   */
  public static fromObject(object: { [k: string]: any }): Point3D;

  /**
   * Creates a plain object from a Point3D message. Also converts values to other types if specified.
   * @param message Point3D
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Point3D, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Point3D to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Box3D. */
export interface IBox3D {
  /** Box3D center */
  center: IPoint3D;

  /** Box3D length */
  length: number;

  /** Box3D width */
  width: number;

  /** Box3D height */
  height: number;

  /** Box3D heading */
  heading: number;
}

/** Represents a Box3D. */
export class Box3D implements IBox3D {
  /**
   * Constructs a new Box3D.
   * @param [properties] Properties to set
   */
  constructor(properties?: IBox3D);

  /** Box3D center. */
  public center: IPoint3D;

  /** Box3D length. */
  public length: number;

  /** Box3D width. */
  public width: number;

  /** Box3D height. */
  public height: number;

  /** Box3D heading. */
  public heading: number;

  /**
   * Creates a new Box3D instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Box3D instance
   */
  public static create(properties?: IBox3D): Box3D;

  /**
   * Encodes the specified Box3D message. Does not implicitly {@link Box3D.verify|verify} messages.
   * @param message Box3D message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IBox3D, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Box3D message, length delimited. Does not implicitly {@link Box3D.verify|verify} messages.
   * @param message Box3D message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IBox3D, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Box3D message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Box3D
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Box3D;

  /**
   * Decodes a Box3D message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Box3D
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Box3D;

  /**
   * Verifies a Box3D message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Box3D message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Box3D
   */
  public static fromObject(object: { [k: string]: any }): Box3D;

  /**
   * Creates a plain object from a Box3D message. Also converts values to other types if specified.
   * @param message Box3D
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Box3D, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Box3D to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Trajectory. */
export interface ITrajectory {
  /** Trajectory x */
  x: number;

  /** Trajectory y */
  y: number;

  /** Trajectory z */
  z: number;

  /** Trajectory heading */
  heading: number;

  /** Trajectory velocityX */
  velocityX: number;

  /** Trajectory velocityY */
  velocityY: number;

  /** Trajectory relativeTimestamp */
  relativeTimestamp: number | Long;
}

/** Represents a Trajectory. */
export class Trajectory implements ITrajectory {
  /**
   * Constructs a new Trajectory.
   * @param [properties] Properties to set
   */
  constructor(properties?: ITrajectory);

  /** Trajectory x. */
  public x: number;

  /** Trajectory y. */
  public y: number;

  /** Trajectory z. */
  public z: number;

  /** Trajectory heading. */
  public heading: number;

  /** Trajectory velocityX. */
  public velocityX: number;

  /** Trajectory velocityY. */
  public velocityY: number;

  /** Trajectory relativeTimestamp. */
  public relativeTimestamp: number | Long;

  /**
   * Creates a new Trajectory instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Trajectory instance
   */
  public static create(properties?: ITrajectory): Trajectory;

  /**
   * Encodes the specified Trajectory message. Does not implicitly {@link Trajectory.verify|verify} messages.
   * @param message Trajectory message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: ITrajectory, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Trajectory message, length delimited. Does not implicitly {@link Trajectory.verify|verify} messages.
   * @param message Trajectory message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: ITrajectory, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Trajectory message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Trajectory
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Trajectory;

  /**
   * Decodes a Trajectory message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Trajectory
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Trajectory;

  /**
   * Verifies a Trajectory message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Trajectory message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Trajectory
   */
  public static fromObject(object: { [k: string]: any }): Trajectory;

  /**
   * Creates a plain object from a Trajectory message. Also converts values to other types if specified.
   * @param message Trajectory
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Trajectory, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Trajectory to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Header. */
export interface IHeader {
  /** Header version */
  version?: Uint8Array | null;

  /** Header timestamp */
  timestamp?: number | Long | null;

  /** Header relativeTimestamp */
  relativeTimestamp?: number | Long | null;

  /** Header fps */
  fps?: number | null;
}

/** Represents a Header. */
export class Header implements IHeader {
  /**
   * Constructs a new Header.
   * @param [properties] Properties to set
   */
  constructor(properties?: IHeader);

  /** Header version. */
  public version: Uint8Array;

  /** Header timestamp. */
  public timestamp: number | Long;

  /** Header relativeTimestamp. */
  public relativeTimestamp: number | Long;

  /** Header fps. */
  public fps: number;

  /**
   * Creates a new Header instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Header instance
   */
  public static create(properties?: IHeader): Header;

  /**
   * Encodes the specified Header message. Does not implicitly {@link Header.verify|verify} messages.
   * @param message Header message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IHeader, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Header message, length delimited. Does not implicitly {@link Header.verify|verify} messages.
   * @param message Header message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IHeader, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Header message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Header
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Header;

  /**
   * Decodes a Header message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Header
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Header;

  /**
   * Verifies a Header message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Header message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Header
   */
  public static fromObject(object: { [k: string]: any }): Header;

  /**
   * Creates a plain object from a Header message. Also converts values to other types if specified.
   * @param message Header
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Header, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Header to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Pose. */
export interface IPose {
  /** Pose x */
  x: number;

  /** Pose y */
  y: number;

  /** Pose z */
  z: number;

  /** Pose heading */
  heading: number;

  /** Pose pitch */
  pitch: number;

  /** Pose roll */
  roll: number;

  /** Pose latitude */
  latitude?: number | null;

  /** Pose longitude */
  longitude?: number | null;

  /** Pose altitude */
  altitude?: number | null;

  /** Pose status */
  status?: number | null;

  /** Pose state */
  state?: string | null;
}

/** Represents a Pose. */
export class Pose implements IPose {
  /**
   * Constructs a new Pose.
   * @param [properties] Properties to set
   */
  constructor(properties?: IPose);

  /** Pose x. */
  public x: number;

  /** Pose y. */
  public y: number;

  /** Pose z. */
  public z: number;

  /** Pose heading. */
  public heading: number;

  /** Pose pitch. */
  public pitch: number;

  /** Pose roll. */
  public roll: number;

  /** Pose latitude. */
  public latitude: number;

  /** Pose longitude. */
  public longitude: number;

  /** Pose altitude. */
  public altitude: number;

  /** Pose status. */
  public status: number;

  /** Pose state. */
  public state: string;

  /**
   * Creates a new Pose instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Pose instance
   */
  public static create(properties?: IPose): Pose;

  /**
   * Encodes the specified Pose message. Does not implicitly {@link Pose.verify|verify} messages.
   * @param message Pose message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IPose, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Pose message, length delimited. Does not implicitly {@link Pose.verify|verify} messages.
   * @param message Pose message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IPose, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Pose message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Pose
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Pose;

  /**
   * Decodes a Pose message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Pose
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Pose;

  /**
   * Verifies a Pose message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Pose message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Pose
   */
  public static fromObject(object: { [k: string]: any }): Pose;

  /**
   * Creates a plain object from a Pose message. Also converts values to other types if specified.
   * @param message Pose
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Pose, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Pose to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of an Object. */
export interface IObject {
  /** Object id */
  id: number;

  /** Object type */
  type: Object.Type;

  /** Object confidence */
  confidence: number;

  /** Object box */
  box: IBox3D;

  /** Object velocityX */
  velocityX: number;

  /** Object velocityY */
  velocityY: number;

  /** Object angleRate */
  angleRate: number;

  /** Object accelX */
  accelX: number;

  /** Object valid */
  valid: boolean;

  /** Object status */
  status: Object.Status;

  /** Object age */
  age: number;

  /** Object trajectory */
  trajectory?: ITrajectory[] | null;
}

/** Represents an Object. */
export class Object implements IObject {
  /**
   * Constructs a new Object.
   * @param [properties] Properties to set
   */
  constructor(properties?: IObject);

  /** Object id. */
  public id: number;

  /** Object type. */
  public type: Object.Type;

  /** Object confidence. */
  public confidence: number;

  /** Object box. */
  public box: IBox3D;

  /** Object velocityX. */
  public velocityX: number;

  /** Object velocityY. */
  public velocityY: number;

  /** Object angleRate. */
  public angleRate: number;

  /** Object accelX. */
  public accelX: number;

  /** Object valid. */
  public valid: boolean;

  /** Object status. */
  public status: Object.Status;

  /** Object age. */
  public age: number;

  /** Object trajectory. */
  public trajectory: ITrajectory[];

  /**
   * Creates a new Object instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Object instance
   */
  public static create(properties?: IObject): object;

  /**
   * Encodes the specified Object message. Does not implicitly {@link Object.verify|verify} messages.
   * @param message Object message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IObject, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Object message, length delimited. Does not implicitly {@link Object.verify|verify} messages.
   * @param message Object message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IObject, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes an Object message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Object
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): object;

  /**
   * Decodes an Object message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Object
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): object;

  /**
   * Verifies an Object message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates an Object message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Object
   */
  public static fromObject(object: { [k: string]: any }): object;

  /**
   * Creates a plain object from an Object message. Also converts values to other types if specified.
   * @param message Object
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: object, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Object to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

export namespace Object {
  /** Type enum. */
  enum Type {
    UNKNOWN = 0,
    VEHICLE = 1,
    PEDESTRIAN = 2,
    CYCLIST = 3,
    CONE = 4,
  }

  /** Status enum. */
  enum Status {
    UNDEFINED = 0,
    STATIC = 1,
    STOPPED = 2,
    MOVING = 3,
  }
}

/** Properties of a Trafficlight. */
export interface ITrafficlight {
  /** Trafficlight id */
  id: number;

  /** Trafficlight pictogram */
  pictogram: Trafficlight.Pictogram;

  /** Trafficlight color */
  color: Trafficlight.Color;

  /** Trafficlight confidence */
  confidence: number;

  /** Trafficlight name */
  name: string;
}

/** Represents a Trafficlight. */
export class Trafficlight implements ITrafficlight {
  /**
   * Constructs a new Trafficlight.
   * @param [properties] Properties to set
   */
  constructor(properties?: ITrafficlight);

  /** Trafficlight id. */
  public id: number;

  /** Trafficlight pictogram. */
  public pictogram: Trafficlight.Pictogram;

  /** Trafficlight color. */
  public color: Trafficlight.Color;

  /** Trafficlight confidence. */
  public confidence: number;

  /** Trafficlight name. */
  public name: string;

  /**
   * Creates a new Trafficlight instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Trafficlight instance
   */
  public static create(properties?: ITrafficlight): Trafficlight;

  /**
   * Encodes the specified Trafficlight message. Does not implicitly {@link Trafficlight.verify|verify} messages.
   * @param message Trafficlight message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: ITrafficlight, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Trafficlight message, length delimited. Does not implicitly {@link Trafficlight.verify|verify} messages.
   * @param message Trafficlight message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: ITrafficlight, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Trafficlight message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Trafficlight
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Trafficlight;

  /**
   * Decodes a Trafficlight message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Trafficlight
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Trafficlight;

  /**
   * Verifies a Trafficlight message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Trafficlight message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Trafficlight
   */
  public static fromObject(object: { [k: string]: any }): Trafficlight;

  /**
   * Creates a plain object from a Trafficlight message. Also converts values to other types if specified.
   * @param message Trafficlight
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Trafficlight, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Trafficlight to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

export namespace Trafficlight {
  /** Pictogram enum. */
  enum Pictogram {
    OTHER = 0,
    UP_ARROW = 1,
    LEFT_ARROW = 2,
    RIGHT_ARROW = 3,
    DOWN_ARROW = 4,
    UTURN = 5,
  }

  /** Color enum. */
  enum Color {
    RED = 0,
    GREEN = 1,
    YELLOW = 2,
    OFF = 3,
  }
}

/** Properties of a FreespaceInfo. */
export interface IFreespaceInfo {
  /** FreespaceInfo xMin */
  xMin: number;

  /** FreespaceInfo xMax */
  xMax: number;

  /** FreespaceInfo yMin */
  yMin: number;

  /** FreespaceInfo yMax */
  yMax: number;

  /** FreespaceInfo zMin */
  zMin: number;

  /** FreespaceInfo zMax */
  zMax: number;

  /** FreespaceInfo resolution */
  resolution: number;

  /** FreespaceInfo xNum */
  xNum: number;

  /** FreespaceInfo yNum */
  yNum: number;
}

/** Represents a FreespaceInfo. */
export class FreespaceInfo implements IFreespaceInfo {
  /**
   * Constructs a new FreespaceInfo.
   * @param [properties] Properties to set
   */
  constructor(properties?: IFreespaceInfo);

  /** FreespaceInfo xMin. */
  public xMin: number;

  /** FreespaceInfo xMax. */
  public xMax: number;

  /** FreespaceInfo yMin. */
  public yMin: number;

  /** FreespaceInfo yMax. */
  public yMax: number;

  /** FreespaceInfo zMin. */
  public zMin: number;

  /** FreespaceInfo zMax. */
  public zMax: number;

  /** FreespaceInfo resolution. */
  public resolution: number;

  /** FreespaceInfo xNum. */
  public xNum: number;

  /** FreespaceInfo yNum. */
  public yNum: number;

  /**
   * Creates a new FreespaceInfo instance using the specified properties.
   * @param [properties] Properties to set
   * @returns FreespaceInfo instance
   */
  public static create(properties?: IFreespaceInfo): FreespaceInfo;

  /**
   * Encodes the specified FreespaceInfo message. Does not implicitly {@link FreespaceInfo.verify|verify} messages.
   * @param message FreespaceInfo message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IFreespaceInfo, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified FreespaceInfo message, length delimited. Does not implicitly {@link FreespaceInfo.verify|verify} messages.
   * @param message FreespaceInfo message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IFreespaceInfo, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a FreespaceInfo message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns FreespaceInfo
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): FreespaceInfo;

  /**
   * Decodes a FreespaceInfo message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns FreespaceInfo
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): FreespaceInfo;

  /**
   * Verifies a FreespaceInfo message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a FreespaceInfo message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns FreespaceInfo
   */
  public static fromObject(object: { [k: string]: any }): FreespaceInfo;

  /**
   * Creates a plain object from a FreespaceInfo message. Also converts values to other types if specified.
   * @param message FreespaceInfo
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: FreespaceInfo, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this FreespaceInfo to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Freespace. */
export interface IFreespace {
  /** Freespace info */
  info?: IFreespaceInfo | null;

  /** Freespace cells */
  cells?: Uint8Array | null;
}

/** Represents a Freespace. */
export class Freespace implements IFreespace {
  /**
   * Constructs a new Freespace.
   * @param [properties] Properties to set
   */
  constructor(properties?: IFreespace);

  /** Freespace info. */
  public info?: IFreespaceInfo | null;

  /** Freespace cells. */
  public cells: Uint8Array;

  /**
   * Creates a new Freespace instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Freespace instance
   */
  public static create(properties?: IFreespace): Freespace;

  /**
   * Encodes the specified Freespace message. Does not implicitly {@link Freespace.verify|verify} messages.
   * @param message Freespace message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IFreespace, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Freespace message, length delimited. Does not implicitly {@link Freespace.verify|verify} messages.
   * @param message Freespace message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IFreespace, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Freespace message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Freespace
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Freespace;

  /**
   * Decodes a Freespace message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Freespace
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Freespace;

  /**
   * Verifies a Freespace message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Freespace message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Freespace
   */
  public static fromObject(object: { [k: string]: any }): Freespace;

  /**
   * Creates a plain object from a Freespace message. Also converts values to other types if specified.
   * @param message Freespace
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Freespace, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Freespace to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a CameraImage. */
export interface ICameraImage {
  /** CameraImage cameraName */
  cameraName: string;

  /** CameraImage image */
  image: Uint8Array;
}

/** Represents a CameraImage. */
export class CameraImage implements ICameraImage {
  /**
   * Constructs a new CameraImage.
   * @param [properties] Properties to set
   */
  constructor(properties?: ICameraImage);

  /** CameraImage cameraName. */
  public cameraName: string;

  /** CameraImage image. */
  public image: Uint8Array;

  /**
   * Creates a new CameraImage instance using the specified properties.
   * @param [properties] Properties to set
   * @returns CameraImage instance
   */
  public static create(properties?: ICameraImage): CameraImage;

  /**
   * Encodes the specified CameraImage message. Does not implicitly {@link CameraImage.verify|verify} messages.
   * @param message CameraImage message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: ICameraImage, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified CameraImage message, length delimited. Does not implicitly {@link CameraImage.verify|verify} messages.
   * @param message CameraImage message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: ICameraImage, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a CameraImage message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns CameraImage
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): CameraImage;

  /**
   * Decodes a CameraImage message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns CameraImage
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): CameraImage;

  /**
   * Verifies a CameraImage message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a CameraImage message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns CameraImage
   */
  public static fromObject(object: { [k: string]: any }): CameraImage;

  /**
   * Creates a plain object from a CameraImage message. Also converts values to other types if specified.
   * @param message CameraImage
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: CameraImage, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this CameraImage to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Radar. */
export interface IRadar {
  /** Radar radarName */
  radarName?: string | null;

  /** Radar radarObject */
  radarObject?: IObject[] | null;
}

/** Represents a Radar. */
export class Radar implements IRadar {
  /**
   * Constructs a new Radar.
   * @param [properties] Properties to set
   */
  constructor(properties?: IRadar);

  /** Radar radarName. */
  public radarName: string;

  /** Radar radarObject. */
  public radarObject: IObject[];

  /**
   * Creates a new Radar instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Radar instance
   */
  public static create(properties?: IRadar): Radar;

  /**
   * Encodes the specified Radar message. Does not implicitly {@link Radar.verify|verify} messages.
   * @param message Radar message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IRadar, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Radar message, length delimited. Does not implicitly {@link Radar.verify|verify} messages.
   * @param message Radar message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IRadar, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Radar message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Radar
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Radar;

  /**
   * Decodes a Radar message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Radar
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Radar;

  /**
   * Verifies a Radar message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Radar message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Radar
   */
  public static fromObject(object: { [k: string]: any }): Radar;

  /**
   * Creates a plain object from a Radar message. Also converts values to other types if specified.
   * @param message Radar
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Radar, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Radar to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a Detection. */
export interface IDetection {
  /** Detection header */
  header?: IHeader | null;

  /** Detection object */
  object?: IObject[] | null;

  /** Detection freespace */
  freespace?: Uint8Array | null;

  /** Detection points */
  points?: Uint8Array | null;

  /** Detection image */
  image?: ICameraImage[] | null;

  /** Detection radar */
  radar?: IRadar[] | null;

  /** Detection pose */
  pose?: IPose | null;

  /** Detection light */
  light?: ITrafficlight[] | null;
}

/** Represents a Detection. */
export class Detection implements IDetection {
  /**
   * Constructs a new Detection.
   * @param [properties] Properties to set
   */
  constructor(properties?: IDetection);

  /** Detection header. */
  public header?: IHeader | null;

  /** Detection object. */
  public object: IObject[];

  /** Detection freespace. */
  public freespace: Uint8Array;

  /** Detection points. */
  public points: Uint8Array;

  /** Detection image. */
  public image: ICameraImage[];

  /** Detection radar. */
  public radar: IRadar[];

  /** Detection pose. */
  public pose?: IPose | null;

  /** Detection light. */
  public light: ITrafficlight[];

  /**
   * Creates a new Detection instance using the specified properties.
   * @param [properties] Properties to set
   * @returns Detection instance
   */
  public static create(properties?: IDetection): Detection;

  /**
   * Encodes the specified Detection message. Does not implicitly {@link Detection.verify|verify} messages.
   * @param message Detection message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: IDetection, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified Detection message, length delimited. Does not implicitly {@link Detection.verify|verify} messages.
   * @param message Detection message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: IDetection, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a Detection message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns Detection
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): Detection;

  /**
   * Decodes a Detection message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns Detection
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): Detection;

  /**
   * Verifies a Detection message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a Detection message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns Detection
   */
  public static fromObject(object: { [k: string]: any }): Detection;

  /**
   * Creates a plain object from a Detection message. Also converts values to other types if specified.
   * @param message Detection
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: Detection, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this Detection to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a LidarPointcloud. */
export interface ILidarPointcloud {
  /** LidarPointcloud lidarName */
  lidarName: string;

  /** LidarPointcloud points */
  points: Uint8Array;

  /** LidarPointcloud attr */
  attr?: Uint8Array | null;

  /** LidarPointcloud type */
  type?: string | null;
}

/** Represents a LidarPointcloud. */
export class LidarPointcloud implements ILidarPointcloud {
  /**
   * Constructs a new LidarPointcloud.
   * @param [properties] Properties to set
   */
  constructor(properties?: ILidarPointcloud);

  /** LidarPointcloud lidarName. */
  public lidarName: string;

  /** LidarPointcloud points. */
  public points: Uint8Array;

  /** LidarPointcloud attr. */
  public attr: Uint8Array;

  /** LidarPointcloud type. */
  public type: string;

  /**
   * Creates a new LidarPointcloud instance using the specified properties.
   * @param [properties] Properties to set
   * @returns LidarPointcloud instance
   */
  public static create(properties?: ILidarPointcloud): LidarPointcloud;

  /**
   * Encodes the specified LidarPointcloud message. Does not implicitly {@link LidarPointcloud.verify|verify} messages.
   * @param message LidarPointcloud message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: ILidarPointcloud, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified LidarPointcloud message, length delimited. Does not implicitly {@link LidarPointcloud.verify|verify} messages.
   * @param message LidarPointcloud message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: ILidarPointcloud, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a LidarPointcloud message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns LidarPointcloud
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): LidarPointcloud;

  /**
   * Decodes a LidarPointcloud message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns LidarPointcloud
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): LidarPointcloud;

  /**
   * Verifies a LidarPointcloud message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a LidarPointcloud message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns LidarPointcloud
   */
  public static fromObject(object: { [k: string]: any }): LidarPointcloud;

  /**
   * Creates a plain object from a LidarPointcloud message. Also converts values to other types if specified.
   * @param message LidarPointcloud
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: LidarPointcloud, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this LidarPointcloud to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a CameraImageBytes. */
export interface ICameraImageBytes {
  /** CameraImageBytes cameraName */
  cameraName: string;

  /** CameraImageBytes image */
  image: Uint8Array;
}

/** Represents a CameraImageBytes. */
export class CameraImageBytes implements ICameraImageBytes {
  /**
   * Constructs a new CameraImageBytes.
   * @param [properties] Properties to set
   */
  constructor(properties?: ICameraImageBytes);

  /** CameraImageBytes cameraName. */
  public cameraName: string;

  /** CameraImageBytes image. */
  public image: Uint8Array;

  /**
   * Creates a new CameraImageBytes instance using the specified properties.
   * @param [properties] Properties to set
   * @returns CameraImageBytes instance
   */
  public static create(properties?: ICameraImageBytes): CameraImageBytes;

  /**
   * Encodes the specified CameraImageBytes message. Does not implicitly {@link CameraImageBytes.verify|verify} messages.
   * @param message CameraImageBytes message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: ICameraImageBytes, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified CameraImageBytes message, length delimited. Does not implicitly {@link CameraImageBytes.verify|verify} messages.
   * @param message CameraImageBytes message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: ICameraImageBytes, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a CameraImageBytes message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns CameraImageBytes
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): CameraImageBytes;

  /**
   * Decodes a CameraImageBytes message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns CameraImageBytes
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): CameraImageBytes;

  /**
   * Verifies a CameraImageBytes message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a CameraImageBytes message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns CameraImageBytes
   */
  public static fromObject(object: { [k: string]: any }): CameraImageBytes;

  /**
   * Creates a plain object from a CameraImageBytes message. Also converts values to other types if specified.
   * @param message CameraImageBytes
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: CameraImageBytes, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this CameraImageBytes to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}

/** Properties of a LidarPointcloudMap. */
export interface ILidarPointcloudMap {
  /** LidarPointcloudMap lp */
  lp?: ILidarPointcloud[] | null;

  /** LidarPointcloudMap image */
  image?: ICameraImageBytes[] | null;
}

/** Represents a LidarPointcloudMap. */
export class LidarPointcloudMap implements ILidarPointcloudMap {
  /**
   * Constructs a new LidarPointcloudMap.
   * @param [properties] Properties to set
   */
  constructor(properties?: ILidarPointcloudMap);

  /** LidarPointcloudMap lp. */
  public lp: ILidarPointcloud[];

  /** LidarPointcloudMap image. */
  public image: ICameraImageBytes[];

  /**
   * Creates a new LidarPointcloudMap instance using the specified properties.
   * @param [properties] Properties to set
   * @returns LidarPointcloudMap instance
   */
  public static create(properties?: ILidarPointcloudMap): LidarPointcloudMap;

  /**
   * Encodes the specified LidarPointcloudMap message. Does not implicitly {@link LidarPointcloudMap.verify|verify} messages.
   * @param message LidarPointcloudMap message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encode(message: ILidarPointcloudMap, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Encodes the specified LidarPointcloudMap message, length delimited. Does not implicitly {@link LidarPointcloudMap.verify|verify} messages.
   * @param message LidarPointcloudMap message or plain object to encode
   * @param [writer] Writer to encode to
   * @returns Writer
   */
  public static encodeDelimited(message: ILidarPointcloudMap, writer?: $protobuf.Writer): $protobuf.Writer;

  /**
   * Decodes a LidarPointcloudMap message from the specified reader or buffer.
   * @param reader Reader or buffer to decode from
   * @param [length] Message length if known beforehand
   * @returns LidarPointcloudMap
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decode(reader: $protobuf.Reader | Uint8Array, length?: number): LidarPointcloudMap;

  /**
   * Decodes a LidarPointcloudMap message from the specified reader or buffer, length delimited.
   * @param reader Reader or buffer to decode from
   * @returns LidarPointcloudMap
   * @throws {Error} If the payload is not a reader or valid buffer
   * @throws {$protobuf.util.ProtocolError} If required fields are missing
   */
  public static decodeDelimited(reader: $protobuf.Reader | Uint8Array): LidarPointcloudMap;

  /**
   * Verifies a LidarPointcloudMap message.
   * @param message Plain object to verify
   * @returns `null` if valid, otherwise the reason why it is not
   */
  public static verify(message: { [k: string]: any }): string | null;

  /**
   * Creates a LidarPointcloudMap message from a plain object. Also converts values to their respective internal types.
   * @param object Plain object
   * @returns LidarPointcloudMap
   */
  public static fromObject(object: { [k: string]: any }): LidarPointcloudMap;

  /**
   * Creates a plain object from a LidarPointcloudMap message. Also converts values to other types if specified.
   * @param message LidarPointcloudMap
   * @param [options] Conversion options
   * @returns Plain object
   */
  public static toObject(message: LidarPointcloudMap, options?: $protobuf.IConversionOptions): { [k: string]: any };

  /**
   * Converts this LidarPointcloudMap to JSON.
   * @returns JSON object
   */
  public toJSON(): { [k: string]: any };
}
