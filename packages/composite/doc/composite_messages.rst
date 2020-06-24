.. _composite_message:

Composite Messages
==================

Robotics hardware includes many different parts that can be actuated in various ways. Examples
include a differential base, holonomic base, or multi-joint arm controlled with joint positions or
joint speeds. In Isaac SDK, various nodes--like global waypoint planners, local trajectory planners,
controllers, and drivers--need to exchange messages to communicate states and commands for robotics
hardware. Furthermore, not only are single states required, but sometimes timeseries of states,
batches of timeseries, or states carrying information about multiple instances.

To address this diversity of messaging requirements, Isaac SDK features the Composite message: a
single message type that can address all of the above use cases.

Measurement Types
-----------------

A Composite message can include the following measurement types:

+------------------------------+--------------------------------------------------+----------------------------------------------+
| Type                         | Description                                      | Format                                       |
+==============================+==================+==============================+===============================================+
| :code:`None`                 | The measure of the quantity is not included in   | scalar, vector                               |
|                              | the list of measurement types and thus not       | matrix, tensor                               |
|                              | specified.                                       |                                              |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`Time`                 | The time of an event, used in particular to      | scalar                                       |
|                              | store timestamps for timeseries                  |                                              |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`Mass`                 | The weight of an entity                          | kilogram [kg]                                |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`Position`             | A Euclidean vector representing the position of  | n-dimensional vector                         |
|                              | an object in space relative to a reference       | meters [m] or unitless [1]                   |
|                              | frame: for example, the position of a mobile     |                                              |
|                              | robot in a room. This measurement is used in a   |                                              |
|                              | general sense to store the state of an           |                                              |
|                              | n-dimensional system, for example the joint      |                                              |
|                              | “positions” of a multi-joint robotics arm.       |                                              |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`Speed`                | The rate of change of the position of an object  | n-dimensional vector                         |
|                              | relative to a reference frame over time. This    | meters/second [m/s] or unitless/second [1/s] |
|                              | measurement is also used in a general sense to   |                                              |
|                              | store the rate of change of the state of an      |                                              |
|                              | n-dimensionsal system.                           |                                              |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`Acceleration`         | The rate of change of speed.                     | n-dimensional vector or scalar               |
|                              |                                                  | meters/s/s [m/s^2] or unitless/s/s [1/s^2]   |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`Rotation`             | The rotation of an entity in 2D or 3D space      | 2D: normalized complex as 2 scalars          |
|                              | relative to an anchor point. Rotations are best  | 3D: normalized quaternion as 4 scalars       |
|                              | expressed as unit complex numbers (cos(a),       |                                              |
|                              | sin(a)) or unit quaternions. Avoid using angles  |                                              |
|                              | (for 2D) or Euler angles (for 3D).               |                                              |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`AngularSpeed`         | Rate of change of rotation over time.            | 2D: scalar; [rad/s] or [1/s]                 |
|                              |                                                  | 3D: 3-dim vector; [rad/s] or [1/s]           |
+------------------------------+--------------------------------------------------+----------------------------------------------+
| :code:`AngularAcceleration`  | Rate of change of angular velocity over time     | 2D: scalar; [rad/s^2] or [1/s^2]             |
|                              |                                                  | 3D: 3-dim vector; [rad/s^2] or [1/s^2]       |
+------------------------------+--------------------------------------------------+----------------------------------------------+

Examples for Using Composite Message
------------------------------------

Arm Joint Speeds
^^^^^^^^^^^^^^^^

The following Composite message describes the joint speeds of a robotics arm:

.. code::

   proto: {
     "schema": [
       {"entity": "base_2", "element_type": Float64, "measure": Speed},
       {"entity": "foo", "element_type": Float64, "measure": Position},
       {"entity": "elbow", "element_type": Float64, "measure": Speed},
       {"entity": "base_1", "element_type": Float64, "measure": Speed},
       {"entity": "wrist", "element_type": Float64, "measure": Speed}
     ],
     "schema_hash": "...",
     "values": {
       "element_type": Float64,
       "sizes": [4],
       "dataBufferIndex": 0
     }
   }
   buffers: [[0.3, 0.7, -0.4, 0.2, 0.1]]

This component reads the message:

.. code-block:: cpp

   class MyArm4Controller {
    public:
     void start() {
       speed_parser_.requestSchema({{"base_1", "base_2", "elbow", "wrist"},
       CompositeParser::Speed});
     }
     void tick() {
       Vector4f speeds;
       if (!speed_parser_.parse(rx_command().getProto(), rx_command().buffers(), speeds)) {
         reportFailure("Could not parse message: %s", speed_parser_.error_str());
         return;
       }
     }
     ISAAC_RX(CompositeProto, command);
    private:
     CompositeParser speed_parser_;
   };

The output vector is [0.2, 0.3, -0.4, 0.1]

Base Trajectory Command
^^^^^^^^^^^^^^^^^^^^^^^

The following Composite message contains a trajectory command for a base:

.. code::

   proto: {
     "schema": [,
       {"entity": "time", "element_type": Float64, "measure": Time},
       {"entity": "base", "element_type": Float64, "measure": Position},
       {"entity": "base", "element_type": Float64, "measure": LinearSpeed},
       {"entity": "base", "element_type": Float64, "measure": AngularSpeed}
     ],
     "schema_hash": "...",
     "values": {
       "element_type": Float64,
       "sizes": [5, 4],
       "dataBufferIndex": 0
     }
   }
   buffers: [[0.30, 17.4, 0.70, -0.40], [0.35, 17.8, 0.64, -0.38], [0.40, 18.2, 0.61, -0.36], [0.44, 18.5, 0.56, -0.34], [0.47, 18.7, 0.59, -0.31]]

This component processes the command:

.. code-block:: cpp

   class MyTrajectoryReceiver {
    public:
     void start() {
       speed_parser_.requestSchema({{"base", LinearSpeed}, {"base", AngularSpeed}});
     }
     void tick() {
       Timeseries<Vector2f, float> series;
       if (!speed_parser_.parse(rx_command().getProto(), rx_command().buffers(), "time", series)) {
         reportFailure("Could not parse message: %s", speed_parser_.error_str());
         return;
       }
     }
     ISAAC_RX(CompositeProto, command);
    private:
     CompositeParser speed_parser_;
   };

The output time series is: (0.30, [0.70, -0.40]), (0.35, [0.64, -0.38]), (0.40, [0.61, -0.36])
, (0.44, [0.56, -0.34]), (0.47, [0.59, -0.31])

Arm Joints and End Effector Command
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following Composite message contains command for both arm joints and an end effector. In this
case the end effector state is binary, so the measure is `None`:

.. code::

   proto: {
     "schema": [
       {"entity": "elbow", "element_type": Float64, "measure": Speed},
       {"entity": "base", "element_type": Float64, "measure": Speed},
       {"entity": "wrist", "element_type": Float64, "measure": Speed},
       {"entity": "gripper", "element_type": Float64, "measure": None}
     ],
     "schema_hash": "...",
     "values": {
       "element_type": Float64,
       "sizes": [5],
       "dataBufferIndex": 0
     }
   }
   buffers: [[0.3, 0.4, -0.2, 1]]

The component processes the command:

.. code-block:: cpp

   class MyArmAndGripperController {
    public:
     void start() {
       arm_parser_.requestSchema({"base", "elbow", "wrist"}, CompositeParser::Speed);
       gripper_parser_.requestSchema({"gripper"}, CompositeParser::Position);
     }
     void tick() {
       Vector3d arm_joint_speeds;
       if (!arm_parser_.parse(rx_command().getProto(), rx_command().buffers(), arm_joint_speeds)) {
         reportFailure("Could not parse message: %s", parser_.error_str());
         return;
       }
       Vector1f gripper_position;
       if (!gripper_parser_.parse(rx_command().getProto(), rx_command().buffers(), gripper_position)) {
         reportFailure("Could not parse message: %s", parser_.error_str());
         return;
       }
     }
     ISAAC_RX(CompositeProto, command);
    private:
     CompositeParser arm_parser_;
     CompositeParser gripper_parser_;
   };

The outputs are: arm_joint_speeds [0.4, 0.3, -0.2], gripper_position [1]

.. _CompositeMetric and CompositeAtlas:

CompositeMetric and CompositeAtlas
----------------------------------

CompositeProto can also be used to represent waypoints in configuration space, using cask as
storage for atlas (a set of named composite waypoints) and uuid as waypoint name identification.
A distance metric must be defined to compare the “closeness” of a CompositeProto to the waypoint.

Components
^^^^^^^^^^^^^^^

The :ref:`isaac.composite.CompositeMetric` component defines how to compute the distance between two
composite protos. It contains a schema representing quantities to use in distance
computation, the p-norm used for each quantity, and its weight in the total distance.
Use cases include moving through a set of joint angle waypoints or turning suction end effectors
on or off.

The :ref:`isaac.composite.CompositeAtlas` component provides access to waypoints stored in cask.
This is a thread-safe way to support access from multiple components.

The :ref:`isaac.composite.CompositePublisher` component reads a list of waypoints from atlas and
publishes the whole path as a CompositeProto. The waypoints all have the same schema and are
represented as batches in the CompositeProto.

The :ref:`isaac.composite.FollowPath` component receives a path from CompositePublisher and
publishes one waypoint at a time. It receives the current state and computes the distance to the
current waypoint. Once the distance is within tolerance, the next waypoint is published.
The CompositeMetric component must be attached to the same node to specify how distance is
calculated.

Computing Distance with CompositeMetric
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The snippet below computes the distance between two input proto messages on command and state
channels. Note that, if a codelet uses the CompositeMetric component, you should place it in the
same node as the component.

.. code-block:: cpp

   // get metric component
   metric_ = node()->getComponent<CompositeMetric>();
   // try to set schema for metric from command message
   metric_->setOrLoadSchema(ReadSchema(tx_command().getProto()));
   // get the schema used the CompositeMetric
   const auto schema = metric_->getSchema();
   // parse the two states to compare. error handle is omitted
   if (schema) {
     parser.requestSchema(*schema)
     VectorXd x1(schema->getElementCount()),  x2(schema->getElementCount());
     if (parser.parse(tx_command().getProto(), tx_command().buffers(), x1) &&
         parser.parse(tx_state().getProto(), tx_state().buffers(), x2)) {
       double distance = *metric_->distance(x1, x2);
     }
   }

Creating Cask in a Python Script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Python snippet below shows how to create a cask with composite waypoints:

.. code-block:: python

   from engine.pyalice import Cask
   from engine.pyalice.Composite import create_composite_message

   cask = Cask(cask_root, writable=True)
   msg = create_composite_message(quantities, values)
   msg.uuid = name
   cask.write_message(msg)

Using FollowPath with CompositeAtlas
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The JSON snippet below shows how to create a follow-path application:

.. code-block::

   "graph": {
     "nodes": [
       {
         "name": "atlas",
         "components": [
          {
            "name": "CompositeAtlas",
            "type": "isaac:::composite::CompositeAtlas"
          }
        ]
       },
       {
         "name": "follow_path",
         "components": [
           {
             "name": "ledger",
             "type": "isaac::alice::MessageLedger"
           },
           {
             "name": "CompositeMetric",
             "type": "isaac:::composite::CompositeMetric"
           },
           {
             "name": "CompositePublisher",
             "type": "isaac::composite::CompositePublisher"
           },
           {
             "name": "FollowPath",
             "type": "isaac::composite::FollowPath"
           }
         ]
       }
     ],
     "edges": [
      {
        "source": "follow_path/CompositePublisher/path",
        "target": "follow_path/FollowPath/path"
      }
     ]
   },
   "config": {
     "follow_path": {
       "CompositePublisher": {
         "tick_period": "10Hz",
         "atlas": "atlas/CompositeAtlas",
         "path": ["cart_observe", "cart_align", "cart_dropoff"]
       },
       "CompositePublisher": {
         "tick_period": "10Hz",
         "wait_time": 1.0,
         "tolerance": 0.05
       }
     }
   }

JSON Serialization
------------------

JSON serialization of Measure, Quantity, and Schema is required to configure the composite schema
for a codelet through ISAAC_PARAM.

**Measure**: Use capnp JSON string serialization of the CompositeProto::Measure enum.
NLOHMANN_JSON_SERIALIZE_ENUM cannot be used due to a namespace limitation.

.. code::

   // in codelet
   ISAAC_PARAM(std::string, measure)
   // in app.json configs
   "measure": "angularSpeed"

**Quantity**: Consists of a JSON array of entity (string), measure (string), and an optional
dimension (array of int).

.. code::

   // in codelet
   ISAAC_PARAM(composite::Quantity, quantity)
   // in app.json configs
   "quantity": ["tool", "position", [3]]
   // or
   "quantity": ["elbow", "position"] // dimension default to [1]

Schema
^^^^^^

**Option 1**: A list of quantities

.. code::

   // in codelet
   ISAAC_PARAM(composite::Schema, schema)
   // in app.json configs
   "schema": [["tool", "position", [3]], ["tool", "rotation", [4]], ["elbow", "speed"]]

**Option 2**: An array of entities (string) and a measure; dimension defaults to [1]

.. code::

   // in codelet
   ISAAC_PARAM(composite::Schema, schema)
   // in app.json configs
   "schema": {
     "entity": ["shoulder", "elbow", "wrist"],
     "measure": "speed"
   }

