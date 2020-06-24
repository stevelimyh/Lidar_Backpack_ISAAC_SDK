..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _log-folder:

Log Folder Convention
---------------------

The log in an Isaac application is a directory; not a single file. The directory naming convention
is as follows:

.. image:: folderPath.png
   :scale: 50%
   :align: center

The path to the log folder is made up of three parts: the *base directory*, an *application UUID*
directory and a *tag*. The *base directory* needs to be a directory where the user running the
application has write privileges. The *application UUID* is a string representing a unique ID per
execution of the application. This is a new unique ID every time the application runs. It cannot
be changed or predetermined. Finally, the *tag* is an optional folder that lets the user
distinguish between different logs captured during a single execution of the application.

The only components of the log folder that are user-defined are the *base directory* and the *tag*.

.. note:: The default value for the base directory is ``/tmp/isaac`` and in Linux, the contents of
          ``/tmp`` are erased with every reboot.

To save log files to a user-defined folder, the folder must be created first. The
:code:`Invalid path for log.` error indicates that the directory did not exist at the time
the log file attempted to save.

Recorder Component
------------------

Applications that must record a component's channel must define a node containing a recorder
component, and then connect all the components to be recorded to that recorder component.

As in any Isaac SDK application, the graph can be created with C++ or with a JSON configuration
file. This section provides procedures for both methods, for record and replay functionality.
JSON configuration is recommended however.

A recorder component definition is shown in the following JSON snippet:

.. code-block:: json

    {
      "name": "recorder",
      "components": [
        {
          "name": "message_ledger",
          "type": "isaac::alice::MessageLedger"
        },
        {
          "name": "isaac.alice.Recorder",
          "type": "isaac::alice::Recorder"
        }
      ]
    }


To log channels *a* and *b* of components *X* and *Y* respectively, define the following JSON edges:

.. code-block:: javascript

    {
      "source": "<Node Name>/X/a",
      "target": "recorder/isaac.alice.Recorder/a"
    },
    {
      "source": "<Node Name>/Y/b",
      "target": "recorder/isaac.alice.Recorder/b"
    }


The initial definition of the recorder component is the following:

.. code-block:: javascript

  "recorder": {
      "isaac.alice.Recorder": {
      "base_directory": "/tmp/isaac",
      "tag": "test",
      "start_recording_automatically": false
    }
  }

The fields that can be configured are: the ``base_directory`` and the ``tag``, which define the
location of the log, and ``start_recording_automatically``, which controls recording at application
startup.

Replay Component
----------------

To replay a recorded log, the application must define a node containing a replay component and
then connect all the destination components to the replay component.

The initial definition of the replay component is the following:

.. code-block:: javascript

    {
      "name": "replay",
      "components": [
        {
          "name": "message_ledger",
          "type": "isaac::alice::MessageLedger"
        },
        {
          "name": "isaac.alice.Replay",
          "type": "isaac::alice::Replay"
        }
      ]
    }


Connect the replay component to all target channels for which the corresponding source channel was
recorded. The channels of the components that the replay component is replacing must be
disconnected in the JSON also. Continue the example in the previous section for replay, with the
following JSON code:

.. code-block:: javascript

    {
      "source": "replay/isaac.alice.Replay/a",
      "target": "<Node Name>/M/a"
    },
    {
      "source": "replay/isaac.alice.Replay/a",
      "target": "<Node Name>/N/a"
    },
    {
      "source": "replay/isaac.alice.Replay/b",
      "target": "<Node Name>/P/b"
    }


The channel *a* sends replayed messages to components *M* and *N*, and channel *b* to component
*P*. Not all recorded channels must be replayed. One channel can be replayed to more than one
target.

Same as in the previous section, the Replay node can have its initial configuration:

.. code-block:: javascript

  "replay": {
      "isaac.alice.Replay": {
      "cask_directory": "/tmp/isaac/e0d7caae-a70a-11e8-8c38-91bfb5eade6f/test000",
      "replay_time_offset": 0,
      "use_recorded_message_time": false
    }
  }


Set the ``cask_directory`` to the log folder. See :ref:`log-folder` for more information.

The ``replay_time_offset`` parameter is an offset in nanoseconds from where to start the replay.
Its default value is zero or no offset.

The ``use_recorded_message_time`` specifies the time stamp to be used as publish time for
a message during the replay. When ``false`` it uses the publish time of the replay component, and
when ``true`` it uses the time stamp from recorded log messages. By default this option is ``false``.

To replay a log multiple times during a single application execution without killing and restarting,
the ``use_recorded_message_time`` parameter should be set to ``false`` to avoid unexpected
behavior during replay. In order to disable automatic start of the replay component, omit
``cask_directory`` from the configuration.
