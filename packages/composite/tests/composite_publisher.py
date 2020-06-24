'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import time

from engine.pyalice import Application
'''
A new test application which uses the CompositeAtlas and CompositeWaypointPublisher components to
read and publish the composite from a cask.
'''


def setup_app():
    app = Application(name="composite_atlas", modules=["composite"])
    atlas = app.add("atlas").add(app.registry.isaac.composite.CompositeAtlas)
    atlas.config["cask"] = "packages/composite/tests/waypoints"

    pub_node = app.add("pub")
    pub = pub_node.add(app.registry.isaac.composite.CompositePublisher, name="CompositePublisher")
    pub.config.tick_period = "20Hz"
    pub.config.atlas = "atlas/CompositeAtlas"
    pub.config.path = ["cart", "dolly"]
    return app, pub


if __name__ == '__main__':
    # test schema from waypoint
    app, _ = setup_app()
    app.start()
    time.sleep(0.1)
    msg = app.receive("pub", "CompositePublisher", "path")
    assert msg, 'Cannot receive path message'
    assert len(msg.json['quantities']) == 6, 'Path schema number of quantities incorrect'
    tensor = msg.tensor
    assert tensor.shape == (2, 1, 6), 'Path tensor shape incorrect'
    app.stop()

    # test schema from config
    app, pub = setup_app()
    schema = {
        "entity": ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint'],
        "measure": "position"
    }
    pub.config.use_config_schema = True
    pub.config.schema = schema
    app.start()
    time.sleep(0.1)
    msg = app.receive("pub", "CompositePublisher", "path")
    assert msg, 'Cannot receive path message'
    assert len(msg.json['quantities']) == 3, 'Path schema number of quantities incorrect'
    tensor = msg.tensor
    assert tensor.shape == (2, 1, 3), 'Path tensor shape incorrect'
    app.stop()
