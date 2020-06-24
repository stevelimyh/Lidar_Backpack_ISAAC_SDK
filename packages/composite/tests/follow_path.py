'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from engine.pyalice import Application, Node
from engine.pyalice.bindings import Status

'''
A new test application which uses the CompositeAtlas and CompositeWaypointPublisher components to
read and publish the composite from a cask.
'''


def setup_app():
  app = Application(name="follow_path", modules=["composite"])
  atlas = app.add("atlas").add(app.registry.isaac.composite.CompositeAtlas)
  atlas.config["cask"] = "packages/composite/tests/waypoints"

  pub_node = app.add("pub")
  pub = pub_node.add(app.registry.isaac.composite.CompositePublisher, name="CompositePublisher")
  pub.config.tick_period = "20Hz"
  pub.config.atlas = "atlas/CompositeAtlas"
  pub.config.path = ["cart", "dolly"]

  follow_node = app.add("follow")
  follow = follow_node.add(app.registry.isaac.composite.FollowPath)
  follow.config.tick_period = "10Hz"
  follow.config.wait_time = 0.05
  follow_node.add(app.registry.isaac.composite.CompositeMetric)

  app.connect(pub, "path", follow, "path")
  app.connect(follow, "goal", follow, "state")
  return app, follow_node


if __name__ == '__main__':
  app, follow_node = setup_app()
  app.start_wait_stop(1.0)
  assert follow_node.status == Status.Success, "Follow path fails"
