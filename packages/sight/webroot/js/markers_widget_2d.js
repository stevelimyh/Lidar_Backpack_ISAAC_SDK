/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

function Create2DMarkerWidget(initialPosition, initialOrientation) {
  let position = initialPosition.clone();
  let orientation = initialOrientation.clone();
  return {
    "set": (transform) => {
      position.copy(transform.position);
      orientation.copy(transform.orientation);
    },
    "value": () => {
      return {
        "position": position,
        "orientation": orientation,
      };
    },
  };
}

function Create2DMarkersWidget() {

  let widgets = {};
  let widgetInteractive = {};
  let hoveredWidget;
  let activeWidget;
  let initialOrientationGuidePointWorld = undefined;
  let orientationGuidePointWorld = undefined;
  let initialOrientationWorld = undefined;
  let desiredTransformationMode = undefined;

  let reset = () => {
    widgets = {};
    widgetInteractive = {};
    initialOrientationGuidePointWorld = undefined;
    orientationGuidePointWorld = undefined;
    initialOrientationWorld = undefined;
  };

  function addMarker(name, positionWorld, orientationWorld) {
    if (name in widgets) {
      throw "widget by name " + name + "already added";
    }

    widgetInteractive[name] = true;
    let widget = Create2DMarkerWidget(positionWorld, orientationWorld);
    widgets[name] = widget;
  }

  function* markers () {
    for (let name in widgets) {
      yield name;
    }
  };

  // "active" means the widget is currently being hovered or manipulated
  let widgetIsActive = (name) => {
    if (!(name in widgets)) {
      return false;
    }

    let widget = widgets[name];
    if (widget === activeWidget) {
      return true;
    }
    return false;
  };

  let activeInteraction = widgetIsActive;

  let enableInteraction = (name) => {
    if (!(name in widgets)) return;
    widgetInteractive[name] = true;
  };

  let disableInteraction = (name) => {
    if (!(name in widgets)) return;
    if (activeInteraction(name)) {
      throw "cannot disable interaction gracefully while there is an active interaction, please "
          + "call stealInteraction";
    } else {
      widgetInteractive[name] = false;
    }
  };

  let set = (transformations) => {
    for (let name in transformations) {
      if (!widgets[name]) continue;
      if (widgetInteractive[name]) {
        throw "forcefully setting transforms while widget '" + name +
            "' is interactive, please call disableInteraction method first";
      }
      let transformation = transformations[name];
      widgets[name].set(transformation);
    }
  };

  let update =
      (hoveredWidgetName,
       selectedWidgetName,
       indicatedDesiredTransformationMode,
       cursorRayPointWorld,
       cursorRayDirectionWorld,
       mouseTransformationMode) => {
        orientationGuidePointWorld = undefined;
        if (mouseTransformationMode) {
          if (!activeWidget && selectedWidgetName == hoveredWidgetName) {
            activeWidget = hoveredWidget;
          }

          if (activeWidget) {
            switch(mouseTransformationMode) {
              case "translation": {
                let projectedCursorPositionWorld = cursorRayPointWorld.clone();
                projectedCursorPositionWorld.addScaledVector(
                  cursorRayDirectionWorld,
                    -cursorRayPointWorld.z/cursorRayDirectionWorld.z
                );

                let currentTransform = activeWidget.value();
                activeWidget.set({
                  "position": new THREE.Vector3(projectedCursorPositionWorld.x,
                                                projectedCursorPositionWorld.y,
                                                currentTransform.position.z),
                  "orientation": currentTransform.orientation,
                });
                break;
              }
              case "orientation": {
                let projectedCursorPositionWorld = cursorRayPointWorld.clone();
                projectedCursorPositionWorld.addScaledVector(
                  cursorRayDirectionWorld,
                  (activeWidget.value().position.z - cursorRayPointWorld.z)/
                    cursorRayDirectionWorld.z
                );

                if (!initialOrientationWorld) {
                  initialOrientationWorld = activeWidget.value().orientation.clone();
                }

                const d =
                      projectedCursorPositionWorld.distanceToSquared(activeWidget.value().position);
                if (d > 0) {
                  orientationGuidePointWorld = projectedCursorPositionWorld.clone();
                  if (!initialOrientationGuidePointWorld) {
                    initialOrientationGuidePointWorld = orientationGuidePointWorld.clone();
                  }

                  if (initialOrientationGuidePointWorld) {
                    const v0 = initialOrientationGuidePointWorld.clone();
                    const v1 = orientationGuidePointWorld.clone();
                    const p = activeWidget.value().position;
                    v0.sub(p).normalize();
                    v1.sub(p).normalize();
                    let orientationChangeWorld = new THREE.Quaternion().setFromUnitVectors(v0, v1);
                    activeWidget.set({
                      "position": p,
                      "orientation": orientationChangeWorld.multiply(initialOrientationWorld),
                    });
                  }
                }

                break;
              }
            }
          }
        } else {
          activeWidget = undefined;
          initialOrientationGuidePointWorld = undefined;
          initialOrientationWorld = undefined;

          if (hoveredWidgetName) {
            hoveredWidget = widgets[hoveredWidgetName];
            desiredTransformationMode = indicatedDesiredTransformationMode;
          } else {
            hoveredWidget = undefined;
            desiredTransformationMode = undefined;
          }
        }
      };

  return {
    "desiredTransformationMode": () => desiredTransformationMode,
    "addMarker": addMarker,
    "reset": reset,
    "transform": (name) => widgets[name].value(),
    "containsMarker": (name) => widgets.hasOwnProperty(name),
    "set": set,
    "markers": markers,
    "enableInteraction": enableInteraction,
    "disableInteraction": disableInteraction,
    "interactionEnabled": (name) => widgetInteractive[name],
    "activeInteraction": activeInteraction,
    "update": update,
    "orientationGuideLineSegment": () => {
      if (orientationGuidePointWorld) {
        return {
          "start": activeWidget.value().position,
          "end": orientationGuidePointWorld,
        };
      } else {
        return undefined;
      }
    },
  };
}
