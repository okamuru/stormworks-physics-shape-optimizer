import QtQuick
import QtQuick3D
import StormworksPhysicsGpu 1.0

Rectangle {
    id: root
    color: "#101828"
    property real zoom: 1.0
    property real panX: 0.0
    property real panY: 0.0
    property real sceneSpan: 1.0
    property real fitDiameter: 1.0
    property real ghostOpacity: 0.0
    property int shapeCount: 0
    property quaternion orientation: Qt.quaternion(
        0.9131794572, 0.1941022873, -0.3505367637, -0.0745088905)

    View3D {
        anchors.fill: parent
        camera: camera
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#101828"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        OrthographicCamera {
            id: camera
            position: Qt.vector3d(0, 0, Math.max(root.sceneSpan * 4.0, 10.0))
            clipNear: Math.max(root.sceneSpan * 0.002, 0.01)
            clipFar: Math.max(root.sceneSpan * 20.0, 100.0)
            horizontalMagnification: Math.max(
                0.1, Math.min(root.width, root.height) * 0.86 / root.fitDiameter * root.zoom)
            verticalMagnification: horizontalMagnification
        }

        Node {
            position: Qt.vector3d(
                root.panX / camera.horizontalMagnification,
                -root.panY / camera.verticalMagnification,
                0)
            rotation: root.orientation

            Model {
                geometry: PhysicsShapeGeometry {
                    objectName: "ghostGeometry"
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: "white"
                        vertexColorsEnabled: true
                        opacity: root.ghostOpacity
                        cullMode: Material.BackFaceCulling
                        depthDrawMode: Material.NeverDepthDraw
                    }
                ]
            }

            Model {
                geometry: PhysicsShapeOutlineGeometry {
                    objectName: "ghostOutlines"
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: "#9AA4B2"
                        opacity: Math.min(root.ghostOpacity, 0.45)
                        lineWidth: 1.0
                        cullMode: Material.NoCulling
                        depthDrawMode: Material.NeverDepthDraw
                    }
                ]
            }

            Model {
                geometry: PhysicsShapeGeometry {
                    objectName: "physicsGeometry"
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: "white"
                        vertexColorsEnabled: true
                        opacity: 1.0
                        cullMode: Material.BackFaceCulling
                        depthDrawMode: Material.AlwaysDepthDraw
                    }
                ]
            }

            Model {
                geometry: PhysicsShapeOutlineGeometry {
                    objectName: "physicsOutlines"
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: "#D0D5DD"
                        opacity: 1.0
                        lineWidth: 1.0
                        cullMode: Material.NoCulling
                        depthDrawMode: Material.NeverDepthDraw
                    }
                ]
            }

            Model {
                geometry: PhysicsShapeOutlineGeometry {
                    objectName: "selectionOutlines"
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: "#FDE68A"
                        opacity: 1.0
                        lineWidth: 2.0
                        cullMode: Material.NoCulling
                        depthDrawMode: Material.NeverDepthDraw
                    }
                ]
            }

            Model {
                geometry: PhysicsShapeOutlineGeometry {
                    objectName: "hoverOutlines"
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: "white"
                        opacity: 1.0
                        lineWidth: 3.0
                        cullMode: Material.NoCulling
                        depthDrawMode: Material.NeverDepthDraw
                    }
                ]
            }
        }
    }

    Text {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 14
        color: "#F2F4F7"
        font.bold: true
        text: root.shapeCount + " Shapes • GPU"
    }
}
